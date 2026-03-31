using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Rendering;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App;

/// <summary>
/// Main application window that orchestrates the road editor, traffic simulation, and rendering.
/// Hosts a SkiaSharp canvas for rendering, a toolbar for tool selection, and a slider panel
/// for tuning steering parameters. Runs the simulation at a 30 Hz fixed timestep, with
/// rendering driven by a ~60 FPS WinForms timer.
/// </summary>
public class MainForm : Form
{
    private readonly Camera _camera = new();
    private readonly SkiaCanvas _canvas;
    private readonly RoadGraph _roadGraph = new();
    private readonly RoadRenderer _roadRenderer = new();
    private readonly VehicleRenderer _vehicleRenderer = new();
    private readonly VehicleStore _vehicles = new();
    private readonly EditorState _editorState = new();
    private readonly RoadTool _roadTool = new();
    private readonly DeleteTool _deleteTool = new();
    private readonly UIRenderer _uiRenderer = new();
    private readonly SliderPanel _sliderPanel = new();
    private readonly SpawnPointTool _spawnPointTool = new();
    private readonly MarkerRenderer _spawnPointRenderer = new(new SkiaSharp.SKColor(40, 200, 80, 200));
    private readonly DestinationTool _destinationTool = new();
    private readonly SignalTool _signalTool = new();
    private readonly LaneRestrictionTool _laneRestrictionTool = new();
    private readonly MarkerRenderer _destinationRenderer = new(new SkiaSharp.SKColor(200, 60, 40, 200));
    private readonly SpatialGrid _vehicleGrid = new();
    private readonly StopLineCache _stopLineCache = new();
    private readonly EdgeSpatialGrid _edgeSpatialGrid = new();
    private readonly TrafficSignalSystem _trafficSignals = new();
    private readonly StopSignSystem _stopSigns = new();
    private readonly YieldSignSystem _yieldSigns = new();
    private readonly IntersectionArcCache _intersectionArcs = new();
    private readonly VehicleInfoPanel _vehicleInfoPanel = new();
    private readonly VehicleSpawner _spawner;
    private readonly GraphChangeHandler _graphChangeHandler;
    private readonly SimulationLoop _simLoop;
    private readonly SceneRenderer _sceneRenderer;

    private Point _lastMousePos;
    private Point _currentMousePos;
    private bool _isPanning;

    public MainForm()
    {
        Text = "ROADS - Traffic Simulation";
        Width = 1280;
        Height = 720;
        DoubleBuffered = true;
        StartPosition = FormStartPosition.CenterScreen;

        _editorState.ActiveTool = EditorTool.Road;
        _spawner = new VehicleSpawner(_roadGraph, _vehicles, _vehicleGrid);
        _graphChangeHandler = new GraphChangeHandler(_roadGraph, _editorState, _vehicles, _edgeSpatialGrid, _spawner);
        _simLoop = new SimulationLoop(_roadGraph, _vehicles, _vehicleGrid, _stopLineCache, _intersectionArcs, _edgeSpatialGrid, _trafficSignals, _stopSigns, _yieldSigns, _spawner, _editorState);
        _sceneRenderer = new SceneRenderer(_roadRenderer, _vehicleRenderer, _spawnPointRenderer, _destinationRenderer, _uiRenderer, _sliderPanel, _vehicleInfoPanel, _laneRestrictionTool);

        _sliderPanel.AddSlider("Kp", 0.5f, 10f, () => SteeringController.Kp, v => SteeringController.Kp = v);
        _sliderPanel.AddSlider("Kd", 0f, 5f, () => SteeringController.Kd, v => SteeringController.Kd = v);
        _sliderPanel.AddSlider("Max Steer", 0.1f, 1.5f, () => SteeringController.MaxSteer, v => SteeringController.MaxSteer = v);
        _sliderPanel.AddSlider("Target Speed", 1f, 30f, () => SteeringController.TargetSpeed, v => SteeringController.TargetSpeed = v);
        _sliderPanel.AddSlider("Lookahead Base", 0.5f, 15f, () => SteeringController.LookaheadBase, v => SteeringController.LookaheadBase = v);
        _sliderPanel.AddSlider("Lookahead/Speed", 0f, 2f, () => SteeringController.LookaheadPerSpeed, v => SteeringController.LookaheadPerSpeed = v);
        _sliderPanel.AddSlider("Lateral Gain", 0f, 3f, () => SteeringController.Klat, v => SteeringController.Klat = v);

        FormClosed += (_, _) => SteeringController.Shutdown();

        _canvas = new SkiaCanvas();
        _canvas.Dock = DockStyle.Fill;
        _canvas.PaintSurface += OnPaintSurface;
        _canvas.MouseDown += OnCanvasMouseDown;
        _canvas.MouseUp += OnCanvasMouseUp;
        _canvas.MouseMove += OnCanvasMouseMove;
        _canvas.MouseWheel += OnCanvasMouseWheel;
        _canvas.KeyDown += OnCanvasKeyDown;
        _canvas.PreviewKeyDown += (_, e) => e.IsInputKey = true;
        Controls.Add(_canvas);

        var timer = new System.Windows.Forms.Timer();
        timer.Interval = 16; // ~60 FPS
        timer.Tick += OnTick;
        timer.Start();
    }

    private void OnTick(object? sender, EventArgs e)
    {
        _simLoop.Tick();
        _canvas.Invalidate();
    }

    private void DumpVehicleDiag(int i)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"=== Vehicle {i} dump at {DateTime.Now:HH:mm:ss.fff} ===");
        sb.AppendLine($"  Pos: ({_vehicles.PosX[i]:F2}, {_vehicles.PosY[i]:F2})");
        sb.AppendLine($"  Heading: {_vehicles.Heading[i] * 180f / MathF.PI:F1} deg");
        sb.AppendLine($"  Speed: {_vehicles.Speed[i]:F3} m/s");
        sb.AppendLine($"  Throttle: {_vehicles.Throttle[i]:F3}  Brake: {_vehicles.Brake[i]:F3}");
        sb.AppendLine($"  SteeringAngle: {_vehicles.SteeringAngle[i] * 180f / MathF.PI:F1} deg");

        int edgeIdx = _vehicles.CurrentEdge[i];
        sb.AppendLine($"  CurrentEdge: {edgeIdx}  EdgeProgress: {_vehicles.EdgeProgress[i]:F4}");
        if (edgeIdx >= 0 && edgeIdx < _roadGraph.Edges.Count && _roadGraph.Edges[edgeIdx].FromNode >= 0)
        {
            var edge = _roadGraph.Edges[edgeIdx];
            sb.AppendLine($"    Edge: {edge.FromNode} -> {edge.ToNode}  Length: {edge.Length:F2}m  SpeedLimit: {edge.SpeedLimit:F1} m/s  Lanes: {edge.LaneCount}");
            float stopT = _stopLineCache.GetStopTAtToNode(edgeIdx);
            float fromT = _stopLineCache.GetStopTAtFromNode(edgeIdx);
            sb.AppendLine($"    StopT: fromNode={fromT:F4} toNode={stopT:F4}");

            var signal = _trafficSignals.GetSignal(edgeIdx);
            var stopSignal = _stopSigns.GetSignal(edgeIdx, _roadGraph, i);
            var yieldSignal = _yieldSigns.GetSignal(edgeIdx, _roadGraph);
            sb.AppendLine($"    Signals: traffic={signal} stopSign={stopSignal} yield={yieldSignal}");

            var node = _roadGraph.Nodes[edge.ToNode];
            sb.AppendLine($"    ToNode {edge.ToNode}: flags={node.Flags}");
        }

        sb.AppendLine($"  CurrentArc: {_vehicles.CurrentArc[i]}  ArcProgress: {_vehicles.ArcProgress[i]:F4}");
        if (_vehicles.CurrentArc[i] >= 0)
        {
            var arc = _intersectionArcs.GetArc(_vehicles.CurrentArc[i]);
            sb.AppendLine($"    Arc: node={arc.NodeIndex} inEdge={arc.IncomingEdge} outEdge={arc.OutgoingEdge} inLane={arc.IncomingLane} outLane={arc.OutgoingLane} length={arc.Length:F2}m speedLimit={arc.SpeedLimit:F1}");
        }

        sb.AppendLine($"  CurrentLane: {_vehicles.CurrentLane[i]}  TargetLane: {_vehicles.TargetLane[i]}  LaneChangeProgress: {_vehicles.LaneChangeProgress[i]:F3}");
        sb.AppendLine($"  PrevHeadingError: {_vehicles.PrevHeadingError[i]:F4}");

        var path = _vehicles.Path[i];
        int pathIdx = _vehicles.PathIndex[i];
        sb.AppendLine($"  PathIndex: {pathIdx}/{path?.Count ?? 0}");
        if (path != null)
        {
            for (int p = 0; p < path.Count; p++)
            {
                int pe = path[p];
                string marker = p == pathIdx ? " <-- current" : "";
                if (pe >= 0 && pe < _roadGraph.Edges.Count && _roadGraph.Edges[pe].FromNode >= 0)
                {
                    var e = _roadGraph.Edges[pe];
                    sb.AppendLine($"    [{p}] edge {pe}: {e.FromNode}->{e.ToNode} len={e.Length:F1}m{marker}");
                }
                else
                {
                    sb.AppendLine($"    [{p}] edge {pe}: DEFUNCT{marker}");
                }
            }
        }

        sb.AppendLine();
        File.AppendAllText("diag_vehicle.log", sb.ToString());
    }

    /// <summary>
    /// Handles keyboard input: V to spawn vehicles, +/- to adjust lane count on selected edge,
    /// [/] to adjust speed limit, T to toggle the slider panel, Space to pause/unpause,
    /// comma/period to decrease/increase simulation speed (1x-64x).
    /// </summary>
    private void OnCanvasKeyDown(object? sender, KeyEventArgs e)
    {
        // Press 'V' to spawn a vehicle with a random path
        if (e.KeyCode == Keys.V && _roadGraph.ActiveEdgeCount > 0)
        {
            _spawner.SpawnRandom();
            e.Handled = true;
        }

        // +/- to change lane count on selected edge
        if (_editorState.ActiveTool == EditorTool.Select && _editorState.SelectedEdge >= 0)
        {
            var edge = _roadGraph.Edges[_editorState.SelectedEdge];
            if (edge.FromNode >= 0)
            {
                if (e.KeyCode == Keys.Oemplus || e.KeyCode == Keys.Add)
                {
                    _roadGraph.SetLaneCount(_editorState.SelectedEdge, (byte)(edge.LaneCount + 1));
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.OemMinus || e.KeyCode == Keys.Subtract)
                {
                    _roadGraph.SetLaneCount(_editorState.SelectedEdge, (byte)(edge.LaneCount - 1));
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.OemCloseBrackets) // ] = increase speed limit
                {
                    _roadGraph.SetSpeedLimit(_editorState.SelectedEdge, edge.SpeedLimit + 2.235f);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.OemOpenBrackets) // [ = decrease speed limit
                {
                    _roadGraph.SetSpeedLimit(_editorState.SelectedEdge, edge.SpeedLimit - 2.235f);
                    e.Handled = true;
                }
            }
        }

        // Delete key to delete selected node
        if (e.KeyCode == Keys.Delete && _editorState.ActiveTool == EditorTool.Select
            && _editorState.SelectedNode >= 0)
        {
            _roadGraph.RemoveNode(_editorState.SelectedNode);
            _editorState.SelectedNode = -1;
            e.Handled = true;
        }

        // L key: toggle lane restriction mode (requires selected node)
        if (e.KeyCode == Keys.L && _editorState.ActiveTool == EditorTool.Select)
        {
            if (_editorState.LaneRestrictionMode)
            {
                _editorState.LaneRestrictionMode = false;
                _editorState.LaneRestrictionEdge = -1;
            }
            else if (_editorState.SelectedNode >= 0)
            {
                _editorState.LaneRestrictionMode = true;
                _editorState.LaneRestrictionEdge = -1;
            }
            e.Handled = true;
        }

        // C key: set lane restrictions to geometry defaults at selected node
        if (e.KeyCode == Keys.C && _editorState.LaneRestrictionMode && _editorState.SelectedNode >= 0)
        {
            _roadGraph.SetGeometryDefaultRestrictionsAtNode(_editorState.SelectedNode, _stopLineCache);
            e.Handled = true;
        }

        // Escape exits lane restriction mode
        if (e.KeyCode == Keys.Escape && _editorState.LaneRestrictionMode)
        {
            _editorState.LaneRestrictionMode = false;
            _editorState.LaneRestrictionEdge = -1;
            e.Handled = true;
        }

        // 1-4 keys: select lane index in lane restriction mode
        if (_editorState.LaneRestrictionMode && _editorState.LaneRestrictionEdge >= 0
            && e.KeyCode >= Keys.D1 && e.KeyCode <= Keys.D4)
        {
            byte lane = (byte)(e.KeyCode - Keys.D1);
            var edge = _roadGraph.Edges[_editorState.LaneRestrictionEdge];
            if (lane < edge.LaneCount)
                _editorState.LaneRestrictionLane = lane;
            e.Handled = true;
        }

        if (e.KeyCode == Keys.T)
        {
            _sliderPanel.Visible = !_sliderPanel.Visible;
            e.Handled = true;
        }

        // Time scale controls: Space=pause, >=faster, <=slower
        if (e.KeyCode == Keys.Space)
        {
            _simLoop.Paused = !_simLoop.Paused;
            e.Handled = true;
        }
        if (e.KeyCode == Keys.OemPeriod)
        {
            _simLoop.TimeScaleExponent++;
            _simLoop.Paused = false;
            e.Handled = true;
        }
        if (e.KeyCode == Keys.Oemcomma)
        {
            if (_simLoop.TimeScaleExponent > 0) _simLoop.TimeScaleExponent--;
            e.Handled = true;
        }

        // G = toggle arc conflict debug overlay + logging
        if (e.KeyCode == Keys.G)
        {
            VehicleRenderer.ShowArcConflicts = !VehicleRenderer.ShowArcConflicts;
            if (VehicleRenderer.ShowArcConflicts)
                SteeringController.DebugLoggingEnabled = true;
            e.Handled = true;
        }

        // D = dump selected vehicle debug info to file
        if (e.KeyCode == Keys.D && _editorState.SelectedVehicle >= 0
            && _editorState.SelectedVehicle < _vehicles.Count)
        {
            DumpVehicleDiag(_editorState.SelectedVehicle);
            e.Handled = true;
        }

        // F = toggle per-frame diagnostic logging for selected vehicle
        if (e.KeyCode == Keys.F && _editorState.SelectedVehicle >= 0
            && _editorState.SelectedVehicle < _vehicles.Count)
        {
            if (_vehicles.DiagVehicle == _editorState.SelectedVehicle)
            {
                _vehicles.DiagVehicle = -1;
                SteeringController.LogDiag(_vehicles, _editorState.SelectedVehicle, "DIAG_OFF");
            }
            else
            {
                _vehicles.DiagVehicle = _editorState.SelectedVehicle;
                SteeringController.LogDiag(_vehicles, _editorState.SelectedVehicle, "DIAG_ON");
            }
            e.Handled = true;
        }
    }

    /// <summary>
    /// Renders the entire scene: background grid, roads with signals/signs, vehicles,
    /// spawn/destination points, control point handles (in Select mode), road tool preview,
    /// snap indicator, and the UI overlay (toolbar + status text + slider panel).
    /// </summary>
    private void OnPaintSurface(object? sender, SKCanvas canvas, SKImageInfo info)
    {
        _sceneRenderer.Render(canvas, info, _camera, _roadGraph, _vehicles, _editorState,
            _stopLineCache, _intersectionArcs,
            _trafficSignals, _stopSigns, _yieldSigns, _simLoop,
            _spawner.SpawnNodeCount, _currentMousePos);
    }

    /// <summary>
    /// Handles mouse-down: middle button starts panning, left button dispatches to the
    /// slider panel, toolbar hit-test, or the active editor tool (Select, Road, Delete,
    /// SpawnPoint, Destination, Signal). Right button cancels the road tool.
    /// </summary>
    private void OnCanvasMouseDown(object? sender, MouseEventArgs e)
    {
        _canvas.Focus();

        if (e.Button == MouseButtons.Middle)
        {
            _isPanning = true;
            _lastMousePos = e.Location;
            _canvas.Cursor = Cursors.SizeAll;
            return;
        }

        if (e.Button == MouseButtons.Left)
        {
            // Check slider panel first
            if (_sliderPanel.OnMouseDown(e.X, e.Y))
                return;

            // Check toolbar
            var hitTool = _uiRenderer.HitTest(e.X, e.Y);
            if (hitTool.HasValue)
            {
                _editorState.ResetToolState();
                _editorState.ActiveTool = hitTool.Value;
                return;
            }

            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            var worldVec = new Vector2(worldPos.X, worldPos.Y);

            switch (_editorState.ActiveTool)
            {
                case EditorTool.Select:
                {
                    // Lane restriction mode: click to select outgoing lane or toggle incoming lane
                    if (_editorState.LaneRestrictionMode && _editorState.SelectedNode >= 0)
                    {
                        _laneRestrictionTool.OnClick(worldVec, _roadGraph, _editorState, _stopLineCache);
                        break;
                    }

                    // Try vehicle hit-test first
                    var hitResults = new List<int>();
                    _vehicleGrid.QueryFiltered(worldVec.X, worldVec.Y, 5f, _vehicles.PosX, _vehicles.PosY, hitResults);
                    int closestVeh = -1;
                    float closestVehDist = float.MaxValue;
                    foreach (int vi in hitResults)
                    {
                        if (_vehicles.State[vi] != VehicleState.Driving) continue;
                        float d = Vector2.Distance(worldVec, new Vector2(_vehicles.PosX[vi], _vehicles.PosY[vi]));
                        if (d < closestVehDist)
                        {
                            closestVehDist = d;
                            closestVeh = vi;
                        }
                    }

                    if (closestVeh >= 0 && closestVehDist < 5f)
                    {
                        _editorState.SelectedVehicle = closestVeh;
                        _editorState.SelectedNode = -1;
                        _editorState.SelectedEdge = -1;
                        break;
                    }

                    _editorState.SelectedVehicle = -1;

                    // Find nearest node and control point, pick whichever is closer
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    var (cpEdgeIdx, cpIdx) = _roadGraph.FindNearestControlPoint(worldVec, EditorState.SnapDistance);

                    float nodeDist = nearNode >= 0
                        ? Vector2.Distance(worldVec, _roadGraph.Nodes[nearNode].Position)
                        : float.MaxValue;
                    float cpDist = cpEdgeIdx >= 0
                        ? Vector2.Distance(worldVec, cpIdx == 1
                            ? _roadGraph.Edges[cpEdgeIdx].ControlPoint1
                            : _roadGraph.Edges[cpEdgeIdx].ControlPoint2)
                        : float.MaxValue;

                    if (nearNode >= 0 && nodeDist <= cpDist)
                    {
                        _editorState.SelectedNode = nearNode;
                        _editorState.SelectedEdge = -1;
                        _editorState.DragNodeIndex = nearNode;
                        _canvas.Cursor = Cursors.Hand;
                    }
                    else if (cpEdgeIdx >= 0)
                    {
                        _editorState.SelectedNode = -1;
                        _editorState.DragEdgeIndex = cpEdgeIdx;
                        _editorState.DragControlPointIndex = cpIdx;
                        _editorState.SelectedEdge = cpEdgeIdx;
                        _canvas.Cursor = Cursors.Hand;
                    }
                    else
                    {
                        _editorState.SelectedNode = -1;
                        // Select nearest edge
                        int nearEdge = _edgeSpatialGrid.FindNearestEdge(_roadGraph, worldVec, EditorState.SnapDistance);
                        _editorState.SelectedEdge = nearEdge;
                    }
                    break;
                }
                case EditorTool.Road:
                    _roadTool.OnClick(worldVec, _roadGraph, _editorState, _edgeSpatialGrid);
                    _graphChangeHandler.HandleIfNeeded();
                    break;
                case EditorTool.Delete:
                    _deleteTool.OnClick(worldVec, _roadGraph, _edgeSpatialGrid);
                    _graphChangeHandler.HandleIfNeeded();
                    break;
                case EditorTool.SpawnPoint:
                    _spawnPointTool.OnClick(worldVec, _roadGraph);
                    break;
                case EditorTool.Destination:
                    _destinationTool.OnClick(worldVec, _roadGraph);
                    break;
                case EditorTool.Signal:
                    _signalTool.OnClick(worldVec, _roadGraph);
                    break;
            }
        }
        else if (e.Button == MouseButtons.Right)
        {
            var rWorldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            var rWorldVec = new Vector2(rWorldPos.X, rWorldPos.Y);

            switch (_editorState.ActiveTool)
            {
                case EditorTool.Road:
                    _roadTool.OnCancel(_editorState);
                    break;
                case EditorTool.SpawnPoint:
                    RemoveNearestFlag(rWorldVec, NodeFlags.Spawn);
                    break;
                case EditorTool.Destination:
                    RemoveNearestFlag(rWorldVec, NodeFlags.Destination);
                    break;
                case EditorTool.Signal:
                    _signalTool.OnRightClick(rWorldVec, _roadGraph, _edgeSpatialGrid,
                        _trafficSignals, _stopSigns, _yieldSigns);
                    break;
                case EditorTool.Select:
                {
                    // Right-click on edge: split it to create a new node
                    var (nearEdge, nearT) = _edgeSpatialGrid.FindNearestEdgeWithT(
                        _roadGraph, rWorldVec, EditorState.SnapDistance);
                    if (nearEdge >= 0)
                    {
                        nearT = Math.Clamp(nearT, 0.05f, 0.95f);
                        _roadGraph.SplitEdge(nearEdge, nearT);
                        _graphChangeHandler.HandleIfNeeded();
                    }
                    break;
                }
            }
        }
    }

    /// <summary>
    /// Handles mouse-up: ends panning (middle button) or control point dragging (left button).
    /// </summary>
    private void OnCanvasMouseUp(object? sender, MouseEventArgs e)
    {
        if (e.Button == MouseButtons.Middle)
        {
            _isPanning = false;
            _canvas.Cursor = Cursors.Default;
        }
        else if (e.Button == MouseButtons.Left)
        {
            _sliderPanel.OnMouseUp();
            if (_editorState.IsDraggingNode)
            {
                // Split edges at any crossings created by the drag
                if (_editorState.DragCrossingPreviews.Count > 0)
                    _roadGraph.SplitNodeEdgeCrossings(_editorState.DragNodeIndex);

                _editorState.DragCrossingPreviews.Clear();
                _editorState.DragNodeIndex = -1;
                _canvas.Cursor = Cursors.Default;
                _graphChangeHandler.HandleIfNeeded();
            }
            else if (_editorState.IsDraggingControlPoint)
            {
                _editorState.DragEdgeIndex = -1;
                _editorState.DragControlPointIndex = -1;
                _canvas.Cursor = Cursors.Default;
                _graphChangeHandler.HandleIfNeeded();
            }
        }
    }

    /// <summary>
    /// Handles mouse movement: updates slider drags, pans the camera, or moves a dragged
    /// control point in Select mode.
    /// </summary>
    private void OnCanvasMouseMove(object? sender, MouseEventArgs e)
    {
        _currentMousePos = e.Location;

        if (_sliderPanel.OnMouseMove(e.X, e.Y))
            return;

        if (_isPanning)
        {
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _camera.Pan(dx, dy);
            _lastMousePos = e.Location;
        }
        else if (_editorState.IsDraggingNode)
        {
            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            _roadGraph.MoveNode(_editorState.DragNodeIndex, new Vector2(worldPos.X, worldPos.Y));

            // Detect crossing previews for edges connected to the dragged node
            _editorState.DragCrossingPreviews.Clear();
            var crossings = _roadGraph.FindNodeEdgeCrossings(_editorState.DragNodeIndex);
            foreach (var (_, _, _, _, pos) in crossings)
                _editorState.DragCrossingPreviews.Add(pos);
        }
        else if (_editorState.IsDraggingControlPoint)
        {
            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            _roadGraph.SetControlPoint(
                _editorState.DragEdgeIndex,
                _editorState.DragControlPointIndex,
                new Vector2(worldPos.X, worldPos.Y));
        }
    }

    /// <summary>
    /// Zooms the camera toward/away from the mouse cursor position.
    /// </summary>
    private void OnCanvasMouseWheel(object? sender, MouseEventArgs e)
    {
        float zoomFactor = e.Delta > 0 ? 1.15f : 1f / 1.15f;
        _camera.ZoomAt(zoomFactor, e.X, e.Y, _canvas.Width, _canvas.Height);
    }

    /// <summary>
    /// Clears the given flag from the nearest flagged node within snap distance (right-click removal).
    /// </summary>
    private void RemoveNearestFlag(Vector2 worldPos, NodeFlags flag)
    {
        int bestNode = -1;
        float bestDist = EditorState.SnapDistance * EditorState.SnapDistance;
        for (int i = 0; i < _roadGraph.Nodes.Count; i++)
        {
            var node = _roadGraph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if (!node.Flags.HasFlag(flag)) continue;
            float d = Vector2.DistanceSquared(worldPos, node.Position);
            if (d < bestDist)
            {
                bestDist = d;
                bestNode = i;
            }
        }
        if (bestNode >= 0)
            _roadGraph.SetNodeFlags(bestNode, _roadGraph.Nodes[bestNode].Flags & ~flag);
    }

}
