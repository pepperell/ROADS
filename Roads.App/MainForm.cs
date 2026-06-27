using System.Diagnostics;
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
    private readonly SpatialGrid _vehicleGrid = new();
    private readonly StopLineCache _stopLineCache = new();
    private readonly EdgeSpatialGrid _edgeSpatialGrid = new();
    private readonly TrafficSignalSystem _trafficSignals = new();
    private readonly StopSignSystem _stopSigns = new();
    private readonly YieldSignSystem _yieldSigns = new();
    private readonly IntersectionArcCache _intersectionArcs = new();
    private readonly VehicleInfoPanel _vehicleInfoPanel = new();
    private readonly MinimapRenderer _minimap = new();
    private readonly PerformanceHud _perfHud = new();
    private readonly StatisticsPanel _statisticsPanel = new();
    private readonly Stopwatch _perfStopwatch = new();
    private readonly Stopwatch _autoSaveClock = Stopwatch.StartNew();
    private readonly POIRegistry _poiRegistry = new();
    private readonly VehicleSpawner _spawner;
    private readonly PopulationManager _populationManager;
    private readonly GraphChangeHandler _graphChangeHandler;
    private readonly SimulationLoop _simLoop;
    private readonly SceneRenderer _sceneRenderer;
    private readonly Persistence.AutoSaveManager _autoSave;

    private Point _lastMousePos;
    private Point _currentMousePos;
    private bool _isPanning;
    /// <summary>Whether a left-button drag is currently scrubbing the camera via the minimap.</summary>
    private bool _draggingMinimap;

    /// <summary>Screen position where a drag-candidate click occurred.</summary>
    private Point _dragStartScreenPos;
    /// <summary>World-space offset from the cursor to the dragged item's position at click time.</summary>
    private Vector2 _dragOffset;
    /// <summary>Whether the drag dead zone (5 px) has been exceeded.</summary>
    private bool _dragActive;
    private const int DragDeadZone = 5;

    /// <summary>World-space radius within which the Destination placement tool will attach a
    /// connector to the nearest road. Larger than SnapDistance so a destination can be dropped
    /// in open space and still reach a road.</summary>
    private const float DestPlacementSearchRadius = 200f;

    /// <summary>Autobench (see Program <c>--autobench</c>): target frame count (0 = disabled).</summary>
    private readonly int _autoBenchFrames;
    /// <summary>Frames elapsed since the autobench run started.</summary>
    private int _autoBenchFrameCount;

    /// <summary>Per-vehicle count of consecutive simulation ticks spent essentially stopped
    /// (speed &lt; <see cref="StuckSpeedThreshold"/>), reset to 0 the moment it moves. Updated every
    /// tick in <see cref="OnTick"/>; read by the deadlock-capture dump (press <c>D</c>) to report how
    /// long each car has been wedged and to gather a whole stuck cluster. Index-aligned to the
    /// VehicleStore; follows swap-and-pop removals via <see cref="OnVehicleRemoving"/>.</summary>
    private int[] _stuckTicks = System.Array.Empty<int>();
    /// <summary>Speed (m/s) below which a vehicle counts as "stopped" for stuck-time tracking.</summary>
    private const float StuckSpeedThreshold = 0.1f;
    /// <summary>Ticks stopped before a vehicle is treated as a deadlock candidate by the cluster dump
    /// (press <c>D</c> with nothing selected). ~6 s at the sim tick rate — long past any stop-sign wait
    /// or queue, so a normal red light never trips it.</summary>
    private const int StuckTickThreshold = 360;

    public MainForm(int autoBenchFrames = 0)
    {
        _autoBenchFrames = autoBenchFrames;
        Text = "ROADS - Traffic Simulation";
        Width = 1280;
        Height = 720;
        DoubleBuffered = true;
        StartPosition = FormStartPosition.CenterScreen;

        _editorState.ActiveTool = EditorTool.Road;
        _spawner = new VehicleSpawner(_roadGraph, _vehicles, _vehicleGrid);
        _populationManager = new PopulationManager(_roadGraph, _vehicles, _vehicleGrid, _poiRegistry, SimulationLoop.MaxVehicles);
        _graphChangeHandler = new GraphChangeHandler(_roadGraph, _editorState, _vehicles, _edgeSpatialGrid, _spawner);
        _simLoop = new SimulationLoop(_roadGraph, _vehicles, _vehicleGrid, _stopLineCache, _intersectionArcs, _edgeSpatialGrid, _trafficSignals, _stopSigns, _yieldSigns, _spawner, _populationManager, _editorState, _graphChangeHandler);
        _sceneRenderer = new SceneRenderer(_roadRenderer, _vehicleRenderer, _spawnPointRenderer, _uiRenderer, _sliderPanel, _vehicleInfoPanel, _laneRestrictionTool, _minimap, _statisticsPanel);
        // AutoSaveManager shares the same object references used by SaveMap() so the
        // backup format is identical to a manual save (minus vehicles, which are transient).
        // Triggered from the render-timer tick so it runs on the UI thread with no locking.
        _autoSave = new Persistence.AutoSaveManager(_roadGraph, _vehicles, _camera,
            _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals, _populationManager);

        // Centralized vehicle-removal fixup: editor-held vehicle indices follow
        // swap-and-pop moves and drop on bulk clears (see VehicleStore.VehicleRemoving).
        // The vehicle spatial grid holds indices too — it self-heals on the same events
        // so editor-time hit-tests between grid rebuilds never see stale indices.
        _vehicles.VehicleRemoving += OnVehicleRemoving;
        _vehicles.VehiclesCleared += OnVehiclesCleared;
        _vehicles.VehicleRemoving += _vehicleGrid.OnEntityRemoving;
        _vehicles.VehiclesCleared += _vehicleGrid.Clear;

        // Migrate stop/yield exemptions across edge splits (they are keyed by edge index, which
        // SplitEdge changes) so splitting an exempt approach doesn't silently re-stop the road.
        _roadGraph.EdgeSplit += _stopSigns.OnEdgeSplit;
        _roadGraph.EdgeSplit += _yieldSigns.OnEdgeSplit;

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
        _perfStopwatch.Restart();
        _simLoop.Tick();
        _perfHud.RecordSimTime(_perfStopwatch.Elapsed.TotalMilliseconds);

        // Accumulate wall time and trigger a backup when the interval elapses.
        // The auto-save clock is independent of simulation time and pause state
        // so that backups run on a predictable wall-clock cadence.
        double tickWall = _autoSaveClock.Elapsed.TotalSeconds;
        _autoSaveClock.Restart();
        _autoSave.MaybeSave(tickWall);

        _canvas.Invalidate();

        if (_autoBenchFrames > 0)
            AutoBenchStep();

        UpdateStuckTracking();
    }

    /// <summary>
    /// Per-tick maintenance of <see cref="_stuckTicks"/>: increments the counter for every vehicle
    /// essentially stopped, and resets the moment it moves. Cheap O(n) pass that powers the
    /// deadlock-capture dump (press <c>D</c>). Skipped while paused so a deliberately-paused sim
    /// doesn't inflate the counters.
    /// </summary>
    private void UpdateStuckTracking()
    {
        if (_simLoop.Paused) return;
        if (_stuckTicks.Length < _vehicles.Count)
            System.Array.Resize(ref _stuckTicks, _vehicles.Count + 64);

        for (int v = 0; v < _vehicles.Count; v++)
        {
            if (_vehicles.State[v] == VehicleState.Driving && _vehicles.Speed[v] < StuckSpeedThreshold)
                _stuckTicks[v]++;
            else
                _stuckTicks[v] = 0;
        }
    }

    /// <summary>
    /// Captures a live deadlock to diag_vehicle.log. With a vehicle selected, dumps it and walks its
    /// blocking chain, dumping every distinct car in the chain so the full cycle is recorded from one
    /// keypress. With nothing selected, dumps every vehicle stopped past <see cref="StuckTickThreshold"/>
    /// — the whole wedged cluster. The arc/intersection state that causes the wedge is NOT saved to the
    /// map, so this text dump is the only durable record; use it the moment a deadlock is on screen.
    /// </summary>
    private void CaptureDeadlock()
    {
        var dumped = new HashSet<int>();

        if (_editorState.SelectedVehicle >= 0 && _editorState.SelectedVehicle < _vehicles.Count)
        {
            // Selected car is the entry point: dump it, then follow "nearest ahead" through the chain
            // (which terminates at a signal-held car, a moving car, or a revisited car = a true cycle).
            int cur = _editorState.SelectedVehicle;
            for (int hop = 0; hop < 16 && cur >= 0 && cur < _vehicles.Count; hop++)
            {
                if (!dumped.Add(cur)) break; // cycle closed
                DumpVehicleDiag(cur);
                int next = NearestVehicleAhead(cur, out _, out _);
                if (next < 0 || _vehicles.Speed[next] > 1.0f) break;
                cur = next;
            }
        }
        else
        {
            // No selection: dump the whole cluster of long-stopped cars.
            for (int v = 0; v < _vehicles.Count; v++)
                if (v < _stuckTicks.Length && _stuckTicks[v] >= StuckTickThreshold)
                {
                    DumpVehicleDiag(v);
                    dumped.Add(v);
                }
            if (dumped.Count == 0)
            {
                // Nothing met the threshold; still record the worst offender so a keypress is never a no-op.
                int worst = -1, worstTicks = 0;
                for (int v = 0; v < _vehicles.Count && v < _stuckTicks.Length; v++)
                    if (_stuckTicks[v] > worstTicks) { worstTicks = _stuckTicks[v]; worst = v; }
                if (worst >= 0) DumpVehicleDiag(worst);
            }
        }
    }

    /// <summary>
    /// Drives the headless 10K benchmark (Program <c>--autobench</c>): builds the stress scene on
    /// the first frame, appends metrics to benchmark.log over the final 30 frames (so a parser can
    /// average a stable window), then closes the app at the target frame count.
    /// </summary>
    private void AutoBenchStep()
    {
        if (_autoBenchFrameCount == 0)
            GenerateStressScene(5f); // benchmark at the 5x target view (culling active), not the overview

        _autoBenchFrameCount++;

        if (_autoBenchFrameCount > _autoBenchFrames - 30)
            CaptureBaseline();

        if (_autoBenchFrameCount >= _autoBenchFrames)
            Close();
    }

    /// <summary>
    /// Keeps editor-held vehicle indices valid across swap-and-pop removals: an index
    /// equal to the removed slot clears; one equal to the swapped-from (last) slot
    /// follows that vehicle to its new index.
    /// </summary>
    private void OnVehicleRemoving(int removed, int swappedFrom)
    {
        _editorState.SelectedVehicle = FixupVehicleIndex(_editorState.SelectedVehicle, removed, swappedFrom);
        _editorState.HoveredVehicle = FixupVehicleIndex(_editorState.HoveredVehicle, removed, swappedFrom);

        // Keep the stuck-time tracker index-aligned across swap-and-pop: the vehicle at swappedFrom
        // moved into the removed slot, so its counter follows it.
        if (swappedFrom >= 0 && removed >= 0 && removed < _stuckTicks.Length && swappedFrom < _stuckTicks.Length)
            _stuckTicks[removed] = _stuckTicks[swappedFrom];
    }

    /// <summary>Drops editor-held vehicle indices when the store is bulk-cleared.</summary>
    private void OnVehiclesCleared()
    {
        _editorState.SelectedVehicle = -1;
        _editorState.HoveredVehicle = -1;
    }

    private static int FixupVehicleIndex(int holder, int removed, int swappedFrom)
    {
        if (holder < 0) return holder; // nothing held — and never match the swappedFrom=-1 sentinel
        if (holder == removed) return -1;
        if (holder == swappedFrom) return removed;
        return holder;
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
            if (_stopSigns.CanQuery(_roadGraph) && _yieldSigns.CanQuery(_roadGraph))
            {
                var stopSignal = _stopSigns.GetSignal(edgeIdx, _roadGraph, i);
                var yieldSignal = _yieldSigns.GetSignal(edgeIdx, _roadGraph);
                sb.AppendLine($"    Signals: traffic={signal} stopSign={stopSignal} yield={yieldSignal}");
            }
            else
            {
                sb.AppendLine($"    Signals: traffic={signal} stopSign=n/a yield=n/a (right-of-way tracking stale)");
            }

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

        // Intent / off-road context.
        sb.AppendLine($"  ResidentId: {_vehicles.ResidentId[i]}  DistToRoadSq: {_vehicles.DistToRoadSq[i]:F2}");
        int destNode = _vehicles.DestinationNode[i];
        if (destNode >= 0 && destNode < _roadGraph.Nodes.Count)
            sb.AppendLine($"  Destination: node {destNode} (POI {_roadGraph.Nodes[destNode].PointOfInterest}, flags {_roadGraph.Nodes[destNode].Flags})");

        // --- BLOCKER ANALYSIS: why is this car not moving? ---
        sb.AppendLine("  --- BLOCKER ANALYSIS ---");

        int stuckTicks = i < _stuckTicks.Length ? _stuckTicks[i] : 0;
        sb.AppendLine($"  Stuck for: {stuckTicks} ticks (~{stuckTicks / 60f:F1}s continuously stopped at ~60Hz)");

        int leader = NearestVehicleAhead(i, out float gap, out float lat);
        if (leader >= 0)
            sb.AppendLine($"  Nearest ahead: vehicle {leader} gap={gap:F2}m lateral={lat:F2}m " +
                $"speed={_vehicles.Speed[leader]:F2} edge={_vehicles.CurrentEdge[leader]} arc={_vehicles.CurrentArc[leader]} prog={_vehicles.EdgeProgress[leader]:F3}");
        else
            sb.AppendLine("  Nearest ahead: none within 30m (so it is held by a signal/stop, not a car)");

        // Stop-sign first-come-first-served detail (explains a never-granted green).
        if (edgeIdx >= 0 && edgeIdx < _roadGraph.Edges.Count && _roadGraph.Edges[edgeIdx].FromNode >= 0
            && _stopSigns.CanQuery(_roadGraph))
            sb.AppendLine($"  StopSign FCFS: {_stopSigns.DescribeStopState(_roadGraph, edgeIdx)}");

        // Intended turn arc + conflicting arcs and who occupies them (the intersection-deadlock view).
        if (_vehicles.CurrentArc[i] < 0 && edgeIdx >= 0 && path != null && pathIdx + 1 < path.Count)
        {
            int nextEdge = path[pathIdx + 1];
            if (nextEdge >= 0 && nextEdge < _roadGraph.Edges.Count && _roadGraph.Edges[nextEdge].FromNode >= 0)
            {
                byte inLane = _vehicles.CurrentLane[i];
                byte outLane = (byte)Math.Min(inLane, _roadGraph.Edges[nextEdge].LaneCount - 1);
                int arcIdx = _intersectionArcs.GetArcIndex(edgeIdx, nextEdge, inLane, outLane);
                sb.AppendLine($"  Intended turn: edge {edgeIdx} -> {nextEdge}, arc={arcIdx}");
                if (arcIdx >= 0)
                {
                    foreach (int c in _intersectionArcs.GetConflictingArcs(arcIdx))
                    {
                        var ca = _intersectionArcs.GetArc(c);
                        sb.AppendLine($"    conflictArc {c} ({ca.IncomingEdge}->{ca.OutgoingEdge}) occupants: {VehiclesOnArc(c)}");
                    }
                }
            }
        }

        // All arcs at the node being entered, with occupants (arc state is NOT saved, so this live
        // snapshot is the only way to see who is wedged mid-intersection).
        int approachNode = _vehicles.CurrentArc[i] >= 0
            ? _intersectionArcs.GetArc(_vehicles.CurrentArc[i]).NodeIndex
            : (edgeIdx >= 0 && edgeIdx < _roadGraph.Edges.Count ? _roadGraph.Edges[edgeIdx].ToNode : -1);
        if (approachNode >= 0)
        {
            sb.AppendLine($"  Arcs at node {approachNode}:");
            foreach (int a in _intersectionArcs.GetArcsAtNode(approachNode))
            {
                var ad = _intersectionArcs.GetArc(a);
                string occ = VehiclesOnArc(a);
                if (occ != "(empty)")
                    sb.AppendLine($"    arc {a} ({ad.IncomingEdge}->{ad.OutgoingEdge}) occupants: {occ}");
            }
        }

        // Follow the queue/conflict ahead to expose a deadlock cycle (A->B->...->A).
        sb.AppendLine($"  Blocking chain: {BuildBlockingChain(i)}");

        sb.AppendLine();
        File.AppendAllText("diag_vehicle.log", sb.ToString());
    }

    /// <summary>
    /// Nearest other driving vehicle ahead of <paramref name="v"/> within a 30 m forward cone
    /// (generous ±5 m lateral so cross-traffic at an intersection is caught). Returns its index and
    /// the bumper gap, or -1 if nothing is ahead (then the vehicle is held by a signal/stop, not a car).
    /// </summary>
    private int NearestVehicleAhead(int v, out float gap, out float lateral)
    {
        gap = float.MaxValue; lateral = 0f;
        float vx = _vehicles.PosX[v], vy = _vehicles.PosY[v];
        float fx, fy;
        int e = _vehicles.CurrentEdge[v];
        if (_vehicles.CurrentArc[v] < 0 && e >= 0 && e < _roadGraph.Edges.Count && _roadGraph.Edges[e].FromNode >= 0)
        {
            var t = _roadGraph.EvaluateBezierTangent(e, _vehicles.EdgeProgress[v]);
            float tl = t.Length();
            if (tl > 0.001f) { fx = t.X / tl; fy = t.Y / tl; }
            else { fx = MathF.Cos(_vehicles.Heading[v]); fy = MathF.Sin(_vehicles.Heading[v]); }
        }
        else { fx = MathF.Cos(_vehicles.Heading[v]); fy = MathF.Sin(_vehicles.Heading[v]); }

        var buf = new List<int>();
        _vehicleGrid.QueryFiltered(vx, vy, 30f, _vehicles.PosX, _vehicles.PosY, buf);
        int best = -1; float bestFwd = float.MaxValue;
        foreach (int o in buf)
        {
            if (o == v || _vehicles.State[o] != VehicleState.Driving) continue;
            float dx = _vehicles.PosX[o] - vx, dy = _vehicles.PosY[o] - vy;
            float fwd = dx * fx + dy * fy;
            if (fwd <= 0f) continue;
            float la = -dx * fy + dy * fx;
            if (MathF.Abs(la) > 5f) continue;
            if (fwd < bestFwd) { bestFwd = fwd; best = o; lateral = la; }
        }
        if (best >= 0) gap = bestFwd - SimConstants.VehicleLength;
        return best;
    }

    /// <summary>Comma-separated indices of driving vehicles currently on the given arc, or "(empty)".</summary>
    private string VehiclesOnArc(int arcIdx)
    {
        var list = new List<int>();
        for (int o = 0; o < _vehicles.Count; o++)
            if (_vehicles.State[o] == VehicleState.Driving && _vehicles.CurrentArc[o] == arcIdx)
                list.Add(o);
        return list.Count == 0 ? "(empty)" : string.Join(",", list);
    }

    /// <summary>
    /// Walks "nearest vehicle ahead" from <paramref name="v"/> to expose the head of the queue or a
    /// deadlock cycle. Stops at a vehicle with no car ahead (signal/stop-held), a moving non-blocker,
    /// or a revisited vehicle (a true deadlock loop, flagged explicitly).
    /// </summary>
    private string BuildBlockingChain(int v)
    {
        var sb = new System.Text.StringBuilder();
        var seen = new HashSet<int>();
        int cur = v;
        for (int hop = 0; hop < 16; hop++)
        {
            if (!seen.Add(cur)) { sb.Append($"{cur} <== DEADLOCK CYCLE"); break; }
            sb.Append(cur);
            int next = NearestVehicleAhead(cur, out float g, out _);
            if (next < 0) { sb.Append(" -> [held by signal/stop or clear]"); break; }
            if (_vehicles.Speed[next] > 1.0f && g > 8f) { sb.Append($" -> {next} (moving, not blocking)"); break; }
            sb.Append($" -[{g:F1}m]-> ");
            cur = next;
        }
        return sb.ToString();
    }

    /// <summary>
    /// Handles keyboard input: V to spawn vehicles, +/- to adjust lane count on selected edge,
    /// [/] to adjust speed limit, T to toggle the slider panel, P to toggle the performance HUD,
    /// M to toggle the minimap, N to toggle the statistics panel, Space to pause/unpause,
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

        // R key: cycle the selected edge's RoadType (Residential → Arterial → Highway → Dirt → Residential)
        if (e.KeyCode == Keys.R && _editorState.ActiveTool == EditorTool.Select
            && _editorState.SelectedEdge >= 0)
        {
            var cycleEdge = _roadGraph.Edges[_editorState.SelectedEdge];
            if (cycleEdge.FromNode >= 0)
            {
                RoadType nextType = cycleEdge.RoadType switch
                {
                    RoadType.Residential => RoadType.Arterial,
                    RoadType.Arterial    => RoadType.Highway,
                    RoadType.Highway     => RoadType.Dirt,
                    _                   => RoadType.Residential,
                };
                _roadGraph.SetEdgeRoadType(_editorState.SelectedEdge, nextType);
                e.Handled = true;
            }
        }

        // O key: cycle the selected road two-way → one-way → one-way reversed → two-way.
        // The two one-way states are topologically identical, so the cycle step is tracked
        // in EditorState and reset when a different edge is cycled.
        if (e.KeyCode == Keys.O && !e.Control && _editorState.ActiveTool == EditorTool.Select
            && _editorState.SelectedEdge >= 0
            && _roadGraph.Edges[_editorState.SelectedEdge].FromNode >= 0)
        {
            int sel = _editorState.SelectedEdge;
            if (_editorState.OneWayCycleEdge != sel)
            {
                _editorState.OneWayCycleEdge = sel;
                // Seed the step from the current topology so a loaded/just-selected road
                // cycles sensibly: two-way starts at 0, an existing one-way at 1.
                _editorState.OneWayCycleStep = _roadGraph.FindReverseEdge(sel) >= 0 ? 0 : 1;
            }

            switch (_editorState.OneWayCycleStep)
            {
                case 0: _roadGraph.MakeOneWay(sel); _editorState.OneWayCycleStep = 1; break;
                case 1: _roadGraph.ReverseOneWay(sel); _editorState.OneWayCycleStep = 2; break;
                default:
                    // Flip the edge back to its original orientation before re-adding the reverse,
                    // so a full cycle returns to the EXACT starting two-way road. Without this,
                    // ReverseOneWay's in-place flip persists and the next lap's one-way (and its
                    // arrows) points the opposite direction.
                    _roadGraph.ReverseOneWay(sel);
                    _roadGraph.MakeTwoWay(sel);
                    _editorState.OneWayCycleStep = 0;
                    break;
            }
            e.Handled = true;
        }

        // J key: toggle single-lane two-way (shared lane) on a selected two-way road. Valid only
        // on a two-way road (a pair); one physical lane shared both ways, gated against oncoming.
        if (e.KeyCode == Keys.J && !e.Control && _editorState.ActiveTool == EditorTool.Select
            && _editorState.SelectedEdge >= 0
            && _roadGraph.Edges[_editorState.SelectedEdge].FromNode >= 0)
        {
            int sel = _editorState.SelectedEdge;
            if (_roadGraph.FindReverseEdge(sel) >= 0)
            {
                bool isShared = (_roadGraph.Edges[sel].Flags & EdgeFlags.SharedLane) != 0;
                _roadGraph.SetSharedLane(sel, !isShared);
            }
            e.Handled = true;
        }

        // Delete key to delete selected node. Routed through the population coordinator so a
        // populated node drains its people out before disappearing instead of vanishing instantly.
        if (e.KeyCode == Keys.Delete && _editorState.ActiveTool == EditorTool.Select
            && _editorState.SelectedNode >= 0)
        {
            _populationManager.RequestDeleteNode(_editorState.SelectedNode);
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

        if (e.KeyCode == Keys.P)
        {
            _perfHud.Visible = !_perfHud.Visible;
            e.Handled = true;
        }

        if (e.KeyCode == Keys.M)
        {
            _minimap.Visible = !_minimap.Visible;
            e.Handled = true;
        }

        // N = toggle statistics panel (vehicle count, avg speed, congestion)
        if (e.KeyCode == Keys.N)
        {
            _statisticsPanel.Visible = !_statisticsPanel.Visible;
            e.Handled = true;
        }

        // H = toggle the congestion heat-map overlay
        if (e.KeyCode == Keys.H)
        {
            _sceneRenderer.HeatMapEnabled = !_sceneRenderer.HeatMapEnabled;
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

        // D = capture a live deadlock to diag_vehicle.log. With a vehicle selected, dumps it and walks
        // the blocking chain (the full cycle); with NOTHING selected, dumps every car stuck > ~6s (the
        // whole wedged cluster). Press it the moment a deadlock is on screen — the arc/intersection
        // wedge state is not saved, so this text dump is the only durable record.
        if (e.KeyCode == Keys.D)
        {
            CaptureDeadlock();
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

        // Ctrl+S: Save map
        if (e.KeyCode == Keys.S && e.Control)
        {
            SaveMap();
            e.Handled = true;
        }

        // Ctrl+O: Load map
        if (e.KeyCode == Keys.O && e.Control)
        {
            LoadMap();
            e.Handled = true;
        }

        // K: generate stress-test grid scene with bulk vehicle spawn
        if (e.KeyCode == Keys.K)
        {
            GenerateStressScene();
            e.Handled = true;
        }

        // B: capture performance baseline snapshot to benchmark.log
        if (e.KeyCode == Keys.B)
        {
            CaptureBaseline();
            e.Handled = true;
        }
    }

    /// <summary>
    /// Clears all map data and resets the editor to a blank state, as if the app just opened.
    /// </summary>
    private void NewMap()
    {
        var result = MessageBox.Show("Create a new map? All unsaved changes will be lost.",
            "New Map", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
        if (result != DialogResult.OK) return;

        _vehicles.ClearAll();
        _roadGraph.LoadFromData(new List<World.RoadNode>(), new List<World.RoadEdge>());

        // Reset per-map traffic-control overrides (exemptions, phase rotations). They
        // survive graph edits by design, so replacing the whole map must clear them
        // explicitly — mirrors MapSerializer.Load's clear-then-set semantics; otherwise
        // the old map's overrides silently apply to reused node/edge indices.
        _stopSigns.SetExemptEdges(new List<int>());
        _yieldSigns.SetExemptEdges(new List<int>());
        _trafficSignals.SetPhaseRotations(new List<(int, byte)>());

        _simLoop.RebuildWorldCaches();

        _camera.CenterX = 0;
        _camera.CenterY = 0;
        _camera.Zoom = 5.0f;
        _simLoop.Clock.TimeOfDay = 8.0;
        _simLoop.Paused = false;
        _simLoop.TimeScaleExponent = 0;

        _editorState.SelectedEdge = -1;
        _editorState.SelectedNode = -1;
        _editorState.SelectedVehicle = -1;
        _editorState.LaneRestrictionMode = false;
        _editorState.LaneRestrictionEdge = -1;
        _editorState.RoadStartNode = null;
        _editorState.ActiveTool = Editor.EditorTool.Select;
    }

    /// <summary>
    /// Prompts for a file path and saves the current map. Asks whether to include vehicles.
    /// A <c>.json</c> file name writes a human-readable snapshot via <see cref="Persistence.MapJsonSerializer"/>;
    /// any other extension uses the binary <see cref="Persistence.MapSerializer"/> format.
    /// </summary>
    private void SaveMap()
    {
        bool wasPaused = _simLoop.Paused;
        _simLoop.Paused = true;

        using var dlg = new SaveFileDialog
        {
            Title = "Save Map",
            Filter = "ROADS Map (*.roads)|*.roads|JSON (*.json)|*.json",
            DefaultExt = "roads"
        };
        if (dlg.ShowDialog() != DialogResult.OK)
        {
            _simLoop.Paused = wasPaused;
            return;
        }

        var result = MessageBox.Show("Include vehicles in save?",
            "Save Options", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
        bool includeVehicles = result == DialogResult.Yes;

        try
        {
            bool asJson = System.IO.Path.GetExtension(dlg.FileName)
                .Equals(".json", StringComparison.OrdinalIgnoreCase);
            if (asJson)
            {
                Persistence.MapJsonSerializer.Save(dlg.FileName, _roadGraph, _vehicles,
                    _camera, _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals,
                    _populationManager, includeVehicles);
            }
            else
            {
                Persistence.MapSerializer.Save(dlg.FileName, _roadGraph, _vehicles,
                    _camera, _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals,
                    _populationManager, includeVehicles);
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Save failed: {ex.Message}", "Error",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
        }

        _simLoop.Paused = wasPaused;
    }

    /// <summary>
    /// Prompts for a file path and loads a map. If the file contains vehicles,
    /// asks whether to load them.
    /// </summary>
    private void LoadMap()
    {
        using var dlg = new OpenFileDialog
        {
            Title = "Load Map",
            Filter = "ROADS Map (*.roads)|*.roads",
            DefaultExt = "roads"
        };
        if (dlg.ShowDialog() != DialogResult.OK) return;

        bool loadVehicles = false;

        try
        {
            // Peek at file to check for vehicle data before prompting
            using (var peek = File.OpenRead(dlg.FileName))
            using (var pr = new BinaryReader(peek))
            {
                pr.ReadBytes(4); // magic
                pr.ReadUInt16(); // version
                byte flags = pr.ReadByte();
                bool hasVehicles = (flags & 1) != 0;

                if (hasVehicles)
                {
                    var result = MessageBox.Show("This map contains vehicles. Load them?",
                        "Load Options", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
                    loadVehicles = result == DialogResult.Yes;
                }
            }

            Persistence.MapSerializer.Load(dlg.FileName, _roadGraph, _vehicles,
                _camera, _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals,
                _populationManager, loadVehicles);
            _simLoop.RebuildWorldCaches();

            // Start paused after loading
            _simLoop.Paused = true;

            // Reset editor state
            _editorState.SelectedEdge = -1;
            _editorState.SelectedNode = -1;
            _editorState.SelectedVehicle = -1;
            _editorState.LaneRestrictionMode = false;
            _editorState.LaneRestrictionEdge = -1;
            _editorState.RoadStartNode = null;
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Load failed: {ex.Message}", "Error",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    /// <summary>
    /// Renders the entire scene: background grid, roads with signals/signs, vehicles,
    /// spawn/destination points, control point handles (in Select mode), road tool preview,
    /// snap indicator, and the UI overlay (toolbar + status text + slider panel).
    /// </summary>
    private void OnPaintSurface(object? sender, SKCanvas canvas, SKImageInfo info)
    {
        _perfStopwatch.Restart();
        _sceneRenderer.Render(canvas, info, _camera, _roadGraph, _vehicles, _editorState,
            _stopLineCache, _intersectionArcs,
            _trafficSignals, _stopSigns, _yieldSigns, _simLoop,
            _spawner.SpawnNodeCount, _currentMousePos);
        _perfHud.RecordDrawTime(_perfStopwatch.Elapsed.TotalMilliseconds);
        _perfHud.Draw(canvas, _vehicles.Count, info.Width, info.Height);
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

            // Minimap click-to-jump: center the camera on the clicked world point and begin a
            // drag-scrub. Checked before tools so a click on the panel never selects or places
            // anything behind it; jumps only when the map has roads (otherwise the box is blank).
            if (_minimap.HitTest(e.X, e.Y))
            {
                if (_minimap.TryScreenToWorld(e.X, e.Y, out var minimapWorld))
                {
                    _camera.CenterOnWorld(minimapWorld.X, minimapWorld.Y);
                    _draggingMinimap = true;
                }
                return;
            }

            // Check POI submenu (only visible when Destination tool active)
            if (_editorState.ActiveTool == EditorTool.Destination)
            {
                var hitPOI = _uiRenderer.HitTestPOI(e.X, e.Y);
                if (hitPOI.HasValue)
                {
                    _editorState.SelectedPOIType = hitPOI.Value;
                    return;
                }
            }

            // Check toolbar
            var hitTool = _uiRenderer.HitTest(e.X, e.Y);
            if (hitTool.HasValue)
            {
                _editorState.ResetToolState();
                _editorState.ActiveTool = hitTool.Value;
                return;
            }

            // Check action buttons
            var hitAction = _uiRenderer.HitTestAction(e.X, e.Y);
            if (hitAction.HasValue)
            {
                switch (hitAction.Value)
                {
                    case Rendering.UIAction.New: NewMap(); break;
                    case Rendering.UIAction.Save: SaveMap(); break;
                    case Rendering.UIAction.Load: LoadMap(); break;
                    case Rendering.UIAction.Pause:
                        _simLoop.Paused = !_simLoop.Paused;
                        break;
                    case Rendering.UIAction.SpeedDown:
                        if (_simLoop.TimeScaleExponent > 0) _simLoop.TimeScaleExponent--;
                        break;
                    case Rendering.UIAction.SpeedUp:
                        _simLoop.TimeScaleExponent++;
                        _simLoop.Paused = false;
                        break;
                }
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

                    // Find nearest node (hit radius matches the visual highlight)
                    int nearNode = _roadGraph.FindNearestNode(worldVec, 5f);

                    // Only hit-test control points whose handles are currently visible:
                    // on the selected edge, or on edges adjacent to the selected node.
                    int cpEdgeIdx = -1, cpIdx = -1;
                    if (_editorState.SelectedEdge >= 0 || _editorState.SelectedNode >= 0)
                    {
                        var (ce, ci) = _roadGraph.FindNearestControlPoint(worldVec, EditorState.SnapDistance);
                        if (ce >= 0)
                        {
                            bool accept = false;
                            var cePrimary = ce;
                            int ceReverse = _roadGraph.FindReverseEdge(ce);
                            if (ceReverse >= 0 && ceReverse < ce) cePrimary = ceReverse;

                            // Accept if CP belongs to the selected edge
                            if (_editorState.SelectedEdge >= 0)
                            {
                                int selPrimary = _editorState.SelectedEdge;
                                int selReverse = _roadGraph.FindReverseEdge(selPrimary);
                                if (selReverse >= 0 && selReverse < selPrimary) selPrimary = selReverse;
                                if (cePrimary == selPrimary) accept = true;
                            }

                            // Accept if CP belongs to an edge adjacent to the selected node
                            if (!accept && _editorState.SelectedNode >= 0)
                            {
                                var edge = _roadGraph.Edges[ce];
                                if (edge.FromNode == _editorState.SelectedNode
                                    || edge.ToNode == _editorState.SelectedNode)
                                    accept = true;
                            }

                            if (accept)
                            {
                                cpEdgeIdx = ce;
                                cpIdx = ci;
                            }
                        }
                    }

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
                        _dragStartScreenPos = e.Location;
                        _dragOffset = _roadGraph.Nodes[nearNode].Position - worldVec;
                        _dragActive = false;
                        _canvas.Cursor = Cursors.Hand;
                    }
                    else if (cpEdgeIdx >= 0)
                    {
                        // Check if the click is actually closer to an edge curve than to the CP handle.
                        // If so, prefer edge selection (the user clicked the road, not the handle).
                        var (nearEdgeForCp, nearEdgeT) = _edgeSpatialGrid.FindNearestEdgeWithT(_roadGraph, worldVec, EditorState.SnapDistance);
                        float edgeDist = float.MaxValue;
                        if (nearEdgeForCp >= 0)
                        {
                            var edgePt = _roadGraph.EvaluateBezier(nearEdgeForCp, nearEdgeT);
                            edgeDist = Vector2.Distance(worldVec, edgePt);
                        }

                        if (nearEdgeForCp >= 0 && edgeDist < cpDist)
                        {
                            // Click is closer to the edge curve — select the edge
                            _editorState.SelectedNode = -1;
                            _editorState.SelectedEdge = nearEdgeForCp;
                        }
                        else
                        {
                            // Click is closer to the CP handle — start CP drag
                            var cpEdge = _roadGraph.Edges[cpEdgeIdx];
                            bool adjacentToSelectedNode = _editorState.SelectedNode >= 0
                                && (cpEdge.FromNode == _editorState.SelectedNode
                                    || cpEdge.ToNode == _editorState.SelectedNode);
                            if (!adjacentToSelectedNode)
                            {
                                _editorState.SelectedNode = -1;
                                _editorState.SelectedEdge = cpEdgeIdx;
                            }
                            _editorState.DragEdgeIndex = cpEdgeIdx;
                            _editorState.DragControlPointIndex = cpIdx;
                            var cpPos = cpIdx == 1
                                ? cpEdge.ControlPoint1
                                : cpEdge.ControlPoint2;
                            _dragStartScreenPos = e.Location;
                            _dragOffset = cpPos - worldVec;
                            _dragActive = false;
                            _canvas.Cursor = Cursors.Hand;
                        }
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
                    break;
                case EditorTool.Delete:
                    _deleteTool.OnClick(worldVec, _roadGraph, _edgeSpatialGrid, _populationManager);
                    break;
                case EditorTool.SpawnPoint:
                    _spawnPointTool.OnClick(worldVec, _roadGraph);
                    break;
                case EditorTool.Destination:
                    // Legacy fallback: if over an existing eligible node, flag/toggle it.
                    if (!_destinationTool.OnClick(worldVec, _roadGraph, _editorState.SelectedPOIType))
                    {
                        // Placement: use the ghost computed on the last mouse-move (recompute foot at click
                        // for safety, since the graph may have changed). Requires a nearby road.
                        _edgeSpatialGrid.RebuildIfNeeded(_roadGraph);
                        var (nearEdge, nearT) = _edgeSpatialGrid.FindNearestEdgeWithT(
                            _roadGraph, worldVec, DestPlacementSearchRadius);
                        if (nearEdge >= 0)
                        {
                            _destinationTool.PlaceAndConnect(worldVec, nearEdge, nearT,
                                _roadGraph, _editorState.SelectedPOIType, _stopSigns);
                        }
                    }
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
                    RemoveNearestDestination(rWorldVec);
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
            _draggingMinimap = false;
            if (_editorState.IsDraggingNode)
            {
                // Split edges at any crossings created by the drag
                if (_editorState.DragCrossingPreviews.Count > 0)
                    _roadGraph.SplitNodeEdgeCrossings(_editorState.DragNodeIndex);

                _editorState.DragCrossingPreviews.Clear();
                _editorState.DragNodeIndex = -1;
                _canvas.Cursor = Cursors.Default;
            }
            else if (_editorState.IsDraggingControlPoint)
            {
                _editorState.DragEdgeIndex = -1;
                _editorState.DragControlPointIndex = -1;
                _canvas.Cursor = Cursors.Default;
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

        // Clear the destination-placement ghost up front; the Destination hover case below
        // recomputes it when applicable. This guarantees no stale ghost renders down any
        // early-return path (UI, pan, slider, node/control-point drag) — e.g. during a
        // middle-button pan while the Destination tool is active.
        _editorState.GhostDestPos = null;
        _editorState.GhostFootPos = null;
        _editorState.GhostEdge = -1;

        // Update button hover state; suppress map hovers when over UI
        if (_uiRenderer.UpdateHover(e.X, e.Y))
        {
            _editorState.HoveredNode = -1;
            _editorState.HoveredEdge = -1;
            _editorState.HoveredVehicle = -1;
            return;
        }

        if (_sliderPanel.OnMouseMove(e.X, e.Y))
            return;

        // Drag across the minimap to scrub the camera continuously.
        if (_draggingMinimap && _minimap.TryScreenToWorld(e.X, e.Y, out var minimapWorld))
        {
            _camera.CenterOnWorld(minimapWorld.X, minimapWorld.Y);
            return;
        }

        if (_isPanning)
        {
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _camera.Pan(dx, dy);
            _lastMousePos = e.Location;
        }
        else if (_editorState.IsDraggingNode)
        {
            if (!_dragActive)
            {
                int dx = e.X - _dragStartScreenPos.X;
                int dy = e.Y - _dragStartScreenPos.Y;
                if (dx * dx + dy * dy < DragDeadZone * DragDeadZone) return;
                _dragActive = true;
            }
            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            _roadGraph.MoveNode(_editorState.DragNodeIndex,
                new Vector2(worldPos.X, worldPos.Y) + _dragOffset);

            // Detect crossing previews for edges connected to the dragged node
            _editorState.DragCrossingPreviews.Clear();
            var crossings = _roadGraph.FindNodeEdgeCrossings(_editorState.DragNodeIndex);
            foreach (var (_, _, _, _, pos) in crossings)
                _editorState.DragCrossingPreviews.Add(pos);
        }
        else if (_editorState.IsDraggingControlPoint)
        {
            if (!_dragActive)
            {
                int dx = e.X - _dragStartScreenPos.X;
                int dy = e.Y - _dragStartScreenPos.Y;
                if (dx * dx + dy * dy < DragDeadZone * DragDeadZone) return;
                _dragActive = true;
            }
            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            _roadGraph.SetControlPoint(
                _editorState.DragEdgeIndex,
                _editorState.DragControlPointIndex,
                new Vector2(worldPos.X, worldPos.Y) + _dragOffset);
        }
        else
        {
            var worldPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
            var worldVec = new Vector2(worldPos.X, worldPos.Y);

            switch (_editorState.ActiveTool)
            {
                case EditorTool.Select:
                {
                    // Check for vehicle under cursor first
                    var vehHits = new List<int>();
                    _vehicleGrid.QueryFiltered(worldVec.X, worldVec.Y, 5f,
                        _vehicles.PosX, _vehicles.PosY, vehHits);
                    int closestVeh = -1;
                    float closestVehDist = float.MaxValue;
                    foreach (int vi in vehHits)
                    {
                        if (_vehicles.State[vi] != VehicleState.Driving) continue;
                        float d = Vector2.Distance(worldVec, new Vector2(_vehicles.PosX[vi], _vehicles.PosY[vi]));
                        if (d < closestVehDist) { closestVehDist = d; closestVeh = vi; }
                    }

                    if (closestVeh >= 0 && closestVehDist < 5f)
                    {
                        _editorState.HoveredVehicle = closestVeh;
                        _editorState.HoveredNode = -1;
                        _editorState.HoveredEdge = -1;
                    }
                    else
                    {
                        _editorState.HoveredVehicle = -1;
                        int nearNode = _roadGraph.FindNearestNode(worldVec, 5f);
                        if (nearNode >= 0)
                        {
                            _editorState.HoveredNode = nearNode;
                            _editorState.HoveredEdge = -1;
                        }
                        else
                        {
                            _editorState.HoveredNode = -1;
                            _editorState.HoveredEdge = _edgeSpatialGrid.FindNearestEdge(_roadGraph, worldVec, EditorState.SnapDistance);
                        }
                    }
                    break;
                }
                case EditorTool.Delete:
                {
                    int nearNode = _roadGraph.FindNearestNode(worldVec, 5f);
                    if (nearNode >= 0)
                    {
                        _editorState.HoveredNode = nearNode;
                        _editorState.HoveredEdge = -1;
                    }
                    else
                    {
                        _editorState.HoveredNode = -1;
                        _editorState.HoveredEdge = _edgeSpatialGrid.FindNearestEdge(_roadGraph, worldVec, EditorState.SnapDistance);
                    }
                    break;
                }
                case EditorTool.SpawnPoint:
                case EditorTool.Signal:
                {
                    _editorState.HoveredEdge = -1;
                    _editorState.HoveredNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    break;
                }
                case EditorTool.Destination:
                {
                    _editorState.HoveredEdge = -1;
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);

                    // Snap to (and later flag) an existing node only when it is eligible AND not
                    // already carrying a marker — never grab a spawn/destination node. Otherwise
                    // fall through to placement (the ghost), which drops a NEW destination.
                    // (Ghost fields were cleared at the top of this handler, so the snap and
                    // no-nearby-road cases simply leave them null.)
                    bool snapToNode = nearNode >= 0 && _roadGraph.CanPlaceMarker(nearNode)
                        && (_roadGraph.Nodes[nearNode].Flags & (NodeFlags.Spawn | NodeFlags.Destination)) == 0;

                    if (snapToNode)
                    {
                        _editorState.HoveredNode = nearNode; // legacy flag-existing path; no ghost
                    }
                    else
                    {
                        _editorState.HoveredNode = -1;
                        _edgeSpatialGrid.RebuildIfNeeded(_roadGraph);
                        var (nearEdge, nearT) = _edgeSpatialGrid.FindNearestEdgeWithT(
                            _roadGraph, worldVec, DestPlacementSearchRadius);
                        if (nearEdge >= 0)
                        {
                            // ComputeFootPoint treats a flagged endpoint as non-existent — the foot
                            // lands on the road just shy of it (a split), never on the marked node.
                            _editorState.GhostDestPos = worldVec;
                            _editorState.GhostFootPos = DestinationTool.ComputeFootPoint(_roadGraph, nearEdge, nearT);
                            _editorState.GhostEdge = nearEdge;
                            _editorState.GhostT = nearT;
                        }
                    }
                    break;
                }
                default:
                    _editorState.HoveredNode = -1;
                    _editorState.HoveredEdge = -1;
                    break;
            }
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

    /// <summary>
    /// Right-click removal for destination nodes: clears both the Destination flag and POI type.
    /// </summary>
    private void RemoveNearestDestination(Vector2 worldPos)
    {
        int bestNode = -1;
        float bestDist = EditorState.SnapDistance * EditorState.SnapDistance;
        for (int i = 0; i < _roadGraph.Nodes.Count; i++)
        {
            var node = _roadGraph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if (!node.Flags.HasFlag(NodeFlags.Destination)) continue;
            float d = Vector2.DistanceSquared(worldPos, node.Position);
            if (d < bestDist)
            {
                bestDist = d;
                bestNode = i;
            }
        }
        if (bestNode >= 0)
        {
            _roadGraph.SetNodeFlags(bestNode, _roadGraph.Nodes[bestNode].Flags & ~NodeFlags.Destination);
            _roadGraph.SetNodePOIType(bestNode, POIType.None);
        }
    }

    /// <summary>
    /// Replaces the current map with a 50×50 grid road network and bulk-spawns 10,000 vehicles
    /// for Phase 5 stress-testing. No confirmation dialog — the intent is immediate load testing.
    /// Camera is centered on the grid center and zoomed out to fit the full ~5 km extent.
    /// </summary>
    private void GenerateStressScene(float camZoom = 0.2f)
    {
        const int gridCols = 50, gridRows = 50;
        const float spacing = 100f;
        const int vehicleCount = 10000;

        _vehicles.ClearAll();
        var (nodes, edges) = Roads.App.World.GridNetworkGenerator.Generate(gridCols, gridRows, spacing);
        _roadGraph.LoadFromData(nodes, edges);

        // Reset per-map traffic-control overrides exactly as NewMap() does.
        _stopSigns.SetExemptEdges(new List<int>());
        _yieldSigns.SetExemptEdges(new List<int>());
        _trafficSignals.SetPhaseRotations(new List<(int, byte)>());

        _simLoop.RebuildWorldCaches();

        int spawned = _spawner.SpawnBulk(vehicleCount);

        // Center on the grid middle, zoomed out to fit the ~5 km grid. CenterX/CenterY are
        // screen-pixel pan offsets (not world coords), so set Zoom first then use CenterOnWorld.
        _camera.Zoom = camZoom;
        _camera.CenterOnWorld((gridCols - 1) * spacing / 2f, (gridRows - 1) * spacing / 2f);

        _simLoop.Clock.TimeOfDay = 12.0;
        _simLoop.Paused = false;
        _simLoop.TimeScaleExponent = 0;

        // Reset editor selection/tool exactly as NewMap() does.
        _editorState.SelectedEdge = -1;
        _editorState.SelectedNode = -1;
        _editorState.SelectedVehicle = -1;
        _editorState.LaneRestrictionMode = false;
        _editorState.LaneRestrictionEdge = -1;
        _editorState.RoadStartNode = null;
        _editorState.ActiveTool = Editor.EditorTool.Select;

        // Show FPS HUD immediately so the user sees the load metrics.
        _perfHud.Visible = true;

        System.Diagnostics.Debug.WriteLine($"[StressScene] grid {gridCols}x{gridRows}, spawned {spawned}/{vehicleCount} vehicles");
    }

    /// <summary>
    /// Captures a non-intrusive performance baseline snapshot to benchmark.log.
    /// Reads per-frame stats from <see cref="_perfHud"/> (which already drains the
    /// pathfind accumulators each frame) so this method does not reset any shared state.
    /// </summary>
    private void CaptureBaseline()
    {
        // Off-road correctness tripwire: count Driving vehicles more than ~2 lanes off their lane
        // center. ~0 normally; a spike flags steering/projection drift from an optimization.
        int offroad = 0;
        const float offRoadDistSq = 49f; // (~7 m)^2
        var distToRoad = _vehicles.DistToRoadSq;
        for (int i = 0; i < _vehicles.Count; i++)
            if (_vehicles.State[i] == Roads.App.Vehicles.VehicleState.Driving && distToRoad[i] > offRoadDistSq)
                offroad++;

        Roads.App.Rendering.BenchmarkCapture.Capture(
            _perfHud.AvgFps, _perfHud.AvgSimMs, _perfHud.AvgDrawMs,
            _perfHud.LastPathfindMs, _perfHud.LastPathfindCalls,
            _vehicles.Count, Roads.App.Vehicles.SteeringController.LastConflictCoOccupancy, offroad,
            _simLoop.LastTiming);
        System.Diagnostics.Debug.WriteLine($"[Baseline] captured: fps={_perfHud.AvgFps:F1}, vehicles={_vehicles.Count}");
    }

}
