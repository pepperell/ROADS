using System.Diagnostics;
using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Rendering;
using Roads.App.Vehicles;
using Roads.App.World;
using Ui = Roads.App.Rendering.Ui;

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
    private readonly NodeTool _nodeTool = new();
    private readonly DeleteTool _deleteTool = new();
    private readonly UpdateSegmentTool _updateSegmentTool = new();
    private readonly DestinationTool _destinationTool = new();
    private readonly SignalTool _signalTool = new();
    private readonly LaneRestrictionTool _laneRestrictionTool = new();
    private readonly WaterTool _waterTool = new();
    private readonly WaterLayer _waterLayer = new();
    private readonly SpatialGrid _vehicleGrid = new();
    private readonly StopLineCache _stopLineCache = new();
    private readonly EdgeSpatialGrid _edgeSpatialGrid = new();
    private readonly TrafficSignalSystem _trafficSignals = new();
    private readonly StopSignSystem _stopSigns = new();
    private readonly YieldSignSystem _yieldSigns = new();
    private readonly IntersectionArcCache _intersectionArcs = new();
    /// <summary>Root of the retained-mode screen-space UI (z-order, hover, mouse capture).
    /// Panels migrate into it step-by-step; legacy immediate-mode renderers are checked
    /// after it in the input handlers until they are retired.</summary>
    private readonly Ui.UiRoot _uiRoot = new();
    private readonly Ui.MinimapPanel _minimapPanel;
    private readonly Ui.StatisticsPanel _statisticsPanel;
    private readonly Ui.PerformanceHudPanel _hudPanel;
    private readonly Ui.LegendPanel _legendPanel = new();
    private readonly Ui.SettingsDialog _settingsDialog;

    /// <summary>Last-APPLIED application settings — the values the live systems currently
    /// run with, mirrored in settings.json. Mutated only by <see cref="ApplyStagedSettings"/>
    /// (Settings dialog Apply/OK) and <see cref="MutateSettings"/> (hotkey toggles), both of
    /// which re-push to the live systems and persist.</summary>
    private Core.AppSettings _settings;

    /// <summary>Pause state captured when the Settings dialog opened, restored on close —
    /// so closing Settings never un-pauses a sim the user had manually paused.</summary>
    private bool _settingsWasPaused;
    /// <summary>Frame-timing telemetry; <see cref="PerfTelemetry.Sample"/> runs once per
    /// paint regardless of HUD visibility (single consumer of the pathfind accumulators).</summary>
    private readonly PerfTelemetry _perfTelemetry = new();
    private readonly Stopwatch _perfStopwatch = new();
    private readonly Stopwatch _autoSaveClock = Stopwatch.StartNew();
    private readonly POIRegistry _poiRegistry = new();
    private readonly VehicleSpawner _spawner;
    private readonly PopulationManager _populationManager;
    private readonly GraphChangeHandler _graphChangeHandler;
    private readonly SimulationLoop _simLoop;
    private readonly SceneRenderer _sceneRenderer;
    private readonly Persistence.AutoSaveManager _autoSave;
    private readonly Audio.AudioEngine _audioEngine;

    private Point _lastMousePos;
    private Point _currentMousePos;
    private bool _isPanning;

    /// <summary>Screen position where a drag-candidate click occurred.</summary>
    private Point _dragStartScreenPos;
    // (Minimap scrub-drag state now lives in UiRoot's mouse capture.)
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
    /// <summary>SIM substeps stopped before a vehicle is treated as a deadlock candidate by the cluster
    /// dump (press <c>D</c> with nothing selected). Counted in sim substeps (30 Hz), NOT UI frames, so the
    /// duration is correct at any fast-forward speed — 120 ≈ 4 s of sim time, long past normal stop-and-go.</summary>
    private const int StuckTickThreshold = 120;

    public MainForm(int autoBenchFrames = 0)
    {
        _autoBenchFrames = autoBenchFrames;
        Text = "ROADS - Traffic Simulation";
        Width = 1280;
        Height = 720;
        DoubleBuffered = true;
        StartPosition = FormStartPosition.CenterScreen;

        // Persisted user settings first: the population cap feeds the manager's ctor, and
        // ApplySettings below (once every target exists) pushes the rest before frame one.
        _settings = Persistence.SettingsStore.Load();

        // The app always opens paused with the Select tool active (New/Load match this).
        _editorState.ActiveTool = EditorTool.Select;
        _spawner = new VehicleSpawner(_roadGraph, _vehicles, _vehicleGrid);
        _populationManager = new PopulationManager(_roadGraph, _vehicles, _vehicleGrid, _poiRegistry, _settings.MaxVehicles);
        _graphChangeHandler = new GraphChangeHandler(_roadGraph, _editorState, _vehicles, _edgeSpatialGrid, _spawner);
        _simLoop = new SimulationLoop(_roadGraph, _vehicles, _vehicleGrid, _stopLineCache, _intersectionArcs, _edgeSpatialGrid, _trafficSignals, _stopSigns, _yieldSigns, _spawner, _populationManager, _editorState, _graphChangeHandler);
        _simLoop.Paused = true;

        // Procedural sound engine (self-disables when no audio device exists).
        _audioEngine = new Audio.AudioEngine(_vehicles, _roadGraph, _trafficSignals, _simLoop, _camera);

        // Retained-mode UI tree (bottom→top add order = draw order; input hits topmost first).
        _uiRoot.Add(new Ui.MenuBar(_editorState, OnUiAction));
        _uiRoot.Add(new Ui.PoiSubmenu(_editorState));
        _uiRoot.Add(new Ui.RoadSubmenu(_editorState));
        _uiRoot.Add(new Ui.SignalSubmenu(_editorState));
        _uiRoot.Add(new Ui.WaterSubmenu(_editorState));
        _uiRoot.Add(_legendPanel);
        _uiRoot.Add(new Ui.ClockPanel(_simLoop, OnUiAction));

        // Bottom-left stack (second column, right of the shortcut legend): HUD at the very
        // bottom, statistics above it, then vehicle info, selection info on top — positioned
        // from measured heights so no combination overlaps. The HUD panel is laid out here
        // but drawn by OnPaintSurface (ExternallyDrawn) so it stays above the minimap and
        // outside the measured draw window.
        _statisticsPanel = new Ui.StatisticsPanel(_vehicles, _simLoop, _roadGraph);
        _hudPanel = new Ui.PerformanceHudPanel(_perfTelemetry, _vehicles);
        var bottomLeftStack = new Ui.BottomLeftStack();
        bottomLeftStack.Add(_hudPanel);
        bottomLeftStack.Add(_statisticsPanel);
        bottomLeftStack.Add(new Ui.VehicleInfoPanel(_vehicles, _roadGraph, _editorState, _intersectionArcs));
        bottomLeftStack.Add(new Ui.SelectionInfoPanel(_roadGraph, _editorState));
        _uiRoot.Add(bottomLeftStack);

        _minimapPanel = new Ui.MinimapPanel(_camera, _roadGraph, _waterLayer);
        _uiRoot.Add(_minimapPanel);

        // Settings dialog last: topmost hit-testing (its scrim must swallow every click).
        // It is ExternallyDrawn — painted by OnPaintSurface after the perf HUD, which is
        // itself drawn after the whole UI pass and would otherwise cover the dialog.
        _settingsDialog = new Ui.SettingsDialog(() => _settings, ApplyStagedSettings, OnSettingsClosed);
        _uiRoot.Add(_settingsDialog);

        _sceneRenderer = new SceneRenderer(_roadRenderer, _vehicleRenderer, _laneRestrictionTool, _uiRoot, _waterLayer);
        // AutoSaveManager shares the same object references used by SaveMap() so the
        // backup format is identical to a manual save (minus vehicles, which are transient).
        // Triggered from the render-timer tick so it runs on the UI thread with no locking.
        _autoSave = new Persistence.AutoSaveManager(_roadGraph, _vehicles, _camera,
            _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals, _populationManager, _waterLayer);

        // Push the loaded settings into every live system before the first frame (all
        // targets — panels, renderer, autosave, sim — exist by this point).
        ApplySettings();

        // A default.roads beside the exe's working directory auto-loads at startup
        // (vehicles included, no prompt) — "resume my world" for regular players.
        TryLoadDefaultMap();

        // Centralized vehicle-removal fixup: editor-held vehicle indices follow
        // swap-and-pop moves and drop on bulk clears (see VehicleStore.VehicleRemoving).
        // The vehicle spatial grid holds indices too — it self-heals on the same events
        // so editor-time hit-tests between grid rebuilds never see stale indices.
        _vehicles.VehicleRemoving += OnVehicleRemoving;
        _vehicles.VehiclesCleared += OnVehiclesCleared;
        _vehicles.VehicleRemoving += _vehicleGrid.OnEntityRemoving;
        _vehicles.VehiclesCleared += _vehicleGrid.Clear;
        _vehicles.VehicleRemoving += _audioEngine.OnVehicleRemoving;
        _vehicles.VehiclesCleared += _audioEngine.OnVehiclesCleared;
        SteeringController.BreakerFreed += _audioEngine.OnBreakerFreed;

        // Migrate stop/yield exemptions across edge splits (they are keyed by edge index, which
        // SplitEdge changes) so splitting an exempt approach doesn't silently re-stop the road.
        _roadGraph.EdgeSplit += _stopSigns.OnEdgeSplit;
        _roadGraph.EdgeSplit += _yieldSigns.OnEdgeSplit;

        FormClosed += (_, _) =>
        {
            SteeringController.Shutdown();
            _audioEngine.Dispose();
        };

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
        _perfTelemetry.RecordSimTime(_perfStopwatch.Elapsed.TotalMilliseconds);

        // Drive the sound engine from the post-tick state (positions/clock current).
        _audioEngine.Update(_canvas.Width, _canvas.Height);

        // Accumulate wall time and trigger a backup when the interval elapses.
        // The auto-save clock is independent of simulation time and pause state
        // so that backups run on a predictable wall-clock cadence.
        double tickWall = _autoSaveClock.Elapsed.TotalSeconds;
        _autoSaveClock.Restart();
        _autoSave.MaybeSave(tickWall);

        UpdateCursor();
        _canvas.Invalidate();

        if (_autoBenchFrames > 0)
            AutoBenchStep();

        UpdateStuckTracking();
    }

    /// <summary>
    /// Applies the active tool's mouse cursor once per tick (see
    /// <see cref="Ui.ToolCursors.ForTool"/> — currently the finger pointer everywhere).
    /// Per-frame (rather than per-tool-switch call sites) so keyboard and menu tool
    /// changes are covered without hooking every mutation of ActiveTool; the
    /// assignment only fires on change.
    /// </summary>
    private void UpdateCursor()
    {
        var desired = Ui.ToolCursors.ForTool(_editorState.ActiveTool);
        if (!ReferenceEquals(_canvas.Cursor, desired))
            _canvas.Cursor = desired;
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

        // Advance by the SIM substeps this Tick covered (not +1 per UI frame), so stuck-time reflects
        // elapsed simulation time and stays correct when fast-forwarded — at high time-scale one UI frame
        // is many sim substeps, and a per-frame counter would wildly under-count a jam.
        int simTicks = _simLoop.LastTickSubsteps;
        for (int v = 0; v < _vehicles.Count; v++)
        {
            if (_vehicles.State[v] == VehicleState.Driving && _vehicles.Speed[v] < StuckSpeedThreshold)
                _stuckTicks[v] += simTicks;
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
            // Off-lane sweep: a displaced vehicle orbiting off-road at speed never
            // registers as stopped, so it would be invisible to the stuck filter above.
            // Sweep in any Driving edge-vehicle > ~7 m from its lane center as well
            // (arc vehicles excluded: DistToRoadSq is held at 0 on arcs).
            const float offLaneDistSq = 49f;
            for (int v = 0; v < _vehicles.Count; v++)
                if (_vehicles.State[v] == Roads.App.Vehicles.VehicleState.Driving
                    && _vehicles.CurrentArc[v] < 0
                    && _vehicles.DistToRoadSq[v] > offLaneDistSq && dumped.Add(v))
                    DumpVehicleDiag(v);
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
        sb.AppendLine($"  Stuck for: {stuckTicks} sim-ticks (~{stuckTicks / 30f:F1}s of sim time continuously stopped)");

        int leader = NearestVehicleAhead(i, out float gap, out float lat);
        if (leader >= 0)
            sb.AppendLine($"  Nearest ahead: vehicle {leader} gap={gap:F2}m lateral={lat:F2}m " +
                $"speed={_vehicles.Speed[leader]:F2} edge={_vehicles.CurrentEdge[leader]} arc={_vehicles.CurrentArc[leader]} prog={_vehicles.EdgeProgress[leader]:F3}");
        else
            sb.AppendLine("  Nearest ahead: none within 30m (so it is held by a signal/stop, not a car)");

        // Stop-sign first-come-first-served detail (explains a never-granted green). Dumped even when
        // CanQuery is false (e.g. just after a pause rebuilt caches) — the arrays still hold the last
        // Update's state, which is exactly what we want to inspect; flagged STALE so it's not misread.
        if (edgeIdx >= 0 && edgeIdx < _roadGraph.Edges.Count && _roadGraph.Edges[edgeIdx].FromNode >= 0)
        {
            string stale = _stopSigns.CanQuery(_roadGraph) ? "" : " [STALE: not updated since last cache rebuild]";
            sb.AppendLine($"  StopSign FCFS: {_stopSigns.DescribeStopState(_roadGraph, edgeIdx)}{stale}");
            // Full per-incoming-edge FCFS state of the approached node — shows which approach (if any,
            // incl. a stale phantom) is winning the queue when a clear waiter is never served.
            int toNode = _roadGraph.Edges[edgeIdx].ToNode;
            if (toNode >= 0 && _stopSigns.IsStopSign(toNode))
                sb.AppendLine(_stopSigns.DescribeNodeFull(_roadGraph, toNode));
        }

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
        if (best >= 0)
            gap = bestFwd - Vehicles.VehicleTypeDimensions.GetHalfLength(_vehicles.PreferredVehicle[v])
                - Vehicles.VehicleTypeDimensions.GetHalfLength(_vehicles.PreferredVehicle[best]);
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
    /// [/] to adjust speed limit, P to toggle the performance HUD, M to toggle the minimap,
    /// N to toggle the statistics panel, Space to pause/unpause, comma/period to
    /// decrease/increase simulation speed (1x-64x). While the Settings dialog is open,
    /// Escape cancels it and every other key is swallowed (modal).
    /// </summary>
    private void OnCanvasKeyDown(object? sender, KeyEventArgs e)
    {
        // Modal Settings dialog: Escape cancels it; every other shortcut is swallowed
        // while it is open. Must run before ANY other key handling (including the
        // universal Escape -> CancelActiveTool below).
        if (_settingsDialog.Visible)
        {
            if (e.KeyCode == Keys.Escape)
                _settingsDialog.Cancel();
            e.Handled = true;
            return;
        }

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

        // Escape = universal cancel (same as right-click): aborts the in-progress tool
        // operation (road chain, lane-restrict mode, selection) one step per press;
        // with nothing left to cancel it switches back to the Select tool.
        if (e.KeyCode == Keys.Escape)
        {
            CancelActiveTool();
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

        // Panel/overlay toggles route through MutateSettings so the quick keys, the
        // Settings dialog, and settings.json always agree.
        if (e.KeyCode == Keys.P)
        {
            MutateSettings(s => s.ShowPerformanceHud = !s.ShowPerformanceHud);
            e.Handled = true;
        }

        if (e.KeyCode == Keys.M)
        {
            MutateSettings(s => s.ShowMinimap = !s.ShowMinimap);
            e.Handled = true;
        }

        // N = toggle statistics panel (vehicle count, avg speed, congestion)
        if (e.KeyCode == Keys.N)
        {
            MutateSettings(s => s.ShowStatistics = !s.ShowStatistics);
            e.Handled = true;
        }

        // H = toggle the congestion heat-map overlay
        if (e.KeyCode == Keys.H)
        {
            MutateSettings(s => s.HeatMapEnabled = !s.HeatMapEnabled);
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

        // G = toggle arc conflict debug overlay + logging (turning the overlay ON also
        // forces logging on — never off — the historical pairing)
        if (e.KeyCode == Keys.G)
        {
            MutateSettings(s =>
            {
                s.ShowArcConflicts = !s.ShowArcConflicts;
                if (s.ShowArcConflicts) s.DebugLogging = true;
            });
            e.Handled = true;
        }

        // D = capture a live deadlock to diag_vehicle.log. With a vehicle selected, dumps it and walks
        // the blocking chain (the full cycle); with NOTHING selected, dumps every car stuck > ~6s (the
        // whole wedged cluster) plus any car > ~7 m off its lane center (off-road spinners never
        // register as stopped). Press it the moment a deadlock is on screen — the arc/intersection
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

        // W: select the Water tool (mirrors the toolbar button's click body)
        if (e.KeyCode == Keys.W)
        {
            _editorState.ResetToolState();
            _editorState.ActiveTool = Editor.EditorTool.Water;
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
        _waterLayer.Clear();

        // Reset per-map traffic-control overrides (exemptions, phase rotations). They
        // survive graph edits by design, so replacing the whole map must clear them
        // explicitly — mirrors MapSerializer.Load's clear-then-set semantics; otherwise
        // the old map's overrides silently apply to reused node/edge indices.
        _stopSigns.SetExemptEdges(new List<int>());
        _yieldSigns.SetExemptEdges(new List<int>());
        _trafficSignals.SetPhaseRotations(new List<(int, byte)>());

        _simLoop.RebuildWorldCaches();
        _sceneRenderer.OnMapReplaced();

        _camera.CenterX = 0;
        _camera.CenterY = 0;
        _camera.Zoom = 5.0f;
        _simLoop.Clock.TimeOfDay = 8.0;
        _simLoop.Paused = true; // new maps start paused, matching app open and Load
        _simLoop.TimeScaleExponent = 0;

        _editorState.ResetToolState();
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
                    _populationManager, _waterLayer, includeVehicles);
            }
            else
            {
                Persistence.MapSerializer.Save(dlg.FileName, _roadGraph, _vehicles,
                    _camera, _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals,
                    _populationManager, _waterLayer, includeVehicles);
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

        try
        {
            bool loadVehicles = MapHasVehicles(dlg.FileName)
                && MessageBox.Show("This map contains vehicles. Load them?",
                    "Load Options", MessageBoxButtons.YesNo, MessageBoxIcon.Question) == DialogResult.Yes;
            LoadMapFromFile(dlg.FileName, loadVehicles);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Load failed: {ex.Message}", "Error",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
    }

    /// <summary>Name of the map auto-loaded at startup when present in the working
    /// directory (beside settings.json and backups/).</summary>
    private const string DefaultMapFile = "default.roads";

    /// <summary>Startup auto-load of <see cref="DefaultMapFile"/>: silent when the file
    /// is absent, vehicles restored without prompting when present (the file is the
    /// user's "resume my world" save). A corrupt file warns and falls back to the
    /// empty map — startup must never be blocked by a bad default.</summary>
    private void TryLoadDefaultMap()
    {
        if (!File.Exists(DefaultMapFile)) return;
        try
        {
            LoadMapFromFile(DefaultMapFile, MapHasVehicles(DefaultMapFile));
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Could not load {DefaultMapFile}: {ex.Message}\n\nStarting with an empty map.",
                "Default Map", MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }
    }

    /// <summary>Header peek: whether the map file was saved with vehicle data.</summary>
    private static bool MapHasVehicles(string path)
    {
        using var peek = File.OpenRead(path);
        using var reader = new BinaryReader(peek);
        reader.ReadBytes(4); // magic
        reader.ReadUInt16(); // version
        return (reader.ReadByte() & 1) != 0;
    }

    /// <summary>Loads a map file into the live world and resets sim/editor state —
    /// the shared core of the Load dialog and the default.roads startup path. The app
    /// is left paused with the Select tool active (the New/Load convention).</summary>
    private void LoadMapFromFile(string path, bool loadVehicles)
    {
        Persistence.MapSerializer.Load(path, _roadGraph, _vehicles,
            _camera, _simLoop.Clock, _stopSigns, _yieldSigns, _trafficSignals,
            _populationManager, _waterLayer, loadVehicles);
        _simLoop.RebuildWorldCaches();
        _sceneRenderer.OnMapReplaced();
        _simLoop.Paused = true;
        _editorState.ResetToolState();
        _editorState.ActiveTool = Editor.EditorTool.Select;
    }

    /// <summary>
    /// Renders the entire scene: background grid, roads with signals/signs, vehicles,
    /// destination points, control point handles (in Select mode), road tool preview,
    /// snap indicator, and the retained-mode UI overlay.
    /// </summary>
    private void OnPaintSurface(object? sender, SKCanvas canvas, SKImageInfo info)
    {
        _perfStopwatch.Restart();
        _sceneRenderer.Render(canvas, info, _camera, _roadGraph, _vehicles, _editorState,
            _stopLineCache, _intersectionArcs,
            _trafficSignals, _stopSigns, _yieldSigns, _simLoop,
            _currentMousePos);
        _perfTelemetry.RecordDrawTime(_perfStopwatch.Elapsed.TotalMilliseconds);
        // Sample runs unconditionally (pathfind-accumulator drain + benchmark publication);
        // the HUD panel is drawn here — outside the measured draw window and above every
        // other overlay — and skips itself when hidden.
        _perfTelemetry.Sample();
        _hudPanel.Draw(canvas);
        // The modal Settings dialog is ExternallyDrawn and painted after the HUD so it
        // sits above every overlay (no-op while hidden).
        _settingsDialog.Draw(canvas);
    }

    /// <summary>
    /// Handles UI action buttons: New/Save/Load from the menu bar, Pause/speed from the
    /// clock panel's transport row. Tool buttons set EditorState directly inside
    /// <see cref="Ui.MenuBar"/>; only actions that need MainForm's dialogs and sim-loop
    /// access route through here.
    /// </summary>
    private void OnUiAction(Ui.UIAction action)
    {
        switch (action)
        {
            case Ui.UIAction.New: NewMap(); break;
            case Ui.UIAction.Save: SaveMap(); break;
            case Ui.UIAction.Load: LoadMap(); break;
            case Ui.UIAction.Settings: OpenSettings(); break;
            case Ui.UIAction.Pause:
                _simLoop.Paused = !_simLoop.Paused;
                break;
            case Ui.UIAction.SpeedDown:
                if (_simLoop.TimeScaleExponent > 0) _simLoop.TimeScaleExponent--;
                break;
            case Ui.UIAction.SpeedUp:
                _simLoop.TimeScaleExponent++;
                _simLoop.Paused = false;
                break;
        }
    }

    /// <summary>
    /// Pushes every value in <see cref="_settings"/> into the live systems. The ONLY
    /// place settings reach those systems, so applied state and settings.json can never
    /// disagree. Idempotent; called at startup and by every settings mutation.
    /// </summary>
    private void ApplySettings()
    {
        _sceneRenderer.GridEnabled = _settings.ShowGrid;
        _sceneRenderer.HeatMapEnabled = _settings.HeatMapEnabled;
        _hudPanel.Visible = _settings.ShowPerformanceHud;
        _minimapPanel.Visible = _settings.ShowMinimap;
        _statisticsPanel.Visible = _settings.ShowStatistics;
        _legendPanel.Visible = _settings.ShowLegend;

        _populationManager.MaxActiveVehicles = _settings.MaxVehicles;
        _simLoop.Clock.GameSecondsPerRealSecond = _settings.GameSecondsPerRealSecond;
        _autoSave.IntervalSeconds = _settings.AutosaveIntervalSeconds;
        _autoSave.MaxBackups = _settings.AutosaveMaxBackups;

        SteeringController.Kp = _settings.Kp;
        SteeringController.Kd = _settings.Kd;
        SteeringController.MaxSteer = _settings.MaxSteer;
        SteeringController.TargetSpeed = _settings.TargetSpeed;
        SteeringController.LookaheadBase = _settings.LookaheadBase;
        SteeringController.LookaheadPerSpeed = _settings.LookaheadPerSpeed;
        SteeringController.Klat = _settings.Klat;

        VehicleRenderer.ShowArcConflicts = _settings.ShowArcConflicts;
        SteeringController.DebugLoggingEnabled = _settings.DebugLogging;

        _audioEngine.ApplySettings(_settings);
    }

    /// <summary>Settings-dialog Apply/OK path: adopts the staged record as applied,
    /// pushes it live, and persists it.</summary>
    private void ApplyStagedSettings(Core.AppSettings staged)
    {
        _settings = staged;
        ApplySettings();
        Persistence.SettingsStore.Save(_settings);
    }

    /// <summary>Hotkey path (H/M/N/P/G): mutates the applied settings directly, pushes
    /// live, and persists — so quick toggles stay in sync with the dialog and the file.</summary>
    private void MutateSettings(Action<Core.AppSettings> mutate)
    {
        mutate(_settings);
        ApplySettings();
        Persistence.SettingsStore.Save(_settings);
    }

    /// <summary>Opens the Settings dialog: pauses the simulation for the dialog's
    /// lifetime, remembering the prior state (the SaveMap idiom) so closing restores
    /// a user-chosen pause rather than force-resuming.</summary>
    private void OpenSettings()
    {
        _settingsWasPaused = _simLoop.Paused;
        _simLoop.Paused = true;
        _settingsDialog.Open();
    }

    /// <summary>Dialog-closed callback (OK, Cancel, or Escape): restores the pause state
    /// captured by <see cref="OpenSettings"/>.</summary>
    private void OnSettingsClosed()
    {
        _simLoop.Paused = _settingsWasPaused;
    }

    /// <summary>
    /// Handles mouse-down: middle button starts panning, left button dispatches to the
    /// retained-mode UI first, then the active editor tool (one case per
    /// <see cref="EditorTool"/> value). Right button is the universal cancel (see
    /// <see cref="CancelActiveTool"/>).
    /// </summary>
    private void OnCanvasMouseDown(object? sender, MouseEventArgs e)
    {
        _canvas.Focus();

        // While the Settings dialog is open only LEFT clicks proceed (the dialog's scrim
        // consumes them via UiRoot) — middle-pan and right-click cancel bypass UiRoot
        // entirely and must be gated here.
        if (_settingsDialog.Visible && e.Button != MouseButtons.Left)
            return;

        if (e.Button == MouseButtons.Middle)
        {
            _isPanning = true;
            _lastMousePos = e.Location;
            _canvas.Cursor = Cursors.SizeAll;
            return;
        }

        if (e.Button == MouseButtons.Left)
        {
            // Retained-mode UI gets first claim on left clicks: the topmost panel under the
            // cursor consumes the down (background clicks included) and takes mouse capture
            // for drag-style panels (minimap scrub, slider thumbs).
            if (_uiRoot.OnMouseDown(e.X, e.Y))
                return;

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

                    // Visible Bézier handles get FIRST claim on the click — they draw
                    // topmost, so they hit-test topmost. Without this the click fell
                    // through: a control point usually lies within a meter of its own
                    // curve (and often near a node), so the distance-based vehicle/node/
                    // edge tests below won even when the click was dead on the handle.
                    if (TryStartControlPointDrag(worldVec, e.Location))
                        break;

                    // Try vehicle hit-test next
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

                    // Find nearest node (generous fixed pick radius; the highlight itself
                    // draws at the zoom-scaled node-dot size)
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.NodePickDistance);
                    if (nearNode >= 0)
                    {
                        _editorState.SelectedNode = nearNode;
                        _editorState.SelectedEdge = -1;
                        _editorState.DragNodeIndex = nearNode;
                        _dragStartScreenPos = e.Location;
                        _dragOffset = _roadGraph.Nodes[nearNode].Position - worldVec;
                        _dragActive = false;
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
                    break;
                case EditorTool.Node:
                    _nodeTool.OnClick(worldVec, _roadGraph, _edgeSpatialGrid);
                    break;
                case EditorTool.Delete:
                    _deleteTool.OnClick(worldVec, _roadGraph, _edgeSpatialGrid, _populationManager);
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
                                _roadGraph, _editorState.SelectedPOIType);
                        }
                    }
                    break;
                case EditorTool.Signal:
                    // Change Type: a click cycles the node's signal type. Tuning lives on
                    // the signals submenu's dedicated Rotate tool.
                    _signalTool.OnClick(worldVec, _roadGraph);
                    break;
                case EditorTool.SignalRotate:
                    _signalTool.RotatePhase(worldVec, _roadGraph, _trafficSignals);
                    break;
                case EditorTool.SignalExempt:
                    _signalTool.ToggleExemption(worldVec, _roadGraph, _stopSigns, _yieldSigns);
                    break;
                case EditorTool.SignalControl:
                {
                    // Toggle the clicked traffic light between fixed-time (default) and
                    // actuated control. Only light nodes respond; the flag flip bumps the
                    // graph version, so the signal system re-projects it on the next tick.
                    int ctrlNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    if (ctrlNode >= 0 && _roadGraph.Nodes[ctrlNode].Flags.HasFlag(NodeFlags.TrafficLight))
                        _roadGraph.SetNodeFlags(ctrlNode, _roadGraph.Nodes[ctrlNode].Flags ^ NodeFlags.ActuatedSignal);
                    break;
                }
                case EditorTool.UpdateSegment:
                    _updateSegmentTool.OnClick(worldVec, _roadGraph, _editorState, _edgeSpatialGrid);
                    break;
                case EditorTool.Water:
                    _waterTool.OnClick(worldVec, _waterLayer, _editorState);
                    break;
            }
        }
        else if (e.Button == MouseButtons.Right)
        {
            // Right-click is the universal CANCEL (same as ESC): abort whatever the
            // active tool has in progress, or fall back to the Select tool.
            CancelActiveTool();
        }
    }

    /// <summary>
    /// Shared right-click / ESC "cancel" semantics, one step per invocation: aborts the
    /// in-progress road chain, then an in-progress water stroke or pending stream chain,
    /// then exits lane-restrict mode, then clears any selection; with nothing left to
    /// cancel, switches the active tool back to Select.
    /// </summary>
    private void CancelActiveTool()
    {
        if (_editorState.IsDrawingRoad)
        {
            _roadTool.OnCancel(_editorState);
            return;
        }
        if (_editorState.IsPaintingWater)
        {
            // Abort an in-progress brush/erase stroke (Esc mid-drag).
            _editorState.IsPaintingWater = false;
            _editorState.WaterLastDabPos = null;
            return;
        }
        if (_editorState.WaterStreamAnchor is not null)
        {
            // Drop the pending stream chain; a second cancel then falls through to Select.
            _editorState.WaterStreamAnchor = null;
            _editorState.WaterStreamPrevDir = null;
            return;
        }
        if (_editorState.LaneRestrictionMode)
        {
            _editorState.LaneRestrictionMode = false;
            _editorState.LaneRestrictionEdge = -1;
            return;
        }
        if (_editorState.SelectedNode >= 0 || _editorState.SelectedEdge >= 0
            || _editorState.SelectedVehicle >= 0)
        {
            _editorState.SelectedNode = -1;
            _editorState.SelectedEdge = -1;
            _editorState.SelectedVehicle = -1;
            return;
        }
        _editorState.ResetToolState();
        _editorState.ActiveTool = EditorTool.Select;
    }

    /// <summary>
    /// Starts a control-point drag when the click lands on a VISIBLE Bézier handle — both
    /// handles of the selected edge, or the node-adjacent handle of each edge incident to
    /// the selected node (exactly the set SceneRenderer.DrawControlPointHandles draws).
    /// Handles draw topmost in Select mode, so they take the click BEFORE the
    /// vehicle/node/edge hit-tests; the grab radius mirrors the drawn handle radius
    /// (max(3, 5/zoom)) plus slop. Handles on a two-way pair resolve to the primary
    /// (lower-index) edge, matching the renderer; SetControlPoint syncs the reverse twin
    /// during the drag.
    /// </summary>
    /// <returns>True when a drag was started (the click is consumed).</returns>
    private bool TryStartControlPointDrag(Vector2 worldVec, Point screenPos)
    {
        float grabRadius = MathF.Max(4.5f, 8f / _camera.Zoom);
        float bestDistSq = grabRadius * grabRadius;
        int bestEdge = -1, bestCp = -1;
        bool bestIsNodeAdjacent = false;

        // Tests one handle, first mapping the edge to its drawn primary twin (lower index
        // of a two-way pair) and flipping the CP side when the mapping crosses the pair.
        void Test(int edgeIdx, int cpSide, bool nodeAdjacent)
        {
            if (edgeIdx < 0 || edgeIdx >= _roadGraph.Edges.Count) return;
            int rev = _roadGraph.FindReverseEdge(edgeIdx);
            if (rev >= 0 && rev < edgeIdx) { edgeIdx = rev; cpSide = cpSide == 1 ? 2 : 1; }
            var edge = _roadGraph.Edges[edgeIdx];
            if (edge.FromNode < 0) return;
            var cp = cpSide == 1 ? edge.ControlPoint1 : edge.ControlPoint2;
            float d = Vector2.DistanceSquared(worldVec, cp);
            if (d < bestDistSq)
            {
                bestDistSq = d;
                bestEdge = edgeIdx;
                bestCp = cpSide;
                bestIsNodeAdjacent = nodeAdjacent;
            }
        }

        // Both handles of the selected edge (side flips cancel out across the pair).
        if (_editorState.SelectedEdge >= 0)
        {
            Test(_editorState.SelectedEdge, 1, nodeAdjacent: false);
            Test(_editorState.SelectedEdge, 2, nodeAdjacent: false);
        }

        // The node-adjacent handle of each edge incident to the selected node.
        int selNode = _editorState.SelectedNode;
        if (selNode >= 0 && selNode < _roadGraph.Nodes.Count
            && !float.IsNaN(_roadGraph.Nodes[selNode].Position.X))
        {
            foreach (int eIdx in _roadGraph.GetOutgoingEdges(selNode)) Test(eIdx, 1, nodeAdjacent: true);
            foreach (int eIdx in _roadGraph.GetIncomingEdges(selNode)) Test(eIdx, 2, nodeAdjacent: true);
        }

        if (bestEdge < 0) return false;

        // A node-adjacent handle keeps the node selected (its handle set stays visible);
        // an edge handle keeps its edge selected.
        if (!bestIsNodeAdjacent)
        {
            _editorState.SelectedNode = -1;
            _editorState.SelectedEdge = bestEdge;
        }
        _editorState.DragEdgeIndex = bestEdge;
        _editorState.DragControlPointIndex = bestCp;
        var dragEdge = _roadGraph.Edges[bestEdge];
        var cpPos = bestCp == 1 ? dragEdge.ControlPoint1 : dragEdge.ControlPoint2;
        _dragStartScreenPos = screenPos;
        _dragOffset = cpPos - worldVec;
        _dragActive = false;
        _canvas.Cursor = Cursors.Hand;
        return true;
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
            _uiRoot.OnMouseUp(e.X, e.Y);
            if (_editorState.IsPaintingWater)
            {
                // End of a brush/erase stroke.
                _editorState.IsPaintingWater = false;
                _editorState.WaterLastDabPos = null;
            }
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

        // Clear the placement ghosts up front; the Destination/Node hover cases below
        // recompute them when applicable. This guarantees no stale ghost renders down any
        // early-return path (UI, pan, slider, node/control-point drag) — e.g. during a
        // middle-button pan while the Destination or Node tool is active.
        _editorState.GhostDestPos = null;
        _editorState.GhostFootPos = null;
        _editorState.GhostEdge = -1;
        _editorState.NodeGhostPos = null;
        _editorState.RoadCrossingPreviews.Clear();
        _editorState.RoadAnchorGhostPos = null;
        _editorState.RoadPreviewCp1 = null;
        _editorState.RoadPreviewCp2 = null;
        _editorState.WaterGhostPos = null;

        // A captured UI drag (minimap scrub, slider thumb) owns every move until release,
        // even with the cursor far outside the panel. Capture with NO button held is
        // stale — a modal dialog opened inside a Click (New/Save/Load) swallowed the
        // matching mouse-up — so release it here and fall through to normal routing;
        // hover then works immediately after the dialog closes instead of only
        // self-healing on the next click.
        if (_uiRoot.HasCapture)
        {
            if (e.Button != MouseButtons.None)
            {
                _uiRoot.OnMouseMove(e.X, e.Y);
                return;
            }
            _uiRoot.OnMouseUp(e.X, e.Y);
        }

        // Middle-drag pan runs before UI hover so panning never stalls while the cursor
        // crosses a panel or button.
        if (_isPanning)
        {
            float dx = e.X - _lastMousePos.X;
            float dy = e.Y - _lastMousePos.Y;
            _camera.Pan(dx, dy);
            _lastMousePos = e.Location;
            return;
        }

        // Water paint/erase stroke also runs before the UI hover gate (like panning) so
        // an in-progress drag never stalls while the cursor crosses a panel.
        if (_editorState.IsPaintingWater)
        {
            if (e.Button == MouseButtons.Left)
            {
                var paintPos = _camera.ScreenToWorld(e.X, e.Y, _canvas.Width, _canvas.Height);
                var paintVec = new Vector2(paintPos.X, paintPos.Y);
                _waterTool.OnDrag(paintVec, _waterLayer, _editorState);
                _editorState.WaterGhostPos = paintVec;
                return;
            }
            // Self-heal: the matching mouse-up was lost (released outside the window).
            _editorState.IsPaintingWater = false;
            _editorState.WaterLastDabPos = null;
        }

        // Hovering any retained-mode panel suppresses map hover highlights.
        if (_uiRoot.OnMouseMove(e.X, e.Y))
        {
            _editorState.HoveredNode = -1;
            _editorState.HoveredEdge = -1;
            _editorState.HoveredVehicle = -1;
            return;
        }

        if (_editorState.IsDraggingNode)
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
                        int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.NodePickDistance);
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
                case EditorTool.Water:
                {
                    // No map hover with the Water tool — just the brush/stream ghost.
                    _editorState.HoveredNode = -1;
                    _editorState.HoveredEdge = -1;
                    _editorState.HoveredVehicle = -1;
                    _editorState.WaterGhostPos = worldVec;
                    break;
                }
                case EditorTool.Delete:
                {
                    // Highlight exactly what a click will delete: the node under the cursor
                    // (which goes together with all its attached segments), otherwise the
                    // nearest edge.
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.NodePickDistance);
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
                case EditorTool.Signal:
                {
                    _editorState.HoveredEdge = -1;
                    _editorState.HoveredNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    break;
                }
                case EditorTool.SignalRotate:
                {
                    // Only traffic-light nodes are valid rotate targets.
                    _editorState.HoveredEdge = -1;
                    int rotNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    _editorState.HoveredNode = rotNode >= 0
                        && _roadGraph.Nodes[rotNode].Flags.HasFlag(NodeFlags.TrafficLight) ? rotNode : -1;
                    break;
                }
                case EditorTool.SignalExempt:
                {
                    // Highlight the exact approach edge a click would toggle: the incoming
                    // edge nearest the cursor at the nearest stop/yield node.
                    _editorState.HoveredNode = -1;
                    _editorState.HoveredEdge = -1;
                    int exNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    if (exNode >= 0 && (_roadGraph.Nodes[exNode].Flags
                            & (NodeFlags.StopSign | NodeFlags.Yield)) != 0)
                        _editorState.HoveredEdge = SignalTool.FindNearestIncomingEdge(worldVec, _roadGraph, exNode);
                    break;
                }
                case EditorTool.SignalControl:
                {
                    // Only traffic-light nodes are valid targets, so only they highlight.
                    _editorState.HoveredEdge = -1;
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    _editorState.HoveredNode = nearNode >= 0
                        && _roadGraph.Nodes[nearNode].Flags.HasFlag(NodeFlags.TrafficLight) ? nearNode : -1;
                    break;
                }
                case EditorTool.UpdateSegment:
                {
                    // Edges only: the tool retypes segments, nodes are not targets.
                    _editorState.HoveredNode = -1;
                    _editorState.HoveredEdge = _edgeSpatialGrid.FindNearestEdge(_roadGraph, worldVec, EditorState.SnapDistance);
                    break;
                }
                case EditorTool.Node:
                {
                    // Over an existing node: highlight it, no ghost (a click adds nothing).
                    // Otherwise show the ghost exactly where the click will create the node
                    // (snapped onto the nearest road, or free at the cursor).
                    _editorState.HoveredEdge = -1;
                    _editorState.HoveredNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);
                    if (_editorState.HoveredNode < 0)
                    {
                        var nodeGhost = NodeTool.ComputeGhost(worldVec, _roadGraph, _edgeSpatialGrid);
                        _editorState.NodeGhostPos = nodeGhost?.Pos;
                        _editorState.NodeGhostRadius = nodeGhost?.Radius ?? 0f;
                    }
                    break;
                }
                case EditorTool.Road:
                {
                    _editorState.HoveredNode = -1;
                    _editorState.HoveredEdge = -1;

                    // Anchor ghost, shown at all times: where a click would land — the
                    // chain start before the first click, the segment end while drawing.
                    var anchorGhost = RoadTool.ComputeAnchorGhost(worldVec, _roadGraph, _edgeSpatialGrid);
                    _editorState.RoadAnchorGhostPos = anchorGhost;

                    // Ghost the intersection nodes the in-progress segment will create
                    // where the preview crosses existing roads (the commit runs the
                    // same crossing detection on the real edge). The segment ends at the
                    // SNAPPED anchor, matching the edge the commit creates. Cleared at
                    // the top of this handler, so the list is stale-free on every path.
                    if (_editorState.IsDrawingRoad)
                    {
                        Vector2 startPos;
                        int ignoreNode = -1;
                        if (_editorState.RoadStartNode is { } startNode)
                        {
                            if (startNode >= _roadGraph.Nodes.Count
                                || float.IsNaN(_roadGraph.Nodes[startNode].Position.X))
                                break; // start node went defunct mid-draw
                            startPos = _roadGraph.Nodes[startNode].Position;
                            ignoreNode = startNode;
                        }
                        else
                        {
                            startPos = _editorState.RoadStartAnchorPos!.Value;
                        }

                        // Curved mode: preview the same tangent-continuous Bezier the
                        // commit will create, and probe crossings along it.
                        if (_editorState.SelectedCurved
                            && RoadTool.TryGetChainTangent(_roadGraph, _editorState) is { } tangent)
                        {
                            var (cp1, cp2) = RoadTool.ComputeCurveControls(startPos, anchorGhost, tangent);
                            _editorState.RoadPreviewCp1 = cp1;
                            _editorState.RoadPreviewCp2 = cp2;
                            _roadGraph.FindCurveCrossings(startPos, cp1, cp2, anchorGhost,
                                ignoreNode, _editorState.RoadStartEdge, _editorState.RoadCrossingPreviews);
                        }
                        else
                        {
                            _roadGraph.FindSegmentCrossings(startPos, anchorGhost, ignoreNode,
                                _editorState.RoadStartEdge, _editorState.RoadCrossingPreviews);
                        }
                    }
                    break;
                }
                case EditorTool.Destination:
                {
                    _editorState.HoveredEdge = -1;
                    int nearNode = _roadGraph.FindNearestNode(worldVec, EditorState.SnapDistance);

                    // Snap to (and later flag) an existing node only when it is eligible AND not
                    // already carrying a marker — never grab an existing destination node. Otherwise
                    // fall through to placement (the ghost), which drops a NEW destination.
                    // (Ghost fields were cleared at the top of this handler, so the snap and
                    // no-nearby-road cases simply leave them null.)
                    bool snapToNode = nearNode >= 0 && _roadGraph.CanPlaceMarker(nearNode)
                        && (_roadGraph.Nodes[nearNode].Flags & NodeFlags.Destination) == 0;

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
        // Wheel zoom bypasses UiRoot, so the modal Settings dialog gates it here.
        if (_settingsDialog.Visible) return;

        float zoomFactor = e.Delta > 0 ? 1.15f : 1f / 1.15f;
        _camera.ZoomAt(zoomFactor, e.X, e.Y, _canvas.Width, _canvas.Height);
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
        _waterLayer.Clear(); // stress scene replaces the whole map, water included

        // Reset per-map traffic-control overrides exactly as NewMap() does.
        _stopSigns.SetExemptEdges(new List<int>());
        _yieldSigns.SetExemptEdges(new List<int>());
        _trafficSignals.SetPhaseRotations(new List<(int, byte)>());

        _simLoop.RebuildWorldCaches();
        _sceneRenderer.OnMapReplaced();

        int spawned = _spawner.SpawnBulk(vehicleCount);

        // Center on the grid middle, zoomed out to fit the ~5 km grid. CenterX/CenterY are
        // screen-pixel pan offsets (not world coords), so set Zoom first then use CenterOnWorld.
        _camera.Zoom = camZoom;
        _camera.CenterOnWorld((gridCols - 1) * spacing / 2f, (gridRows - 1) * spacing / 2f);

        _simLoop.Clock.TimeOfDay = 12.0;
        _simLoop.Paused = false;
        _simLoop.TimeScaleExponent = 0;

        // Reset editor selection/tool exactly as NewMap() does.
        _editorState.ResetToolState();
        _editorState.ActiveTool = Editor.EditorTool.Select;

        // Show FPS HUD immediately so the user sees the load metrics.
        _hudPanel.Visible = true;

        System.Diagnostics.Debug.WriteLine($"[StressScene] grid {gridCols}x{gridRows}, spawned {spawned}/{vehicleCount} vehicles");
    }

    /// <summary>
    /// Captures a non-intrusive performance baseline snapshot to benchmark.log.
    /// Reads per-frame stats from <see cref="_perfTelemetry"/> (whose Sample already
    /// drains the pathfind accumulators each frame) so this method does not reset any
    /// shared state.
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
            _perfTelemetry.AvgFps, _perfTelemetry.AvgSimMs, _perfTelemetry.AvgDrawMs,
            _perfTelemetry.LastPathfindMs, _perfTelemetry.LastPathfindCalls,
            _vehicles.Count, Roads.App.Vehicles.SteeringController.LastConflictCoOccupancy, offroad,
            _simLoop.LastTiming);
        System.Diagnostics.Debug.WriteLine($"[Baseline] captured: fps={_perfTelemetry.AvgFps:F1}, vehicles={_vehicles.Count}");
    }

}
