using System.Diagnostics;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App;

/// <summary>
/// Runs the simulation at a fixed 30 Hz timestep with time scaling.
/// Orchestrates cache rebuilds, traffic system updates, vehicle AI, physics,
/// rerouting, and auto-spawning each tick.
/// </summary>
public class SimulationLoop
{
    private readonly Stopwatch _stopwatch = Stopwatch.StartNew();
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly SpatialGrid _vehicleGrid;
    private readonly StopLineCache _stopLineCache;
    private readonly IntersectionArcCache _intersectionArcs;
    private readonly EdgeSpatialGrid _edgeSpatialGrid;
    private readonly TrafficSignalSystem _trafficSignals;
    private readonly StopSignSystem _stopSigns;
    private readonly YieldSignSystem _yieldSigns;
    private readonly VehicleSpawner _spawner;
    private readonly PopulationManager _populationManager;
    private readonly EditorState _editorState;
    private readonly GraphChangeHandler _graphChangeHandler;

    private double _lastSimTime;
    private double _simAccumulator;
    private bool _paused;
    private int _timeScaleExponent; // 0=1x, 1=2x, ... 6=64x

    /// <summary>Fixed simulation timestep in seconds (30 Hz).</summary>
    public const float SimDt = 1f / 30f;

    /// <summary>Game clock tracking time of day (0–24 hours).</summary>
    public SimulationClock Clock { get; } = new();

    /// <summary>Maximum number of vehicles allowed in the simulation.</summary>
    public const int MaxVehicles = 200;

    /// <summary>Gets or sets whether the simulation is paused.</summary>
    public bool Paused { get => _paused; set => _paused = value; }

    /// <summary>Gets or sets the time scale exponent (0=1x, 1=2x, ... 6=64x).</summary>
    public int TimeScaleExponent
    {
        get => _timeScaleExponent;
        set => _timeScaleExponent = Math.Clamp(value, 0, 6);
    }

    /// <summary>Gets the current time scale multiplier.</summary>
    public int TimeScale => _paused ? 0 : (1 << _timeScaleExponent);

    /// <summary>Gets the population manager for resident/schedule info.</summary>
    public PopulationManager Population => _populationManager;

    public SimulationLoop(RoadGraph graph, VehicleStore vehicles,
        SpatialGrid vehicleGrid, StopLineCache stopLineCache,
        IntersectionArcCache intersectionArcs, EdgeSpatialGrid edgeSpatialGrid,
        TrafficSignalSystem trafficSignals, StopSignSystem stopSigns,
        YieldSignSystem yieldSigns, VehicleSpawner spawner,
        PopulationManager populationManager, EditorState editorState,
        GraphChangeHandler graphChangeHandler)
    {
        _graph = graph;
        _vehicles = vehicles;
        _vehicleGrid = vehicleGrid;
        _stopLineCache = stopLineCache;
        _intersectionArcs = intersectionArcs;
        _edgeSpatialGrid = edgeSpatialGrid;
        _trafficSignals = trafficSignals;
        _stopSigns = stopSigns;
        _yieldSigns = yieldSigns;
        _spawner = spawner;
        _populationManager = populationManager;
        _editorState = editorState;
        _graphChangeHandler = graphChangeHandler;
        _lastSimTime = _stopwatch.Elapsed.TotalSeconds;
    }

    /// <summary>
    /// Advances the simulation by one or more fixed timesteps based on elapsed wall time.
    /// Step 0, every call (paused or not): GraphChangeHandler.HandleIfNeeded converges
    /// graph-change fix-ups — editor call sites do not invoke it manually.
    /// Update order within each timestep:
    /// 1. Rebuild the vehicle spatial grid and all world caches (see <see cref="RebuildWorldCaches"/>)
    /// 2. Update traffic systems (signals, stop signs, yield signs)
    /// 3. Lane change logic (must precede steering)
    /// 4. Steering controller (depends on all of the above)
    /// 5. Vehicle physics (applies steering/throttle/brake computed in step 4)
    /// 6. Reroute finished vehicles and auto-spawn new ones
    /// While paused, only HandleIfNeeded and <see cref="RebuildWorldCaches"/> run, so
    /// editor changes take effect immediately but simulation time and signal timers do
    /// not advance.
    /// </summary>
    public void Tick()
    {
        double now = _stopwatch.Elapsed.TotalSeconds;
        double wallElapsed = now - _lastSimTime;
        _lastSimTime = now;

        // Step 0: converge graph-change fix-ups (stale selections, marker flags,
        // vehicles on defunct edges) before any cache rebuild or vehicle update.
        // Runs in both paused and active modes; O(1) when the graph is unchanged.
        _graphChangeHandler.HandleIfNeeded();

        if (_paused)
        {
            // Keep caches and traffic-control systems current so geometry drags
            // and signal-type/exemption/phase toggles take effect immediately
            // while paused. Signal timers do not advance (Update is not called).
            RebuildWorldCaches();
            return;
        }

        _simAccumulator += wallElapsed * TimeScale;
        const int MaxStepsPerFrame = 128;
        int steps = 0;

        while (_simAccumulator >= SimDt && steps < MaxStepsPerFrame)
        {
            _vehicleGrid.Rebuild(_vehicles.PosX, _vehicles.PosY, _vehicles.Count);
            RebuildWorldCaches();
            _trafficSignals.Update(_graph, SimDt);
            _stopSigns.Update(_graph, _vehicles, _stopLineCache, SimDt);
            _yieldSigns.Update(_graph, _vehicles, _stopLineCache, _intersectionArcs, SimDt);
            LaneChangeLogic.UpdateAll(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache, SimDt);
            LaneChangeLogic.ApplyMergeSpeedBias(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache);
            SteeringController.UpdateAll(_vehicles, _graph, _vehicleGrid, _stopLineCache, _intersectionArcs, _trafficSignals, _stopSigns, _yieldSigns, SimDt);
            VehiclePhysics.UpdateAll(_vehicles, SimDt);

            _spawner.RerouteFinished();
            _populationManager.Update(SimDt, Clock.TimeOfDay, Clock.DayNumber);
            _spawner.ScheduleModeActive = _populationManager.ScheduleModeEnabled;
            _spawner.AutoSpawn(SimDt, MaxVehicles);
            Clock.Advance(SimDt);

            _simAccumulator -= SimDt;
            steps++;
        }

        // Clear stale vehicle selection
        int selVeh = _editorState.SelectedVehicle;
        if (selVeh >= 0 && (selVeh >= _vehicles.Count || _vehicles.State[selVeh] != VehicleState.Driving))
            _editorState.SelectedVehicle = -1;
    }

    /// <summary>Graph version after the last normalize pass; phase 1 of
    /// <see cref="RebuildWorldCaches"/> is skipped while it matches.</summary>
    private int _lastNormalizedVersion = -1;

    /// <summary>
    /// Maintains all graph-derived state in two phases.
    /// Phase 1 (normalize) is the ONLY place graph mutation is permitted during cache
    /// maintenance; it runs only when the graph version changed and may bump Version:
    /// signal auto-assignment (lights before stop signs — the stop policy reads the
    /// TrafficLight flag), then the stop-line rebuild, then default lane restrictions
    /// (which read stop-line tangents). No normalize step reads what a later step
    /// writes, so a single pass converges (verified by a debug-only second pass).
    /// Phase 2 (rebuild) is a pure projection of the settled graph into caches —
    /// asserted mutation-free in debug builds. After this method returns, every cache
    /// is current with graph.Version; no follow-up cascade occurs on the next call.
    /// Safe to call every frame: each step early-outs in O(1) when the graph version
    /// and its dirty flag are unchanged. Does not rebuild the vehicle spatial grid
    /// (position-based, rebuilt per simulation step) and does not call the signal
    /// systems' Update methods (signal timers must not advance while paused).
    /// </summary>
    public void RebuildWorldCaches()
    {
        // Phase 1 — Normalize derived graph state.
        if (_graph.Version != _lastNormalizedVersion)
        {
            TrafficSignalSystem.AutoAssign(_graph);
            StopSignSystem.AutoAssign(_graph);
            _stopLineCache.RebuildIfNeeded(_graph);
            _graph.ApplyDefaultLaneRestrictions(_stopLineCache);
#if DEBUG
            // Empirically verify single-pass convergence: a second pass must not bump.
            int converged = _graph.Version;
            TrafficSignalSystem.AutoAssign(_graph);
            StopSignSystem.AutoAssign(_graph);
            _graph.ApplyDefaultLaneRestrictions(_stopLineCache);
            Debug.Assert(_graph.Version == converged,
                "Normalize did not converge in one pass.");
#endif
            _lastNormalizedVersion = _graph.Version; // post-mutation: settled
        }

        // Phase 2 — Pure rebuilds at the settled version.
        int settled = _graph.Version;
        _stopLineCache.RebuildIfNeeded(_graph);
        _intersectionArcs.RebuildIfNeeded(_graph, _stopLineCache);
        _edgeSpatialGrid.RebuildIfNeeded(_graph);
        _trafficSignals.RebuildIfNeeded(_graph);
        _stopSigns.RebuildIfNeeded(_graph);
        _yieldSigns.RebuildIfNeeded(_graph);
        Debug.Assert(_graph.Version == settled,
            "Rebuild phase mutated the graph — RebuildIfNeeded steps must be pure reads.");
    }
}
