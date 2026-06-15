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

    /// <summary>Edge spatial grid (rebuilt on graph change) — used by the renderer for visible-edge culling.</summary>
    public EdgeSpatialGrid EdgeGrid => _edgeSpatialGrid;

    /// <summary>
    /// Per-tick wall-clock breakdown of the simulation subsystems in milliseconds,
    /// summed across all fixed substeps executed in the most recent active <see cref="Tick"/>.
    /// Pure profiling instrumentation — has no effect on behavior. The buckets sum to
    /// approximately the total sim-tick time (minus small un-instrumented bookkeeping).
    /// </summary>
    public readonly record struct SimTimingBreakdown(
        double GridMs, double CachesMs, double SignalsMs, double LaneChangeMs,
        double SteeringMs, double PhysicsMs, double RerouteMs, double PopulationMs);

    /// <summary>Subsystem timing from the most recent active <see cref="Tick"/> (substeps &gt; 0).</summary>
    public SimTimingBreakdown LastTiming { get; private set; }

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

        // Watchdog: assert the vehicle↔resident index mappings are internally consistent
        // (debug-only). Runs after HandleIfNeeded so it is armed while paused too.
        _populationManager.ValidateMappings();

        if (_paused)
        {
            // Keep caches and traffic-control systems current so geometry drags
            // and signal-type/exemption/phase toggles take effect immediately
            // while paused. Signal timers do not advance (Update is not called).
            // The vehicle grid rebuilds too so editor hit-tests (hover/click
            // selection) work while paused — e.g. right after a map load, when the
            // grid was bulk-cleared and no sim step has indexed the new vehicles.
            _vehicleGrid.Rebuild(_vehicles.PosX, _vehicles.PosY, _vehicles.Count);
            RebuildWorldCaches();
            return;
        }

        // Spiral-of-death guard: cap the real frame time we account for in one Tick. Without this,
        // a slow frame (heavy load) injects a huge backlog, the loop runs up to MaxStepsPerFrame
        // substeps to "catch up", and the sim advances many ticks between renders — vehicles jump.
        // Capping wallElapsed keeps an overloaded sim smooth (a few substeps/frame, slow-motion)
        // while preserving fast-forward: a high TimeScale still scales the injected sim-time up,
        // because the spiral comes from a slow frame, not from a high TimeScale.
        const int MaxStepsPerFrame = 128;
        const double MaxFrameTime = 2.0 / 30.0; // ≈ at most 2 substeps of backlog per frame at 1x
        _simAccumulator += Math.Min(wallElapsed, MaxFrameTime) * TimeScale;
        int steps = 0;

        // Per-subsystem timing accumulators (Stopwatch ticks), summed over all substeps
        // this Tick. Converted to ms and published in LastTiming after the loop.
        long gridT = 0, cacheT = 0, sigT = 0, laneT = 0, steerT = 0, physT = 0, rerouteT = 0, popT = 0;

        while (_simAccumulator >= SimDt && steps < MaxStepsPerFrame)
        {
            long ts = Stopwatch.GetTimestamp();
            _vehicleGrid.Rebuild(_vehicles.PosX, _vehicles.PosY, _vehicles.Count);
            long t1 = Stopwatch.GetTimestamp(); gridT += t1 - ts;

            RebuildWorldCaches();
            long t2 = Stopwatch.GetTimestamp(); cacheT += t2 - t1;

            _trafficSignals.Update(_graph, SimDt);
            _stopSigns.Update(_graph, _vehicles, _stopLineCache, SimDt);
            _yieldSigns.Update(_graph, _vehicles, _stopLineCache, _intersectionArcs, SimDt);
            long t3 = Stopwatch.GetTimestamp(); sigT += t3 - t2;

            LaneChangeLogic.UpdateAll(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache, SimDt);
            LaneChangeLogic.ApplyMergeSpeedBias(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache);
            long t4 = Stopwatch.GetTimestamp(); laneT += t4 - t3;

            SteeringController.UpdateAll(_vehicles, _graph, _vehicleGrid, _stopLineCache, _intersectionArcs, _trafficSignals, _stopSigns, _yieldSigns, SimDt);
            long t5 = Stopwatch.GetTimestamp(); steerT += t5 - t4;

            VehiclePhysics.UpdateAll(_vehicles, SimDt);
            long t6 = Stopwatch.GetTimestamp(); physT += t6 - t5;

            _spawner.RerouteFinished();
            long t7 = Stopwatch.GetTimestamp(); rerouteT += t7 - t6;

            _populationManager.Update(SimDt, Clock.TimeOfDay, Clock.DayNumber);
            _spawner.ScheduleModeActive = _populationManager.ScheduleModeEnabled;
            _spawner.AutoSpawn(SimDt, MaxVehicles);
            long t8 = Stopwatch.GetTimestamp(); popT += t8 - t7;

            Clock.Advance(SimDt);

            _simAccumulator -= SimDt;
            steps++;
        }

        // If we hit the step cap, the sim can't keep up: drop the residual backlog so it runs in
        // smooth slow-motion rather than accumulating unbounded debt (the spiral).
        if (steps >= MaxStepsPerFrame)
            _simAccumulator = 0;

        if (steps > 0)
        {
            double toMs = 1000.0 / Stopwatch.Frequency;
            LastTiming = new SimTimingBreakdown(
                gridT * toMs, cacheT * toMs, sigT * toMs, laneT * toMs,
                steerT * toMs, physT * toMs, rerouteT * toMs, popT * toMs);
        }

        // Watchdog: centralized removal fixup (VehicleStore.VehicleRemoving /
        // VehiclesCleared) keeps the selection in range; out of range here means a
        // removal path bypassed the store's notification.
        int selVeh = _editorState.SelectedVehicle;
        Debug.Assert(selVeh < _vehicles.Count,
            "SelectedVehicle out of range — a vehicle removal bypassed centralized fixup.");
        if (selVeh >= _vehicles.Count)
            _editorState.SelectedVehicle = -1; // release-mode self-heal
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
