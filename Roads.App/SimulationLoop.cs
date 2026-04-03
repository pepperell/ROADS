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
    private readonly EditorState _editorState;

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

    public SimulationLoop(RoadGraph graph, VehicleStore vehicles,
        SpatialGrid vehicleGrid, StopLineCache stopLineCache,
        IntersectionArcCache intersectionArcs, EdgeSpatialGrid edgeSpatialGrid,
        TrafficSignalSystem trafficSignals, StopSignSystem stopSigns,
        YieldSignSystem yieldSigns, VehicleSpawner spawner, EditorState editorState)
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
        _editorState = editorState;
        _lastSimTime = _stopwatch.Elapsed.TotalSeconds;
    }

    /// <summary>
    /// Advances the simulation by one or more fixed timesteps based on elapsed wall time.
    /// Update order within each timestep:
    /// 1. Rebuild spatial indices and traffic systems
    /// 2. Update traffic systems (signals, stop signs, yield signs)
    /// 3. Lane change logic (must precede steering)
    /// 4. Steering controller (depends on all of the above)
    /// 5. Vehicle physics (applies steering/throttle/brake computed in step 4)
    /// 6. Reroute finished vehicles and auto-spawn new ones
    /// </summary>
    public void Tick()
    {
        double now = _stopwatch.Elapsed.TotalSeconds;
        double wallElapsed = now - _lastSimTime;
        _lastSimTime = now;

        if (_paused)
        {
            // Keep caches current so arcs/stop-lines render correctly
            // while the user drags nodes or control points.
            _stopLineCache.RebuildIfNeeded(_graph);
            _graph.ApplyDefaultLaneRestrictions(_stopLineCache);
            _intersectionArcs.RebuildIfNeeded(_graph, _stopLineCache);
            _edgeSpatialGrid.RebuildIfNeeded(_graph);
            return;
        }

        _simAccumulator += wallElapsed * TimeScale;
        const int MaxStepsPerFrame = 128;
        int steps = 0;

        while (_simAccumulator >= SimDt && steps < MaxStepsPerFrame)
        {
            _vehicleGrid.Rebuild(_vehicles.PosX, _vehicles.PosY, _vehicles.Count);
            _stopLineCache.RebuildIfNeeded(_graph);
            _graph.ApplyDefaultLaneRestrictions(_stopLineCache);
            _intersectionArcs.RebuildIfNeeded(_graph, _stopLineCache);
            _edgeSpatialGrid.RebuildIfNeeded(_graph);
            _trafficSignals.RebuildIfNeeded(_graph);
            _stopSigns.RebuildIfNeeded(_graph);
            _yieldSigns.RebuildIfNeeded(_graph);
            _trafficSignals.Update(_graph, SimDt);
            _stopSigns.Update(_graph, _vehicles, _stopLineCache, SimDt);
            _yieldSigns.Update(_graph, _vehicles, _stopLineCache, _intersectionArcs, SimDt);
            LaneChangeLogic.UpdateAll(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache, SimDt);
            LaneChangeLogic.ApplyMergeSpeedBias(_vehicles, _graph, _vehicleGrid, _intersectionArcs, _stopLineCache);
            SteeringController.UpdateAll(_vehicles, _graph, _vehicleGrid, _stopLineCache, _intersectionArcs, _trafficSignals, _stopSigns, _yieldSigns, SimDt);
            VehiclePhysics.UpdateAll(_vehicles, SimDt);

            _spawner.RerouteFinished();
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
}
