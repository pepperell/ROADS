using System.Diagnostics;
using System.Numerics;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Central orchestrator for the town life system. Owns all residents, processes
/// schedule-driven departures and arrivals, and manages the population lifecycle.
/// When no Home POIs exist, the system is inactive and legacy random spawning continues.
/// </summary>
public class PopulationManager
{
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly SpatialGrid _vehicleGrid;
    private readonly POIRegistry _poiRegistry;

    private readonly List<Resident> _residents = new();

    /// <summary>Maps vehicleIndex → residentId for O(1) swap-and-pop fixup.</summary>
    private readonly Dictionary<int, int> _vehicleToResident = new();

    /// <summary>
    /// In-progress graceful-deletion drains, one per node that is being removed while it still
    /// has population (residents at it, or — for a home — residents who live there). A drain keeps
    /// the node and its incident edges fully present and open, drives its people out over a couple
    /// of phases, then removes the edges and the node. Processed every tick by
    /// <see cref="ProcessDrains"/>. See the "graceful deletion" design.
    /// </summary>
    private readonly List<NodeDrain> _drains = new();

    /// <summary>
    /// Number of consecutive ticks a single resident may fail to spawn out of a drain before it is
    /// removed outright, so a permanently-blocked departure (no path, jammed entry) cannot wedge a
    /// drain forever. ~600 ticks ≈ several seconds of real time at the sim tick rate.
    /// </summary>
    private const int DrainSpawnFailLimit = 600;

    /// <summary>
    /// One node being gracefully drained before deletion. While a drain is active its
    /// <see cref="IncidentEdges"/> are kept non-defunct (so spawned cars have road to drive on) and
    /// the node persists as a temporary spawn point. See <see cref="ProcessDrains"/> for the
    /// two-phase lifecycle (spawn residents out, then close &amp; finalize).
    /// </summary>
    private sealed class NodeDrain
    {
        /// <summary>The node being drained and ultimately removed.</summary>
        public int Node;
        /// <summary>True if <see cref="Node"/> is a Home POI: its whole household emigrates to an entry/exit node.</summary>
        public bool IsHome;
        /// <summary>All edges with <c>FromNode==Node</c> or <c>ToNode==Node</c> at drain start — held
        /// open during the drain, then closed and removed when empty.</summary>
        public List<int> IncidentEdges = new();
        /// <summary>False during the spawn-out phase; true once the close-and-finalize phase has begun.</summary>
        public bool Closing;
        /// <summary>Consecutive ticks the head dormant resident has failed to spawn out (see
        /// <see cref="DrainSpawnFailLimit"/>).</summary>
        public int FailStreak;
    }

    /// <summary>Residents queued for departure when active vehicle count is at capacity.</summary>
    private readonly Queue<int> _departureQueue = new();

    /// <summary>Maximum number of vehicles that can be active simultaneously.</summary>
    private int _maxActiveVehicles;

    /// <summary>Maximum spawns per simulation tick (prevents pop-in during rush hour).</summary>
    private const int MaxSpawnsPerTick = 5;

    /// <summary>Minimum clearance in meters around a spawn point.</summary>
    private const float SpawnClearance = 8f;

    /// <summary>Time headway (seconds) used to size the spawn gap for at-speed entry/exit move-ins,
    /// so cars entering mid-flow keep a safe following distance instead of bunching at the edge.</summary>
    private const float MoveInHeadwaySeconds = 2.5f;

    /// <summary>Through-traffic spawn rate, in cars per second per housed resident at the peak
    /// time-of-day factor (1.0). Scaled by <see cref="HousedPopulation"/> and
    /// <see cref="TimeOfDayTrafficFactor"/>. Tunable; the spawn-clearance gate caps real bursts.</summary>
    private const float ThroughTrafficBaseRate = 0.001f;

    /// <summary>Upper bound on the fractional-car spawn credit, so a blocked entry/exit node can't
    /// build a backlog that floods once it clears.</summary>
    private const float ThroughAccumulatorCap = 2f;

    /// <summary>Max through-traffic spawns attempted per tick (the accumulator normally yields ≪1).</summary>
    private const int ThroughMaxPerTick = 2;

    /// <summary>Fractional-car spawn credit accumulated for entry/exit through-traffic.</summary>
    private float _throughSpawnAccumulator;

    private readonly List<int> _spawnBlockedBuffer = new();

    /// <summary>Reused buffer of ENTRY-capable entry/exit nodes (an OUTGOING edge — a lane into
    /// town — so a vehicle can spawn there and drive in), from the most recent
    /// <see cref="GatherEntryNodes"/> call.</summary>
    private readonly List<int> _entryNodesBuffer = new();
    /// <summary>Reused buffer of EXIT-capable entry/exit nodes (an INCOMING edge — a lane out of
    /// town — so a vehicle can drive there and despawn), from the most recent
    /// <see cref="GatherExitNodes"/> call.</summary>
    private readonly List<int> _exitNodesBuffer = new();
    private readonly List<(int node, float distSq)> _moveInCandidates = new();

    /// <summary>Reusable per-pass map of Work node → number of residents currently employed there
    /// (the employment headcount, distinct from <see cref="POIRegistry"/>'s physical-presence count).
    /// Rebuilt at the start of each job-assignment pass; see the "Employment / job assignment" region.</summary>
    private readonly Dictionary<int, int> _employmentCount = new();
    /// <summary>Reusable per-pass list of resident indices seeking a job, shuffled before assignment.</summary>
    private readonly List<int> _jobSeekers = new();
    /// <summary>Resident indices whose <see cref="Resident.WorkNode"/> changed during the current
    /// incremental job pass — their schedules are regenerated so work trips appear/disappear.</summary>
    private readonly HashSet<int> _jobChanged = new();

    private int _lastDayNumber = -1;
    private int _poiGraphVersion = -1;

    /// <summary>Home/Work POI counts at the last incremental job pass. The graph version bumps on ANY
    /// edit (speed limit, lane count, …), but a job reassignment is only warranted when the population
    /// or workplace count actually changed (requirement: "any change in population or workplace count").
    /// Initialized to -1 so the first pass always runs. See <see cref="UpdateForGraphChange"/>.</summary>
    private int _lastHomeCount = -1;
    private int _lastWorkCount = -1;

    /// <summary>Whether schedule mode is active (Home POIs exist).</summary>
    public bool ScheduleModeEnabled { get; private set; }

    /// <summary>Total number of residents in the town.</summary>
    public int TotalResidents => _residents.Count;

    /// <summary>Number of residents currently driving.</summary>
    public int ActiveDrivers => _vehicleToResident.Count;

    /// <summary>Number of residents currently holding a job (<c>WorkNode &gt;= 0</c>). Read-only,
    /// computed per call (instantaneous, no averaging).</summary>
    public int EmployedWorkers
    {
        get
        {
            int n = 0;
            foreach (var r in _residents)
                if (r.WorkNode >= 0) n++;
            return n;
        }
    }

    /// <summary>Number of workers (archetype wants work, not emigrating) without a job. Read-only,
    /// computed per call.</summary>
    public int UnemployedWorkers
    {
        get
        {
            int n = 0;
            foreach (var r in _residents)
                if (r.WorkNode < 0 && IsEmployable(r)) n++;
            return n;
        }
    }

    /// <summary>Total unfilled job slots across all live workplaces (Σ capacity − employed). Read-only,
    /// computed per call.</summary>
    public int JobOpenings
    {
        get
        {
            int openings = _poiRegistry.GetTotalCapacity(POIType.Work) - EmployedWorkers;
            return openings > 0 ? openings : 0;
        }
    }

    /// <summary>Read-only access to the resident list.</summary>
    public IReadOnlyList<Resident> Residents => _residents;

    /// <summary>The POI registry used by this population manager.</summary>
    public POIRegistry POIRegistry => _poiRegistry;

    public PopulationManager(RoadGraph graph, VehicleStore vehicles,
        SpatialGrid vehicleGrid, POIRegistry poiRegistry, int maxActiveVehicles)
    {
        _graph = graph;
        _vehicles = vehicles;
        _vehicleGrid = vehicleGrid;
        _poiRegistry = poiRegistry;
        _maxActiveVehicles = maxActiveVehicles;

        // Centralized index fixup: the store notifies on every removal/bulk clear,
        // regardless of which system initiated it (see VehicleStore.VehicleRemoving).
        vehicles.VehicleRemoving += OnVehicleRemoving;
        vehicles.VehiclesCleared += OnVehiclesCleared;
    }

    /// <summary>
    /// Called each simulation tick. Checks for POI changes, processes departures
    /// and arrivals, and handles day rollover.
    /// </summary>
    public void Update(float simDt, double timeOfDay, int dayNumber)
    {
        // Rebuild POI registry if graph changed; re-evaluate schedule mode
        _poiRegistry.RebuildIfNeeded(_graph);
        bool hasHomes = _poiRegistry.GetNodesOfType(POIType.Home).Count > 0;

        if (hasHomes && !ScheduleModeEnabled)
        {
            ScheduleModeEnabled = true;
            RebuildPopulation(timeOfDay);
        }
        else if (!hasHomes && ScheduleModeEnabled)
        {
            ScheduleModeEnabled = false;
            ClearPopulation();
        }

        if (!ScheduleModeEnabled) return;

        // Check if POIs changed while schedule mode is active
        if (_poiGraphVersion != _graph.Version)
        {
            _poiGraphVersion = _graph.Version;
            UpdateForGraphChange(timeOfDay);
        }

        // Day rollover
        if (dayNumber != _lastDayNumber && _lastDayNumber >= 0)
            OnDayRollover(timeOfDay);
        _lastDayNumber = dayNumber;

        // Process arrivals (vehicles that reached their destination)
        ProcessArrivals(timeOfDay);

        // First appearance: drive OffMap residents in from an entry/exit node, then process scheduled
        // departures. Both draw from one per-tick spawn budget to bound pop-in.
        int movedIn = ProcessMoveIns(timeOfDay, MaxSpawnsPerTick);
        int departed = ProcessDepartures(timeOfDay, MaxSpawnsPerTick - movedIn);

        // Ambient through-traffic: non-resident cars entering at one active entry/exit node and
        // leaving via a different one, at a population- and time-of-day-scaled rate.
        ProcessThroughTraffic(simDt, timeOfDay);

        // Graceful deletion: spawn people out of draining nodes and out of their homes (emigrants),
        // then close & remove emptied roads. Shares the remaining per-tick spawn budget so a busy
        // rush hour doesn't get an extra burst of drain pop-in.
        ProcessDrains(timeOfDay, Math.Max(0, MaxSpawnsPerTick - movedIn - departed));
    }

    /// <summary>
    /// Builds the initial population based on available Home POIs.
    /// Creates one resident per home capacity slot.
    /// </summary>
    private void RebuildPopulation(double timeOfDay)
    {
        ClearPopulation();

        var homeNodes = _poiRegistry.GetNodesOfType(POIType.Home);
        if (homeNodes.Count == 0) return;

        // Create everyone unemployed (WorkNode = -1); the job-assignment pass below is the only place
        // that hands out jobs, honoring workplace capacity. Schedules are generated afterward so they
        // reflect the assigned WorkNode (work trips only for those who actually got a job).
        int id = 0;
        for (int h = 0; h < homeNodes.Count; h++)
        {
            int homeNode = homeNodes[h];
            int capacity = _poiRegistry.GetCapacity(homeNode, POIType.Home);

            for (int slot = 0; slot < capacity; slot++)
            {
                var traits = DriverPersonalityGenerator.GenerateRandom();
                var (cr, cg, cb) = VehicleStore.RandomCarColor();

                _residents.Add(new Resident
                {
                    Id = id++,
                    HomeNode = homeNode,
                    WorkNode = -1,
                    Traits = traits,
                    ColorR = cr, ColorG = cg, ColorB = cb,
                    // Created off-map: residents must drive in from an entry/exit node before they
                    // are at home. They occupy no POI until they arrive (see ProcessMoveIns).
                    Activity = ResidentActivity.OffMap,
                    VehicleIndex = -1,
                    CurrentPOINode = -1,
                    ScheduleIndex = 0,
                });
            }
        }

        // Assign jobs capacity-limited (nearest-home-first, random order), then build schedules.
        RunTotalJobAssignment();
        foreach (var resident in _residents)
        {
            resident.Schedule = ScheduleGenerator.GenerateWeekday(resident.Traits.Archetype, resident.WorkNode);
            AdvanceSchedulePastTime(resident, timeOfDay);
        }

        _poiGraphVersion = _graph.Version;
    }

    private void ClearPopulation()
    {
        // Remove ALL vehicles, not just resident-linked ones. When the resident population
        // takes over (schedule mode activating, e.g. right after a map load), any leftover
        // legacy vehicles are stale — loaded vehicles lose their resident identity on load
        // (ResidentId is not serialized), so they would otherwise persist as immortal
        // wanderers. Residents repopulate from POIs on the following ticks.
        for (int i = _vehicles.Count - 1; i >= 0; i--)
            _vehicles.Remove(i);
        _residents.Clear();
        _vehicleToResident.Clear();
        _departureQueue.Clear();
        _poiRegistry.ClearOccupancy();
    }

    /// <summary>
    /// Adopts a resident population loaded from a saved map (v2) instead of regenerating one.
    /// Rebuilds the vehicle↔resident links from each driving resident's <see cref="Resident.VehicleIndex"/>,
    /// restores POI occupancy for dormant residents, and enters schedule mode WITHOUT going
    /// through the activation path (which would <see cref="RebuildPopulation"/> and wipe the
    /// loaded cars). Called by MapSerializer.Load after the vehicle section has loaded.
    /// </summary>
    public void AdoptLoadedPopulation(IReadOnlyList<Resident> residents)
    {
        _poiRegistry.RebuildIfNeeded(_graph);

        _residents.Clear();
        _residents.AddRange(residents);
        _vehicleToResident.Clear();
        _departureQueue.Clear();
        _poiRegistry.ClearOccupancy();

        // The resident list is authoritative for vehicle ownership: reset all vehicle links,
        // then re-link each driving resident to its vehicle. This keeps VehicleStore.ResidentId,
        // the _vehicleToResident dict, and Resident.VehicleIndex mutually consistent (the
        // invariants ValidateMappings asserts).
        for (int i = 0; i < _vehicles.Count; i++)
            _vehicles.ResidentId[i] = -1;

        foreach (var res in _residents)
        {
            if (IsDriving(res.Activity)
                && res.VehicleIndex >= 0 && res.VehicleIndex < _vehicles.Count)
            {
                // Driving or MovingIn: owns a loaded vehicle — re-link it.
                _vehicleToResident[res.VehicleIndex] = res.Id;
                _vehicles.ResidentId[res.VehicleIndex] = res.Id;
            }
            else if (res.Activity == ResidentActivity.OffMap)
            {
                // Not yet on the map — stays off-map, occupies no POI, has no vehicle.
                res.VehicleIndex = -1;
            }
            else
            {
                // Dormant (or stale vehicle reference): not in a vehicle — occupy its POI.
                res.Activity = ResidentActivity.Dormant;
                res.VehicleIndex = -1;
                if (res.CurrentPOINode >= 0 && res.CurrentPOINode < _graph.Nodes.Count)
                {
                    var poiType = _graph.Nodes[res.CurrentPOINode].PointOfInterest;
                    if (poiType != POIType.None)
                        _poiRegistry.TryOccupy(res.CurrentPOINode, poiType);
                }
            }
        }

        // Only enter schedule mode if the map actually has Home POIs to drive residents from.
        // A map with no homes is a legacy free-spawn map; leaving schedule mode OFF preserves the
        // loaded (non-resident) vehicles, which the next Update would otherwise wipe via
        // ClearPopulation the moment it sees no homes while schedule mode is on.
        ScheduleModeEnabled = _poiRegistry.GetNodesOfType(POIType.Home).Count > 0;

        // Force a graph-change pass on the FIRST Update after load by NOT pinning _poiGraphVersion to
        // the current version. That first pass (UpdateForGraphChange → RunIncrementalJobAssignment)
        // reconciles any dangling employment from the loaded save (a WorkNode whose workplace no
        // longer exists, possible in an old/edited map) AND regenerates the affected residents'
        // schedules with a real time-of-day — all through the one incremental code path, instead of
        // duplicating reconcile+regen here without a clock. For a normally-saved map (employment
        // already valid) this pass changes nothing.
        _poiGraphVersion = -1;
        _lastDayNumber = -1; // first Update sets the day baseline without a spurious rollover
    }

    /// <summary>
    /// Handles graph changes while schedule mode is active. Removes residents
    /// whose home no longer exists and adds residents for new homes.
    /// </summary>
    private void UpdateForGraphChange(double timeOfDay)
    {
        // Remove residents whose home node is defunct
        for (int i = _residents.Count - 1; i >= 0; i--)
        {
            var r = _residents[i];
            // Emigrating residents own their own lifecycle: the drain marked them when their home
            // began being removed, and they are removed only on arrival at an entry/exit node (see
            // HandleArrival). Skip them here so a finalized home-drain doesn't instant-erase the
            // very household it is gracefully driving off the map.
            if (r.Emigrating) continue;
            int home = r.HomeNode;
            if (home >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[home].Position.X)
                || !_graph.Nodes[home].Flags.HasFlag(NodeFlags.Destination)
                || _graph.Nodes[home].PointOfInterest != POIType.Home)
            {
                if (IsDriving(r.Activity) && r.VehicleIndex >= 0)
                    RemoveVehicle(r);
                if (r.CurrentPOINode >= 0)
                    _poiRegistry.Vacate(r.CurrentPOINode);
                _residents.RemoveAt(i);
            }
        }

        // Re-index IDs after removal
        for (int i = 0; i < _residents.Count; i++)
            _residents[i].Id = i;

        // Fix vehicleToResident mapping
        _vehicleToResident.Clear();
        for (int i = 0; i < _residents.Count; i++)
        {
            if (IsDriving(_residents[i].Activity) && _residents[i].VehicleIndex >= 0)
            {
                _vehicleToResident[_residents[i].VehicleIndex] = i;
                _vehicles.ResidentId[_residents[i].VehicleIndex] = i;
            }
        }

        // Check for new home nodes that don't have residents yet
        var homeNodes = _poiRegistry.GetNodesOfType(POIType.Home);
        var existingHomes = new HashSet<int>();
        foreach (var r in _residents)
            existingHomes.Add(r.HomeNode);

        foreach (int homeNode in homeNodes)
        {
            if (existingHomes.Contains(homeNode)) continue;

            int capacity = _poiRegistry.GetCapacity(homeNode, POIType.Home);
            for (int slot = 0; slot < capacity; slot++)
            {
                int id = _residents.Count;
                var traits = DriverPersonalityGenerator.GenerateRandom();
                var (cr, cg, cb) = VehicleStore.RandomCarColor();

                var resident = new Resident
                {
                    Id = id,
                    HomeNode = homeNode,
                    // Created unemployed; the incremental job pass below finds them a nearby job if a
                    // slot is open. Schedule starts in the no-work form and is regenerated if hired.
                    WorkNode = -1,
                    Traits = traits,
                    ColorR = cr, ColorG = cg, ColorB = cb,
                    // Created off-map; drives in from an entry/exit node before being at home.
                    Activity = ResidentActivity.OffMap,
                    VehicleIndex = -1,
                    CurrentPOINode = -1,
                    ScheduleIndex = 0,
                };
                resident.Schedule = ScheduleGenerator.GenerateWeekday(traits.Archetype, -1);
                AdvanceSchedulePastTime(resident, timeOfDay);
                _residents.Add(resident);
            }
        }

        // Reconcile employment and fill open slots — but only when the home or workplace COUNT
        // actually changed. The graph version also bumps on pure road edits (speed limit, lane count,
        // restrictions) that leave POIs untouched; those need no reassignment, so skip the pass.
        int homeCount = homeNodes.Count;
        int workCount = _poiRegistry.GetNodesOfType(POIType.Work).Count;
        if (homeCount != _lastHomeCount || workCount != _lastWorkCount)
        {
            _lastHomeCount = homeCount;
            _lastWorkCount = workCount;
            RunIncrementalJobAssignment(timeOfDay);
        }
    }

    // ── Employment / job assignment ─────────────────────────────────────
    //
    // Jobs are a capacity-limited resource. A resident's WorkNode is the single source of truth for
    // employment (>=0 employed at that Work node, -1 unemployed); a workplace's employment headcount
    // is just the number of residents pointing at it, recomputed per pass (passes are event-driven and
    // rare). Only "workers" (ScheduleGenerator.WantsWork) ever hold a job; emigrants never do. Two
    // entry points drive it:
    //   • RunTotalJobAssignment       — midnight: every worker drops their job and re-competes (random order).
    //   • RunIncrementalJobAssignment — on any graph change: un-employ workers whose workplace vanished,
    //     then fill open slots from the currently-unemployed pool (new workers, displaced staff, freed slots).
    // Each seeker claims the OPEN slot nearest its home and that slot is counted immediately, so later
    // seekers in the (shuffled) order see it taken. ResolveDestination reads WorkNode live, so changing
    // WorkNode retargets future work trips automatically; only an employed↔unemployed flip needs a
    // schedule regen (to add/remove the work trips themselves).

    /// <summary>True if the resident is a worker (has work in their daily routine) and not leaving town.</summary>
    private bool IsEmployable(Resident r) =>
        !r.Emigrating && ScheduleGenerator.WantsWork(r.Traits.Archetype);

    /// <summary>True if a node index is currently a live Work POI (exists, destination, type Work).</summary>
    private bool IsLiveWorkNode(int node)
    {
        if (node < 0 || node >= _graph.Nodes.Count) return false;
        var n = _graph.Nodes[node];
        return !float.IsNaN(n.Position.X)
            && n.Flags.HasFlag(NodeFlags.Destination)
            && n.PointOfInterest == POIType.Work;
    }

    /// <summary>
    /// Rebuilds <see cref="_employmentCount"/> from residents' current WorkNode values. Any WorkNode
    /// that no longer references a live Work POI (its workplace was removed) is cleared to -1 — this
    /// is the reconcile that un-employs a deleted workplace's staff — and that resident is recorded in
    /// <see cref="_jobChanged"/> so its schedule regenerates. Emigrants are already at -1 and ignored.
    /// </summary>
    private void RebuildEmploymentCount()
    {
        _employmentCount.Clear();
        for (int i = 0; i < _residents.Count; i++)
        {
            var r = _residents[i];
            int w = r.WorkNode;
            if (w < 0) continue;
            if (!IsLiveWorkNode(w))
            {
                r.WorkNode = -1;       // workplace gone → unemployed
                _jobChanged.Add(i);
                continue;
            }
            _employmentCount.TryGetValue(w, out int c);
            _employmentCount[w] = c + 1;
        }
    }

    /// <summary>
    /// Finds the Work node nearest <paramref name="fromPos"/> whose employment headcount
    /// (<see cref="_employmentCount"/>) is below its capacity, or -1 if every workplace is full / none
    /// exist. Keyed on the employment headcount, NOT POIRegistry physical occupancy.
    /// </summary>
    private int FindNearestWorkWithCapacity(Vector2 fromPos)
    {
        var workNodes = _poiRegistry.GetNodesOfType(POIType.Work);
        int best = -1;
        float bestDistSq = float.MaxValue;
        for (int i = 0; i < workNodes.Count; i++)
        {
            int w = workNodes[i];
            _employmentCount.TryGetValue(w, out int c);
            if (c >= _poiRegistry.GetCapacity(w, POIType.Work)) continue;
            float distSq = Vector2.DistanceSquared(_graph.Nodes[w].Position, fromPos);
            if (distSq < bestDistSq) { bestDistSq = distSq; best = w; }
        }
        return best;
    }

    /// <summary>
    /// Assigns each seeker in <see cref="_jobSeekers"/> to the open Work slot nearest its HOME, in a
    /// randomized order (fairness), claiming each slot immediately so later seekers see reduced
    /// capacity. Seekers who find no open slot stay unemployed. Every resident whose WorkNode value
    /// changes is recorded in <see cref="_jobChanged"/>. Assumes <see cref="_employmentCount"/> already
    /// reflects the workers keeping their jobs.
    /// </summary>
    private void AssignSeekers()
    {
        // Fisher–Yates shuffle — the "randomized reassignment order for fairness".
        for (int i = _jobSeekers.Count - 1; i > 0; i--)
        {
            int j = Random.Shared.Next(i + 1);
            (_jobSeekers[i], _jobSeekers[j]) = (_jobSeekers[j], _jobSeekers[i]);
        }

        foreach (int rid in _jobSeekers)
        {
            var r = _residents[rid];
            int home = r.HomeNode;
            if (home < 0 || home >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[home].Position.X))
                continue;
            int w = FindNearestWorkWithCapacity(_graph.Nodes[home].Position);
            if (w < 0) continue; // no open slot anywhere — remains unemployed
            if (r.WorkNode != w)
            {
                r.WorkNode = w;
                _jobChanged.Add(rid);
            }
            _employmentCount.TryGetValue(w, out int c);
            _employmentCount[w] = c + 1; // claim the slot
        }
    }

    /// <summary>
    /// Total (midnight) reassignment: every employable worker drops their job and re-competes in
    /// random order. Does NOT regenerate schedules — the caller (<see cref="OnDayRollover"/>,
    /// <see cref="RebuildPopulation"/>) does that from the freshly assigned WorkNode.
    /// </summary>
    private void RunTotalJobAssignment()
    {
        _jobChanged.Clear();

        // Everyone drops their job (emigrants are already -1 and excluded by IsEmployable).
        foreach (var r in _residents)
            if (IsEmployable(r))
                r.WorkNode = -1;

        _employmentCount.Clear(); // all workplaces start empty for a fresh competition
        _jobSeekers.Clear();
        for (int i = 0; i < _residents.Count; i++)
            if (IsEmployable(_residents[i]))
                _jobSeekers.Add(i);

        AssignSeekers();
    }

    /// <summary>
    /// Incremental reassignment after a graph change: un-employ workers whose workplace vanished
    /// (reconcile), then fill open slots from the currently-unemployed worker pool (random order,
    /// nearest-first). Residents whose employment status flips get their schedule regenerated so work
    /// trips appear/disappear. Employed workers whose workplace still exists keep their jobs until the
    /// nightly total pass.
    /// </summary>
    /// <param name="timeOfDay">Current time of day in fractional hours (for schedule advancement).</param>
    private void RunIncrementalJobAssignment(double timeOfDay)
    {
        _jobChanged.Clear();

        // Reconcile dangling employment AND build the headcount of workers keeping their jobs.
        RebuildEmploymentCount();

        // Seekers = employable workers without a job (new arrivals, displaced staff, the chronically
        // unemployed who might now fill a freed slot).
        _jobSeekers.Clear();
        for (int i = 0; i < _residents.Count; i++)
        {
            var r = _residents[i];
            if (IsEmployable(r) && r.WorkNode < 0)
                _jobSeekers.Add(i);
        }

        AssignSeekers();

        // Regenerate schedules only for residents whose WorkNode changed (gained or lost work trips,
        // or moved workplaces). A currently-driving resident finishes its in-flight trip and picks up
        // the new schedule when it next goes dormant (same contract OnDayRollover relies on).
        foreach (int rid in _jobChanged)
        {
            var r = _residents[rid];
            r.Schedule = ScheduleGenerator.GenerateWeekday(r.Traits.Archetype, r.WorkNode);
            r.ScheduleIndex = 0;
            AdvanceSchedulePastTime(r, timeOfDay);
        }
    }

    // ── Graceful deletion (drains) ──────────────────────────────────────
    //
    // Deleting a node or road is routed through RequestDeleteNode/RequestDeleteEdge instead of
    // calling graph.Remove* directly. If the affected node has no population the removal is
    // immediate (today's behavior); otherwise it becomes a NodeDrain that drives the people out
    // before anything disappears. The editor (worker D) calls these entry points.

    /// <summary>
    /// Editor entry point for deleting a node. If the node has population — any resident dormant
    /// at it, or (if it is a Home) any resident who lives there — it begins a graceful drain that
    /// drives those people out before removing the node; otherwise the node is removed immediately.
    /// Idempotent: a node already being drained is left to its in-progress drain.
    /// </summary>
    /// <param name="node">Index of the node to delete.</param>
    public void RequestDeleteNode(int node)
    {
        if (node < 0 || node >= _graph.Nodes.Count) return;
        if (float.IsNaN(_graph.Nodes[node].Position.X)) return; // already defunct
        if (HasActiveDrain(node)) return;

        if (NodeHasPopulation(node))
            BeginDrain(node);
        else
            _graph.RemoveNode(node);
    }

    /// <summary>
    /// Editor entry point for deleting a road segment. Removes the edge and its reverse, but if
    /// doing so would orphan an endpoint that still has population, it instead begins a graceful
    /// drain on that endpoint (whose incident edge set includes the edge being deleted, holding it
    /// open so the people there have road to drive out on). An endpoint that would survive the
    /// removal (still has other roads) is left untouched. With no populated orphan, the edge and
    /// its reverse are removed immediately.
    /// </summary>
    /// <param name="edge">Index of the edge to delete.</param>
    public void RequestDeleteEdge(int edge)
    {
        if (edge < 0 || edge >= _graph.Edges.Count) return;
        var e = _graph.Edges[edge];
        if (e.FromNode < 0) return; // already defunct

        int reverse = _graph.FindReverseEdge(edge);
        int fromNode = e.FromNode;
        int toNode = e.ToNode;

        // Determine which endpoints removing this edge (+reverse) would orphan, and whether such an
        // orphan still has population to drive out first.
        bool beganDrain = false;
        if (WouldOrphan(fromNode, edge, reverse) && NodeHasPopulation(fromNode) && !HasActiveDrain(fromNode))
        {
            BeginDrain(fromNode);
            beganDrain = true;
        }
        if (toNode != fromNode
            && WouldOrphan(toNode, edge, reverse) && NodeHasPopulation(toNode) && !HasActiveDrain(toNode))
        {
            BeginDrain(toNode);
            beganDrain = true;
        }

        // If a drain holds the edge open, the drain's finalize step removes it once empty. If the
        // edge is already part of an existing drain's incident set, that drain owns it too. Only
        // remove it now when no drain is responsible for it.
        if (beganDrain || EdgeHeldByDrain(edge) || (reverse >= 0 && EdgeHeldByDrain(reverse)))
            return;

        _graph.RemoveEdge(edge);
        if (reverse >= 0)
            _graph.RemoveEdge(reverse);
    }

    /// <summary>
    /// True if removing <paramref name="edge"/> (and its <paramref name="reverse"/>) would leave
    /// <paramref name="node"/> with no incident edges at all — i.e. it would be orphaned and marked
    /// defunct. Counts current incident edges and subtracts the ones about to be removed.
    /// </summary>
    private bool WouldOrphan(int node, int edge, int reverse)
    {
        int incident = 0;
        for (int i = 0; i < _graph.Edges.Count; i++)
        {
            if (i == edge || i == reverse) continue;
            var ge = _graph.Edges[i];
            if (ge.FromNode < 0) continue;
            if (ge.FromNode == node || ge.ToNode == node) { incident++; break; }
        }
        return incident == 0;
    }

    /// <summary>
    /// True if a node currently has population that must be drained before deletion: any resident
    /// dormant at it, or — when the node is a Home POI — any resident whose home it is (including
    /// residents who are out driving or dormant elsewhere; the whole household emigrates).
    /// </summary>
    private bool NodeHasPopulation(int node)
    {
        bool isHome = node >= 0 && node < _graph.Nodes.Count
            && _graph.Nodes[node].PointOfInterest == POIType.Home;
        foreach (var r in _residents)
        {
            if (r.Activity == ResidentActivity.Dormant && r.CurrentPOINode == node) return true;
            if (isHome && r.HomeNode == node) return true;
        }
        return false;
    }

    /// <summary>True if a drain is already active for the given node.</summary>
    private bool HasActiveDrain(int node)
    {
        foreach (var d in _drains)
            if (d.Node == node) return true;
        return false;
    }

    /// <summary>True if some active drain holds the given edge in its incident-edge set.</summary>
    private bool EdgeHeldByDrain(int edge)
    {
        foreach (var d in _drains)
            if (d.IncidentEdges.Contains(edge)) return true;
        return false;
    }

    /// <summary>
    /// Begins a graceful drain on a node. Captures every edge incident to the node (so they stay
    /// open and routable while people drive out), then de-registers the node as a POI target so no
    /// new arrivals are routed to it. For a Home node, the whole household emigrates via an entry/exit
    /// node: EVERY resident with <c>HomeNode==node</c> is flagged <see cref="Resident.Emigrating"/> BEFORE the POI flags
    /// are cleared (so the flag change can't race a resident off this node first). A no-op if a
    /// drain for the node already exists.
    /// </summary>
    /// <param name="node">Index of the node to drain.</param>
    private void BeginDrain(int node)
    {
        if (HasActiveDrain(node)) return;
        if (node < 0 || node >= _graph.Nodes.Count) return;

        bool isHome = _graph.Nodes[node].PointOfInterest == POIType.Home;

        var incident = new List<int>();
        for (int i = 0; i < _graph.Edges.Count; i++)
        {
            var e = _graph.Edges[i];
            if (e.FromNode < 0) continue;
            if (e.FromNode == node || e.ToNode == node)
                incident.Add(i);
        }

        if (isHome)
        {
            // Flag the whole household BEFORE clearing flags: every resident living here leaves via
            // an entry/exit node, whether they're at home, out at another POI, or already driving.
            // An emigrant no longer holds a job, so free its workplace slot now; the incremental job
            // pass (triggered by the POI-flag clear below bumping the graph version) then fills the
            // freed slots from the unemployed pool.
            foreach (var r in _residents)
                if (r.HomeNode == node)
                {
                    r.Emigrating = true;
                    r.WorkNode = -1;
                }
        }

        // De-register the node as a destination so no new residents/arrivals target it. Clearing
        // the Destination flag removes it from the POI registry (which keys off that flag); also
        // clear its POI type so it is unambiguously no longer a Home/Work/etc. target.
        var flags = _graph.Nodes[node].Flags;
        if (flags.HasFlag(NodeFlags.Destination))
        {
            _graph.SetNodeFlags(node, flags & ~NodeFlags.Destination);
            _graph.SetNodePOIType(node, POIType.None);
        }

        _drains.Add(new NodeDrain
        {
            Node = node,
            IsHome = isHome,
            IncidentEdges = incident,
            Closing = false,
            FailStreak = 0,
        });
    }

    /// <summary>
    /// Per-tick drain processing, run after departures/through-traffic and sharing their spawn
    /// budget. Each drain advances through two phases: spawn its dormant residents out (toward an
    /// entry/exit node for a home, toward their next itinerary stop otherwise), then close its incident
    /// edges and remove them and the node once every incident edge has emptied. Also drives the
    /// emigration of out-of-house residents whose home is being drained. Index-safe and idempotent:
    /// a drain whose node/edges have vanished underneath it self-cancels.
    /// </summary>
    /// <param name="timeOfDay">Current time of day in fractional hours.</param>
    /// <param name="budget">Remaining per-tick spawn budget shared with departures/move-ins.</param>
    private void ProcessDrains(double timeOfDay, int budget)
    {
        if (_drains.Count == 0) return;

        // Out-of-house emigrants (driving, or dormant somewhere other than a draining node) are
        // retargeted/spawned toward an entry/exit node regardless of which drain owns them. This shares
        // the same spawn budget as the drains below.
        budget = ProcessEmigration(budget);

        for (int di = _drains.Count - 1; di >= 0; di--)
        {
            var drain = _drains[di];

            // Self-cancel if the node has already been removed underneath us.
            if (drain.Node < 0 || drain.Node >= _graph.Nodes.Count
                || float.IsNaN(_graph.Nodes[drain.Node].Position.X))
            {
                _drains.RemoveAt(di);
                continue;
            }

            if (!drain.Closing)
                ProcessDrainSpawnOut(drain, timeOfDay, ref budget);
            else
                ProcessDrainClosing(drain, di);
        }
    }

    /// <summary>
    /// Drain phase 1 — spawn out the residents dormant at the draining node. A home drives its
    /// people toward the nearest reachable entry/exit node (they are already flagged Emigrating); any
    /// other node sends them to their next itinerary stop (or home if their schedule is exhausted).
    /// Throttled by the shared spawn budget; spawn clearance spaces successive cars. A resident that
    /// repeatedly fails to spawn is removed after <see cref="DrainSpawnFailLimit"/> ticks so a
    /// blocked departure can't wedge the drain. When no dormant residents remain at the node, the
    /// drain advances to the closing phase. If a home has no exit-capable entry/exit node at all, its
    /// resident-at-node are removed immediately (graceful exit is impossible) and the drain closes.
    /// </summary>
    private void ProcessDrainSpawnOut(NodeDrain drain, double timeOfDay, ref int budget)
    {
        // No-exit fallback for a home: without an exit-capable entry/exit node, the household cannot
        // leave gracefully. Remove every dormant resident at the node (and clear their emigrating
        // intent so the population loop doesn't keep them) and proceed straight to closing.
        if (drain.IsHome && GatherExitNodes().Count == 0)
        {
            for (int i = _residents.Count - 1; i >= 0; i--)
            {
                var r = _residents[i];
                if (r.Activity == ResidentActivity.Dormant && r.CurrentPOINode == drain.Node)
                    RemoveResidentAt(i);
            }
            drain.Closing = true;
            return;
        }

        // Find the first dormant resident still at the node and try to spawn it out.
        int idx = -1;
        for (int i = 0; i < _residents.Count; i++)
        {
            var r = _residents[i];
            if (r.Activity == ResidentActivity.Dormant && r.CurrentPOINode == drain.Node) { idx = i; break; }
        }

        if (idx < 0)
        {
            // Nobody left at the node — close the roads.
            drain.FailStreak = 0;
            drain.Closing = true;
            return;
        }

        if (budget <= 0) return; // out of spawn budget this tick; retry next tick (no fail charged)

        var resident = _residents[idx];
        bool spawned;
        if (drain.IsHome || resident.Emigrating)
        {
            // Emigrant: drive to the nearest reachable entry/exit node and leave. This covers a
            // resident whose HOME was deleted (so it is emigrating) while it sits dormant at some
            // OTHER node that is now also being drained — it must still leave via an exit, not follow
            // its old itinerary (whose next stop may be its own draining home or, with WorkNode now
            // -1, an unresolvable work trip).
            spawned = TrySpawnEmigrantFromNode(resident, drain.Node);
        }
        else
        {
            // Normal resident: head to the next itinerary stop, or home if the schedule is done.
            int toNode = resident.ScheduleIndex < resident.Schedule.Length
                ? ResolveDestination(resident, resident.Schedule[resident.ScheduleIndex])
                : resident.HomeNode;
            spawned = SpawnResidentVehicle(resident, drain.Node, toNode,
                ResidentActivity.Driving, advanceSchedule: false, enterAtSpeed: false);
        }

        if (spawned)
        {
            budget--;
            drain.FailStreak = 0;
        }
        else
        {
            // Blocked / no path: count the failure and, past the limit, remove the stuck resident so
            // the drain can always make progress.
            drain.FailStreak++;
            if (drain.FailStreak >= DrainSpawnFailLimit)
            {
                RemoveResidentAt(idx);
                drain.FailStreak = 0;
            }
        }
    }

    /// <summary>
    /// Drain phase 2 — close the incident edges (and their reverses) to new entry/routing, let the
    /// cars already on them finish crossing, and once every incident edge is empty, reopen-then-
    /// remove the edges and the node, dropping the drain. <paramref name="drainIndex"/> is this
    /// drain's index in <see cref="_drains"/> for removal.
    /// </summary>
    private void ProcessDrainClosing(NodeDrain drain, int drainIndex)
    {
        // Mark every incident edge (and reverse) closed: excluded from new routes, entry blocked.
        // Idempotent — SetEdgeClosed only bumps Version on a real change.
        foreach (int e in drain.IncidentEdges)
        {
            if (e < 0 || e >= _graph.Edges.Count) continue;
            if (_graph.Edges[e].FromNode < 0) continue; // already gone
            _graph.SetEdgeClosed(e, true);
            int rev = _graph.FindReverseEdge(e);
            if (rev >= 0) _graph.SetEdgeClosed(rev, true);
        }

        // Wait until no vehicle is on any incident edge (counting reverses).
        foreach (int e in drain.IncidentEdges)
        {
            if (e < 0 || e >= _graph.Edges.Count) continue;
            if (_graph.Edges[e].FromNode < 0) continue;
            if (_vehicles.AnyVehicleOnEdge(e)) return;
            int rev = _graph.FindReverseEdge(e);
            if (rev >= 0 && _vehicles.AnyVehicleOnEdge(rev)) return;
        }

        // All clear — finalize. Reopen first so the transient closed flag never outlives the edge,
        // then remove the edges and the node.
        foreach (int e in drain.IncidentEdges)
        {
            if (e < 0 || e >= _graph.Edges.Count) continue;
            int rev = _graph.FindReverseEdge(e);
            _graph.SetEdgeClosed(e, false);
            if (rev >= 0) _graph.SetEdgeClosed(rev, false);
            if (_graph.Edges[e].FromNode >= 0)
                _graph.RemoveEdge(e);
            if (rev >= 0 && rev < _graph.Edges.Count && _graph.Edges[rev].FromNode >= 0)
                _graph.RemoveEdge(rev);
        }

        if (drain.Node >= 0 && drain.Node < _graph.Nodes.Count
            && !float.IsNaN(_graph.Nodes[drain.Node].Position.X))
            _graph.RemoveNode(drain.Node);

        _drains.RemoveAt(drainIndex);
    }

    /// <summary>
    /// Drives emigrating residents toward an entry/exit node: dormant emigrants NOT sitting at a
    /// draining node are spawned toward the nearest exit-capable entry/exit node (drawing from the
    /// shared spawn budget), and driving emigrants are retargeted so their current trip ends at the
    /// nearest one instead. Emigrants dormant AT a draining node are handled by that drain's spawn-out
    /// phase. With no exit-capable entry/exit node on the map, emigration is impossible — the residents
    /// are removed (the no-exit fallback). Returns the remaining spawn budget.
    /// </summary>
    private int ProcessEmigration(int budget)
    {
        bool hasExit = GatherExitNodes().Count > 0;

        for (int i = _residents.Count - 1; i >= 0; i--)
        {
            var r = _residents[i];
            if (!r.Emigrating) continue;

            if (!hasExit)
            {
                // No active entry/exit node exists: graceful emigration is impossible. Driving
                // emigrants finish their current trip (HandleArrival removes them); dormant ones are
                // removed now.
                if (r.Activity == ResidentActivity.Dormant)
                    RemoveResidentAt(i);
                continue;
            }

            if (r.Activity == ResidentActivity.Driving)
            {
                RetargetDrivingEmigrantToExit(r);
            }
            else if (r.Activity == ResidentActivity.Dormant)
            {
                // Skip emigrants still sitting at a draining node — their drain spawns them out, so
                // the drain's clearance spacing and budgeting stay authoritative for that node.
                if (HasActiveDrain(r.CurrentPOINode)) continue;
                if (budget <= 0) continue;
                if (TrySpawnEmigrantFromNode(r, r.CurrentPOINode))
                    budget--;
            }
            // OffMap/MovingIn emigrants: they have no home to return to once they arrive — the
            // arrival/abort paths and the next emigration pass route them onward.
        }

        return budget;
    }

    /// <summary>
    /// Spawns a (presumed dormant) resident from <paramref name="fromNode"/> toward the nearest
    /// reachable exit-capable entry/exit node, as a normal driving trip that will be removed on arrival
    /// (the resident is already flagged <see cref="Resident.Emigrating"/>). Returns true on a
    /// successful spawn. Tries nodes nearest-first and falls through on any unreachable/blocked one.
    /// </summary>
    private bool TrySpawnEmigrantFromNode(Resident resident, int fromNode)
    {
        if (fromNode < 0 || fromNode >= _graph.Nodes.Count
            || float.IsNaN(_graph.Nodes[fromNode].Position.X))
            return false;

        var exits = GatherExitNodes();
        if (exits.Count == 0) return false;

        var fromPos = _graph.Nodes[fromNode].Position;
        _moveInCandidates.Clear();
        foreach (int exit in exits)
        {
            if (exit == fromNode) continue;
            if (exit < 0 || exit >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[exit].Position.X)) continue;
            _moveInCandidates.Add((exit, Vector2.DistanceSquared(_graph.Nodes[exit].Position, fromPos)));
        }
        _moveInCandidates.Sort((a, b) => a.distSq.CompareTo(b.distSq));

        foreach (var (exit, _) in _moveInCandidates)
        {
            if (SpawnResidentVehicle(resident, fromNode, exit,
                    ResidentActivity.Driving, advanceSchedule: false, enterAtSpeed: false))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Retargets an already-driving emigrant so its current trip ends at the nearest reachable
    /// exit-capable entry/exit node. Rebuilds its path from the edge it is on (pathfind from that edge's
    /// ToNode to the node, then prepend the current edge) the way VehicleSpawner.RerouteFinished
    /// does. If the vehicle is already heading to / sitting at an entry/exit node (or no reroute is
    /// found), it is left as-is and arrival handling removes it. Idempotent: re-running while already
    /// routed to the nearest entry/exit node changes nothing meaningful.
    /// </summary>
    private void RetargetDrivingEmigrantToExit(Resident resident)
    {
        int vi = resident.VehicleIndex;
        if (vi < 0 || vi >= _vehicles.Count) return;

        int currentEdge = _vehicles.CurrentEdge[vi];
        if (currentEdge < 0 || currentEdge >= _graph.Edges.Count) return;
        var ce = _graph.Edges[currentEdge];
        if (ce.FromNode < 0) return; // on a defunct edge; GraphChangeHandler will deal with it
        int startNode = ce.ToNode;

        // Already at an entry/exit node at the end of the current edge: let arrival remove it.
        if (IsEntryExitNode(startNode)) return;

        // Gather the exit-capable entry/exit nodes (returns the shared _exitNodesBuffer); we only read
        // it within this method, before any call that could re-gather and clobber it.
        var exits = GatherExitNodes();
        if (exits.Count == 0) return;

        // If the vehicle is already routed to an entry/exit node, leave its path alone.
        int curDest = _vehicles.DestinationNode[vi];
        if (IsEntryExitNode(curDest)) return;

        var startPos = _graph.Nodes[startNode].Position;
        List<int>? best = null;
        float bestDistSq = float.MaxValue;
        int bestExit = -1;
        foreach (int exit in exits)
        {
            if (exit == startNode) continue;
            var p = Pathfinder.FindPath(_graph, startNode, exit, currentEdge);
            if (p == null || p.Count == 0) continue;
            float d = Vector2.DistanceSquared(_graph.Nodes[exit].Position, startPos);
            if (best == null || d < bestDistSq) { best = p; bestDistSq = d; bestExit = exit; }
        }

        if (best == null) return; // no reachable entry/exit node from here; arrival/next pass handles it

        best.Insert(0, currentEdge);
        _vehicles.Path[vi] = best;
        _vehicles.PathIndex[vi] = 0;
        _vehicles.DestinationNode[vi] = bestExit;
    }

    /// <summary>
    /// True if the node is an entry/exit destination marker (Destination flag and
    /// <see cref="POIType.EntryExit"/>). Does not check entry/exit capability — callers that need
    /// spawn origins or despawn targets gather them via <see cref="GatherEntryNodes"/> /
    /// <see cref="GatherExitNodes"/>.
    /// </summary>
    private bool IsEntryExitNode(int node)
        => node >= 0 && node < _graph.Nodes.Count
           && _graph.Nodes[node].Flags.HasFlag(NodeFlags.Destination)
           && _graph.Nodes[node].PointOfInterest == POIType.EntryExit;

    /// <summary>
    /// Fills and returns <see cref="_entryNodesBuffer"/> with the ENTRY-capable entry/exit nodes —
    /// those with at least one OUTGOING edge (a lane into town), so a vehicle can spawn there and
    /// drive in. One-way roads qualify: the upstream end of a one-way road is entry-capable. Source
    /// nodes come from <c>_poiRegistry.GetNodesOfType(POIType.EntryExit)</c>. Overwritten by the next
    /// call, so consume it before re-invoking.
    /// </summary>
    private List<int> GatherEntryNodes()
    {
        _entryNodesBuffer.Clear();
        foreach (int n in _poiRegistry.GetNodesOfType(POIType.EntryExit))
            if (_graph.GetOutgoingEdges(n).Count > 0)
                _entryNodesBuffer.Add(n);
        return _entryNodesBuffer;
    }

    /// <summary>
    /// Fills and returns <see cref="_exitNodesBuffer"/> with the EXIT-capable entry/exit nodes —
    /// those with at least one INCOMING edge (a lane out of town), so a vehicle can drive there and
    /// despawn. One-way roads qualify: the downstream end of a one-way road is exit-capable.
    /// Overwritten by the next call, so consume it before re-invoking.
    /// </summary>
    private List<int> GatherExitNodes()
    {
        _exitNodesBuffer.Clear();
        foreach (int n in _poiRegistry.GetNodesOfType(POIType.EntryExit))
            if (_graph.GetIncomingEdges(n).Count > 0)
                _exitNodesBuffer.Add(n);
        return _exitNodesBuffer;
    }

    /// <summary>
    /// True when a pass-through trip is possible — at least one entry-capable node and one
    /// exit-capable node exist that are not the single same node. Assumes <see cref="GatherEntryNodes"/>
    /// and <see cref="GatherExitNodes"/> have just populated their buffers.
    /// </summary>
    private bool CanPassThrough()
    {
        int e = _entryNodesBuffer.Count, x = _exitNodesBuffer.Count;
        if (e == 0 || x == 0) return false;
        // The only no-pass case is exactly one entry and one exit that are the same node.
        return !(e == 1 && x == 1 && _entryNodesBuffer[0] == _exitNodesBuffer[0]);
    }

    /// <summary>
    /// Removes the resident at <paramref name="index"/> from the population entirely: drops its
    /// vehicle (if driving), vacates its POI, then re-indexes resident IDs and rebuilds the
    /// vehicle↔resident maps so all three views stay consistent — the same fixup
    /// <see cref="UpdateForGraphChange"/> performs after a removal. Used by the drain/emigration
    /// fallbacks and by emigrant arrivals.
    /// </summary>
    private void RemoveResidentAt(int index)
    {
        if (index < 0 || index >= _residents.Count) return;
        var r = _residents[index];
        if (IsDriving(r.Activity) && r.VehicleIndex >= 0)
            RemoveVehicle(r);
        if (r.CurrentPOINode >= 0)
            _poiRegistry.Vacate(r.CurrentPOINode);
        _residents.RemoveAt(index);
        ReindexResidents();
    }

    /// <summary>
    /// Re-indexes resident IDs to their list positions and rebuilds the vehicle↔resident lookups
    /// (the <c>_vehicleToResident</c> dict and <c>VehicleStore.ResidentId</c> back-pointers) so all
    /// three stay consistent after a structural change to <see cref="_residents"/>. Mirrors the
    /// fixup block in <see cref="UpdateForGraphChange"/>.
    /// </summary>
    private void ReindexResidents()
    {
        for (int i = 0; i < _residents.Count; i++)
            _residents[i].Id = i;

        _vehicleToResident.Clear();
        for (int i = 0; i < _residents.Count; i++)
        {
            if (IsDriving(_residents[i].Activity) && _residents[i].VehicleIndex >= 0)
            {
                _vehicleToResident[_residents[i].VehicleIndex] = i;
                _vehicles.ResidentId[_residents[i].VehicleIndex] = i;
            }
        }
    }

    private void OnDayRollover(double timeOfDay)
    {
        // Total job reassignment: every worker drops their job and re-competes for the nearest open
        // slot in a fresh randomized order (fairness). Done before regenerating schedules so each
        // schedule reflects the new WorkNode.
        RunTotalJobAssignment();

        for (int i = 0; i < _residents.Count; i++)
        {
            var r = _residents[i];
            r.Schedule = ScheduleGenerator.GenerateWeekday(r.Traits.Archetype, r.WorkNode);
            r.ScheduleIndex = 0;
            // Advance past any already-passed departures (a no-op at ~00:00, but keeps this consistent
            // with every other schedule-(re)generation site and robust if rollover ever fires off-zero).
            AdvanceSchedulePastTime(r, timeOfDay);
            // Driving residents keep going; their new schedule starts after they arrive
        }
    }

    /// <summary>
    /// Checks all active resident vehicles for arrival at destination.
    /// On arrival: removes from VehicleStore, sets resident dormant at destination POI.
    /// </summary>
    private void ProcessArrivals(double timeOfDay)
    {
        // Iterate backwards since we may remove vehicles
        for (int i = _vehicles.Count - 1; i >= 0; i--)
        {
            int resId = _vehicles.ResidentId[i];
            if (resId < 0) continue;

            // Same arrival criteria as RerouteFinished
            if (_vehicles.CurrentArc[i] >= 0) continue;
            if (_vehicles.Speed[i] > 0.01f) continue;
            if (_vehicles.EdgeProgress[i] < 0.99f) continue;

            var path = _vehicles.Path[i];
            int pathIdx = _vehicles.PathIndex[i];
            if (path != null && pathIdx + 1 < path.Count) continue;

            // This vehicle has arrived
            HandleArrival(i, resId, timeOfDay);
        }
    }

    private void HandleArrival(int vehicleIndex, int residentId, double timeOfDay)
    {
        if (residentId < 0 || residentId >= _residents.Count) return;
        var resident = _residents[residentId];

        // Emigrants leave the simulation when they arrive (at an entry/exit node, or any node when no
        // entry/exit node is reachable): remove the vehicle AND the resident from the population — never re-park them
        // dormant. RemoveResidentAt drops the vehicle, vacates any POI, and re-indexes everything.
        if (resident.Emigrating)
        {
            RemoveResidentAt(residentId);
            return;
        }

        // Capture before RemoveVehicle: its VehicleRemoving fixup mutates Activity.
        bool wasMovingIn = resident.Activity == ResidentActivity.MovingIn;

        int destNode = _vehicles.DestinationNode[vehicleIndex];

        // Determine POI type at destination
        POIType destType = POIType.None;
        if (destNode >= 0 && destNode < _graph.Nodes.Count)
            destType = _graph.Nodes[destNode].PointOfInterest;

        // Try to occupy the destination
        if (destNode >= 0 && destType != POIType.None)
            _poiRegistry.TryOccupy(destNode, destType);

        // Remove vehicle from store
        RemoveVehicle(resident);

        // Set resident dormant at destination
        resident.Activity = ResidentActivity.Dormant;
        resident.CurrentPOINode = destNode;

        // A completed move-in (entry/exit node → home) consumed no schedule entry. Skip past any
        // departures whose time already passed during the drive in — except a still-in-effect Work
        // trip, which the advance stops ON so the resident departs for work late (see
        // AdvanceSchedulePastTime) — so the resident resumes its normal schedule if a trip is still
        // ahead today, or stays home until the next-day rollover.
        if (wasMovingIn)
            AdvanceSchedulePastTime(resident, timeOfDay);
    }

    /// <summary>
    /// Processes departures: checks dormant residents whose schedule entry departure
    /// time has arrived, and spawns them onto the road. Returns the number of vehicles spawned
    /// (so the caller can subtract them from the shared per-tick spawn budget).
    /// </summary>
    private int ProcessDepartures(double timeOfDay, int budget)
    {
        if (budget <= 0) return 0;
        int spawnsThisTick = 0;

        // Process departure queue first (residents who were deferred)
        while (_departureQueue.Count > 0 && spawnsThisTick < budget
            && _vehicles.Count < _maxActiveVehicles)
        {
            int resId = _departureQueue.Dequeue();
            if (resId >= _residents.Count) continue;
            var r = _residents[resId];
            if (r.Activity != ResidentActivity.Dormant) continue;
            if (TrySpawnResident(r))
                spawnsThisTick++;
        }

        // Check dormant residents for scheduled departures
        for (int i = 0; i < _residents.Count && spawnsThisTick < budget; i++)
        {
            var r = _residents[i];
            if (r.Activity != ResidentActivity.Dormant) continue;
            if (r.ScheduleIndex >= r.Schedule.Length) continue;

            float depTime = r.Schedule[r.ScheduleIndex].DepartureTime;
            if (timeOfDay < depTime) continue;

            // Time to depart
            if (_vehicles.Count >= _maxActiveVehicles)
            {
                _departureQueue.Enqueue(r.Id);
                continue;
            }

            if (TrySpawnResident(r))
                spawnsThisTick++;
        }

        return spawnsThisTick;
    }

    /// <summary>True for activities that own a VehicleStore slot (a scheduled trip or a move-in).</summary>
    private static bool IsDriving(ResidentActivity a)
        => a == ResidentActivity.Driving || a == ResidentActivity.MovingIn;

    /// <summary>
    /// Attempts to spawn a resident onto the road for their next scheduled trip, departing from
    /// their current POI node. Returns true if successful.
    /// </summary>
    private bool TrySpawnResident(Resident resident)
    {
        if (resident.ScheduleIndex >= resident.Schedule.Length) return false;

        int fromNode = resident.CurrentPOINode;
        var entry = resident.Schedule[resident.ScheduleIndex];
        int toNode = ResolveDestination(resident, entry);

        return SpawnResidentVehicle(resident, fromNode, toNode,
            ResidentActivity.Driving, advanceSchedule: true, enterAtSpeed: false);
    }

    /// <summary>
    /// Drives every off-map resident into the region from an entry-capable entry/exit node (their
    /// one-time first appearance), up to <paramref name="budget"/> spawns. Strict gate: does nothing
    /// unless at least one ENTRY-capable entry/exit node exists (one with an outgoing lane into town).
    /// Returns the number spawned.
    /// </summary>
    private int ProcessMoveIns(double timeOfDay, int budget)
    {
        if (budget <= 0) return 0;

        // Gate: move-ins need at least one entry-capable entry/exit node to enter from.
        GatherEntryNodes(); // fills _entryNodesBuffer, consumed by TryMoveIn below
        if (_entryNodesBuffer.Count == 0) return 0;

        // Bound ATTEMPTS (not just successes): when the entry is blocked (clearance), every
        // TryMoveIn does a pathfind and fails, so capping attempts keeps a congested entry from
        // churning O(residents) pathfinds per tick. Spacing is enforced by the spawn clearance.
        int spawned = 0, attempts = 0;
        for (int i = 0; i < _residents.Count && attempts < budget; i++)
        {
            var r = _residents[i];
            if (r.Activity != ResidentActivity.OffMap) continue;
            if (_vehicles.Count >= _maxActiveVehicles) break;
            attempts++;
            if (TryMoveIn(r))
                spawned++;
        }
        return spawned;
    }

    /// <summary>
    /// Spawns a resident's move-in trip from the entry-capable entry/exit node nearest their home that
    /// yields a valid path, driving them to their home node. Uses <see cref="_entryNodesBuffer"/>
    /// (already populated by <see cref="ProcessMoveIns"/>).
    /// </summary>
    private bool TryMoveIn(Resident resident)
    {
        int home = resident.HomeNode;
        if (home < 0 || home >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[home].Position.X))
            return false;
        var homePos = _graph.Nodes[home].Position;

        // Try entry-capable entry/exit nodes nearest-home first; fall through to the next on any
        // failure (no path / blocked / etc.).
        _moveInCandidates.Clear();
        foreach (int n in _entryNodesBuffer)
        {
            if (n < 0 || n >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[n].Position.X)) continue;
            _moveInCandidates.Add((n, Vector2.DistanceSquared(_graph.Nodes[n].Position, homePos)));
        }
        _moveInCandidates.Sort((a, b) => a.distSq.CompareTo(b.distSq));

        foreach (var (node, _) in _moveInCandidates)
        {
            if (SpawnResidentVehicle(resident, node, home,
                    ResidentActivity.MovingIn, advanceSchedule: false, enterAtSpeed: true))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Number of residents settled in the region (moved in — Dormant or Driving), used to scale
    /// through-traffic volume. Residents still off-map or mid-move-in don't count yet.
    /// </summary>
    private int HousedPopulation
    {
        get
        {
            int n = 0;
            foreach (var r in _residents)
                if (r.Activity == ResidentActivity.Dormant || r.Activity == ResidentActivity.Driving)
                    n++;
            return n;
        }
    }

    /// <summary>
    /// Realistic diurnal traffic multiplier (~0..1.1): a small overnight trickle, a morning peak
    /// near 08:00, a gentle midday plateau, and an evening peak near 17:30, tapering at night.
    /// </summary>
    private static float TimeOfDayTrafficFactor(double timeOfDay)
    {
        float h = (float)timeOfDay;
        float morning = MathF.Exp(-((h - 8f) * (h - 8f)) / (2f * 1.5f * 1.5f));
        float evening = MathF.Exp(-((h - 17.5f) * (h - 17.5f)) / (2f * 2f * 2f));
        float midday = 0.35f * MathF.Exp(-((h - 13f) * (h - 13f)) / (2f * 3f * 3f));
        const float overnight = 0.06f;
        return Math.Clamp(overnight + 0.95f * morning + evening + midday, 0f, 1.2f);
    }

    /// <summary>
    /// Spawns non-resident through-traffic entering at an entry-capable entry/exit node and bound for
    /// a distinct exit-capable one, at a rate of <see cref="ThroughTrafficBaseRate"/> × housed
    /// population × <see cref="TimeOfDayTrafficFactor"/>. Strict gate: requires a housed population and
    /// a valid pass-through pair (an entry-capable node and a different exit-capable node — e.g. the
    /// two ends of a one-way road).
    /// </summary>
    private void ProcessThroughTraffic(float simDt, double timeOfDay)
    {
        int housed = HousedPopulation;
        GatherEntryNodes(); // _entryNodesBuffer (spawn origins)
        GatherExitNodes();  // _exitNodesBuffer (despawn targets) — both consumed by TrySpawnThroughCar
        if (housed <= 0 || !CanPassThrough())
        {
            _throughSpawnAccumulator = 0f; // no backlog builds while inactive
            return;
        }

        float rate = ThroughTrafficBaseRate * housed * TimeOfDayTrafficFactor(timeOfDay); // cars/sec
        _throughSpawnAccumulator = Math.Min(_throughSpawnAccumulator + rate * simDt, ThroughAccumulatorCap);

        int spawned = 0;
        while (_throughSpawnAccumulator >= 1f && spawned < ThroughMaxPerTick
            && _vehicles.Count < _maxActiveVehicles)
        {
            _throughSpawnAccumulator -= 1f; // a blocked entry still spends the credit (backpressure)
            if (TrySpawnThroughCar())
                spawned++;
        }
    }

    /// <summary>
    /// Spawns one non-resident through-car entering at a random entry-capable entry/exit node and
    /// bound for a DIFFERENT random exit-capable entry/exit node, entering at speed in a random lane.
    /// The car carries no destination of its own — it targets the chosen exit node and despawns there
    /// (see VehicleSpawner.RerouteFinished). Returns true on success.
    /// </summary>
    private bool TrySpawnThroughCar()
    {
        // Buffers were freshly populated by ProcessThroughTraffic: spawn at an entry-capable node,
        // despawn at a distinct exit-capable node (the CanPassThrough gate guaranteed a valid pair).
        if (_entryNodesBuffer.Count == 0 || _exitNodesBuffer.Count == 0) return false;

        int spawn = _entryNodesBuffer[Random.Shared.Next(_entryNodesBuffer.Count)];

        // Pick an exit target distinct from the spawn node, scanning from a random offset. The only
        // way to find none is an exit set of exactly {spawn}, which the gate already excluded.
        int n = _exitNodesBuffer.Count;
        int start = Random.Shared.Next(n);
        int target = -1;
        for (int k = 0; k < n; k++)
        {
            int t = _exitNodesBuffer[(start + k) % n];
            if (t != spawn) { target = t; break; }
        }
        if (target < 0) return false;

        var traits = DriverPersonalityGenerator.GenerateRandom();
        var (cr, cg, cb) = VehicleStore.RandomCarColor();
        return CreateVehicle(spawn, target, traits, cr, cg, cb, residentId: -1, enterAtSpeed: true) >= 0;
    }

    /// <summary>
    /// Creates a vehicle for a resident travelling <paramref name="fromNode"/> → <paramref name="toNode"/>,
    /// copies traits/color, links the resident, and transitions it to <paramref name="newActivity"/>.
    /// Shared by scheduled departures (advanceSchedule: true, Driving) and entry/exit move-ins
    /// (advanceSchedule: false, MovingIn). Returns false (changing nothing) if the node is invalid,
    /// the destination is unreachable/identical, or the spawn point is blocked.
    /// </summary>
    private bool SpawnResidentVehicle(Resident resident, int fromNode, int toNode,
        ResidentActivity newActivity, bool advanceSchedule, bool enterAtSpeed)
    {
        int vi = CreateVehicle(fromNode, toNode, resident.Traits,
            resident.ColorR, resident.ColorG, resident.ColorB, resident.Id, enterAtSpeed);
        if (vi < 0) return false;

        // Update resident state
        resident.Activity = newActivity;
        resident.VehicleIndex = vi;
        if (advanceSchedule)
            resident.ScheduleIndex++;

        // Vacate the POI they're leaving
        if (resident.CurrentPOINode >= 0)
            _poiRegistry.Vacate(resident.CurrentPOINode);
        resident.CurrentPOINode = -1;

        // Track in reverse lookup
        _vehicleToResident[vi] = resident.Id;

        return true;
    }

    /// <summary>
    /// Low-level spawn: creates a vehicle travelling <paramref name="fromNode"/> → <paramref name="toNode"/>
    /// with the given traits/color and <paramref name="residentId"/> (-1 for non-resident through-traffic).
    /// <paramref name="enterAtSpeed"/> true → a random lane at the driver's free-flow desired speed
    /// (speedLimit × SpeedBias, the model SteeringController uses), as if arriving from off-map, with a
    /// spawn gap scaled to that speed; false → from rest in lane 0 on the path centerline. Returns the
    /// new vehicle index, or -1 (changing nothing) if the node is invalid, the destination is
    /// unreachable/identical, or the spawn point is blocked.
    /// </summary>
    private int CreateVehicle(int fromNode, int toNode, in DriverTraits t,
        byte colorR, byte colorG, byte colorB, int residentId, bool enterAtSpeed)
    {
        if (fromNode < 0 || fromNode >= _graph.Nodes.Count
            || float.IsNaN(_graph.Nodes[fromNode].Position.X))
            return -1;
        if (toNode < 0 || toNode == fromNode) return -1;

        // Must have a way out of the departure node and a route to the destination.
        if (_graph.GetOutgoingEdges(fromNode).Count == 0) return -1;
        var path = Pathfinder.FindPath(_graph, fromNode, toNode, -1);
        if (path == null || path.Count == 0) return -1;

        const float startT = 0.05f;
        int startEdge = path[0];
        var startEdgeData = _graph.Edges[startEdge];

        byte lane = 0;
        float entrySpeed = 0f;
        Vector2 spawnPos;
        if (enterAtSpeed)
        {
            lane = (byte)Random.Shared.Next(Math.Max(1, (int)startEdgeData.LaneCount));
            float baseSpeed = startEdgeData.SpeedLimit > 0f ? startEdgeData.SpeedLimit : SteeringController.TargetSpeed;
            entrySpeed = baseSpeed * t.SpeedBias;
            spawnPos = GeometryUtil.OffsetRight(_graph, startEdge, startT,
                GeometryUtil.LaneLateralOffset(_graph, startEdge, lane));
        }
        else
        {
            spawnPos = _graph.EvaluateBezier(startEdge, startT);
        }

        // Check spawn clearance at the actual spawn point. An at-speed entry needs a gap scaled to
        // its entry speed (a safe time headway) so cars don't appear nose-to-tail and brake into a
        // bunch; a from-rest departure only needs the small base clearance.
        float clearance = enterAtSpeed
            ? Math.Max(SpawnClearance, entrySpeed * MoveInHeadwaySeconds)
            : SpawnClearance;
        _spawnBlockedBuffer.Clear();
        _vehicleGrid.QueryFiltered(spawnPos.X, spawnPos.Y, clearance,
            _vehicles.PosX, _vehicles.PosY, _spawnBlockedBuffer);
        if (_spawnBlockedBuffer.Count > 0) return -1;

        // Spawn the vehicle
        var tangent = _graph.EvaluateBezierTangent(startEdge, startT);
        float heading = MathF.Atan2(tangent.Y, tangent.X);
        int vi = _vehicles.Add(spawnPos.X, spawnPos.Y, heading, startEdge);

        // Write personality and color
        _vehicles.Aggressiveness[vi] = t.Aggressiveness;
        _vehicles.SpeedBias[vi] = t.SpeedBias;
        _vehicles.ReactionTime[vi] = t.ReactionTime;
        _vehicles.SteeringSharpness[vi] = t.SteeringSharpness;
        _vehicles.BrakingComfort[vi] = t.BrakingComfort;
        _vehicles.LaneChangeBias[vi] = t.LaneChangeBias;
        _vehicles.PatienceTimer[vi] = t.PatienceTimer;
        _vehicles.PreferredVehicle[vi] = t.PreferredVehicle;
        _vehicles.Archetype[vi] = (byte)t.Archetype;
        _vehicles.ColorR[vi] = colorR;
        _vehicles.ColorG[vi] = colorG;
        _vehicles.ColorB[vi] = colorB;
        _vehicles.Path[vi] = path;
        _vehicles.PathIndex[vi] = 0;
        _vehicles.EdgeProgress[vi] = startT;
        _vehicles.CurrentLane[vi] = lane;
        _vehicles.TargetLane[vi] = lane;
        _vehicles.Speed[vi] = entrySpeed;
        _vehicles.DestinationNode[vi] = toNode;
        _vehicles.ResidentId[vi] = residentId;
        return vi;
    }

    /// <summary>
    /// Resolves a schedule entry destination to a specific node index.
    /// For Home/Work, uses the resident's assigned nodes. For other types,
    /// finds the nearest available POI.
    /// </summary>
    private int ResolveDestination(Resident resident, ScheduleEntry entry)
    {
        switch (entry.Destination)
        {
            case POIType.Home:
                return resident.HomeNode;
            case POIType.Work:
                return resident.WorkNode;
            default:
                // Shop/Leisure/etc. A NearestPOI trip (the midday errand) goes to the closest
                // available POI; every other trip picks a random available POI, so destinations
                // vary each time a resident goes out. Fallback to Leisure (same mode), then home.
                var fromPos = resident.CurrentPOINode >= 0 && resident.CurrentPOINode < _graph.Nodes.Count
                    ? _graph.Nodes[resident.CurrentPOINode].Position
                    : Vector2.Zero;
                int found = entry.NearestPOI
                    ? _poiRegistry.FindNearestAvailable(_graph, entry.Destination, fromPos)
                    : _poiRegistry.FindRandomAvailable(entry.Destination);
                if (found >= 0) return found;
                found = entry.NearestPOI
                    ? _poiRegistry.FindNearestAvailable(_graph, POIType.Leisure, fromPos)
                    : _poiRegistry.FindRandomAvailable(POIType.Leisure);
                if (found >= 0) return found;
                // Last resort: go home
                return resident.HomeNode;
        }
    }

    /// <summary>
    /// Removes a resident's vehicle from the store. All index fixup (this resident's
    /// links, the swapped-in vehicle's resident, editor selection) happens centrally
    /// via the store's VehicleRemoving event.
    /// </summary>
    private void RemoveVehicle(Resident resident)
    {
        int vi = resident.VehicleIndex;
        if (vi < 0 || vi >= _vehicles.Count) return;
        _vehicles.Remove(vi);
        resident.VehicleIndex = -1; // redundant with the event fixup; belt-and-braces
    }

    /// <summary>
    /// Centralized swap-and-pop fixup, invoked by VehicleStore for every removal
    /// regardless of call site. Detaches the removed vehicle's resident — an
    /// externally-removed mid-trip resident goes home dormant and departs again on its
    /// next schedule entry (POI occupancy is deliberately untouched; drift self-corrects
    /// on the next population rebuild, and HandleArrival overwrites Activity/CurrentPOINode
    /// with the true destination right after its own removal) — and redirects the
    /// swapped-in vehicle's resident links to its new index.
    /// </summary>
    private void OnVehicleRemoving(int removed, int swappedFrom)
    {
        int rid = _vehicles.ResidentId[removed];
        if (rid >= 0 && rid < _residents.Count)
        {
            var resident = _residents[rid];
            resident.VehicleIndex = -1;
            if (resident.Activity == ResidentActivity.Driving)
            {
                resident.Activity = ResidentActivity.Dormant;
                resident.CurrentPOINode = resident.HomeNode;
            }
            else if (resident.Activity == ResidentActivity.MovingIn)
            {
                // Move-in trip aborted (e.g. its road was deleted): send the resident back
                // off-map to retry the drive-in. (HandleArrival's own RemoveVehicle overwrites
                // this immediately afterward with the true dormant-at-home arrival state.)
                resident.Activity = ResidentActivity.OffMap;
                resident.CurrentPOINode = -1;
            }
        }
        _vehicleToResident.Remove(removed);

        if (swappedFrom >= 0)
        {
            // Pre-swap: the last slot's data is still intact and will move to `removed`
            int srid = _vehicles.ResidentId[swappedFrom];
            _vehicleToResident.Remove(swappedFrom);
            if (srid >= 0 && srid < _residents.Count)
            {
                _residents[srid].VehicleIndex = removed;
                _vehicleToResident[removed] = srid;
            }
        }
    }

    /// <summary>
    /// Invoked by VehicleStore.ClearAll when the world is replaced (new map / load):
    /// every vehicle index is void, so the population resets entirely. The next Update
    /// rebuilds a fresh population from the loaded map's POIs.
    /// </summary>
    private void OnVehiclesCleared()
    {
        _residents.Clear();
        _vehicleToResident.Clear();
        _departureQueue.Clear();
        _poiRegistry.ClearOccupancy();
        ScheduleModeEnabled = false;
    }

    /// <summary>
    /// Advances a resident's schedule index past entries whose departure time has already
    /// passed the current time of day — EXCEPT a missed Work trip that is still in effect
    /// (its departure has passed but the following entry's has not, i.e. the resident
    /// would be at work right now). For an employed resident who is not already at their
    /// workplace, the index stops ON that Work entry so the next departure pass sends
    /// them to work late, instead of silently skipping the workday: residents who are
    /// hired mid-morning, load in from a save, or move in after their departure time
    /// still go to work. Idempotent — re-running at the same time never moves the index
    /// further, so the move-in arrival re-advance is safe.
    /// </summary>
    private static void AdvanceSchedulePastTime(Resident resident, double timeOfDay)
    {
        while (resident.ScheduleIndex < resident.Schedule.Length
            && resident.Schedule[resident.ScheduleIndex].DepartureTime < timeOfDay)
        {
            var entry = resident.Schedule[resident.ScheduleIndex];
            // The stay has ended once the NEXT entry's departure is also due.
            bool stayEnded = resident.ScheduleIndex + 1 < resident.Schedule.Length
                && resident.Schedule[resident.ScheduleIndex + 1].DepartureTime < timeOfDay;
            if (!stayEnded && entry.Destination == POIType.Work
                && resident.WorkNode >= 0 && resident.CurrentPOINode != resident.WorkNode)
                return;
            resident.ScheduleIndex++;
        }
    }

    /// <summary>
    /// Debug-only watchdog asserting the vehicle↔resident index mappings are internally
    /// consistent in all three directions: the <c>_vehicleToResident</c> dict, the
    /// <c>VehicleStore.ResidentId</c> back-pointer, and each resident's <c>VehicleIndex</c>
    /// all agree. A violation means a vehicle removal bypassed the centralized fixup
    /// (<see cref="OnVehicleRemoving"/>). Compiled out of release builds.
    /// </summary>
    [Conditional("DEBUG")]
    public void ValidateMappings()
    {
        foreach (var kvp in _vehicleToResident)
        {
            int vi = kvp.Key, rid = kvp.Value;
            Debug.Assert(vi >= 0 && vi < _vehicles.Count,
                "ValidateMappings: dict holds an out-of-range vehicle index.");
            if (vi < 0 || vi >= _vehicles.Count) continue;
            Debug.Assert(_vehicles.ResidentId[vi] == rid,
                "ValidateMappings: vehicle's ResidentId disagrees with the dict.");
            Debug.Assert(rid >= 0 && rid < _residents.Count && _residents[rid].VehicleIndex == vi,
                "ValidateMappings: resident does not point back to its mapped vehicle.");
        }

        for (int i = 0; i < _vehicles.Count; i++)
        {
            int rid = _vehicles.ResidentId[i];
            if (rid < 0) continue;
            Debug.Assert(_vehicleToResident.TryGetValue(i, out int mapped) && mapped == rid,
                "ValidateMappings: vehicle with a ResidentId is missing/wrong in the dict.");
            Debug.Assert(rid < _residents.Count && _residents[rid].VehicleIndex == i,
                "ValidateMappings: vehicle's resident does not point back to it.");
        }

        foreach (var r in _residents)
        {
            int vi = r.VehicleIndex;
            if (vi < 0) continue;
            Debug.Assert(vi < _vehicles.Count && _vehicles.ResidentId[vi] == r.Id
                && _vehicleToResident.TryGetValue(vi, out int rid2) && rid2 == r.Id,
                "ValidateMappings: driving resident's VehicleIndex is stale or inconsistent.");
        }
    }

    /// <summary>
    /// Returns the resident associated with a vehicle index, or null if none.
    /// </summary>
    public Resident? GetResidentForVehicle(int vehicleIndex)
    {
        if (_vehicleToResident.TryGetValue(vehicleIndex, out int resId)
            && resId >= 0 && resId < _residents.Count)
            return _residents[resId];
        return null;
    }
}
