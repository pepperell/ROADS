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

    /// <summary>Residents queued for departure when active vehicle count is at capacity.</summary>
    private readonly Queue<int> _departureQueue = new();

    /// <summary>Maximum number of vehicles that can be active simultaneously.</summary>
    private int _maxActiveVehicles;

    /// <summary>Maximum spawns per simulation tick (prevents pop-in during rush hour).</summary>
    private const int MaxSpawnsPerTick = 5;

    /// <summary>Minimum clearance in meters around a spawn point.</summary>
    private const float SpawnClearance = 8f;

    private readonly List<int> _spawnBlockedBuffer = new();
    private int _lastDayNumber = -1;
    private int _poiGraphVersion = -1;

    /// <summary>Whether schedule mode is active (Home POIs exist).</summary>
    public bool ScheduleModeEnabled { get; private set; }

    /// <summary>Total number of residents in the town.</summary>
    public int TotalResidents => _residents.Count;

    /// <summary>Number of residents currently driving.</summary>
    public int ActiveDrivers => _vehicleToResident.Count;

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
        ProcessArrivals();

        // Process departures (dormant residents whose schedule time has come)
        ProcessDepartures(timeOfDay);
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

        var workNodes = _poiRegistry.GetNodesOfType(POIType.Work);

        int id = 0;
        for (int h = 0; h < homeNodes.Count; h++)
        {
            int homeNode = homeNodes[h];
            int capacity = _poiRegistry.GetCapacity(homeNode, POIType.Home);

            for (int slot = 0; slot < capacity; slot++)
            {
                int workNode = workNodes.Count > 0
                    ? workNodes[Random.Shared.Next(workNodes.Count)]
                    : -1;

                var traits = DriverPersonalityGenerator.GenerateRandom();
                var (cr, cg, cb) = VehicleStore.RandomCarColor();

                var resident = new Resident
                {
                    Id = id++,
                    HomeNode = homeNode,
                    WorkNode = workNode,
                    Traits = traits,
                    ColorR = cr, ColorG = cg, ColorB = cb,
                    Activity = ResidentActivity.Dormant,
                    VehicleIndex = -1,
                    CurrentPOINode = homeNode,
                    ScheduleIndex = 0,
                };

                resident.Schedule = ScheduleGenerator.GenerateWeekday(traits.Archetype, workNode);
                // Advance ScheduleIndex past any entries whose departure time has already passed
                AdvanceSchedulePastTime(resident, timeOfDay);

                _residents.Add(resident);
                _poiRegistry.TryOccupy(homeNode, POIType.Home);
            }
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
            if (res.Activity == ResidentActivity.Driving
                && res.VehicleIndex >= 0 && res.VehicleIndex < _vehicles.Count)
            {
                _vehicleToResident[res.VehicleIndex] = res.Id;
                _vehicles.ResidentId[res.VehicleIndex] = res.Id;
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

        ScheduleModeEnabled = true;
        _poiGraphVersion = _graph.Version;
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
            int home = r.HomeNode;
            if (home >= _graph.Nodes.Count || float.IsNaN(_graph.Nodes[home].Position.X)
                || !_graph.Nodes[home].Flags.HasFlag(NodeFlags.Destination)
                || _graph.Nodes[home].PointOfInterest != POIType.Home)
            {
                if (r.Activity == ResidentActivity.Driving && r.VehicleIndex >= 0)
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
            if (_residents[i].Activity == ResidentActivity.Driving && _residents[i].VehicleIndex >= 0)
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

        var workNodes = _poiRegistry.GetNodesOfType(POIType.Work);

        foreach (int homeNode in homeNodes)
        {
            if (existingHomes.Contains(homeNode)) continue;

            int capacity = _poiRegistry.GetCapacity(homeNode, POIType.Home);
            for (int slot = 0; slot < capacity; slot++)
            {
                int id = _residents.Count;
                int workNode = workNodes.Count > 0
                    ? workNodes[Random.Shared.Next(workNodes.Count)]
                    : -1;
                var traits = DriverPersonalityGenerator.GenerateRandom();
                var (cr, cg, cb) = VehicleStore.RandomCarColor();

                var resident = new Resident
                {
                    Id = id,
                    HomeNode = homeNode,
                    WorkNode = workNode,
                    Traits = traits,
                    ColorR = cr, ColorG = cg, ColorB = cb,
                    Activity = ResidentActivity.Dormant,
                    VehicleIndex = -1,
                    CurrentPOINode = homeNode,
                    ScheduleIndex = 0,
                };
                resident.Schedule = ScheduleGenerator.GenerateWeekday(traits.Archetype, workNode);
                AdvanceSchedulePastTime(resident, timeOfDay);
                _residents.Add(resident);
                _poiRegistry.TryOccupy(homeNode, POIType.Home);
            }
        }
    }

    private void OnDayRollover(double timeOfDay)
    {
        for (int i = 0; i < _residents.Count; i++)
        {
            var r = _residents[i];
            r.Schedule = ScheduleGenerator.GenerateWeekday(r.Traits.Archetype, r.WorkNode);
            r.ScheduleIndex = 0;
            // Driving residents keep going; their new schedule starts after they arrive
        }
    }

    /// <summary>
    /// Checks all active resident vehicles for arrival at destination.
    /// On arrival: removes from VehicleStore, sets resident dormant at destination POI.
    /// </summary>
    private void ProcessArrivals()
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
            HandleArrival(i, resId);
        }
    }

    private void HandleArrival(int vehicleIndex, int residentId)
    {
        if (residentId < 0 || residentId >= _residents.Count) return;
        var resident = _residents[residentId];

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
    }

    /// <summary>
    /// Processes departures: checks dormant residents whose schedule entry departure
    /// time has arrived, and spawns them onto the road.
    /// </summary>
    private void ProcessDepartures(double timeOfDay)
    {
        int spawnsThisTick = 0;

        // Process departure queue first (residents who were deferred)
        while (_departureQueue.Count > 0 && spawnsThisTick < MaxSpawnsPerTick
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
        for (int i = 0; i < _residents.Count && spawnsThisTick < MaxSpawnsPerTick; i++)
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
    }

    /// <summary>
    /// Attempts to spawn a resident onto the road from their current POI node.
    /// Returns true if successful.
    /// </summary>
    private bool TrySpawnResident(Resident resident)
    {
        if (resident.ScheduleIndex >= resident.Schedule.Length) return false;

        int fromNode = resident.CurrentPOINode;
        if (fromNode < 0 || fromNode >= _graph.Nodes.Count
            || float.IsNaN(_graph.Nodes[fromNode].Position.X))
            return false;

        var entry = resident.Schedule[resident.ScheduleIndex];
        int toNode = ResolveDestination(resident, entry);
        if (toNode < 0 || toNode == fromNode) return false;

        // Find an outgoing edge from the departure node
        var outgoing = _graph.GetOutgoingEdges(fromNode);
        if (outgoing.Count == 0) return false;

        // Try pathfinding
        int startEdge = outgoing[Random.Shared.Next(outgoing.Count)];
        var path = Pathfinder.FindPath(_graph, fromNode, toNode, -1);
        if (path == null || path.Count == 0) return false;

        // Prepend the start edge if not already included
        if (path[0] != startEdge)
        {
            // The pathfinder returns edges from the start node; use as-is
        }

        // Check spawn clearance
        var spawnPos = _graph.EvaluateBezier(path[0], 0.05f);
        _spawnBlockedBuffer.Clear();
        _vehicleGrid.QueryFiltered(spawnPos.X, spawnPos.Y, SpawnClearance,
            _vehicles.PosX, _vehicles.PosY, _spawnBlockedBuffer);
        if (_spawnBlockedBuffer.Count > 0) return false;

        // Spawn the vehicle
        var tangent = _graph.EvaluateBezierTangent(path[0], 0.05f);
        float heading = MathF.Atan2(tangent.Y, tangent.X);
        int vi = _vehicles.Add(spawnPos.X, spawnPos.Y, heading, path[0]);

        // Write resident personality and color
        var t = resident.Traits;
        _vehicles.Aggressiveness[vi] = t.Aggressiveness;
        _vehicles.SpeedBias[vi] = t.SpeedBias;
        _vehicles.ReactionTime[vi] = t.ReactionTime;
        _vehicles.SteeringSharpness[vi] = t.SteeringSharpness;
        _vehicles.BrakingComfort[vi] = t.BrakingComfort;
        _vehicles.LaneChangeBias[vi] = t.LaneChangeBias;
        _vehicles.PatienceTimer[vi] = t.PatienceTimer;
        _vehicles.PreferredVehicle[vi] = t.PreferredVehicle;
        _vehicles.Archetype[vi] = (byte)t.Archetype;
        _vehicles.ColorR[vi] = resident.ColorR;
        _vehicles.ColorG[vi] = resident.ColorG;
        _vehicles.ColorB[vi] = resident.ColorB;
        _vehicles.Path[vi] = path;
        _vehicles.PathIndex[vi] = 0;
        _vehicles.EdgeProgress[vi] = 0.05f;
        _vehicles.DestinationNode[vi] = toNode;
        _vehicles.ResidentId[vi] = resident.Id;

        // Update resident state
        resident.Activity = ResidentActivity.Driving;
        resident.VehicleIndex = vi;
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
                // Find nearest available POI of the requested type
                var fromPos = resident.CurrentPOINode >= 0 && resident.CurrentPOINode < _graph.Nodes.Count
                    ? _graph.Nodes[resident.CurrentPOINode].Position
                    : Vector2.Zero;
                int found = _poiRegistry.FindNearestAvailable(_graph, entry.Destination, fromPos);
                if (found >= 0) return found;
                // Fallback: try Leisure, then any destination
                found = _poiRegistry.FindNearestAvailable(_graph, POIType.Leisure, fromPos);
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
    /// Advances a resident's schedule index past entries whose departure time
    /// has already passed the current time of day.
    /// </summary>
    private static void AdvanceSchedulePastTime(Resident resident, double timeOfDay)
    {
        while (resident.ScheduleIndex < resident.Schedule.Length
            && resident.Schedule[resident.ScheduleIndex].DepartureTime < timeOfDay)
        {
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
