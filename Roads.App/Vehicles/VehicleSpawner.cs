using System.Numerics;
using Roads.App.Core;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Manages vehicle spawning, pathfinding, and rerouting. Manual spawns (V key) start on a
/// random active edge; scheduled traffic (residents, through-cars) enters via
/// PopulationManager at EntryExit nodes. Destination locations are determined by
/// NodeFlags.Destination on road graph nodes.
/// When one or more ACTIVE (two-way) EntryExit destination nodes exist, finished non-resident
/// cars enter "regionMode": instead of picking a new random destination they route to and despawn
/// at the nearest reachable entry/exit node (see <see cref="RerouteFinished"/>).
/// </summary>
public class VehicleSpawner
{
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly SpatialGrid _vehicleGrid;
    private readonly List<int> _spawnBlockedBuffer = new();
    private readonly List<int> _destNodeCache = new();
    private readonly List<int> _entryExitCache = new();
    private int _cacheGraphVersion = -1;

    public VehicleSpawner(RoadGraph graph, VehicleStore vehicles, SpatialGrid vehicleGrid)
    {
        _graph = graph;
        _vehicles = vehicles;
        _vehicleGrid = vehicleGrid;
    }

    /// <summary>Number of nodes with the Destination flag (updated on cache rebuild).</summary>
    public int DestinationNodeCount => EnsureCache()._destNodeCache.Count;

    private VehicleSpawner EnsureCache()
    {
        if (_cacheGraphVersion != _graph.Version)
        {
            _graph.GetNodesWithFlag(NodeFlags.Destination, _destNodeCache);
            // Despawn targets are EXIT-capable entry/exit nodes: a node a finished car can drive TO
            // and despawn at needs at least one INCOMING edge (a lane out of town). Outgoing is NOT
            // required — the downstream end of a one-way road is a valid exit. An outgoing-only node
            // (no incoming) is unreachable as a target and is correctly excluded, so a lone one-way
            // entry node can't flip regionMode on and make RerouteFinished delete free-roam cars
            // (FindPathToNearestEntryExit would return null); it falls back to FindRandomReroute.
            _entryExitCache.Clear();
            foreach (int n in _destNodeCache)
                if (_graph.Nodes[n].PointOfInterest == POIType.EntryExit
                    && _graph.GetIncomingEdges(n).Count > 0)
                    _entryExitCache.Add(n);
            _cacheGraphVersion = _graph.Version;
        }
        return this;
    }

    /// <summary>
    /// Spawns a vehicle with a pathfinding route, starting on a random active edge.
    /// </summary>
    public void SpawnRandom()
    {
        EnsureCache();

        if (_graph.ActiveEdgeCount == 0) return;

        // Pick a random active start edge
        int startEdge = -1;
        for (int a = 0; a < 100; a++)
        {
            int idx = SimRandom.Next(_graph.Edges.Count);
            if (_graph.Edges[idx].FromNode >= 0) { startEdge = idx; break; }
        }
        if (startEdge < 0) return;

        SpawnOnEdge(startEdge, 0f);
    }

    /// <summary>
    /// Checks all vehicles that are stopped at the end of their path and assigns a new
    /// random destination route. Removes the vehicle if no valid path can be found.
    /// </summary>
    private readonly HashSet<int> _reroutedEdgesThisTick = new();

    public void RerouteFinished()
    {
        EnsureCache();
        _reroutedEdgesThisTick.Clear();

        for (int i = _vehicles.Count - 1; i >= 0; i--)
        {
            // Skip resident vehicles — PopulationManager handles their arrivals
            if (_vehicles.ResidentId[i] >= 0) continue;

            if (_vehicles.CurrentArc[i] >= 0) continue;
            if (_vehicles.Speed[i] > 0.01f) continue;
            if (_vehicles.EdgeProgress[i] < 0.99f) continue;

            var path = _vehicles.Path[i];
            int pathIdx = _vehicles.PathIndex[i];
            if (path != null && pathIdx + 1 < path.Count) continue;

            int currentEdge = _vehicles.CurrentEdge[i];

            if (currentEdge < 0 || currentEdge >= _graph.Edges.Count
                || _graph.Edges[currentEdge].FromNode < 0)
            {
                _vehicles.Remove(i);
                continue;
            }

            var curEdge = _graph.Edges[currentEdge];
            int startNode = curEdge.ToNode;

            // Entry/exit model: when entry/exit nodes exist, a finished non-resident car has no
            // destination of its own — it heads for an entry/exit node and leaves. Despawn once it
            // has arrived at one (checked before the per-edge dedup so a queue of arrivals all clear
            // in one pass).
            bool regionMode = _entryExitCache.Count > 0;
            if (regionMode && IsEntryExitNode(startNode))
            {
                _vehicles.Remove(i);
                continue;
            }

            // Routing below pathfinds — dedup to one reroute per edge per tick to avoid thundering.
            if (!_reroutedEdgesThisTick.Add(currentEdge)) continue;

            int destNode;
            List<int>? newPath = regionMode
                ? FindPathToNearestEntryExit(startNode, currentEdge, out destNode)
                : FindRandomReroute(startNode, currentEdge, out destNode);

            if (newPath == null)
            {
                _vehicles.Remove(i);
                continue;
            }
            newPath.Insert(0, currentEdge);
            _vehicles.Path[i] = newPath;
            _vehicles.PathIndex[i] = 0;
            _vehicles.Speed[i] = 0f;
            _vehicles.DestinationNode[i] = destNode;
        }
    }

    /// <summary>True if the node is an entry/exit destination marker.</summary>
    private bool IsEntryExitNode(int node)
        => node >= 0 && node < _graph.Nodes.Count
           && _graph.Nodes[node].Flags.HasFlag(NodeFlags.Destination)
           && _graph.Nodes[node].PointOfInterest == POIType.EntryExit;

    /// <summary>
    /// Finds a path from <paramref name="startNode"/> (via <paramref name="currentEdge"/>) to the
    /// nearest reachable entry/exit node. Returns the path (excluding the current edge) and the
    /// chosen entry/exit node, or null if none is reachable.
    /// </summary>
    private List<int>? FindPathToNearestEntryExit(int startNode, int currentEdge, out int destNode)
    {
        destNode = -1;
        List<int>? best = null;
        float bestDistSq = float.MaxValue;
        var startPos = _graph.Nodes[startNode].Position;
        foreach (int exit in _entryExitCache)
        {
            if (exit == startNode) continue;
            var p = Pathfinder.FindPath(_graph, startNode, exit, currentEdge);
            if (p == null || p.Count == 0) continue;
            float d = Vector2.DistanceSquared(_graph.Nodes[exit].Position, startPos);
            if (best == null || d < bestDistSq) { best = p; bestDistSq = d; destNode = exit; }
        }
        return best;
    }

    /// <summary>
    /// Legacy free-roam reroute: picks a random Destination node reachable from
    /// <paramref name="startNode"/>. Returns the path and chosen node, or null if none found.
    /// </summary>
    private List<int>? FindRandomReroute(int startNode, int currentEdge, out int destNode)
    {
        destNode = -1;
        for (int attempt = 0; attempt < 50; attempt++)
        {
            if (_destNodeCache.Count == 0) break;
            int candidate = _destNodeCache[SimRandom.Next(_destNodeCache.Count)];
            if (candidate == startNode) continue;
            var p = Pathfinder.FindPath(_graph, startNode, candidate, currentEdge);
            if (p != null && p.Count > 0) { destNode = candidate; return p; }
        }
        return null;
    }

    /// <summary>
    /// Picks a random destination node and pathfinds from a given edge, trying both forward and
    /// reverse directions. Returns the best path, or null if no path found.
    /// </summary>
    public List<int>? FindNewPath(int currentEdge, float currentT)
    {
        return FindPathToRandomDestination(currentEdge, currentT,
            out _, out _, out _);
    }

    private List<int>? FindPathToRandomDestination(int currentEdge, float currentT,
        out int bestStartEdge, out float bestStartT, out int destNode)
    {
        EnsureCache();
        bestStartEdge = currentEdge;
        bestStartT = currentT;
        destNode = -1;

        var edge = _graph.Edges[currentEdge];
        int reverseEdge = _graph.FindReverseEdge(currentEdge);
        float reverseT = 1f - currentT;

        for (int attempt = 0; attempt < 50; attempt++)
        {
            if (_destNodeCache.Count == 0) break;
            int dn = _destNodeCache[SimRandom.Next(_destNodeCache.Count)];

            // Try forward direction
            List<int>? fwdFull = null;
            if (dn != edge.ToNode)
            {
                var fwdPath = Pathfinder.FindPath(_graph, edge.ToNode, dn, currentEdge);
                if (fwdPath != null && fwdPath.Count > 0)
                {
                    fwdFull = new List<int> { currentEdge };
                    fwdFull.AddRange(fwdPath);
                }
            }
            if (dn == edge.ToNode)
                fwdFull = new List<int> { currentEdge };

            // Try reverse direction
            List<int>? revFull = null;
            if (reverseEdge >= 0 && reverseEdge < _graph.Edges.Count
                && _graph.Edges[reverseEdge].FromNode >= 0)
            {
                var revEdge = _graph.Edges[reverseEdge];
                if (dn != revEdge.ToNode)
                {
                    var revPath = Pathfinder.FindPath(_graph, revEdge.ToNode, dn, reverseEdge);
                    if (revPath != null && revPath.Count > 0)
                    {
                        revFull = new List<int> { reverseEdge };
                        revFull.AddRange(revPath);
                    }
                }
                if (dn == revEdge.ToNode)
                    revFull = new List<int> { reverseEdge };
            }

            // Pick shorter path
            if (fwdFull != null && revFull != null)
            {
                destNode = dn;
                if (revFull.Count < fwdFull.Count)
                { bestStartEdge = reverseEdge; bestStartT = reverseT; return revFull; }
                else
                { bestStartEdge = currentEdge; bestStartT = currentT; return fwdFull; }
            }
            if (fwdFull != null)
            { destNode = dn; bestStartEdge = currentEdge; bestStartT = currentT; return fwdFull; }
            if (revFull != null)
            { destNode = dn; bestStartEdge = reverseEdge; bestStartT = reverseT; return revFull; }
        }

        return null;
    }

    private void SpawnOnEdge(int startEdge, float startT)
    {
        var bestPath = FindPathToRandomDestination(startEdge, startT,
            out int bestStartEdge, out float bestStartT, out int bestDestNode);

        if (bestPath == null) return;

        var pos = _graph.EvaluateBezier(bestStartEdge, bestStartT);

        if (IsSpawnBlocked(pos.X, pos.Y))
            return;

        var tangent = _graph.EvaluateBezierTangent(bestStartEdge, bestStartT);
        float heading = MathF.Atan2(tangent.Y, tangent.X);
        int vi = _vehicles.Add(pos.X, pos.Y, heading, bestStartEdge);
        var traits = DriverPersonalityGenerator.GenerateRandom();
        _vehicles.Aggressiveness[vi] = traits.Aggressiveness;
        _vehicles.SpeedBias[vi] = traits.SpeedBias;
        _vehicles.ReactionTime[vi] = traits.ReactionTime;
        _vehicles.SteeringSharpness[vi] = traits.SteeringSharpness;
        _vehicles.BrakingComfort[vi] = traits.BrakingComfort;
        _vehicles.LaneChangeBias[vi] = traits.LaneChangeBias;
        _vehicles.PatienceTimer[vi] = traits.PatienceTimer;
        _vehicles.PreferredVehicle[vi] = traits.PreferredVehicle;
        _vehicles.Archetype[vi] = (byte)traits.Archetype;
        _vehicles.Path[vi] = bestPath;
        _vehicles.PathIndex[vi] = 0;
        _vehicles.EdgeProgress[vi] = bestStartT;
        _vehicles.DestinationNode[vi] = bestDestNode;
    }

    private bool IsSpawnBlocked(float x, float y)
    {
        const float minSpawnClearance = 8f;
        _spawnBlockedBuffer.Clear();
        _vehicleGrid.QueryFiltered(x, y, minSpawnClearance, _vehicles.PosX, _vehicles.PosY, _spawnBlockedBuffer);
        return _spawnBlockedBuffer.Count > 0;
    }

    /// <summary>
    /// Bulk stress spawn: places up to <paramref name="count"/> vehicles at random positions
    /// on the graph, each routed to a random destination node.
    /// Bypasses node Destination flags and the 8 m IsSpawnBlocked clearance check
    /// so it works on generated grids (which have no flagged nodes) and achieves high density.
    /// Each vehicle gets one <see cref="Pathfinder.FindPath"/> call.
    /// </summary>
    /// <param name="count">Target number of vehicles to spawn.</param>
    /// <returns>The number of vehicles actually spawned (may be less if the graph is sparse).</returns>
    public int SpawnBulk(int count)
    {
        // Build a list of active node indices (nodes with valid positions).
        var activeNodes = new List<int>(_graph.Nodes.Count);
        for (int i = 0; i < _graph.Nodes.Count; i++)
        {
            if (!float.IsNaN(_graph.Nodes[i].Position.X))
                activeNodes.Add(i);
        }

        // Build a list of active edge indices.
        var activeEdges = new List<int>(_graph.Edges.Count);
        for (int i = 0; i < _graph.Edges.Count; i++)
        {
            if (_graph.Edges[i].FromNode >= 0)
                activeEdges.Add(i);
        }

        if (activeNodes.Count == 0 || activeEdges.Count == 0)
            return 0;

        int spawned = 0;
        const int maxAttemptsPerSlot = 5;
        // Total attempt budget: count * maxAttemptsPerSlot (on a connected grid, paths almost always exist).
        int totalAttemptBudget = count * maxAttemptsPerSlot;
        int totalAttempts = 0;

        while (spawned < count && totalAttempts < totalAttemptBudget)
        {
            // Pick a random active edge and start param.
            int startEdge = activeEdges[SimRandom.Next(activeEdges.Count)];
            float t = 0.1f + SimRandom.NextSingle() * 0.8f; // [0.1, 0.9]

            // Try up to maxAttemptsPerSlot different destination nodes for this slot.
            List<int>? full = null;
            int destNode = -1;
            int edgeToNode = _graph.Edges[startEdge].ToNode;

            for (int attempt = 0; attempt < maxAttemptsPerSlot; attempt++)
            {
                totalAttempts++;
                int candidateDest = activeNodes[SimRandom.Next(activeNodes.Count)];

                if (candidateDest == edgeToNode)
                {
                    // Already at destination — single-edge path.
                    full = new List<int> { startEdge };
                    destNode = candidateDest;
                    break;
                }

                var tail = Pathfinder.FindPath(_graph, edgeToNode, candidateDest, startEdge);
                if (tail != null && tail.Count > 0)
                {
                    full = new List<int> { startEdge };
                    full.AddRange(tail);
                    destNode = candidateDest;
                    break;
                }
            }

            if (full == null)
                continue;

            // Evaluate position and heading on the start edge.
            var pos = _graph.EvaluateBezier(startEdge, t);
            var tan = _graph.EvaluateBezierTangent(startEdge, t);
            float heading = MathF.Atan2(tan.Y, tan.X);

            int vi = _vehicles.Add(pos.X, pos.Y, heading, startEdge);

            // Apply random driver personality traits (same fields as SpawnOnEdge).
            var traits = DriverPersonalityGenerator.GenerateRandom();
            _vehicles.Aggressiveness[vi] = traits.Aggressiveness;
            _vehicles.SpeedBias[vi] = traits.SpeedBias;
            _vehicles.ReactionTime[vi] = traits.ReactionTime;
            _vehicles.SteeringSharpness[vi] = traits.SteeringSharpness;
            _vehicles.BrakingComfort[vi] = traits.BrakingComfort;
            _vehicles.LaneChangeBias[vi] = traits.LaneChangeBias;
            _vehicles.PatienceTimer[vi] = traits.PatienceTimer;
            _vehicles.PreferredVehicle[vi] = traits.PreferredVehicle;
            _vehicles.Archetype[vi] = (byte)traits.Archetype;

            _vehicles.Path[vi] = full;
            _vehicles.PathIndex[vi] = 0;
            _vehicles.EdgeProgress[vi] = t;
            _vehicles.DestinationNode[vi] = destNode;

            spawned++;
        }

        return spawned;
    }
}
