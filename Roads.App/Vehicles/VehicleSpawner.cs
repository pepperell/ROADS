using System.Numerics;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Manages vehicle spawning, pathfinding, and rerouting. Handles both manual spawns
/// (V key) and automatic spawning from spawn-flagged nodes on a timed interval.
/// Spawn and destination locations are determined by NodeFlags.Spawn and NodeFlags.Destination
/// on road graph nodes.
/// </summary>
public class VehicleSpawner
{
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly SpatialGrid _vehicleGrid;
    private readonly List<int> _spawnBlockedBuffer = new();
    private readonly List<int> _spawnNodeCache = new();
    private readonly List<int> _destNodeCache = new();
    private int _cacheGraphVersion = -1;
    private float _spawnTimer;

    /// <summary>When true, AutoSpawn is suppressed (PopulationManager handles spawning).</summary>
    public bool ScheduleModeActive { get; set; }

    public VehicleSpawner(RoadGraph graph, VehicleStore vehicles, SpatialGrid vehicleGrid)
    {
        _graph = graph;
        _vehicles = vehicles;
        _vehicleGrid = vehicleGrid;
    }

    /// <summary>Number of nodes with the Spawn flag (updated on cache rebuild).</summary>
    public int SpawnNodeCount => EnsureCache()._spawnNodeCache.Count;

    /// <summary>Number of nodes with the Destination flag (updated on cache rebuild).</summary>
    public int DestinationNodeCount => EnsureCache()._destNodeCache.Count;

    private VehicleSpawner EnsureCache()
    {
        if (_cacheGraphVersion != _graph.Version)
        {
            _graph.GetNodesWithFlag(NodeFlags.Spawn, _spawnNodeCache);
            _graph.GetNodesWithFlag(NodeFlags.Destination, _destNodeCache);
            _cacheGraphVersion = _graph.Version;
        }
        return this;
    }

    /// <summary>
    /// Spawns a vehicle with a pathfinding route. Uses a random spawn node if available,
    /// otherwise picks a random active edge as the start location.
    /// </summary>
    public void SpawnRandom()
    {
        EnsureCache();

        if (_spawnNodeCache.Count > 0)
        {
            int nodeIdx = _spawnNodeCache[Random.Shared.Next(_spawnNodeCache.Count)];
            SpawnFromNode(nodeIdx);
            return;
        }

        if (_graph.ActiveEdgeCount == 0) return;

        // Fallback: pick a random active start edge
        int startEdge = -1;
        for (int a = 0; a < 100; a++)
        {
            int idx = Random.Shared.Next(_graph.Edges.Count);
            if (_graph.Edges[idx].FromNode >= 0) { startEdge = idx; break; }
        }
        if (startEdge < 0) return;

        SpawnOnEdge(startEdge, 0f);
    }

    /// <summary>
    /// Spawns a vehicle from a spawn-flagged node, picking a random outgoing edge.
    /// </summary>
    private void SpawnFromNode(int nodeIndex)
    {
        var node = _graph.Nodes[nodeIndex];
        if (float.IsNaN(node.Position.X) || node.EdgeCount == 0) return;

        var outgoing = _graph.GetOutgoingEdges(nodeIndex);
        int edgeIdx = outgoing[Random.Shared.Next(outgoing.Count)];
        SpawnOnEdge(edgeIdx, 0.05f);
    }

    /// <summary>
    /// Auto-spawns vehicles from spawn-flagged nodes on a timed interval, respecting the vehicle cap.
    /// Called once per simulation tick from the simulation loop.
    /// </summary>
    public void AutoSpawn(float dt, int maxVehicles)
    {
        if (ScheduleModeActive) return;

        EnsureCache();
        if (_spawnNodeCache.Count == 0 || _destNodeCache.Count == 0 || _vehicles.Count >= maxVehicles)
            return;

        _spawnTimer += dt;
        const float spawnInterval = 2f;
        while (_spawnTimer >= spawnInterval)
        {
            _spawnTimer -= spawnInterval;
            if (_vehicles.Count >= maxVehicles) break;
            int nodeIdx = _spawnNodeCache[Random.Shared.Next(_spawnNodeCache.Count)];
            SpawnFromNode(nodeIdx);
        }
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
            if (!_reroutedEdgesThisTick.Add(currentEdge)) continue;

            if (currentEdge < 0 || currentEdge >= _graph.Edges.Count
                || _graph.Edges[currentEdge].FromNode < 0)
            {
                Console.Error.WriteLine($"[Reroute] Removing vehicle {i}: defunct edge {currentEdge}");
                _vehicles.Remove(i);
                continue;
            }

            var curEdge = _graph.Edges[currentEdge];
            int startNode = curEdge.ToNode;

            List<int>? newPath = null;
            for (int attempt = 0; attempt < 50 && newPath == null; attempt++)
            {
                if (_destNodeCache.Count == 0) break;
                int destNode = _destNodeCache[Random.Shared.Next(_destNodeCache.Count)];

                if (destNode == startNode) continue;

                var p = Pathfinder.FindPath(_graph, startNode, destNode, currentEdge);
                if (p != null && p.Count > 0)
                {
                    newPath = p;
                    _vehicles.DestinationNode[i] = destNode;
                }
            }

            if (newPath == null)
            {
                Console.Error.WriteLine($"[Reroute] Removing vehicle {i}: no path from node {startNode}");
                _vehicles.Remove(i);
                continue;
            }
            newPath.Insert(0, currentEdge);
            _vehicles.Path[i] = newPath;
            _vehicles.PathIndex[i] = 0;
            _vehicles.Speed[i] = 0f;
        }
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
            int dn = _destNodeCache[Random.Shared.Next(_destNodeCache.Count)];

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
    /// Bypasses node Spawn/Destination flags and the 8 m IsSpawnBlocked clearance check
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
            int startEdge = activeEdges[Random.Shared.Next(activeEdges.Count)];
            float t = 0.1f + Random.Shared.NextSingle() * 0.8f; // [0.1, 0.9]

            // Try up to maxAttemptsPerSlot different destination nodes for this slot.
            List<int>? full = null;
            int destNode = -1;
            int edgeToNode = _graph.Edges[startEdge].ToNode;

            for (int attempt = 0; attempt < maxAttemptsPerSlot; attempt++)
            {
                totalAttempts++;
                int candidateDest = activeNodes[Random.Shared.Next(activeNodes.Count)];

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
