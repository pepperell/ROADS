using System.Numerics;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Manages vehicle spawning, pathfinding, and rerouting. Handles both manual spawns
/// (V key) and automatic spawning from spawn points on a timed interval.
/// </summary>
public class VehicleSpawner
{
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly List<SpawnPoint> _spawnPoints;
    private readonly List<DestinationPoint> _destinations;
    private readonly SpatialGrid _vehicleGrid;
    private readonly List<int> _spawnBlockedBuffer = new();
    private float _spawnTimer;

    public VehicleSpawner(RoadGraph graph, VehicleStore vehicles,
        List<SpawnPoint> spawnPoints, List<DestinationPoint> destinations,
        SpatialGrid vehicleGrid)
    {
        _graph = graph;
        _vehicles = vehicles;
        _spawnPoints = spawnPoints;
        _destinations = destinations;
        _vehicleGrid = vehicleGrid;
    }

    /// <summary>
    /// Spawns a vehicle with a pathfinding route. Uses a random spawn point if available,
    /// otherwise picks a random active edge as the start location.
    /// </summary>
    public void SpawnRandom()
    {
        // If spawn points exist, use one randomly
        if (_spawnPoints.Count > 0)
        {
            var sp = _spawnPoints[Random.Shared.Next(_spawnPoints.Count)];
            SpawnFromPoint(sp);
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
    /// Spawns a vehicle from a specific spawn point, validating that its edge is still active.
    /// </summary>
    public void SpawnFromPoint(SpawnPoint sp)
    {
        if (sp.EdgeIndex >= _graph.Edges.Count || _graph.Edges[sp.EdgeIndex].FromNode < 0)
            return;

        SpawnOnEdge(sp.EdgeIndex, sp.EdgeT);
    }

    /// <summary>
    /// Auto-spawns vehicles from spawn points on a timed interval, respecting the vehicle cap.
    /// Called once per simulation tick from the simulation loop.
    /// </summary>
    public void AutoSpawn(float dt, int maxVehicles)
    {
        if (_spawnPoints.Count == 0 || _destinations.Count == 0 || _vehicles.Count >= maxVehicles)
            return;

        _spawnTimer += dt;
        const float spawnInterval = 2f; // one vehicle every 2 seconds per spawn point
        while (_spawnTimer >= spawnInterval)
        {
            _spawnTimer -= spawnInterval;
            if (_vehicles.Count >= maxVehicles) break;
            var sp = _spawnPoints[Random.Shared.Next(_spawnPoints.Count)];
            SpawnFromPoint(sp);
        }
    }

    /// <summary>
    /// Checks all vehicles that are stopped at the end of their path and assigns a new
    /// random destination route. Removes the vehicle if no valid path can be found.
    /// </summary>
    /// <summary>Edges that had a vehicle rerouted this tick (prevents overlapping reroutes).</summary>
    private readonly HashSet<int> _reroutedEdgesThisTick = new();

    public void RerouteFinished()
    {
        _reroutedEdgesThisTick.Clear();

        for (int i = _vehicles.Count - 1; i >= 0; i--)
        {
            if (_vehicles.CurrentArc[i] >= 0) continue; // traversing intersection arc
            if (_vehicles.Speed[i] > 0.01f) continue;
            if (_vehicles.EdgeProgress[i] < 0.99f) continue;

            var path = _vehicles.Path[i];
            int pathIdx = _vehicles.PathIndex[i];
            // Only reroute if at end of path (or no path)
            if (path != null && pathIdx + 1 < path.Count) continue;

            // Only reroute one vehicle per edge per tick to prevent physical overlap
            int currentEdge = _vehicles.CurrentEdge[i];
            if (!_reroutedEdgesThisTick.Add(currentEdge)) continue;

            // Find new destination from current position
            if (currentEdge < 0 || currentEdge >= _graph.Edges.Count
                || _graph.Edges[currentEdge].FromNode < 0)
            {
                Console.Error.WriteLine($"[Reroute] Removing vehicle {i}: defunct edge {currentEdge}");
                _vehicles.Remove(i);
                continue;
            }

            // Vehicle is at the end of its edge — pathfind from the ToNode directly
            // instead of using FindNewPath (which prepends the current edge)
            var curEdge = _graph.Edges[currentEdge];
            int startNode = curEdge.ToNode;

            List<int>? newPath = null;
            for (int attempt = 0; attempt < 50 && newPath == null; attempt++)
            {
                if (_destinations.Count == 0) break;
                var dest = _destinations[Random.Shared.Next(_destinations.Count)];
                if (dest.EdgeIndex < 0 || dest.EdgeIndex >= _graph.Edges.Count
                    || _graph.Edges[dest.EdgeIndex].FromNode < 0)
                    continue;

                var destEdge = _graph.Edges[dest.EdgeIndex];
                var destPos = _graph.EvaluateBezier(dest.EdgeIndex, dest.EdgeT);
                var fromPos = _graph.Nodes[destEdge.FromNode].Position;
                var toPos = _graph.Nodes[destEdge.ToNode].Position;
                int destNode = Vector2.DistanceSquared(destPos, fromPos) < Vector2.DistanceSquared(destPos, toPos)
                    ? destEdge.FromNode : destEdge.ToNode;

                if (destNode == startNode) continue; // already there

                var p = Pathfinder.FindPath(_graph, startNode, destNode);
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
            // Prepend the current edge so the vehicle transitions through an
            // intersection arc to the new path's first edge naturally.
            newPath.Insert(0, currentEdge);
            _vehicles.Path[i] = newPath;
            _vehicles.PathIndex[i] = 0;
            _vehicles.Speed[i] = 0f;
        }
    }

    /// <summary>
    /// Picks a random destination and pathfinds from a given edge, trying both forward and
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
        bestStartEdge = currentEdge;
        bestStartT = currentT;
        destNode = -1;

        var edge = _graph.Edges[currentEdge];
        int reverseEdge = _graph.FindReverseEdge(currentEdge);
        float reverseT = 1f - currentT;

        for (int attempt = 0; attempt < 50; attempt++)
        {
            if (_destinations.Count == 0) break;
            var dest = _destinations[Random.Shared.Next(_destinations.Count)];
            if (dest.EdgeIndex < 0 || dest.EdgeIndex >= _graph.Edges.Count
                || _graph.Edges[dest.EdgeIndex].FromNode < 0)
                continue;

            var destEdge = _graph.Edges[dest.EdgeIndex];
            var destPos = _graph.EvaluateBezier(dest.EdgeIndex, dest.EdgeT);
            var fromPos = _graph.Nodes[destEdge.FromNode].Position;
            var toPos = _graph.Nodes[destEdge.ToNode].Position;
            int dn = Vector2.DistanceSquared(destPos, fromPos) < Vector2.DistanceSquared(destPos, toPos)
                ? destEdge.FromNode : destEdge.ToNode;

            // Try forward direction
            List<int>? fwdFull = null;
            if (dn != edge.ToNode)
            {
                var fwdPath = Pathfinder.FindPath(_graph, edge.ToNode, dn);
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
                    var revPath = Pathfinder.FindPath(_graph, revEdge.ToNode, dn);
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
}
