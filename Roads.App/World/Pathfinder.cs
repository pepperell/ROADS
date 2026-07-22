using System.Diagnostics;
using System.Numerics;
using System.Threading;

namespace Roads.App.World;

/// <summary>
/// Edge-based A* shortest-path finder over the road graph. Uses travel time
/// (edge length / speed limit) as cost and Euclidean distance / max speed as an
/// admissible heuristic. States are edge indices (not nodes), so the turn matrix
/// is respected at every expansion — U-turns are never generated.
/// Reuses scratch buffers via ThreadStatic to avoid per-call allocations.
/// </summary>
public static class Pathfinder
{
    // ---------------------------------------------------------------------------
    // Lightweight pathfinding timing accumulators.
    // Counters are read-and-reset once per frame by the HUD via ReadPathfindStatsAndReset().
    // Interlocked provides cheap thread safety for a future off-thread pathfinder;
    // today the pathfinder runs single-threaded.
    // ---------------------------------------------------------------------------

    /// <summary>Accumulated Stopwatch ticks spent inside FindPath across all calls since last reset.</summary>
    private static long _totalTicks;
    /// <summary>Number of FindPath calls since last reset.</summary>
    private static int _callCount;

    /// <summary>
    /// Returns the total time spent in <see cref="FindPath"/> and the call count since the
    /// last call to this method, then resets both accumulators to zero.
    /// Counters are read-and-reset once per frame by the HUD.
    /// </summary>
    public static (double totalMs, int calls) ReadPathfindStatsAndReset()
    {
        long ticks = Interlocked.Exchange(ref _totalTicks, 0L);
        int calls = Interlocked.Exchange(ref _callCount, 0);
        double totalMs = ticks * 1000.0 / Stopwatch.Frequency;
        return (totalMs, calls);
    }

    /// <summary>Reusable g-score dictionary keyed by edge index, cleared per call.</summary>
    [ThreadStatic] private static Dictionary<int, float>? _gScore;
    /// <summary>Reusable came-from dictionary: edge index → previous edge index.</summary>
    [ThreadStatic] private static Dictionary<int, int>? _cameFrom;
    /// <summary>Reusable closed set of edge indices, cleared per call.</summary>
    [ThreadStatic] private static HashSet<int>? _closed;
    /// <summary>Reusable priority queue of edge indices, cleared per call.</summary>
    [ThreadStatic] private static PriorityQueue<int, float>? _open;

    /// <summary>
    /// Finds the shortest path from <paramref name="startNode"/> to <paramref name="endNode"/>
    /// using edge-based A*. Cost is travel time (edge length / speed limit). The turn matrix
    /// is checked at every expansion so generated paths never contain U-turns.
    /// </summary>
    /// <param name="graph">Road graph to search.</param>
    /// <param name="startNode">Index of the start node.</param>
    /// <param name="endNode">Index of the destination node.</param>
    /// <param name="incomingEdge">
    /// If >= 0, only seed edges reachable from this incoming edge (via turn matrix) are considered.
    /// Use this when the caller will prepend <paramref name="incomingEdge"/> to the result to
    /// prevent a U-turn at the path boundary.
    /// </param>
    /// <returns>List of edge indices forming the path, or <c>null</c> if no path exists.</returns>
    public static List<int>? FindPath(RoadGraph graph, int startNode, int endNode, int incomingEdge = -1)
    {
        long t0 = Stopwatch.GetTimestamp();

        if (startNode == endNode)
        {
            Interlocked.Add(ref _totalTicks, Stopwatch.GetTimestamp() - t0);
            Interlocked.Increment(ref _callCount);
            return new List<int>();
        }
        if (float.IsNaN(graph.Nodes[startNode].Position.X))
        {
            Interlocked.Add(ref _totalTicks, Stopwatch.GetTimestamp() - t0);
            Interlocked.Increment(ref _callCount);
            return null;
        }
        if (float.IsNaN(graph.Nodes[endNode].Position.X))
        {
            Interlocked.Add(ref _totalTicks, Stopwatch.GetTimestamp() - t0);
            Interlocked.Increment(ref _callCount);
            return null;
        }

        var endPos = graph.Nodes[endNode].Position;
        // Admissibility: the heuristic divides straight-line distance by the graph's
        // actual maximum speed limit (version-cached), so it never overestimates the
        // remaining travel time regardless of how fast the fastest road is.
        float invMaxSpeed = 1f / graph.MaxSpeedLimit;

        // Reuse scratch buffers
        var gScore = _gScore ??= new Dictionary<int, float>();
        var cameFrom = _cameFrom ??= new Dictionary<int, int>();
        var closed = _closed ??= new HashSet<int>();
        var open = _open ??= new PriorityQueue<int, float>();

        gScore.Clear();
        cameFrom.Clear();
        closed.Clear();
        open.Clear();

        // Seed: outgoing edges from startNode, optionally filtered by turn matrix
        var seedEdges = incomingEdge >= 0
            ? graph.GetAllowedTurns(startNode, incomingEdge)
            : GetOutgoingEdgeList(graph, startNode);

        foreach (int edgeIdx in seedEdges)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;
            // Skip edges closed by an in-progress drain so new routes avoid a closing edge
            // while cars already on it finish crossing (the edge is still traversable).
            if (graph.IsEdgeClosed(edgeIdx)) continue;
            int toNode = edge.ToNode;
            if (float.IsNaN(graph.Nodes[toNode].Position.X)) continue;

            float speed = edge.SpeedLimit;
            if (speed < 0.1f) speed = 0.1f;
            float cost = edge.Length / speed;

            gScore[edgeIdx] = cost;
            open.Enqueue(edgeIdx, cost + Heuristic(graph.Nodes[toNode].Position, endPos, invMaxSpeed));
        }

        while (open.Count > 0)
        {
            int currentEdge = open.Dequeue();

            var currentEdgeData = graph.Edges[currentEdge];
            int toNode = currentEdgeData.ToNode;

            if (toNode == endNode)
            {
                var result = ReconstructPath(cameFrom, currentEdge);
                Interlocked.Add(ref _totalTicks, Stopwatch.GetTimestamp() - t0);
                Interlocked.Increment(ref _callCount);
                return result;
            }

            if (!closed.Add(currentEdge))
                continue;

            float currentG = gScore[currentEdge];

            // Expand via turn matrix — only legal turns from this edge
            var allowedTurns = graph.GetAllowedTurns(toNode, currentEdge);
            foreach (int neighborEdge in allowedTurns)
            {
                var neighborData = graph.Edges[neighborEdge];
                if (neighborData.FromNode < 0) continue;
                // Skip edges closed by an in-progress drain (same rationale as the seed loop):
                // exclude a closing edge from new routes while its current traffic clears.
                if (graph.IsEdgeClosed(neighborEdge)) continue;

                int neighborToNode = neighborData.ToNode;
                if (float.IsNaN(graph.Nodes[neighborToNode].Position.X)) continue;

                float speed = neighborData.SpeedLimit;
                if (speed < 0.1f) speed = 0.1f;
                float edgeCost = neighborData.Length / speed;

                float tentativeG = currentG + edgeCost;

                if (!gScore.TryGetValue(neighborEdge, out float bestG) || tentativeG < bestG)
                {
                    gScore[neighborEdge] = tentativeG;
                    cameFrom[neighborEdge] = currentEdge;
                    float fScore = tentativeG + Heuristic(graph.Nodes[neighborToNode].Position, endPos, invMaxSpeed);
                    open.Enqueue(neighborEdge, fScore);
                }
            }
        }

        Interlocked.Add(ref _totalTicks, Stopwatch.GetTimestamp() - t0);
        Interlocked.Increment(ref _callCount);
        return null; // no path found
    }

    /// <summary>
    /// Given a set of reachable (outEdge, outLane, arcIdx) options from the vehicle's current lane,
    /// picks the one with the shortest path back to <paramref name="destinationNode"/>.
    /// Called when a vehicle is in the wrong lane at a stop line and must take an alternative route.
    /// </summary>
    /// <param name="graph">Road graph to search.</param>
    /// <param name="options">Reachable arcs from the vehicle's current (inEdge, inLane).</param>
    /// <param name="destinationNode">The vehicle's original destination node.</param>
    /// <returns>
    /// The best arc index, outgoing edge, and new path (starting from outEdge),
    /// or (-1, -1, null) if no route exists.
    /// </returns>
    public static (int arcIdx, int outEdge, List<int>? newPath) FindBestReroute(
        RoadGraph graph,
        List<(int outEdge, byte outLane, int arcIdx)> options,
        int destinationNode)
    {
        if (destinationNode < 0) return (-1, -1, null);

        int bestArc = -1;
        int bestOutEdge = -1;
        List<int>? bestPath = null;
        float bestCost = float.MaxValue;

        // Track which outEdges we've already evaluated to avoid duplicate pathfinds
        var evaluated = new HashSet<int>();

        foreach (var (outEdge, _, arcIdx) in options)
        {
            if (!evaluated.Add(outEdge)) continue;

            var outEdgeData = graph.Edges[outEdge];
            if (outEdgeData.FromNode < 0) continue;
            // Skip edges closed by an in-progress drain (mirrors FindPath's seed/neighbor
            // exclusion): rerouting a wrong-lane vehicle ONTO the closing edge would pin
            // it at the steering closed-edge gate's stop line for the whole drain. The
            // check sits before the direct-to-destination early return so that shortcut
            // can never pick a closed edge either.
            if (graph.IsEdgeClosed(outEdge)) continue;

            int fromNode = outEdgeData.ToNode;
            if (fromNode == destinationNode)
            {
                // outEdge leads directly to destination
                var path = new List<int> { outEdge };
                return (arcIdx, outEdge, path);
            }

            var tailPath = FindPath(graph, fromNode, destinationNode, outEdge);
            if (tailPath == null) continue;

            // Compute total cost: outEdge cost + tail path cost
            float speed = outEdgeData.SpeedLimit;
            if (speed < 0.1f) speed = 0.1f;
            float cost = outEdgeData.Length / speed;
            foreach (int edgeIdx in tailPath)
            {
                var e = graph.Edges[edgeIdx];
                float s = e.SpeedLimit;
                if (s < 0.1f) s = 0.1f;
                cost += e.Length / s;
            }

            if (cost < bestCost)
            {
                bestCost = cost;
                bestArc = arcIdx;
                bestOutEdge = outEdge;
                bestPath = new List<int>(tailPath.Count + 1) { outEdge };
                bestPath.AddRange(tailPath);
            }
        }

        return (bestArc, bestOutEdge, bestPath);
    }

    /// <summary>Admissible A* heuristic: Euclidean distance at the graph's current maximum
    /// speed limit (passed as its reciprocal, computed once per search from
    /// <see cref="RoadGraph.MaxSpeedLimit"/>) — never overestimates remaining travel time.</summary>
    private static float Heuristic(Vector2 from, Vector2 to, float invMaxSpeed)
    {
        return Vector2.Distance(from, to) * invMaxSpeed;
    }

    /// <summary>Traces back through the cameFrom map to build the edge-index path.</summary>
    private static List<int> ReconstructPath(Dictionary<int, int> cameFrom, int endEdge)
    {
        var path = new List<int>();
        int current = endEdge;

        while (true)
        {
            path.Add(current);
            if (!cameFrom.TryGetValue(current, out int prev))
                break;
            current = prev;
        }

        path.Reverse();
        return path;
    }

    /// <summary>Gets all outgoing edges from a node as a list (for seed when no incoming edge).</summary>
    private static List<int> GetOutgoingEdgeList(RoadGraph graph, int nodeIndex)
    {
        var result = new List<int>();
        foreach (int edge in graph.GetOutgoingEdges(nodeIndex))
            result.Add(edge);
        return result;
    }
}
