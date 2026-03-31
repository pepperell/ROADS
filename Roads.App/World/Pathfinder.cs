using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// A* shortest-path finder over the road graph. Uses travel time (edge length / speed limit)
/// as cost and Euclidean distance / max speed as an admissible heuristic.
/// Reuses scratch buffers via ThreadStatic to avoid per-call allocations.
/// </summary>
public static class Pathfinder
{
    /// <summary>Reusable g-score dictionary, cleared per call.</summary>
    [ThreadStatic] private static Dictionary<int, float>? _gScore;
    /// <summary>Reusable came-from dictionary, cleared per call.</summary>
    [ThreadStatic] private static Dictionary<int, (int prevNode, int edgeIndex)>? _cameFrom;
    /// <summary>Reusable closed set, cleared per call.</summary>
    [ThreadStatic] private static HashSet<int>? _closed;
    /// <summary>Reusable priority queue, cleared per call.</summary>
    [ThreadStatic] private static PriorityQueue<int, float>? _open;

    /// <summary>
    /// Finds the shortest path from <paramref name="startNode"/> to <paramref name="endNode"/> using A*.
    /// Cost is travel time (edge length / speed limit).
    /// </summary>
    /// <param name="graph">Road graph to search.</param>
    /// <param name="startNode">Index of the start node.</param>
    /// <param name="endNode">Index of the destination node.</param>
    /// <returns>List of edge indices forming the path, or <c>null</c> if no path exists.</returns>
    public static List<int>? FindPath(RoadGraph graph, int startNode, int endNode)
    {
        if (startNode == endNode) return new List<int>();
        if (float.IsNaN(graph.Nodes[startNode].Position.X)) return null;
        if (float.IsNaN(graph.Nodes[endNode].Position.X)) return null;

        var endPos = graph.Nodes[endNode].Position;

        // Reuse scratch buffers
        var gScore = _gScore ??= new Dictionary<int, float>();
        var cameFrom = _cameFrom ??= new Dictionary<int, (int, int)>();
        var closed = _closed ??= new HashSet<int>();
        var open = _open ??= new PriorityQueue<int, float>();

        gScore.Clear();
        cameFrom.Clear();
        closed.Clear();
        open.Clear();

        gScore[startNode] = 0f;
        open.Enqueue(startNode, Heuristic(graph.Nodes[startNode].Position, endPos));

        while (open.Count > 0)
        {
            int current = open.Dequeue();

            if (current == endNode)
                return ReconstructPath(cameFrom, endNode);

            if (!closed.Add(current))
                continue;

            float currentG = gScore[current];

            foreach (int edgeIdx in graph.GetOutgoingEdges(current))
            {
                var edge = graph.Edges[edgeIdx];
                if (edge.FromNode < 0) continue; // defunct

                int neighbor = edge.ToNode;
                if (closed.Contains(neighbor)) continue;
                if (float.IsNaN(graph.Nodes[neighbor].Position.X)) continue; // defunct node

                // Cost = travel time: length / speed limit
                float speed = edge.SpeedLimit;
                if (speed < 0.1f) speed = 0.1f;
                float edgeCost = edge.Length / speed;

                float tentativeG = currentG + edgeCost;

                if (!gScore.TryGetValue(neighbor, out float bestG) || tentativeG < bestG)
                {
                    gScore[neighbor] = tentativeG;
                    cameFrom[neighbor] = (current, edgeIdx);
                    float fScore = tentativeG + Heuristic(graph.Nodes[neighbor].Position, endPos);
                    open.Enqueue(neighbor, fScore);
                }
            }
        }

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

            int fromNode = outEdgeData.ToNode;
            if (fromNode == destinationNode)
            {
                // outEdge leads directly to destination
                var path = new List<int> { outEdge };
                return (arcIdx, outEdge, path);
            }

            var tailPath = FindPath(graph, fromNode, destinationNode);
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

    /// <summary>Admissible A* heuristic: Euclidean distance / max plausible speed (30 m/s).</summary>
    private static float Heuristic(Vector2 from, Vector2 to)
    {
        return Vector2.Distance(from, to) / 30f;
    }

    /// <summary>Traces back through the cameFrom map to build the edge-index path from start to end.</summary>
    private static List<int> ReconstructPath(Dictionary<int, (int prevNode, int edgeIndex)> cameFrom, int endNode)
    {
        var path = new List<int>();
        int current = endNode;

        while (cameFrom.TryGetValue(current, out var prev))
        {
            path.Add(prev.edgeIndex);
            current = prev.prevNode;
        }

        path.Reverse();
        return path;
    }
}
