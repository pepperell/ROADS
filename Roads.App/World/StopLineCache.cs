using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Caches the parametric t position of stop lines at both ends of each edge.
/// Stop lines are set back from intersections based on the width and angle of crossing roads,
/// so vehicles stop before blocking cross-traffic. Rebuilds automatically when the graph changes.
/// </summary>
public class StopLineCache
{
    /// <summary>Assumed lane width in meters for stop line offset calculations.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;
    /// <summary>Minimum angle (~5 degrees) between roads to count as a crossing; near-parallel roads are skipped.</summary>
    private const float MinAngle = 0.087f;
    /// <summary>Maximum fraction of edge length a stop line can be set back from the node.</summary>
    private const float MaxDistanceFraction = 0.4f;

    /// <summary>Per-edge parametric t of the stop line near the ToNode end.</summary>
    private float[] _stopTAtToNode = Array.Empty<float>();
    /// <summary>Per-edge parametric t of the stop line near the FromNode end.</summary>
    private float[] _stopTAtFromNode = Array.Empty<float>();
    /// <summary>Graph version when the cache was last rebuilt.</summary>
    private int _cachedVersion = -1;

    /// <summary>
    /// Gets the parametric t of the stop line near the ToNode end of the edge.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge.</param>
    /// <returns>Parametric t value (0–1) where the stop line sits; 1.0 if no setback is needed.</returns>
    public float GetStopTAtToNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _stopTAtToNode.Length) return 1f;
        return _stopTAtToNode[edgeIndex];
    }

    /// <summary>
    /// Gets the parametric t of the stop line near the FromNode end of the edge.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge.</param>
    /// <returns>Parametric t value (0–1) where the stop line sits; 0.0 if no setback is needed.</returns>
    public float GetStopTAtFromNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _stopTAtFromNode.Length) return 0f;
        return _stopTAtFromNode[edgeIndex];
    }

    /// <summary>
    /// Rebuilds the stop line cache if the graph has changed since the last rebuild.
    /// Must be called before querying stop-t values each frame.
    /// </summary>
    /// <param name="graph">Road graph to compute stop lines from.</param>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version) return;
        Rebuild(graph);
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Recomputes stop line t-values for all edges in the graph.
    /// </summary>
    /// <param name="graph">Road graph to compute stop lines from.</param>
    private void Rebuild(RoadGraph graph)
    {
        int edgeCount = graph.Edges.Count;
        if (_stopTAtToNode.Length < edgeCount)
        {
            _stopTAtToNode = new float[edgeCount];
            _stopTAtFromNode = new float[edgeCount];
        }

        for (int i = 0; i < edgeCount; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0)
            {
                _stopTAtToNode[i] = 1f;
                _stopTAtFromNode[i] = 0f;
                continue;
            }

            _stopTAtToNode[i] = ComputeStopT(graph, i, atToNode: true);
            _stopTAtFromNode[i] = ComputeStopT(graph, i, atToNode: false);
        }
    }

    /// <summary>
    /// Computes the stop line t-value for one end of an edge by examining all crossing roads
    /// at the endpoint node. The stop line is set back far enough that a vehicle stopped there
    /// doesn't block perpendicular traffic.
    /// </summary>
    /// <param name="graph">Road graph.</param>
    /// <param name="edgeIndex">Edge to compute the stop line for.</param>
    /// <param name="atToNode">If <c>true</c>, compute for the ToNode end; otherwise for the FromNode end.</param>
    /// <returns>Parametric t value along the edge where the stop line should be drawn.</returns>
    private float ComputeStopT(RoadGraph graph, int edgeIndex, bool atToNode)
    {
        var edge = graph.Edges[edgeIndex];
        int nodeIndex = atToNode ? edge.ToNode : edge.FromNode;

        // Get this edge's tangent direction at the node
        // At ToNode (t=1), tangent points into the node; at FromNode (t=0), tangent points away from node
        var tangent = atToNode
            ? graph.EvaluateBezierTangent(edgeIndex, 1f)
            : graph.EvaluateBezierTangent(edgeIndex, 0f);
        float tangentLen = tangent.Length();
        if (tangentLen < 0.001f) return atToNode ? 1f : 0f;
        var dir = tangent / tangentLen;

        float maxDistance = 0f;

        // Check all outgoing edges from this node
        foreach (int otherEdge in graph.GetOutgoingEdges(nodeIndex))
        {
            float dist = ComputeCrossingDistance(graph, edgeIndex, otherEdge, nodeIndex, dir, isOutgoing: true);
            if (dist > maxDistance) maxDistance = dist;
        }

        // Check all incoming edges to this node
        foreach (int otherEdge in graph.GetIncomingEdges(nodeIndex))
        {
            float dist = ComputeCrossingDistance(graph, edgeIndex, otherEdge, nodeIndex, dir, isOutgoing: false);
            if (dist > maxDistance) maxDistance = dist;
        }

        // Clamp distance
        float maxAllowed = edge.Length * MaxDistanceFraction;
        if (maxDistance > maxAllowed) maxDistance = maxAllowed;

        if (maxDistance < 0.01f) return atToNode ? 1f : 0f;

        // Convert distance to t value
        if (atToNode)
        {
            float stopT = 1f - maxDistance / edge.Length;
            return MathF.Max(0.5f, MathF.Min(stopT, 0.999f));
        }
        else
        {
            float stopT = maxDistance / edge.Length;
            return MathF.Max(0.001f, MathF.Min(stopT, 0.5f));
        }
    }

    /// <summary>
    /// Computes the distance a stop line must be set back due to a single crossing road.
    /// Uses the angle between the two roads and the width of the crossing road.
    /// </summary>
    /// <param name="graph">Road graph.</param>
    /// <param name="edgeIndex">The edge whose stop line is being computed.</param>
    /// <param name="otherEdge">A crossing road edge at the shared node.</param>
    /// <param name="nodeIndex">The shared intersection node.</param>
    /// <param name="dir">Normalized tangent direction of <paramref name="edgeIndex"/> at the node.</param>
    /// <param name="isOutgoing">Whether <paramref name="otherEdge"/> is outgoing from <paramref name="nodeIndex"/>.</param>
    /// <returns>Required setback distance in meters, or 0 if the roads are near-parallel or share the same corridor.</returns>
    private float ComputeCrossingDistance(RoadGraph graph, int edgeIndex, int otherEdge, int nodeIndex, Vector2 dir, bool isOutgoing)
    {
        if (otherEdge == edgeIndex) return 0f;

        var edge = graph.Edges[edgeIndex];
        var other = graph.Edges[otherEdge];

        // Skip reverse edge (same road, opposite direction)
        if (other.FromNode == edge.ToNode && other.ToNode == edge.FromNode) return 0f;

        // Get other edge's tangent at this node
        Vector2 otherTangent;
        if (isOutgoing)
        {
            // Outgoing from node: other.FromNode == nodeIndex, tangent at t=0
            otherTangent = graph.EvaluateBezierTangent(otherEdge, 0f);
        }
        else
        {
            // Incoming to node: other.ToNode == nodeIndex, tangent at t=1
            otherTangent = graph.EvaluateBezierTangent(otherEdge, 1f);
        }

        float otherLen = otherTangent.Length();
        if (otherLen < 0.001f) return 0f;
        var otherDir = otherTangent / otherLen;

        // Compute angle between the two road directions
        float dot = MathF.Abs(dir.X * otherDir.X + dir.Y * otherDir.Y);
        dot = MathF.Min(dot, 1f); // clamp for numerical safety
        float angle = MathF.Acos(dot);

        if (angle < MinAngle) return 0f; // near-parallel, skip

        float sinAngle = MathF.Sin(angle);
        if (sinAngle < 0.01f) return 0f;

        // Distance = half-width of other road / sin(angle)
        float halfWidthOther = other.LaneCount * LaneWidth;
        return halfWidthOther / sinAngle;
    }
}
