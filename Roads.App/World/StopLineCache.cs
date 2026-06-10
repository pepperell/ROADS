using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Caches the parametric t position of stop lines at both ends of each edge.
/// Stop lines are set back from intersections based on the width and angle of crossing roads,
/// so vehicles stop before blocking cross-traffic. Computes per-side (left/right of tangent)
/// trim values so that boundary lines at acute-angle Y-intersections are asymmetric — pushed
/// back further on the sharp-angle side. Rebuilds automatically when the graph changes.
/// </summary>
public class StopLineCache
{
    /// <summary>Assumed lane width in meters for stop line offset calculations.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;
    /// <summary>Minimum angle (~15 degrees) between roads to count as a crossing; near-parallel/anti-parallel roads are skipped.</summary>
    private const float MinAngle = 0.262f;
    /// <summary>
    /// Minimum effective sin(angle) for crossing distance calculation.
    /// Caps crossing distance at halfWidth / MinSinAngle (~2× halfWidth at 30°),
    /// preventing unrealistically large intersections at acute angles.
    /// </summary>
    private const float MinSinAngle = 0.5f;
    /// <summary>Maximum fraction of edge length a stop line can be set back from the node.</summary>
    private const float MaxDistanceFraction = 0.4f;

    /// <summary>Per-edge parametric t of the stop line near the ToNode end (max of both sides; used for vehicles).</summary>
    private float[] _stopTAtToNode = Array.Empty<float>();
    /// <summary>Per-edge parametric t of the stop line near the FromNode end (max of both sides; used for vehicles).</summary>
    private float[] _stopTAtFromNode = Array.Empty<float>();

    /// <summary>Per-edge left-boundary (negative offset) trim t at the ToNode end.</summary>
    private float[] _leftTrimAtToNode = Array.Empty<float>();
    /// <summary>Per-edge right-boundary (positive offset) trim t at the ToNode end.</summary>
    private float[] _rightTrimAtToNode = Array.Empty<float>();
    /// <summary>Per-edge left-boundary trim t at the FromNode end.</summary>
    private float[] _leftTrimAtFromNode = Array.Empty<float>();
    /// <summary>Per-edge right-boundary trim t at the FromNode end.</summary>
    private float[] _rightTrimAtFromNode = Array.Empty<float>();

    /// <summary>Graph version when the cache was last rebuilt.</summary>
    private int _cachedVersion = -1;

    /// <summary>
    /// Gets the parametric t of the stop line near the ToNode end (max of both sides).
    /// Used for vehicle stopping, signal placement, and intersection arcs.
    /// </summary>
    public float GetStopTAtToNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _stopTAtToNode.Length) return 1f;
        return _stopTAtToNode[edgeIndex];
    }

    /// <summary>
    /// Gets the parametric t of the stop line near the FromNode end (max of both sides).
    /// </summary>
    public float GetStopTAtFromNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _stopTAtFromNode.Length) return 0f;
        return _stopTAtFromNode[edgeIndex];
    }

    /// <summary>Gets the left-boundary (negative offset) trim t at the ToNode end.</summary>
    public float GetLeftTrimAtToNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _leftTrimAtToNode.Length) return 1f;
        return _leftTrimAtToNode[edgeIndex];
    }

    /// <summary>Gets the right-boundary (positive offset) trim t at the ToNode end.</summary>
    public float GetRightTrimAtToNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _rightTrimAtToNode.Length) return 1f;
        return _rightTrimAtToNode[edgeIndex];
    }

    /// <summary>Gets the left-boundary trim t at the FromNode end.</summary>
    public float GetLeftTrimAtFromNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _leftTrimAtFromNode.Length) return 0f;
        return _leftTrimAtFromNode[edgeIndex];
    }

    /// <summary>Gets the right-boundary trim t at the FromNode end.</summary>
    public float GetRightTrimAtFromNode(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _rightTrimAtFromNode.Length) return 0f;
        return _rightTrimAtFromNode[edgeIndex];
    }

    /// <summary>
    /// Rebuilds the stop line cache if the graph has changed since the last rebuild.
    /// Must be called before querying stop-t values each frame.
    /// </summary>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version) return;
        Rebuild(graph);
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Recomputes stop line t-values for all edges in the graph.
    /// </summary>
    private void Rebuild(RoadGraph graph)
    {
        int edgeCount = graph.Edges.Count;
        if (_stopTAtToNode.Length < edgeCount)
        {
            _stopTAtToNode = new float[edgeCount];
            _stopTAtFromNode = new float[edgeCount];
            _leftTrimAtToNode = new float[edgeCount];
            _rightTrimAtToNode = new float[edgeCount];
            _leftTrimAtFromNode = new float[edgeCount];
            _rightTrimAtFromNode = new float[edgeCount];
        }

        for (int i = 0; i < edgeCount; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0)
            {
                _stopTAtToNode[i] = 1f;
                _stopTAtFromNode[i] = 0f;
                _leftTrimAtToNode[i] = 1f;
                _rightTrimAtToNode[i] = 1f;
                _leftTrimAtFromNode[i] = 0f;
                _rightTrimAtFromNode[i] = 0f;
                continue;
            }

            ComputeAllStopTs(graph, i, atToNode: true,
                out _stopTAtToNode[i], out _leftTrimAtToNode[i], out _rightTrimAtToNode[i]);
            ComputeAllStopTs(graph, i, atToNode: false,
                out _stopTAtFromNode[i], out _leftTrimAtFromNode[i], out _rightTrimAtFromNode[i]);
        }
    }

    /// <summary>
    /// Computes the overall stop-T and per-side boundary trim t-values for one end of an edge.
    /// The overall stop-T (for vehicles) uses the maximum setback of both sides.
    /// Per-side trims account for the road's own width when the crossing road is at an acute angle,
    /// so boundaries are pushed back further on the sharp-angle side.
    /// </summary>
    private void ComputeAllStopTs(RoadGraph graph, int edgeIndex, bool atToNode,
        out float stopT, out float leftTrimT, out float rightTrimT)
    {
        float defaultT = atToNode ? 1f : 0f;

        var edge = graph.Edges[edgeIndex];
        int nodeIndex = atToNode ? edge.ToNode : edge.FromNode;

        var tangent = atToNode
            ? graph.EvaluateBezierTangent(edgeIndex, 1f)
            : graph.EvaluateBezierTangent(edgeIndex, 0f);
        float tangentLen = tangent.Length();
        if (tangentLen < 0.001f) { stopT = defaultT; leftTrimT = defaultT; rightTrimT = defaultT; return; }
        var dir = tangent / tangentLen;

        float halfWidthSelf = edge.LaneCount * LaneWidth;
        float maxLeftDist = 0f;
        float maxRightDist = 0f;

        foreach (int otherEdge in graph.GetOutgoingEdges(nodeIndex))
            AccumulateCrossingDistances(graph, edgeIndex, otherEdge, nodeIndex, dir, halfWidthSelf,
                isOutgoing: true, ref maxLeftDist, ref maxRightDist);

        foreach (int otherEdge in graph.GetIncomingEdges(nodeIndex))
            AccumulateCrossingDistances(graph, edgeIndex, otherEdge, nodeIndex, dir, halfWidthSelf,
                isOutgoing: false, ref maxLeftDist, ref maxRightDist);

        // Clamp
        float maxAllowed = edge.Length * MaxDistanceFraction;
        maxLeftDist = MathF.Min(maxLeftDist, maxAllowed);
        maxRightDist = MathF.Min(maxRightDist, maxAllowed);
        float maxDist = MathF.Max(maxLeftDist, maxRightDist);

        // Convert distances to t-values using arc-length parameterization
        float clampLo = atToNode ? 0.5f : 0.001f;
        float clampHi = atToNode ? 0.999f : 0.5f;

        stopT = maxDist < 0.01f ? defaultT
            : MathF.Max(clampLo, MathF.Min(ArcLengthToT(graph, edgeIndex, maxDist, atToNode), clampHi));
        leftTrimT = maxLeftDist < 0.01f ? defaultT
            : MathF.Max(clampLo, MathF.Min(ArcLengthToT(graph, edgeIndex, maxLeftDist, atToNode), clampHi));
        rightTrimT = maxRightDist < 0.01f ? defaultT
            : MathF.Max(clampLo, MathF.Min(ArcLengthToT(graph, edgeIndex, maxRightDist, atToNode), clampHi));
    }

    /// <summary>
    /// Computes per-side crossing distances for a single crossing road and accumulates them
    /// into the left/right maximums. Uses the crossing road's position (left or right of the
    /// current road via cross product) to assign asymmetric distances:
    /// near-side = (hSelf·cosθ + hOther) / sinθ, far-side = (hOther − hSelf·cosθ) / sinθ.
    /// </summary>
    private void AccumulateCrossingDistances(RoadGraph graph, int edgeIndex, int otherEdge,
        int nodeIndex, Vector2 dir, float halfWidthSelf, bool isOutgoing,
        ref float maxLeftDist, ref float maxRightDist)
    {
        if (otherEdge == edgeIndex) return;

        var edge = graph.Edges[edgeIndex];
        var other = graph.Edges[otherEdge];

        // Skip reverse edge (same road, opposite direction)
        if (other.FromNode == edge.ToNode && other.ToNode == edge.FromNode) return;

        // Get other edge's tangent at this node
        Vector2 otherTangent = isOutgoing
            ? graph.EvaluateBezierTangent(otherEdge, 0f)
            : graph.EvaluateBezierTangent(otherEdge, 1f);

        float otherLen = otherTangent.Length();
        if (otherLen < 0.001f) return;
        var otherDir = otherTangent / otherLen;

        // Compute acute angle between the two road directions
        float absDot = MathF.Min(MathF.Abs(dir.X * otherDir.X + dir.Y * otherDir.Y), 1f);
        float angle = MathF.Acos(absDot);
        if (angle < MinAngle) return;

        float sinAngle = MathF.Max(MathF.Sin(angle), MinSinAngle);
        float cosAngle = absDot; // cos of the acute angle

        float halfWidthOther = other.LaneCount * LaneWidth;

        // Near-side: boundary facing the crossing road needs full geometric setback
        float dNear = (halfWidthSelf * cosAngle + halfWidthOther) / sinAngle;
        // Far-side: boundary away from the crossing road needs less setback
        float dFar = MathF.Max(0f, (halfWidthOther - halfWidthSelf * cosAngle) / sinAngle);

        // Determine which side the crossing road is on using 2D cross product.
        // In Y-down: cross > 0 means otherDir is to the RIGHT of dir.
        float cross = dir.X * otherDir.Y - dir.Y * otherDir.X;

        if (cross >= 0)
        {
            // Other road is to the right → right boundary is near side
            if (dNear > maxRightDist) maxRightDist = dNear;
            if (dFar > maxLeftDist) maxLeftDist = dFar;
        }
        else
        {
            // Other road is to the left → left boundary is near side
            if (dNear > maxLeftDist) maxLeftDist = dNear;
            if (dFar > maxRightDist) maxRightDist = dFar;
        }
    }

    /// <summary>
    /// Walks along the Bezier curve from an endpoint, accumulating arc length, and returns
    /// the parametric t at which the desired distance is reached. This gives correct results
    /// regardless of control point placement (handle length).
    /// </summary>
    private static float ArcLengthToT(RoadGraph graph, int edgeIndex, float distance, bool fromToNode)
    {
        const int Steps = 20;

        var prev = graph.EvaluateBezier(edgeIndex, fromToNode ? 1f : 0f);
        float accumulated = 0f;

        for (int i = 1; i <= Steps; i++)
        {
            float t = fromToNode
                ? 1f - (float)i / Steps
                : (float)i / Steps;

            var pos = graph.EvaluateBezier(edgeIndex, t);
            float segLen = (pos - prev).Length();
            accumulated += segLen;

            if (accumulated >= distance)
            {
                // Interpolate within this segment
                float overshoot = accumulated - distance;
                float frac = segLen > 0.001f ? (segLen - overshoot) / segLen : 0f;
                float tPrev = fromToNode
                    ? 1f - (float)(i - 1) / Steps
                    : (float)(i - 1) / Steps;
                return tPrev + (t - tPrev) * frac;
            }

            prev = pos;
        }

        // Distance exceeds half the curve — return the far limit
        return fromToNode ? 0f : 1f;
    }
}
