using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Generates and caches cubic Bezier arcs connecting incoming lane endpoints to outgoing lane
/// endpoints at each intersection node. Arcs provide smooth turn geometry for vehicles traversing
/// intersections. Rebuilds automatically when the graph version changes.
/// </summary>
public class IntersectionArcCache
{
    private const float LaneWidth = SimConstants.LaneWidth;
    private const float MinArmLength = 1.5f;
    private const float MaxArmLength = 15f;

    private IntersectionArc[] _arcs = Array.Empty<IntersectionArc>();
    private int _arcCount;
    private int _cachedVersion = -1;

    /// <summary>Lookup: (incomingEdge, outgoingEdge, inLane, outLane) → arc index.</summary>
    private readonly Dictionary<(int, int, byte, byte), int> _lookup = new();

    /// <summary>Reverse lookup: (incomingEdge, inLane) → list of reachable (outEdge, outLane, arcIdx) tuples.</summary>
    private readonly Dictionary<(int, byte), List<(int outEdge, byte outLane, int arcIdx)>> _reachableFromLane = new();

    /// <summary>Per-node start index into _arcs for node-level queries.</summary>
    private int[] _nodeArcStartIdx = Array.Empty<int>();
    /// <summary>Per-node count of arcs.</summary>
    private int[] _nodeArcCount = Array.Empty<int>();

    /// <summary>Total number of arcs in the cache.</summary>
    public int ArcCount => _arcCount;

    /// <summary>
    /// Returns the arc index for a specific turn and lane combination, or -1 if none exists.
    /// </summary>
    public int GetArcIndex(int incomingEdge, int outgoingEdge, byte inLane, byte outLane)
    {
        return _lookup.TryGetValue((incomingEdge, outgoingEdge, inLane, outLane), out int idx) ? idx : -1;
    }

    /// <summary>
    /// Returns all (outEdge, outLane, arcIndex) tuples reachable from the given incoming edge and lane,
    /// or null if none exist. Used by forced-reroute logic when a vehicle cannot take its intended turn.
    /// </summary>
    /// <param name="inEdge">Incoming edge index.</param>
    /// <param name="inLane">Incoming lane index.</param>
    /// <returns>List of reachable options, or null.</returns>
    public List<(int outEdge, byte outLane, int arcIdx)>? GetReachableFromLane(int inEdge, byte inLane)
    {
        return _reachableFromLane.TryGetValue((inEdge, inLane), out var list) ? list : null;
    }

    /// <summary>
    /// Returns the arc struct at the given index.
    /// </summary>
    public IntersectionArc GetArc(int arcIndex)
    {
        return _arcs[arcIndex];
    }

    /// <summary>
    /// Returns all arc indices at a node as a contiguous span (for rendering).
    /// </summary>
    public ReadOnlySpan<int> GetArcsAtNode(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodeArcStartIdx.Length) return ReadOnlySpan<int>.Empty;
        int start = _nodeArcStartIdx[nodeIndex];
        int count = _nodeArcCount[nodeIndex];
        if (count == 0) return ReadOnlySpan<int>.Empty;
        return new ReadOnlySpan<int>(_nodeArcIndices, start, count);
    }

    /// <summary>Flat array of arc indices grouped by node, for GetArcsAtNode.</summary>
    private int[] _nodeArcIndices = Array.Empty<int>();

    /// <summary>Per-arc list of conflicting arc indices (arcs at the same node whose geometry intersects).</summary>
    private int[][] _conflictingArcs = Array.Empty<int[]>();

    /// <summary>
    /// Returns the arc indices that conflict (geometrically intersect) with the given arc.
    /// </summary>
    public ReadOnlySpan<int> GetConflictingArcs(int arcIndex)
    {
        if (arcIndex < 0 || arcIndex >= _conflictingArcs.Length) return ReadOnlySpan<int>.Empty;
        return _conflictingArcs[arcIndex] ?? ReadOnlySpan<int>.Empty;
    }

    /// <summary>
    /// Evaluates the cubic Bezier position on an arc at parameter t.
    /// </summary>
    public Vector2 EvaluateArc(int arcIndex, float t)
    {
        var arc = _arcs[arcIndex];
        float u = 1f - t;
        return u * u * u * arc.P0
             + 3f * u * u * t * arc.P1
             + 3f * u * t * t * arc.P2
             + t * t * t * arc.P3;
    }

    /// <summary>
    /// Evaluates the tangent (first derivative) of an arc at parameter t.
    /// </summary>
    public Vector2 EvaluateArcTangent(int arcIndex, float t)
    {
        var arc = _arcs[arcIndex];
        float u = 1f - t;
        return 3f * u * u * (arc.P1 - arc.P0)
             + 6f * u * t * (arc.P2 - arc.P1)
             + 3f * t * t * (arc.P3 - arc.P2);
    }

    /// <summary>
    /// Rebuilds the arc cache if the graph has changed since the last rebuild.
    /// Must be called after StopLineCache has been rebuilt.
    /// </summary>
    public void RebuildIfNeeded(RoadGraph graph, StopLineCache stopLines)
    {
        if (_cachedVersion == graph.Version) return;
        Rebuild(graph, stopLines);
        _cachedVersion = graph.Version;
    }

    private void Rebuild(RoadGraph graph, StopLineCache stopLines)
    {
        _lookup.Clear();
        _reachableFromLane.Clear();
        _arcCount = 0;

        int nodeCount = graph.Nodes.Count;
        if (_nodeArcStartIdx.Length < nodeCount)
        {
            _nodeArcStartIdx = new int[nodeCount];
            _nodeArcCount = new int[nodeCount];
        }
        else
        {
            Array.Clear(_nodeArcStartIdx, 0, nodeCount);
            Array.Clear(_nodeArcCount, 0, nodeCount);
        }

        // Note: default lane restrictions are now applied by RoadGraph.ApplyDefaultLaneRestrictions()
        // before this cache rebuilds, so the version bump happens predictably and this method
        // remains a pure read of the graph.

        // First pass: count arcs per node to size arrays
        var arcList = new List<IntersectionArc>();
        var nodeArcIdxList = new List<int>();

        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue; // defunct

            var incoming = graph.GetIncomingEdges(n);
            if (incoming.Count == 0) continue;

            int nodeStart = nodeArcIdxList.Count;

            foreach (int inEdge in incoming)
            {
                var inEdgeData = graph.Edges[inEdge];
                if (inEdgeData.FromNode < 0) continue; // defunct

                var allowedTurns = graph.GetAllowedTurns(n, inEdge);

                foreach (int outEdge in allowedTurns)
                {
                    var outEdgeData = graph.Edges[outEdge];
                    if (outEdgeData.FromNode < 0) continue; // defunct

                    // Generate arcs for all valid lane combinations
                    GenerateLaneArcs(graph, stopLines, n, inEdge, inEdgeData, outEdge, outEdgeData, arcList, nodeArcIdxList);
                }
            }

            _nodeArcStartIdx[n] = nodeStart;
            _nodeArcCount[n] = nodeArcIdxList.Count - nodeStart;
        }

        // Compact into arrays
        _arcCount = arcList.Count;
        if (_arcs.Length < _arcCount)
            _arcs = new IntersectionArc[_arcCount];
        for (int i = 0; i < _arcCount; i++)
            _arcs[i] = arcList[i];

        _nodeArcIndices = nodeArcIdxList.ToArray();

        // Precompute arc-arc conflicts (geometric intersection) at each node
        BuildConflictTable(graph);
    }

    /// <summary>
    /// Builds the per-arc conflict table by testing geometric intersection of arc pairs
    /// at each node. Two arcs conflict if their sampled polylines have crossing segments.
    /// </summary>
    private void BuildConflictTable(RoadGraph graph)
    {
        if (_conflictingArcs.Length < _arcCount)
            _conflictingArcs = new int[_arcCount][];
        else
            Array.Clear(_conflictingArcs, 0, _arcCount);

        const int sampleCount = 10;
        int nodeCount = graph.Nodes.Count;

        // Sample all arcs into polylines once
        var samples = new Vector2[_arcCount][];
        for (int i = 0; i < _arcCount; i++)
        {
            samples[i] = new Vector2[sampleCount + 1];
            for (int s = 0; s <= sampleCount; s++)
                samples[i][s] = EvaluateArc(i, s / (float)sampleCount);
        }

        // For each node, test all arc pairs
        for (int n = 0; n < nodeCount; n++)
        {
            if (n >= _nodeArcStartIdx.Length) break;
            int start = _nodeArcStartIdx[n];
            int count = _nodeArcCount[n];
            if (count < 2) continue;

            for (int ai = 0; ai < count; ai++)
            {
                int arcA = _nodeArcIndices[start + ai];
                List<int>? conflicts = null;

                for (int bi = ai + 1; bi < count; bi++)
                {
                    int arcB = _nodeArcIndices[start + bi];

                    // Conflict if polylines cross OR if both arcs exit to the same edge+lane
                    bool geometricConflict = PolylinesIntersect(samples[arcA], samples[arcB]);
                    bool sharedOutgoing = _arcs[arcA].OutgoingEdge == _arcs[arcB].OutgoingEdge
                                       && _arcs[arcA].OutgoingLane == _arcs[arcB].OutgoingLane;
                    if (geometricConflict || sharedOutgoing)
                    {
                        conflicts ??= new List<int>();
                        conflicts.Add(arcB);

                        // Add reverse mapping too
                        if (_conflictingArcs[arcB] == null)
                            _conflictingArcs[arcB] = new[] { arcA };
                        else
                        {
                            var existing = _conflictingArcs[arcB];
                            var expanded = new int[existing.Length + 1];
                            existing.CopyTo(expanded, 0);
                            expanded[existing.Length] = arcA;
                            _conflictingArcs[arcB] = expanded;
                        }
                    }
                }

                if (conflicts != null)
                {
                    if (_conflictingArcs[arcA] == null)
                        _conflictingArcs[arcA] = conflicts.ToArray();
                    else
                    {
                        var existing = _conflictingArcs[arcA];
                        var merged = new int[existing.Length + conflicts.Count];
                        existing.CopyTo(merged, 0);
                        conflicts.CopyTo(merged, existing.Length);
                        _conflictingArcs[arcA] = merged;
                    }
                }
            }
        }
    }

    /// <summary>
    /// Tests whether two polylines (arrays of sample points) have any crossing segments.
    /// </summary>
    private static bool PolylinesIntersect(Vector2[] polyA, Vector2[] polyB)
    {
        for (int i = 0; i < polyA.Length - 1; i++)
        {
            for (int j = 0; j < polyB.Length - 1; j++)
            {
                if (SegmentsIntersect(polyA[i], polyA[i + 1], polyB[j], polyB[j + 1]))
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Tests if two line segments (a1-a2 and b1-b2) intersect, excluding shared endpoints.
    /// </summary>
    private static bool SegmentsIntersect(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2)
    {
        float d1x = a2.X - a1.X, d1y = a2.Y - a1.Y;
        float d2x = b2.X - b1.X, d2y = b2.Y - b1.Y;
        float cross = d1x * d2y - d1y * d2x;
        if (MathF.Abs(cross) < 1e-8f) return false; // parallel

        float dx = b1.X - a1.X, dy = b1.Y - a1.Y;
        float t = (dx * d2y - dy * d2x) / cross;
        float u = (dx * d1y - dy * d1x) / cross;

        // Exclude exact endpoints (shared start/end points of arcs at same node)
        const float eps = 0.02f;
        return t > eps && t < 1f - eps && u > eps && u < 1f - eps;
    }

    private void GenerateLaneArcs(
        RoadGraph graph, StopLineCache stopLines,
        int nodeIndex, int inEdge, RoadEdge inEdgeData, int outEdge, RoadEdge outEdgeData,
        List<IntersectionArc> arcList, List<int> nodeArcIdxList)
    {
        float inStopT = stopLines.GetStopTAtToNode(inEdge);
        float outStartT = stopLines.GetStopTAtFromNode(outEdge);

        // Get tangent directions at the junction points
        var inTangent = graph.EvaluateBezierTangent(inEdge, inStopT);
        var outTangent = graph.EvaluateBezierTangent(outEdge, outStartT);
        float inTanLen = inTangent.Length();
        float outTanLen = outTangent.Length();
        if (inTanLen < 0.001f || outTanLen < 0.001f) return;

        var inDir = inTangent / inTanLen;
        var outDir = outTangent / outTanLen;

        // Turn speed factor based on dot product of tangent directions
        float dot = inDir.X * outDir.X + inDir.Y * outDir.Y;
        float speedFactor;
        if (dot >= 0.9f) speedFactor = 1.0f;
        else if (dot >= 0.5f) speedFactor = 0.7f;
        else if (dot >= 0.0f) speedFactor = 0.5f;
        else speedFactor = 0.35f;

        float baseSpeedLimit = MathF.Min(
            inEdgeData.SpeedLimit > 0 ? inEdgeData.SpeedLimit : 13.4f,
            outEdgeData.SpeedLimit > 0 ? outEdgeData.SpeedLimit : 13.4f);

        // Determine turn direction for lane mapping
        // Cross product in Y-down coords: positive = right turn, negative = left turn
        float cross = inDir.X * outDir.Y - inDir.Y * outDir.X;

        byte inLaneCount = inEdgeData.LaneCount;
        byte outLaneCount = outEdgeData.LaneCount;

        // Generate lane-to-lane arcs, respecting per-lane restrictions
        var lanePairs = GetLanePairs(inLaneCount, outLaneCount, cross);

        // Filter by per-lane restrictions: if a lane has explicit restrictions,
        // only generate arcs for allowed (outEdge, outLane) pairs
        for (int i = lanePairs.Count - 1; i >= 0; i--)
        {
            var (inLane, outLane) = lanePairs[i];
            var restrictions = graph.GetLaneRestrictions(inEdge, inLane);
            if (restrictions != null && !restrictions.Contains((outEdge, outLane)))
                lanePairs.RemoveAt(i);
        }

        foreach (var (inLane, outLane) in lanePairs)
        {
            float inLaneOffset = LaneWidth * (0.5f + inLane);
            float outLaneOffset = LaneWidth * (0.5f + outLane);

            var p0 = OffsetRight(graph, inEdge, inStopT, inLaneOffset);
            var p3 = OffsetRight(graph, outEdge, outStartT, outLaneOffset);

            float dist = Vector2.Distance(p0, p3);
            float armLength = MathF.Max(MinArmLength, MathF.Min(dist / 3f, MaxArmLength));

            var p1 = p0 + inDir * armLength;
            var p2 = p3 - outDir * armLength;

            float length = EstimateBezierLength(p0, p1, p2, p3);

            var arc = new IntersectionArc
            {
                NodeIndex = nodeIndex,
                IncomingEdge = inEdge,
                OutgoingEdge = outEdge,
                IncomingLane = inLane,
                OutgoingLane = outLane,
                P0 = p0,
                P1 = p1,
                P2 = p2,
                P3 = p3,
                Length = length,
                SpeedLimit = baseSpeedLimit * speedFactor,
            };

            int arcIdx = arcList.Count;
            arcList.Add(arc);
            nodeArcIdxList.Add(arcIdx);
            _lookup[(inEdge, outEdge, inLane, outLane)] = arcIdx;

            var reachKey = (inEdge, inLane);
            if (!_reachableFromLane.TryGetValue(reachKey, out var reachList))
            {
                reachList = new List<(int, byte, int)>();
                _reachableFromLane[reachKey] = reachList;
            }
            reachList.Add((outEdge, outLane, arcIdx));
        }
    }

    /// <summary>
    /// Determines which lane pairs to generate arcs for, based on turn direction.
    /// Delegates to <see cref="RoadGraph.ComputeGeometryLanePairs"/> to ensure
    /// consistency between arc generation and lane restriction defaults.
    /// </summary>
    private static List<(byte inLane, byte outLane)> GetLanePairs(byte inLaneCount, byte outLaneCount, float cross)
    {
        return RoadGraph.ComputeGeometryLanePairs(inLaneCount, outLaneCount, cross);
    }

    /// <summary>
    /// Offsets a point on a Bezier curve to the right of the travel direction.
    /// Delegates to <see cref="GeometryUtil.OffsetRight"/>.
    /// </summary>
    private static Vector2 OffsetRight(RoadGraph graph, int edgeIdx, float t, float offset)
        => GeometryUtil.OffsetRight(graph, edgeIdx, t, offset);

    /// <summary>
    /// Estimates cubic Bezier arc length using a 16-segment polyline approximation.
    /// Delegates to <see cref="GeometryUtil.EstimateBezierLength"/>.
    /// </summary>
    private static float EstimateBezierLength(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3)
        => GeometryUtil.EstimateBezierLength(p0, p1, p2, p3);
}
