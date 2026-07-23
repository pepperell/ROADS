using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Caches the parametric t position of stop lines at both ends of each edge.
/// Stop lines are set back from intersections based on the width and angle of crossing roads,
/// so vehicles stop before blocking cross-traffic. Crossing angles use each leg's LOCAL
/// approach direction — a secant over the first <see cref="ApproachProbeDistance"/> meters
/// of curve (see <see cref="ApproachDir"/>) — so a curved leg whose handle joins obliquely
/// gets the clearance of its drawn angle, not of its node-to-node chord. Computes per-side
/// (left/right of tangent) trim values so that boundary lines at acute-angle Y-intersections
/// are asymmetric — pushed back further on the sharp-angle side. At signalized (traffic-light) approaches the vehicle
/// stop-T gets an EXTRA <see cref="SimConstants.SignalCrosswalkSetback"/> so the continental
/// crosswalk fits between the stop line and the junction; the boundary trims deliberately do
/// not move (they define the junction fill / boundary-line geometry). Reading the TrafficLight
/// flag here relies on signal auto-assignment running before this rebuild in
/// SimulationLoop.RebuildWorldCaches' normalize phase. Rebuilds automatically when the graph
/// changes (flag edits bump the version).
/// </summary>
public class StopLineCache
{
    /// <summary>Assumed lane width in meters for stop line offset calculations.</summary>
    /// <summary>
    /// Lower edge (~15 degrees) of the crossing taper. Below this a leg is a through-continuation
    /// and contributes no setback; above it the contribution ramps in (see <see cref="ContinuationBand"/>).
    /// </summary>
    private const float MinAngle = 0.262f;
    /// <summary>
    /// Width (~15 degrees) of the smooth ramp above <see cref="MinAngle"/> over which a crossing's
    /// setback contribution fades from zero to full. Without this ramp the contribution would switch
    /// on abruptly at <see cref="MinAngle"/> (the clamped denominator makes it jump straight to ~13m),
    /// so the intersection would visibly snap in size as a leg is dragged through the transition.
    /// </summary>
    private const float ContinuationBand = 0.262f;
    /// <summary>
    /// Minimum effective sin(angle) for crossing distance calculation.
    /// Caps crossing distance at halfWidth / MinSinAngle (~2× halfWidth at 30°),
    /// preventing unrealistically large intersections at acute angles.
    /// </summary>
    private const float MinSinAngle = 0.5f;
    /// <summary>Maximum fraction of edge length a stop line can be set back from the node.</summary>
    private const float MaxDistanceFraction = 0.4f;
    /// <summary>
    /// Arc-length distance (m) from the node at which a leg's local approach direction is
    /// probed (see <see cref="ApproachDir"/>). Chosen at the scale of typical intersection
    /// clearances so the secant reflects the road's drawn direction across the junction
    /// area; legs shorter than this probe fall back to the far endpoint (the chord).
    /// </summary>
    private const float ApproachProbeDistance = 10f;

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

        // Approach direction = local secant at this end (see ApproachDir), NOT the
        // node-to-node chord and NOT the instantaneous endpoint tangent. The chord ignores
        // curvature: a curved leg whose handle joins the crossing road obliquely still read
        // as perpendicular, so the acute-side clearance came out far too small and boundary
        // lines / stop lines poked into the junction. The raw endpoint tangent is the
        // opposite extreme (control-point-sensitive jitter, which the chord was originally
        // chosen to avoid). The secant reflects the road as actually drawn across the
        // junction area and changes smoothly with edits.
        var dir = ApproachDir(graph, edgeIndex, atToNode);
        if (dir == Vector2.Zero) { stopT = defaultT; leftTrimT = defaultT; rightTrimT = defaultT; return; }

        // Local endpoint tangent at this node (zero if degenerate). Used only to recognize a
        // smooth through-road as a continuation (not a crossing) — see AccumulateCrossingDistances.
        var selfTangent = EndpointTangentDir(graph, edgeIndex, atToNode);

        float halfWidthSelf = GeometryUtil.RoadHalfWidth(graph, edgeIndex);
        float maxLeftDist = 0f;
        float maxRightDist = 0f;

        foreach (int otherEdge in graph.GetOutgoingEdges(nodeIndex))
            AccumulateCrossingDistances(graph, nodeIndex, edgeIndex, otherEdge, dir, selfTangent, halfWidthSelf,
                atToNode, ref maxLeftDist, ref maxRightDist);

        foreach (int otherEdge in graph.GetIncomingEdges(nodeIndex))
            AccumulateCrossingDistances(graph, nodeIndex, edgeIndex, otherEdge, dir, selfTangent, halfWidthSelf,
                atToNode, ref maxLeftDist, ref maxRightDist);

        // Clamp
        float maxAllowed = edge.Length * MaxDistanceFraction;
        maxLeftDist = MathF.Min(maxLeftDist, maxAllowed);
        maxRightDist = MathF.Min(maxRightDist, maxAllowed);
        float maxDist = MathF.Max(maxLeftDist, maxRightDist);

        // Signalized approaches reserve room for the continental crosswalk between the
        // stop line and the junction: the vehicle stop-T is pulled back by
        // SignalCrosswalkSetback (still under the edge-length cap) while the per-side
        // boundary trims stay at the geometric clearance, so the junction fill and
        // boundary lines do not grow — the crosswalk band occupies the reserved strip.
        // Dirt approaches get no crosswalk (no paint), so no reservation either.
        float stopDist = maxDist;
        if (maxDist >= 0.01f && edge.RoadType != RoadType.Dirt
            && (graph.Nodes[nodeIndex].Flags & NodeFlags.TrafficLight) != 0)
            stopDist = MathF.Min(maxDist + SimConstants.SignalCrosswalkSetback, maxAllowed);

        // Convert distances to t-values using arc-length parameterization
        float clampLo = atToNode ? 0.5f : 0.001f;
        float clampHi = atToNode ? 0.999f : 0.5f;

        stopT = stopDist < 0.01f ? defaultT
            : MathF.Max(clampLo, MathF.Min(ArcLengthToT(graph, edgeIndex, stopDist, atToNode), clampHi));
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
    ///
    /// A leg can be skipped as a through-continuation (not a crossing) ONLY when it leaves
    /// the node on the OPPOSITE side of this edge (away-oriented directions anti-parallel):
    /// a leg leaving on the SAME side is a merge and always gets full crossing treatment no
    /// matter how shallow the angle — an angle-only test made sharp merges lose every
    /// setback, so their stop lines and signals vanished. Opposite-side legs are then judged
    /// near-collinear by EITHER the local approach secant OR the local endpoint tangent.
    /// Both tests are needed: a smooth through-road can bend within the probe distance (an
    /// S through the node — the secants alone would mistake it for a crossing and balloon
    /// the joint, while the tangents are collinear), and dragging a node can leave a stale
    /// tangent angle (see RoadGraph.MoveNode) that the secant — which follows the drawn
    /// curve — corrects. The crossing-angle MAGNITUDE uses the secants.
    /// <paramref name="selfTangent"/> is this edge's normalized endpoint tangent at the
    /// node (Zero if degenerate); <paramref name="selfAtToNode"/> says which end of THIS
    /// edge the node is (needed to away-orient <paramref name="dir"/> for the side test).
    /// </summary>
    private void AccumulateCrossingDistances(RoadGraph graph, int nodeIndex, int edgeIndex, int otherEdge,
        Vector2 dir, Vector2 selfTangent, float halfWidthSelf, bool selfAtToNode,
        ref float maxLeftDist, ref float maxRightDist)
    {
        if (otherEdge == edgeIndex) return;

        var edge = graph.Edges[edgeIndex];
        var other = graph.Edges[otherEdge];

        // Skip reverse edge (same road, opposite direction)
        if (other.FromNode == edge.ToNode && other.ToNode == edge.FromNode) return;

        // Crossing road's local approach direction at the node (secant probe — see
        // ApproachDir). It auto-orients to the travel direction (points away from the node
        // for an outgoing edge, toward it for an incoming one), matching the self-direction
        // convention in ComputeAllStopTs. Zero means a degenerate edge.
        bool otherIncoming = other.ToNode == nodeIndex;
        var otherDir = ApproachDir(graph, otherEdge, atToNode: otherIncoming);
        if (otherDir == Vector2.Zero) return;

        // Acute angle between the two approach directions — drives the setback magnitude.
        float absDot = MathF.Min(MathF.Abs(dir.X * otherDir.X + dir.Y * otherDir.Y), 1f);
        float angle = MathF.Acos(absDot);

        // Side test with away-oriented directions (pointing OUT of the node along each leg):
        // only an OPPOSITE-side leg (away-dot < 0) can be a through-continuation. A leg
        // leaving on the SAME side is a merge regardless of angle and keeps weight 1.
        // Continuous at the branch boundary: away-dot ≈ 0 means ~90° legs, where the
        // continuation branch also yields weight 1.
        var awaySelf = selfAtToNode ? -dir : dir;
        var awayOther = otherIncoming ? -otherDir : otherDir;
        float weight = 1f;
        if (awaySelf.X * awayOther.X + awaySelf.Y * awayOther.Y < 0f)
        {
            // Opposite-side leg: judge continuation by secant collinearity, refined by the
            // endpoint tangents when both are valid AND themselves opposite-side (an
            // anti-parallel tangent pair marks a smooth through-joint).
            float skipAngle = angle;
            var otherTangent = EndpointTangentDir(graph, otherEdge, atToNode: otherIncoming);
            if (selfTangent != Vector2.Zero && otherTangent != Vector2.Zero)
            {
                float tanDot = selfTangent.X * otherTangent.X + selfTangent.Y * otherTangent.Y;
                if (tanDot < 0f)
                    skipAngle = MathF.Min(skipAngle, MathF.Acos(MathF.Min(-tanDot, 1f)));
            }

            // Smoothly fade the crossing in across [MinAngle, MinAngle + ContinuationBand]
            // instead of switching it on abruptly, so the intersection size changes
            // continuously (no snap) as a leg is dragged through the continuation→crossing
            // transition. weight is 0 below MinAngle.
            weight = ContinuationWeight(skipAngle);
            if (weight <= 0f) return;
        }

        float sinAngle = MathF.Max(MathF.Sin(angle), MinSinAngle);
        float cosAngle = absDot; // cos of the acute angle

        float halfWidthOther = GeometryUtil.RoadHalfWidth(graph, otherEdge);

        // Near-side: boundary facing the crossing road needs full geometric setback
        float dNear = weight * (halfWidthSelf * cosAngle + halfWidthOther) / sinAngle;
        // Far-side: boundary away from the crossing road needs less setback
        float dFar = weight * MathF.Max(0f, (halfWidthOther - halfWidthSelf * cosAngle) / sinAngle);

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
    /// Smoothstep ramp for a crossing's setback contribution: 0 at or below <see cref="MinAngle"/>,
    /// 1 at or above <see cref="MinAngle"/> + <see cref="ContinuationBand"/>, with a C1-continuous
    /// transition between (zero slope at both ends) so the intersection size has no kink or jump
    /// as the crossing angle changes.
    /// </summary>
    private static float ContinuationWeight(float angle)
    {
        float x = (angle - MinAngle) / ContinuationBand;
        if (x <= 0f) return 0f;
        if (x >= 1f) return 1f;
        return x * x * (3f - 2f * x);
    }

    /// <summary>
    /// Local approach direction of an edge at one of its end nodes, oriented along the
    /// travel direction (into the node at the ToNode end, away from it at the FromNode
    /// end — the chord convention the crossing math was built on). Computed as the secant
    /// from the node to the curve point <see cref="ApproachProbeDistance"/> of arc length
    /// away: unlike the node-to-node chord it reflects how a CURVED leg actually meets the
    /// junction (an oblique handle gives an oblique approach), and unlike the instantaneous
    /// endpoint tangent it integrates the first meters of curve, staying stable under small
    /// control-point edits. Legs shorter than the probe degrade to the chord. Returns
    /// <see cref="Vector2.Zero"/> for degenerate (zero-length) geometry.
    /// </summary>
    private static Vector2 ApproachDir(RoadGraph graph, int edgeIndex, bool atToNode)
    {
        float tProbe = ArcLengthToT(graph, edgeIndex, ApproachProbeDistance, atToNode);
        var nodePos = graph.EvaluateBezier(edgeIndex, atToNode ? 1f : 0f);
        var probe = graph.EvaluateBezier(edgeIndex, tProbe);
        var secant = atToNode ? nodePos - probe : probe - nodePos;
        float len = secant.Length();
        return len < 0.001f ? Vector2.Zero : secant / len;
    }

    /// <summary>
    /// Returns the normalized Bezier tangent at one endpoint of an edge, oriented to point
    /// along the curve away from that endpoint, or <see cref="Vector2.Zero"/> if the tangent
    /// is degenerate (control point coincident with the node). Used to detect smooth
    /// through-continuations independently of the node-to-node chord.
    /// </summary>
    private static Vector2 EndpointTangentDir(RoadGraph graph, int edgeIndex, bool atToNode)
    {
        var tangent = graph.EvaluateBezierTangent(edgeIndex, atToNode ? 1f : 0f);
        // At the ToNode the tangent points into the node; flip so it points away (out of the node).
        if (atToNode) tangent = -tangent;
        float len = tangent.Length();
        return len < 0.001f ? Vector2.Zero : tangent / len;
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
