using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Caches the parametric t position of stop lines at both ends of each edge.
/// Stop lines are set back from intersections based on the width and angle of crossing roads,
/// so vehicles stop before blocking cross-traffic. Crossing angles use each leg's LOCAL
/// approach direction — a secant over the first <see cref="ApproachProbeDistance"/> meters
/// of curve (see <see cref="ApproachDir"/>) — so a curved leg whose handle joins obliquely
/// gets the clearance of its drawn angle, not of its node-to-node chord. Every setback is
/// measured by an overlap walk along the actual curves (<see cref="WalkOverlapDistances"/>),
/// so per-side (left/right of tangent) boundary trims are naturally asymmetric at
/// acute-angle Y-intersections and stop lines land where the roads genuinely split —
/// including outside the long shared pavement of a shallow merge. At signalized (traffic-light) approaches the vehicle
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
    /// on abruptly at <see cref="MinAngle"/> (the overlap-walk distance is large at such shallow
    /// angles), so the intersection would visibly snap in size as a leg is dragged through the
    /// continuation→crossing transition.
    /// </summary>
    private const float ContinuationBand = 0.262f;
    /// <summary>Target arc-length step (m) between overlap-walk samples along this edge.</summary>
    private const float OverlapWalkStep = 1.5f;
    /// <summary>Upper bound on overlap-walk samples (very long edges sample coarser).</summary>
    private const int OverlapMaxWalkSteps = 128;
    /// <summary>Polyline segments the crossing road is sampled into for overlap distance tests.</summary>
    private const int OverlapOtherSamples = 20;
    /// <summary>
    /// Safety margin (m) added past the refined overlap→clear release point so boundary
    /// lines stay UNDER-length despite polyline/interpolation error — a line must stop
    /// short of the junction, never poke into it; the corner curves bridge the rest.
    /// </summary>
    private const float ClearanceMargin = 0.25f;
    /// <summary>
    /// Consecutive clear samples (both corners outside the crossing road's asphalt) after
    /// which the overlap walk stops early. The walk runs for EVERY crossing pair on every
    /// rebuild — including per-frame rebuilds while dragging — and at ordinary angles the
    /// corners clear within a few meters, so this bounds the per-pair cost; geometry that
    /// re-touches the same edge after this much clearance without an intervening node is
    /// degenerate (the road tool splits real crossings).
    /// </summary>
    private const int OverlapClearExitStreak = 4;
    /// <summary>Maximum fraction of edge length a stop line can be set back from the node.</summary>
    private const float MaxDistanceFraction = 0.4f;
    /// <summary>
    /// Minimum stop-line inset (m) from the node at approaches whose node is
    /// stop-controlled (stop sign / traffic light) but whose overlap-based setback is
    /// zero — e.g. the far approach of an oblique Y, whose roadway nothing overlaps.
    /// Wherever a sign is posted a line exists; overlap only ever pushes it FURTHER back.
    /// Kept small: on such a side the junction boundary is essentially the node itself.
    /// </summary>
    private const float MinControlledStopInset = 1.5f;
    /// <summary>
    /// Extra setback (m) added to EVERY stop line on top of its overlap/minimum clearance,
    /// so the bar sits comfortably short of the junction rather than kissing it. Applies
    /// to the vehicle stop-T only — boundary trims stay purely overlap-based.
    /// </summary>
    private const float StopLineExtraSetback = 2f;
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

    /// <summary>Rebuild-time scratch: the crossing road's centerline polyline for the
    /// overlap walk (<see cref="OverlapOtherSamples"/> segments).</summary>
    private readonly Vector2[] _otherPoly = new Vector2[OverlapOtherSamples + 1];

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

        // DRAWN half-width (geometric × per-type multiplier): the walk's corner points must
        // sit at the asphalt edge the player sees — the curb line — not the narrower
        // geometric edge, or trims land inside the drawn pavement (amplified by 1/sin on
        // the acute side into a visible boundary overshoot).
        float halfWidthSelf = GeometryUtil.RoadHalfWidth(graph, edgeIndex)
            * RoadTypeDefaults.GetDrawnWidthMultiplier(edge.RoadType);
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

        float stopDist = maxDist;
        var nodeFlags = graph.Nodes[nodeIndex].Flags;

        // A stop-controlled node (sign or light) always yields a drawable stop line: zero
        // overlap means nothing pushes the line back — NOT that there is no line (the sign
        // is still posted, e.g. the far approach of an oblique Y, whose roadway nothing
        // overlaps). Floor the distance with a minimal inset so the line sits at the
        // junction edge, then pull every line back by the extra setback. Boundary trims
        // are untouched, staying purely overlap-based.
        if ((nodeFlags & (NodeFlags.TrafficLight | NodeFlags.StopSign)) != 0)
            stopDist = MathF.Min(
                MathF.Max(stopDist, MinControlledStopInset) + StopLineExtraSetback, maxAllowed);

        // Signalized approaches reserve room for the continental crosswalk between the
        // stop line and the junction: the vehicle stop-T is pulled back by
        // SignalCrosswalkSetback (still under the edge-length cap) while the per-side
        // boundary trims stay at the geometric clearance, so the junction fill and
        // boundary lines do not grow — the crosswalk band occupies the reserved strip.
        // Dirt approaches get no crosswalk (no paint), so no reservation either.
        if (stopDist >= 0.01f && edge.RoadType != RoadType.Dirt
            && (nodeFlags & NodeFlags.TrafficLight) != 0)
            stopDist = MathF.Min(stopDist + SimConstants.SignalCrosswalkSetback, maxAllowed);

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
    /// Computes per-side crossing distances for a single crossing road via the overlap walk
    /// (<see cref="WalkOverlapDistances"/>) and accumulates them into the left/right
    /// maximums. The walk follows the actual curves to the split point at every angle, so
    /// there is no separate straight-line formula and no method handover as a leg is
    /// dragged sharper.
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

        // One uniform method at every angle: walk the actual curves and trim each boundary
        // where its corner really clears the other road's asphalt. (A straight-line formula
        // formerly handled angles above ~30°; the walk's true-curve split points look
        // better and remove the method handover entirely.) DRAWN half-width, matching the
        // self side — clearance is measured against the pavement as rendered.
        float halfWidthOther = GeometryUtil.RoadHalfWidth(graph, otherEdge)
            * RoadTypeDefaults.GetDrawnWidthMultiplier(other.RoadType);
        WalkOverlapDistances(graph, edgeIndex, otherEdge, selfAtToNode,
            halfWidthSelf, halfWidthOther, out float dLeft, out float dRight);

        float dLeftAdd = weight * dLeft;
        float dRightAdd = weight * dRight;
        if (dLeftAdd > maxLeftDist) maxLeftDist = dLeftAdd;
        if (dRightAdd > maxRightDist) maxRightDist = dRightAdd;
    }

    /// <summary>
    /// Overlap walk — THE setback measurement for every crossing pair: marches along this
    /// edge's curve away from the node, accumulating arc length, and returns for each
    /// boundary (left/right of travel, corners at ±<paramref name="halfWidthSelf"/>) the
    /// arc distance at which the corner comes CLEAR of the crossing road's asphalt
    /// (distance to its centerline at least <paramref name="halfWidthOther"/>). Boundary
    /// trims and stop lines then land where the roads actually split apart — following the
    /// real curves, so a merge leg bending away releases exactly at its visible separation
    /// point; at wide angles the result matches the classic straight-line clearance. Each
    /// overlap→clear transition is refined by bisection inside the bracketing step and
    /// biased to the CLEAR side plus <see cref="ClearanceMargin"/>, so trims move smoothly
    /// with geometry edits and boundary lines are always UNDER-length (never poking into
    /// the junction), with the corner curves bridging the remainder. The walk (and thus
    /// every returned distance) is capped at <see cref="MaxDistanceFraction"/> of the edge
    /// (the caller's clamp) and exits early after <see cref="OverlapClearExitStreak"/>
    /// consecutive clear samples, which keeps ordinary right-angle pairs to a handful of
    /// samples.
    /// </summary>
    private void WalkOverlapDistances(RoadGraph graph, int edgeIndex, int otherEdge, bool atToNode,
        float halfWidthSelf, float halfWidthOther, out float dLeft, out float dRight)
    {
        for (int i = 0; i <= OverlapOtherSamples; i++)
            _otherPoly[i] = graph.EvaluateBezier(otherEdge, i / (float)OverlapOtherSamples);

        var edge = graph.Edges[edgeIndex];
        float maxWalk = edge.Length * MaxDistanceFraction;
        int steps = Math.Clamp((int)MathF.Ceiling(edge.Length / OverlapWalkStep), 16, OverlapMaxWalkSteps);
        float clearSq = halfWidthOther * halfWidthOther;

        dLeft = 0f;
        dRight = 0f;
        var prev = graph.EvaluateBezier(edgeIndex, atToNode ? 1f : 0f);
        float tPrev = atToNode ? 1f : 0f;
        float accumulated = 0f;
        int clearStreak = 0;
        // Pending flags: the side overlapped at the previous sample and its exact release
        // point awaits the next clear sample (refined in the bracketing interval). Seeded
        // with the corners AT the node (d = 0): overlap normally runs from the node
        // outward, and on long edges (coarse steps) a short wide-angle mouth crossing —
        // a boundary cutting across an oblique leg's opening for only a couple of meters —
        // can be SHORTER than one step. Without the seed the first sample lands already
        // clear, the overlap goes undetected, and the curb line runs across the other
        // road's mouth. The seed also routes every near-node release through the bisection,
        // making trims sub-step accurate at ordinary junctions too.
        bool leftPending = CornerOverlaps(graph, edgeIndex, tPrev, -halfWidthSelf, clearSq);
        bool rightPending = CornerOverlaps(graph, edgeIndex, tPrev, halfWidthSelf, clearSq);
        float leftOverT = tPrev, leftOverD = 0f, rightOverT = tPrev, rightOverD = 0f;
        for (int s = 1; s <= steps; s++)
        {
            float t = atToNode ? 1f - s / (float)steps : s / (float)steps;
            var pos = graph.EvaluateBezier(edgeIndex, t);
            accumulated += (pos - prev).Length();
            prev = pos;
            tPrev = t;
            if (accumulated > maxWalk) break;

            bool leftOverlap = CornerOverlaps(graph, edgeIndex, t, -halfWidthSelf, clearSq);
            bool rightOverlap = CornerOverlaps(graph, edgeIndex, t, halfWidthSelf, clearSq);

            if (leftOverlap)
            {
                leftPending = true;
                leftOverT = t;
                leftOverD = accumulated;
            }
            else if (leftPending)
            {
                dLeft = RefineClearance(graph, edgeIndex, leftOverT, leftOverD, t, accumulated,
                    -halfWidthSelf, clearSq);
                leftPending = false;
            }

            if (rightOverlap)
            {
                rightPending = true;
                rightOverT = t;
                rightOverD = accumulated;
            }
            else if (rightPending)
            {
                dRight = RefineClearance(graph, edgeIndex, rightOverT, rightOverD, t, accumulated,
                    halfWidthSelf, clearSq);
                rightPending = false;
            }

            if (leftOverlap || rightOverlap) clearStreak = 0;
            else if (++clearStreak >= OverlapClearExitStreak) break;
        }

        // Overlap ran into the cap (or the curve end) without a confirmed release: trim at
        // the walk limit — the caller's MaxDistanceFraction clamp applies the same bound.
        if (leftPending) dLeft = maxWalk;
        if (rightPending) dRight = maxWalk;
    }

    /// <summary>
    /// Bisects the overlap→clear bracket found by the walk (3 iterations ≈ 1/8 step) and
    /// returns the release distance biased to the CLEAR side plus
    /// <see cref="ClearanceMargin"/>, guaranteeing under-length boundary lines. Arc
    /// distance is interpolated linearly inside the (short) bracket.
    /// </summary>
    private float RefineClearance(RoadGraph graph, int edgeIndex, float tOver, float dOver,
        float tClear, float dClear, float cornerOffset, float clearSq)
    {
        for (int i = 0; i < 3; i++)
        {
            float tMid = (tOver + tClear) * 0.5f;
            float dMid = (dOver + dClear) * 0.5f;
            if (CornerOverlaps(graph, edgeIndex, tMid, cornerOffset, clearSq))
            {
                tOver = tMid;
                dOver = dMid;
            }
            else
            {
                tClear = tMid;
                dClear = dMid;
            }
        }
        return dClear + ClearanceMargin;
    }

    /// <summary>True when this edge's boundary corner (lateral <paramref name="cornerOffset"/>,
    /// negative = left of travel) at parameter <paramref name="t"/> lies inside the crossing
    /// road's asphalt (closer than √<paramref name="clearSq"/> to <see cref="_otherPoly"/>).</summary>
    private bool CornerOverlaps(RoadGraph graph, int edgeIndex, float t, float cornerOffset, float clearSq)
    {
        var pos = graph.EvaluateBezier(edgeIndex, t);
        var tan = graph.EvaluateBezierTangent(edgeIndex, t);
        float len = tan.Length();
        if (len < 0.001f) return false;
        float nx = -tan.Y / len, ny = tan.X / len; // right normal (Y-down)
        var pt = new Vector2(pos.X + nx * cornerOffset, pos.Y + ny * cornerOffset);
        return DistSqToOtherPoly(pt, clearSq) < clearSq;
    }

    /// <summary>Squared distance from a point to the crossing road's centerline polyline
    /// (<see cref="_otherPoly"/>, filled by <see cref="WalkOverlapDistances"/>). Returns
    /// early once a segment closer than <paramref name="stopBelow"/> is found — callers
    /// only need to know whether the point is inside that clearance, not the exact
    /// distance beyond it.</summary>
    private float DistSqToOtherPoly(Vector2 p, float stopBelow)
    {
        float best = float.MaxValue;
        for (int i = 0; i < OverlapOtherSamples; i++)
        {
            var a = _otherPoly[i];
            var ab = _otherPoly[i + 1] - a;
            float lenSq = ab.LengthSquared();
            float dsq;
            if (lenSq < 1e-12f)
            {
                dsq = Vector2.DistanceSquared(p, a);
            }
            else
            {
                float t = Math.Clamp(Vector2.Dot(p - a, ab) / lenSq, 0f, 1f);
                dsq = Vector2.DistanceSquared(p, a + ab * t);
            }
            if (dsq < best)
            {
                best = dsq;
                if (best < stopBelow) return best;
            }
        }
        return best;
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
