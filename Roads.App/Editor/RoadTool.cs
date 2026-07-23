using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that builds road segments by clicking to place nodes, using the road
/// toolbar's sticky options (type, per-direction width, one-way, shared-lane, and the
/// straight/curved drawing mode).
/// The FIRST click of a chain only RECORDS the start anchor — an existing node, a pending
/// on-road split point, or a pending free position (the latter two drawn as a ghost node)
/// — without mutating the graph, so a right-click/ESC cancel leaves nothing behind.
/// The second click commits: the pending anchor materializes (splitting its edge when
/// on-road), the end anchor resolves the same way, and the segment is created with the
/// sticky options applied — routed THROUGH every existing node the drawn geometry passes
/// over (within <see cref="EditorState.SnapDistance"/>, the endpoint snap radius), so
/// drawing across existing nodes commits one leg per hop instead of one disconnected
/// line (see <see cref="PlanRoute"/>, which re-probes recursively as splits bend the
/// legs; the ghost preview shows the same legs).
/// The chain then continues from the now-real end node. In CURVED mode each committed
/// leg leaves its start node tangent to the previous one (see
/// <see cref="TryGetChainTangent"/>), bending as an arc-like Bezier toward its end.
/// </summary>
public class RoadTool
{
    /// <summary>
    /// Handles a click to place or extend a road segment. The first click of a chain
    /// records the start anchor via <see cref="BeginChain"/> (no graph mutation); each
    /// later click materializes the pending start if any, resolves the clicked end anchor
    /// (snap to node / split nearby edge / new node, within
    /// <see cref="EditorState.SnapDistance"/>), and commits the planned legs — one per
    /// pass-through node the drawn geometry crosses, then the final leg to the end anchor
    /// — each a two-way pair (or a single directed edge when the one-way option is set)
    /// with the sticky road options applied and crossed edges split into intersections.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="state">Editor state tracking the in-progress chain anchor.</param>
    /// <param name="edgeSpatialGrid">Optional spatial grid for edge-snap and T-intersection support.</param>
    public void OnClick(Vector2 worldPos, RoadGraph graph, EditorState state, EdgeSpatialGrid? edgeSpatialGrid = null)
    {
        if (!state.IsDrawingRoad)
        {
            BeginChain(worldPos, graph, state, edgeSpatialGrid);
            return;
        }

        // Capture the curved-mode tangent BEFORE the anchors mutate the graph: the end
        // anchor's split could make the previous-segment edge defunct mid-commit.
        Vector2? tangent = state.SelectedCurved ? TryGetChainTangent(graph, state) : null;

        // Plan the route on the SAME geometry the ghost preview showed, BEFORE any
        // mutation: pass-through nodes plus the min-distance-adjusted crossing split
        // points. Existing node indices are stable across the splits below, and pending
        // split points re-locate by position if their host edge is consumed; copied out
        // because the scratch list is shared with the preview path.
        RoutePoint[] routePoints = System.Array.Empty<RoutePoint>();
        var endProbe = ProbeAnchor(worldPos, graph, edgeSpatialGrid);
        Vector2 planStart = state.RoadStartNode is { } sn
            ? graph.Nodes[sn].Position // NaN when defunct mid-draw → no route planned
            : state.RoadStartAnchorPos!.Value;
        if (!float.IsNaN(planStart.X))
        {
            PlanRoute(graph, planStart, state.RoadStartNode ?? -1, state.RoadStartEdge,
                endProbe.Position, endProbe.Node, endProbe.Edge,
                tangent, state.SelectedCurved, CommitLegsScratch, RouteScratch);
            if (RouteScratch.Count > 0) routePoints = RouteScratch.ToArray();
        }

        // Commit order matters: the start anchor's deferred split runs first, each
        // route-point leg commits against the CURRENT graph, and the end anchor resolves
        // LAST (fresh lookups, grid rebuilt), so an end click on the same edge as the
        // pending start lands on the correct half.
        state.RoadStartNode = ResolveStartAnchor(graph, state);
        state.RoadStartEdge = -1;
        state.RoadStartAnchorPos = null;

        foreach (var routePoint in routePoints)
        {
            int node = MaterializeRoutePoint(graph, routePoint, edgeSpatialGrid);
            if (node < 0 || node == state.RoadStartNode)
                continue;
            CommitLeg(graph, state, node, tangent);
            // Further legs continue the chain from the leg just committed.
            tangent = state.SelectedCurved ? TryGetChainTangent(graph, state) : null;
        }

        int endNode = ResolveAnchor(worldPos, graph, edgeSpatialGrid);
        if (endNode != state.RoadStartNode)
            CommitLeg(graph, state, endNode, tangent);
    }

    /// <summary>
    /// Turns a planned route point into a real node at commit time: an existing node
    /// passes through (dropped if it went defunct); a pending split point splits its host
    /// edge at the planned parameter. A host edge consumed by an EARLIER materialization
    /// (two planned splits on one edge, or the start anchor's deferred split) is healed by
    /// position: splits preserve the curve trace, so the point lies exactly on a live
    /// descendant half — re-split there, or reuse a node that already sits on the spot.
    /// Returns -1 when the point cannot be materialized (caller skips its leg).
    /// </summary>
    private static int MaterializeRoutePoint(RoadGraph graph, RoutePoint routePoint,
        EdgeSpatialGrid? edgeSpatialGrid)
    {
        if (routePoint.Node >= 0)
            return !float.IsNaN(graph.Nodes[routePoint.Node].Position.X) ? routePoint.Node : -1;

        int edge = routePoint.SplitEdge;
        float t = routePoint.SplitT;
        if (edge < 0 || edge >= graph.Edges.Count || graph.Edges[edge].FromNode < 0)
        {
            int nearNode = graph.FindNearestNode(routePoint.Pos, 1f);
            if (nearNode >= 0) return nearNode;
            if (edgeSpatialGrid == null) return -1;
            edgeSpatialGrid.RebuildIfNeeded(graph);
            (edge, t) = edgeSpatialGrid.FindNearestEdgeWithT(graph, routePoint.Pos, 1f);
            if (edge < 0) return -1;
        }
        float margin = SplitMarginT(graph, edge);
        var (midNode, _, _) = graph.SplitEdge(edge, Math.Clamp(t, margin, 1f - margin));
        return midNode;
    }

    /// <summary>
    /// Commits one leg of a click — <see cref="EditorState.RoadStartNode"/> to
    /// <paramref name="endNode"/> — with the sticky options applied, curved-mode control
    /// points from <paramref name="tangent"/>, and crossing splits, then advances the
    /// chain (RoadPrevEdge, RoadStartNode). When the two nodes are already connected in
    /// EITHER direction the leg is REUSED: nothing is created (a new segment never
    /// overlaps an existing one) and the chain just advances across the existing road.
    /// The ghost preview omits reuse legs the same way (see <see cref="PlanPreviewLegs"/>).
    /// </summary>
    private static void CommitLeg(RoadGraph graph, EditorState state, int endNode, Vector2? tangent)
    {
        int startNode = state.RoadStartNode!.Value;
        if (startNode == endNode) return;

        int existingForward = FindDirectEdge(graph, startNode, endNode);
        if (existingForward >= 0 || FindDirectEdge(graph, endNode, startNode) >= 0)
        {
            // RoadPrevEdge feeds the next leg's curved-mode tangent; with only the
            // opposite direction existing there is no edge ENDING at endNode to chain
            // from, and TryGetChainTangent falls back to the node-continuation rule.
            state.RoadPrevEdge = existingForward;
            state.RoadStartNode = endNode;
            return;
        }

        int forwardEdge = graph.AddEdge(startNode, endNode);
        if (!state.SelectedOneWay)
            graph.AddEdge(endNode, startNode);

        // Apply the sticky road options BEFORE splitting: the setters mirror to the
        // reverse edge, and SplitEdge copies type/lanes/speed/flags onto both halves.
        graph.SetEdgeRoadType(forwardEdge, state.SelectedRoadType);
        graph.SetLaneCount(forwardEdge, state.SelectedLaneCount);
        if (!state.SelectedOneWay && state.SelectedSharedLane)
            graph.SetSharedLane(forwardEdge, true);

        // Curved mode: bend the leg so it leaves its start node along the tangent,
        // BEFORE splitting — crossing detection samples the edge's actual Bezier, and
        // SplitEdge subdivides it, so the curve shape survives. SetControlPoint mirrors
        // onto the reverse edge and recomputes lengths.
        if (tangent is { } tanDir)
        {
            var (cp1, cp2) = ComputeCurveControls(
                graph.Nodes[startNode].Position, graph.Nodes[endNode].Position, tanDir);
            graph.SetControlPoint(forwardEdge, 1, cp1);
            graph.SetControlPoint(forwardEdge, 2, cp2);
        }

        // Only split forward edge; SplitEdge handles reverse automatically.
        // The trailing half (ending at endNode) is the next leg's tangent source.
        state.RoadPrevEdge = SplitAtCrossings(graph, forwardEdge);
        state.RoadStartNode = endNode;
    }

    /// <summary>Index of an active edge running DIRECTLY from <paramref name="from"/> to
    /// <paramref name="to"/>, or -1.</summary>
    private static int FindDirectEdge(RoadGraph graph, int from, int to)
    {
        foreach (int e in graph.GetOutgoingEdges(from))
            if (graph.Edges[e].ToNode == to) return e;
        return -1;
    }

    /// <summary>
    /// The tangent direction (unit vector, pointing the way the new segment should leave
    /// its start) that curved mode must honor, or <c>null</c> when there is no reference:
    /// the end tangent of the chain's previous segment when one exists, otherwise — for a
    /// chain STARTED on an existing dead-end node — the outward continuation of that
    /// node's single road. Chains started in open space or on a pending on-road anchor
    /// have no reference (a branch tangent to the road it starts on would be degenerate).
    /// </summary>
    public static Vector2? TryGetChainTangent(RoadGraph graph, EditorState state)
    {
        if (state.RoadStartNode is not { } startNode) return null;

        // Previous segment of this chain (kept as the trailing half after splits).
        int prev = state.RoadPrevEdge;
        if (prev >= 0 && prev < graph.Edges.Count
            && graph.Edges[prev].FromNode >= 0 && graph.Edges[prev].ToNode == startNode)
        {
            var tan = graph.EvaluateBezierTangent(prev, 1f);
            if (tan.LengthSquared() > 1e-6f) return Vector2.Normalize(tan);
        }

        return NodeContinuationTangent(graph, startNode);
    }

    /// <summary>
    /// Outward continuation tangent at a DEAD-END node (all incident edges connect to one
    /// neighbor): the direction the existing road is heading when it arrives, so a curved
    /// chain started there extends the road smoothly. <c>null</c> at junctions and
    /// mid-road nodes, where the continuation is ambiguous.
    /// </summary>
    private static Vector2? NodeContinuationTangent(RoadGraph graph, int node)
    {
        int neighbor = -1, inEdge = -1, outEdge = -1;
        foreach (int e in graph.GetIncomingEdges(node))
        {
            int n = graph.Edges[e].FromNode;
            if (neighbor >= 0 && n != neighbor) return null;
            neighbor = n;
            inEdge = e;
        }
        foreach (int e in graph.GetOutgoingEdges(node))
        {
            int n = graph.Edges[e].ToNode;
            if (neighbor >= 0 && n != neighbor) return null;
            neighbor = n;
            outEdge = e;
        }
        if (neighbor < 0) return null; // isolated node — nothing to continue

        // Tangent pointing AWAY from the neighbor: an incoming edge already points into
        // the node (take its end tangent); an outgoing-only edge (one-way starting here)
        // points at the neighbor, so negate its start tangent.
        var tan = inEdge >= 0
            ? graph.EvaluateBezierTangent(inEdge, 1f)
            : -graph.EvaluateBezierTangent(outEdge, 0f);
        return tan.LengthSquared() > 1e-6f ? Vector2.Normalize(tan) : null;
    }

    /// <summary>
    /// Control points for a curved segment from <paramref name="a"/> to <paramref name="b"/>
    /// that leaves <paramref name="a"/> along <paramref name="tangentDir"/> (unit). The end
    /// tangent is the start tangent MIRRORED across the chord — the circular-arc property —
    /// so consecutive curved segments chain into a smooth, constant-feeling bend rather
    /// than straightening out at each node. Handle length d/3 approximates the arc.
    /// </summary>
    public static (Vector2 cp1, Vector2 cp2) ComputeCurveControls(Vector2 a, Vector2 b, Vector2 tangentDir)
    {
        var chord = b - a;
        float d = chord.Length();
        if (d < 0.01f)
            return (a + chord * (1f / 3f), a + chord * (2f / 3f));

        var chordDir = chord / d;
        // Mirror of the start tangent across the chord direction (arc end tangent).
        float dot = Vector2.Dot(tangentDir, chordDir);
        var endDir = chordDir * (2f * dot) - tangentDir;

        return (a + tangentDir * (d / 3f), b - endDir * (d / 3f));
    }

    /// <summary>Cubic control points the planned segment will have: tangent-continuous
    /// arc controls in curved mode (when a tangent reference exists), otherwise the
    /// collinear thirds of a straight segment (an exactly-linear cubic).</summary>
    private static (Vector2 cp1, Vector2 cp2) PlannedControls(Vector2 a, Vector2 b, Vector2? tangent)
        => tangent is { } t
            ? ComputeCurveControls(a, b, t)
            : (a + (b - a) * (1f / 3f), a + (b - a) * (2f / 3f));

    /// <summary>One leg of the road tool's planned commit: cubic geometry (straight legs
    /// use the collinear thirds, tracing the line exactly) plus the existing node the leg
    /// starts at (-1 when it starts at a pending anchor or split point). <see cref="Reuse"/>
    /// marks a leg that already exists as road — its endpoints are connected, or they sit
    /// on the same host edge — so the commit creates nothing there and the preview omits
    /// it from the band/centerline. The preview draws every non-reuse leg and a ghost
    /// node at each interior junction; the crossing probe runs per leg with the leg's own
    /// start node excluded.</summary>
    public readonly record struct PreviewLeg(Vector2 Start, Vector2 Cp1, Vector2 Cp2, Vector2 End,
        int StartNode, bool Reuse);

    /// <summary>One interior stop of a planned route: an EXISTING node
    /// (<see cref="Node"/> ≥ 0), or a PENDING split of <see cref="SplitEdge"/> at
    /// <see cref="SplitT"/> — the intersection node a crossing will create, min-distance
    /// adjusted. <see cref="Pos"/> is always the stop's world position.</summary>
    public readonly record struct RoutePoint(Vector2 Pos, int Node, int SplitEdge, float SplitT)
    {
        public static RoutePoint AtNode(RoadGraph graph, int node)
            => new(graph.Nodes[node].Position, node, -1, 0f);
        public static RoutePoint AtSplit(Vector2 pos, int edge, float t)
            => new(pos, -1, edge, t);
    }

    /// <summary>
    /// Plans the legs the NEXT click would commit, for the ghost preview: the full route
    /// from the chain start through every pass-through node and min-distance-adjusted
    /// crossing point to the snapped end anchor. Appends the pending split positions (the
    /// new intersection nodes the commit will create) to
    /// <paramref name="crossingGhosts"/>. Same planner as the commit, so the ghost shows
    /// the actual segments the click creates rather than the raw start→end line.
    /// </summary>
    public static void PlanPreviewLegs(RoadGraph graph, EditorState state, Vector2 startPos,
        Vector2 endPos, int startNode, int endNode, int endAnchorEdge,
        List<PreviewLeg> legs, List<Vector2> crossingGhosts)
    {
        Vector2? tangent = state.SelectedCurved ? TryGetChainTangent(graph, state) : null;
        PlanRoute(graph, startPos, startNode, state.RoadStartEdge, endPos, endNode, endAnchorEdge,
            tangent, state.SelectedCurved, legs, RouteScratch);
        foreach (var routePoint in RouteScratch)
            if (routePoint.Node < 0)
                crossingGhosts.Add(routePoint.Pos);
    }

    /// <summary>Fixed-point cap for <see cref="PlanRoute"/>'s recursive discovery. Each
    /// extra pass only fires when the previous one found new route points, so real routes
    /// converge in one or two; the cap is a runaway backstop, and the final leg build
    /// always runs after the last accepted discovery.</summary>
    private const int MaxPlanPasses = 8;

    /// <summary>
    /// The single route planner behind both the ghost preview and the commit. Routes the
    /// drawn geometry through every existing node it passes within snap distance of, and
    /// through a planned split point on every edge it crosses — each new split point
    /// enforced to sit at least <see cref="EditorState.SnapDistance"/> from EVERY other
    /// node (existing or planned, start to finish) by sliding it along its host edge away
    /// from the violation until clear, or snapping it onto the host edge's endpoint node
    /// when the slide runs out (see <see cref="PlaceCrossing"/> — the thin-V rule).
    /// Discovery is RECURSIVE: routing through a stop bends the legs away from the
    /// original line, which can bring them within snap distance of further nodes or
    /// across further edges, so it reruns on each rebuilt leg until nothing new is found
    /// (reuse legs are existing roads and are never split). Produces the final ordered
    /// route-point list and the matching legs.
    /// </summary>
    private static void PlanRoute(RoadGraph graph, Vector2 startPos, int startNode, int startAnchorEdge,
        Vector2 endPos, int endNode, int endAnchorEdge, Vector2? tangent, bool curved,
        List<PreviewLeg> legs, List<RoutePoint> route)
    {
        route.Clear();
        UsedNodeScratch.Clear();
        if (startNode >= 0) UsedNodeScratch.Add(startNode);
        if (endNode >= 0) UsedNodeScratch.Add(endNode);

        for (int pass = 0; ; pass++)
        {
            BuildLegs(graph, startPos, startNode, startAnchorEdge, endPos, endNode, endAnchorEdge,
                tangent, curved, route, legs);
            if (pass >= MaxPlanPasses) break;
            if (!DiscoverRoutePoints(graph, legs, route, startPos, endPos, endNode, startAnchorEdge,
                    UsedNodeScratch)) break;
        }
    }

    /// <summary>Builds the leg chain for the current route (start → each route point →
    /// end), with per-leg control points, reuse detection, and tangent chaining.</summary>
    private static void BuildLegs(RoadGraph graph, Vector2 startPos, int startNode, int startAnchorEdge,
        Vector2 endPos, int endNode, int endAnchorEdge, Vector2? tangent, bool curved,
        List<RoutePoint> route, List<PreviewLeg> legs)
    {
        legs.Clear();
        Vector2 cur = startPos;
        int curNode = startNode;
        int curEdge = startNode >= 0 ? -1 : startAnchorEdge;
        foreach (var routePoint in route)
            AddPlannedLeg(graph, legs, ref cur, ref curNode, ref curEdge, ref tangent, curved,
                routePoint.Pos, routePoint.Node, routePoint.SplitEdge);
        AddPlannedLeg(graph, legs, ref cur, ref curNode, ref curEdge, ref tangent, curved,
            endPos, endNode, endNode >= 0 ? -1 : endAnchorEdge);
    }

    /// <summary>One discovery pass of <see cref="PlanRoute"/>: probes every non-reuse leg
    /// for pass-through nodes AND edge crossings on ITS geometry, placing each crossing
    /// with the min-node-distance rule, and splices the finds into the route at that
    /// leg's position (curve-ordered within the leg). The used set keeps each node a stop
    /// at most once, bounding the fixed point. Returns true when anything new was found
    /// (legs must then be rebuilt).</summary>
    private static bool DiscoverRoutePoints(RoadGraph graph, List<PreviewLeg> legs,
        List<RoutePoint> route, Vector2 startPos, Vector2 endPos, int endNode,
        int startAnchorEdge, HashSet<int> used)
    {
        bool added = false;
        NextRouteScratch.Clear();

        // Positions every candidate must keep the minimum distance from: both endpoints
        // and every stop of the current route; accepted candidates are appended as the
        // pass walks, so later finds respect earlier ones.
        PlannedPosScratch.Clear();
        PlannedPosScratch.Add(startPos);
        PlannedPosScratch.Add(endPos);
        foreach (var routePoint in route)
            PlannedPosScratch.Add(routePoint.Pos);

        // NODE PRIORITY: crossings are only placed once the legs are stable w.r.t.
        // pass-through nodes. When the geometry passes near a junction AND crosses that
        // junction's road beside it, routing through the node IS the whole answer —
        // placing the crossing too would add a second stop a minimum-distance away and
        // commit a zig-zag through both. Deferring crossings one pass lets the legs
        // rebuild through the node first; a crossing that still exists on the rebuilt
        // leg is real and gets placed next pass.
        bool nodesThisPass = false;
        for (int i = 0; i < legs.Count && !nodesThisPass; i++)
        {
            var probeLeg = legs[i];
            if (probeLeg.Reuse) continue;
            int probeEndNode = i < legs.Count - 1 ? route[i].Node : endNode;
            FindPassThroughNodes(graph, probeLeg.Start, probeLeg.Cp1, probeLeg.Cp2, probeLeg.End,
                probeLeg.StartNode, probeEndNode, PassCandidateScratch);
            foreach (var (_, node) in PassCandidateScratch)
                if (!used.Contains(node)) { nodesThisPass = true; break; }
        }

        for (int i = 0; i < legs.Count; i++)
        {
            if (i > 0) NextRouteScratch.Add(route[i - 1]); // the stop this leg starts at

            var leg = legs[i];
            if (leg.Reuse) continue; // an existing road — not ours to split
            int legEndNode = i < legs.Count - 1 ? route[i].Node : endNode;

            FindPassThroughNodes(graph, leg.Start, leg.Cp1, leg.Cp2, leg.End,
                leg.StartNode, legEndNode, PassCandidateScratch);
            if (nodesThisPass)
                CrossCandidateScratch.Clear(); // deferred to the next pass (see above)
            else
                graph.FindCurveCrossingsDetailed(leg.Start, leg.Cp1, leg.Cp2, leg.End,
                    leg.StartNode, i == 0 ? startAnchorEdge : -1, CrossCandidateScratch);

            // Merge both kinds ordered along the leg so min-distance placement sees its
            // predecessors first (Node ≥ 0 → pass-through; else crossing on Edge at TOther).
            MergedCandidateScratch.Clear();
            foreach (var (t, node) in PassCandidateScratch)
                MergedCandidateScratch.Add((t, node, -1, 0f));
            foreach (var (otherEdge, tSelf, tOther, _) in CrossCandidateScratch)
                MergedCandidateScratch.Add((tSelf, -1, otherEdge, tOther));
            MergedCandidateScratch.Sort(static (a, b) => a.T.CompareTo(b.T));

            foreach (var candidate in MergedCandidateScratch)
            {
                RoutePoint routePoint;
                if (candidate.Node >= 0)
                {
                    if (!used.Add(candidate.Node)) continue;
                    routePoint = RoutePoint.AtNode(graph, candidate.Node);
                }
                else
                {
                    if (PlaceCrossing(graph, candidate.Edge, candidate.TOther, used) is not { } placed)
                        continue;
                    if (placed.Node >= 0 && !used.Add(placed.Node)) continue;
                    routePoint = placed;
                }
                NextRouteScratch.Add(routePoint);
                PlannedPosScratch.Add(routePoint.Pos);
                added = true;
            }
        }

        if (added)
        {
            route.Clear();
            route.AddRange(NextRouteScratch);
        }
        return added;
    }

    /// <summary>How far (m) <see cref="PlaceCrossing"/> will slide a planned crossing
    /// along its host edge, each direction, hunting for a spot clear of every node.</summary>
    private const float MaxCrossingSlide = 100f;

    /// <summary>Slide granularity (m) for <see cref="PlaceCrossing"/>.</summary>
    private const float CrossingSlideStep = 0.5f;

    /// <summary>
    /// Places the intersection node a crossing will create on its host edge, enforcing
    /// the minimum node spacing (<see cref="EditorState.SnapDistance"/>): starting from
    /// the geometric crossing, the split point slides along the host edge — nearest valid
    /// spot first, either direction — until it clears every existing node and every
    /// already-planned position (the thin-V tip rule: both arms' crossings get pulled
    /// away from the tip until all spacing criteria hold). If no clear spot exists within
    /// range, the crossing SNAPS to a host-edge endpoint node instead (preferring one not
    /// already on the route). Returns null when nothing works (the crossing is dropped —
    /// the commit's per-leg crossing split remains as the safety net).
    /// </summary>
    private static RoutePoint? PlaceCrossing(RoadGraph graph, int edge, float tCross, HashSet<int> used)
    {
        var hostEdge = graph.Edges[edge];
        float len = MathF.Max(hostEdge.Length, 0.01f);
        float marginT = SplitMarginT(graph, edge);
        float minDist = EditorState.SnapDistance;
        float minDistSq = minDist * minDist;
        var crossPos = graph.EvaluateBezier(edge, tCross);

        // Obstacles: every active node within slide reach (the host edge's endpoint nodes
        // are always among them, so endpoint spacing is enforced by the same check).
        float reach = MathF.Min(MaxCrossingSlide, len) + minDist;
        float reachSq = reach * reach;
        ObstacleScratch.Clear();
        var nodes = graph.Nodes;
        for (int n = 0; n < nodes.Count; n++)
        {
            var pos = nodes[n].Position;
            if (float.IsNaN(pos.X)) continue;
            if (Vector2.DistanceSquared(pos, crossPos) <= reachSq)
                ObstacleScratch.Add(pos);
        }

        bool Valid(float t, out Vector2 pos)
        {
            pos = graph.EvaluateBezier(edge, t);
            foreach (var obstacle in ObstacleScratch)
                if (Vector2.DistanceSquared(pos, obstacle) < minDistSq) return false;
            foreach (var planned in PlannedPosScratch)
                if (Vector2.DistanceSquared(pos, planned) < minDistSq) return false;
            return true;
        }

        // March outward from the crossing, alternating directions so the nearest valid
        // spot wins; both bounds are the split margins near the endpoint nodes.
        float dt = CrossingSlideStep / len;
        int maxSteps = (int)(MathF.Min(MaxCrossingSlide, len) / CrossingSlideStep) + 1;
        for (int step = 0; step <= maxSteps; step++)
        {
            float tUp = tCross + step * dt;
            if (tUp <= 1f - marginT && Valid(tUp, out var posUp))
                return RoutePoint.AtSplit(posUp, edge, tUp);
            if (step == 0) continue;
            float tDown = tCross - step * dt;
            if (tDown >= marginT && Valid(tDown, out var posDown))
                return RoutePoint.AtSplit(posDown, edge, tDown);
        }

        // No clear spot — snap onto an endpoint node of the host edge instead, preferring
        // one that is not already part of the route.
        int nearEnd = tCross < 0.5f ? hostEdge.FromNode : hostEdge.ToNode;
        int farEnd = tCross < 0.5f ? hostEdge.ToNode : hostEdge.FromNode;
        if (!used.Contains(nearEnd)) return RoutePoint.AtNode(graph, nearEnd);
        if (!used.Contains(farEnd)) return RoutePoint.AtNode(graph, farEnd);
        return null;
    }

    /// <summary>Appends one planned leg and advances the chaining state the same way the
    /// commit does: the next leg starts at this leg's end, and in curved mode continues
    /// along this leg's end tangent — the cubic derivative direction at t=1 (P3 − CP2)
    /// for a leg the commit will create, or the REAL existing road's continuation for a
    /// reuse leg, mirroring what <see cref="TryGetChainTangent"/> will see after that leg
    /// commits. A leg is REUSE when road already exists between its endpoints: the two
    /// nodes are directly connected, or the endpoints share a host edge (two planned
    /// splits of one edge, or a planned split and its own edge's endpoint node) — after
    /// materialization those are adjacent by construction.</summary>
    private static void AddPlannedLeg(RoadGraph graph, List<PreviewLeg> legs, ref Vector2 cur,
        ref int curNode, ref int curEdge, ref Vector2? tangent, bool curved,
        Vector2 end, int endNode, int endSplitEdge)
    {
        int existingForward = curNode >= 0 && endNode >= 0 ? FindDirectEdge(graph, curNode, endNode) : -1;
        bool reuse;
        if (curNode >= 0 && endNode >= 0)
            reuse = existingForward >= 0 || FindDirectEdge(graph, endNode, curNode) >= 0;
        else if (curEdge >= 0 && endSplitEdge >= 0)
            reuse = SameRoad(graph, curEdge, endSplitEdge);
        else if (curEdge >= 0 && endNode >= 0)
            reuse = IsEdgeEndpoint(graph, curEdge, endNode);
        else if (endSplitEdge >= 0 && curNode >= 0)
            reuse = IsEdgeEndpoint(graph, endSplitEdge, curNode);
        else
            reuse = false;

        var (cp1, cp2) = PlannedControls(cur, end, tangent);
        legs.Add(new PreviewLeg(cur, cp1, cp2, end, curNode, reuse));

        if (curved)
        {
            if (!reuse)
            {
                // A straight-committed leg (no tangent reference) still hands the next
                // leg its end direction — exactly what the trailing real edge will report.
                var outDir = end - cp2;
                if (outDir.LengthSquared() > 1e-6f) tangent = Vector2.Normalize(outDir);
            }
            else if (existingForward >= 0)
            {
                var t = graph.EvaluateBezierTangent(existingForward, 1f);
                tangent = t.LengthSquared() > 1e-6f ? Vector2.Normalize(t) : null;
            }
            else if (curNode >= 0 && endNode >= 0)
            {
                // Only the opposite direction exists — no edge ends at the junction, so
                // the commit's tangent lookup falls back to the node-continuation rule.
                tangent = NodeContinuationTangent(graph, endNode);
            }
            else
            {
                // Reuse along a host edge (split points/endpoints of one road): the
                // continuation direction is the road's own — the leg chord approximates
                // it well enough for the preview.
                var dir = end - cur;
                tangent = dir.LengthSquared() > 1e-6f ? Vector2.Normalize(dir) : tangent;
            }
        }

        cur = end;
        curNode = endNode;
        curEdge = endSplitEdge;
    }

    /// <summary>True when the two edge indices are the same road (identical, or a
    /// two-way pair's twins).</summary>
    private static bool SameRoad(RoadGraph graph, int e1, int e2)
        => e1 == e2 || graph.FindReverseEdge(e1) == e2;

    /// <summary>True when <paramref name="node"/> is an endpoint of <paramref name="edge"/>.</summary>
    private static bool IsEdgeEndpoint(RoadGraph graph, int edge, int node)
        => graph.Edges[edge].FromNode == node || graph.Edges[edge].ToNode == node;

    /// <summary>
    /// Finds the existing nodes the planned leg passes over — active nodes within
    /// <see cref="EditorState.SnapDistance"/> of the cubic (the SAME snap radius the
    /// endpoint anchors use), excluding the endpoints' own nodes and anything within
    /// <see cref="SimConstants.MinSplitSetback"/> meters of either end (the end anchors
    /// own that zone; a stop butted against an end would make a degenerate stub leg).
    /// Results are (leg parameter, node) pairs ordered by curve parameter, ready to merge
    /// with the leg's crossings. Shared by the ghost preview and the commit planner.
    /// </summary>
    private static void FindPassThroughNodes(RoadGraph graph, Vector2 p0, Vector2 c1, Vector2 c2,
        Vector2 p3, int excludeA, int excludeB, List<(float T, int Node)> result)
    {
        result.Clear();

        // Sample the cubic once (same 20-segment fidelity as crossing detection) and
        // accumulate its arc length for the end-setback tests below.
        const int segments = 20;
        Span<Vector2> pts = stackalloc Vector2[segments + 1];
        float len = 0f;
        for (int i = 0; i <= segments; i++)
        {
            float t = (float)i / segments, u = 1f - t;
            pts[i] = p0 * (u * u * u) + c1 * (3f * u * u * t) + c2 * (3f * u * t * t) + p3 * (t * t * t);
            if (i > 0) len += Vector2.Distance(pts[i - 1], pts[i]);
        }
        if (len <= 2f * SimConstants.MinSplitSetback) return; // no room for an interior stop

        float snapSq = EditorState.SnapDistance * EditorState.SnapDistance;
        float setbackSq = SimConstants.MinSplitSetback * SimConstants.MinSplitSetback;

        // Hull AABB (the cubic lies inside its control hull) inflated by the snap radius.
        float minX = MathF.Min(MathF.Min(p0.X, p3.X), MathF.Min(c1.X, c2.X)) - EditorState.SnapDistance;
        float maxX = MathF.Max(MathF.Max(p0.X, p3.X), MathF.Max(c1.X, c2.X)) + EditorState.SnapDistance;
        float minY = MathF.Min(MathF.Min(p0.Y, p3.Y), MathF.Min(c1.Y, c2.Y)) - EditorState.SnapDistance;
        float maxY = MathF.Max(MathF.Max(p0.Y, p3.Y), MathF.Max(c1.Y, c2.Y)) + EditorState.SnapDistance;

        var nodes = graph.Nodes;
        for (int n = 0; n < nodes.Count; n++)
        {
            if (n == excludeA || n == excludeB) continue;
            var pos = nodes[n].Position;
            if (float.IsNaN(pos.X)) continue; // defunct
            if (pos.X < minX || pos.X > maxX || pos.Y < minY || pos.Y > maxY) continue;
            if (Vector2.DistanceSquared(pos, p0) < setbackSq
                || Vector2.DistanceSquared(pos, p3) < setbackSq) continue;

            // Closest approach to the sampled polyline, tracking the curve parameter.
            float bestDistSq = float.MaxValue, bestT = 0f;
            for (int i = 0; i < segments; i++)
            {
                var a = pts[i];
                var ab = pts[i + 1] - a;
                float abLenSq = ab.LengthSquared();
                float u = abLenSq > 1e-12f ? Math.Clamp(Vector2.Dot(pos - a, ab) / abLenSq, 0f, 1f) : 0f;
                float dSq = Vector2.DistanceSquared(pos, a + ab * u);
                if (dSq < bestDistSq)
                {
                    bestDistSq = dSq;
                    bestT = (i + u) / segments;
                }
            }
            if (bestDistSq > snapSq) continue;
            // Interior hits only — same fixed-distance end rule the crossing setback uses.
            if (bestT * len < SimConstants.MinSplitSetback
                || (1f - bestT) * len < SimConstants.MinSplitSetback) continue;

            result.Add((bestT, n));
        }

        result.Sort(static (x, y) => x.T.CompareTo(y.T));
    }

    // ── Planner scratch buffers ─────────────────────────────────────────────
    // All UI-thread only, like every editor tool. The commit copies RouteScratch out
    // before mutating the graph (the preview shares it on the next hover pass).

    /// <summary>Route result buffer, shared by the preview plan and the commit.</summary>
    private static readonly List<RoutePoint> RouteScratch = new();

    /// <summary>Next-pass route being spliced together by <see cref="DiscoverRoutePoints"/>.</summary>
    private static readonly List<RoutePoint> NextRouteScratch = new();

    /// <summary>Nodes already fixed into the route (endpoints + accepted stops) —
    /// <see cref="DiscoverRoutePoints"/>'s dedup/termination set.</summary>
    private static readonly HashSet<int> UsedNodeScratch = new();

    /// <summary>Positions of every planned stop plus both endpoints — the min-distance
    /// context for <see cref="PlaceCrossing"/>.</summary>
    private static readonly List<Vector2> PlannedPosScratch = new();

    /// <summary>Existing-node obstacle positions for one <see cref="PlaceCrossing"/> call.</summary>
    private static readonly List<Vector2> ObstacleScratch = new();

    /// <summary>Per-leg pass-through discovery buffer for <see cref="DiscoverRoutePoints"/>.</summary>
    private static readonly List<(float T, int Node)> PassCandidateScratch = new();

    /// <summary>Per-leg crossing discovery buffer for <see cref="DiscoverRoutePoints"/>.</summary>
    private static readonly List<(int otherEdge, float tSelf, float tOther, Vector2 pos)> CrossCandidateScratch = new();

    /// <summary>Merged, leg-ordered candidates (Node ≥ 0 → pass-through, else crossing).</summary>
    private static readonly List<(float T, int Node, int Edge, float TOther)> MergedCandidateScratch = new();

    /// <summary>Leg buffer for the COMMIT's route plan (the preview owns
    /// <see cref="EditorState.RoadPreviewLegs"/>; the commit only needs the route points
    /// but <see cref="PlanRoute"/> plans legs to run discovery on).</summary>
    private static readonly List<PreviewLeg> CommitLegsScratch = new();

    /// <summary>
    /// Where a click at <paramref name="worldPos"/> would anchor: an existing node
    /// (<see cref="Node"/> ≥ 0), a clamped split point on the nearest road
    /// (<see cref="Edge"/> ≥ 0), or a free position. <see cref="Position"/> is always the
    /// anchor's world position. Shared by the first click (<see cref="BeginChain"/>), the
    /// commit (<see cref="ResolveAnchor"/>), and the hover ghost
    /// (<see cref="ComputeAnchorGhost"/>) so all three always agree.
    /// </summary>
    private readonly record struct Anchor(int Node, int Edge, float T, Vector2 Position);

    /// <summary>Probes the anchor a click at <paramref name="worldPos"/> would use, without
    /// mutating the graph: snap node → clamped on-road split → free position. An on-road
    /// split landing within <see cref="EditorState.SnapDistance"/> of its edge's endpoint
    /// node snaps to that node instead — the minimum node spacing applies to the anchors
    /// exactly as it does to planned crossings.</summary>
    private static Anchor ProbeAnchor(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid? edgeSpatialGrid)
    {
        int existingNode = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (existingNode >= 0)
            return new Anchor(existingNode, -1, 0f, graph.Nodes[existingNode].Position);

        if (edgeSpatialGrid != null)
        {
            edgeSpatialGrid.RebuildIfNeeded(graph);
            var (nearEdge, nearT) = edgeSpatialGrid.FindNearestEdgeWithT(graph, worldPos, EditorState.SnapDistance);
            if (nearEdge >= 0)
            {
                nearT = Math.Clamp(nearT, SplitMarginT(graph, nearEdge), 1f - SplitMarginT(graph, nearEdge));
                var pos = graph.EvaluateBezier(nearEdge, nearT);

                // Minimum node spacing: never create an on-road anchor node within snap
                // distance of the edge's own endpoint — use the endpoint itself.
                var edge = graph.Edges[nearEdge];
                float snapSq = EditorState.SnapDistance * EditorState.SnapDistance;
                float dFrom = Vector2.DistanceSquared(pos, graph.Nodes[edge.FromNode].Position);
                float dTo = Vector2.DistanceSquared(pos, graph.Nodes[edge.ToNode].Position);
                if (dFrom < snapSq || dTo < snapSq)
                {
                    int endpoint = dFrom <= dTo ? edge.FromNode : edge.ToNode;
                    return new Anchor(endpoint, -1, 0f, graph.Nodes[endpoint].Position);
                }
                return new Anchor(-1, nearEdge, nearT, pos);
            }
        }

        return new Anchor(-1, -1, 0f, worldPos);
    }

    /// <summary>
    /// The anchor a click at <paramref name="worldPos"/> would use, for the hover pass:
    /// the position feeds the ghost node shown at ALL times with the Road tool (snapped
    /// existing node, clamped on-road split point, or raw cursor in empty space); the
    /// snapped node / host edge feed the route planner. Same probe the click uses, so
    /// ghost = result.
    /// </summary>
    public static (Vector2 Position, int Node, int Edge) ProbeAnchorInfo(
        Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeSpatialGrid)
    {
        var anchor = ProbeAnchor(worldPos, graph, edgeSpatialGrid);
        return (anchor.Position, anchor.Node, anchor.Edge);
    }

    /// <summary>
    /// Records the chain's start anchor WITHOUT mutating the graph: an existing node when
    /// within snap distance, otherwise a pending split point on the nearest road, otherwise
    /// a pending free position — the pending kinds render as a ghost node until committed.
    /// </summary>
    private static void BeginChain(Vector2 worldPos, RoadGraph graph, EditorState state,
        EdgeSpatialGrid? edgeSpatialGrid)
    {
        state.RoadPrevEdge = -1; // fresh chain: no previous segment yet
        var anchor = ProbeAnchor(worldPos, graph, edgeSpatialGrid);
        if (anchor.Node >= 0)
        {
            state.RoadStartNode = anchor.Node;
            return;
        }
        state.RoadStartEdge = anchor.Edge;
        state.RoadStartT = anchor.T;
        state.RoadStartAnchorPos = anchor.Position;
    }

    /// <summary>
    /// Materializes the pending start anchor at commit time: an existing node passes
    /// through; a pending on-road anchor splits its edge NOW (deferred from the first
    /// click); a pending free anchor adds its node. A pending edge that went stale between
    /// clicks (defunct/out of range) falls back to a free node at the recorded ghost
    /// position, so the committed road always starts where the ghost showed.
    /// </summary>
    private static int ResolveStartAnchor(RoadGraph graph, EditorState state)
    {
        if (state.RoadStartNode is { } startNode) return startNode;

        int edge = state.RoadStartEdge;
        if (edge >= 0 && edge < graph.Edges.Count && graph.Edges[edge].FromNode >= 0)
        {
            var (midNode, _, _) = graph.SplitEdge(edge, state.RoadStartT);
            return midNode;
        }
        return graph.AddNode(state.RoadStartAnchorPos!.Value);
    }

    /// <summary>
    /// Resolves a click position to a node, mutating as needed: snaps to an existing node,
    /// splits the nearest road within snap distance, or adds a new node. The split is
    /// clamped a fixed DISTANCE from the endpoints (not a t-fraction, which grows with
    /// road length) so long roads can be split close to where the user clicked. The probe
    /// rebuilds the edge grid first — the start anchor's split may have just changed the graph.
    /// </summary>
    private static int ResolveAnchor(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid? edgeSpatialGrid)
    {
        var anchor = ProbeAnchor(worldPos, graph, edgeSpatialGrid);
        if (anchor.Node >= 0) return anchor.Node;
        if (anchor.Edge >= 0)
        {
            var (midNode, _, _) = graph.SplitEdge(anchor.Edge, anchor.T);
            return midNode;
        }
        return graph.AddNode(anchor.Position);
    }

    /// <summary>
    /// Detects where the new edge crosses existing edges and splits both at each crossing.
    /// Crossings are processed in ascending parametric order so that earlier splits don't
    /// invalidate later split positions; a segment that crosses the SAME existing edge
    /// more than once re-anchors the later crossings onto the surviving half (split
    /// records with exact t remapping), so every crossing gets its intersection node.
    /// Called by <see cref="OnClick"/> after creating a new edge.
    /// </summary>
    /// <param name="graph">Road graph containing the edges.</param>
    /// <param name="newEdgeIndex">Index of the newly created forward edge to check for crossings.</param>
    /// <returns>The trailing piece of the new edge — the half ending at the segment's end
    /// node after all splits (the new edge itself when nothing crossed). Curved mode uses
    /// its end tangent to continue the chain.</returns>
    private static int SplitAtCrossings(RoadGraph graph, int newEdgeIndex)
    {
        var crossings = graph.FindEdgeCrossings(newEdgeIndex);
        if (crossings.Count == 0) return newEdgeIndex;

        // Sort by tSelf ascending so we can split the new edge from start to end
        crossings.Sort((a, b) => a.tSelf.CompareTo(b.tSelf));

        // Step 1: Split all crossed "other" edges at their crossing points.
        // Track the midNode created at each crossing so we can reuse it in Step 2.
        // A curved segment can cross the SAME existing edge more than once; each split
        // defuncts the edge it splits, so later crossings of that edge re-anchor onto
        // the live descendant half via the split records below. The t remap is EXACT:
        // De Casteljau subdivision at T yields halves tracing the original curve on
        // [0,T] and [T,1], so original t maps to t/T on the first half and
        // (t-T)/(1-T) on the second. (Halves are appended, never slot-reused, so a
        // record chain always walks forward and terminates.)
        var splits = new Dictionary<int, (int FirstHalf, int SecondHalf, float T)>();
        var midNodes = new int[crossings.Count];
        for (int i = 0; i < crossings.Count; i++)
        {
            var (otherEdge, _, tOther) = crossings[i];

            // Re-anchor through any splits earlier crossings made to this edge.
            while (splits.TryGetValue(otherEdge, out var rec))
            {
                if (tOther <= rec.T)
                {
                    otherEdge = rec.FirstHalf;
                    tOther /= MathF.Max(rec.T, 1e-6f);
                }
                else
                {
                    otherEdge = rec.SecondHalf;
                    tOther = (tOther - rec.T) / MathF.Max(1f - rec.T, 1e-6f);
                }
            }

            if (graph.Edges[otherEdge].FromNode < 0)
            {
                midNodes[i] = -1; // defunct outside this pass's records — skip
                continue;
            }

            // The remapped t can land near the descendant's endpoint (two crossings
            // close together on the original edge), where an unclamped split would be
            // degenerate — clamp by the same distance margin Step 2 uses.
            float margin = SplitMarginT(graph, otherEdge);
            float tClamped = Math.Clamp(tOther, margin, 1f - margin);

            var (midNode, firstHalf, secondHalf) = graph.SplitEdge(otherEdge, tClamped);
            splits[otherEdge] = (firstHalf, secondHalf, tClamped);
            midNodes[i] = midNode;
        }

        // Step 2: Split the new edge at each tSelf, ascending.
        // Reuse the midNodes from Step 1 so both roads share the same intersection node.
        int currentEdge = newEdgeIndex;
        float consumedT = 0f;

        for (int i = 0; i < crossings.Count; i++)
        {
            if (midNodes[i] < 0) continue; // skipped crossing
            if (graph.Edges[currentEdge].FromNode < 0) break; // edge already defunct

            float tSelf = crossings[i].tSelf;

            // Rescale t into the remaining portion of the curve
            float localT = (tSelf - consumedT) / (1f - consumedT);
            float margin = SplitMarginT(graph, currentEdge);
            localT = Math.Clamp(localT, margin, 1f - margin);

            var (_, _, secondHalf) = graph.SplitEdge(currentEdge, localT, midNodes[i]);

            // The second half (higher t) continues to be split
            currentEdge = secondHalf;
            consumedT = tSelf;
        }

        return currentEdge;
    }

    /// <summary>
    /// Parametric margin corresponding to <see cref="SimConstants.MinSplitSetback"/> meters
    /// at each end of an edge, capped at 0.45 so the clamp range stays valid on very short
    /// edges. Distance-based so the un-splittable zone near a node does not scale with length.
    /// </summary>
    private static float SplitMarginT(RoadGraph graph, int edgeIndex)
    {
        float len = MathF.Max(graph.Edges[edgeIndex].Length, 0.01f);
        return MathF.Min(0.45f, SimConstants.MinSplitSetback / len);
    }

    /// <summary>
    /// Cancels the in-progress road by resetting all tool state.
    /// </summary>
    /// <param name="state">Editor state to reset.</param>
    public void OnCancel(EditorState state)
    {
        state.ResetToolState();
    }
}
