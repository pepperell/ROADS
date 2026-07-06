using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that builds road segments by clicking to place nodes, using the road
/// toolbar's sticky options (type, per-direction width, one-way, shared-lane).
/// The FIRST click of a chain only RECORDS the start anchor — an existing node, a pending
/// on-road split point, or a pending free position (the latter two drawn as a ghost node)
/// — without mutating the graph, so a right-click/ESC cancel leaves nothing behind.
/// The second click commits: the pending anchor materializes (splitting its edge when
/// on-road), the end anchor resolves the same way, and the edge (pair, unless one-way) is
/// created with the sticky options applied. The chain then continues from the now-real
/// end node.
/// </summary>
public class RoadTool
{
    /// <summary>
    /// Handles a click to place or extend a road segment. The first click of a chain
    /// records the start anchor via <see cref="BeginChain"/> (no graph mutation); each
    /// later click materializes the pending start if any, resolves the clicked end anchor
    /// (snap to node / split nearby edge / new node, within
    /// <see cref="EditorState.SnapDistance"/>), creates the edge — a two-way pair, or a
    /// single directed edge (draw direction) when the one-way option is set — applies the
    /// sticky road options, and splits any crossed edges to form intersections.
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

        // Commit order matters: the start anchor's deferred split runs first, then the
        // end anchor resolves against the CURRENT graph (fresh lookups, grid rebuilt), so
        // an end click on the same edge as the pending start lands on the correct half.
        int startNode = ResolveStartAnchor(graph, state);
        int endNode = ResolveAnchor(worldPos, graph, edgeSpatialGrid);

        if (startNode != endNode)
        {
            int forwardEdge = graph.AddEdge(startNode, endNode);
            if (!state.SelectedOneWay)
                graph.AddEdge(endNode, startNode);

            // Apply the sticky road options BEFORE splitting: the setters mirror to the
            // reverse edge, and SplitEdge copies type/lanes/speed/flags onto both halves.
            graph.SetEdgeRoadType(forwardEdge, state.SelectedRoadType);
            graph.SetLaneCount(forwardEdge, state.SelectedLaneCount);
            if (!state.SelectedOneWay && state.SelectedSharedLane)
                graph.SetSharedLane(forwardEdge, true);

            // Only split forward edge; SplitEdge handles reverse automatically
            SplitAtCrossings(graph, forwardEdge);
        }

        // The (real, committed) end node becomes the start of the next segment.
        state.RoadStartNode = endNode;
        state.RoadStartEdge = -1;
        state.RoadStartAnchorPos = null;
    }

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
    /// mutating the graph: snap node → clamped on-road split → free position.</summary>
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
                return new Anchor(-1, nearEdge, nearT, graph.EvaluateBezier(nearEdge, nearT));
            }
        }

        return new Anchor(-1, -1, 0f, worldPos);
    }

    /// <summary>
    /// World position where a click would anchor — the hover ghost shown at ALL times with
    /// the Road tool: on the snapped existing node, the clamped on-road split point, or
    /// the raw cursor in empty space. Same probe the click uses, so ghost = result.
    /// </summary>
    public static Vector2 ComputeAnchorGhost(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeSpatialGrid)
        => ProbeAnchor(worldPos, graph, edgeSpatialGrid).Position;

    /// <summary>
    /// Records the chain's start anchor WITHOUT mutating the graph: an existing node when
    /// within snap distance, otherwise a pending split point on the nearest road, otherwise
    /// a pending free position — the pending kinds render as a ghost node until committed.
    /// </summary>
    private static void BeginChain(Vector2 worldPos, RoadGraph graph, EditorState state,
        EdgeSpatialGrid? edgeSpatialGrid)
    {
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
    /// invalidate later split positions. Called by <see cref="OnClick"/> after creating a new edge.
    /// </summary>
    /// <param name="graph">Road graph containing the edges.</param>
    /// <param name="newEdgeIndex">Index of the newly created forward edge to check for crossings.</param>
    private static void SplitAtCrossings(RoadGraph graph, int newEdgeIndex)
    {
        var crossings = graph.FindEdgeCrossings(newEdgeIndex);
        if (crossings.Count == 0) return;

        // Sort by tSelf ascending so we can split the new edge from start to end
        crossings.Sort((a, b) => a.tSelf.CompareTo(b.tSelf));

        // Step 1: Split all crossed "other" edges at their crossing points.
        // Track the midNode created at each crossing so we can reuse it in Step 2.
        var midNodes = new int[crossings.Count];
        for (int i = 0; i < crossings.Count; i++)
        {
            var (otherEdge, _, tOther) = crossings[i];
            if (graph.Edges[otherEdge].FromNode < 0)
            {
                midNodes[i] = -1; // already split/defunct
                continue;
            }
            var (midNode, _, _) = graph.SplitEdge(otherEdge, tOther);
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
