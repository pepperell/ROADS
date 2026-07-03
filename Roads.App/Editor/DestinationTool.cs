using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that places or removes destination POI markers.
/// Two modes, disambiguated by hit-test (the caller tries <see cref="OnClick"/> first and
/// falls back to <see cref="PlaceAndConnect"/> when it returns false):
/// <list type="bullet">
/// <item><see cref="OnClick"/> — legacy "flag an existing node" path. Flags the nearest eligible
/// UNMARKED node (≤ 2 outgoing edges, no existing destination flag) as a destination. It
/// never snaps to an already-flagged node — that returns false so placement runs instead.
/// Removing a destination is done via right-click (RemoveNearestDestination).</item>
/// <item><see cref="PlaceAndConnect"/> — placement path used when no eligible UNMARKED node is
/// under the cursor: splits the nearest edge and drops a new destination node linked by a connector.</item>
/// </list>
/// </summary>
public class DestinationTool
{
    /// <summary>
    /// Flags the nearest eligible UNMARKED node within snap distance as a destination of
    /// <paramref name="poiType"/>. Returns false when there is no such node — including when the
    /// nearest node is already a destination, which is deliberately NOT snapped to — so the
    /// caller falls through to <see cref="PlaceAndConnect"/> and places a new destination
    /// instead of grabbing/retyping the existing one.
    /// </summary>
    /// <returns>True if a node was flagged.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, POIType poiType)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.CanPlaceMarker(node)) return false;

        var flags = graph.Nodes[node].Flags;
        // Never snap to a node that already carries a destination marker.
        if ((flags & NodeFlags.Destination) != 0) return false;

        graph.SetNodeFlags(node, flags | NodeFlags.Destination);
        graph.SetNodePOIType(node, poiType);
        return true;
    }

    /// <summary>
    /// Commits a placed destination: splits the nearest edge at the foot point to create an
    /// on-road node, adds a new destination node at <paramref name="cursorWorld"/> with the given
    /// POI type, and connects the two with a connector road.
    ///
    /// Home POIs get a residential-driveway treatment: the connector is a <b>dirt, single-lane
    /// two-way</b> road, and placing it adds <b>exactly one stop</b> — on the new driveway approach.
    /// The node is flagged StopSign|ManualSignal (manual so the normalize phase won't promote it to
    /// an all-way stop); the through road the driveway joins keeps flowing (the split's two new
    /// halves are exempted); and no pre-existing approach is touched, so a second home added to an
    /// existing junction keeps the first driveway's stop and the through road — regardless of road
    /// type — never gains a spurious stop. Other POI types keep a plain two-way connector.
    ///
    /// Index-safety: <see cref="RoadGraph.SplitEdge"/> invalidates edge indices and rebuilds
    /// adjacency, so the split is performed FIRST and only its returned (stable) node/edge indices
    /// are carried forward. <see cref="RoadGraph.AddNode"/> returns a stable node index; subsequent
    /// <see cref="RoadGraph.AddEdge"/> calls return stable edge indices and do not invalidate
    /// node indices.
    /// </summary>
    /// <param name="cursorWorld">World position of the new destination node.</param>
    /// <param name="nearEdge">Nearest edge index (must be &gt;= 0; caller already validated).</param>
    /// <param name="nearT">Unclamped parametric foot position on <paramref name="nearEdge"/>.</param>
    /// <param name="graph">Road graph.</param>
    /// <param name="poiType">POI type to assign to the new destination node.</param>
    /// <param name="stopSigns">Stop-sign system, for the driveway stop + through-road exemptions (Home only).</param>
    /// <returns>True if a destination was placed.</returns>
    public bool PlaceAndConnect(Vector2 cursorWorld, int nearEdge, float nearT,
        RoadGraph graph, POIType poiType, StopSignSystem stopSigns)
    {
        if (nearEdge < 0 || nearEdge >= graph.Edges.Count || graph.Edges[nearEdge].FromNode < 0)
            return false;

        // Determine the on-road node. Reuse an endpoint ONLY when it is unflagged and the foot is
        // within the split setback of it; otherwise split at the clamped t. A flagged endpoint (an
        // existing destination node) is treated as non-existent: rather than reuse it, we clamp the
        // foot onto the road just shy of it and split there, so the connector attaches to the road
        // (the dirt driveway or the original road) but never extends from another marked node.
        int reuse = ProspectiveFootNode(graph, nearEdge, nearT);
        int footNode;
        // The through road's two incoming halves at the new node when we split (the road the
        // driveway joins). -1 when we reuse an existing node (its approaches are already set up).
        int throughInA = -1, throughInB = -1;
        if (reuse >= 0 && (graph.Nodes[reuse].Flags & NodeFlags.Destination) == 0)
        {
            footNode = reuse;                                // reuse unflagged endpoint, no split
        }
        else
        {
            float t = ClampedSplitT(graph, nearEdge, nearT);
            var (midNode, firstHalf, secondHalf) = graph.SplitEdge(nearEdge, t);  // SPLIT FIRST — invalidates edge indices
            footNode = midNode;
            throughInA = firstHalf;                          // F -> footNode (incoming)
            throughInB = graph.FindReverseEdge(secondHalf);  // T -> footNode (incoming; -1 if one-way)
        }

        // Add the destination node at the cursor (node index is stable hereafter).
        int destNode = graph.AddNode(cursorWorld);
        var f = graph.Nodes[destNode].Flags;
        graph.SetNodeFlags(destNode, f | NodeFlags.Destination);
        graph.SetNodePOIType(destNode, poiType);

        // Two-way connector between the on-road node and the destination node.
        int connOut = graph.AddEdge(footNode, destNode);   // intersection -> home
        int connIn = graph.AddEdge(destNode, footNode);    // home -> intersection (the driveway that stops)

        if (poiType == POIType.Home)
        {
            // Dirt, single-lane two-way driveway (both setters sync the reverse edge).
            graph.SetEdgeRoadType(connOut, RoadType.Dirt);
            graph.SetSharedLane(connOut, true);

            // Add exactly ONE stop — the new driveway — without disturbing any existing approach.
            // The through road the driveway joins must keep flowing: when we split a road to attach,
            // its two new halves (throughInA/B) would otherwise stop at the new stop node, so exempt
            // them. At a reused node there is no split, so existing approaches (a main road, or
            // another home's driveway) keep their stop/exempt status untouched.
            var nodeFlags = graph.Nodes[footNode].Flags;
            graph.SetNodeFlags(footNode, nodeFlags | NodeFlags.StopSign | NodeFlags.ManualSignal);
            stopSigns.SetEdgeExempt(connIn, false);                       // the new driveway stops
            if (throughInA >= 0) stopSigns.SetEdgeExempt(throughInA, true);
            if (throughInB >= 0) stopSigns.SetEdgeExempt(throughInB, true);
        }

        return true;
    }

    /// <summary>
    /// Minimum distance (m) the connector's split keeps from a FLAGGED endpoint — larger than the
    /// normal <see cref="SimConstants.MinSplitSetback"/> so a new junction is never created right up
    /// against an existing destination node when attaching to a road that ends at one.
    /// </summary>
    private const float FlaggedNodeSetback = 10f;

    /// <summary>Required setback (m) from one edge endpoint: the larger flagged setback when that
    /// node is a destination, otherwise the normal split setback.</summary>
    private static float EndSetback(RoadGraph graph, int nodeIndex)
    {
        if (nodeIndex >= 0 && nodeIndex < graph.Nodes.Count
            && (graph.Nodes[nodeIndex].Flags & NodeFlags.Destination) != 0)
            return FlaggedNodeSetback;
        return SimConstants.MinSplitSetback;
    }

    /// <summary>
    /// Parametric split position for a foot at (<paramref name="nearEdge"/>, <paramref name="nearT"/>),
    /// clamped away from each endpoint by <see cref="EndSetback"/> — the larger flagged setback on a
    /// side whose endpoint is a marked node, the normal setback otherwise. On a too-short edge the two
    /// setbacks overlap, so the split lands at the midpoint of the overlap.
    /// </summary>
    private static float ClampedSplitT(RoadGraph graph, int nearEdge, float nearT)
    {
        var edge = graph.Edges[nearEdge];
        float len = MathF.Max(edge.Length, 0.01f);
        float lo = EndSetback(graph, edge.FromNode) / len;
        float hi = 1f - EndSetback(graph, edge.ToNode) / len;
        if (lo >= hi) return 0.5f * (lo + hi);   // edge too short to honor both setbacks
        return Math.Clamp(nearT, lo, hi);
    }

    /// <summary>
    /// The node a connector would attach to for a foot at (<paramref name="nearEdge"/>,
    /// <paramref name="nearT"/>): the near or far endpoint when the foot is within the split setback
    /// of one (it would be reused rather than split), or -1 when the foot is mid-edge (a fresh split
    /// node would be created). Shared by <see cref="PlaceAndConnect"/> and
    /// <see cref="ComputeFootPoint"/> so the ghost and the commit agree.
    /// </summary>
    private static int ProspectiveFootNode(RoadGraph graph, int nearEdge, float nearT)
    {
        if (nearEdge < 0 || nearEdge >= graph.Edges.Count) return -1;
        var edge = graph.Edges[nearEdge];
        if (edge.FromNode < 0) return -1;
        float len = MathF.Max(edge.Length, 0.01f);
        if (nearT * len < SimConstants.MinSplitSetback) return edge.FromNode;
        if ((1f - nearT) * len < SimConstants.MinSplitSetback) return edge.ToNode;
        return -1; // mid-edge → a fresh, unflagged split node
    }

    /// <summary>
    /// World position where a connector placed at (<paramref name="nearEdge"/>, <paramref name="nearT"/>)
    /// will attach — the single source of truth shared by the placement ghost and
    /// <see cref="PlaceAndConnect"/>. An UNFLAGGED endpoint within the split setback is reused (its
    /// node position); otherwise the point is the on-curve position at the endpoint-clamped parameter
    /// where the commit will split. A FLAGGED endpoint (an existing destination node) is treated as
    /// non-existent — the attach point lands on the road just shy of it, never on the node — so the
    /// ghost foot matches the committed attach point exactly.
    /// </summary>
    public static Vector2 ComputeFootPoint(RoadGraph graph, int nearEdge, float nearT)
    {
        int reuse = ProspectiveFootNode(graph, nearEdge, nearT);
        if (reuse >= 0 && (graph.Nodes[reuse].Flags & NodeFlags.Destination) == 0)
            return graph.Nodes[reuse].Position;
        return graph.EvaluateBezier(nearEdge, ClampedSplitT(graph, nearEdge, nearT));
    }
}
