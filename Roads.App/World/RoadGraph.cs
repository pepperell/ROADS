using System.Diagnostics;
using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Directed road graph with cubic Bézier edge geometry. Stores nodes (intersections/endpoints)
/// and edges (one-way road segments). Two-way roads are represented as a pair of opposing edges.
/// Maintains compact flat adjacency arrays, a turn matrix (no U-turns), and an auto-incrementing
/// version counter for cache invalidation. Defunct nodes have NaN position; defunct edges have
/// FromNode set to -1.
/// </summary>
public class RoadGraph
{
    /// <summary>All nodes (intersections and endpoints) in the graph.</summary>
    private readonly List<RoadNode> _nodes = new();
    /// <summary>All edges (directed road segments) in the graph. Defunct edges have FromNode == -1.</summary>
    private readonly List<RoadEdge> _edges = new();

    /// <summary>Flat array of outgoing edge indices, indexed by RoadNode.EdgeStartIdx/EdgeCount.</summary>
    private int[] _flatOutgoing = Array.Empty<int>();
    /// <summary>Flat array of incoming edge indices, indexed by _incomingStartIdx/_incomingCount.</summary>
    private int[] _flatIncoming = Array.Empty<int>();
    /// <summary>Per-node start index into _flatIncoming.</summary>
    private int[] _incomingStartIdx = Array.Empty<int>();
    /// <summary>Per-node count of incoming edges.</summary>
    private byte[] _incomingCount = Array.Empty<byte>();

    /// <summary>Per-edge cached reverse edge index (-1 if none). Rebuilt during RebuildAdjacency.</summary>
    private int[] _reverseEdgeCache = Array.Empty<int>();

    /// <summary>Per-node set of allowed (incomingEdge, outgoingEdge) turn pairs. U-turns are excluded.</summary>
    private readonly Dictionary<int, HashSet<(int incoming, int outgoing)>> _turnMatrix = new();

    /// <summary>
    /// Per-lane turn restrictions. Key: (incomingEdge, laneIndex). Value: set of allowed
    /// (outgoingEdge, outgoingLane) pairs. If no entry exists for a (edge, lane), all turns
    /// are allowed (auto mode). If entry exists, only listed pairs get intersection arcs.
    /// </summary>
    private readonly Dictionary<(int inEdge, byte inLane), HashSet<(int outEdge, byte outLane)>> _laneRestrictions = new();

    /// <summary>
    /// Nodes whose lane restrictions were set by the user (via the lane-restriction tool
    /// or loaded from a saved map), as opposed to auto-derived geometry defaults. The
    /// normalize-phase <see cref="ApplyDefaultLaneRestrictions"/> skips these and refreshes
    /// every other multi-lane intersection, so a node that gains an edge (e.g. a branch
    /// added by a later split) re-derives its defaults instead of keeping stale ones, while
    /// genuine user customizations are preserved. Only user-node restrictions are saved;
    /// auto defaults are rebuilt on load.
    /// </summary>
    private readonly HashSet<int> _userLaneRestrictionNodes = new();

    /// <summary>
    /// Transient set of "closed" edge indices used by the graceful-deletion drain. A closed
    /// edge stays fully present and traversable (NOT defunct) — vehicles already on it finish
    /// crossing — but it is excluded from new route planning (<see cref="Pathfinder"/> skips it)
    /// and blocks new entry at intersection arcs. This is a side-set rather than an
    /// <see cref="EdgeFlags"/> bit so the closed state is never persisted: a mid-drain save must
    /// not write a "closed" road. The set is purely runtime and is cleared on every full graph
    /// reset (<see cref="LoadFromData"/>). Toggle via <see cref="SetEdgeClosed"/>; query via
    /// <see cref="IsEdgeClosed"/>.
    /// </summary>
    private readonly HashSet<int> _closedEdges = new();

    /// <summary>Read-only view of all nodes in the graph.</summary>
    public IReadOnlyList<RoadNode> Nodes => _nodes;
    /// <summary>Read-only view of all edges in the graph (includes defunct entries).</summary>
    public IReadOnlyList<RoadEdge> Edges => _edges;

    /// <summary>Count of active (non-defunct) edges.</summary>
    public int ActiveEdgeCount { get; private set; }

    /// <summary>
    /// Monotonic counter incremented on every observable graph mutation: nodes (position,
    /// flags, POI type, defunct marking), edges (add/remove, lane count, speed limit,
    /// control points), lane restrictions, and the derived adjacency/turn matrices that
    /// change with them. This is the invalidation bus for all graph-derived state —
    /// geometry caches (StopLineCache, IntersectionArcCache, EdgeSpatialGrid), the render
    /// path cache, the three traffic-control systems, vehicle spawn/destination caches,
    /// the POI/population layer, and GraphChangeHandler all hold a cached copy and lazily
    /// rebuild when it differs. The contract:
    /// (1) every public mutator increments it before returning — unconditionally if the
    ///     mutation always changes state, conditionally if the call may be a no-op
    ///     (ClearLaneRestrictions, StripMarkerFlagsFromIntersections);
    /// (2) consumers compare for equality only, so multiple bumps within one operation
    ///     (e.g. SplitEdge's internal AddNode plus its own final bump) are harmless;
    /// (3) private surgery helpers (MarkNodeDefunct, SplitEdgeSingle) deliberately do not
    ///     bump — their public callers bump once at the end of the whole operation;
    /// (4) normalize steps (signal AutoAssign, ApplyDefaultLaneRestrictions) bump like any
    ///     mutator — SimulationLoop.RebuildWorldCaches confines them to its normalize
    ///     phase, so cache rebuilds remain pure reads.
    /// </summary>
    public int Version { get; private set; }

    /// <summary>
    /// Raised once per single-edge split (the primary edge and, if present, its reverse) with
    /// (oldEdge, firstHalf, secondHalf). External per-edge state keyed by edge index — the stop-sign
    /// and yield exemptions — subscribes to migrate onto the surviving half. An exemption is about
    /// the edge's ToNode, which <c>secondHalf</c> (Mid→ToNode) inherits; <c>firstHalf</c>
    /// (FromNode→Mid) approaches the new mid node. (Lane restrictions live in this class and are
    /// migrated directly by <see cref="MigrateLaneRestrictionsForSplit"/>, not via this event.)
    /// </summary>
    public event Action<int, int, int>? EdgeSplit;

    /// <summary>
    /// Sets the traffic control flags on a node and increments the graph version.
    /// </summary>
    /// <param name="index">Index of the node to modify.</param>
    /// <param name="flags">New flags value.</param>
    public void SetNodeFlags(int index, NodeFlags flags)
    {
        var node = _nodes[index];
        node.Flags = flags;
        _nodes[index] = node;
        Version++;
    }

    /// <summary>
    /// Sets the Point of Interest type on a node.
    /// </summary>
    public void SetNodePOIType(int index, POIType type)
    {
        var node = _nodes[index];
        node.PointOfInterest = type;
        _nodes[index] = node;
        Version++;
    }

    /// <summary>
    /// Adds a new node at the given world position and increments the graph version.
    /// SplitEdge also calls this, producing two version bumps in one operation —
    /// harmless, since version consumers compare for equality only.
    /// </summary>
    /// <param name="position">World-space position in meters.</param>
    /// <returns>Index of the newly created node.</returns>
    public int AddNode(Vector2 position)
    {
        int index = _nodes.Count;
        _nodes.Add(new RoadNode { Position = position });

        // A new node has no incoming edges yet: grow the incoming-adjacency arrays if
        // needed and zero this slot, which may hold stale data from a previous, larger
        // graph (the arrays are grow-only and RebuildAdjacency writes only the active
        // range). Consumers rebuild on the version bump below and must see a
        // consistent graph.
        if (_incomingStartIdx.Length < _nodes.Count)
        {
            Array.Resize(ref _incomingStartIdx, _nodes.Count);
            Array.Resize(ref _incomingCount, _nodes.Count);
        }
        _incomingStartIdx[index] = 0;
        _incomingCount[index] = 0;

        Version++;
        return index;
    }

    /// <summary>
    /// Adds a directed edge between two nodes with default control points (straight line),
    /// rebuilds adjacency and turn matrices, and increments the graph version.
    /// </summary>
    /// <param name="fromNode">Index of the start node.</param>
    /// <param name="toNode">Index of the end node.</param>
    /// <returns>Index of the newly created edge.</returns>
    public int AddEdge(int fromNode, int toNode)
    {
        var from = _nodes[fromNode];
        var to = _nodes[toNode];

        var diff = to.Position - from.Position;
        float length = diff.Length();

        // Default control points at 1/3 and 2/3 along the straight line
        var cp1 = from.Position + diff * (1f / 3f);
        var cp2 = from.Position + diff * (2f / 3f);

        var edge = new RoadEdge
        {
            FromNode = fromNode,
            ToNode = toNode,
            Length = length,
            SpeedLimit = RoadTypeDefaults.GetDefaultSpeedLimit(RoadType.Residential),
            LaneCount = 1,
            RoadType = RoadType.Residential,
            Flags = EdgeFlags.None,
            ControlPoint1 = cp1,
            ControlPoint2 = cp2,
        };

        int edgeIndex = _edges.Count;
        _edges.Add(edge);
        ActiveEdgeCount++;
        Version++;

        RebuildAdjacency();

        // Rebuild turn matrices at both endpoints
        RebuildTurnMatrix(fromNode);
        RebuildTurnMatrix(toNode);

        return edgeIndex;
    }

    /// <summary>
    /// Marks an edge as defunct, rebuilds adjacency and turn matrices, and marks
    /// orphaned endpoints as defunct. Increments the graph version.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to remove.</param>
    public void RemoveEdge(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;

        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return; // already defunct

        int fromNode = edge.FromNode;
        int toNode = edge.ToNode;

        // Clean up lane restrictions referencing this edge
        for (byte lane = 0; lane < edge.LaneCount; lane++)
            _laneRestrictions.Remove((edgeIndex, lane));
        // Also remove this edge as an outgoing target from other edges' restrictions
        PruneLaneRestrictionTarget(edgeIndex);

        // Mark edge as defunct so renderer skips it
        var defunct = _edges[edgeIndex];
        defunct.FromNode = -1;
        defunct.ToNode = -1;
        _edges[edgeIndex] = defunct;
        ActiveEdgeCount--;
        Version++;

        RebuildAdjacency();

        // Rebuild turn matrices at affected nodes
        RebuildTurnMatrix(fromNode);
        RebuildTurnMatrix(toNode);

        // Mark orphaned nodes as defunct
        if (IsNodeOrphaned(fromNode))
            MarkNodeDefunct(fromNode);
        if (IsNodeOrphaned(toNode))
            MarkNodeDefunct(toNode);
    }

    /// <summary>Checks whether a node has zero outgoing and zero incoming edges.</summary>
    private bool IsNodeOrphaned(int nodeIndex)
    {
        return _nodes[nodeIndex].EdgeCount == 0 && _incomingCount[nodeIndex] == 0;
    }

    /// <summary>
    /// Marks a node as defunct by setting its position to NaN, clearing marker flags, and
    /// removing its turn matrix. Does NOT increment Version — public callers (RemoveEdge,
    /// RemoveNode) bump once at the end of the full operation.
    /// </summary>
    private void MarkNodeDefunct(int nodeIndex)
    {
        var node = _nodes[nodeIndex];
        node.Position = new System.Numerics.Vector2(float.NaN, float.NaN);
        node.Flags &= ~NodeFlags.Destination;
        node.PointOfInterest = POIType.None;
        _nodes[nodeIndex] = node;
        _turnMatrix.Remove(nodeIndex);
        _userLaneRestrictionNodes.Remove(nodeIndex);
    }

    /// <summary>
    /// Marks an edge as closed (or reopens it) for the graceful-deletion drain. A closed edge
    /// remains fully present and traversable — it is NOT marked defunct, so vehicles already on
    /// it (or spawned onto it during the drain) keep moving and finish crossing — but new routes
    /// avoid it (<see cref="Pathfinder"/> skips closed edges) and new entry at intersection arcs
    /// is blocked. The closed state is transient and never serialized (see
    /// <see cref="_closedEdges"/>). Increments <see cref="Version"/> only when the set actually
    /// changes, so route-dependent caches refresh exactly once per real transition (and a no-op
    /// toggle does not churn caches).
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to close or reopen.</param>
    /// <param name="closed">True to close (exclude from routing / block entry); false to reopen.</param>
    public void SetEdgeClosed(int edgeIndex, bool closed)
    {
        bool changed = closed ? _closedEdges.Add(edgeIndex) : _closedEdges.Remove(edgeIndex);
        if (changed) Version++;
    }

    /// <summary>
    /// Returns true if the edge is currently closed for the drain (route-excluded and
    /// entry-blocked, but still traversable). See <see cref="SetEdgeClosed"/>.
    /// </summary>
    public bool IsEdgeClosed(int edgeIndex) => _closedEdges.Contains(edgeIndex);

    /// <summary>
    /// Rebuild compact flat adjacency arrays from the current edge list.
    /// Updates EdgeStartIdx/EdgeCount on each RoadNode for outgoing edges,
    /// and parallel arrays for incoming edges.
    /// </summary>
    private void RebuildAdjacency()
    {
        int nodeCount = _nodes.Count;

        // Temporary count arrays
        var outCount = new int[nodeCount];
        var inCount = new int[nodeCount];

        int totalOut = 0, totalIn = 0;
        for (int i = 0; i < _edges.Count; i++)
        {
            var e = _edges[i];
            if (e.FromNode < 0) continue; // defunct
            outCount[e.FromNode]++;
            inCount[e.ToNode]++;
            totalOut++;
            totalIn++;
        }

        // Resize flat arrays if needed (only grow, never shrink — avoids GC churn)
        if (_flatOutgoing.Length < totalOut)
            _flatOutgoing = new int[totalOut];
        if (_flatIncoming.Length < totalIn)
            _flatIncoming = new int[totalIn];
        if (_incomingStartIdx.Length < nodeCount)
        {
            _incomingStartIdx = new int[nodeCount];
            _incomingCount = new byte[nodeCount];
        }

        // Compute prefix sums for start indices and update node fields
        int outPos = 0, inPos = 0;
        for (int n = 0; n < nodeCount; n++)
        {
            // The per-node degree fields stay byte-sized (255+ edges at ONE node is
            // pathological geometry, not a scale target) — but a silent (byte) wrap
            // would make the node read a truncated or foreign list, so trip loudly.
            Debug.Assert(outCount[n] <= byte.MaxValue,
                $"Node {n} has {outCount[n]} outgoing edges - RoadNode.EdgeCount (byte) would wrap.");
            Debug.Assert(inCount[n] <= byte.MaxValue,
                $"Node {n} has {inCount[n]} incoming edges - _incomingCount (byte) would wrap.");
            var node = _nodes[n];
            node.EdgeStartIdx = outPos;
            node.EdgeCount = (byte)outCount[n];
            _nodes[n] = node;

            _incomingStartIdx[n] = inPos;
            _incomingCount[n] = (byte)inCount[n];

            outPos += outCount[n];
            inPos += inCount[n];
        }

        // Clear stale entries beyond the active range — the arrays are grow-only, so a
        // smaller graph replacing a larger one (e.g. New Map after a load) would
        // otherwise leave old counts reachable through node indices added later.
        if (_incomingCount.Length > nodeCount)
        {
            Array.Clear(_incomingStartIdx, nodeCount, _incomingStartIdx.Length - nodeCount);
            Array.Clear(_incomingCount, nodeCount, _incomingCount.Length - nodeCount);
        }

        // Fill flat arrays (reuse outCount/inCount as placement cursors, reset to 0)
        Array.Clear(outCount, 0, nodeCount);
        Array.Clear(inCount, 0, nodeCount);

        for (int i = 0; i < _edges.Count; i++)
        {
            var e = _edges[i];
            if (e.FromNode < 0) continue;

            int from = e.FromNode;
            _flatOutgoing[_nodes[from].EdgeStartIdx + outCount[from]] = i;
            outCount[from]++;

            int to = e.ToNode;
            _flatIncoming[_incomingStartIdx[to] + inCount[to]] = i;
            inCount[to]++;
        }

        // Rebuild reverse edge cache. A twin is the opposite-direction edge tracing the
        // SAME curve, so its control points are mirrored (its CP1 at my CP2 and vice
        // versa — an invariant every mutation maintains: creation at the thirds,
        // SetControlPoint twin-sync, MoveNode's symmetric translate/rescale, De
        // Casteljau split halves). Endpoint matching alone is ambiguous when two
        // DISTINCT roads connect the same node pair (e.g. a U-shaped segment plus a
        // straight connector between the same two intersections): the first endpoint
        // match could pair a road with the OTHER road's opposite direction — the
        // renderer then skips a "non-primary" edge that is really an unpaired road
        // (it vanishes), and twin-synced mutations cross roads. Prefer the mirrored
        // candidate; fall back to the first endpoint match (old behavior) only when
        // no candidate mirrors.
        int edgeCount = _edges.Count;
        if (_reverseEdgeCache.Length < edgeCount)
            _reverseEdgeCache = new int[edgeCount];
        const float twinCpEpsilonSq = 1f; // twins match to float noise; distinct roads differ by meters
        for (int i = 0; i < edgeCount; i++)
        {
            _reverseEdgeCache[i] = -1;
            var e = _edges[i];
            if (e.FromNode < 0) continue;
            int fallback = -1;
            foreach (int candidate in GetOutgoingEdges(e.ToNode))
            {
                var c = _edges[candidate];
                if (c.ToNode != e.FromNode) continue;
                if (Vector2.DistanceSquared(c.ControlPoint1, e.ControlPoint2) < twinCpEpsilonSq
                    && Vector2.DistanceSquared(c.ControlPoint2, e.ControlPoint1) < twinCpEpsilonSq)
                {
                    fallback = candidate;
                    break;
                }
                if (fallback < 0) fallback = candidate;
            }
            _reverseEdgeCache[i] = fallback;
        }

#if DEBUG
        // ActiveEdgeCount drift tripwire: every mutator updates the cached count BEFORE
        // calling RebuildAdjacency, so a mismatch here means an increment/decrement was
        // missed or doubled (the L5 class of bug — it previously drifted +1 per two-way
        // split and healed only on load). Debug builds only; O(E).
        int activeActual = 0;
        for (int i = 0; i < edgeCount; i++)
            if (_edges[i].FromNode >= 0) activeActual++;
        Debug.Assert(ActiveEdgeCount == activeActual,
            $"ActiveEdgeCount drift: cached {ActiveEdgeCount}, actual {activeActual}");
#endif
    }

    /// <summary>
    /// Rebuild the turn matrix for a node: allow all incoming->outgoing turns except
    /// U-turns (back onto the reverse of the same road segment). The one exception is a
    /// dead-end: if an incoming edge has no non-U-turn outgoing option, the U-turn is
    /// allowed so a vehicle can turn around at the end of a road (otherwise it would be
    /// trapped and removed). This is flag-agnostic — it applies to any terminal node,
    /// not just Destination nodes.
    /// </summary>
    private void RebuildTurnMatrix(int nodeIndex)
    {
        var incoming = GetIncomingEdges(nodeIndex);
        var outgoing = GetOutgoingEdges(nodeIndex);

        if (incoming.Count == 0 || outgoing.Count == 0)
        {
            _turnMatrix.Remove(nodeIndex);
            return;
        }

        var turns = new HashSet<(int, int)>();
        foreach (int inEdge in incoming)
        {
            var inE = _edges[inEdge];
            int uTurnEdge = -1;
            bool hasNonUTurn = false;
            foreach (int outEdge in outgoing)
            {
                // U-turn: back onto the arriving road's own reverse TWIN. A different
                // road that also returns to the same neighbor node (two parallel roads
                // between the same intersections) is a normal turn, not a U-turn —
                // an endpoint-based test here wrongly banned switching between such
                // parallel roads at their shared nodes.
                if (outEdge == FindReverseEdge(inEdge))
                {
                    uTurnEdge = outEdge;
                    continue;
                }
                turns.Add((inEdge, outEdge));
                hasNonUTurn = true;
            }

            // Dead-end for this approach: the only way out is back the way we came.
            // Allow the U-turn so the vehicle can turn around instead of being trapped.
            if (!hasNonUTurn && uTurnEdge >= 0)
                turns.Add((inEdge, uTurnEdge));
        }

        _turnMatrix[nodeIndex] = turns;
    }

    /// <summary>
    /// Gets the outgoing edge indices for a node as a contiguous array segment.
    /// </summary>
    /// <param name="nodeIndex">Index of the node.</param>
    /// <returns>Array segment of outgoing edge indices, or empty if none.</returns>
    public ArraySegment<int> GetOutgoingEdges(int nodeIndex)
    {
        var node = _nodes[nodeIndex];
        if (node.EdgeCount == 0 || _flatOutgoing.Length == 0)
            return ArraySegment<int>.Empty;
        return new ArraySegment<int>(_flatOutgoing, node.EdgeStartIdx, node.EdgeCount);
    }

    /// <summary>
    /// Gets the incoming edge indices for a node as a contiguous array segment.
    /// </summary>
    /// <param name="nodeIndex">Index of the node.</param>
    /// <returns>Array segment of incoming edge indices, or empty if none.</returns>
    public ArraySegment<int> GetIncomingEdges(int nodeIndex)
    {
        if (nodeIndex >= _incomingCount.Length || _incomingCount[nodeIndex] == 0 || _flatIncoming.Length == 0)
            return ArraySegment<int>.Empty;
        return new ArraySegment<int>(_flatIncoming, _incomingStartIdx[nodeIndex], _incomingCount[nodeIndex]);
    }

    /// <summary>
    /// True when a node is a junction where traffic control (stop/yield/light) is meaningful:
    /// at least two streams enter AND at least three distinct roads meet. Distinct *neighbor
    /// nodes* are counted, not directed edges, so a one-way merge (two incoming one-way edges
    /// from distinct sources plus one outgoing — 2 incoming, 3 neighbors) qualifies while a
    /// straight pass-through (2 neighbors) does not. For two-way graphs each road contributes
    /// exactly one incoming edge and one neighbor, so this returns the identical result to
    /// <c>GetIncomingEdges(node).Count &gt;= 3</c> — only one-way junctions change.
    /// This is the single source of truth for "is this an intersection" across the signal
    /// editor (<see cref="Editor.SignalTool"/>), the auto-assign policies
    /// (<see cref="StopSignSystem"/>), and <see cref="StripMarkerFlagsFromIntersections"/>.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to test.</param>
    public bool IsTrafficControlJunction(int nodeIndex)
    {
        var incoming = GetIncomingEdges(nodeIndex);
        if (incoming.Count < 2) return false; // a single entering stream cannot conflict

        // Distinct neighbor nodes across incoming sources and outgoing destinations.
        // Degree is tiny (clamped well under 8), so a small stack buffer + linear scan
        // is cheaper than a HashSet and allocation-free. Early-out once 3 are seen.
        Span<int> neighbors = stackalloc int[16];
        int count = 0;

        foreach (int e in incoming)
        {
            int from = _edges[e].FromNode;
            if (from < 0) continue;
            bool seen = false;
            for (int k = 0; k < count; k++) if (neighbors[k] == from) { seen = true; break; }
            if (!seen && count < neighbors.Length) neighbors[count++] = from;
        }
        foreach (int e in GetOutgoingEdges(nodeIndex))
        {
            int to = _edges[e].ToNode;
            if (to < 0) continue;
            bool seen = false;
            for (int k = 0; k < count; k++) if (neighbors[k] == to) { seen = true; break; }
            if (!seen && count < neighbors.Length) neighbors[count++] = to;
        }

        return count >= 3;
    }

    /// <summary>
    /// Gets allowed outgoing edges from a node given the incoming edge.
    /// Excludes U-turns (back onto the reverse of the same road segment).
    /// If no turn matrix exists for the node, all outgoing edges are returned.
    /// </summary>
    /// <param name="nodeIndex">Index of the intersection node.</param>
    /// <param name="incomingEdgeIndex">Index of the edge the vehicle arrived on.</param>
    /// <returns>List of outgoing edge indices that form valid turns.</returns>
    public List<int> GetAllowedTurns(int nodeIndex, int incomingEdgeIndex)
    {
        var result = new List<int>();
        var outgoing = GetOutgoingEdges(nodeIndex);

        if (!_turnMatrix.TryGetValue(nodeIndex, out var turns))
        {
            // No turn matrix = allow all outgoing (e.g. spawn point)
            foreach (int edge in outgoing)
                result.Add(edge);
            return result;
        }

        foreach (int outEdge in outgoing)
        {
            if (turns.Contains((incomingEdgeIndex, outEdge)))
                result.Add(outEdge);
        }
        return result;
    }

    // --- Per-lane turn restriction API ---

    /// <summary>
    /// Gets the allowed (outEdge, outLane) pairs for a specific incoming edge and lane.
    /// Returns null if no restrictions are set (auto mode — geometry-based defaults).
    /// </summary>
    public HashSet<(int outEdge, byte outLane)>? GetLaneRestrictions(int inEdge, byte inLane)
    {
        return _laneRestrictions.TryGetValue((inEdge, inLane), out var set) ? set : null;
    }

    /// <summary>
    /// Toggles a single (outEdge, outLane) connection for an incoming lane.
    /// Creates the restriction set on first toggle (switching from auto to explicit).
    /// </summary>
    public void ToggleLaneConnection(int inEdge, byte inLane, int outEdge, byte outLane)
    {
        var key = (inEdge, inLane);
        if (!_laneRestrictions.TryGetValue(key, out var set))
        {
            set = new HashSet<(int, byte)>();
            _laneRestrictions[key] = set;
        }
        var pair = (outEdge, outLane);
        if (!set.Remove(pair))
            set.Add(pair);
        // User customization: this node's restrictions are now manual, so the normalize
        // phase must preserve them instead of re-deriving geometry defaults.
        if (inEdge >= 0 && inEdge < _edges.Count)
            _userLaneRestrictionNodes.Add(_edges[inEdge].ToNode);
        Version++;
    }

    /// <summary>
    /// Removes all per-lane restrictions for an edge (all lanes revert to auto mode).
    /// </summary>
    public void ClearLaneRestrictions(int inEdge)
    {
        bool removed = false;
        if (inEdge < 0 || inEdge >= _edges.Count) return;
        byte laneCount = _edges[inEdge].LaneCount;
        for (byte lane = 0; lane < laneCount; lane++)
        {
            if (_laneRestrictions.Remove((inEdge, lane)))
                removed = true;
        }
        if (removed) Version++;
    }

    /// <summary>
    /// (Re)derives default lane restrictions at every multi-lane intersection node that is
    /// not user-customized (see <see cref="_userLaneRestrictionNodes"/>). Left lanes turn
    /// left, right lanes turn right, etc. Re-deriving auto nodes — rather than skipping any
    /// node that already has restrictions — ensures a node that gained an edge after its
    /// first auto-default (e.g. a branch added by a later split) refreshes instead of
    /// keeping a stale set. Runs in the normalize phase of SimulationLoop.RebuildWorldCaches;
    /// requires a current StopLineCache (reads tangent directions at stop-line positions).
    /// Idempotent: <see cref="SetGeometryDefaultRestrictionsAtNode"/> bumps Version only when
    /// the derived restrictions actually change, so a settled graph converges in one pass.
    /// </summary>
    /// <param name="stopLines">Stop-line cache (must be up to date).</param>
    public void ApplyDefaultLaneRestrictions(StopLineCache stopLines)
    {
        int nodeCount = _nodes.Count;
        for (int n = 0; n < nodeCount; n++)
        {
            var node = _nodes[n];
            if (float.IsNaN(node.Position.X)) continue;

            // Skip only user-customized nodes; auto nodes are re-derived so that a node
            // which gained an edge (e.g. a branch added by a later split) refreshes its
            // defaults instead of keeping the stale set from its earlier topology.
            if (_userLaneRestrictionNodes.Contains(n)) continue;

            var outgoing = GetOutgoingEdges(n);
            if (outgoing.Count < 2) continue;

            var incoming = GetIncomingEdges(n);
            bool hasMultiLane = false;
            foreach (int inEdge in incoming)
            {
                if (_edges[inEdge].FromNode < 0) continue;
                if (_edges[inEdge].LaneCount >= 2) { hasMultiLane = true; break; }
            }
            if (!hasMultiLane) continue;

            SetGeometryDefaultRestrictionsAtNode(n, stopLines);
        }
    }

    /// <summary>
    /// Sets explicit per-lane restrictions at a node to match the geometry-based defaults
    /// (the same lane pairing logic used by <see cref="IntersectionArcCache"/>), replacing
    /// any existing restrictions on the node's incoming edges, and increments the graph
    /// version. Requires <paramref name="stopLines"/> to compute tangent directions at
    /// stop-line positions. Called by <see cref="ApplyDefaultLaneRestrictions"/> and by
    /// the editor's reset-to-defaults action.
    /// </summary>
    /// <param name="nodeIndex">The intersection node index.</param>
    /// <param name="stopLines">Stop-line cache (must be up to date).</param>
    public void SetGeometryDefaultRestrictionsAtNode(int nodeIndex, StopLineCache stopLines)
    {
        // (Re)deriving from geometry makes this node auto, not user-customized.
        _userLaneRestrictionNodes.Remove(nodeIndex);

        var incoming = GetIncomingEdges(nodeIndex);

        // Build the geometry defaults into a temp map first. Apply them only if they differ
        // from what is already stored, so this is idempotent: ApplyDefaultLaneRestrictions
        // re-derives every auto node on each graph change and must converge without bumping
        // Version (else the normalize phase fails its single-pass convergence assert).
        var computed = new Dictionary<(int inEdge, byte inLane), HashSet<(int, byte)>>();
        var processed = new List<int>(); // non-degenerate incoming edges actually handled
        foreach (int inEdge in incoming)
        {
            var inEdgeData = _edges[inEdge];
            if (inEdgeData.FromNode < 0) continue;
            byte inLaneCount = inEdgeData.LaneCount;

            float inStopT = stopLines.GetStopTAtToNode(inEdge);
            var inTangent = EvaluateBezierTangent(inEdge, inStopT);
            float inTanLen = inTangent.Length();
            if (inTanLen < 0.001f) continue;
            var inDir = inTangent / inTanLen;
            processed.Add(inEdge);

            var allowedTurns = GetAllowedTurns(nodeIndex, inEdge);
            foreach (int outEdge in allowedTurns)
            {
                var outEdgeData = _edges[outEdge];
                if (outEdgeData.FromNode < 0) continue;

                float outStartT = stopLines.GetStopTAtFromNode(outEdge);
                var outTangent = EvaluateBezierTangent(outEdge, outStartT);
                float outTanLen = outTangent.Length();
                if (outTanLen < 0.001f) continue;
                var outDir = outTangent / outTanLen;

                // Cross product in Y-down coords: positive = right turn, negative = left turn
                float cross = inDir.X * outDir.Y - inDir.Y * outDir.X;
                byte outLaneCount = outEdgeData.LaneCount;

                var lanePairs = ComputeGeometryLanePairs(inLaneCount, outLaneCount, cross);
                foreach (var (inLane, outLane) in lanePairs)
                {
                    var key = (inEdge, inLane);
                    if (!computed.TryGetValue(key, out var set))
                    {
                        set = new HashSet<(int, byte)>();
                        computed[key] = set;
                    }
                    set.Add((outEdge, outLane));
                }
            }
        }

        // Does the computed default differ from what's stored on the processed edges?
        bool changed = false;
        foreach (int inEdge in processed)
        {
            byte inLaneCount = _edges[inEdge].LaneCount;
            for (byte lane = 0; lane < inLaneCount && !changed; lane++)
            {
                var key = (inEdge, lane);
                bool hasOld = _laneRestrictions.TryGetValue(key, out var oldSet);
                bool hasNew = computed.TryGetValue(key, out var newSet);
                if (hasOld != hasNew || (hasOld && !oldSet!.SetEquals(newSet!)))
                    changed = true;
            }
            if (changed) break;
        }

        if (!changed) return;

        // Replace the processed edges' restrictions with the computed defaults.
        foreach (int inEdge in processed)
        {
            byte inLaneCount = _edges[inEdge].LaneCount;
            for (byte lane = 0; lane < inLaneCount; lane++)
                _laneRestrictions.Remove((inEdge, lane));
        }
        foreach (var kvp in computed)
            _laneRestrictions[kvp.Key] = kvp.Value;

        Version++;
    }

    /// <summary>
    /// Computes geometry-based lane pairs for a turn, matching the logic in
    /// <see cref="IntersectionArcCache"/>. Straight-through maps lane N→N (clamped),
    /// right turns use the rightmost lane, left turns use the leftmost lane.
    /// </summary>
    /// <param name="inLaneCount">Number of lanes on the incoming edge.</param>
    /// <param name="outLaneCount">Number of lanes on the outgoing edge.</param>
    /// <param name="cross">Cross product of incoming and outgoing directions (positive = right turn in Y-down).</param>
    /// <returns>List of (inLane, outLane) pairs.</returns>
    public static List<(byte inLane, byte outLane)> ComputeGeometryLanePairs(byte inLaneCount, byte outLaneCount, float cross)
    {
        var pairs = new List<(byte, byte)>();
        if (MathF.Abs(cross) < SimConstants.TurnThreshold)
        {
            // Straight-through: map lane N → lane N (clamped)
            int minLanes = Math.Min(inLaneCount, outLaneCount);
            for (byte i = 0; i < minLanes; i++)
                pairs.Add((i, i));
        }
        else if (cross > 0)
        {
            // Right turn: rightmost lane only
            byte inLane = (byte)(inLaneCount - 1);
            byte outLane = (byte)(outLaneCount - 1);
            pairs.Add((inLane, outLane));
        }
        else
        {
            // Left turn: leftmost lane only
            pairs.Add((0, 0));
        }

        return pairs;
    }

    /// <summary>
    /// Returns the geometry-default allowed (outEdge, outLane) set for a given incoming
    /// edge and lane at its ToNode. Used as the visual fallback when no explicit
    /// per-lane restrictions have been set.
    /// </summary>
    public HashSet<(int outEdge, byte outLane)> GetGeometryDefaultLaneTargets(
        int inEdge, byte inLane, StopLineCache stopLines)
    {
        var result = new HashSet<(int, byte)>();
        var inEdgeData = _edges[inEdge];
        int nodeIndex = inEdgeData.ToNode;
        if (nodeIndex < 0) return result;
        byte inLaneCount = inEdgeData.LaneCount;

        float inStopT = stopLines.GetStopTAtToNode(inEdge);
        var inTangent = EvaluateBezierTangent(inEdge, inStopT);
        float inTanLen = inTangent.Length();
        if (inTanLen < 0.001f) return result;
        var inDir = inTangent / inTanLen;

        var allowedTurns = GetAllowedTurns(nodeIndex, inEdge);
        foreach (int outEdge in allowedTurns)
        {
            var outEdgeData = _edges[outEdge];
            if (outEdgeData.FromNode < 0) continue;

            float outStartT = stopLines.GetStopTAtFromNode(outEdge);
            var outTangent = EvaluateBezierTangent(outEdge, outStartT);
            float outTanLen = outTangent.Length();
            if (outTanLen < 0.001f) continue;
            var outDir = outTangent / outTanLen;

            float cross = inDir.X * outDir.Y - inDir.Y * outDir.X;
            byte outLaneCount = outEdgeData.LaneCount;

            var lanePairs = ComputeGeometryLanePairs(inLaneCount, outLaneCount, cross);
            foreach (var (pairInLane, outLane) in lanePairs)
            {
                if (pairInLane == inLane)
                    result.Add((outEdge, outLane));
            }
        }
        return result;
    }

    /// <summary>
    /// Removes a defunct outgoing edge from all lane restriction sets.
    /// </summary>
    private void PruneLaneRestrictionTarget(int removedEdge)
    {
        foreach (var set in _laneRestrictions.Values)
            set.RemoveWhere(pair => pair.outEdge == removedEdge);
    }

    /// <summary>
    /// Returns true if any lane on this edge has explicit turn restrictions.
    /// </summary>
    public bool HasLaneRestrictions(int inEdge)
    {
        if (inEdge < 0 || inEdge >= _edges.Count) return false;
        byte laneCount = _edges[inEdge].LaneCount;
        for (byte lane = 0; lane < laneCount; lane++)
        {
            if (_laneRestrictions.ContainsKey((inEdge, lane)))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Returns true if any incoming edge at this node has explicit lane restrictions
    /// (set manually by the user or previously auto-applied).
    /// </summary>
    public bool HasAnyLaneRestrictionsAtNode(int nodeIndex)
    {
        var incoming = GetIncomingEdges(nodeIndex);
        foreach (int inEdge in incoming)
        {
            if (HasLaneRestrictions(inEdge))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Finds the reverse edge for a given edge (same two nodes, opposite direction).
    /// Uses a cached lookup table rebuilt during adjacency updates for O(1) access.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to find the reverse of.</param>
    /// <returns>Index of the reverse edge, or -1 if none exists.</returns>
    public int FindReverseEdge(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _reverseEdgeCache.Length) return -1;
        return _reverseEdgeCache[edgeIndex];
    }

    /// <summary>Upper clamp for per-edge lane counts. Also bounds the orphan sweep in
    /// <see cref="PruneLaneRestrictionsForLaneCount"/> — keep the two in sync by using
    /// this constant for both.</summary>
    private const byte MaxLaneCount = 4;

    /// <summary>
    /// Sets the lane count for an edge and its reverse, clamped to [1, 4], and prunes
    /// lane restrictions that the new count invalidates (see
    /// <see cref="PruneLaneRestrictionsForLaneCount"/>).
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to modify.</param>
    /// <param name="laneCount">Desired lane count (clamped to 1–4).</param>
    public void SetLaneCount(int edgeIndex, byte laneCount)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        byte clamped = Math.Clamp(laneCount, (byte)1, MaxLaneCount);
        edge.LaneCount = clamped;
        _edges[edgeIndex] = edge;
        PruneLaneRestrictionsForLaneCount(edgeIndex, clamped);
        // Also update the reverse (opposite direction) edge
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0)
        {
            var rev = _edges[reverse];
            rev.LaneCount = clamped;
            _edges[reverse] = rev;
            PruneLaneRestrictionsForLaneCount(reverse, clamped);
        }
        Version++;
    }

    /// <summary>
    /// Reconciles <see cref="_laneRestrictions"/> with a lane count that was just set on
    /// <paramref name="edgeIndex"/>. Two inconsistencies are possible after a shrink, and
    /// both are removed here (the sweep runs to <see cref="MaxLaneCount"/> rather than the
    /// old count, so orphans carried in by pre-fix save files are cleaned up too):
    /// keys for removed in-lanes — unreachable by every other cleanup path (they all
    /// iterate lanes below the CURRENT count) and resurrecting if lanes are later
    /// re-added — and (thisEdge, outLane) pairs in ANY set that target a removed lane,
    /// which the arc cache silently filters. A set left EMPTY by that pair-pruning
    /// reverts to auto (key removed): an empty set generates ZERO arcs for its in-lane,
    /// and on a user-customized node auto-defaults never heal it, so the vehicle would
    /// hold at the stop line forever. Sets the user emptied deliberately (no pair removed
    /// here) are left alone. <see cref="_userLaneRestrictionNodes"/> is intentionally
    /// untouched, matching <see cref="ClearLaneRestrictions"/>: a keyless lane already
    /// behaves as geometry-default at arc generation regardless of the node marker.
    /// Callers bump <see cref="Version"/>.
    /// </summary>
    private void PruneLaneRestrictionsForLaneCount(int edgeIndex, byte newCount)
    {
        // Keys for in-lanes at/above the new count.
        for (byte lane = newCount; lane < MaxLaneCount; lane++)
            _laneRestrictions.Remove((edgeIndex, lane));

        // Pairs targeting out-lanes of this edge at/above the new count, in every set.
        List<(int, byte)>? emptied = null;
        foreach (var kvp in _laneRestrictions)
        {
            int removed = kvp.Value.RemoveWhere(p => p.outEdge == edgeIndex && p.outLane >= newCount);
            if (removed > 0 && kvp.Value.Count == 0)
                (emptied ??= new List<(int, byte)>()).Add(kvp.Key);
        }
        if (emptied != null)
            foreach (var key in emptied)
                _laneRestrictions.Remove(key);
    }

    /// <summary>
    /// Toggles single-lane two-way (<see cref="EdgeFlags.SharedLane"/>) on a two-way road: sets or
    /// clears the flag on BOTH edges of the pair and, when enabling, forces the lane count to 1 (one
    /// physical lane shared by both directions). No-op on a one-way road (no reverse edge), where the
    /// flag has no meaning. Vehicles entering a shared edge are gated against oncoming traffic by the
    /// steering controller's shared-lane gate.
    /// </summary>
    /// <summary>
    /// Sets or clears <see cref="EdgeFlags.Bridge"/> on an edge and its reverse twin (if
    /// any). A bridge passes over everything it crosses: crossing detection skips bridge
    /// edges in both directions, so the segment connects to the network only at its end
    /// nodes, and the renderer gives it the full bridge treatment.
    /// </summary>
    public void SetBridge(int edgeIndex, bool bridge)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        if (_edges[edgeIndex].FromNode < 0) return;

        void Apply(int e)
        {
            var edge = _edges[e];
            var flags = bridge ? edge.Flags | EdgeFlags.Bridge : edge.Flags & ~EdgeFlags.Bridge;
            if (flags == edge.Flags) return;
            edge.Flags = flags;
            _edges[e] = edge;
            Version++;
        }

        Apply(edgeIndex);
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0) Apply(reverse);
    }

    public void SetSharedLane(int edgeIndex, bool shared)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        if (_edges[edgeIndex].FromNode < 0) return;
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse < 0) return; // shared-lane is a two-way concept

        ApplySharedLaneFlag(edgeIndex, shared);
        ApplySharedLaneFlag(reverse, shared);
        Version++;
    }

    /// <summary>Sets or clears <see cref="EdgeFlags.SharedLane"/> on one edge, forcing 1 lane
    /// when set — a lane-count shrink, so the same restriction pruning as
    /// <see cref="SetLaneCount"/> applies.</summary>
    private void ApplySharedLaneFlag(int edgeIndex, bool shared)
    {
        var e = _edges[edgeIndex];
        if (shared)
        {
            e.Flags |= EdgeFlags.SharedLane;
            e.LaneCount = 1;
        }
        else
        {
            e.Flags &= ~EdgeFlags.SharedLane;
        }
        _edges[edgeIndex] = e;
        if (shared)
            PruneLaneRestrictionsForLaneCount(edgeIndex, 1);
    }

    /// <summary>Version at which <see cref="MaxSpeedLimit"/> was last computed.</summary>
    private int _maxSpeedLimitVersion = -1;
    private float _maxSpeedLimit = 13.4f;

    /// <summary>
    /// Highest speed limit (m/s) among active edges, lazily recomputed when the graph
    /// version changes; 13.4 (the residential default) on an empty graph. Pure derived
    /// state — never bumps the version, so it is safe to read from cache-rebuild phases.
    /// Used by the Pathfinder's A* heuristic, which must divide straight-line distance
    /// by the TRUE maximum speed to stay admissible: dividing by anything smaller
    /// overestimates remaining travel time and yields suboptimal routes (fast bypasses
    /// losing to shorter slow streets).
    /// </summary>
    public float MaxSpeedLimit
    {
        get
        {
            if (_maxSpeedLimitVersion != Version)
            {
                float max = 0f;
                for (int i = 0; i < _edges.Count; i++)
                    if (_edges[i].FromNode >= 0 && _edges[i].SpeedLimit > max)
                        max = _edges[i].SpeedLimit;
                _maxSpeedLimit = max > 0.1f ? max : 13.4f;
                _maxSpeedLimitVersion = Version;
            }
            return _maxSpeedLimit;
        }
    }

    /// <summary>
    /// Sets the speed limit for an edge and its reverse edge, clamped to ~5–100 mph.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to modify.</param>
    /// <param name="speedLimitMs">Speed limit in meters per second (clamped to 2.2–44.7 m/s).</param>
    public void SetSpeedLimit(int edgeIndex, float speedLimitMs)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        float clamped = Math.Clamp(speedLimitMs, 2.2f, 44.7f); // ~5 to ~100 mph
        edge.SpeedLimit = clamped;
        _edges[edgeIndex] = edge;
        // Also update the reverse (opposite direction) edge
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0)
        {
            var rev = _edges[reverse];
            rev.SpeedLimit = clamped;
            _edges[reverse] = rev;
        }
        Version++;
    }

    /// <summary>
    /// Sets the road type on an edge and its reverse edge. Also updates each edge's speed
    /// limit to the default for the new type and bumps the graph version so render and
    /// simulation caches invalidate. The speed limit update matches the behavior of
    /// <see cref="RoadTypeDefaults.GetDefaultSpeedLimit"/>.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to modify.</param>
    /// <param name="type">New road classification.</param>
    public void SetEdgeRoadType(int edgeIndex, RoadType type)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        float defaultSpeed = RoadTypeDefaults.GetDefaultSpeedLimit(type);
        edge.RoadType = type;
        edge.SpeedLimit = defaultSpeed;
        _edges[edgeIndex] = edge;
        // Also update the reverse (opposite direction) edge
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0)
        {
            var rev = _edges[reverse];
            rev.RoadType = type;
            rev.SpeedLimit = defaultSpeed;
            _edges[reverse] = rev;
        }
        Version++;
    }

    // ── One-way / two-way conversion ───────────────────────────────────
    //
    // A one-way road is simply a directed edge with no reverse partner — there is no
    // EdgeFlags bit, because topology (FindReverseEdge &lt; 0) is the single source of
    // truth and cannot drift out of sync. These three operations form the editor's
    // single-key cycle: two-way → one-way (selected direction) → one-way (reversed) →
    // two-way. The selected edge index is preserved across all three so the editor's
    // selection and cycle state stay valid.

    /// <summary>
    /// Converts a two-way road to one-way in the direction of <paramref name="edgeIndex"/>
    /// by removing its reverse edge. Vehicles stranded on the removed direction are
    /// re-snapped or removed by <see cref="GraphChangeHandler"/> on the next tick (the same
    /// path used for any edge deletion). No-op (returns false) if the edge is already one-way.
    /// </summary>
    public bool MakeOneWay(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return false;
        if (_edges[edgeIndex].FromNode < 0) return false;
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse < 0) return false; // already one-way
        // edgeIndex keeps both endpoints connected, so removing the reverse cannot orphan
        // either node. RemoveEdge rebuilds adjacency/turn matrices and bumps Version.
        RemoveEdge(reverse);
        return true;
    }

    /// <summary>
    /// Flips the travel direction of a one-way road in place (swaps its end nodes and mirrors
    /// its Bézier control points). The edge index is unchanged. Its per-lane turn restrictions
    /// are cleared (they described turns at the old downstream node and no longer apply); auto
    /// defaults re-derive on the next normalize pass. No-op (returns false) if the edge is not
    /// one-way (a paired/two-way edge must not have its direction flipped independently).
    /// </summary>
    public bool ReverseOneWay(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return false;
        var e = _edges[edgeIndex];
        if (e.FromNode < 0) return false;
        if (FindReverseEdge(edgeIndex) >= 0) return false; // not one-way

        // Drop restrictions keyed on this edge (as an incoming edge) and any that target it
        // (as an outgoing edge); both are tied to the pre-flip direction.
        ClearLaneRestrictions(edgeIndex);
        PruneLaneRestrictionTarget(edgeIndex);

        (e.FromNode, e.ToNode) = (e.ToNode, e.FromNode);
        (e.ControlPoint1, e.ControlPoint2) = (e.ControlPoint2, e.ControlPoint1);
        _edges[edgeIndex] = e;

        RebuildAdjacency();
        RebuildTurnMatrix(e.FromNode);
        RebuildTurnMatrix(e.ToNode);
        Version++;
        return true;
    }

    /// <summary>
    /// Converts a one-way road back to two-way by adding a reverse edge that mirrors the
    /// existing one (mirrored control points; same lanes, speed, type, and flags — preserving
    /// <see cref="EdgeFlags.SharedLane"/>). No-op (returns false) if the edge already has a
    /// reverse partner.
    /// </summary>
    public bool MakeTwoWay(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return false;
        var e = _edges[edgeIndex];
        if (e.FromNode < 0) return false;
        if (FindReverseEdge(edgeIndex) >= 0) return false; // already two-way

        var rev = new RoadEdge
        {
            FromNode = e.ToNode,
            ToNode = e.FromNode,
            Length = e.Length,
            SpeedLimit = e.SpeedLimit,
            LaneCount = e.LaneCount,
            RoadType = e.RoadType,
            Flags = e.Flags,
            ControlPoint1 = e.ControlPoint2,
            ControlPoint2 = e.ControlPoint1,
        };
        _edges.Add(rev);
        ActiveEdgeCount++;
        Version++;

        RebuildAdjacency();
        RebuildTurnMatrix(e.FromNode);
        RebuildTurnMatrix(e.ToNode);
        return true;
    }

    /// <summary>
    /// Removes a node and all edges connected to it (both incoming and outgoing).
    /// Orphaned nodes left by edge removal are also cleaned up.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to remove.</param>
    public void RemoveNode(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodes.Count) return;
        if (float.IsNaN(_nodes[nodeIndex].Position.X)) return; // already defunct

        // Collect all edges connected to this node (both directions)
        var edgesToRemove = new List<int>();
        for (int i = 0; i < _edges.Count; i++)
        {
            var edge = _edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.FromNode == nodeIndex || edge.ToNode == nodeIndex)
                edgesToRemove.Add(i);
        }

        // Collect the other nodes that will be affected
        var affectedNodes = new HashSet<int>();
        foreach (int ei in edgesToRemove)
        {
            var edge = _edges[ei];
            if (edge.FromNode != nodeIndex) affectedNodes.Add(edge.FromNode);
            if (edge.ToNode != nodeIndex) affectedNodes.Add(edge.ToNode);
        }

        // Mark all connected edges as defunct
        foreach (int ei in edgesToRemove)
        {
            var defunct = _edges[ei];
            defunct.FromNode = -1;
            defunct.ToNode = -1;
            _edges[ei] = defunct;
            ActiveEdgeCount--;
        }

        // Mark the node itself as defunct
        MarkNodeDefunct(nodeIndex);

        RebuildAdjacency();

        // Rebuild turn matrices and clean up orphaned neighbors
        foreach (int n in affectedNodes)
        {
            RebuildTurnMatrix(n);
            if (IsNodeOrphaned(n))
                MarkNodeDefunct(n);
        }

        Version++;
    }

    /// <summary>
    /// Moves a node to a new position and updates the control points and lengths of all
    /// connected edges so the Bézier curves follow the node. Control points translate with
    /// the node, and each edge's handle LENGTHS are then rescaled by the chord-length
    /// change (directions preserved): translation alone keeps the old absolute handle
    /// length, so a drag that lengthens an edge would compress its Bézier parametrization
    /// near one end — breaking the uniform t≈distance convention that all Δt·Length math
    /// relies on (IDM gaps, stop lines, signal detection zones). Proportional rescaling
    /// keeps a chord/3 handle at chord/3 while preserving any hand-tuned handle ratio.
    /// Additionally, when the moved node is the DEAD END of a single road, BOTH of that
    /// edge's handles rotate with the chord (each about its own node; the far node acts as
    /// the swing center) — combined with the rescale this is a similarity transform, so
    /// dragging the free end of an upward road down-and-right swings the whole curve like
    /// a rigid pendulum, the tip ending up pointing right. At shared nodes (2+ roads)
    /// handle directions are preserved, since rotating them would kink the other roads'
    /// joints. Moving back to the start position inverts the rescale and the swing exactly
    /// (drag revert).
    /// </summary>
    /// <param name="nodeIndex">Index of the node to move.</param>
    /// <param name="newPosition">New world-space position.</param>
    public void MoveNode(int nodeIndex, Vector2 newPosition)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodes.Count) return;
        if (float.IsNaN(_nodes[nodeIndex].Position.X)) return;

        var oldPosition = _nodes[nodeIndex].Position;
        var delta = newPosition - oldPosition;

        // Dead-end test for the swing rotation: exactly one distinct road at the node.
        int soleRoadKey = -1;
        bool deadEnd = true;
        void CountRoad(int e)
        {
            if (_edges[e].FromNode < 0) return;
            int rev = FindReverseEdge(e);
            int key = rev >= 0 ? Math.Min(e, rev) : e;
            if (soleRoadKey < 0) soleRoadKey = key;
            else if (key != soleRoadKey) deadEnd = false;
        }
        foreach (int e in GetIncomingEdges(nodeIndex)) CountRoad(e);
        foreach (int e in GetOutgoingEdges(nodeIndex)) CountRoad(e);
        if (soleRoadKey < 0) deadEnd = false;

        var node = _nodes[nodeIndex];
        node.Position = newPosition;
        _nodes[nodeIndex] = node;

        // Update connected edges via adjacency lists — O(degree) instead of O(E)
        void UpdateEdge(int i)
        {
            var edge = _edges[i];
            var p0Old = edge.FromNode == nodeIndex ? oldPosition : _nodes[edge.FromNode].Position;
            var p3Old = edge.ToNode == nodeIndex ? oldPosition : _nodes[edge.ToNode].Position;
            float chordOld = Vector2.Distance(p0Old, p3Old);

            if (edge.FromNode == nodeIndex) edge.ControlPoint1 += delta;
            if (edge.ToNode == nodeIndex) edge.ControlPoint2 += delta;
            var p0 = _nodes[edge.FromNode].Position;
            var p3 = _nodes[edge.ToNode].Position;
            float chordNew = Vector2.Distance(p0, p3);

            if (chordOld > 0.001f && chordNew > 0.001f)
            {
                float scale = chordNew / chordOld;
                edge.ControlPoint1 = p0 + (edge.ControlPoint1 - p0) * scale;
                edge.ControlPoint2 = p3 + (edge.ControlPoint2 - p3) * scale;

                // Dead-end swing: rotate BOTH handles by the chord's rotation (each about
                // its own node) — together with the chord-scale above this is a full
                // similarity transform, so the curve keeps its exact shape relative to
                // the swinging road, like a rigid pendulum.
                if (deadEnd)
                {
                    float inv = 1f / (chordOld * chordNew);
                    var co = p3Old - p0Old;
                    var cn = p3 - p0;
                    float cos = (co.X * cn.X + co.Y * cn.Y) * inv;
                    float sin = (co.X * cn.Y - co.Y * cn.X) * inv;
                    var h1 = edge.ControlPoint1 - p0;
                    edge.ControlPoint1 = p0 + new Vector2(h1.X * cos - h1.Y * sin, h1.X * sin + h1.Y * cos);
                    var h2 = edge.ControlPoint2 - p3;
                    edge.ControlPoint2 = p3 + new Vector2(h2.X * cos - h2.Y * sin, h2.X * sin + h2.Y * cos);
                }
            }

            edge.Length = EstimateBezierLength(p0, edge.ControlPoint1, edge.ControlPoint2, p3);
            _edges[i] = edge;
        }
        foreach (int i in GetOutgoingEdges(nodeIndex)) UpdateEdge(i);
        foreach (int i in GetIncomingEdges(nodeIndex)) UpdateEdge(i);

        Version++;
    }

    /// <summary>Keep-band for Bézier handle length as a fraction of chord: every creation
    /// path builds handles at chord/3 (drawing, straight edges, curved-mode both use d/3;
    /// De Casteljau split halves stay within a few percent), so a measured handle-length
    /// histogram of any healthy map is a razor-sharp spike at 1/3 with 91% inside
    /// [0.30, 0.375]. This ±25% band around 1/3 clears that spike with margin, so no
    /// legitimately-drawn curve is ever touched, while the corruption tail (near-zero and
    /// bloated handles left by the pre-fix MoveNode) falls outside and is repaired.</summary>
    private const float HandleKeepLo = 0.25f;         // chord/4
    private const float HandleKeepHi = 5f / 12f;      // chord·0.4167

    /// <summary>
    /// Repairs edges whose Bézier handle lengths have drifted off the chord/3 convention
    /// (see <see cref="HandleKeepLo"/>). An out-of-band handle skews the curve's
    /// parametrization — arc-length no longer tracks t — which silently corrupts ALL
    /// Δt·Length distance math (IDM gaps, stop lines, signal detection zones); the extreme
    /// case is a near-zero handle producing a metre-scale hairpin on a long arterial that
    /// flings vehicles off-lane. The historical source is the pre-fix <see cref="MoveNode"/>
    /// translating handles without rescaling (a node dragged so its edge grew kept the old
    /// absolute handle, now tiny relative to the new chord). Each out-of-band handle is
    /// rescaled to exactly chord/3, preserving its direction (the curve's tangent intent)
    /// unless it is near-zero (&lt; 5% of chord, direction unreliable) in which case it is
    /// rebuilt along the chord — a vanishing handle means that end was essentially straight.
    /// Because both an edge and its reverse twin carry the same world handle at each shared
    /// node, they heal identically, so two-way pairs stay mirror-consistent. Returns the
    /// edge count repaired. Called after map load; idempotent and safe to call anytime.
    /// </summary>
    public int NormalizeDegenerateHandles()
    {
        int repaired = 0;
        for (int i = 0; i < _edges.Count; i++)
        {
            var edge = _edges[i];
            if (edge.FromNode < 0 || edge.ToNode < 0) continue;
            var p0 = _nodes[edge.FromNode].Position;
            var p3 = _nodes[edge.ToNode].Position;
            float chord = Vector2.Distance(p0, p3);
            if (chord < 0.01f) continue;

            bool changed = false;
            Vector2 Fix(Vector2 cp, Vector2 anchor, Vector2 other)
            {
                var handle = cp - anchor;
                float len = handle.Length();
                if (len >= HandleKeepLo * chord && len <= HandleKeepHi * chord) return cp;
                changed = true;
                var dir = len > 0.05f * chord
                    ? handle / len
                    : Vector2.Normalize(other - anchor); // near-zero: rebuild along chord
                return anchor + dir * (chord / 3f);
            }
            edge.ControlPoint1 = Fix(edge.ControlPoint1, p0, p3);
            edge.ControlPoint2 = Fix(edge.ControlPoint2, p3, p0);

            if (changed)
            {
                edge.Length = EstimateBezierLength(p0, edge.ControlPoint1, edge.ControlPoint2, p3);
                _edges[i] = edge;
                repaired++;
            }
        }
        if (repaired > 0) Version++;
        return repaired;
    }

    /// <summary>Margin (rad, ~1.5°) kept between a clamped handle and the adjacent road it
    /// ran into, so a dragged handle can meet a neighbor but never lie exactly on it
    /// (exactly-collinear legs make degenerate junction geometry).</summary>
    private const float HandleClampMargin = 0.026f;

    /// <summary>
    /// Clamps a control-point drag so the handle's direction at its anchor node cannot
    /// swing PAST any adjacent road at that node — the bottom leg of a T can swing up to
    /// ±90°, meeting either main-road leg, but not through them. The neighbor directions
    /// are the adjacent edges' own node-anchored handles (their tangents at the node), and
    /// the clamp is applied per drag step against the angularly nearest neighbor on each
    /// side of the handle's CURRENT direction, so a fast mouse move cannot tunnel through
    /// a leg. Handle length follows the cursor; only the angle is limited. Returns
    /// <paramref name="desired"/> unchanged when the node has no other roads or the
    /// geometry is degenerate. Editor interaction policy — call before
    /// <see cref="SetControlPoint"/> on drag updates (aborts/reverts restore verbatim).
    /// </summary>
    /// <param name="edgeIndex">Edge whose control point is being dragged.</param>
    /// <param name="controlPointIndex">Which control point: 1 (near FromNode) or 2 (near ToNode).</param>
    /// <param name="desired">Requested world-space control-point position.</param>
    public Vector2 ClampControlPointToNeighbors(int edgeIndex, int controlPointIndex, Vector2 desired)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return desired;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return desired;

        int node = controlPointIndex == 1 ? edge.FromNode : edge.ToNode;
        var nodePos = _nodes[node].Position;
        var cur = controlPointIndex == 1 ? edge.ControlPoint1 : edge.ControlPoint2;

        var desiredVec = desired - nodePos;
        var curVec = cur - nodePos;
        float len = desiredVec.Length();
        if (len < 0.001f || curVec.LengthSquared() < 1e-6f) return desired;

        float thetaPrev = MathF.Atan2(curVec.Y, curVec.X);
        float deltaDes = WrapAnglePi(MathF.Atan2(desiredVec.Y, desiredVec.X) - thetaPrev);

        // Nearest neighbor angle on each side of the current direction (relative,
        // counter-clockwise positive). The dragged road's own reverse twin is the same
        // physical road and never constrains.
        int reverse = FindReverseEdge(edgeIndex);
        float ccw = float.MaxValue, cw = float.MinValue;

        void Consider(int e, bool atToNode)
        {
            if (e == edgeIndex || e == reverse) return;
            var other = _edges[e];
            if (other.FromNode < 0) return;
            var handle = atToNode ? other.ControlPoint2 : other.ControlPoint1;
            var v = handle - nodePos;
            if (v.LengthSquared() < 1e-6f) return;
            float r = WrapAnglePi(MathF.Atan2(v.Y, v.X) - thetaPrev);
            if (r > 0f && r < ccw) ccw = r;
            else if (r < 0f && r > cw) cw = r;
        }

        foreach (int e in GetOutgoingEdges(node)) Consider(e, atToNode: false);
        foreach (int e in GetIncomingEdges(node)) Consider(e, atToNode: true);

        float delta = deltaDes;
        if (deltaDes > 0f && ccw < float.MaxValue)
            delta = MathF.Min(deltaDes, MathF.Max(0f, ccw - HandleClampMargin));
        else if (deltaDes < 0f && cw > float.MinValue)
            delta = MathF.Max(deltaDes, MathF.Min(0f, cw + HandleClampMargin));
        if (delta == deltaDes) return desired;

        float theta = thetaPrev + delta;
        return nodePos + new Vector2(MathF.Cos(theta) * len, MathF.Sin(theta) * len);
    }

    /// <summary>Wraps an angle to (−π, π].</summary>
    private static float WrapAnglePi(float a)
    {
        while (a > MathF.PI) a -= 2f * MathF.PI;
        while (a <= -MathF.PI) a += 2f * MathF.PI;
        return a;
    }

    /// <summary>cos of the tolerance (~10°) within which the two handle directions at a
    /// 2-road node count as anti-parallel, making the node LINKED (<see cref="IsLinkedNode"/>).</summary>
    private const float LinkedNodeCosTol = 0.9848f;

    /// <summary>
    /// True when the node is a LINKED smooth joint: exactly two distinct roads meet there
    /// and their node-anchored handles point in (near) opposite directions, so the pair
    /// reads as one continuous road. Created by drawing a road from another road's dead
    /// end (<see cref="AlignHandleToContinuation"/>) and maintained by the editor syncing
    /// the partner handle during drags (<see cref="SyncLinkedPartner"/>). Derived state —
    /// never persisted: a third leg dissolves it automatically, and a kinked 2-road joint
    /// (beyond the angle tolerance) is not linked, so its handles drag independently.
    /// </summary>
    public bool IsLinkedNode(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodes.Count) return false;
        if (float.IsNaN(_nodes[nodeIndex].Position.X)) return false;
        var nodePos = _nodes[nodeIndex].Position;

        int road0 = -1, road1 = -1;
        Vector2 dir0 = default, dir1 = default;
        int roadCount = 0;

        bool Consider(int e, bool atToNode)
        {
            var edge = _edges[e];
            if (edge.FromNode < 0) return true;
            int rev = FindReverseEdge(e);
            int key = rev >= 0 ? Math.Min(e, rev) : e;
            if (key == road0 || key == road1) return true; // twin of an already-seen road
            var h = (atToNode ? edge.ControlPoint2 : edge.ControlPoint1) - nodePos;
            if (h.LengthSquared() < 1e-6f || roadCount >= 2)
            {
                roadCount = 3; // degenerate handle or third road — not linked
                return false;
            }
            if (roadCount == 0) { road0 = key; dir0 = h; }
            else { road1 = key; dir1 = h; }
            roadCount++;
            return true;
        }

        foreach (int e in GetIncomingEdges(nodeIndex))
            if (!Consider(e, atToNode: true)) return false;
        foreach (int e in GetOutgoingEdges(nodeIndex))
            if (!Consider(e, atToNode: false)) return false;
        if (roadCount != 2) return false;

        return Vector2.Dot(Vector2.Normalize(dir0), Vector2.Normalize(dir1)) <= -LinkedNodeCosTol;
    }

    /// <summary>
    /// Rotates the partner road's handle at a linked node to stay anti-parallel with the
    /// just-moved handle of <paramref name="edgeIndex"/> at its
    /// <paramref name="controlPointIndex"/> end, preserving the partner's handle length —
    /// the drag-time half of the linked-node invariant. No-op unless the node hosts
    /// exactly one other road with valid geometry. Callers decide WHEN the joint is
    /// linked (checked before moving the source); this method deliberately skips the
    /// angle test so one fast drag step cannot break the link mid-drag.
    /// </summary>
    public void SyncLinkedPartner(int edgeIndex, int controlPointIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        int node = controlPointIndex == 1 ? edge.FromNode : edge.ToNode;
        var nodePos = _nodes[node].Position;
        var src = (controlPointIndex == 1 ? edge.ControlPoint1 : edge.ControlPoint2) - nodePos;
        float srcLen = src.Length();
        if (srcLen < 0.001f) return;

        if (!TryGetSingleOtherRoad(node, edgeIndex, out int otherEdge, out bool otherAtToNode)) return;
        var other = _edges[otherEdge];
        var oh = (otherAtToNode ? other.ControlPoint2 : other.ControlPoint1) - nodePos;
        float oLen = oh.Length();
        if (oLen < 0.001f) return;

        SetControlPoint(otherEdge, otherAtToNode ? 2 : 1, nodePos - src * (oLen / srcLen));
    }

    /// <summary>
    /// Rotates <paramref name="edgeIndex"/>'s handle at <paramref name="node"/> to
    /// continue the SINGLE other road at that node (anti-parallel node-anchored handles),
    /// keeping the handle's length — called by the road tool when a new leg starts or
    /// ends on another road's dead end, so the joint is born smooth and LINKED
    /// (<see cref="IsLinkedNode"/>). No-op when the node hosts zero or several other
    /// roads, the edge does not end at the node, or geometry is degenerate.
    /// </summary>
    public void AlignHandleToContinuation(int edgeIndex, int node)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        int cpIndex = edge.FromNode == node ? 1 : edge.ToNode == node ? 2 : 0;
        if (cpIndex == 0) return;
        if (!TryGetSingleOtherRoad(node, edgeIndex, out int otherEdge, out bool otherAtToNode)) return;

        var nodePos = _nodes[node].Position;
        var other = _edges[otherEdge];
        var oh = (otherAtToNode ? other.ControlPoint2 : other.ControlPoint1) - nodePos;
        float oLen = oh.Length();
        if (oLen < 0.001f) return;
        var own = (cpIndex == 1 ? edge.ControlPoint1 : edge.ControlPoint2) - nodePos;
        float ownLen = own.Length();
        if (ownLen < 0.001f) return;

        SetControlPoint(edgeIndex, cpIndex, nodePos - oh * (ownLen / oLen));
    }

    /// <summary>
    /// Finds the single distinct road at <paramref name="node"/> other than
    /// <paramref name="edgeIndex"/> and its reverse twin. False when there are zero or
    /// several. The result is reported with which of its ends touches the node (its
    /// node-anchored handle is CP2 at the ToNode end, CP1 at the FromNode end).
    /// </summary>
    private bool TryGetSingleOtherRoad(int node, int edgeIndex, out int otherEdge, out bool otherAtToNode)
    {
        otherEdge = -1;
        otherAtToNode = false;
        int reverse = FindReverseEdge(edgeIndex);
        int otherKey = -1;

        foreach (int e in GetIncomingEdges(node))
        {
            if (e == edgeIndex || e == reverse || _edges[e].FromNode < 0) continue;
            int rev = FindReverseEdge(e);
            int key = rev >= 0 ? Math.Min(e, rev) : e;
            if (otherKey < 0) { otherKey = key; otherEdge = e; otherAtToNode = true; }
            else if (key != otherKey) return false;
        }
        foreach (int e in GetOutgoingEdges(node))
        {
            if (e == edgeIndex || e == reverse || _edges[e].FromNode < 0) continue;
            int rev = FindReverseEdge(e);
            int key = rev >= 0 ? Math.Min(e, rev) : e;
            if (otherKey < 0) { otherKey = key; otherEdge = e; otherAtToNode = false; }
            else if (key != otherKey) return false;
        }
        return otherKey >= 0;
    }

    /// <summary>
    /// Updates a Bézier control point on an edge, recomputes its arc length, and syncs
    /// the corresponding control point on the reverse edge (if one exists).
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to modify.</param>
    /// <param name="controlPointIndex">Which control point: 1 (near FromNode) or 2 (near ToNode).</param>
    /// <param name="position">New world-space position for the control point.</param>
    public void SetControlPoint(int edgeIndex, int controlPointIndex, Vector2 position)
    {
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;

        if (controlPointIndex == 1)
            edge.ControlPoint1 = position;
        else
            edge.ControlPoint2 = position;

        var p0 = _nodes[edge.FromNode].Position;
        var p3 = _nodes[edge.ToNode].Position;
        edge.Length = EstimateBezierLength(p0, edge.ControlPoint1, edge.ControlPoint2, p3);
        _edges[edgeIndex] = edge;

        // Sync reverse edge: its CP1/CP2 are swapped relative to ours
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0)
        {
            var rev = _edges[reverse];
            // Forward CP1 = Reverse CP2, Forward CP2 = Reverse CP1
            if (controlPointIndex == 1)
                rev.ControlPoint2 = position;
            else
                rev.ControlPoint1 = position;

            var pp0 = _nodes[rev.FromNode].Position;
            var pp3 = _nodes[rev.ToNode].Position;
            rev.Length = EstimateBezierLength(pp0, rev.ControlPoint1, rev.ControlPoint2, pp3);
            _edges[reverse] = rev;
        }

        Version++;
    }

    /// <summary>
    /// Finds the nearest Bézier control point handle to a world position.
    /// Only considers the primary edge of each forward/reverse pair (lower index).
    /// </summary>
    /// <param name="position">World-space position to search from.</param>
    /// <param name="maxDistance">Maximum search distance in meters.</param>
    /// <returns>(edgeIndex, cpIndex) where cpIndex is 1 or 2, or (-1, -1) if none is within range.</returns>
    public (int edgeIndex, int cpIndex) FindNearestControlPoint(Vector2 position, float maxDistance)
    {
        int bestEdge = -1;
        int bestCp = -1;
        float bestDist = maxDistance * maxDistance;

        for (int i = 0; i < _edges.Count; i++)
        {
            var edge = _edges[i];
            if (edge.FromNode < 0) continue;
            // Skip reverse edge of a pair — only interact with the primary (lower index)
            int reverse = FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            float d1 = Vector2.DistanceSquared(position, edge.ControlPoint1);
            if (d1 < bestDist)
            {
                bestDist = d1;
                bestEdge = i;
                bestCp = 1;
            }

            float d2 = Vector2.DistanceSquared(position, edge.ControlPoint2);
            if (d2 < bestDist)
            {
                bestDist = d2;
                bestEdge = i;
                bestCp = 2;
            }
        }

        return (bestEdge, bestCp);
    }

    /// <summary>
    /// Finds the nearest active (non-defunct) node to a world position.
    /// </summary>
    /// <param name="position">World-space position to search from.</param>
    /// <param name="maxDistance">Maximum search distance in meters.</param>
    /// <returns>Index of the nearest node, or -1 if none is within range.</returns>
    public int FindNearestNode(Vector2 position, float maxDistance = float.MaxValue)
    {
        int nearest = -1;
        float nearestDist = maxDistance * maxDistance;

        for (int i = 0; i < _nodes.Count; i++)
        {
            if (float.IsNaN(_nodes[i].Position.X)) continue; // skip defunct
            float dist = Vector2.DistanceSquared(position, _nodes[i].Position);
            if (dist < nearestDist)
            {
                nearestDist = dist;
                nearest = i;
            }
        }

        return nearest;
    }

    /// <summary>
    /// Returns whether a node can have the Destination flag (non-defunct, ≤ 2 outgoing edges).
    /// </summary>
    public bool CanPlaceMarker(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodes.Count) return false;
        var node = _nodes[nodeIndex];
        if (float.IsNaN(node.Position.X)) return false;
        return node.EdgeCount <= 2;
    }

    /// <summary>
    /// Collects all non-defunct node indices that have the given flag set.
    /// </summary>
    public void GetNodesWithFlag(NodeFlags flag, List<int> result)
    {
        result.Clear();
        for (int i = 0; i < _nodes.Count; i++)
        {
            var node = _nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if (node.Flags.HasFlag(flag))
                result.Add(i);
        }
    }

    /// <summary>
    /// Strips the Destination flag from any node that now has more than 2 outgoing edges,
    /// and strips signal flags (TrafficLight/StopSign/Yield/ManualSignal) from any node that is
    /// not a <see cref="IsTrafficControlJunction"/> (bends/dead-ends/pass-throughs that aren't real
    /// intersections — including one-way pass-throughs, while one-way merges are kept).
    /// Runs once per tick via GraphChangeHandler whenever the graph version changed.
    /// Increments the graph version only if any flags were actually stripped (same
    /// conditional pattern as ClearLaneRestrictions). Because GraphChangeHandler caches
    /// its handled version before calling this, a strip leaves the graph one version
    /// ahead — the next tick's run strips nothing, does not bump, and converges (same
    /// pattern as signal auto-assignment).
    /// </summary>
    public void StripMarkerFlagsFromIntersections()
    {
        const NodeFlags signalMask = NodeFlags.TrafficLight | NodeFlags.StopSign | NodeFlags.Yield
            | NodeFlags.ManualSignal | NodeFlags.ActuatedSignal;

        bool stripped = false;
        for (int i = 0; i < _nodes.Count; i++)
        {
            var node = _nodes[i];
            if (float.IsNaN(node.Position.X)) continue;

            if (node.EdgeCount > 2 && (node.Flags & NodeFlags.Destination) != 0)
            {
                node.Flags &= ~NodeFlags.Destination;
                node.PointOfInterest = POIType.None;
                _nodes[i] = node;
                stripped = true;
            }

            // Strip signal flags from non-intersections (not a traffic-control junction)
            if (!IsTrafficControlJunction(i) && (node.Flags & signalMask) != 0)
            {
                node.Flags &= ~signalMask;
                _nodes[i] = node;
                stripped = true;
            }
        }

        if (stripped) Version++;
    }

    /// <summary>
    /// Evaluates a cubic Bézier curve for a given edge at parameter t.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge.</param>
    /// <param name="t">Parametric position (0 = FromNode, 1 = ToNode).</param>
    /// <returns>World-space position on the curve.</returns>
    public Vector2 EvaluateBezier(int edgeIndex, float t)
    {
        var edge = _edges[edgeIndex];
        var p0 = _nodes[edge.FromNode].Position;
        var p1 = edge.ControlPoint1;
        var p2 = edge.ControlPoint2;
        var p3 = _nodes[edge.ToNode].Position;

        float u = 1f - t;
        return u * u * u * p0
             + 3f * u * u * t * p1
             + 3f * u * t * t * p2
             + t * t * t * p3;
    }

    /// <summary>
    /// Evaluates the tangent (first derivative) of a cubic Bézier at parameter t.
    /// The result is not normalized; its length reflects the curve's speed at t.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge.</param>
    /// <param name="t">Parametric position (0 = FromNode, 1 = ToNode).</param>
    /// <returns>Tangent vector at the given point on the curve.</returns>
    public Vector2 EvaluateBezierTangent(int edgeIndex, float t)
    {
        var edge = _edges[edgeIndex];
        var p0 = _nodes[edge.FromNode].Position;
        var p1 = edge.ControlPoint1;
        var p2 = edge.ControlPoint2;
        var p3 = _nodes[edge.ToNode].Position;

        float u = 1f - t;
        return 3f * u * u * (p1 - p0)
             + 6f * u * t * (p2 - p1)
             + 3f * t * t * (p3 - p2);
    }

    /// <summary>
    /// Splits an edge at parameter t, creating a new node at the split point.
    /// If the edge has a paired reverse edge, it is also split and re-paired automatically.
    /// Rebuilds adjacency and turn matrices at all affected nodes.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to split.</param>
    /// <param name="t">Parametric position (0–1) where the split occurs.</param>
    /// <param name="existingMidNode">If >= 0, reuse this node instead of creating a new one.</param>
    /// <returns>(midNode, firstHalfEdge, secondHalfEdge) indices for the forward direction.</returns>
    public (int midNode, int firstHalf, int secondHalf) SplitEdge(int edgeIndex, float t, int existingMidNode = -1)
    {
        var edge = _edges[edgeIndex];

        // Create intersection node or reuse an existing one
        int midNode;
        if (existingMidNode >= 0)
        {
            midNode = existingMidNode;
        }
        else
        {
            var mid = EvaluateBezier(edgeIndex, t);
            midNode = AddNode(mid);
        }

        // Find reverse edge BEFORE splitting (adjacency becomes stale after SplitEdgeSingle)
        int reverseEdge = FindReverseEdge(edgeIndex);

        // Split the primary edge
        var (fwdFirst, fwdSecond) = SplitEdgeSingle(edgeIndex, t, midNode);
        // Carry any lane restrictions from the old edge onto its two halves so a split
        // incident to a restricted node doesn't silently revert that approach to allow-all.
        MigrateLaneRestrictionsForSplit(edgeIndex, fwdFirst, fwdSecond);
        // Migrate external per-edge state (stop/yield exemptions) onto the surviving half.
        EdgeSplit?.Invoke(edgeIndex, fwdFirst, fwdSecond);

        // If reverse exists, split it at (1-t) using the same midNode
        if (reverseEdge >= 0 && _edges[reverseEdge].FromNode >= 0)
        {
            // SplitEdgeSingle maintains ActiveEdgeCount itself (net +1 per direction) —
            // no extra increment here, or the count drifts +1 per two-way split.
            var (revFirst, revSecond) = SplitEdgeSingle(reverseEdge, 1f - t, midNode);
            MigrateLaneRestrictionsForSplit(reverseEdge, revFirst, revSecond);
            EdgeSplit?.Invoke(reverseEdge, revFirst, revSecond);
        }

        // Rebuild compact adjacency from edges (updates EdgeStartIdx/EdgeCount on all nodes)
        RebuildAdjacency();

        // Rebuild turn matrices at all affected nodes
        RebuildTurnMatrix(edge.FromNode);
        RebuildTurnMatrix(midNode);
        RebuildTurnMatrix(edge.ToNode);

        Version++;
        return (midNode, fwdFirst, fwdSecond);
    }

    /// <summary>
    /// Splits a single edge at parameter t using De Casteljau subdivision.
    /// Marks the original edge defunct and adds two new edges.
    /// Does NOT rebuild adjacency, turn matrices, or increment Version — the caller
    /// (<see cref="SplitEdge"/>) is responsible for that.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to split.</param>
    /// <param name="t">Parametric position (0–1) for the split.</param>
    /// <param name="midNode">Index of the node at the split point.</param>
    /// <returns>(firstHalfIndex, secondHalfIndex) of the two new edges.</returns>
    private (int firstHalf, int secondHalf) SplitEdgeSingle(int edgeIndex, float t, int midNode)
    {
        var edge = _edges[edgeIndex];
        var p0 = _nodes[edge.FromNode].Position;
        var p1 = edge.ControlPoint1;
        var p2 = edge.ControlPoint2;
        var p3 = _nodes[edge.ToNode].Position;

        // De Casteljau split at t
        var a = Vector2.Lerp(p0, p1, t);
        var b = Vector2.Lerp(p1, p2, t);
        var c = Vector2.Lerp(p2, p3, t);
        var d = Vector2.Lerp(a, b, t);
        var e2 = Vector2.Lerp(b, c, t);
        var mid = Vector2.Lerp(d, e2, t);

        // First half: fromNode -> midNode
        var edge1 = new RoadEdge
        {
            FromNode = edge.FromNode,
            ToNode = midNode,
            Length = EstimateBezierLength(p0, a, d, mid),
            SpeedLimit = edge.SpeedLimit,
            LaneCount = edge.LaneCount,
            RoadType = edge.RoadType,
            Flags = edge.Flags,
            ControlPoint1 = a,
            ControlPoint2 = d,
        };

        // Second half: midNode -> toNode
        var edge2 = new RoadEdge
        {
            FromNode = midNode,
            ToNode = edge.ToNode,
            Length = EstimateBezierLength(mid, e2, c, p3),
            SpeedLimit = edge.SpeedLimit,
            LaneCount = edge.LaneCount,
            RoadType = edge.RoadType,
            Flags = edge.Flags,
            ControlPoint1 = e2,
            ControlPoint2 = c,
        };

        // Add new edges
        int idx1 = _edges.Count;
        _edges.Add(edge1);

        int idx2 = _edges.Count;
        _edges.Add(edge2);

        // Mark old edge as defunct
        var defunct = _edges[edgeIndex];
        defunct.Length = 0;
        defunct.FromNode = -1;
        defunct.ToNode = -1;
        _edges[edgeIndex] = defunct;
        ActiveEdgeCount++; // net +1: removed 1 defunct, added 2 new

        return (idx1, idx2);
    }

    /// <summary>
    /// Re-keys lane restrictions from a just-split edge onto its two halves, so a split
    /// incident to a restricted node keeps that node's turn customization (lane restrictions
    /// are keyed by edge index, which changes on a split). For old edge <c>F→T</c> split into
    /// <c>firstHalf = F→Mid</c> and <c>secondHalf = Mid→T</c>:
    /// the restriction KEYS (the old edge as an incoming edge at T) move to <c>secondHalf</c>
    /// (the new incoming edge at T); restriction TARGETS pointing at the old edge (as an
    /// outgoing edge from F) retarget to <c>firstHalf</c> (the new outgoing edge from F).
    /// Called for both the primary and reverse split.
    /// </summary>
    private void MigrateLaneRestrictionsForSplit(int oldEdge, int firstHalf, int secondHalf)
    {
        // KEYS: old edge was an incoming edge at its ToNode → secondHalf is now that incoming edge.
        byte laneCount = _edges[secondHalf].LaneCount;
        for (byte lane = 0; lane < laneCount; lane++)
        {
            if (_laneRestrictions.Remove((oldEdge, lane), out var set))
                _laneRestrictions[(secondHalf, lane)] = set;
        }

        // TARGETS: old edge was an outgoing edge from its FromNode → firstHalf is now that
        // outgoing edge. Retarget any (oldEdge, outLane) entry in every restriction set.
        foreach (var set in _laneRestrictions.Values)
        {
            if (set.Count == 0) continue;
            var retarget = new List<(int, byte)>();
            foreach (var (outEdge, outLane) in set)
                if (outEdge == oldEdge) retarget.Add((outEdge, outLane));
            foreach (var (outEdge, outLane) in retarget)
            {
                set.Remove((outEdge, outLane));
                set.Add((firstHalf, outLane));
            }
        }
    }

    /// <summary>
    /// Finds all crossings between the given edge and all other active edges.
    /// Skips reverse edges, shared-node edges, and crossings near endpoints.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to check for crossings.</param>
    /// <returns>List of (otherEdgeIndex, tOnSelf, tOnOther) for each crossing found.</returns>
    public List<(int otherEdge, float tSelf, float tOther)> FindEdgeCrossings(int edgeIndex)
    {
        var crossings = new List<(int, float, float)>();
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return crossings; // defunct
        // A bridge passes over everything — it never crosses at grade.
        if ((edge.Flags & EdgeFlags.Bridge) != 0) return crossings;

        const int segments = 20;
        var selfPoints = SampleBezier(edgeIndex, segments);

        // Compute bounding box of this edge's samples for spatial culling
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        for (int s = 0; s <= segments; s++)
        {
            if (selfPoints[s].X < minX) minX = selfPoints[s].X;
            if (selfPoints[s].Y < minY) minY = selfPoints[s].Y;
            if (selfPoints[s].X > maxX) maxX = selfPoints[s].X;
            if (selfPoints[s].Y > maxY) maxY = selfPoints[s].Y;
        }
        // Expand by a margin to account for control point bulge on other edges
        const float margin = 20f;
        minX -= margin; minY -= margin;
        maxX += margin; maxY += margin;

        for (int other = 0; other < _edges.Count; other++)
        {
            if (other == edgeIndex) continue;
            var otherEdge = _edges[other];
            if (otherEdge.FromNode < 0) continue; // defunct
            // Roads pass UNDER bridges — never cross them at grade.
            if ((otherEdge.Flags & EdgeFlags.Bridge) != 0) continue;

            // Bounding box cull: skip edges whose endpoints and control points
            // are entirely outside our edge's bounding box
            var op0 = _nodes[otherEdge.FromNode].Position;
            var op3 = _nodes[otherEdge.ToNode].Position;
            float oMinX = MathF.Min(MathF.Min(op0.X, op3.X), MathF.Min(otherEdge.ControlPoint1.X, otherEdge.ControlPoint2.X));
            float oMaxX = MathF.Max(MathF.Max(op0.X, op3.X), MathF.Max(otherEdge.ControlPoint1.X, otherEdge.ControlPoint2.X));
            float oMinY = MathF.Min(MathF.Min(op0.Y, op3.Y), MathF.Min(otherEdge.ControlPoint1.Y, otherEdge.ControlPoint2.Y));
            float oMaxY = MathF.Max(MathF.Max(op0.Y, op3.Y), MathF.Max(otherEdge.ControlPoint1.Y, otherEdge.ControlPoint2.Y));
            if (oMaxX < minX || oMinX > maxX || oMaxY < minY || oMinY > maxY)
                continue;

            // Skip reverse edge of a pair — only detect crossings with primary edge
            int otherReverse = FindReverseEdge(other);
            if (otherReverse >= 0 && otherReverse < other) continue;

            // Skip the reverse of our own edge
            if (otherEdge.FromNode == edge.ToNode && otherEdge.ToNode == edge.FromNode) continue;

            // Skip edges that share a node (they connect, not cross)
            if (edge.FromNode == otherEdge.FromNode || edge.FromNode == otherEdge.ToNode ||
                edge.ToNode == otherEdge.FromNode || edge.ToNode == otherEdge.ToNode)
                continue;

            var otherPoints = SampleBezier(other, segments);

            for (int i = 0; i < segments; i++)
            {
                for (int j = 0; j < segments; j++)
                {
                    if (TryLineLineIntersection(
                        selfPoints[i], selfPoints[i + 1],
                        otherPoints[j], otherPoints[j + 1],
                        out float u, out float v))
                    {
                        float tSelf = (i + u) / segments;
                        float tOther = (j + v) / segments;

                        // Skip crossings within a fixed DISTANCE of either road's endpoint
                        // (a split there would be degenerate / coincide with an existing
                        // node). Distance-based, not a t-fraction, so the skipped zone does
                        // not grow with road length — long roads can still be crossed near
                        // their ends.
                        float selfLen = MathF.Max(edge.Length, 0.01f);
                        float otherLen = MathF.Max(otherEdge.Length, 0.01f);
                        float setback = SimConstants.MinSplitSetback;
                        if (tSelf * selfLen < setback || (1f - tSelf) * selfLen < setback ||
                            tOther * otherLen < setback || (1f - tOther) * otherLen < setback)
                            continue;

                        crossings.Add((other, tSelf, tOther));
                    }
                }
            }
        }

        return crossings;
    }

    /// <summary>
    /// Finds where a hypothetical cubic Bezier (the Road tool's planned leg — straight
    /// legs use collinear thirds, which parameterize the cubic exactly linearly) would
    /// cross existing roads, reporting the crossed edge and both curve parameters so the
    /// route planner can place (and slide) the future intersection node on the crossed
    /// edge. Mirrors <see cref="FindEdgeCrossings"/>'s rules (20-segment sampling,
    /// primary edges only, the endpoint setback on both curves) without mutating
    /// anything, so the ghost preview and the committed intersections agree.
    /// <paramref name="ignoreNode"/> excludes edges sharing the segment's start node
    /// (they connect, not cross); <paramref name="ignoreEdge"/> excludes the pending
    /// start anchor's edge and its reverse twin (the segment starts ON that road).
    /// </summary>
    public void FindCurveCrossingsDetailed(Vector2 p0, Vector2 c1, Vector2 c2, Vector2 p3,
        int ignoreNode, int ignoreEdge, List<(int otherEdge, float tSelf, float tOther, Vector2 pos)> result)
    {
        result.Clear();
        float selfLen = MathF.Max(EstimateBezierLength(p0, c1, c2, p3), 0.01f);
        if (selfLen < 0.02f) return;

        const int segments = 20;
        var selfPoints = new Vector2[segments + 1];
        for (int i = 0; i <= segments; i++)
        {
            float t = (float)i / segments;
            float u1 = 1f - t;
            selfPoints[i] = p0 * (u1 * u1 * u1) + c1 * (3f * u1 * u1 * t)
                + c2 * (3f * u1 * t * t) + p3 * (t * t * t);
        }

        // Curve bounding box (control points bound the hull), expanded for the bulge on
        // other edges' control points.
        const float margin = 20f;
        float minX = MathF.Min(MathF.Min(p0.X, p3.X), MathF.Min(c1.X, c2.X)) - margin;
        float maxX = MathF.Max(MathF.Max(p0.X, p3.X), MathF.Max(c1.X, c2.X)) + margin;
        float minY = MathF.Min(MathF.Min(p0.Y, p3.Y), MathF.Min(c1.Y, c2.Y)) - margin;
        float maxY = MathF.Max(MathF.Max(p0.Y, p3.Y), MathF.Max(c1.Y, c2.Y)) + margin;

        float setback = SimConstants.MinSplitSetback;

        for (int other = 0; other < _edges.Count; other++)
        {
            if (other == ignoreEdge) continue;
            var otherEdge = _edges[other];
            if (otherEdge.FromNode < 0) continue; // defunct
            // Roads pass UNDER bridges — never cross them at grade.
            if ((otherEdge.Flags & EdgeFlags.Bridge) != 0) continue;

            // Skip reverse twins: of a two-way pair only the primary is tested, and the
            // ignored anchor edge's twin is skipped along with it.
            int otherReverse = FindReverseEdge(other);
            if (otherReverse >= 0 && (otherReverse < other || otherReverse == ignoreEdge)) continue;

            // Edges sharing the start node connect rather than cross.
            if (ignoreNode >= 0
                && (otherEdge.FromNode == ignoreNode || otherEdge.ToNode == ignoreNode))
                continue;

            // Bounding box cull (same policy as FindEdgeCrossings).
            var op0 = _nodes[otherEdge.FromNode].Position;
            var op3 = _nodes[otherEdge.ToNode].Position;
            float oMinX = MathF.Min(MathF.Min(op0.X, op3.X), MathF.Min(otherEdge.ControlPoint1.X, otherEdge.ControlPoint2.X));
            float oMaxX = MathF.Max(MathF.Max(op0.X, op3.X), MathF.Max(otherEdge.ControlPoint1.X, otherEdge.ControlPoint2.X));
            float oMinY = MathF.Min(MathF.Min(op0.Y, op3.Y), MathF.Min(otherEdge.ControlPoint1.Y, otherEdge.ControlPoint2.Y));
            float oMaxY = MathF.Max(MathF.Max(op0.Y, op3.Y), MathF.Max(otherEdge.ControlPoint1.Y, otherEdge.ControlPoint2.Y));
            if (oMaxX < minX || oMinX > maxX || oMaxY < minY || oMinY > maxY)
                continue;

            var otherPoints = SampleBezier(other, segments);
            float otherLen = MathF.Max(otherEdge.Length, 0.01f);

            for (int i = 0; i < segments; i++)
            {
                for (int j = 0; j < segments; j++)
                {
                    if (TryLineLineIntersection(selfPoints[i], selfPoints[i + 1],
                            otherPoints[j], otherPoints[j + 1], out float u, out float v))
                    {
                        float tSelf = (i + u) / segments;
                        float tOther = (j + v) / segments;

                        // Same endpoint setback the commit applies: crossings within a fixed
                        // distance of either curve's end would coincide with an existing node.
                        if (tSelf * selfLen < setback || (1f - tSelf) * selfLen < setback ||
                            tOther * otherLen < setback || (1f - tOther) * otherLen < setback)
                            continue;

                        result.Add((other, tSelf, tOther,
                            selfPoints[i] + (selfPoints[i + 1] - selfPoints[i]) * u));
                    }
                }
            }
        }
    }

    /// <summary>
    /// Finds all crossing world positions for edges connected to a given node.
    /// Returns the intersection points (for preview rendering) and the detailed
    /// crossing data (for later splitting).
    /// </summary>
    public List<(int connectedEdge, int otherEdge, float tSelf, float tOther, Vector2 position)>
        FindNodeEdgeCrossings(int nodeIndex)
    {
        var results = new List<(int, int, float, float, Vector2)>();
        var visited = new HashSet<int>(); // avoid duplicate edge checks

        for (int i = 0; i < _edges.Count; i++)
        {
            var edge = _edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.FromNode != nodeIndex && edge.ToNode != nodeIndex) continue;

            // Skip reverse edge if we already processed its primary
            int reverse = FindReverseEdge(i);
            if (reverse >= 0 && reverse < i && visited.Contains(reverse)) continue;
            visited.Add(i);

            var crossings = FindEdgeCrossings(i);
            foreach (var (otherEdge, tSelf, tOther) in crossings)
            {
                var pos = EvaluateBezier(i, tSelf);
                results.Add((i, otherEdge, tSelf, tOther, pos));
            }
        }

        return results;
    }

    /// <summary>
    /// Splits all crossing edges connected to a node. Processes one connected edge
    /// at a time, re-detecting crossings after each round to avoid stale edge indices.
    /// </summary>
    public void SplitNodeEdgeCrossings(int nodeIndex)
    {
        // Snapshot which edges are connected to this node before any splits
        var connectedEdges = new HashSet<int>();
        for (int i = 0; i < _edges.Count; i++)
        {
            var edge = _edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.FromNode != nodeIndex && edge.ToNode != nodeIndex) continue;
            // Only primary edge of each reverse pair
            int reverse = FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;
            connectedEdges.Add(i);
        }

        foreach (int edgeIndex in connectedEdges)
        {
            if (_edges[edgeIndex].FromNode < 0) continue;

            // Re-detect crossings fresh — graph may have changed from prior splits
            var crossings = FindEdgeCrossings(edgeIndex);
            if (crossings.Count == 0) continue;

            crossings.Sort((a, b) => a.tSelf.CompareTo(b.tSelf));

            // Step 1: Split all crossed other edges, collect midNodes
            var midNodes = new int[crossings.Count];
            for (int i = 0; i < crossings.Count; i++)
            {
                var (otherEdge, _, tOther) = crossings[i];
                if (_edges[otherEdge].FromNode < 0)
                {
                    midNodes[i] = -1;
                    continue;
                }
                var (midNode, _, _) = SplitEdge(otherEdge, tOther);
                midNodes[i] = midNode;
            }

            // Step 2: Split connected edge at each crossing, reusing midNodes
            int currentEdge = edgeIndex;
            float consumedT = 0f;

            for (int i = 0; i < crossings.Count; i++)
            {
                if (midNodes[i] < 0) continue;
                if (_edges[currentEdge].FromNode < 0) break;

                float localT = (crossings[i].tSelf - consumedT) / (1f - consumedT);
                localT = Math.Clamp(localT, 0.05f, 0.95f);

                var (_, _, secondHalf) = SplitEdge(currentEdge, localT, midNodes[i]);
                currentEdge = secondHalf;
                consumedT = crossings[i].tSelf;
            }
        }
    }

    /// <summary>Samples a Bézier edge at regular intervals and returns the point array.</summary>
    private Vector2[] SampleBezier(int edgeIndex, int segments)
    {
        var points = new Vector2[segments + 1];
        for (int i = 0; i <= segments; i++)
            points[i] = EvaluateBezier(edgeIndex, (float)i / segments);
        return points;
    }

    /// <summary>Tests two line segments for intersection and returns the parametric t values (u, v) if they cross.</summary>
    private static bool TryLineLineIntersection(
        Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2,
        out float u, out float v)
    {
        u = v = 0;
        var d1 = a2 - a1;
        var d2 = b2 - b1;
        float cross = d1.X * d2.Y - d1.Y * d2.X;
        if (MathF.Abs(cross) < 1e-8f) return false; // parallel

        var diff = b1 - a1;
        u = (diff.X * d2.Y - diff.Y * d2.X) / cross;
        v = (diff.X * d1.Y - diff.Y * d1.X) / cross;

        return u >= 0f && u <= 1f && v >= 0f && v <= 1f;
    }

    /// <summary>
    /// Estimates arc length of a cubic Bézier using a 16-segment polyline approximation.
    /// Delegates to <see cref="GeometryUtil.EstimateBezierLength"/>.
    /// </summary>
    private static float EstimateBezierLength(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3)
        => GeometryUtil.EstimateBezierLength(p0, p1, p2, p3);

    // ── Serialization helpers ──────────────────────────────────────────

    /// <summary>
    /// Returns every stored lane restriction, including auto-derived geometry defaults.
    /// For serialization use <see cref="GetUserLaneRestrictions"/> instead.
    /// </summary>
    public IEnumerable<((int inEdge, byte inLane) key, HashSet<(int outEdge, byte outLane)> pairs)> GetAllLaneRestrictions()
    {
        foreach (var kvp in _laneRestrictions)
            yield return (kvp.Key, kvp.Value);
    }

    /// <summary>
    /// Returns only user-customized lane restrictions for serialization (restrictions on
    /// nodes in <see cref="_userLaneRestrictionNodes"/>). Auto-derived geometry defaults are
    /// excluded — they are rebuilt on load by the post-load <c>RebuildWorldCaches</c>.
    /// </summary>
    public IEnumerable<((int inEdge, byte inLane) key, HashSet<(int outEdge, byte outLane)> pairs)> GetUserLaneRestrictions()
    {
        foreach (var kvp in _laneRestrictions)
        {
            int inEdge = kvp.Key.inEdge;
            if (inEdge >= 0 && inEdge < _edges.Count
                && _userLaneRestrictionNodes.Contains(_edges[inEdge].ToNode))
                yield return (kvp.Key, kvp.Value);
        }
    }

    /// <summary>
    /// Replaces the entire restriction set for one (inEdge, inLane) key and increments
    /// the graph version. Unlike ToggleLaneConnection, this overwrites rather than
    /// toggles. Called in a loop by MapSerializer.Load after LoadFromData — the per-call
    /// bumps coalesce into the single lazy rebuild cascade that follows the load — but
    /// the method is a safe general-purpose mutator callable at any time.
    /// </summary>
    public void SetLaneRestriction(int inEdge, byte inLane, HashSet<(int outEdge, byte outLane)> pairs)
    {
        _laneRestrictions[(inEdge, inLane)] = pairs;
        // Loaded/overwritten restrictions are user intent — preserve them from the
        // normalize phase's auto re-derivation (only auto defaults are rebuilt).
        if (inEdge >= 0 && inEdge < _edges.Count)
            _userLaneRestrictionNodes.Add(_edges[inEdge].ToNode);
        Version++;
    }

    /// <summary>
    /// Replaces the entire graph with the given nodes and edges, then rebuilds
    /// adjacency, reverse edge cache, and turn matrix. Used during map load.
    /// </summary>
    public void LoadFromData(List<RoadNode> nodes, List<RoadEdge> edges)
    {
        _nodes.Clear();
        _edges.Clear();
        _laneRestrictions.Clear();
        _userLaneRestrictionNodes.Clear();
        _turnMatrix.Clear();
        _closedEdges.Clear(); // closed state is transient and never persisted

        _nodes.AddRange(nodes);
        _edges.AddRange(edges);

        ActiveEdgeCount = _edges.Count(e => e.FromNode >= 0);
        RebuildAdjacency();
        for (int i = 0; i < _nodes.Count; i++)
            RebuildTurnMatrix(i);
        Version++;
    }
}
