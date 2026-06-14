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
        node.Flags &= ~(NodeFlags.Spawn | NodeFlags.Destination);
        node.PointOfInterest = POIType.None;
        _nodes[nodeIndex] = node;
        _turnMatrix.Remove(nodeIndex);
        _userLaneRestrictionNodes.Remove(nodeIndex);
    }

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
            var node = _nodes[n];
            node.EdgeStartIdx = (ushort)outPos;
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

        // Rebuild reverse edge cache
        int edgeCount = _edges.Count;
        if (_reverseEdgeCache.Length < edgeCount)
            _reverseEdgeCache = new int[edgeCount];
        for (int i = 0; i < edgeCount; i++)
        {
            _reverseEdgeCache[i] = -1;
            var e = _edges[i];
            if (e.FromNode < 0) continue;
            foreach (int candidate in GetOutgoingEdges(e.ToNode))
            {
                if (_edges[candidate].ToNode == e.FromNode)
                {
                    _reverseEdgeCache[i] = candidate;
                    break;
                }
            }
        }
    }

    /// <summary>
    /// Rebuild the turn matrix for a node: allow all incoming->outgoing turns except
    /// U-turns (back onto the reverse of the same road segment). The one exception is a
    /// dead-end: if an incoming edge has no non-U-turn outgoing option, the U-turn is
    /// allowed so a vehicle can turn around at the end of a road (otherwise it would be
    /// trapped and removed). This is flag-agnostic — it applies to any terminal node,
    /// not just Destination/Spawn nodes.
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
                var outE = _edges[outEdge];
                // U-turn: incoming from A->node, outgoing node->A
                if (inE.FromNode == outE.ToNode)
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

    /// <summary>
    /// Sets the lane count for an edge, clamped to [1, 4].
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to modify.</param>
    /// <param name="laneCount">Desired lane count (clamped to 1–4).</param>
    public void SetLaneCount(int edgeIndex, byte laneCount)
    {
        if (edgeIndex < 0 || edgeIndex >= _edges.Count) return;
        var edge = _edges[edgeIndex];
        if (edge.FromNode < 0) return;
        byte clamped = Math.Clamp(laneCount, (byte)1, (byte)4);
        edge.LaneCount = clamped;
        _edges[edgeIndex] = edge;
        // Also update the reverse (opposite direction) edge
        int reverse = FindReverseEdge(edgeIndex);
        if (reverse >= 0)
        {
            var rev = _edges[reverse];
            rev.LaneCount = clamped;
            _edges[reverse] = rev;
        }
        Version++;
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
    /// Moves a node to a new position and updates the control points and lengths
    /// of all connected edges so the Bézier curves follow the node.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to move.</param>
    /// <param name="newPosition">New world-space position.</param>
    public void MoveNode(int nodeIndex, Vector2 newPosition)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodes.Count) return;
        if (float.IsNaN(_nodes[nodeIndex].Position.X)) return;

        var oldPosition = _nodes[nodeIndex].Position;
        var delta = newPosition - oldPosition;

        var node = _nodes[nodeIndex];
        node.Position = newPosition;
        _nodes[nodeIndex] = node;

        // Update connected edges via adjacency lists — O(degree) instead of O(E)
        void UpdateEdge(int i)
        {
            var edge = _edges[i];
            if (edge.FromNode == nodeIndex) edge.ControlPoint1 += delta;
            if (edge.ToNode == nodeIndex) edge.ControlPoint2 += delta;
            var p0 = _nodes[edge.FromNode].Position;
            var p3 = _nodes[edge.ToNode].Position;
            edge.Length = EstimateBezierLength(p0, edge.ControlPoint1, edge.ControlPoint2, p3);
            _edges[i] = edge;
        }
        foreach (int i in GetOutgoingEdges(nodeIndex)) UpdateEdge(i);
        foreach (int i in GetIncomingEdges(nodeIndex)) UpdateEdge(i);

        Version++;
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
    /// Returns whether a node can have Spawn or Destination flags (non-defunct, ≤ 2 outgoing edges).
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
    /// Strips Spawn/Destination flags from any node that now has more than 2 outgoing edges,
    /// and strips signal flags (TrafficLight/StopSign/Yield/ManualSignal) from nodes with fewer
    /// than 3 incoming edges (bends/dead-ends that aren't real intersections).
    /// Runs once per tick via GraphChangeHandler whenever the graph version changed.
    /// Increments the graph version only if any flags were actually stripped (same
    /// conditional pattern as ClearLaneRestrictions). Because GraphChangeHandler caches
    /// its handled version before calling this, a strip leaves the graph one version
    /// ahead — the next tick's run strips nothing, does not bump, and converges (same
    /// pattern as signal auto-assignment).
    /// </summary>
    public void StripMarkerFlagsFromIntersections()
    {
        const NodeFlags signalMask = NodeFlags.TrafficLight | NodeFlags.StopSign | NodeFlags.Yield | NodeFlags.ManualSignal;

        bool stripped = false;
        for (int i = 0; i < _nodes.Count; i++)
        {
            var node = _nodes[i];
            if (float.IsNaN(node.Position.X)) continue;

            if (node.EdgeCount > 2 && (node.Flags & (NodeFlags.Spawn | NodeFlags.Destination)) != 0)
            {
                node.Flags &= ~(NodeFlags.Spawn | NodeFlags.Destination);
                node.PointOfInterest = POIType.None;
                _nodes[i] = node;
                stripped = true;
            }

            // Strip signal flags from non-intersections (< 3 incoming edges)
            if (GetIncomingEdges(i).Count < 3 && (node.Flags & signalMask) != 0)
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

        // If reverse exists, split it at (1-t) using the same midNode
        if (reverseEdge >= 0 && _edges[reverseEdge].FromNode >= 0)
        {
            var (revFirst, revSecond) = SplitEdgeSingle(reverseEdge, 1f - t, midNode);
            MigrateLaneRestrictionsForSplit(reverseEdge, revFirst, revSecond);
            ActiveEdgeCount++; // net +1 for the reverse split too
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

        _nodes.AddRange(nodes);
        _edges.AddRange(edges);

        ActiveEdgeCount = _edges.Count(e => e.FromNode >= 0);
        RebuildAdjacency();
        for (int i = 0; i < _nodes.Count; i++)
            RebuildTurnMatrix(i);
        Version++;
    }
}
