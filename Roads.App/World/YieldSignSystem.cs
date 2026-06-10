using System.Numerics;
using Roads.App;
using Roads.App.Vehicles;

namespace Roads.App.World;

/// <summary>
/// Manages yield sign behavior at intersection nodes. Vehicles approaching a yield node
/// slow down or stop depending on whether cross-traffic is detected on other incoming edges.
/// Returns Green (no traffic), Yellow (traffic approaching), or Red (traffic very close).
/// </summary>
public class YieldSignSystem
{
    /// <summary>Per-node flag indicating whether this node has a yield sign.</summary>
    private bool[] _isYield = Array.Empty<bool>();

    /// <summary>Per-edge highest progress value of the lead vehicle (only for yield-bound edges).</summary>
    private float[] _edgeLeadProgress = Array.Empty<float>();
    /// <summary>Per-edge speed of the lead vehicle approaching the yield intersection.</summary>
    private float[] _edgeLeadSpeed = Array.Empty<float>();
    /// <summary>Per-edge distance in meters from the lead vehicle to the stop line.</summary>
    private float[] _edgeDistToStop = Array.Empty<float>();

    /// <summary>Per-edge flag: true if this edge's ToNode is a yield intersection.</summary>
    private bool[] _isYieldEdge = Array.Empty<bool>();
    /// <summary>Per-edge: true if this edge arrives at a yield node (for cross-traffic tracking, ignores exemptions).</summary>
    private bool[] _isAtYieldNode = Array.Empty<bool>();
    /// <summary>Per-edge: true if this edge is exempt from its node's yield sign (user toggled off).</summary>
    private bool[] _edgeExempt = Array.Empty<bool>();
    /// <summary>Per-edge: true if a vehicle is currently on an arc originating from this edge at a yield node.</summary>
    private bool[] _edgeHasArcVehicle = Array.Empty<bool>();

    /// <summary>Graph version when the system was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Set when per-edge exempt flags change, forcing a rebuild even if graph version hasn't changed.</summary>
    private bool _dirty;

    /// <summary>Cross-product threshold for classifying a turn as left or right (vs straight).</summary>
    private const float TurnThreshold = SimConstants.TurnThreshold;
    /// <summary>Dot-product threshold for classifying two incoming edges as oncoming (roughly opposite).</summary>
    private const float OncomingDotThreshold = -0.5f;

    /// <summary>Meters from stop line within which cross-traffic counts as "approaching".</summary>
    private const float CrossTrafficSearchDist = 20f;
    /// <summary>Meters from stop line within which cross-traffic triggers a full stop (Red).</summary>
    private const float CrossTrafficCloseDist = 8f;
    /// <summary>Speed threshold (m/s) above which a vehicle is considered "moving" cross-traffic.</summary>
    private const float MovingSpeedThreshold = 0.5f;

    /// <summary>
    /// Checks whether a node has a yield sign.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to query.</param>
    /// <returns><c>true</c> if the node has a yield sign; otherwise <c>false</c>.</returns>
    public bool IsYield(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _isYield.Length)
            return false;
        return _isYield[nodeIndex];
    }

    /// <summary>
    /// Sets whether an edge is exempt from its node's yield sign (toggled off by the user).
    /// </summary>
    public void SetEdgeExempt(int edgeIndex, bool exempt)
    {
        if (edgeIndex < 0 || edgeIndex >= _edgeExempt.Length) return;
        _edgeExempt[edgeIndex] = exempt;
        _dirty = true;
    }

    /// <summary>
    /// Checks whether an edge is exempt from its node's yield sign.
    /// </summary>
    public bool IsEdgeExempt(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edgeExempt.Length) return false;
        return _edgeExempt[edgeIndex];
    }

    /// <summary>
    /// Rebuilds yield node and edge flags when the graph changes.
    /// Must be called before <see cref="Update"/> and <see cref="GetSignal"/> each frame.
    /// </summary>
    /// <param name="graph">Road graph to analyze for yield sign assignment.</param>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version && !_dirty) return;
        _cachedVersion = graph.Version;
        _dirty = false;

        int nodeCount = graph.Nodes.Count;
        int edgeCount = graph.Edges.Count;

        if (_isYield.Length < nodeCount)
            _isYield = new bool[nodeCount];

        if (_edgeLeadProgress.Length < edgeCount)
        {
            bool[] oldExempt = _edgeExempt;

            _edgeLeadProgress = new float[edgeCount];
            _edgeLeadSpeed = new float[edgeCount];
            _edgeDistToStop = new float[edgeCount];
            _isYieldEdge = new bool[edgeCount];
            _isAtYieldNode = new bool[edgeCount];
            _edgeExempt = new bool[edgeCount];
            _edgeHasArcVehicle = new bool[edgeCount];

            Array.Copy(oldExempt, _edgeExempt, Math.Min(oldExempt.Length, edgeCount));
        }

        // Mark yield nodes from flags
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) { _isYield[n] = false; continue; }
            _isYield[n] = node.Flags.HasFlag(NodeFlags.Yield);
        }

        // Mark which edges lead to yield nodes
        Array.Clear(_isYieldEdge, 0, _isYieldEdge.Length);
        Array.Clear(_isAtYieldNode, 0, _isAtYieldNode.Length);
        for (int e = 0; e < edgeCount; e++)
        {
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            _isAtYieldNode[e] = edge.ToNode < nodeCount && _isYield[edge.ToNode];
            _isYieldEdge[e] = _isAtYieldNode[e] && !_edgeExempt[e];
        }
    }

    /// <summary>
    /// Scans all driving vehicles to find the lead vehicle on each yield-bound edge.
    /// <see cref="RebuildIfNeeded"/> must be called before this method each frame.
    /// </summary>
    /// <param name="graph">Road graph for edge length lookups.</param>
    /// <param name="vehicles">Vehicle store with current positions and speeds.</param>
    /// <param name="stopLines">Stop line cache for stop-t values.</param>
    /// <param name="dt">Delta time in seconds (unused but kept for interface consistency).</param>
    public void Update(RoadGraph graph, VehicleStore vehicles, StopLineCache stopLines, IntersectionArcCache arcCache, float dt)
    {
        int edgeCount = Math.Min(graph.Edges.Count, _edgeLeadProgress.Length);

        // Reset lead vehicle tracking
        Array.Clear(_edgeLeadProgress, 0, edgeCount);
        for (int e = 0; e < edgeCount; e++)
        {
            _edgeLeadSpeed[e] = float.MaxValue;
            _edgeDistToStop[e] = float.MaxValue;
        }

        // Single pass: find lead vehicle per yield edge
        // Skip vehicles on intersection arcs — they're tracked separately below
        for (int v = 0; v < vehicles.Count; v++)
        {
            if (vehicles.State[v] != VehicleState.Driving) continue;
            if (vehicles.CurrentArc[v] >= 0) continue;
            int edge = vehicles.CurrentEdge[v];
            if (edge < 0 || edge >= edgeCount) continue;
            if (!_isAtYieldNode[edge]) continue;

            float progress = vehicles.EdgeProgress[v];
            if (progress > _edgeLeadProgress[edge])
            {
                _edgeLeadProgress[edge] = progress;
                _edgeLeadSpeed[edge] = vehicles.Speed[v];

                float stopT = stopLines.GetStopTAtToNode(edge);
                float edgeLength = graph.Edges[edge].Length;
                _edgeDistToStop[edge] = (stopT - progress) * edgeLength;
            }
        }

        // Track vehicles actively on arcs at yield nodes
        Array.Clear(_edgeHasArcVehicle, 0, edgeCount);
        for (int v = 0; v < vehicles.Count; v++)
        {
            if (vehicles.State[v] != VehicleState.Driving) continue;
            int arcIdx = vehicles.CurrentArc[v];
            if (arcIdx < 0) continue;
            var arc = arcCache.GetArc(arcIdx);
            int inEdge = arc.IncomingEdge;
            if (inEdge >= 0 && inEdge < edgeCount && _isAtYieldNode[inEdge])
                _edgeHasArcVehicle[inEdge] = true;
        }
    }

    /// <summary>
    /// Gets the signal for a vehicle approaching a yield intersection.
    /// <see cref="Update"/> must be called before this method each frame.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge the vehicle is on.</param>
    /// <param name="graph">Road graph for incoming-edge lookups.</param>
    /// <returns>
    /// <see cref="SignalState.Green"/> if no cross-traffic;
    /// <see cref="SignalState.Yellow"/> if cross-traffic is approaching;
    /// <see cref="SignalState.Red"/> if cross-traffic is very close.
    /// </returns>
    public SignalState GetSignal(int edgeIndex, RoadGraph graph)
    {
        if (edgeIndex < 0 || edgeIndex >= _isYieldEdge.Length)
            return SignalState.Green;
        if (!_isYieldEdge[edgeIndex])
            return SignalState.Green;

        var edge = graph.Edges[edgeIndex];
        if (edge.FromNode < 0) return SignalState.Green;

        int toNode = edge.ToNode;
        if (toNode < 0 || toNode >= _isYield.Length) return SignalState.Green;
        if (!_isYield[toNode]) return SignalState.Green;

        // Check all OTHER incoming edges for cross-traffic
        var incoming = graph.GetIncomingEdges(toNode);
        bool hasCrossTraffic = false;
        bool hasCloseCrossTraffic = false;

        foreach (int otherEdge in incoming)
        {
            if (otherEdge == edgeIndex) continue;
            if (otherEdge < 0 || otherEdge >= _edgeLeadProgress.Length) continue;

            // Vehicle actively on an arc from this edge = close cross-traffic
            if (_edgeHasArcVehicle[otherEdge])
            {
                hasCrossTraffic = true;
                hasCloseCrossTraffic = true;
                continue;
            }

            if (_edgeLeadProgress[otherEdge] == 0f) continue; // no vehicle on this edge

            float dist = _edgeDistToStop[otherEdge];
            float speed = _edgeLeadSpeed[otherEdge];

            // Only count vehicles that are actually moving toward the intersection
            if (dist < 0f) continue;                      // already past stop line
            if (speed < MovingSpeedThreshold) continue;    // stopped or creeping — not a threat

            if (dist < CrossTrafficSearchDist)
            {
                hasCrossTraffic = true;
                if (dist < CrossTrafficCloseDist)
                    hasCloseCrossTraffic = true;
            }
        }

        if (hasCloseCrossTraffic)
            return SignalState.Red;
        if (hasCrossTraffic)
            return SignalState.Yellow;
        return SignalState.Green;
    }

    /// <summary>
    /// Turn-aware yield signal. Determines right-of-way based on the vehicle's intended
    /// turn direction: straight-through traffic has priority, left turns yield to oncoming
    /// through traffic, right turns yield to cross traffic from the left.
    /// Falls back to the turn-unaware overload when turn classification fails.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge the vehicle is on.</param>
    /// <param name="outgoingEdge">Index of the next edge the vehicle will take (turn destination).</param>
    /// <param name="graph">Road graph for tangent and incoming-edge lookups.</param>
    /// <returns>Signal state reflecting turn-specific right-of-way.</returns>
    public SignalState GetSignal(int edgeIndex, int outgoingEdge, RoadGraph graph)
    {
        if (edgeIndex < 0 || edgeIndex >= _isYieldEdge.Length)
            return SignalState.Green;
        if (!_isYieldEdge[edgeIndex])
            return SignalState.Green;

        var edge = graph.Edges[edgeIndex];
        if (edge.FromNode < 0) return SignalState.Green;

        int toNode = edge.ToNode;
        if (toNode < 0 || toNode >= _isYield.Length) return SignalState.Green;
        if (!_isYield[toNode]) return SignalState.Green;

        // Classify the turn this vehicle intends to make
        var turn = ClassifyTurn(edgeIndex, outgoingEdge, graph);

        // Through traffic has priority at yield signs — no need to yield
        if (turn == TurnType.Straight)
            return SignalState.Green;

        // Check cross-traffic, filtered by turn type
        var incoming = graph.GetIncomingEdges(toNode);
        bool hasCrossTraffic = false;
        bool hasCloseCrossTraffic = false;

        foreach (int otherEdge in incoming)
        {
            if (otherEdge == edgeIndex) continue;
            if (otherEdge < 0 || otherEdge >= _edgeLeadProgress.Length) continue;

            // Vehicle actively on an arc = threat regardless of turn type
            if (_edgeHasArcVehicle[otherEdge])
            {
                hasCrossTraffic = true;
                hasCloseCrossTraffic = true;
                continue;
            }

            if (_edgeLeadProgress[otherEdge] == 0f) continue;

            float dist = _edgeDistToStop[otherEdge];
            float speed = _edgeLeadSpeed[otherEdge];
            if (dist < 0f) continue;
            if (speed < MovingSpeedThreshold) continue;

            bool isOncoming = IsOncoming(edgeIndex, otherEdge, graph);

            // Left turn: only yield to oncoming traffic (approaching from opposite direction)
            if (turn == TurnType.Left && !isOncoming) continue;

            // Right turn: only yield to non-oncoming cross traffic (from the left)
            if (turn == TurnType.Right && isOncoming) continue;

            if (dist < CrossTrafficSearchDist)
            {
                hasCrossTraffic = true;
                if (dist < CrossTrafficCloseDist)
                    hasCloseCrossTraffic = true;
            }
        }

        if (hasCloseCrossTraffic)
            return SignalState.Red;
        if (hasCrossTraffic)
            return SignalState.Yellow;
        return SignalState.Green;
    }

    private enum TurnType { Straight, Left, Right }

    /// <summary>
    /// Classifies the turn from an incoming edge to an outgoing edge.
    /// Delegates to <see cref="GeometryUtil.ClassifyTurn"/> for the shared computation.
    /// </summary>
    private static TurnType ClassifyTurn(int inEdge, int outEdge, RoadGraph graph)
    {
        var turn = GeometryUtil.ClassifyTurn(graph, inEdge, outEdge);
        return turn switch
        {
            GeometryUtil.TurnDirection.Right => TurnType.Right,
            GeometryUtil.TurnDirection.Left => TurnType.Left,
            _ => TurnType.Straight,
        };
    }

    /// <summary>
    /// Returns true if otherEdge is approaching from roughly the opposite direction as inEdge
    /// (i.e., oncoming traffic on the same road). Uses dot product of arrival tangents.
    /// </summary>
    private static bool IsOncoming(int inEdge, int otherEdge, RoadGraph graph)
    {
        var inTangent = graph.EvaluateBezierTangent(inEdge, 0.95f);
        var otherTangent = graph.EvaluateBezierTangent(otherEdge, 0.95f);
        float inLen = inTangent.Length();
        float otherLen = otherTangent.Length();
        if (inLen < 0.001f || otherLen < 0.001f) return false;

        float dot = (inTangent.X * otherTangent.X + inTangent.Y * otherTangent.Y)
                    / (inLen * otherLen);
        return dot < OncomingDotThreshold;
    }

    // ── Serialization helpers ──────────────────────────────────────────

    /// <summary>Returns indices of all edges marked exempt from yield enforcement.</summary>
    public List<int> GetExemptEdges()
    {
        var result = new List<int>();
        for (int i = 0; i < _edgeExempt.Length; i++)
            if (_edgeExempt[i]) result.Add(i);
        return result;
    }

    /// <summary>Restores edge exemptions from a saved list of edge indices.</summary>
    public void SetExemptEdges(List<int> edges)
    {
        Array.Clear(_edgeExempt, 0, _edgeExempt.Length);
        foreach (int e in edges)
            if (e >= 0 && e < _edgeExempt.Length) _edgeExempt[e] = true;
        _dirty = true;
    }
}
