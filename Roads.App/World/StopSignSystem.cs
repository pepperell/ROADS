using Roads.App.Vehicles;

namespace Roads.App.World;

/// <summary>
/// Manages all-way stop sign behavior at intersection nodes. Vehicles must come to a full
/// stop, wait a minimum time, then are granted right-of-way in first-come-first-served order.
/// Auto-assigns stop signs to nodes with 2+ incoming edges that have angular spread and no
/// traffic light or yield (unless manually overridden).
/// </summary>
public class StopSignSystem
{
    /// <summary>Per-node flag indicating whether this node has a stop sign.</summary>
    private bool[] _isStopSign = Array.Empty<bool>();
    /// <summary>Per-node: edge index currently granted right-of-way, or -1 if none.</summary>
    private int[] _currentlyServingEdge = Array.Empty<int>();
    /// <summary>Per-node: simulation time when right-of-way was granted.</summary>
    private float[] _grantedTime = Array.Empty<float>();
    /// <summary>Per-node: vehicle index that was granted right-of-way, or -1 if none.</summary>
    private int[] _servedVehicle = Array.Empty<int>();
    /// <summary>Per-node: simulation time when the intersection becomes clear after a vehicle departs.</summary>
    private float[] _clearanceEndTime = Array.Empty<float>();

    /// <summary>Per-edge: simulation time when the lead vehicle first stopped at the line (NaN if none).</summary>
    private float[] _edgeArrivalTime = Array.Empty<float>();
    /// <summary>Per-edge: whether a vehicle is currently stopped at the stop line.</summary>
    private bool[] _edgeHasStoppedVeh = Array.Empty<bool>();
    /// <summary>Per-edge: highest progress of the lead vehicle on this edge.</summary>
    private float[] _edgeLeadProgress = Array.Empty<float>();
    /// <summary>Per-edge: speed of the lead vehicle.</summary>
    private float[] _edgeLeadSpeed = Array.Empty<float>();
    /// <summary>Per-edge: vehicle index of the lead vehicle, or -1 if none.</summary>
    private int[] _edgeLeadVehicle = Array.Empty<int>();

    /// <summary>Per-edge: true if this edge's ToNode is a stop-sign intersection.</summary>
    private bool[] _isStopSignEdge = Array.Empty<bool>();
    /// <summary>Per-edge: true if this edge is exempt from its node's stop sign (user toggled off).</summary>
    private bool[] _edgeExempt = Array.Empty<bool>();

    /// <summary>Accumulated simulation time in seconds.</summary>
    private float _simTime;
    /// <summary>Graph version when the system was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Set when per-edge exempt flags change, forcing a rebuild even if graph version hasn't changed.</summary>
    private bool _dirty;

    /// <summary>Seconds a vehicle must be stopped before it's eligible to proceed.</summary>
    private const float MinWaitTime = 1.0f;
    /// <summary>Maximum seconds a vehicle can hold right-of-way before it expires.</summary>
    private const float MaxGrantTime = 4.0f;
    /// <summary>Speed threshold (m/s) below which a vehicle is considered "stopped".</summary>
    private const float StopSpeedThreshold = 0.1f;
    /// <summary>Seconds to wait after a vehicle departs before granting the next vehicle.</summary>
    private const float IntersectionClearanceTime = 1.5f;
    /// <summary>Distance in meters from stop line within which a vehicle counts as "at the stop".</summary>
    private const float StopDistanceThreshold = 6f;

    /// <summary>
    /// Checks whether a node has a stop sign.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to query.</param>
    /// <returns><c>true</c> if the node has a stop sign; otherwise <c>false</c>.</returns>
    public bool IsStopSign(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _isStopSign.Length)
            return false;
        return _isStopSign[nodeIndex];
    }

    /// <summary>
    /// Sets whether an edge is exempt from its node's stop sign (toggled off by the user).
    /// </summary>
    public void SetEdgeExempt(int edgeIndex, bool exempt)
    {
        if (edgeIndex < 0 || edgeIndex >= _edgeExempt.Length) return;
        _edgeExempt[edgeIndex] = exempt;
        _dirty = true;
    }

    /// <summary>
    /// Checks whether an edge is exempt from its node's stop sign.
    /// </summary>
    public bool IsEdgeExempt(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edgeExempt.Length) return false;
        return _edgeExempt[edgeIndex];
    }

    /// <summary>
    /// Rebuilds stop sign node/edge flags and auto-assigns stop signs when the graph changes.
    /// Must be called before <see cref="Update"/> and <see cref="GetSignal"/> each frame.
    /// </summary>
    /// <param name="graph">Road graph to analyze for stop sign assignment.</param>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version && !_dirty) return;
        _cachedVersion = graph.Version;
        _dirty = false;

        int nodeCount = graph.Nodes.Count;
        int edgeCount = graph.Edges.Count;

        // Resize per-node arrays
        if (_isStopSign.Length < nodeCount)
        {
            bool[] oldIsStop = _isStopSign;
            int[] oldServing = _currentlyServingEdge;
            float[] oldGranted = _grantedTime;
            int[] oldServedVeh = _servedVehicle;
            float[] oldClearance = _clearanceEndTime;

            _isStopSign = new bool[nodeCount];
            _currentlyServingEdge = new int[nodeCount];
            _grantedTime = new float[nodeCount];
            _servedVehicle = new int[nodeCount];
            _clearanceEndTime = new float[nodeCount];

            // Initialize new entries to -1 (no one being served)
            Array.Fill(_currentlyServingEdge, -1);
            Array.Fill(_servedVehicle, -1);

            // Preserve existing state for nodes that remain
            int copyCount = Math.Min(oldIsStop.Length, nodeCount);
            Array.Copy(oldIsStop, _isStopSign, copyCount);
            Array.Copy(oldServing, 0, _currentlyServingEdge, 0, copyCount);
            Array.Copy(oldGranted, _grantedTime, copyCount);
            Array.Copy(oldServedVeh, 0, _servedVehicle, 0, Math.Min(oldServedVeh.Length, nodeCount));
            Array.Copy(oldClearance, _clearanceEndTime, Math.Min(oldClearance.Length, nodeCount));
        }

        // Resize per-edge arrays
        if (_edgeArrivalTime.Length < edgeCount)
        {
            float[] oldArrival = _edgeArrivalTime;
            bool[] oldStopped = _edgeHasStoppedVeh;
            bool[] oldExempt = _edgeExempt;

            _edgeArrivalTime = new float[edgeCount];
            _edgeHasStoppedVeh = new bool[edgeCount];
            _edgeLeadProgress = new float[edgeCount];
            _edgeLeadSpeed = new float[edgeCount];
            _edgeLeadVehicle = new int[edgeCount];
            _isStopSignEdge = new bool[edgeCount];
            _edgeExempt = new bool[edgeCount];

            int copyCount = Math.Min(oldArrival.Length, edgeCount);
            Array.Copy(oldArrival, _edgeArrivalTime, copyCount);
            Array.Copy(oldStopped, _edgeHasStoppedVeh, copyCount);
            Array.Copy(oldExempt, _edgeExempt, Math.Min(oldExempt.Length, edgeCount));
        }

        // Auto-assign stop signs: nodes with 3+ incoming edges that aren't traffic lights
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) { _isStopSign[n] = false; continue; }

            var incoming = graph.GetIncomingEdges(n);

            // Respect manual overrides; only auto-assign for non-manual nodes
            bool isManual = node.Flags.HasFlag(NodeFlags.ManualSignal);
            bool shouldBeStop = isManual
                ? node.Flags.HasFlag(NodeFlags.StopSign)
                : (incoming.Count >= 3
                    && !node.Flags.HasFlag(NodeFlags.TrafficLight)
                    && !node.Flags.HasFlag(NodeFlags.Yield)
                    && HasAngularSpread(graph, incoming));

            _isStopSign[n] = shouldBeStop;

            if (!isManual)
            {
                if (shouldBeStop && !node.Flags.HasFlag(NodeFlags.StopSign))
                    graph.SetNodeFlags(n, node.Flags | NodeFlags.StopSign);
                else if (!shouldBeStop && node.Flags.HasFlag(NodeFlags.StopSign))
                    graph.SetNodeFlags(n, node.Flags & ~NodeFlags.StopSign);
            }

            if (!shouldBeStop)
                _currentlyServingEdge[n] = -1;
        }

        // Note: new entries are initialized to -1 during array resize above,
        // so no separate initialization pass is needed.

        // Mark which edges lead to stop-sign nodes
        Array.Clear(_isStopSignEdge, 0, _isStopSignEdge.Length);
        for (int e = 0; e < edgeCount; e++)
        {
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            _isStopSignEdge[e] = edge.ToNode < nodeCount && _isStopSign[edge.ToNode] && !_edgeExempt[e];
        }
    }

    /// <summary>
    /// Updates lead-vehicle tracking, detects stopped vehicles at stop lines, and resolves
    /// right-of-way on a first-come-first-served basis.
    /// <see cref="RebuildIfNeeded"/> must be called before this method each frame.
    /// </summary>
    /// <param name="graph">Road graph for edge/node data.</param>
    /// <param name="vehicles">Vehicle store with current positions and speeds.</param>
    /// <param name="stopLines">Stop line cache for stop-t values.</param>
    /// <param name="dt">Delta time in seconds.</param>
    public void Update(RoadGraph graph, VehicleStore vehicles, StopLineCache stopLines, float dt)
    {
        _simTime += dt;

        int edgeCount = Math.Min(graph.Edges.Count, _edgeLeadProgress.Length);
        int nodeCount = Math.Min(graph.Nodes.Count, _isStopSign.Length);

        // Reset lead vehicle tracking
        Array.Clear(_edgeLeadProgress, 0, edgeCount);
        for (int e = 0; e < edgeCount; e++)
        {
            _edgeLeadSpeed[e] = float.MaxValue;
            _edgeLeadVehicle[e] = -1;
        }

        // Single pass: find lead vehicle per stop-sign edge
        // Skip vehicles on intersection arcs — they've already passed through
        for (int v = 0; v < vehicles.Count; v++)
        {
            if (vehicles.State[v] != VehicleState.Driving) continue;
            if (vehicles.CurrentArc[v] >= 0) continue;
            int edge = vehicles.CurrentEdge[v];
            if (edge < 0 || edge >= edgeCount) continue;
            if (!_isStopSignEdge[edge]) continue;

            float progress = vehicles.EdgeProgress[v];
            if (progress > _edgeLeadProgress[edge])
            {
                _edgeLeadProgress[edge] = progress;
                _edgeLeadSpeed[edge] = vehicles.Speed[v];
                _edgeLeadVehicle[edge] = v;
            }
        }

        // Detect stopped vehicles at stop lines
        for (int e = 0; e < edgeCount; e++)
        {
            if (!_isStopSignEdge[e]) continue;

            float stopT = stopLines.GetStopTAtToNode(e);
            float leadProgress = _edgeLeadProgress[e];
            float leadSpeed = _edgeLeadSpeed[e];
            float edgeLength = graph.Edges[e].Length;
            float distToStop = (stopT - leadProgress) * edgeLength;

            bool isStopped = leadProgress > 0f
                && distToStop >= 0f && distToStop < StopDistanceThreshold
                && leadSpeed < StopSpeedThreshold;

            if (isStopped && !_edgeHasStoppedVeh[e])
            {
                // Just arrived at stop line
                _edgeArrivalTime[e] = _simTime;
                _edgeHasStoppedVeh[e] = true;
            }
            else if (!isStopped)
            {
                _edgeHasStoppedVeh[e] = false;
                _edgeArrivalTime[e] = float.NaN;
            }
        }

        // Queue resolution per stop-sign node
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isStopSign[n]) continue;

            int serving = _currentlyServingEdge[n];

            // Check if the served edge's vehicles have departed or grant expired
            if (serving >= 0)
            {
                int servedVeh = _servedVehicle[n];
                bool expired = _simTime - _grantedTime[n] >= MaxGrantTime;

                // Served vehicle departed if: it's no longer the lead on this edge
                // (it transitioned to the next edge, was removed, or a different vehicle is now lead)
                bool departed = servedVeh < 0
                    || _edgeLeadVehicle[serving] != servedVeh
                    || _edgeLeadProgress[serving] == 0f;

                if (departed || expired)
                {
                    _currentlyServingEdge[n] = -1;
                    _servedVehicle[n] = -1;
                    _clearanceEndTime[n] = _simTime + IntersectionClearanceTime;
                    serving = -1;
                }
            }

            // If no one is being served, wait for intersection to clear, then grant next
            if (serving < 0 && _simTime >= _clearanceEndTime[n])
            {
                var incoming = graph.GetIncomingEdges(n);
                float earliestArrival = float.MaxValue;
                int bestEdge = -1;

                foreach (int edgeIdx in incoming)
                {
                    if (edgeIdx < 0 || edgeIdx >= edgeCount) continue;
                    if (!_edgeHasStoppedVeh[edgeIdx]) continue;

                    float waited = _simTime - _edgeArrivalTime[edgeIdx];
                    if (waited < MinWaitTime) continue;

                    if (_edgeArrivalTime[edgeIdx] < earliestArrival)
                    {
                        earliestArrival = _edgeArrivalTime[edgeIdx];
                        bestEdge = edgeIdx;
                    }
                }

                if (bestEdge >= 0)
                {
                    _currentlyServingEdge[n] = bestEdge;
                    _grantedTime[n] = _simTime;
                    _servedVehicle[n] = _edgeLeadVehicle[bestEdge];
                }
            }
        }

    }

    /// <summary>
    /// Gets the signal state for a vehicle approaching a stop-sign intersection.
    /// <see cref="Update"/> must be called before this method each frame.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge the vehicle is on.</param>
    /// <param name="graph">Road graph for edge data.</param>
    /// <param name="vehicleIndex">Vehicle index (currently unused; reserved for per-vehicle grants).</param>
    /// <returns><see cref="SignalState.Green"/> if the edge has right-of-way; otherwise <see cref="SignalState.Red"/>.</returns>
    public SignalState GetSignal(int edgeIndex, RoadGraph graph, int vehicleIndex = -1)
    {
        if (edgeIndex < 0 || edgeIndex >= _isStopSignEdge.Length)
            return SignalState.Green;
        if (!_isStopSignEdge[edgeIndex])
            return SignalState.Green;

        var edge = graph.Edges[edgeIndex];
        if (edge.FromNode < 0) return SignalState.Green;

        int toNode = edge.ToNode;
        if (toNode < 0 || toNode >= _isStopSign.Length) return SignalState.Green;
        if (!_isStopSign[toNode]) return SignalState.Green;

        if (_currentlyServingEdge[toNode] != edgeIndex)
            return SignalState.Red;

        // All vehicles on the served edge get green (all lanes go at once, like a real stop sign)
        return SignalState.Green;
    }

    /// <summary>
    /// Check if incoming edges approach from genuinely different directions (not collinear).
    /// Two roads meeting end-to-end approach from opposite directions (folded angles ≈ same).
    /// A real intersection has at least one pair of incoming edges whose folded angles differ
    /// by more than 30°, accounting for wrap-around at π.
    /// </summary>
    private static bool HasAngularSpread(RoadGraph graph, ArraySegment<int> incoming)
    {
        const float minSpread = MathF.PI / 6f; // 30° threshold
        const int maxEdges = 16;

        Span<float> folded = stackalloc float[maxEdges];
        int count = 0;

        foreach (int edgeIdx in incoming)
        {
            if (count >= maxEdges) break;
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;

            var tangent = graph.EvaluateBezierTangent(edgeIdx, 1.0f);
            float angle = MathF.Atan2(tangent.Y, tangent.X);
            // Fold mod π: opposing directions map to same value
            float f = angle % MathF.PI;
            if (f < 0) f += MathF.PI;
            folded[count++] = f;
        }

        if (count < 2) return false;

        // Check all pairs: if any pair differs by more than the threshold, it's a real intersection.
        // Uses wrap-aware distance in [0, π) space.
        for (int i = 0; i < count; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                float diff = MathF.Abs(folded[i] - folded[j]);
                float wrapDiff = MathF.PI - diff;
                if (MathF.Min(diff, wrapDiff) > minSpread)
                    return true;
            }
        }

        return false;
    }

    // ── Serialization helpers ──────────────────────────────────────────

    /// <summary>Returns indices of all edges marked exempt from stop sign enforcement.</summary>
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
