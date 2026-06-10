using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// Traffic light phase: Green (go), Yellow (caution), or Red (stop).
/// Also reused by stop-sign and yield systems to communicate go/wait decisions.
/// </summary>
public enum SignalState : byte
{
    /// <summary>Proceed through the intersection.</summary>
    Green,
    /// <summary>Caution — prepare to stop (traffic lights) or slow for cross-traffic (yield).</summary>
    Yellow,
    /// <summary>Stop and wait.</summary>
    Red,
}

/// <summary>
/// Manages traffic light cycling for intersection nodes. Auto-assigns traffic lights
/// to nodes with 4+ incoming edges (unless manually overridden). Groups incoming edges
/// into two opposing phase groups by approach angle, then cycles Green/Yellow/Red
/// on a fixed timer.
/// </summary>
public class TrafficSignalSystem
{
    /// <summary>Per-edge signal state (only meaningful for edges arriving at a traffic-light node).</summary>
    private SignalState[] _edgeSignal = Array.Empty<SignalState>();
    /// <summary>Per-edge phase group assignment (0 or 1).</summary>
    private byte[] _edgePhaseGroup = Array.Empty<byte>();

    /// <summary>Per-node cycle timer in seconds.</summary>
    private float[] _nodeTimer = Array.Empty<float>();
    /// <summary>Per-node flag indicating whether this node has a traffic light.</summary>
    private bool[] _isTrafficLight = Array.Empty<bool>();
    /// <summary>Per-node phase rotation offset (shifts the split point in phase group computation).</summary>
    private byte[] _nodePhaseRotation = Array.Empty<byte>();

    /// <summary>Graph version when signal data was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Set when phase rotation changes, forcing a rebuild even if graph version hasn't changed.</summary>
    private bool _dirty;

    /// <summary>Duration of the green phase in seconds.</summary>
    private const float GreenDuration = 30f;
    /// <summary>Duration of the yellow phase in seconds.</summary>
    private const float YellowDuration = 5f;
    /// <summary>Duration of one phase (green + yellow) in seconds.</summary>
    private const float PhaseDuration = GreenDuration + YellowDuration;
    /// <summary>Duration of a full two-phase cycle in seconds.</summary>
    private const float CycleDuration = PhaseDuration * 2;

    /// <summary>
    /// Gets the signal state for an edge approaching its ToNode.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to query.</param>
    /// <returns>The current signal state, or <see cref="SignalState.Green"/> if the ToNode has no traffic light.</returns>
    public SignalState GetSignal(int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= _edgeSignal.Length)
            return SignalState.Green;
        return _edgeSignal[edgeIndex];
    }

    /// <summary>
    /// Checks whether a node has an active traffic light.
    /// </summary>
    /// <param name="nodeIndex">Index of the node to query.</param>
    /// <returns><c>true</c> if the node has a traffic light; otherwise <c>false</c>.</returns>
    public bool IsTrafficLight(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _isTrafficLight.Length)
            return false;
        return _isTrafficLight[nodeIndex];
    }

    /// <summary>
    /// Rotates the traffic light phase grouping at a node, cycling which pair of
    /// opposing approaches share a green phase.
    /// </summary>
    public void RotatePhase(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodePhaseRotation.Length) return;
        _nodePhaseRotation[nodeIndex]++;
        _dirty = true;
    }

    /// <summary>
    /// Gets the current phase rotation offset for a node.
    /// </summary>
    public byte GetPhaseRotation(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= _nodePhaseRotation.Length) return 0;
        return _nodePhaseRotation[nodeIndex];
    }

    /// <summary>
    /// Rebuilds signal data when the graph changes. Auto-assigns traffic lights
    /// to intersections with 4+ incoming edges (unless manually overridden).
    /// Must be called before <see cref="Update"/> and <see cref="GetSignal"/> each frame.
    /// </summary>
    /// <param name="graph">Road graph to analyze for traffic light assignment.</param>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version && !_dirty) return;
        _cachedVersion = graph.Version;
        _dirty = false;

        int edgeCount = graph.Edges.Count;
        int nodeCount = graph.Nodes.Count;

        // Resize arrays if needed
        if (_edgeSignal.Length < edgeCount)
        {
            _edgeSignal = new SignalState[edgeCount];
            _edgePhaseGroup = new byte[edgeCount];
        }

        bool[] oldIsTrafficLight = _isTrafficLight;
        float[] oldTimer = _nodeTimer;
        byte[] oldRotation = _nodePhaseRotation;

        if (_isTrafficLight.Length < nodeCount)
        {
            _isTrafficLight = new bool[nodeCount];
            _nodeTimer = new float[nodeCount];
            _nodePhaseRotation = new byte[nodeCount];

            Array.Copy(oldRotation, _nodePhaseRotation, Math.Min(oldRotation.Length, nodeCount));
        }

        // Clear all edge signals to Green — non-traffic-light edges must not retain stale Red
        Array.Clear(_edgeSignal, 0, Math.Min(edgeCount, _edgeSignal.Length));

        // Auto-assign traffic lights to nodes with 3+ incoming edges
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue; // defunct

            var incoming = graph.GetIncomingEdges(n);

            // Respect manual overrides; only auto-assign for non-manual nodes
            bool isManual = node.Flags.HasFlag(NodeFlags.ManualSignal);
            bool shouldBeLight = isManual
                ? node.Flags.HasFlag(NodeFlags.TrafficLight)
                : incoming.Count >= 4;

            _isTrafficLight[n] = shouldBeLight;

            if (shouldBeLight)
            {
                // Set flag on node if not already set
                if (!node.Flags.HasFlag(NodeFlags.TrafficLight))
                    graph.SetNodeFlags(n, node.Flags | NodeFlags.TrafficLight);

                // Preserve existing timer, or randomize for new lights
                if (n < oldIsTrafficLight.Length && oldIsTrafficLight[n])
                    _nodeTimer[n] = oldTimer[n];
                else if (_nodeTimer[n] == 0f)
                    _nodeTimer[n] = Random.Shared.NextSingle() * CycleDuration;

                ComputePhaseGroups(graph, n, incoming);
            }
            else if (!isManual)
            {
                // Clear flag if it was auto-set (don't touch manual nodes)
                if (node.Flags.HasFlag(NodeFlags.TrafficLight))
                    graph.SetNodeFlags(n, node.Flags & ~NodeFlags.TrafficLight);
            }
        }

        // Initialize signal states from current timers
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isTrafficLight[n]) continue;
            UpdateNodeSignals(graph, n);
        }
    }

    /// <summary>
    /// Advances signal timers and updates per-edge signal states.
    /// <see cref="RebuildIfNeeded"/> must be called before this method each frame.
    /// </summary>
    /// <param name="graph">Road graph for incoming-edge lookups.</param>
    /// <param name="dt">Delta time in seconds since last update.</param>
    public void Update(RoadGraph graph, float dt)
    {
        int nodeCount = Math.Min(graph.Nodes.Count, _isTrafficLight.Length);
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isTrafficLight[n]) continue;

            _nodeTimer[n] += dt;
            if (_nodeTimer[n] >= CycleDuration)
                _nodeTimer[n] -= CycleDuration;

            UpdateNodeSignals(graph, n);
        }
    }

    /// <summary>
    /// Maps the current timer value to signal states for all incoming edges of a node.
    /// Called by <see cref="Update"/> and <see cref="RebuildIfNeeded"/>.
    /// </summary>
    /// <param name="graph">Road graph for incoming-edge lookups.</param>
    /// <param name="nodeIndex">Index of the traffic-light node.</param>
    private void UpdateNodeSignals(RoadGraph graph, int nodeIndex)
    {
        float timer = _nodeTimer[nodeIndex];
        var incoming = graph.GetIncomingEdges(nodeIndex);

        // Determine state for each phase group
        // [0, Green)                        -> group 0 = Green,  group 1 = Red
        // [Green, Phase)                    -> group 0 = Yellow, group 1 = Red
        // [Phase, Phase+Green)              -> group 0 = Red,    group 1 = Green
        // [Phase+Green, Cycle)              -> group 0 = Red,    group 1 = Yellow
        SignalState group0State, group1State;

        if (timer < GreenDuration)
        {
            group0State = SignalState.Green;
            group1State = SignalState.Red;
        }
        else if (timer < PhaseDuration)
        {
            group0State = SignalState.Yellow;
            group1State = SignalState.Red;
        }
        else if (timer < PhaseDuration + GreenDuration)
        {
            group0State = SignalState.Red;
            group1State = SignalState.Green;
        }
        else
        {
            group0State = SignalState.Red;
            group1State = SignalState.Yellow;
        }

        foreach (int edgeIdx in incoming)
        {
            if (edgeIdx < 0 || edgeIdx >= _edgePhaseGroup.Length) continue;
            _edgeSignal[edgeIdx] = _edgePhaseGroup[edgeIdx] == 0 ? group0State : group1State;
        }
    }

    /// <summary>
    /// Groups incoming edges into two phase groups by approach angle.
    /// Opposing approaches (~180 degrees apart) get the same phase so they're green together.
    /// Uses angle folding (mod pi) so opposite directions map to the same value,
    /// then splits at the largest gap in folded space.
    /// </summary>
    /// <param name="graph">Road graph for tangent evaluation.</param>
    /// <param name="nodeIndex">Index of the traffic-light node.</param>
    /// <param name="incoming">Incoming edge indices at this node.</param>
    private void ComputePhaseGroups(RoadGraph graph, int nodeIndex, ArraySegment<int> incoming)
    {
        if (incoming.Count == 0) return;

        // Compute approach angle for each incoming edge (tangent at t=1)
        Span<(int edgeIdx, float angle, float folded)> approaches = stackalloc (int, float, float)[incoming.Count];
        int count = 0;

        foreach (int edgeIdx in incoming)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue; // defunct

            var tangent = graph.EvaluateBezierTangent(edgeIdx, 1.0f);
            float angle = MathF.Atan2(tangent.Y, tangent.X);
            // Fold mod π: opposing directions (differ by π) map to the same value
            float folded = angle % MathF.PI;
            if (folded < 0) folded += MathF.PI;
            approaches[count++] = (edgeIdx, angle, folded);
        }

        if (count <= 1)
        {
            for (int i = 0; i < count; i++)
                _edgePhaseGroup[approaches[i].edgeIdx] = 0;
            return;
        }

        // Sort by folded angle
        for (int i = 0; i < count - 1; i++)
            for (int j = i + 1; j < count; j++)
                if (approaches[j].folded < approaches[i].folded)
                    (approaches[i], approaches[j]) = (approaches[j], approaches[i]);

        // Find largest gap in folded space [0, π)
        float maxGap = 0f;
        int splitAfter = 0;

        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            float gap = approaches[next].folded - approaches[i].folded;
            if (next == 0) gap += MathF.PI; // wrap around in [0, π)
            if (gap > maxGap)
            {
                maxGap = gap;
                splitAfter = i;
            }
        }

        // Apply user-configured phase rotation to shift the split point
        byte rotation = _nodePhaseRotation[nodeIndex];
        if (rotation > 0 && count > 2)
            splitAfter = (splitAfter + rotation) % count;

        // Assign groups: edges after the split are group 0, before are group 1
        for (int i = 0; i < count; i++)
        {
            int idx = (splitAfter + 1 + i) % count;
            _edgePhaseGroup[approaches[idx].edgeIdx] = (byte)(i < (count + 1) / 2 ? 0 : 1);
        }
    }

    // ── Serialization helpers ──────────────────────────────────────────

    /// <summary>Returns all non-zero phase rotations for serialization.</summary>
    public List<(int nodeIndex, byte rotation)> GetPhaseRotations()
    {
        var result = new List<(int, byte)>();
        for (int i = 0; i < _nodePhaseRotation.Length; i++)
            if (_nodePhaseRotation[i] != 0) result.Add((i, _nodePhaseRotation[i]));
        return result;
    }

    /// <summary>Restores phase rotations from a saved list.</summary>
    public void SetPhaseRotations(List<(int nodeIndex, byte rotation)> rotations)
    {
        Array.Clear(_nodePhaseRotation, 0, _nodePhaseRotation.Length);
        foreach (var (node, rot) in rotations)
            if (node >= 0 && node < _nodePhaseRotation.Length) _nodePhaseRotation[node] = rot;
        _dirty = true;
    }
}
