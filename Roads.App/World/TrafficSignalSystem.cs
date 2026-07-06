using System.Diagnostics;
using System.Numerics;

using Roads.App.Core;
using Roads.App.Vehicles;

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
/// Manages traffic light cycling for intersection nodes. <see cref="AutoAssign"/>
/// normalizes the TrafficLight flag on nodes with 4+ incoming edges (unless manually
/// overridden); <see cref="RebuildIfNeeded"/> then projects node flags into internal
/// arrays, groups incoming edges into two opposing phase groups by approach angle, and
/// derives each node's ITE yellow duration from its fastest approach. <see cref="Update"/>
/// runs a per-node six-segment state machine (green→yellow→all-red per group). Green
/// termination depends on the node's control type: fixed-time (default) greens run a
/// fixed duration; actuated greens (<see cref="NodeFlags.ActuatedSignal"/>) gap-out early
/// when their street is empty, extend while it has detected vehicles, max-out against
/// waiting cross demand, and rest in green when the cross street is empty.
/// </summary>
public class TrafficSignalSystem
{
    /// <summary>Per-edge signal state (only meaningful for edges arriving at a traffic-light node).</summary>
    private SignalState[] _edgeSignal = Array.Empty<SignalState>();
    /// <summary>Per-edge phase group assignment (0 or 1).</summary>
    private byte[] _edgePhaseGroup = Array.Empty<byte>();
    /// <summary>Per-edge vehicle-detection flag for actuated control; rebuilt each <see cref="Update"/>
    /// tick when at least one actuated light exists (see <see cref="RebuildEdgeDemand"/>).</summary>
    private bool[] _edgeDemand = Array.Empty<bool>();

    /// <summary>Per-node cycle segment: 0=G0, 1=Y0, 2=all-red, 3=G1, 4=Y1, 5=all-red
    /// (phase group 0's green/yellow, clearance, then group 1's, clearance).</summary>
    private byte[] _nodePhase = Array.Empty<byte>();
    /// <summary>Per-node time (seconds) spent in the current cycle segment.</summary>
    private float[] _nodeTimeInPhase = Array.Empty<float>();
    /// <summary>Per-node ITE yellow duration in seconds (see <see cref="ComputeYellowDuration"/>).</summary>
    private float[] _nodeYellow = Array.Empty<float>();
    /// <summary>Per-node flag indicating whether this node has a traffic light.</summary>
    private bool[] _isTrafficLight = Array.Empty<bool>();
    /// <summary>Per-node actuated-control flag (projected from <see cref="NodeFlags.ActuatedSignal"/>).</summary>
    private bool[] _isActuated = Array.Empty<bool>();
    /// <summary>Per-node phase rotation offset (shifts the split point in phase group computation).</summary>
    private byte[] _nodePhaseRotation = Array.Empty<byte>();
    /// <summary>Count of actuated lights after the last rebuild — gates the per-tick demand pass.</summary>
    private int _actuatedCount;

    /// <summary>Graph version when signal data was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Set when phase rotation changes, forcing a rebuild even if graph version hasn't changed.</summary>
    private bool _dirty;

    /// <summary>Green duration in seconds for fixed-time nodes (typical urban two-phase split).</summary>
    private const float GreenDuration = 30f;
    /// <summary>All-red clearance between phases in seconds (standard practice 1–2 s).</summary>
    private const float AllRedDuration = 2f;
    /// <summary>Actuated control: minimum green before a phase may gap out.</summary>
    private const float MinGreen = 8f;
    /// <summary>Actuated control: maximum green while cross demand is waiting.</summary>
    private const float MaxGreen = 40f;
    /// <summary>Actuated control: detection-zone length upstream of the node in meters
    /// (emulates stop-line detector loops).</summary>
    private const float DetectionDistance = 45f;
    /// <summary>ITE yellow formula: driver perception-reaction time in seconds.</summary>
    private const float YellowReactionTime = 1.0f;
    /// <summary>ITE yellow formula: design deceleration in m/s² (the standard 10 ft/s²).</summary>
    private const float YellowDecel = 3.05f;

    /// <summary>
    /// Gets the signal state for an edge approaching its ToNode.
    /// Debug-asserts that <see cref="RebuildIfNeeded"/> has run at least once (arrays are
    /// sized there). Version currency is deliberately NOT asserted: the renderer reads
    /// signals every frame including while paused, where data may legitimately be one
    /// frame stale.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge to query.</param>
    /// <returns>The current signal state, or <see cref="SignalState.Green"/> if the ToNode has no traffic light.</returns>
    public SignalState GetSignal(int edgeIndex)
    {
        Debug.Assert(_cachedVersion != -1,
            "TrafficSignalSystem.GetSignal called before any RebuildIfNeeded — signal arrays are unsized.");

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
    /// Rotates the traffic light phase grouping at a node. At 4-way intersections this
    /// cycles through the three canonical 2+2 pairings (opposite pairs, then each of the
    /// two adjacent pairings); at other intersections it shifts the folded-angle split
    /// point. The stored counter accumulates per click and is interpreted modulo the
    /// pairing count inside ComputePhaseGroups.
    /// Grows the rotation array on demand, so the write is never dropped and the call is
    /// safe in any order relative to <see cref="RebuildIfNeeded"/>.
    /// </summary>
    public void RotatePhase(int nodeIndex)
    {
        if (nodeIndex < 0) return;
        EnsureRotationCapacity(nodeIndex);
        _nodePhaseRotation[nodeIndex]++;
        _dirty = true;
    }

    /// <summary>
    /// Grows the rotation array to include <paramref name="nodeIndex"/> so setters are
    /// order-independent (callers need not rebuild first); RebuildIfNeeded re-normalizes
    /// array sizes on the next pass.
    /// </summary>
    private void EnsureRotationCapacity(int nodeIndex)
    {
        if (nodeIndex >= _nodePhaseRotation.Length)
            Array.Resize(ref _nodePhaseRotation, nodeIndex + 1);
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
    /// Normalizes auto-assigned traffic-light flags to the current graph topology:
    /// non-manual nodes get NodeFlags.TrafficLight iff they have 4+ incoming edges.
    /// Manual nodes (NodeFlags.ManualSignal) are never touched — their flags are the
    /// truth. Reads and writes only graph node flags (bumping Version on change),
    /// touches no system state, and is idempotent. Runs in the normalize phase of
    /// SimulationLoop.RebuildWorldCaches, BEFORE <see cref="StopSignSystem.AutoAssign"/>
    /// (whose policy reads the TrafficLight flag) and before the pure RebuildIfNeeded calls.
    /// </summary>
    /// <param name="graph">Road graph whose flags to normalize.</param>
    public static void AutoAssign(RoadGraph graph)
    {
        int nodeCount = graph.Nodes.Count;
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;               // defunct
            if (node.Flags.HasFlag(NodeFlags.ManualSignal)) continue; // manual = truth

            bool shouldBeLight = graph.GetIncomingEdges(n).Count >= 4;
            if (shouldBeLight && !node.Flags.HasFlag(NodeFlags.TrafficLight))
                graph.SetNodeFlags(n, node.Flags | NodeFlags.TrafficLight);
            else if (!shouldBeLight && node.Flags.HasFlag(NodeFlags.TrafficLight))
                // The actuated bit goes with the light, so a future re-light starts
                // at the fixed-time default.
                graph.SetNodeFlags(n, node.Flags & ~(NodeFlags.TrafficLight | NodeFlags.ActuatedSignal));
        }
    }

    /// <summary>
    /// Projects node flags into signal arrays when the graph changes or a phase
    /// rotation marked the system dirty: sizes arrays, derives the traffic-light and
    /// actuated sets from node flags, preserves phase state for existing lights
    /// (randomizing new ones), recomputes phase groups and per-node ITE yellow durations,
    /// and initializes per-edge signal states.
    /// Pure read of the graph — performs no mutation. <see cref="AutoAssign"/> must
    /// have normalized flags first whenever the graph changed (the ordering inside
    /// SimulationLoop.RebuildWorldCaches enforces this).
    /// Must be called before <see cref="Update"/> and <see cref="GetSignal"/> each frame.
    /// </summary>
    /// <param name="graph">Road graph to read traffic light flags from.</param>
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
            _edgeDemand = new bool[edgeCount];
        }

        bool[] oldIsTrafficLight = _isTrafficLight;
        byte[] oldPhase = _nodePhase;
        float[] oldTimeInPhase = _nodeTimeInPhase;
        byte[] oldRotation = _nodePhaseRotation;

        if (_isTrafficLight.Length < nodeCount)
        {
            _isTrafficLight = new bool[nodeCount];
            _isActuated = new bool[nodeCount];
            _nodePhase = new byte[nodeCount];
            _nodeTimeInPhase = new float[nodeCount];
            _nodeYellow = new float[nodeCount];
            _nodePhaseRotation = new byte[nodeCount];

            Array.Copy(oldRotation, _nodePhaseRotation, Math.Min(oldRotation.Length, nodeCount));
        }

        // Clear all edge signals to Green — non-traffic-light edges must not retain stale Red
        Array.Clear(_edgeSignal, 0, Math.Min(edgeCount, _edgeSignal.Length));

        // Derive the traffic-light set from node flags (normalized by AutoAssign)
        _actuatedCount = 0;
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            bool isLight = !float.IsNaN(node.Position.X)
                && node.Flags.HasFlag(NodeFlags.TrafficLight);
            _isTrafficLight[n] = isLight;
            if (!isLight) continue;

            _isActuated[n] = node.Flags.HasFlag(NodeFlags.ActuatedSignal);
            if (_isActuated[n]) _actuatedCount++;

            // Preserve phase state for existing lights; start new lights at a random point
            // within a random green so neighboring signals don't run in lockstep.
            if (n < oldIsTrafficLight.Length && oldIsTrafficLight[n])
            {
                _nodePhase[n] = oldPhase[n];
                _nodeTimeInPhase[n] = oldTimeInPhase[n];
            }
            else
            {
                _nodePhase[n] = SimRandom.Next(2) == 0 ? (byte)0 : (byte)3;
                _nodeTimeInPhase[n] = SimRandom.NextSingle() * GreenDuration;
            }

            var incoming = graph.GetIncomingEdges(n);
            _nodeYellow[n] = ComputeYellowDuration(graph, incoming);
            ComputePhaseGroups(graph, n, incoming);
        }

        // Initialize signal states from current phase state
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isTrafficLight[n]) continue;
            UpdateNodeSignals(graph, n);
        }
    }

    /// <summary>
    /// ITE yellow change interval for a node: y = t + v/(2a) (flat-grade kinematic form)
    /// with the fastest incoming approach's speed limit as v, clamped to the 3.0–6.5 s
    /// band used in practice — ~3 s on 25 mph residential streets, ~4.3 s on 45 mph
    /// arterials, ~6 s at highway speeds. Recomputed on rebuild, so speed-limit and
    /// road-type edits retime the yellow.
    /// </summary>
    private static float ComputeYellowDuration(RoadGraph graph, ArraySegment<int> incoming)
    {
        float vMax = 0f;
        foreach (int e in incoming)
        {
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            float v = edge.SpeedLimit > 0f ? edge.SpeedLimit : RoadTypeDefaults.GetDefaultSpeedLimit(edge.RoadType);
            vMax = MathF.Max(vMax, v);
        }
        return Math.Clamp(YellowReactionTime + vMax / (2f * YellowDecel), 3f, 6.5f);
    }

    /// <summary>
    /// Advances every light's phase state machine and updates per-edge signal states.
    /// When any actuated light exists, first refreshes per-edge vehicle detection from
    /// <paramref name="vehicles"/> (skipped entirely on all-fixed-time maps).
    /// <see cref="RebuildIfNeeded"/> must be called before this method each frame
    /// (debug-asserted).
    /// </summary>
    /// <param name="graph">Road graph for incoming-edge lookups.</param>
    /// <param name="vehicles">Vehicle store scanned for detection-zone demand at actuated lights.</param>
    /// <param name="dt">Delta time in seconds since last update.</param>
    public void Update(RoadGraph graph, VehicleStore vehicles, float dt)
    {
        Debug.Assert(_cachedVersion == graph.Version && !_dirty,
            "TrafficSignalSystem.Update: RebuildIfNeeded must run first — signal data is stale relative to the graph.");

        if (_actuatedCount > 0)
            RebuildEdgeDemand(graph, vehicles);

        int nodeCount = Math.Min(graph.Nodes.Count, _isTrafficLight.Length);
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isTrafficLight[n]) continue;

            AdvancePhaseMachine(graph, n, dt);
            UpdateNodeSignals(graph, n);
        }
    }

    /// <summary>
    /// Single pass over driving vehicles marking approaches that have a vehicle inside the
    /// detection zone (the last <see cref="DetectionDistance"/> meters before the node) —
    /// the sim's stand-in for the stop-line detector loops actuated controllers use.
    /// Vehicles already on intersection arcs are ignored (they have left the approach).
    /// </summary>
    private void RebuildEdgeDemand(RoadGraph graph, VehicleStore vehicles)
    {
        int edgeCount = Math.Min(graph.Edges.Count, _edgeDemand.Length);
        Array.Clear(_edgeDemand, 0, edgeCount);

        for (int v = 0; v < vehicles.Count; v++)
        {
            if (vehicles.State[v] != VehicleState.Driving) continue;
            if (vehicles.CurrentArc[v] >= 0) continue;
            int edge = vehicles.CurrentEdge[v];
            if (edge < 0 || edge >= edgeCount || _edgeDemand[edge]) continue;
            var e = graph.Edges[edge];
            if (e.FromNode < 0) continue;

            float remaining = (1f - vehicles.EdgeProgress[v]) * e.Length;
            if (remaining <= DetectionDistance)
                _edgeDemand[edge] = true;
        }
    }

    /// <summary>Whether any incoming approach of the node in the given phase group has a detected vehicle.</summary>
    private bool GroupDemand(RoadGraph graph, int nodeIndex, byte group)
    {
        foreach (int edgeIdx in graph.GetIncomingEdges(nodeIndex))
        {
            if (edgeIdx < 0 || edgeIdx >= _edgeDemand.Length) continue;
            if (_edgePhaseGroup[edgeIdx] == group && _edgeDemand[edgeIdx]) return true;
        }
        return false;
    }

    /// <summary>
    /// Advances a node's six-segment phase machine by one timestep. Greens end on the fixed
    /// timer (fixed-time nodes) or by gap-out/max-out against detected demand (actuated
    /// nodes, which rest in green while the cross street is empty); yellows run the node's
    /// ITE duration; the all-red clearance is fixed. One sim substep is far smaller than any
    /// segment duration, so at most one transition occurs per call.
    /// </summary>
    private void AdvancePhaseMachine(RoadGraph graph, int nodeIndex, float dt)
    {
        byte phase = _nodePhase[nodeIndex];
        float t = _nodeTimeInPhase[nodeIndex] + dt;

        bool advance;
        switch (phase)
        {
            case 0: // group 0 green
            case 3: // group 1 green
                if (_isActuated[nodeIndex])
                {
                    byte greenGroup = phase == 0 ? (byte)0 : (byte)1;
                    bool ownDemand = GroupDemand(graph, nodeIndex, greenGroup);
                    bool crossDemand = GroupDemand(graph, nodeIndex, (byte)(1 - greenGroup));
                    // Gap-out once the green street is empty (after MinGreen), max-out at
                    // MaxGreen — both only when cross demand is actually waiting; with an
                    // empty cross street, rest in green indefinitely.
                    advance = crossDemand && t >= MinGreen && (!ownDemand || t >= MaxGreen);
                    if (!advance) t = MathF.Min(t, MaxGreen); // keep the timer bounded while resting
                }
                else
                {
                    advance = t >= GreenDuration;
                }
                break;
            case 1: // yellows
            case 4:
                advance = t >= _nodeYellow[nodeIndex];
                break;
            default: // 2, 5: all-red clearance
                advance = t >= AllRedDuration;
                break;
        }

        if (advance)
        {
            _nodePhase[nodeIndex] = (byte)((phase + 1) % 6);
            _nodeTimeInPhase[nodeIndex] = 0f;
        }
        else
        {
            _nodeTimeInPhase[nodeIndex] = t;
        }
    }

    /// <summary>
    /// Maps the current cycle segment to signal states for all incoming edges of a node.
    /// Called by <see cref="Update"/> and <see cref="RebuildIfNeeded"/>. During the all-red
    /// clearance segments both groups read Red.
    /// </summary>
    /// <param name="graph">Road graph for incoming-edge lookups.</param>
    /// <param name="nodeIndex">Index of the traffic-light node.</param>
    private void UpdateNodeSignals(RoadGraph graph, int nodeIndex)
    {
        SignalState group0State = SignalState.Red, group1State = SignalState.Red;
        switch (_nodePhase[nodeIndex])
        {
            case 0: group0State = SignalState.Green; break;
            case 1: group0State = SignalState.Yellow; break;
            case 3: group1State = SignalState.Green; break;
            case 4: group1State = SignalState.Yellow; break;
            // 2, 5: all-red clearance — both groups stay Red
        }

        foreach (int edgeIdx in graph.GetIncomingEdges(nodeIndex))
        {
            if (edgeIdx < 0 || edgeIdx >= _edgePhaseGroup.Length) continue;
            _edgeSignal[edgeIdx] = _edgePhaseGroup[edgeIdx] == 0 ? group0State : group1State;
        }
    }

    /// <summary>
    /// Groups incoming edges into two phase groups by approach angle.
    /// Opposing approaches (~180 degrees apart) get the same phase so they're green together.
    /// For 4-way intersections, the user rotation selects among the three canonical 2+2
    /// pairings derived from the circular (unfolded) angle order — opposite pairs, then
    /// each of the two adjacent pairings — so every configuration is reachable and the
    /// result is independent of the direction the roads were drawn.
    /// For other approach counts, uses angle folding (mod pi) so opposite directions map
    /// to the same value, splits at the largest gap in folded space, and the rotation
    /// shifts the split point.
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

        byte rotation = _nodePhaseRotation[nodeIndex];

        if (count == 4)
        {
            // Sort by unfolded angle — the circular order of approaches around the
            // intersection is purely geometric, unlike the folded order, whose
            // within-pair ordering depends on sub-degree tangent noise (draw direction).
            for (int i = 0; i < count - 1; i++)
                for (int j = i + 1; j < count; j++)
                    if (approaches[j].angle < approaches[i].angle)
                        (approaches[i], approaches[j]) = (approaches[j], approaches[i]);

            // The three distinct 2+2 pairings of [a0,a1,a2,a3] in circular order:
            //   pairing 0: {a0,a2} vs {a1,a3} — opposite pairs (the default)
            //   pairing 1: {a0,a1} vs {a2,a3}
            //   pairing 2: {a1,a2} vs {a3,a0}
            int pairing = rotation % 3;
            for (int i = 0; i < count; i++)
            {
                byte group = pairing switch
                {
                    0 => (byte)(i % 2),
                    1 => (byte)(i / 2),
                    _ => (byte)(((i + 3) % 4) / 2),
                };
                _edgePhaseGroup[approaches[i].edgeIdx] = group;
            }
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

    /// <summary>
    /// Restores phase rotations from a saved list, replacing any existing rotations.
    /// Grows the rotation array on demand, so no entry is dropped regardless of call
    /// order relative to <see cref="RebuildIfNeeded"/>.
    /// </summary>
    public void SetPhaseRotations(List<(int nodeIndex, byte rotation)> rotations)
    {
        Array.Clear(_nodePhaseRotation, 0, _nodePhaseRotation.Length);
        int maxIndex = -1;
        foreach (var (node, _) in rotations)
            if (node > maxIndex) maxIndex = node;
        if (maxIndex >= 0)
            EnsureRotationCapacity(maxIndex);
        foreach (var (node, rot) in rotations)
            if (node >= 0) _nodePhaseRotation[node] = rot;
        _dirty = true;
    }
}
