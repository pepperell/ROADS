using System.Diagnostics;
using Roads.App.Vehicles;

namespace Roads.App.World;

/// <summary>
/// Manages all-way stop sign behavior at intersection nodes. Vehicles must come to a full
/// stop, wait a minimum time, then are granted right-of-way in first-come-first-served order.
/// <see cref="AutoAssign"/> normalizes the StopSign flag on nodes with 3+ incoming edges
/// that have angular spread and no traffic light or yield (unless manually overridden);
/// <see cref="RebuildIfNeeded"/> then projects node flags into internal arrays.
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

    /// <summary>Per-edge: true if this edge is class-exempt — at an AUTO stop node whose
    /// approaches span multiple road classes, the fastest class flows free (minor-road
    /// stop). Recomputed from road types on every rebuild, never persisted, and never
    /// applied at ManualSignal nodes (there the user's explicit exemptions are the truth).</summary>
    private bool[] _classExempt = Array.Empty<bool>();

    /// <summary>Accumulated simulation time in seconds.</summary>
    private float _simTime;
    /// <summary>Graph version when the system was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Set when per-edge exempt flags change, forcing a rebuild even if graph version hasn't changed.</summary>
    private bool _dirty;
    /// <summary>Protocol tracking for debug asserts: true once <see cref="Update"/> has run
    /// after the most recent real rebuild. See <see cref="CanQuery"/>.</summary>
    private bool _updatedSinceRebuild;

    /// <summary>Seconds a vehicle must be stopped before it's eligible to proceed.</summary>
    private const float MinWaitTime = 1.0f;
    /// <summary>Maximum seconds a vehicle can hold right-of-way before it expires.</summary>
    private const float MaxGrantTime = 4.0f;
    /// <summary>Speed threshold (m/s) below which a vehicle is considered "stopped".</summary>
    private const float StopSpeedThreshold = 0.1f;
    /// <summary>Seconds to wait after a vehicle departs before granting the next vehicle.</summary>
    private const float IntersectionClearanceTime = 1.5f;
    /// <summary>Distance in meters from the stop line within which a vehicle's FRONT BUMPER
    /// counts as "at the stop". Measured bumper-to-line (center distance minus the vehicle's
    /// per-type half-length) — a center-based window would never admit long vehicles, whose
    /// centers rest a full half body-length short of the line (6 m for a bus).</summary>
    private const float StopDistanceThreshold = 3.75f;

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
    /// Grows the exempt array on demand, so the write is never dropped and the call is
    /// safe in any order relative to <see cref="RebuildIfNeeded"/>.
    /// </summary>
    public void SetEdgeExempt(int edgeIndex, bool exempt)
    {
        if (edgeIndex < 0) return;
        EnsureExemptCapacity(edgeIndex);
        _edgeExempt[edgeIndex] = exempt;
        _dirty = true;
    }

    /// <summary>
    /// Grows the exempt array to include <paramref name="edgeIndex"/> so setters are
    /// order-independent (callers need not rebuild first); RebuildIfNeeded re-normalizes
    /// array sizes on the next pass.
    /// </summary>
    private void EnsureExemptCapacity(int edgeIndex)
    {
        if (edgeIndex >= _edgeExempt.Length)
            Array.Resize(ref _edgeExempt, edgeIndex + 1);
    }

    /// <summary>
    /// Migrates an edge exemption across a <see cref="RoadGraph.SplitEdge"/> (subscribe to
    /// <see cref="RoadGraph.EdgeSplit"/>). The exemption means "do not stop at this edge's ToNode",
    /// so it follows <paramref name="secondHalf"/> (Mid→ToNode) — the new approach to that node.
    /// Without this, splitting an exempt main-road approach (e.g. when a new driveway is attached to
    /// that road) would silently re-stop the main road at the existing intersection.
    /// </summary>
    public void OnEdgeSplit(int oldEdge, int firstHalf, int secondHalf)
    {
        if (oldEdge < 0 || oldEdge >= _edgeExempt.Length || !_edgeExempt[oldEdge]) return;
        EnsureExemptCapacity(secondHalf);
        _edgeExempt[secondHalf] = true;
        _edgeExempt[oldEdge] = false; // old edge is now defunct
        _dirty = true;
    }

    /// <summary>
    /// Checks whether an edge is EFFECTIVELY exempt from its node's stop sign: either
    /// user-toggled (<see cref="SetEdgeExempt"/>, persisted with the map) or class-exempt
    /// (auto-derived major approach at a minor-road stop). Renderers use this, so exempt
    /// approaches draw neither a stop line nor a sign post.
    /// </summary>
    public bool IsEdgeExempt(int edgeIndex)
    {
        if (edgeIndex < 0) return false;
        return (edgeIndex < _edgeExempt.Length && _edgeExempt[edgeIndex])
            || (edgeIndex < _classExempt.Length && _classExempt[edgeIndex]);
    }

    /// <summary>
    /// True when right-of-way data is current: rebuilt at the graph's version, no pending
    /// dirty toggle, and <see cref="Update"/> has run since the last real rebuild.
    /// <see cref="GetSignal"/> debug-asserts exactly this predicate; diagnostic callers
    /// (e.g. the vehicle dump) gate on it to avoid reading stale tracking data.
    /// </summary>
    public bool CanQuery(RoadGraph graph) =>
        _cachedVersion == graph.Version && !_dirty && _updatedSinceRebuild;

    /// <summary>
    /// Normalizes auto-assigned stop-sign flags to the current graph topology:
    /// non-manual nodes get NodeFlags.StopSign iff they are a traffic-control junction
    /// (<see cref="RoadGraph.IsTrafficControlJunction"/> — a real intersection incl. one-way
    /// merges, not a bend/pass-through), are not a traffic light or yield node, and their
    /// approaches have angular spread. Manual nodes (NodeFlags.ManualSignal) are never
    /// touched — their flags are the truth. Reads and writes only graph node flags
    /// (bumping Version on change), touches no system state, and is idempotent. Runs
    /// in the normalize phase of SimulationLoop.RebuildWorldCaches, AFTER
    /// <see cref="TrafficSignalSystem.AutoAssign"/> (this policy reads the TrafficLight
    /// flag) and before the pure RebuildIfNeeded calls.
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

            var incoming = graph.GetIncomingEdges(n);
            bool shouldBeStop = graph.IsTrafficControlJunction(n)
                && !node.Flags.HasFlag(NodeFlags.TrafficLight)
                && !node.Flags.HasFlag(NodeFlags.Yield)
                && HasAngularSpread(graph, incoming);

            if (shouldBeStop && !node.Flags.HasFlag(NodeFlags.StopSign))
                graph.SetNodeFlags(n, node.Flags | NodeFlags.StopSign);
            else if (!shouldBeStop && node.Flags.HasFlag(NodeFlags.StopSign))
                graph.SetNodeFlags(n, node.Flags & ~NodeFlags.StopSign);
        }
    }

    /// <summary>
    /// Projects node flags into stop-sign arrays when the graph changes or an exemption
    /// marked the system dirty: sizes arrays (preserving FCFS queue state), derives the
    /// stop-sign set from NodeFlags.StopSign, and marks which edges lead to stop-sign
    /// nodes (honoring user exemptions and deriving class-based ones — at a mixed AUTO
    /// junction the fastest road class flows free). Pure read of the graph — performs no mutation.
    /// <see cref="AutoAssign"/> must have normalized flags first whenever the graph
    /// changed (the ordering inside SimulationLoop.RebuildWorldCaches enforces this).
    /// Must be called before <see cref="Update"/> and <see cref="GetSignal"/> each frame.
    /// A real (non-early-out) rebuild invalidates right-of-way queries until the next
    /// <see cref="Update"/> — see <see cref="CanQuery"/>.
    /// </summary>
    /// <param name="graph">Road graph to read stop sign flags from.</param>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version && !_dirty) return;
        _cachedVersion = graph.Version;
        _dirty = false;
        _updatedSinceRebuild = false; // right-of-way queries are stale until the next Update

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
            _classExempt = new bool[edgeCount];

            int copyCount = Math.Min(oldArrival.Length, edgeCount);
            Array.Copy(oldArrival, _edgeArrivalTime, copyCount);
            Array.Copy(oldStopped, _edgeHasStoppedVeh, copyCount);
            Array.Copy(oldExempt, _edgeExempt, Math.Min(oldExempt.Length, edgeCount));
        }

        // Derive the stop-sign set from node flags (normalized by AutoAssign)
        for (int n = 0; n < nodeCount; n++)
        {
            var node = graph.Nodes[n];
            bool isStop = !float.IsNaN(node.Position.X)
                && node.Flags.HasFlag(NodeFlags.StopSign);
            _isStopSign[n] = isStop;

            if (!isStop)
                _currentlyServingEdge[n] = -1;
        }

        // Note: new entries are initialized to -1 during array resize above,
        // so no separate initialization pass is needed.

        // Mark which edges lead to stop-sign nodes. At AUTO stop nodes (not ManualSignal)
        // whose approaches span multiple road classes, the fastest class present is
        // class-exempt: the minor road stops, the major road flows free (minor-road
        // stop). Manual nodes honor only the explicit per-edge exemptions — cycling a
        // node to a manual stop sign is the escape hatch that forces an all-way stop at
        // a mixed junction.
        Array.Clear(_isStopSignEdge, 0, _isStopSignEdge.Length);
        Array.Clear(_classExempt, 0, _classExempt.Length);
        for (int n = 0; n < nodeCount; n++)
        {
            if (!_isStopSign[n]) continue;
            var incoming = graph.GetIncomingEdges(n);

            int minRank = int.MaxValue, maxRank = int.MinValue;
            if (!graph.Nodes[n].Flags.HasFlag(NodeFlags.ManualSignal))
            {
                foreach (int e in incoming)
                {
                    var edge = graph.Edges[e];
                    if (edge.FromNode < 0) continue;
                    int rank = RoadTypeDefaults.GetRoadClassRank(edge.RoadType, (edge.Flags & EdgeFlags.SharedLane) != 0);
                    minRank = Math.Min(minRank, rank);
                    maxRank = Math.Max(maxRank, rank);
                }
            }
            bool mixedClasses = minRank < maxRank;

            foreach (int e in incoming)
            {
                if (e >= _isStopSignEdge.Length) continue;
                var edge = graph.Edges[e];
                if (edge.FromNode < 0) continue;
                if (mixedClasses && RoadTypeDefaults.GetRoadClassRank(edge.RoadType, (edge.Flags & EdgeFlags.SharedLane) != 0) == maxRank)
                    _classExempt[e] = true;
                _isStopSignEdge[e] = !_edgeExempt[e] && !_classExempt[e];
            }
        }
    }

    /// <summary>
    /// Updates lead-vehicle tracking, detects stopped vehicles at stop lines, and resolves
    /// right-of-way on a first-come-first-served basis.
    /// <see cref="RebuildIfNeeded"/> must be called before this method each frame
    /// (debug-asserted).
    /// </summary>
    /// <param name="graph">Road graph for edge/node data.</param>
    /// <param name="vehicles">Vehicle store with current positions and speeds.</param>
    /// <param name="stopLines">Stop line cache for stop-t values.</param>
    /// <param name="dt">Delta time in seconds.</param>
    public void Update(RoadGraph graph, VehicleStore vehicles, StopLineCache stopLines, float dt)
    {
        Debug.Assert(_cachedVersion == graph.Version && !_dirty,
            "StopSignSystem.Update: RebuildIfNeeded must run first — system is stale relative to the graph.");

        _simTime += dt;

        int edgeCount = Math.Min(graph.Edges.Count, _edgeLeadProgress.Length);
        int nodeCount = Math.Min(graph.Nodes.Count, _isStopSign.Length);

        // Reset lead vehicle tracking. Lead progress resets to -1 (not 0) so a vehicle at
        // progress EXACTLY 0.0 still registers as the lead: an arc-exiter lands at the edge's
        // entry trim (often 0.0), and on a short approach it can sit there — front bumper
        // possibly already inside the stop window — while a 0-init made `progress > lead`
        // fail, leaving the approach invisible to the FCFS (the node then served nobody).
        for (int e = 0; e < edgeCount; e++)
        {
            _edgeLeadProgress[e] = -1f;
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
            if (!_isStopSignEdge[e])
            {
                // Clear any stale stopped-state on an edge that is NOT (or is no longer) a stop-sign
                // approach, so the queue resolver below can't treat it as a phantom waiter. This strands
                // otherwise: when an approach is exempted (or its ToNode loses its stop sign) while a car
                // is stopped at its line, detection stops touching the edge, leaving _edgeHasStoppedVeh
                // true with a fixed _edgeArrivalTime. That phantom then wins the FCFS forever (oldest
                // arrival, but lead vehicle -1), starving the node's real approaches.
                if (_edgeHasStoppedVeh[e]) { _edgeHasStoppedVeh[e] = false; _edgeArrivalTime[e] = float.NaN; }
                continue;
            }

            float stopT = stopLines.GetStopTAtToNode(e);
            float leadProgress = _edgeLeadProgress[e];
            float leadSpeed = _edgeLeadSpeed[e];
            float edgeLength = graph.Edges[e].Length;
            float distToStop = (stopT - leadProgress) * edgeLength;

            // Front-bumper distance: the vehicle stops with its center a half body-length
            // short of the line, so subtract the lead vehicle's per-type half-length.
            int leadVeh = _edgeLeadVehicle[e];
            float frontDist = distToStop - (leadVeh >= 0
                ? VehicleTypeDimensions.GetHalfLength(vehicles.PreferredVehicle[leadVeh])
                : 0f);

            bool isStopped = leadVeh >= 0
                && distToStop >= 0f && frontDist < StopDistanceThreshold
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

                // The served vehicle has departed only when IT has actually left the approach:
                // removed, no longer driving, crossed into the intersection (now on an arc), or
                // moved onto a different edge. We deliberately do NOT treat "a different vehicle is
                // now the lead on this edge" as departure: a stop-sign grant turns the WHOLE edge
                // green (all lanes go at once — see GetSignal), so on a multi-lane approach a
                // parallel-lane car can out-creep the served vehicle and steal the tracked lead.
                // The old proxy (_edgeLeadVehicle[serving] != servedVeh) then fired a false departure
                // on the very next tick of every grant, re-arming clearance before either car could
                // cross — a livelock that strands two cars side-by-side at the line forever. Anchoring
                // departure to the served vehicle itself lets the granted edge stay green long enough
                // for its lane(s) to clear; MaxGrantTime still caps a vehicle blocked downstream.
                bool departed = servedVeh < 0
                    || servedVeh >= vehicles.Count
                    || vehicles.State[servedVeh] != VehicleState.Driving
                    || vehicles.CurrentArc[servedVeh] >= 0
                    || vehicles.CurrentEdge[servedVeh] != serving;

                if (departed || expired)
                {
                    // An EXPIRED (not departed) grantee is blocked in place — e.g. its exit
                    // is a shared-lane segment occupied by an OPPOSING vehicle waiting at
                    // this same node, a circular wait only this resolver can break. Send the
                    // blocked approach to the back of the FCFS queue so the other approaches
                    // get a turn (real stop-sign etiquette when the granted car won't go);
                    // re-granting the oldest arrival would pick the same blocked edge forever.
                    if (!departed && serving < _edgeArrivalTime.Length)
                        _edgeArrivalTime[serving] = _simTime;

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
                    // Only serve an approach that actually has a lead vehicle tracked this tick. Without
                    // this, a stale "stopped" flag (see the clear in the detection pass) would select a
                    // phantom edge whose lead is -1; it would be granted, then immediately judged
                    // "departed" (servedVeh < 0) every cycle — a livelock that never serves the genuine
                    // waiters. Defends the resolver against any path that leaves the flag set with no car.
                    if (_edgeLeadVehicle[edgeIdx] < 0) continue;

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

        _updatedSinceRebuild = true;
    }

    /// <summary>
    /// Gets the signal state for a vehicle approaching a stop-sign intersection.
    /// <see cref="Update"/> must be called before this method each frame — debug-asserted
    /// via <see cref="CanQuery"/>; diagnostic callers should gate on that predicate.
    /// </summary>
    /// <param name="edgeIndex">Index of the edge the vehicle is on.</param>
    /// <param name="graph">Road graph for edge data.</param>
    /// <param name="vehicleIndex">Vehicle index (currently unused; reserved for per-vehicle grants).</param>
    /// <returns><see cref="SignalState.Green"/> if the edge has right-of-way; otherwise <see cref="SignalState.Red"/>.</returns>
    public SignalState GetSignal(int edgeIndex, RoadGraph graph, int vehicleIndex = -1)
    {
        Debug.Assert(CanQuery(graph),
            "StopSignSystem.GetSignal: right-of-way data is stale — RebuildIfNeeded then Update must run before queries.");

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

    /// <summary>
    /// Human-readable first-come-first-served state for an approach edge, for the vehicle
    /// diagnostics dump: whether this edge stops here, who currently holds right-of-way, whether
    /// THIS edge is recognised as stopped at the line, how long it has waited, and the post-departure
    /// clearance remaining. Explains a "green never comes" stall that the signal colour alone hides.
    /// </summary>
    public string DescribeStopState(RoadGraph graph, int edgeIndex)
    {
        if (edgeIndex < 0 || edgeIndex >= graph.Edges.Count) return "n/a";
        int node = graph.Edges[edgeIndex].ToNode;
        if (node < 0 || node >= _isStopSign.Length || !_isStopSign[node]) return "ToNode is not a stop-sign node";
        bool exempt = IsEdgeExempt(edgeIndex);
        if (exempt) return $"node {node}: this approach is EXEMPT (does not stop)";

        int served = node < _currentlyServingEdge.Length ? _currentlyServingEdge[node] : -1;
        int servedVeh = node < _servedVehicle.Length ? _servedVehicle[node] : -1;
        bool stoppedHere = edgeIndex < _edgeHasStoppedVeh.Length && _edgeHasStoppedVeh[edgeIndex];
        float arrival = edgeIndex < _edgeArrivalTime.Length ? _edgeArrivalTime[edgeIndex] : float.NaN;
        float waited = float.IsNaN(arrival) ? 0f : _simTime - arrival;
        float clearIn = node < _clearanceEndTime.Length ? MathF.Max(0f, _clearanceEndTime[node] - _simTime) : 0f;
        return $"node {node}: servedEdge={served} servedVeh={servedVeh} thisEdgeAtLine={stoppedHere} " +
               $"waited={waited:F2}s (min {MinWaitTime:F1}s) clearanceIn={clearIn:F2}s";
    }

    /// <summary>
    /// Full FCFS state for a stop-sign node and EVERY one of its incoming edges, for diagnosing a
    /// "green never comes" stall: the node-level serving/clearance, plus per-edge whether it's a live
    /// stop-sign approach, exempt, recognised at the line, how long it's waited, and the tracked lead
    /// vehicle. Reveals which edge (if any) is winning the queue — including a stale/phantom one.
    /// </summary>
    public string DescribeNodeFull(RoadGraph graph, int node)
    {
        if (node < 0 || node >= _isStopSign.Length || !_isStopSign[node]) return "  FCFS: not a stop-sign node";
        var sb = new System.Text.StringBuilder();
        int served = node < _currentlyServingEdge.Length ? _currentlyServingEdge[node] : -1;
        int servedVeh = node < _servedVehicle.Length ? _servedVehicle[node] : -1;
        float grantAge = node < _grantedTime.Length ? _simTime - _grantedTime[node] : -1f;
        float clearIn = node < _clearanceEndTime.Length ? MathF.Max(0f, _clearanceEndTime[node] - _simTime) : 0f;
        sb.AppendLine($"  FCFS node {node}: serving={served} servedVeh={servedVeh} grantAge={grantAge:F2}s "
            + $"clearanceIn={clearIn:F2}s maxGrant={MaxGrantTime:F1}s");
        foreach (int e in graph.GetIncomingEdges(node))
        {
            bool isSS = e < _isStopSignEdge.Length && _isStopSignEdge[e];
            bool exempt = e < _edgeExempt.Length && _edgeExempt[e];
            bool stopped = e < _edgeHasStoppedVeh.Length && _edgeHasStoppedVeh[e];
            float arr = e < _edgeArrivalTime.Length ? _edgeArrivalTime[e] : float.NaN;
            float waited = float.IsNaN(arr) ? -1f : _simTime - arr;
            int lead = e < _edgeLeadVehicle.Length ? _edgeLeadVehicle[e] : -1;
            float leadProg = e < _edgeLeadProgress.Length ? _edgeLeadProgress[e] : 0f;
            sb.AppendLine($"    inEdge {e} ({graph.Edges[e].FromNode}->{graph.Edges[e].ToNode}): "
                + $"isSS={isSS} exempt={exempt} atLine={stopped} waited={waited:F1}s lead={lead} leadProg={leadProg:F3}");
        }
        return sb.ToString().TrimEnd('\r', '\n');
    }

    /// <summary>Returns indices of all edges marked exempt from stop sign enforcement.</summary>
    public List<int> GetExemptEdges()
    {
        var result = new List<int>();
        for (int i = 0; i < _edgeExempt.Length; i++)
            if (_edgeExempt[i]) result.Add(i);
        return result;
    }

    /// <summary>
    /// Restores edge exemptions from a saved list of edge indices, replacing any existing
    /// exemptions. Grows the exempt array on demand, so no entry is dropped regardless of
    /// call order relative to <see cref="RebuildIfNeeded"/>.
    /// </summary>
    public void SetExemptEdges(List<int> edges)
    {
        Array.Clear(_edgeExempt, 0, _edgeExempt.Length);
        int maxIndex = -1;
        foreach (int e in edges)
            if (e > maxIndex) maxIndex = e;
        if (maxIndex >= 0)
            EnsureExemptCapacity(maxIndex);
        foreach (int e in edges)
            if (e >= 0) _edgeExempt[e] = true;
        _dirty = true;
    }
}
