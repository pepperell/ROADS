using System.Numerics;
using Roads.App;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// PD steering controller and IDM (Intelligent Driver Model) car-following for vehicles.
/// Computes steering angle from heading error + lateral offset, and throttle/brake from
/// IDM acceleration with signal compliance, collision avoidance, and yield behavior.
/// Vehicles on intersection arcs use the same PD controller targeting the arc Bezier.
/// </summary>
public static class SteeringController
{
    /// <summary>Proportional gain for heading error in PD steering.</summary>
    public static float Kp = 2.4f;
    /// <summary>Derivative gain for heading error rate in PD steering.</summary>
    public static float Kd = 0.08f;
    /// <summary>Maximum front-wheel steering angle in radians.</summary>
    public static float MaxSteer = 0.7f;
    /// <summary>Default target speed in m/s (~22 mph), used when edge has no speed limit.</summary>
    public static float TargetSpeed = 10f;
    /// <summary>Base lookahead distance in meters at zero speed.</summary>
    public static float LookaheadBase = 3f;
    /// <summary>Additional lookahead distance per m/s of speed.</summary>
    public static float LookaheadPerSpeed = 0.3f;
    /// <summary>Gain for lateral error correction (cross-track error).</summary>
    public static float Klat = 0.5f;
    /// <summary>Lane width in meters.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;

    /// <summary>Search radius in meters for nearby vehicle queries.</summary>
    private const float CollisionSearchRadius = 30f;
    /// <summary>Vehicle body length in meters.</summary>
    private const float VehicleLength = SimConstants.VehicleLength;


    /// <summary>IDM minimum bumper-to-bumper gap in meters (s0).</summary>
    private const float IdmMinGap = 1.0f;
    /// <summary>IDM desired time headway in seconds (T).</summary>
    private const float IdmTimeHeadway = 1.5f;
    /// <summary>IDM maximum acceleration in m/s^2 (a).</summary>
    private const float IdmMaxAccel = SimConstants.MaxAccel;
    /// <summary>IDM comfortable deceleration in m/s^2 (b).</summary>
    private const float IdmComfortDecel = 2.5f;
    /// <summary>Maximum brake deceleration in m/s^2.</summary>
    private const float MaxBrakeDecel = SimConstants.MaxBrakeDecel;

    /// <summary>Thread-local buffer for spatial grid query results.</summary>
    [ThreadStatic] private static List<int>? _nearbyBuffer;

    /// <summary>Previous tick's PathIndex per vehicle, for cross-tick skip detection.</summary>
    private static int[] _prevPathIdx = Array.Empty<int>();
    /// <summary>Previous tick's EdgeProgress per vehicle, for progress-jump detection.</summary>
    private static float[] _prevEdgeProg = Array.Empty<float>();
    /// <summary>Previous tick's CurrentEdge per vehicle.</summary>
    private static int[] _prevEdge = Array.Empty<int>();

    /// <summary>
    /// Per-pass arc-occupancy index, reused across ticks. Rebuilt at the top of <see cref="UpdateAll"/>
    /// and kept live as vehicles enter/exit arcs during the pass — every CurrentArc write in this class
    /// goes through <see cref="SetArc"/>, so the arc-conflict check (formerly an O(n) full-vehicle scan)
    /// observes mid-pass arc entries and stays exactly equivalent.
    /// </summary>
    private static readonly ArcOccupancyIndex _arcOccupancy = new();

    /// <summary>Thread-local buffer for the merge-into-exit-lane spatial query.</summary>
    [ThreadStatic] private static List<int>? _scan2Buffer;

    /// <summary>
    /// DEBUG tripwire: ordered count of vehicle pairs simultaneously occupying mutually-conflicting
    /// arcs after the last <see cref="UpdateAll"/>. On an uncontrolled network this must stay 0 — a
    /// nonzero value means two vehicles are inside conflicting intersection arcs at once (a collision
    /// regression). Computed only in DEBUG builds (0 in Release); surfaced in benchmark.log.
    /// </summary>
    public static int LastConflictCoOccupancy;

    /// <summary>
    /// Sets a vehicle's CurrentArc and keeps <see cref="_arcOccupancy"/> consistent in lock-step.
    /// EVERY CurrentArc write during the steering pass must route through here — otherwise the
    /// occupancy index goes stale and scan #1 misses a conflict (→ vehicles cross at intersections).
    /// <paramref name="newArc"/> = -1 means the vehicle has left all arcs (back on an edge).
    /// </summary>
    private static void SetArc(VehicleStore store, int index, int newArc)
    {
        int old = store.CurrentArc[index];
        if (old >= 0) _arcOccupancy.Exit(old, index);
        store.CurrentArc[index] = newArc;
        if (newArc >= 0) _arcOccupancy.Enter(newArc, index);
    }

    // --- TEMP diagnostic: per-vehicle steering sub-phase timing, to locate the dominant cost.
    // Remove once the hot phase is identified and fixed. ---
    /// <summary>Per-tick steering sub-phase wall time in ms (summed over substeps) from the last UpdateAll.</summary>
    public struct SteeringProfile { public double ArcMs, ProjectMs, SignalsMs, TransitionMs, SteerMs, SpeedMs; }
    /// <summary>Most recent steering sub-phase breakdown (TEMP diagnostic).</summary>
    public static SteeringProfile LastProfile;
    private static long _tArc, _tProject, _tSignals, _tTransition, _tSteer, _tSpeed;
    // --- end TEMP diagnostic ---

    /// <summary>
    /// Updates a vehicle's steering angle, throttle, brake, and edge/arc progress.
    /// Combines PD heading control, lateral error correction, IDM car-following,
    /// signal/stop-sign/yield compliance, and proximity-based collision avoidance.
    /// Delegates to <see cref="UpdateOnArc"/> when the vehicle is traversing an intersection arc.
    /// </summary>
    public static void Update(VehicleStore store, int index, RoadGraph graph, SpatialGrid grid, StopLineCache stopLines, IntersectionArcCache arcCache, TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns, float dt)
    {
        if (store.State[index] != VehicleState.Driving) return;

        // If vehicle is on an intersection arc, use arc-mode steering
        if (store.CurrentArc[index] >= 0)
        {
            store.DistToRoadSq[index] = 0f; // arc vehicles are always on-road
            long tsArc = System.Diagnostics.Stopwatch.GetTimestamp();           // TEMP
            UpdateOnArc(store, index, graph, grid, stopLines, arcCache, dt);
            _tArc += System.Diagnostics.Stopwatch.GetTimestamp() - tsArc;       // TEMP
            return;
        }

        int edgeIdx = store.CurrentEdge[index];
        if (edgeIdx < 0 || edgeIdx >= graph.Edges.Count) return;

        var edge = graph.Edges[edgeIdx];
        if (edge.FromNode < 0) return; // defunct edge
        float baseSpeedLimit = edge.SpeedLimit > 0f ? edge.SpeedLimit : TargetSpeed;
        float biasedSpeed = baseSpeedLimit * store.SpeedBias[index];
        float targetSpeed = Math.Clamp(biasedSpeed + store.MergeSpeedBias[index], 0f, biasedSpeed * 1.15f);
        float speed = store.Speed[index];
        float progress = store.EdgeProgress[index];
        float edgeLength = edge.Length;
        if (edgeLength < 0.01f) edgeLength = 0.01f;

        long _ts = System.Diagnostics.Stopwatch.GetTimestamp();                 // TEMP
        // Project vehicle position onto the Bezier to compute actual progress
        float vx = store.PosX[index];
        float vy = store.PosY[index];
        int entryPathIdx = store.PathIndex[index]; // snapshot for skip detection
        float rawProgress = FindNearestT(graph, edgeIdx, vx, vy, progress);
        float minProgressT = stopLines.GetStopTAtFromNode(edgeIdx);
        progress = MathF.Max(rawProgress, minProgressT);
        LogDiag(store, index, $"TICK_EDGE proj={progress:F4} raw={rawProgress:F4}");

        // Compute squared distance from vehicle to its lane center for off-road detection
        {
            float laneOff = LaneWidth * (0.5f + store.CurrentLane[index]);
            var laneCenter = OffsetRight(graph, edgeIdx, progress, laneOff);
            float rdx = vx - laneCenter.X;
            float rdy = vy - laneCenter.Y;
            store.DistToRoadSq[index] = rdx * rdx + rdy * rdy;
        }
        _tProject += System.Diagnostics.Stopwatch.GetTimestamp() - _ts;         // TEMP

        _ts = System.Diagnostics.Stopwatch.GetTimestamp();                      // TEMP
        // Evaluate signals, stop signs, and yield signs
        var (signal, yieldSignal) = EvaluateSignals(store, index, edgeIdx, graph, signals, stopSigns, yieldSigns);
        _tSignals += System.Diagnostics.Stopwatch.GetTimestamp() - _ts;         // TEMP

        // Compute stop-line position in t-space
        float stopT = stopLines.GetStopTAtToNode(edgeIdx);
        float halfVehT = (VehicleLength * 0.5f) / edgeLength;
        float stopAtT = stopT - halfVehT;

        // Block crossing on red
        if (CheckRedLightBlocking(store, index, signal, speed, progress, stopAtT, edgeLength, store.BrakingComfort[index]))
            return;

        _ts = System.Diagnostics.Stopwatch.GetTimestamp();                      // TEMP
        // Handle edge transitions (arc entrance, direct jump, or end-of-path stop)
        var transition = HandleEdgeTransition(store, index, graph, stopLines, arcCache, signals, grid, ref edgeIdx, ref edge, ref edgeLength, ref progress, ref rawProgress, ref baseSpeedLimit, ref targetSpeed, ref stopT, ref halfVehT, ref stopAtT, speed, vx, vy);
        _tTransition += System.Diagnostics.Stopwatch.GetTimestamp() - _ts;      // TEMP
        if (transition == TransitionResult.Returned) return;

        store.EdgeProgress[index] = progress;

        _ts = System.Diagnostics.Stopwatch.GetTimestamp();                      // TEMP
        // PD steering
        ComputeSteering(store, index, graph, edgeIdx, rawProgress, progress, edgeLength, minProgressT, vx, vy, dt);
        _tSteer += System.Diagnostics.Stopwatch.GetTimestamp() - _ts;           // TEMP

        _ts = System.Diagnostics.Stopwatch.GetTimestamp();                      // TEMP
        // IDM car-following + throttle/brake
        ApplySpeedControl(store, index, graph, grid, stopLines, edgeIdx, edgeLength, speed, targetSpeed, progress, stopAtT, signal, yieldSignal);
        _tSpeed += System.Diagnostics.Stopwatch.GetTimestamp() - _ts;           // TEMP

        // Detect multi-segment skip within a single tick
        int exitPathIdx = store.PathIndex[index];
        if (exitPathIdx > entryPathIdx + 1)
        {
            LogSkip(
                $"*** V{index} MULTI-SKIP pathIdx {entryPathIdx}->{exitPathIdx} " +
                $"edge={store.CurrentEdge[index]} pos=({vx:F1},{vy:F1}) spd={speed:F2}");
        }
    }

    private enum TransitionResult { Continue, Returned }

    /// <summary>
    /// Evaluates traffic signals, stop signs, and yield signs for an edge.
    /// Returns the merged signal state and the yield signal state.
    /// </summary>
    private static (SignalState signal, SignalState yieldSignal) EvaluateSignals(
        VehicleStore store, int index, int edgeIdx, RoadGraph graph,
        TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns)
    {
        var signal = signals.GetSignal(edgeIdx);
        var stopSignal = stopSigns.GetSignal(edgeIdx, graph, index);
        if (stopSignal != SignalState.Green && signal == SignalState.Green)
            signal = stopSignal;

        int yieldOutEdge = -1;
        {
            var yPath = store.Path[index];
            int yPathIdx = store.PathIndex[index];
            if (yPath != null && yPathIdx + 1 < yPath.Count)
                yieldOutEdge = yPath[yPathIdx + 1];
        }
        var yieldSignal = yieldOutEdge >= 0
            ? yieldSigns.GetSignal(edgeIdx, yieldOutEdge, graph)
            : yieldSigns.GetSignal(edgeIdx, graph);

        return (signal, yieldSignal);
    }

    /// <summary>
    /// Checks if the vehicle is blocked by a red/yellow signal at the stop line.
    /// If blocked, forces full stop and returns true.
    /// </summary>
    private static bool CheckRedLightBlocking(VehicleStore store, int index,
        SignalState signal, float speed, float progress, float stopAtT, float edgeLength, float comfortDecel)
    {
        bool blockedBySignal = signal == SignalState.Red ||
            (signal == SignalState.Yellow && speed * speed / (2f * comfortDecel) * 0.8f < (stopAtT - progress) * edgeLength);
        if (progress >= stopAtT && blockedBySignal)
        {
            store.EdgeProgress[index] = stopAtT - 0.001f;
            store.Speed[index] = 0f;
            store.Throttle[index] = 0f;
            store.Brake[index] = 1.0f;
            return true;
        }
        return false;
    }

    /// <summary>
    /// Handles edge-to-edge transitions: arc entrance, direct edge jump, or end-of-path stop.
    /// Updates edge/progress/speed state via ref parameters when the vehicle transfers to a new edge.
    /// Returns <see cref="TransitionResult.Returned"/> if the caller should return immediately.
    /// </summary>
    private static TransitionResult HandleEdgeTransition(VehicleStore store, int index, RoadGraph graph,
        StopLineCache stopLines, IntersectionArcCache arcCache, TrafficSignalSystem signals, SpatialGrid grid,
        ref int edgeIdx, ref RoadEdge edge, ref float edgeLength, ref float progress,
        ref float rawProgress, ref float baseSpeedLimit, ref float targetSpeed,
        ref float stopT, ref float halfVehT, ref float stopAtT,
        float speed, float vx, float vy)
    {
        var path = store.Path[index];
        int pathIdx = store.PathIndex[index];
        bool hasNextEdge = path != null && pathIdx + 1 < path.Count;
        float transitionT = hasNextEdge ? stopAtT : 1f;

        if (progress >= transitionT && hasNextEdge && path != null)
        {
            int prevEdge = edgeIdx;
            float prevProgress = progress;
            int prevPathIdx = pathIdx;

            int nextEdge = path[pathIdx + 1];
            if (nextEdge < 0 || nextEdge >= graph.Edges.Count || graph.Edges[nextEdge].FromNode < 0)
            {
                store.Path[index] = null;
                store.PathIndex[index] = 0;
                store.Speed[index] = 0f;
                store.Throttle[index] = 0f;
                store.Brake[index] = 0f;
                store.EdgeProgress[index] = 1f;
                return TransitionResult.Returned;
            }
            var nextEdgeData = graph.Edges[nextEdge];

            // Resolve intersection arc (with reroute fallback)
            byte inLane = store.CurrentLane[index];
            int arcIdx = ResolveArc(store, index, graph, arcCache,
                edgeIdx, ref nextEdge, ref nextEdgeData, inLane,
                ref path, ref pathIdx);

            if (arcIdx >= 0)
            {
                var arc = arcCache.GetArc(arcIdx);
                float frameDist = speed * (1f / 30f);
                if (arc.Length > frameDist * 2f)
                {
                    // Check if any vehicle is on a conflicting arc or too close on the same arc
                    var conflicts = arcCache.GetConflictingArcs(arcIdx);
                    bool isTrafficLightNode = signals.IsTrafficLight(arc.NodeIndex);
                    var mySignal = isTrafficLightNode ? signals.GetSignal(arc.IncomingEdge) : SignalState.Green;

                    // Look up only the vehicles occupying conflicting arcs (via the occupancy index)
                    // instead of scanning every vehicle. Equivalent to the former O(n) scan: the set
                    // { j : CurrentArc[j] ∈ conflicts } is exactly the union of those arcs' occupants,
                    // and the index is kept live mid-pass via SetArc so entries earlier this pass show.
                    bool conflictBlocked = false;
                    int blockerJ = -1, blockerArc = -1;
                    foreach (int conflictArc in conflicts)
                    {
                        int occCount = _arcOccupancy.OccupantCount(conflictArc);
                        for (int k = 0; k < occCount; k++)
                        {
                            int j = _arcOccupancy.OccupantAt(conflictArc, k);
                            if (j == index || store.State[j] != VehicleState.Driving) continue;

                            // At traffic-light nodes, same-phase vehicles should not block each other —
                            // the signal already manages inter-phase right-of-way. Only block if arcs come
                            // from different phases (e.g. a stale vehicle still clearing from a prior red).
                            if (isTrafficLightNode && mySignal == SignalState.Green)
                            {
                                var otherArcData = arcCache.GetArc(conflictArc);
                                if (signals.GetSignal(otherArcData.IncomingEdge) == SignalState.Green)
                                    continue; // same phase, both green — proceed
                            }

                            conflictBlocked = true;
                            blockerJ = j;
                            blockerArc = conflictArc;
                            break;
                        }
                        if (conflictBlocked) break;
                    }

                    if (conflictBlocked)
                    {
                        store.EdgeProgress[index] = stopAtT;
                        store.Throttle[index] = 0f;
                        store.Brake[index] = 1.0f;
                        store.Speed[index] = 0f;
                        LogArcConflict(store, index,
                            $"ARC_BLOCKED arcIdx={arcIdx} node={arc.NodeIndex} " +
                            $"conflictArc={blockerArc} blockedByV={blockerJ}",
                            blockerIndex: blockerJ);
                        return TransitionResult.Returned;
                    }

                    // Check if a vehicle on the outgoing edge is mid-lane-change into our lane.
                    // Only block for actual lane-change conflicts: a vehicle whose current lane
                    // is NOT our exit lane but is actively changing INTO it. Vehicles already
                    // traveling in the exit lane are handled by IDM look-ahead on the arc.
                    {
                        int outEdge = arc.OutgoingEdge;
                        byte outLane = arc.OutgoingLane;
                        var outEdgeData = graph.Edges[outEdge];
                        if (outEdge >= 0 && outEdgeData.FromNode >= 0)
                        {
                            float outLength = MathF.Max(outEdgeData.Length, 0.01f);
                            float outStartT = stopLines.GetStopTAtFromNode(outEdge);
                            // Candidates lie within VehicleLength*2 of the exit point (arc.P3 = the
                            // out-lane start), so query the grid around P3 instead of scanning every
                            // vehicle. The exact edgeDist cutoff is re-applied below, so this is an
                            // exact superset of the original full scan (no false negatives).
                            var exitPoint = arc.P3;
                            var scan2 = _scan2Buffer ??= new List<int>();
                            scan2.Clear();
                            grid.QueryFiltered(exitPoint.X, exitPoint.Y,
                                VehicleLength * 2f + SpatialGrid.CellSize,
                                store.PosX, store.PosY, scan2);
                            for (int bi = 0; bi < scan2.Count; bi++)
                            {
                                int j = scan2[bi];
                                if (j == index || store.State[j] != VehicleState.Driving) continue;
                                if (store.CurrentEdge[j] != outEdge || store.CurrentArc[j] >= 0) continue;
                                // Skip vehicles already in our lane — they're handled by arc IDM look-ahead
                                if (store.CurrentLane[j] == outLane) continue;
                                // Only block if this vehicle is actively lane-changing into our exit lane
                                if (store.TargetLane[j] != outLane) continue;

                                float edgeDist = (store.EdgeProgress[j] - outStartT) * outLength;
                                if (edgeDist < VehicleLength * 2f)
                                {
                                    store.EdgeProgress[index] = stopAtT;
                                    store.Throttle[index] = 0f;
                                    store.Brake[index] = 1.0f;
                                    store.Speed[index] = 0f;
                                    LogArcConflict(store, index,
                                        $"ARC_BLOCKED_MERGE arcIdx={arcIdx} outEdge={outEdge} " +
                                        $"outLane={outLane} blockedByV={j} " +
                                        $"otherLane={store.CurrentLane[j]}->{store.TargetLane[j]}",
                                        blockerIndex: j);
                                    return TransitionResult.Returned;
                                }
                            }
                        }
                    }

                    // Enter the intersection arc (SetArc keeps the occupancy index live so later
                    // vehicles this pass see this entry).
                    SetArc(store, index, arcIdx);
                    store.ArcProgress[index] = 0f;
                    store.EdgeProgress[index] = stopAtT;

                    float arcLookahead = MathF.Min((LookaheadBase + speed * LookaheadPerSpeed) / MathF.Max(arc.Length, 0.01f), 1f);
                    var arcTarget = arcCache.EvaluateArc(arcIdx, arcLookahead);
                    float arcDesired = MathF.Atan2(arcTarget.Y - vy, arcTarget.X - vx);
                    store.PrevHeadingError[index] = NormalizeAngle(arcDesired - store.Heading[index]);
                    LogDiag(store, index, $"ARC_ENTER arcIdx={arcIdx} arcLen={arc.Length:F2}");
                    // Check for other vehicles already on arcs at the same node
                    if (DebugLoggingEnabled)
                    {
                        for (int j = 0; j < store.Count; j++)
                        {
                            if (j == index || store.State[j] != VehicleState.Driving) continue;
                            int otherArc = store.CurrentArc[j];
                            if (otherArc < 0) continue;
                            var otherArcData = arcCache.GetArc(otherArc);
                            if (otherArcData.NodeIndex == arc.NodeIndex)
                            {
                                LogArcConflict(store, index,
                                    $"ARC_ENTER_CONFLICT node={arc.NodeIndex} myArc={arcIdx} " +
                                    $"inEdge={arc.IncomingEdge} outEdge={arc.OutgoingEdge} " +
                                    $"otherV={j} otherArc={otherArc} " +
                                    $"otherInEdge={otherArcData.IncomingEdge} otherOutEdge={otherArcData.OutgoingEdge} " +
                                    $"otherSpd={store.Speed[j]:F2} otherBrake={store.Brake[j]:F2}");
                            }
                        }
                    }
                    LogSkip(
                        $"V{index} ARC_ENTER pathIdx={prevPathIdx} " +
                        $"edge {prevEdge}->arc{arcIdx} nextEdge={nextEdge} " +
                        $"prog={prevProgress:F4} transT={transitionT:F4} " +
                        $"pos=({vx:F1},{vy:F1}) spd={speed:F2} lane={inLane} " +
                        $"arcLen={arc.Length:F2} pathLen={path?.Count ?? 0}");
                    return TransitionResult.Returned;
                }
                LogDiag(store, index, $"ARC_SKIP arcIdx={arcIdx} arcLen={arc.Length:F2} (too short)");
            }

            // Direct edge-to-edge transition (no arc or arc too short)
            TransferToNextEdge(store, index, graph, stopLines,
                ref edgeIdx, ref edge, ref edgeLength, ref progress, ref rawProgress,
                ref baseSpeedLimit, ref targetSpeed, ref stopT, ref halfVehT, ref stopAtT,
                nextEdge, nextEdgeData, speed, vx, vy,
                prevEdge, prevProgress, prevPathIdx, ref pathIdx, path);
        }
        else if (progress >= transitionT)
        {
            // No more path — stop
            var p = store.Path[index];
            Console.Error.WriteLine($"[Steering] Vehicle {index} stopped at end: edge={edgeIdx} progress={progress:F3} transitionT={transitionT:F3} pathIdx={store.PathIndex[index]}/{p?.Count ?? 0} arc={store.CurrentArc[index]}");
            progress = 1f;
            store.Speed[index] = 0f;
            store.Throttle[index] = 0f;
            store.Brake[index] = 0f;
            store.EdgeProgress[index] = progress;
            return TransitionResult.Returned;
        }

        return TransitionResult.Continue;
    }

    /// <summary>
    /// Resolves the intersection arc for a lane transition, with reroute fallbacks.
    /// May update path, pathIdx, nextEdge, and nextEdgeData if a reroute occurs.
    /// </summary>
    private static int ResolveArc(VehicleStore store, int index, RoadGraph graph,
        IntersectionArcCache arcCache, int edgeIdx,
        ref int nextEdge, ref RoadEdge nextEdgeData, byte inLane,
        ref List<int>? path, ref int pathIdx)
    {
        byte outLane = (byte)Math.Min(inLane, nextEdgeData.LaneCount - 1);
        int arcIdx = arcCache.GetArcIndex(edgeIdx, nextEdge, inLane, outLane);

        if (arcIdx < 0 && inLane != outLane)
            arcIdx = arcCache.GetArcIndex(edgeIdx, nextEdge, inLane, inLane);

        if (arcIdx < 0)
        {
            var reachable = arcCache.GetReachableFromLane(edgeIdx, inLane);
            if (reachable != null && reachable.Count > 0)
            {
                int destNode = store.DestinationNode[index];
                if (destNode >= 0)
                {
                    var (bestArc, bestOutEdge, newPath) = Pathfinder.FindBestReroute(
                        graph, reachable, destNode);
                    if (bestArc >= 0 && newPath != null)
                    {
                        store.Path[index] = newPath;
                        store.PathIndex[index] = 0;
                        path = newPath;
                        pathIdx = 0;
                        arcIdx = bestArc;
                        nextEdge = bestOutEdge;
                        nextEdgeData = graph.Edges[nextEdge];
                        LogDiag(store, index, $"REROUTE from lane {inLane} -> edge {bestOutEdge}");
                    }
                }

                if (arcIdx < 0)
                {
                    arcIdx = reachable[0].arcIdx;
                    var fallbackArc = arcCache.GetArc(arcIdx);
                    nextEdge = fallbackArc.OutgoingEdge;
                    nextEdgeData = graph.Edges[nextEdge];
                    store.Path[index] = null;
                    store.PathIndex[index] = 0;
                    path = null;
                    pathIdx = 0;
                    LogDiag(store, index, $"REROUTE_FALLBACK -> edge {nextEdge}");
                }
            }
        }

        return arcIdx;
    }

    /// <summary>
    /// Performs a direct edge-to-edge transfer (no arc), updating all edge-tracking state.
    /// </summary>
    private static void TransferToNextEdge(VehicleStore store, int index, RoadGraph graph,
        StopLineCache stopLines,
        ref int edgeIdx, ref RoadEdge edge, ref float edgeLength, ref float progress,
        ref float rawProgress, ref float baseSpeedLimit, ref float targetSpeed,
        ref float stopT, ref float halfVehT, ref float stopAtT,
        int nextEdge, RoadEdge nextEdgeData, float speed, float vx, float vy,
        int prevEdge, float prevProgress, int prevPathIdx, ref int pathIdx, List<int>? path)
    {
        pathIdx++;
        store.PathIndex[index] = pathIdx;

        float nextLength = nextEdgeData.Length;
        if (nextLength < 0.01f) nextLength = 0.01f;

        store.CurrentEdge[index] = nextEdge;
        float nextStartT = stopLines.GetStopTAtFromNode(nextEdge);
        store.EdgeProgress[index] = nextStartT;

        // U-turn at a dead-end: a real U-turn can't fit within one lane width, so pivot the
        // vehicle onto the reverse lane (snap to its start, facing back) rather than trying
        // to steer a 180° turn in place — which would send it off the road. Done before the
        // heading seed below so normal steering continues smoothly from the pivoted pose.
        if (nextEdgeData.ToNode == edge.FromNode)
        {
            float pivotOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index);
            var pivotPos = OffsetRight(graph, nextEdge, nextStartT, pivotOffset);
            vx = pivotPos.X;
            vy = pivotPos.Y;
            store.PosX[index] = vx;
            store.PosY[index] = vy;
            var pivotTan = graph.EvaluateBezierTangent(nextEdge, nextStartT);
            store.Heading[index] = MathF.Atan2(pivotTan.Y, pivotTan.X);
            store.Speed[index] = MathF.Min(store.Speed[index], 3f);
        }

        // Seed PrevHeadingError using vehicle's actual position (no teleport)
        float newLookaheadT = MathF.Min(nextStartT + (LookaheadBase + speed * LookaheadPerSpeed) / nextLength, 1f);
        float newLaneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index);
        var newTarget = OffsetRight(graph, nextEdge, newLookaheadT, newLaneOffset);
        float newDesired = MathF.Atan2(newTarget.Y - vy, newTarget.X - vx);
        store.PrevHeadingError[index] = NormalizeAngle(newDesired - store.Heading[index]);
        LogDiag(store, index, $"EDGE_JUMP -> edge {nextEdge}");

        // Reset lane state for the new edge
        byte newMaxLane = (byte)(nextEdgeData.LaneCount - 1);
        if (store.CurrentLane[index] > newMaxLane)
            store.CurrentLane[index] = newMaxLane;
        store.TargetLane[index] = store.CurrentLane[index];
        store.LaneChangeProgress[index] = 0f;

        // Continue steering on the new edge this tick (no skipped frame)
        edgeIdx = nextEdge;
        edge = nextEdgeData;
        edgeLength = nextLength;
        progress = store.EdgeProgress[index];
        baseSpeedLimit = edge.SpeedLimit > 0f ? edge.SpeedLimit : TargetSpeed;
        float biasedSpeedN = baseSpeedLimit * store.SpeedBias[index];
        targetSpeed = Math.Clamp(biasedSpeedN + store.MergeSpeedBias[index], 0f, biasedSpeedN * 1.15f);
        stopT = stopLines.GetStopTAtToNode(edgeIdx);
        halfVehT = (VehicleLength * 0.5f) / edgeLength;
        stopAtT = stopT - halfVehT;

        rawProgress = FindNearestT(graph, edgeIdx, vx, vy, progress);

        LogSkip(
            $"V{index} EDGE_JUMP pathIdx {prevPathIdx}->{pathIdx} " +
            $"edge {prevEdge}->{nextEdge} prog={prevProgress:F4} " +
            $"pos=({vx:F1},{vy:F1}) spd={speed:F2} lane={store.CurrentLane[index]} " +
            $"newRaw={rawProgress:F4} newProg={progress:F4} " +
            $"arcLookup=noArcOrSkipped pathLen={path?.Count ?? 0}");
    }

    /// <summary>
    /// Computes PD steering: heading error, lateral error, and steering angle.
    /// </summary>
    private static void ComputeSteering(VehicleStore store, int index, RoadGraph graph,
        int edgeIdx, float rawProgress, float progress, float edgeLength,
        float minProgressT, float vx, float vy, float dt)
    {
        Vector2 targetPos = ComputeEdgeLookahead(store, index, graph, edgeIdx, rawProgress, edgeLength, checkCollinearity: true);
        float laneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index);
        if (store.DiagVehicle == index)
        {
            var edgePos = graph.EvaluateBezier(edgeIdx, progress);
            _diagWriter ??= new StreamWriter("diag.log", append: true) { AutoFlush = true };
            _diagWriter.WriteLine(
                $"  LOOK edgePos=({edgePos.X:F2},{edgePos.Y:F2}) dot=({targetPos.X:F2},{targetPos.Y:F2}) " +
                $"prog={progress:F4}");
        }

        float heading = store.Heading[index];
        float dx = targetPos.X - vx;
        float dy = targetPos.Y - vy;

        float desiredHeading = MathF.Atan2(dy, dx);
        float headingError = NormalizeAngle(desiredHeading - heading);

        // Signed lateral error from lane center (suppressed near intersection entry)
        float distFromEntry = (progress - minProgressT) * edgeLength;
        float lateralError = 0f;
        if (distFromEntry > VehicleLength * 2f)
        {
            var laneCenter = OffsetRight(graph, edgeIdx, progress, laneOffset);
            var curveTangent = graph.EvaluateBezierTangent(edgeIdx, progress);
            float tangentLen = curveTangent.Length();
            if (tangentLen > 0.001f)
            {
                float nx = -curveTangent.Y / tangentLen;
                float ny = curveTangent.X / tangentLen;
                lateralError = (vx - laneCenter.X) * nx + (vy - laneCenter.Y) * ny;
            }
        }

        // PD control + lateral correction
        float prevError = store.PrevHeadingError[index];
        float errorDerivative = (headingError - prevError) / dt;
        store.PrevHeadingError[index] = headingError;

        float sharpness = store.SteeringSharpness[index];
        float steer = (Kp * sharpness) * headingError + (Kd * sharpness) * errorDerivative - Klat * lateralError;
        steer = MathF.Max(-MaxSteer, MathF.Min(MaxSteer, steer));
        if (store.DiagVehicle == index)
        {
            _diagWriter ??= new StreamWriter("diag.log", append: true) { AutoFlush = true };
            _diagWriter.WriteLine(
                $"  STEER hdgErr={headingError * 180f / MathF.PI:F1} latErr={lateralError:F3} " +
                $"errDeriv={errorDerivative:F3} steer={steer * 180f / MathF.PI:F1} sharp={sharpness:F2}");
        }

        store.SteeringAngle[index] = steer;
    }

    /// <summary>
    /// Applies IDM car-following with signal/yield compliance and proximity-based collision avoidance.
    /// Sets throttle and brake on the vehicle.
    /// </summary>
    private static void ApplySpeedControl(VehicleStore store, int index, RoadGraph graph,
        SpatialGrid grid, StopLineCache stopLines, int edgeIdx, float edgeLength,
        float speed, float targetSpeed, float progress, float stopAtT,
        SignalState signal, SignalState yieldSignal)
    {
        float comfortDecel = store.BrakingComfort[index];
        float timeHeadway = MapAggressivenessToTimeHeadway(store.Aggressiveness[index]);

        var (aheadDist, leaderSpeed) = FindNearbyThreats(store, index, grid, graph);

        float gap = aheadDist - VehicleLength;
        float deltaV = speed - leaderSpeed;

        // Signal compliance: treat red/yellow as virtual stopped leader at stop line
        if (signal == SignalState.Red || signal == SignalState.Yellow)
        {
            float distToStop = (stopAtT - progress) * edgeLength;
            if (distToStop > 0f && distToStop < gap)
            {
                bool shouldStop = true;
                if (signal == SignalState.Yellow)
                {
                    float stoppingDist = speed * speed / (2f * comfortDecel);
                    shouldStop = distToStop > stoppingDist * 0.8f;
                }
                if (shouldStop)
                {
                    gap = distToStop;
                    deltaV = speed;
                }
            }
        }

        // Yield sign: slow down for cross-traffic, stop only if very close
        if (yieldSignal != SignalState.Green)
        {
            float distToStop = (stopAtT - progress) * edgeLength;
            if (distToStop > 0f && distToStop < gap)
            {
                if (yieldSignal == SignalState.Red)
                {
                    gap = distToStop;
                    deltaV = speed;
                }
                else
                {
                    float creepSpeed = 2.0f;
                    gap = distToStop;
                    deltaV = speed - creepSpeed;
                }
            }
        }

        if (ApplyHardOverlapBrake(store, index, gap)) return;

        float idmAccel = ComputeIdmAcceleration(speed, targetSpeed, gap, deltaV, timeHeadway, comfortDecel);
        var (idmThrottle, idmBrake) = MapIdmToControls(idmAccel);

        store.Brake[index] = idmBrake;
        store.Throttle[index] = MathF.Max(0f, idmThrottle);
    }

    /// <summary>
    /// Updates a vehicle that is currently traversing an intersection arc.
    /// Uses PD steering targeting the arc Bezier and IDM car-following.
    /// Transitions to the next edge when the arc is complete.
    /// </summary>
    private static void UpdateOnArc(VehicleStore store, int index, RoadGraph graph, SpatialGrid grid, StopLineCache stopLines, IntersectionArcCache arcCache, float dt)
    {
        int arcIdx = store.CurrentArc[index];
        var arc = arcCache.GetArc(arcIdx);
        float arcLength = MathF.Max(arc.Length, 0.01f);
        float speed = store.Speed[index];
        float vx = store.PosX[index];
        float vy = store.PosY[index];
        float heading = store.Heading[index];

        // Project vehicle position onto the arc Bezier to compute actual progress
        // (same approach as FindNearestT for edges — keeps progress in sync with physics)
        float arcProgress = store.ArcProgress[index];
        arcProgress = FindNearestTOnArc(arcCache, arcIdx, vx, vy, arcProgress);

        // For short arcs, the vehicle may physically overshoot past P3.
        // Detect this by checking if the vehicle is closer to P3 than to its
        // projected arc position, and moving away from the arc.
        var arcEndPos = arcCache.EvaluateArc(arcIdx, 1f);
        float distToEnd = MathF.Sqrt((vx - arcEndPos.X) * (vx - arcEndPos.X) + (vy - arcEndPos.Y) * (vy - arcEndPos.Y));
        var projPos = arcCache.EvaluateArc(arcIdx, arcProgress);
        float distToProj = MathF.Sqrt((vx - projPos.X) * (vx - projPos.X) + (vy - projPos.Y) * (vy - projPos.Y));
        float remainingDist = (1f - arcProgress) * arcLength;
        if (distToEnd < distToProj || distToEnd < VehicleLength * 0.5f)
            remainingDist = 0f; // force arc completion

        LogDiag(store, index, $"TICK_ARC proj={arcProgress:F4} distEnd={distToEnd:F2} distProj={distToProj:F2}");

        // Check for arc completion (distance-based)
        if (remainingDist < VehicleLength * 0.5f)
        {
            // Exit arc, enter next edge
            var path = store.Path[index];
            int pathIdx = store.PathIndex[index];

            // Use the arc's outgoing edge directly — the path may have been
            // invalidated by a swap-remove while the vehicle was on the arc.
            int nextEdge = arc.OutgoingEdge;
            if (nextEdge < 0 || nextEdge >= graph.Edges.Count || graph.Edges[nextEdge].FromNode < 0)
            {
                SetArc(store, index, -1);
                store.ArcProgress[index] = 0f;
                store.Path[index] = null;
                store.PathIndex[index] = 0;
                store.Speed[index] = 0f;
                store.Throttle[index] = 0f;
                store.Brake[index] = 0f;
                store.EdgeProgress[index] = 1f;
                return;
            }
            var nextEdgeData = graph.Edges[nextEdge];

            // Reconcile PathIndex with the path: find the arc's outgoing edge
            // in the path so subsequent transitions work correctly.
            // Search the entire path since a swap-remove may have replaced it.
            if (path != null)
            {
                bool found = false;
                for (int pi = 0; pi < path.Count; pi++)
                {
                    if (path[pi] == nextEdge)
                    {
                        pathIdx = pi;
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    // Path doesn't contain this edge — invalidate for reroute
                    store.Path[index] = null;
                    pathIdx = 0;
                }
            }
            store.PathIndex[index] = pathIdx;

            {
                float nextLength = nextEdgeData.Length;
                if (nextLength < 0.01f) nextLength = 0.01f;

                SetArc(store, index, -1);
                store.ArcProgress[index] = 0f;
                store.CurrentEdge[index] = nextEdge;
                float nextStartT = stopLines.GetStopTAtFromNode(nextEdge);
                store.EdgeProgress[index] = nextStartT;

                // Seed PrevHeadingError using vehicle's actual position (no teleport)
                float newLookaheadT = MathF.Min(nextStartT + (LookaheadBase + speed * LookaheadPerSpeed) / nextLength, 1f);
                float newLaneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index);
                var newTarget = OffsetRight(graph, nextEdge, newLookaheadT, newLaneOffset);
                float newDesired = MathF.Atan2(newTarget.Y - vy, newTarget.X - vx);
                store.PrevHeadingError[index] = NormalizeAngle(newDesired - heading);
                // Check for other vehicles already near the start of the outgoing edge
                if (DebugLoggingEnabled)
                {
                    for (int j = 0; j < store.Count; j++)
                    {
                        if (j == index || store.State[j] != VehicleState.Driving) continue;
                        if (store.CurrentEdge[j] != nextEdge || store.CurrentArc[j] >= 0) continue;
                        float otherProg = store.EdgeProgress[j];
                        float progDist = MathF.Abs(otherProg - nextStartT) * nextLength;
                        if (progDist < VehicleLength * 2f)
                        {
                            LogArcConflict(store, index,
                                $"ARC_EXIT_OVERLAP edge={nextEdge} myStartT={nextStartT:F4} " +
                                $"otherV={j} otherProg={otherProg:F4} dist={progDist:F2}m " +
                                $"mySpeed={speed:F2} otherSpeed={store.Speed[j]:F2} " +
                                $"myArc={arcIdx} myInEdge={arc.IncomingEdge} " +
                                $"otherLane={store.CurrentLane[j]} myOutLane={arc.OutgoingLane}");
                        }
                    }
                }
                LogDiag(store, index, $"ARC_EXIT -> edge {nextEdge}");
                LogSkip(
                    $"V{index} ARC_EXIT pathIdx->{pathIdx} " +
                    $"arc{arcIdx}->edge {nextEdge} " +
                    $"pos=({vx:F1},{vy:F1}) spd={speed:F2} " +
                    $"arcProg={arcProgress:F4} newEdgeProg={nextStartT:F4} " +
                    $"lane={arc.OutgoingLane} pathLen={path?.Count ?? 0}");

                // Set lane to the arc's outgoing lane
                byte newMaxLane = (byte)(nextEdgeData.LaneCount - 1);
                store.CurrentLane[index] = (byte)Math.Min(arc.OutgoingLane, newMaxLane);
                store.TargetLane[index] = store.CurrentLane[index];
                store.LaneChangeProgress[index] = 0f;
            }

            return;
        }

        store.ArcProgress[index] = arcProgress;

        // PD steering on the arc Bezier
        float lookahead = LookaheadBase + speed * LookaheadPerSpeed;
        float lookaheadT = arcProgress + lookahead / arcLength;
        lookaheadT = MathF.Min(lookaheadT, 1f);

        var targetPos = arcCache.EvaluateArc(arcIdx, lookaheadT);

        float dxTarget = targetPos.X - vx;
        float dyTarget = targetPos.Y - vy;

        float desiredHeading = MathF.Atan2(dyTarget, dxTarget);
        float headingError = NormalizeAngle(desiredHeading - heading);

        // PD steering (no lateral error correction on arcs — the arc IS the path)
        float prevError = store.PrevHeadingError[index];
        float errorDerivative = (headingError - prevError) / dt;
        store.PrevHeadingError[index] = headingError;

        float arcSharpness = store.SteeringSharpness[index];
        float steer = (Kp * arcSharpness) * headingError + (Kd * arcSharpness) * errorDerivative;
        steer = MathF.Max(-MaxSteer, MathF.Min(MaxSteer, steer));
        store.SteeringAngle[index] = steer;

        // IDM car-following (world-space, works regardless of edge/arc)
        float arcBaseSpeed = arc.SpeedLimit > 0f ? arc.SpeedLimit : TargetSpeed;
        float targetSpeed = arcBaseSpeed * store.SpeedBias[index];
        var (aheadDist, leaderSpeed) = FindVehicleAhead(store, index, grid, graph);
        int leaderIndex = -1; // track leader for conflict logging

        // Same-arc look-ahead: if another vehicle is ahead on this arc, use path distance
        for (int j = 0; j < store.Count; j++)
        {
            if (j == index || store.State[j] != VehicleState.Driving) continue;
            if (store.CurrentArc[j] != arcIdx) continue;
            if (store.ArcProgress[j] <= arcProgress) continue;

            float pathDist = (store.ArcProgress[j] - arcProgress) * arcLength;
            if (pathDist < aheadDist)
            {
                aheadDist = pathDist;
                leaderSpeed = store.Speed[j];
                leaderIndex = j;
            }
        }

        // Look ahead through the arc boundary onto the outgoing edge:
        // if a vehicle is near the start of the outgoing edge in the same lane,
        // treat (remaining arc distance + their edge distance) as the leader gap.
        {
            float remainingArc = (1f - arcProgress) * arcLength;
            int outEdge = arc.OutgoingEdge;
            var outEdgeData = graph.Edges[outEdge];
            if (outEdge >= 0 && outEdgeData.FromNode >= 0)
            {
                float outLength = MathF.Max(outEdgeData.Length, 0.01f);
                float outStartT = stopLines.GetStopTAtFromNode(outEdge);
                byte myOutLane = arc.OutgoingLane;

                for (int j = 0; j < store.Count; j++)
                {
                    if (j == index || store.State[j] != VehicleState.Driving) continue;
                    if (store.CurrentEdge[j] != outEdge || store.CurrentArc[j] >= 0) continue;
                    if (store.CurrentLane[j] != myOutLane && store.TargetLane[j] != myOutLane) continue;

                    float edgeDist = (store.EdgeProgress[j] - outStartT) * outLength;
                    float totalDist = remainingArc + edgeDist;
                    if (totalDist < aheadDist)
                    {
                        aheadDist = totalDist;
                        leaderSpeed = store.Speed[j];
                        leaderIndex = j;
                    }
                }
            }
        }

        float gap = aheadDist - VehicleLength;
        float deltaV = speed - leaderSpeed;

        if (ApplyHardOverlapBrake(store, index, gap))
        {
            LogArcConflict(store, index,
                $"HARD_OVERLAP_ARC arcIdx={arcIdx} node={arc.NodeIndex} gap={gap:F3} " +
                $"aheadDist={aheadDist:F3} leaderSpd={leaderSpeed:F3}" +
                (leaderIndex >= 0 ? $" blockedByV={leaderIndex}" : ""),
                blockerIndex: leaderIndex);
            return;
        }

        float arcTimeHeadway = MapAggressivenessToTimeHeadway(store.Aggressiveness[index]);
        float arcComfortDecel = store.BrakingComfort[index];
        float idmAccel = ComputeIdmAcceleration(speed, targetSpeed, gap, deltaV, arcTimeHeadway, arcComfortDecel);
        var (idmThrottle, idmBrake) = MapIdmToControls(idmAccel);

        store.Brake[index] = idmBrake;
        store.Throttle[index] = MathF.Max(0f, idmThrottle);
    }

    /// <summary>
    /// Computes IDM (Intelligent Driver Model) acceleration.
    /// </summary>
    /// <param name="speed">Current vehicle speed in m/s.</param>
    /// <summary>
    /// Emergency brake when gap is dangerously small. Returns true if braking was applied.
    /// </summary>
    private static bool ApplyHardOverlapBrake(VehicleStore store, int index, float gap)
    {
        if (gap >= 0.5f || gap >= float.MaxValue) return false;
        store.Speed[index] = MathF.Max(0f, store.Speed[index] * 0.5f);
        store.Throttle[index] = 0f;
        store.Brake[index] = 1.0f;
        return true;
    }

    /// <summary>
    /// Maps Aggressiveness (0–1) to IDM time headway (seconds).
    /// High aggression = close following (0.8s), low aggression = large gap (2.0s).
    /// </summary>
    private static float MapAggressivenessToTimeHeadway(float aggressiveness)
        => 2.0f - aggressiveness * 1.2f;

    /// <param name="desiredSpeed">Target speed in m/s.</param>
    /// <param name="gap">Bumper-to-bumper gap to leader in meters.</param>
    /// <param name="deltaV">Speed difference (positive = closing on leader).</param>
    /// <param name="timeHeadway">Desired time headway in seconds (IDM T parameter).</param>
    /// <param name="comfortDecel">Comfortable deceleration in m/s^2 (IDM b parameter).</param>
    /// <returns>Acceleration in m/s^2 (positive = accelerate, negative = brake).</returns>
    private static float ComputeIdmAcceleration(float speed, float desiredSpeed, float gap, float deltaV,
        float timeHeadway, float comfortDecel)
    {
        // Free-road term: (v/v0)^4
        float vRatio = desiredSpeed > 0.01f ? speed / desiredSpeed : 0f;
        float vRatio2 = vRatio * vRatio;
        float freeRoadTerm = vRatio2 * vRatio2;

        // Desired dynamic gap: s* = s0 + max(0, v*T + v*Δv / (2*sqrt(a*b)))
        float interaction = speed * deltaV / (2f * MathF.Sqrt(IdmMaxAccel * comfortDecel));
        float sStar = IdmMinGap + MathF.Max(0f, speed * timeHeadway + interaction);

        // Gap term: (s*/gap)^2
        float gapTerm = gap > 0.01f ? (sStar / gap) * (sStar / gap) : 100f;

        return Math.Clamp(IdmMaxAccel * (1f - freeRoadTerm - gapTerm), -MaxBrakeDecel, IdmMaxAccel);
    }

    /// <summary>
    /// Computes the edge-mode lookahead target position for a vehicle, including cross-edge
    /// extension for smooth transitions. Shared between steering controller and vehicle renderer.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Vehicle index.</param>
    /// <param name="graph">Road graph for Bezier evaluation.</param>
    /// <param name="edgeIdx">Current edge index.</param>
    /// <param name="rawProgress">Raw projected progress on the edge.</param>
    /// <param name="edgeLength">Length of the current edge.</param>
    /// <param name="checkCollinearity">If true, only extend across collinear edges (for steering). If false, always extend (for rendering).</param>
    /// <returns>The world-space lookahead target position.</returns>
    internal static Vector2 ComputeEdgeLookahead(VehicleStore store, int index, RoadGraph graph,
        int edgeIdx, float rawProgress, float edgeLength, bool checkCollinearity)
    {
        float speed = store.Speed[index];
        float lookahead = LookaheadBase + speed * LookaheadPerSpeed;
        float effectiveLookahead = lookahead;

        if (rawProgress <= 0.001f)
        {
            var edgeStart = graph.EvaluateBezier(edgeIdx, 0f);
            var tangent = graph.EvaluateBezierTangent(edgeIdx, 0f);
            float tangentLen = tangent.Length();
            if (tangentLen > 0.001f)
            {
                float toVehX = store.PosX[index] - edgeStart.X;
                float toVehY = store.PosY[index] - edgeStart.Y;
                float alongEdge = (toVehX * tangent.X + toVehY * tangent.Y) / tangentLen;
                effectiveLookahead = MathF.Max(0f, lookahead + MathF.Min(alongEdge, 0f));
            }
        }

        float lookaheadT = rawProgress + effectiveLookahead / edgeLength;
        float laneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index);

        if (lookaheadT > 1f)
        {
            var path = store.Path[index];
            int pathIdx = store.PathIndex[index];
            if (path != null && pathIdx + 1 < path.Count)
            {
                int nextE = path[pathIdx + 1];
                if (nextE >= 0 && nextE < graph.Edges.Count && graph.Edges[nextE].FromNode >= 0)
                {
                    var nextEd = graph.Edges[nextE];
                    bool extend = true;
                    if (checkCollinearity)
                    {
                        var exitTan = graph.EvaluateBezierTangent(edgeIdx, 1f);
                        var entryTan = graph.EvaluateBezierTangent(nextE, 0f);
                        float exitLen = exitTan.Length();
                        float entryLen = entryTan.Length();
                        float dot = (exitLen > 0.001f && entryLen > 0.001f)
                            ? (exitTan.X * entryTan.X + exitTan.Y * entryTan.Y) / (exitLen * entryLen)
                            : 0f;
                        extend = dot > 0.7f;
                    }

                    if (extend)
                    {
                        float overDist = (lookaheadT - 1f) * edgeLength;
                        var edgeEnd = graph.EvaluateBezier(edgeIdx, 1f);
                        var nextStart = graph.EvaluateBezier(nextE, 0f);
                        float gapDist = Vector2.Distance(edgeEnd, nextStart);
                        overDist = MathF.Max(0f, overDist - gapDist);
                        float nextLen = MathF.Max(nextEd.Length, 0.01f);
                        float nextT = MathF.Min(overDist / nextLen, 1f);
                        return OffsetRight(graph, nextE, nextT, laneOffset);
                    }
                }
            }
            return OffsetRight(graph, edgeIdx, 1f, laneOffset);
        }

        return OffsetRight(graph, edgeIdx, lookaheadT, laneOffset);
    }

    /// <summary>
    /// Maps IDM acceleration to throttle/brake control inputs.
    /// </summary>
    private static (float throttle, float brake) MapIdmToControls(float idmAccel)
    {
        if (idmAccel >= 0f)
            return (MathF.Min(1f, idmAccel / IdmMaxAccel), 0f);
        return (0f, MathF.Min(1f, -idmAccel / MaxBrakeDecel));
    }

    /// <summary>
    /// Combined nearby vehicle scan: performs a single spatial query and computes both the
    /// lane-aware leader (ahead distance + speed) and the omnidirectional proximity distance.
    /// Replaces the previous two separate queries (FindVehicleAhead + FindNearestVehicleDistance).
    /// </summary>
    private static (float aheadDist, float leaderSpeed) FindNearbyThreats(
        VehicleStore store, int index, SpatialGrid grid, RoadGraph graph)
    {
        float vx = store.PosX[index];
        float vy = store.PosY[index];
        float heading = store.Heading[index];
        float cosH = MathF.Cos(heading);
        float sinH = MathF.Sin(heading);
        int myEdge = store.CurrentEdge[index];

        // Compute road tangent direction for all projections (immune to heading lag on curves)
        float fwdCos = cosH, fwdSin = sinH;
        float myProgress = store.EdgeProgress[index];
        float edgeLength = 0f;
        if (myEdge >= 0 && store.CurrentArc[index] < 0)
        {
            edgeLength = MathF.Max(graph.Edges[myEdge].Length, 0.01f);
            var tangent = graph.EvaluateBezierTangent(myEdge, myProgress);
            float tLen = tangent.Length();
            if (tLen > 0.001f)
            {
                fwdCos = tangent.X / tLen;
                fwdSin = tangent.Y / tLen;
                // If heading diverges significantly from road (> 60° off), skip threats
                float alignDot = cosH * fwdCos + sinH * fwdSin;
                if (alignDot < 0.5f)
                    return (float.MaxValue, 0f);
            }
        }

        var nearby = _nearbyBuffer ??= new List<int>();
        nearby.Clear();
        grid.QueryFiltered(vx, vy, CollisionSearchRadius, store.PosX, store.PosY, nearby);

        float minAheadDist = float.MaxValue;
        float leaderSpeed = 0f;

        foreach (int other in nearby)
        {
            if (other == index) continue;
            if (store.State[other] != VehicleState.Driving) continue;

            float otherCosH = MathF.Cos(store.Heading[other]);
            float otherSinH = MathF.Sin(store.Heading[other]);

            // Skip vehicles facing away from their road (mid-U-turn)
            if (store.CurrentArc[other] < 0 && store.CurrentEdge[other] >= 0)
            {
                var otherTangent = graph.EvaluateBezierTangent(
                    store.CurrentEdge[other], store.EdgeProgress[other]);
                float otLen = otherTangent.Length();
                if (otLen > 0.001f)
                {
                    float otAlignDot = otherCosH * (otherTangent.X / otLen)
                                     + otherSinH * (otherTangent.Y / otLen);
                    if (otAlignDot < 0.5f) continue;
                }
            }

            float dx = store.PosX[other] - vx;
            float dy = store.PosY[other] - vy;
            float forward = dx * fwdCos + dy * fwdSin;
            float lateral = -dx * fwdSin + dy * fwdCos;
            float headingDot = cosH * otherCosH + sinH * otherSinH;

            // === Leader detection (ahead in same lane) ===
            // Same-edge vehicles: use path distance along edge (immune to heading and curve geometry)
            if (store.CurrentEdge[other] == myEdge && store.CurrentArc[index] < 0 && store.CurrentArc[other] < 0)
            {
                float otherProgress = store.EdgeProgress[other];
                if (otherProgress > myProgress)
                {
                    float pathDist = (otherProgress - myProgress) * edgeLength;
                    float myCurrentOff = LaneWidth * (0.5f + store.CurrentLane[index]);
                    float myTargetOff = LaneWidth * (0.5f + store.TargetLane[index]);
                    float otherCurrentOff = LaneWidth * (0.5f + store.CurrentLane[other]);
                    float otherTargetOff = LaneWidth * (0.5f + store.TargetLane[other]);
                    float threshold = LaneWidth * 0.7f;
                    bool inLane = MathF.Abs(myCurrentOff - otherCurrentOff) < threshold ||
                                  MathF.Abs(myCurrentOff - otherTargetOff) < threshold ||
                                  MathF.Abs(myTargetOff - otherCurrentOff) < threshold ||
                                  MathF.Abs(myTargetOff - otherTargetOff) < threshold;
                    if (inLane && pathDist < minAheadDist)
                    {
                        minAheadDist = pathDist;
                        leaderSpeed = store.Speed[other];
                    }
                }
            }
            // Cross-edge / arc vehicles: use road-tangent-based projection
            else if (forward > 0f)
            {
                bool diverging = headingDot < 0.85f;
                if (!diverging)
                {
                    bool inLane = MathF.Abs(lateral) <= LaneWidth * 0.7f;
                    if (inLane && forward < minAheadDist)
                    {
                        minAheadDist = forward;
                        leaderSpeed = store.Speed[other];
                    }
                }
            }

        }

        return (minAheadDist, leaderSpeed);
    }

    /// <summary>
    /// Finds the distance and speed of the nearest vehicle ahead in the forward cone.
    /// Delegates to <see cref="FindNearbyThreats"/> (returns only the ahead component).
    /// </summary>
    private static (float distance, float leaderSpeed) FindVehicleAhead(VehicleStore store, int index, SpatialGrid grid, RoadGraph graph)
        => FindNearbyThreats(store, index, grid, graph);

    /// <summary>
    /// Updates steering, throttle, and brake for all active vehicles.
    /// </summary>
    public static void UpdateAll(VehicleStore store, RoadGraph graph, SpatialGrid grid, StopLineCache stopLines, IntersectionArcCache arcCache, TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns, float dt)
    {
        // Grow detection arrays if needed
        int needed = store.Count + 64;
        if (_prevPathIdx.Length < store.Count)
        {
            Array.Resize(ref _prevPathIdx, needed);
            Array.Resize(ref _prevEdgeProg, needed);
            Array.Resize(ref _prevEdge, needed);
        }

        // Rebuild the arc-occupancy index from the current arc state, then keep it live during this
        // pass via SetArc as vehicles enter/exit arcs (scan #1 reads it instead of scanning all vehicles).
        _arcOccupancy.Rebuild(store, arcCache.ArcCount);

        _tArc = _tProject = _tSignals = _tTransition = _tSteer = _tSpeed = 0; // TEMP diagnostic reset

        // Ensure conflict-tracking arrays are sized
        if (DebugLoggingEnabled)
            EnsureConflictArrays(store.Count);

        for (int i = 0; i < store.Count; i++)
        {
            // Check if any active conflict has resolved (no blocking event for cooldown period)
            CheckConflictResolved(store, i);

            // Record pre-update state into ring buffer for conflict context
            RecordRingFrame(store, i);

            int beforeIdx = store.PathIndex[i];
            int beforeEdge = store.CurrentEdge[i];
            float beforeProg = store.EdgeProgress[i];

            Update(store, i, graph, grid, stopLines, arcCache, signals, stopSigns, yieldSigns, dt);

            int afterIdx = store.PathIndex[i];
            int afterEdge = store.CurrentEdge[i];
            float afterProg = store.EdgeProgress[i];

            if (store.State[i] != VehicleState.Driving) goto done;

            // Detect cross-tick skip: pathIdx advanced by 2+ since last tick
            if (afterIdx > _prevPathIdx[i] + 1 && _prevPathIdx[i] >= 0)
            {
                LogSkip(
                    $"*** V{i} CROSS-TICK SKIP prevTick={_prevPathIdx[i]} " +
                    $"thisTick={afterIdx} edge {beforeEdge}->{afterEdge} " +
                    $"pos=({store.PosX[i]:F1},{store.PosY[i]:F1}) spd={store.Speed[i]:F2}");
            }

            // Detect large progress jump on the same edge (> 0.15 in one tick)
            if (afterEdge == _prevEdge[i] && _prevEdge[i] >= 0
                && store.CurrentArc[i] < 0)
            {
                float progDelta = afterProg - _prevEdgeProg[i];
                if (progDelta > 0.15f)
                {
                    float edgeLen = afterEdge < graph.Edges.Count ? graph.Edges[afterEdge].Length : 0f;
                    LogSkip(
                        $"*** V{i} PROG_JUMP edge={afterEdge} prog {_prevEdgeProg[i]:F4}->{afterProg:F4} " +
                        $"delta={progDelta:F4} ({progDelta * edgeLen:F1}m) " +
                        $"pos=({store.PosX[i]:F1},{store.PosY[i]:F1}) spd={store.Speed[i]:F2} " +
                        $"lane={store.CurrentLane[i]} target={store.TargetLane[i]} " +
                        $"lcProg={store.LaneChangeProgress[i]:F2}");
                }
            }

            done:
            _prevPathIdx[i] = afterIdx;
            _prevEdgeProg[i] = afterProg;
            _prevEdge[i] = afterEdge;
        }

        // TEMP diagnostic: publish the steering sub-phase breakdown for this tick.
        double _toMs = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        LastProfile = new SteeringProfile
        {
            ArcMs = _tArc * _toMs,
            ProjectMs = _tProject * _toMs,
            SignalsMs = _tSignals * _toMs,
            TransitionMs = _tTransition * _toMs,
            SteerMs = _tSteer * _toMs,
            SpeedMs = _tSpeed * _toMs,
        };

#if DEBUG
        // Collision tripwire: count vehicle pairs simultaneously on mutually-conflicting arcs.
        // On an uncontrolled network this must remain 0 — nonzero means two vehicles are inside
        // conflicting intersection arcs at once (a regression in the arc-conflict logic). Cheap via
        // the occupancy index. (Signalized nodes may show legitimate same-phase green pairs, so this
        // is a clean assertion only on the unsignalized stress grid.)
        int conflictPairs = 0;
        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            int a = store.CurrentArc[i];
            if (a < 0) continue;
            foreach (int c in arcCache.GetConflictingArcs(a))
                conflictPairs += _arcOccupancy.OccupantCount(c);
        }
        LastConflictCoOccupancy = conflictPairs;
#endif
    }

    /// <summary>
    /// Offsets a point on the Bézier curve to the right of the travel direction.
    /// Delegates to <see cref="GeometryUtil.OffsetRight"/>.
    /// </summary>
    private static Vector2 OffsetRight(RoadGraph graph, int edgeIdx, float t, float offset)
        => GeometryUtil.OffsetRight(graph, edgeIdx, t, offset);

    /// <summary>
    /// Generic parametric nearest-t search: finds the t in [searchMin, searchMax] closest to (px, py)
    /// using the provided evaluate function. Shared by edge and arc projection.
    /// </summary>
    private static float FindNearestTGeneric(float px, float py, float currentT,
        float windowBack, float windowForward, Func<float, Vector2> evaluate)
    {
        float bestT = currentT;
        float bestDist = float.MaxValue;

        float searchMin = MathF.Max(0f, currentT - windowBack);
        float searchMax = MathF.Min(1.0f, currentT + windowForward);
        const int steps = 20;

        for (int i = 0; i <= steps; i++)
        {
            float t = searchMin + (searchMax - searchMin) * i / steps;
            var pt = evaluate(t);
            float dx = pt.X - px;
            float dy = pt.Y - py;
            float dist = dx * dx + dy * dy;
            if (dist < bestDist)
            {
                bestDist = dist;
                bestT = t;
            }
        }

        return bestT;
    }

    /// <summary>
    /// Finds the parametric t on the Bézier closest to (px, py), searching in a window near currentT.
    /// </summary>
    private static float FindNearestT(RoadGraph graph, int edgeIdx, float px, float py, float currentT)
        => FindNearestTGeneric(px, py, currentT, 0.05f, 0.15f, t => graph.EvaluateBezier(edgeIdx, t));

    /// <summary>
    /// Finds the parametric t on an intersection arc Bezier closest to (px, py).
    /// Uses a wider search window than edges since arcs are short and vehicle may lag behind.
    /// </summary>
    private static float FindNearestTOnArc(IntersectionArcCache arcCache, int arcIdx, float px, float py, float currentT)
        => FindNearestTGeneric(px, py, currentT, 0.1f, 0.25f, t => arcCache.EvaluateArc(arcIdx, t));

    /// <summary>Normalizes an angle to the range [-pi, pi].</summary>
    private static float NormalizeAngle(float angle)
    {
        while (angle > MathF.PI) angle -= 2f * MathF.PI;
        while (angle < -MathF.PI) angle += 2f * MathF.PI;
        return angle;
    }

    /// <summary>
    /// Master flag to enable/disable diagnostic file logging (diag.log, skip_debug.log, arc_conflict.log).
    /// When false, LogSkip, LogDiag, and LogArcConflict are no-ops and no file handles are opened.
    /// </summary>
    public static bool DebugLoggingEnabled { get; set; }

    /// <summary>Shared writer for per-frame diagnostic logging.</summary>
    private static StreamWriter? _diagWriter;

    /// <summary>Writer for segment-skip diagnostic logging.</summary>
    private static StreamWriter? _skipWriter;

    /// <summary>Writer for arc conflict diagnostic logging.</summary>
    private static StreamWriter? _arcConflictWriter;

    // ── Conflict throttle tracking ──

    /// <summary>Per-vehicle: true if a blocking conflict is active (held until cooldown expires).</summary>
    private static bool[] _conflictActive = Array.Empty<bool>();
    /// <summary>Per-vehicle: the blocker vehicle index of the ongoing conflict, or -1.</summary>
    private static int[] _activeConflictBlocker = Array.Empty<int>();
    /// <summary>Per-vehicle: sub-step count since the current conflict started.</summary>
    private static int[] _conflictFrameCount = Array.Empty<int>();
    /// <summary>Per-vehicle: timestamp when the current conflict started.</summary>
    private static DateTime[] _conflictStartTime = Array.Empty<DateTime>();
    /// <summary>Per-vehicle: timestamp of the last blocking LogArcConflict call (for cooldown).</summary>
    private static DateTime[] _conflictLastBlockTime = Array.Empty<DateTime>();
    /// <summary>Per-vehicle: timestamp of the last periodic log line for this conflict.</summary>
    private static DateTime[] _conflictLastLogTime = Array.Empty<DateTime>();
    /// <summary>Interval between periodic "still blocked" log lines.</summary>
    private const double ConflictLogIntervalSec = 1.0;
    /// <summary>How long after the last blocking event before a conflict is considered resolved.</summary>
    private const double ConflictResolveCooldownSec = 0.5;

    // ── Pre-conflict ring buffer ──

    /// <summary>Per-vehicle ring buffer storing the last N frames of state before a conflict.</summary>
    private static RingEntry[][]? _ringBuffers;
    /// <summary>Per-vehicle write index into the ring buffer.</summary>
    private static int[]? _ringIndex;
    /// <summary>Number of frames stored in each ring buffer.</summary>
    private const int RingSize = 5;

    private struct RingEntry
    {
        public float PosX, PosY, Speed, EdgeProgress, ArcProgress;
        public int Edge, Arc;
        public byte Lane;
        public DateTime Time;
        public bool Valid;
    }

    /// <summary>Ensures conflict-tracking and ring-buffer arrays are large enough for the vehicle count.</summary>
    private static void EnsureConflictArrays(int count)
    {
        if (_conflictActive.Length >= count) return;
        int oldLen = _activeConflictBlocker.Length;
        int newCap = Math.Max(count, 16);
        Array.Resize(ref _conflictActive, newCap);
        Array.Resize(ref _activeConflictBlocker, newCap);
        Array.Resize(ref _conflictFrameCount, newCap);
        Array.Resize(ref _conflictStartTime, newCap);
        Array.Resize(ref _conflictLastBlockTime, newCap);
        Array.Resize(ref _conflictLastLogTime, newCap);
        // Initialize new blocker slots to -1
        for (int i = oldLen; i < newCap; i++)
            _activeConflictBlocker[i] = -1;

        _ringBuffers ??= Array.Empty<RingEntry[]>();
        _ringIndex ??= Array.Empty<int>();
        if (_ringBuffers.Length < newCap)
        {
            int ringOldLen = _ringBuffers.Length;
            Array.Resize(ref _ringBuffers, newCap);
            Array.Resize(ref _ringIndex, newCap);
            for (int i = ringOldLen; i < newCap; i++)
                _ringBuffers[i] = new RingEntry[RingSize];
        }
    }

    /// <summary>Records one frame of vehicle state into the per-vehicle ring buffer.</summary>
    internal static void RecordRingFrame(VehicleStore store, int index)
    {
        if (!DebugLoggingEnabled) return;
        EnsureConflictArrays(store.Count);
        if (_ringBuffers == null || _ringIndex == null) return;
        int slot = _ringIndex[index] % RingSize;
        _ringBuffers[index][slot] = new RingEntry
        {
            PosX = store.PosX[index],
            PosY = store.PosY[index],
            Speed = store.Speed[index],
            EdgeProgress = store.EdgeProgress[index],
            ArcProgress = store.ArcProgress[index],
            Edge = store.CurrentEdge[index],
            Arc = store.CurrentArc[index],
            Lane = store.CurrentLane[index],
            Time = DateTime.Now,
            Valid = true,
        };
        _ringIndex[index]++;
    }

    /// <summary>Disposes diagnostic log writers and resets conflict state. Call on application shutdown.</summary>
    public static void Shutdown()
    {
        _diagWriter?.Dispose();
        _diagWriter = null;
        _skipWriter?.Dispose();
        _skipWriter = null;
        _arcConflictWriter?.Dispose();
        _arcConflictWriter = null;
        _conflictActive = Array.Empty<bool>();
        _activeConflictBlocker = Array.Empty<int>();
        _conflictFrameCount = Array.Empty<int>();
        _conflictStartTime = Array.Empty<DateTime>();
        _conflictLastBlockTime = Array.Empty<DateTime>();
        _conflictLastLogTime = Array.Empty<DateTime>();
        _ringBuffers = null;
        _ringIndex = null;
    }

    private static void LogSkip(string msg)
    {
        if (!DebugLoggingEnabled) return;
        _skipWriter ??= new StreamWriter("skip_debug.log", append: false) { AutoFlush = true };
        _skipWriter.WriteLine(msg);
    }

    /// <summary>Logs per-frame diagnostic data for a tracked vehicle.</summary>
    internal static void LogDiag(VehicleStore store, int index, string evt)
    {
        if (!DebugLoggingEnabled) return;
        if (store.DiagVehicle != index) return;
        _diagWriter ??= new StreamWriter("diag.log", append: true) { AutoFlush = true };
        _diagWriter.WriteLine(
            $"V{index} pos=({store.PosX[index]:F2},{store.PosY[index]:F2}) " +
            $"hdg={store.Heading[index] * 180f / MathF.PI:F1} spd={store.Speed[index]:F2} " +
            $"edge={store.CurrentEdge[index]} prog={store.EdgeProgress[index]:F4} " +
            $"arc={store.CurrentArc[index]} arcProg={store.ArcProgress[index]:F4} " +
            $"steer={store.SteeringAngle[index] * 180f / MathF.PI:F1} " +
            $"prevErr={store.PrevHeadingError[index]:F4} {evt}");
    }

    /// <summary>
    /// Formats a one-line snapshot of a vehicle's state for inclusion in conflict log entries.
    /// </summary>
    private static string FormatVehicleState(VehicleStore store, int index)
    {
        return $"pos=({store.PosX[index]:F2},{store.PosY[index]:F2}) " +
               $"spd={store.Speed[index]:F2} brake={store.Brake[index]:F2} " +
               $"edge={store.CurrentEdge[index]} edgeProg={store.EdgeProgress[index]:F4} " +
               $"arc={store.CurrentArc[index]} arcProg={store.ArcProgress[index]:F4} " +
               $"lane={store.CurrentLane[index]}->{store.TargetLane[index]}";
    }

    /// <summary>
    /// Logs arc conflict events with blocker state, throttled output, pre-conflict context, and deadlock detection.
    /// Blocking conflicts (ARC_BLOCKED, ARC_BLOCKED_MERGE, HARD_OVERLAP_ARC) are throttled:
    /// first occurrence emits START + ring-buffer history, then periodic summaries every 1s, then RESOLVED when cleared.
    /// Informational events (ARC_ENTER_CONFLICT, ARC_EXIT_OVERLAP) are always logged once.
    /// </summary>
    private static void LogArcConflict(VehicleStore store, int index, string evt, int blockerIndex = -1)
    {
        if (!DebugLoggingEnabled) return;
        _arcConflictWriter ??= new StreamWriter("arc_conflict.log", append: false) { AutoFlush = true };
        EnsureConflictArrays(store.Count);

        var now = DateTime.Now;

        bool isBlocking = evt.StartsWith("ARC_BLOCKED") || evt.StartsWith("HARD_OVERLAP_ARC");

        // Build blocker info string
        string blockerInfo = "";
        if (blockerIndex >= 0 && blockerIndex < store.Count)
        {
            blockerInfo = $" | blockerV{blockerIndex}: {FormatVehicleState(store, blockerIndex)}";
        }

        string selfState = $"[{now:HH:mm:ss.fff}] V{index} {FormatVehicleState(store, index)} {evt}";

        if (!isBlocking)
        {
            // Informational events: always log, no throttling
            _arcConflictWriter.WriteLine($"{selfState}{blockerInfo}");
            return;
        }

        // Update blocker and last-block timestamp
        _activeConflictBlocker[index] = blockerIndex;
        _conflictLastBlockTime[index] = now;

        if (!_conflictActive[index])
        {
            // New conflict — emit START line with ring buffer history
            _conflictActive[index] = true;
            _conflictFrameCount[index] = 1;
            _conflictStartTime[index] = now;
            _conflictLastLogTime[index] = now;

            _arcConflictWriter.WriteLine($"{selfState}{blockerInfo} (START)");

            // Dump ring buffer for this vehicle and blocker
            DumpRingBuffer(index);
            if (blockerIndex >= 0) DumpRingBuffer(blockerIndex);

            // Check for deadlock on first conflict frame
            DetectDeadlock(store, index);
        }
        else
        {
            // Ongoing conflict — only log periodically
            _conflictFrameCount[index]++;
            if ((now - _conflictLastLogTime[index]).TotalSeconds >= ConflictLogIntervalSec)
            {
                float elapsed = (float)(now - _conflictStartTime[index]).TotalSeconds;
                _arcConflictWriter.WriteLine(
                    $"{selfState}{blockerInfo} (blocked {elapsed:F1}s, {_conflictFrameCount[index]} frames)");
                _conflictLastLogTime[index] = now;

                // Re-check for deadlock periodically
                DetectDeadlock(store, index);
            }
        }
    }

    /// <summary>Dumps a vehicle's ring buffer to the arc conflict log.</summary>
    private static void DumpRingBuffer(int vehicleIndex)
    {
        if (_arcConflictWriter == null || _ringBuffers == null || _ringIndex == null) return;
        if (vehicleIndex < 0 || vehicleIndex >= _ringBuffers.Length) return;
        var ring = _ringBuffers[vehicleIndex];
        int writeIdx = _ringIndex[vehicleIndex];
        for (int k = RingSize; k > 0; k--)
        {
            int slot = (writeIdx - k + RingSize * 100) % RingSize;
            if (ring[slot].Valid)
            {
                var e = ring[slot];
                _arcConflictWriter.WriteLine(
                    $"  pre-conflict V{vehicleIndex} [{e.Time:HH:mm:ss.fff}] " +
                    $"pos=({e.PosX:F2},{e.PosY:F2}) spd={e.Speed:F2} " +
                    $"edge={e.Edge} edgeProg={e.EdgeProgress:F4} " +
                    $"arc={e.Arc} arcProg={e.ArcProgress:F4} lane={e.Lane}");
            }
        }
    }

    /// <summary>
    /// Called once per vehicle per tick BEFORE Update. If a conflict is active but
    /// no blocking event has occurred for the cooldown period, emits RESOLVED and clears state.
    /// </summary>
    internal static void CheckConflictResolved(VehicleStore store, int index)
    {
        if (!DebugLoggingEnabled) return;
        if (index >= _conflictActive.Length || !_conflictActive[index]) return;

        var now = DateTime.Now;
        double sinceLast = (now - _conflictLastBlockTime[index]).TotalSeconds;
        if (sinceLast >= ConflictResolveCooldownSec)
        {
            float elapsed = (float)(now - _conflictStartTime[index]).TotalSeconds;
            _arcConflictWriter ??= new StreamWriter("arc_conflict.log", append: false) { AutoFlush = true };
            // Only log RESOLVED for non-trivial conflicts (>0.5s total duration)
            if (elapsed >= 0.5f)
            {
                _arcConflictWriter.WriteLine(
                    $"[{now:HH:mm:ss.fff}] V{index} {FormatVehicleState(store, index)} RESOLVED after {elapsed:F1}s ({_conflictFrameCount[index]} frames)");
            }
            _conflictActive[index] = false;
            _activeConflictBlocker[index] = -1;
            _conflictFrameCount[index] = 0;
        }
    }

    /// <summary>
    /// Walks the blocking chain from a vehicle to detect circular deadlocks.
    /// If a cycle is found, logs a DEADLOCK_DETECTED line listing the full chain.
    /// </summary>
    private static void DetectDeadlock(VehicleStore store, int startIndex)
    {
        if (_arcConflictWriter == null) return;

        // Walk the chain: startIndex -> blocker -> blocker's blocker -> ...
        var chain = new List<int> { startIndex };
        int current = startIndex;
        for (int step = 0; step < store.Count; step++)
        {
            if (current < 0 || current >= _activeConflictBlocker.Length) break;
            int blocker = _activeConflictBlocker[current];
            if (blocker < 0) break; // chain ends — no deadlock

            if (blocker == startIndex)
            {
                // Found a cycle back to start
                chain.Add(blocker);
                string chainStr = string.Join(" -> ", chain.Select(v => $"V{v}"));
                _arcConflictWriter.WriteLine(
                    $"[{DateTime.Now:HH:mm:ss.fff}] DEADLOCK_DETECTED: {chainStr} (cycle length {chain.Count - 1})");
                return;
            }

            if (chain.Contains(blocker))
            {
                // Cycle that doesn't include startIndex — still interesting
                chain.Add(blocker);
                string chainStr = string.Join(" -> ", chain.Select(v => $"V{v}"));
                _arcConflictWriter.WriteLine(
                    $"[{DateTime.Now:HH:mm:ss.fff}] DEADLOCK_DETECTED (indirect): {chainStr}");
                return;
            }

            chain.Add(blocker);
            current = blocker;
        }
    }
}
