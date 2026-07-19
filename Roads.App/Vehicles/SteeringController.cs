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
    /// <summary>
    /// Ceiling on the steering loop's plant gain (speed/wheelbase × driver sharpness,
    /// 1/s per radian of steer) before <see cref="SpeedGainCompensation"/> starts scaling
    /// the command down — see that method for the stability argument. 6 puts the onset at
    /// ≈15 m/s (33 mph) for a default sedan, safely below the ≈20 m/s flip-flop boundary.
    /// Runtime-tunable like Kp/Kd: lower = earlier/softer high-speed steering,
    /// float.MaxValue restores the uncompensated historical behavior exactly.
    /// </summary>
    public static float MaxYawGain = 6f;
    /// <summary>Default target speed in m/s (~22 mph), used when edge has no speed limit.</summary>
    public static float TargetSpeed = 10f;
    /// <summary>Base lookahead distance in meters at zero speed.</summary>
    public static float LookaheadBase = 3f;
    /// <summary>Additional lookahead distance per m/s of speed.</summary>
    public static float LookaheadPerSpeed = 0.3f;
    /// <summary>Gain for lateral error correction (cross-track error).</summary>
    public static float Klat = 0.5f;
    /// <summary>
    /// Cap on the lateral correction term, as a fraction of the heading gain (so it holds
    /// for any user-tuned Kp/Klat combination). The lateral term acts as a constant steering
    /// bias once a vehicle is off-lane; the vehicle's steady-state heading then settles
    /// cap/(Kp·sharpness) radians away from the bearing to its lookahead target. That angle
    /// must stay well below π/2 — at π/2 the closing speed toward the lane reaches zero and
    /// the vehicle orbits its target forever instead of converging (uncapped, any vehicle
    /// displaced more than Kp·π/Klat ≈ 15 m by a map edit was permanently trapped in a
    /// full-lock spin). 0.35 rad ≈ 20°: recovery keeps ≥94% closing speed, while in-lane
    /// corrections (|latErr| ≲ 1.7 m at default gains) are below the cap and unaffected.
    /// </summary>
    private const float LatCorrectionCapFraction = 0.35f;
    /// <summary>Lane width in meters.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;

    /// <summary>Search radius in meters for nearby vehicle queries.</summary>
    private const float CollisionSearchRadius = 30f;

    /// <summary>Body length in meters of vehicle <paramref name="i"/> (per-type, from
    /// <see cref="VehicleTypeDimensions"/>).</summary>
    private static float Len(VehicleStore store, int i)
        => VehicleTypeDimensions.GetLength(store.PreferredVehicle[i]);

    /// <summary>Half body length in meters of vehicle <paramref name="i"/> — its
    /// bumper-to-center distance. Center-to-center distances convert to bumper-to-bumper
    /// gaps by subtracting BOTH vehicles' half-lengths.</summary>
    private static float HalfLen(VehicleStore store, int i)
        => VehicleTypeDimensions.GetHalfLength(store.PreferredVehicle[i]);


    /// <summary>IDM minimum bumper-to-bumper gap in meters (s0).</summary>
    private const float IdmMinGap = 1.0f;
    /// <summary>IDM desired time headway in seconds (T).</summary>
    private const float IdmTimeHeadway = 1.5f;
    /// <summary>IDM comfortable deceleration in m/s^2 (b).</summary>
    private const float IdmComfortDecel = 2.5f;
    /// <summary>Maximum brake deceleration in m/s^2.</summary>
    private const float MaxBrakeDecel = SimConstants.MaxBrakeDecel;

    /// <summary>Launch acceleration (m/s^2) a driver with Aggressiveness 0 is willing to use.</summary>
    private const float DriverAccelBase = 1.4f;
    /// <summary>Additional launch acceleration (m/s^2) at Aggressiveness 1, on top of the base.</summary>
    private const float DriverAccelAggrRange = 1.8f;

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

    /// <summary>
    /// Per-pass count of driving vehicles on each edge (those NOT on an arc), rebuilt at the top of
    /// <see cref="UpdateAll"/>. Only consulted by the single-lane two-way (shared-lane) entry gate to
    /// detect oncoming traffic on the shared segment; the O(n) rebuild is cheap relative to steering.
    /// </summary>
    private static int[] _edgeOccupancy = Array.Empty<int>();

    /// <summary>Per-vehicle count of consecutive sim ticks spent overlap-stalled (fully stopped with a
    /// stopped vehicle overlapping it). Drives the merge deadlock-breaker; reset the moment it clears.</summary>
    private static int[] _overlapStall = Array.Empty<int>();

    /// <summary>Per-vehicle: the leader index the threat scan braked this vehicle for on the current
    /// tick, or -1. Written by ApplySpeedControl / UpdateOnArc, consumed by the deadlock breaker to
    /// detect MUTUAL-LEADER STANDOFFS: two stopped cars at a junction each classifying the other as
    /// its leader through angle-dependent projections (mid-arc vs edge-entry poses) at small POSITIVE
    /// gaps (0.4–1 m) — no physical overlap, so <see cref="HasStoppedOverlap"/> alone never ages the
    /// stall counter and the pair freezes forever. A stopped car braking for a stopped leader that is
    /// NOT path-ahead of it (see <see cref="IsAheadAlongPath"/>) is such a standoff; a normal queue
    /// never qualifies because its leader IS path-ahead.</summary>
    private static int[] _lastLeader = Array.Empty<int>();
    /// <summary>Per-vehicle breaker flag: when set, this vehicle is the "front" of a stuck overlap tangle
    /// and is allowed to creep forward through the overlap (it ignores overlapping cars in its threat /
    /// hard-overlap checks). Set by the breaker pass at the end of <see cref="UpdateAll"/>, consumed by
    /// <see cref="FindNearbyThreats"/> and <see cref="ApplyHardOverlapBrake"/> on the next tick.</summary>
    private static bool[] _breakerProceed = Array.Empty<bool>();
    /// <summary>Sim ticks a vehicle must be overlap-stalled before the breaker frees the tangle's front
    /// (~3 s at the 30 Hz sim rate) — far longer than any legitimate merge/yield wait.</summary>
    private const int OverlapStallTicks = 90;

    /// <summary>Raised on the RISING edge of a vehicle becoming the breaker-freed front of a
    /// stuck tangle (see <see cref="RunDeadlockBreaker"/>) — the audio engine's horn trigger.
    /// Fires on the sim/UI thread during UpdateAll. Null (zero-cost) unless the audio engine
    /// is running; the headless harness never subscribes, so determinism is unaffected.</summary>
    public static event Action<int>? BreakerFreed;

    /// <summary>Thread-local buffer for the merge-into-exit-lane spatial query.</summary>
    [ThreadStatic] private static List<int>? _scan2Buffer;
    /// <summary>Thread-local buffer for the merge-yield / breaker overlap scans.</summary>
    [ThreadStatic] private static List<int>? _overlapBuffer;

    /// <summary>Thread-local buffer for the arc-mode outgoing-edge leader query.</summary>
    [ThreadStatic] private static List<int>? _arcLeaderBuffer;

    /// <summary>
    /// Search radius (m) around an arc's exit point used to find a car-following leader on the
    /// outgoing edge. Covers the IDM-relevant range; a leader beyond it yields a gap too large to
    /// constrain following, so ignoring it is behaviorally equivalent to the former full scan.
    /// </summary>
    private const float ArcLeaderSearchRadius = 80f;

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

    /// <summary>
    /// Rebuilds <see cref="_edgeOccupancy"/> — the count of driving vehicles currently on each edge
    /// (excluding those on intersection arcs). One O(n) pass; consulted only by the shared-lane gate.
    /// </summary>
    private static void RebuildEdgeOccupancy(VehicleStore store, int edgeCount)
    {
        if (_edgeOccupancy.Length < edgeCount)
            _edgeOccupancy = new int[edgeCount];
        else
            Array.Clear(_edgeOccupancy, 0, edgeCount);

        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            if (store.CurrentArc[i] >= 0) continue; // on an arc, not an edge
            int e = store.CurrentEdge[i];
            if ((uint)e < (uint)edgeCount) _edgeOccupancy[e]++;
        }
    }

    /// <summary>
    /// True if the single-lane two-way segment whose oncoming direction is <paramref name="reverseEdge"/>
    /// currently has a vehicle travelling that opposite direction — either already on that edge, or
    /// committed to entering it via an intersection arc at its start node. Used to gate entry onto a
    /// <see cref="EdgeFlags.SharedLane"/> edge so two vehicles never meet head-on on the one shared lane.
    /// The arc-occupancy check (kept live mid-pass via <see cref="SetArc"/>) also resolves the symmetric
    /// both-ends-at-once case: whoever commits to its arc first is seen by the other, which then waits.
    /// </summary>
    private static bool OncomingOnSharedLane(VehicleStore store, RoadGraph graph, IntersectionArcCache arcCache, int reverseEdge)
    {
        // A vehicle physically on the oncoming direction.
        if ((uint)reverseEdge < (uint)_edgeOccupancy.Length && _edgeOccupancy[reverseEdge] > 0)
            return true;

        // A vehicle committed to entering the oncoming direction (on an arc exiting onto it). Arcs
        // feeding reverseEdge live in its FromNode's bucket.
        int rFromNode = graph.Edges[reverseEdge].FromNode;
        if (rFromNode < 0) return false;
        foreach (int arc in arcCache.GetArcsAtNode(rFromNode))
        {
            if (arcCache.GetArc(arc).OutgoingEdge == reverseEdge && _arcOccupancy.OccupantCount(arc) > 0)
                return true;
        }
        return false;
    }

    // ── Merge safety ────────────────────────────────────────────────────
    // A "merge" is two intersection arcs feeding the SAME outgoing edge (a lane-drop, or two roads
    // joining into one). Both arcs end in the same physical throat, so two cars crossing at once collide
    // and get shoved off-lane into a permanent overlap. These helpers let a car yield into a merge that
    // another car is already crossing, so only one uses the throat at a time.

    /// <summary>
    /// True if <paramref name="arcIdx"/> shares its outgoing edge with a conflicting arc (i.e. it is a
    /// merge arc) AND at least one such merge-sibling arc currently has another vehicle on it. Keyed on
    /// live <see cref="_arcOccupancy"/>, so whoever commits to its arc first is seen by the rest.
    /// </summary>
    private static bool MergeSiblingOccupied(IntersectionArcCache arcCache, int arcIdx, int selfIndex)
    {
        int outEdge = arcCache.GetArc(arcIdx).OutgoingEdge;
        foreach (int c in arcCache.GetConflictingArcs(arcIdx))
        {
            if (arcCache.GetArc(c).OutgoingEdge != outEdge) continue; // only siblings that merge onto the same edge
            int occ = _arcOccupancy.OccupantCount(c);
            for (int k = 0; k < occ; k++)
                if (_arcOccupancy.OccupantAt(c, k) != selfIndex) return true;
        }
        return false;
    }

    /// <summary>
    /// True when the vehicle is approaching a merge (its next turn's arc shares its outgoing edge with
    /// another arc) that another vehicle is currently crossing — so it should brake to its stop line
    /// instead of driving into the occupied throat. Read-only: resolves the intended arc via the cache
    /// (no reroute). Returns false when the car has no next edge or the turn has no arc.
    /// </summary>
    private static bool ApproachingOccupiedMerge(VehicleStore store, int index, RoadGraph graph,
        IntersectionArcCache arcCache, int edgeIdx)
    {
        var path = store.Path[index];
        int pathIdx = store.PathIndex[index];
        if (path == null || pathIdx + 1 >= path.Count) return false;
        int nextEdge = path[pathIdx + 1];
        if (nextEdge < 0 || nextEdge >= graph.Edges.Count || graph.Edges[nextEdge].FromNode < 0) return false;
        byte inLane = store.CurrentLane[index];
        byte outLane = (byte)Math.Min(inLane, graph.Edges[nextEdge].LaneCount - 1);
        int arcIdx = arcCache.GetArcIndex(edgeIdx, nextEdge, inLane, outLane);
        if (arcIdx < 0) return false;
        return MergeSiblingOccupied(arcCache, arcIdx, index);
    }

    // ── Merge deadlock-breaker ──────────────────────────────────────────
    // Last-resort recovery: if two cars still wedge in a merge throat despite the yield (a same-tick
    // race, tight geometry, or a pre-existing tangle), they hard-brake on each other forever. The
    // breaker tracks how long each car has been overlap-stalled and, past a threshold, frees the FRONT
    // of the tangle — the car with no stopped car genuinely ahead — by letting it creep forward and
    // bypass the merge gate (via _breakerProceed). The rest hold; once the front drains, the next car
    // becomes the front, and the pile unwinds. Bounded and self-clearing.

    private const float BreakerStoppedSpeed = 0.1f;   // m/s below which a car counts as stopped here
    private const float BreakerCreepSpeed = 1.5f;     // cap the freed front's creep speed
    private const float BreakerCreepThrottle = 0.35f; // gentle forward throttle while creeping out

    /// <summary>True while this vehicle is the breaker-freed front of a stuck overlap tangle: it bypasses
    /// the merge yield/gate and is creep-driven out of the pile. Set by <see cref="RunDeadlockBreaker"/>.</summary>
    private static bool BreakerFront(int index) => index < _breakerProceed.Length && _breakerProceed[index];

    /// <summary>True if another stopped Driving vehicle physically OVERLAPS <paramref name="i"/> — its
    /// body is past <paramref name="i"/>'s in both axes of <paramref name="i"/>'s frame. The lateral bound
    /// spans [LaneWidth·0.7, LaneWidth·0.8]: wide enough to cover everything the threat scan's leader
    /// corridor can mutually hard-brake (the breaker must see every wedge the scan can create), yet under
    /// the lane spacing so it never fires for cars legitimately queued behind/ahead (forward gap ≥ a car
    /// length) or sitting side-by-side in adjacent lanes at a light.</summary>
    private static bool HasStoppedOverlap(VehicleStore store, int i, SpatialGrid grid)
    {
        var buf = _overlapBuffer ??= new List<int>();
        buf.Clear();
        float vx = store.PosX[i], vy = store.PosY[i];
        float cosH = MathF.Cos(store.Heading[i]), sinH = MathF.Sin(store.Heading[i]);
        float myHalf = HalfLen(store, i);
        // Radius must cover the pair test below for ANY other vehicle type, so it is bounded
        // by the longest type's half-length rather than this vehicle's own.
        grid.QueryFiltered(vx, vy, (myHalf + VehicleTypeDimensions.MaxHalfLength) * 1.1f,
            store.PosX, store.PosY, buf);
        foreach (int j in buf)
        {
            if (j == i || store.State[j] != VehicleState.Driving) continue;
            if (store.Speed[j] >= BreakerStoppedSpeed) continue;
            float dx = store.PosX[j] - vx, dy = store.PosY[j] - vy;
            float forward = MathF.Abs(dx * cosH + dy * sinH);
            float lateral = MathF.Abs(-dx * sinH + dy * cosH);
            // Bodies overlap when the center distance drops under the pair's summed
            // half-lengths; the 0.9/0.6 factors keep the deliberate tightness described
            // above. The lateral bound is clamped to [LaneWidth*0.7, LaneWidth*0.8]:
            //  - FLOOR at the threat scan's leader corridor (LaneWidth*0.7): any pair the
            //    scan can mutually hard-brake must be inside breaker coverage, or the pair
            //    deadlocks forever in the gap. Unclamped, a small pair (e.g. SUV +
            //    motorcycle, pairHalf*0.6 ≈ 2.1 m) wedged at a junction corner (~2.3 m
            //    lateral) braked for each other but never aged the stall counter.
            //  - CAP below the lane spacing (3.5 m) so two LONG vehicles legitimately
            //    sitting side-by-side in adjacent lanes can never read as a wedge — an
            //    uncapped pair bound would exceed the lane spacing and let the breaker
            //    creep a queued bus through a red light.
            float pairHalf = myHalf + HalfLen(store, j);
            float lateralBound = Math.Clamp(pairHalf * 0.6f,
                SimConstants.LaneWidth * 0.7f, SimConstants.LaneWidth * 0.8f);
            if (forward < pairHalf * 0.9f && lateral < lateralBound) return true;
        }
        return false;
    }

    /// <summary>True if <paramref name="i"/> is the front of its tangle: no nearby STOPPED vehicle sits
    /// ahead of it ALONG ITS PATH (same edge/arc further along, on its outgoing edge, or on the arc/edge
    /// it feeds into). Path-relative rather than physical-cone, because at a merge the cars are crossed
    /// and shoved off-lane, so a physical "ahead" cone mistakes the car behind on the feeding arc for a
    /// leader and no one is ever the front. The front can be safely creeped out; others hold.</summary>
    private static bool IsTangleFront(VehicleStore store, int i, SpatialGrid grid, RoadGraph graph, IntersectionArcCache arcCache)
    {
        var buf = _overlapBuffer ??= new List<int>();
        buf.Clear();
        grid.QueryFiltered(store.PosX[i], store.PosY[i], Len(store, i) * 3f, store.PosX, store.PosY, buf);
        foreach (int j in buf)
        {
            if (j == i || store.State[j] != VehicleState.Driving) continue;
            if (store.Speed[j] >= BreakerStoppedSpeed) continue;          // a moving car ahead isn't a deadlock
            if (IsAheadAlongPath(store, graph, arcCache, i, j)) return false;
        }
        return true;
    }

    /// <summary>True if vehicle <paramref name="j"/> is ahead of <paramref name="i"/> along i's route:
    /// on the same arc/edge further along, on i's outgoing edge (i on an arc), or on the next edge/arc i
    /// is about to take. Path-relative so it stays correct when merge geometry crosses cars over.</summary>
    private static bool IsAheadAlongPath(VehicleStore store, RoadGraph graph, IntersectionArcCache arcCache, int i, int j)
    {
        int ia = store.CurrentArc[i], ja = store.CurrentArc[j];
        if (ia >= 0)
        {
            if (ja == ia && store.ArcProgress[j] > store.ArcProgress[i]) return true;             // same arc, ahead
            if (ja < 0 && store.CurrentEdge[j] == arcCache.GetArc(ia).OutgoingEdge) return true;  // j exited onto i's outgoing edge
            return false;
        }
        int ie = store.CurrentEdge[i];
        if (ja < 0)
        {
            if (store.CurrentEdge[j] == ie && store.EdgeProgress[j] > store.EdgeProgress[i]) return true; // same edge, ahead
            var path = store.Path[i];
            int pi = store.PathIndex[i];
            if (path != null && pi + 1 < path.Count && store.CurrentEdge[j] == path[pi + 1]) return true; // j on i's next edge
        }
        else if (arcCache.GetArc(ja).IncomingEdge == ie)
        {
            return true; // j is on the arc leaving i's current edge — committed into the intersection ahead of i
        }
        return false;
    }

    /// <summary>
    /// End-of-pass deadlock breaker. Ages an overlap-stall counter per vehicle and, once a car has been
    /// wedged past <see cref="OverlapStallTicks"/>, frees it if it is the tangle front (creep + merge-gate
    /// bypass). Cheap: only stopped cars run the overlap scans.
    /// </summary>
    private static void RunDeadlockBreaker(VehicleStore store, SpatialGrid grid, RoadGraph graph, IntersectionArcCache arcCache)
    {
        int n = store.Count;
        if (_overlapStall.Length < n) Array.Resize(ref _overlapStall, n + 64);
        if (_breakerProceed.Length < n) Array.Resize(ref _breakerProceed, n + 64);

        for (int i = 0; i < n; i++)
        {
            if (store.State[i] != VehicleState.Driving || store.Speed[i] >= BreakerStoppedSpeed)
            {
                _overlapStall[i] = 0;
                _breakerProceed[i] = false;
                continue;
            }

            // A wedge is EITHER a physical body overlap OR a mutual-leader standoff: this
            // stopped car is braking for a stopped "leader" that is not actually ahead of it
            // along its path (angle-dependent projection at a junction — see _lastLeader).
            // Normal queues never age here: their leader is path-ahead.
            bool stalled = HasStoppedOverlap(store, i, grid);
            if (!stalled && i < _lastLeader.Length)
            {
                int ldr = _lastLeader[i];
                stalled = ldr >= 0 && ldr < store.Count
                    && store.State[ldr] == VehicleState.Driving
                    && store.Speed[ldr] < BreakerStoppedSpeed
                    && !IsAheadAlongPath(store, graph, arcCache, i, ldr);
            }
            _overlapStall[i] = stalled ? _overlapStall[i] + 1 : 0;

            bool front = _overlapStall[i] >= OverlapStallTicks && IsTangleFront(store, i, grid, graph, arcCache);
            bool wasFront = _breakerProceed[i];
            _breakerProceed[i] = front;
            if (front && !wasFront)
                BreakerFreed?.Invoke(i);
            if (front && store.Speed[i] < BreakerCreepSpeed)
            {
                store.Brake[i] = 0f;
                store.Throttle[i] = BreakerCreepThrottle;
                LogDiag(store, i, $"MERGE_BREAKER creep front (stall={_overlapStall[i]})");
            }
        }
    }

    /// <summary>
    /// Updates a vehicle's steering angle, throttle, brake, and edge/arc progress.
    /// Combines PD heading control, lateral error correction, IDM car-following,
    /// signal/stop-sign/yield compliance, and proximity-based collision avoidance.
    /// Delegates to <see cref="UpdateOnArc"/> when the vehicle is traversing an intersection arc.
    /// </summary>
    public static void Update(VehicleStore store, int index, RoadGraph graph, SpatialGrid grid, StopLineCache stopLines, IntersectionArcCache arcCache, TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns, float dt)
    {
        if (store.State[index] != VehicleState.Driving) return;

        // Cleared up front so hold paths that return before the threat scan can't leave a
        // stale (possibly swap-reused) leader index for the breaker's standoff detection.
        SetLastLeader(index, -1);

        // If vehicle is on an intersection arc, use arc-mode steering
        if (store.CurrentArc[index] >= 0)
        {
            store.DistToRoadSq[index] = 0f; // arc vehicles are always on-road
            UpdateOnArc(store, index, graph, grid, stopLines, arcCache, dt);
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
            float laneOff = GeometryUtil.LaneLateralOffset(graph, edgeIdx, store.CurrentLane[index]);
            var laneCenter = OffsetRight(graph, edgeIdx, progress, laneOff);
            float rdx = vx - laneCenter.X;
            float rdy = vy - laneCenter.Y;
            store.DistToRoadSq[index] = rdx * rdx + rdy * rdy;
        }

        // Evaluate signals, stop signs, and yield signs
        var (signal, yieldSignal) = EvaluateSignals(store, index, edgeIdx, graph, signals, stopSigns, yieldSigns);

        // Compute stop-line position in t-space (center stops a half body-length short of
        // the line, so the FRONT bumper — not the center — lands on it, per vehicle type).
        // Clamped to the edge's ENTRY stop line: on an edge shorter than the vehicle's
        // half-length the raw target would be negative — an unrepresentable position that
        // holds could write into EdgeProgress, making the vehicle invisible to the
        // stop-sign FCFS (which requires progress > 0) and deadlocking the node. A long
        // vehicle instead waits at the edge start with its nose protruding past the line.
        float stopT = stopLines.GetStopTAtToNode(edgeIdx);
        float halfVehT = HalfLen(store, index) / edgeLength;
        float stopAtT = MathF.Max(stopT - halfVehT, minProgressT);

        // Block crossing on red
        if (CheckRedLightBlocking(store, index, signal, speed, progress, stopAtT, edgeLength, store.BrakingComfort[index]))
            return;

        // Handle edge transitions (arc entrance, direct jump, or end-of-path stop)
        var transition = HandleEdgeTransition(store, index, graph, stopLines, arcCache, signals, grid, ref edgeIdx, ref edge, ref edgeLength, ref progress, ref rawProgress, ref baseSpeedLimit, ref targetSpeed, ref stopT, ref halfVehT, ref stopAtT, speed, vx, vy);
        if (transition == TransitionResult.Returned) return;

        store.EdgeProgress[index] = progress;

        // PD steering
        ComputeSteering(store, index, graph, edgeIdx, rawProgress, progress, edgeLength, minProgressT, vx, vy, dt);

        // IDM car-following + throttle/brake
        ApplySpeedControl(store, index, graph, grid, stopLines, arcCache, edgeIdx, edgeLength, speed, targetSpeed, progress, stopAtT, signal, yieldSignal);

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
                            // Candidates lie within the pair-length cutoff of the exit point (arc.P3
                            // = the out-lane start), so query the grid around P3 instead of scanning
                            // every vehicle. The radius uses the longest type's half-length so it is
                            // a superset of the per-pair cutoff re-applied below (no false negatives).
                            float myHalf = HalfLen(store, index);
                            var exitPoint = arc.P3;
                            var scan2 = _scan2Buffer ??= new List<int>();
                            scan2.Clear();
                            grid.QueryFiltered(exitPoint.X, exitPoint.Y,
                                myHalf + VehicleTypeDimensions.MaxHalfLength
                                    + SimConstants.VehicleLength + SpatialGrid.CellSize,
                                store.PosX, store.PosY, scan2);
                            for (int bi = 0; bi < scan2.Count; bi++)
                            {
                                int j = scan2[bi];
                                if (j == index || store.State[j] != VehicleState.Driving) continue;
                                if (store.CurrentEdge[j] != outEdge || store.CurrentArc[j] >= 0) continue;
                                // Don't-block-the-box: refuse to COMMIT into the intersection while a
                                // stopped vehicle in the exit lane plugs the throat — my body would not
                                // clear the arc and would freeze ON it, blocking every crossing movement
                                // (the anchor of observed spillback-ring gridlocks). Moving exit-lane
                                // traffic is fine (arc IDM look-ahead paces behind it); a breaker front
                                // is exempt so it can still be creeped out of an existing tangle.
                                if (store.CurrentLane[j] == outLane)
                                {
                                    float exitSpan = (store.EdgeProgress[j] - outStartT) * outLength
                                        - HalfLen(store, j);
                                    if (!BreakerFront(index) && store.Speed[j] < 0.5f
                                        && exitSpan < Len(store, index) + 1f)
                                    {
                                        store.EdgeProgress[index] = stopAtT;
                                        store.Throttle[index] = 0f;
                                        store.Brake[index] = 1.0f;
                                        store.Speed[index] = 0f;
                                        LogArcConflict(store, index,
                                            $"ARC_BLOCKED_EXIT_FULL arcIdx={arcIdx} outEdge={outEdge} " +
                                            $"outLane={outLane} blockedByV={j} exitSpan={exitSpan:F2}",
                                            blockerIndex: j);
                                        return TransitionResult.Returned;
                                    }
                                    continue; // moving / far enough — arc IDM look-ahead handles pacing
                                }
                                // Only block if this vehicle is actively lane-changing into our exit lane
                                if (store.TargetLane[j] != outLane) continue;

                                // Blocked while the merger's body plus a sedan-length safety margin
                                // still covers the exit throat (pair half-lengths, per vehicle type).
                                float edgeDist = (store.EdgeProgress[j] - outStartT) * outLength;
                                if (edgeDist < myHalf + HalfLen(store, j) + SimConstants.VehicleLength)
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

                    // Closed-edge gate: a closed edge is draining (will be removed once empty), so
                    // refuse NEW entrants arriving via this intersection arc. Cars already on / spawned
                    // onto the closed edge are unaffected — they don't re-enter through this arc gate,
                    // so they finish crossing and let the edge empty out.
                    if (graph.IsEdgeClosed(arc.OutgoingEdge))
                    {
                        store.EdgeProgress[index] = stopAtT;
                        store.Throttle[index] = 0f;
                        store.Brake[index] = 1.0f;
                        store.Speed[index] = 0f;
                        return TransitionResult.Returned;
                    }

                    // Single-lane two-way (shared-lane) gate: don't enter a one-lane shared segment
                    // while a vehicle is travelling the OPPOSITE direction on it (already on the edge,
                    // or committed to entering it from the far end). Same-direction following is fine —
                    // normal car-following handles spacing. Prevents a head-on on the one shared lane.
                    if ((graph.Edges[arc.OutgoingEdge].Flags & EdgeFlags.SharedLane) != 0)
                    {
                        int rev = graph.FindReverseEdge(arc.OutgoingEdge);
                        if (rev >= 0 && OncomingOnSharedLane(store, graph, arcCache, rev))
                        {
                            store.EdgeProgress[index] = stopAtT;
                            store.Throttle[index] = 0f;
                            store.Brake[index] = 1.0f;
                            store.Speed[index] = 0f;
                            return TransitionResult.Returned;
                        }
                    }

                    // Merge gate: don't enter a merge arc (one sharing its outgoing edge with another arc)
                    // while a vehicle is crossing the merge from a conflicting approach — the two would
                    // collide in the single throat and wedge off-lane. Backstop at the line for the
                    // approach-phase merge yield; mirrors the shared-lane gate above. A breaker front is
                    // exempt so it can drive out of an existing tangle.
                    if (!BreakerFront(index) && MergeSiblingOccupied(arcCache, arcIdx, index))
                    {
                        store.EdgeProgress[index] = stopAtT;
                        store.Throttle[index] = 0f;
                        store.Brake[index] = 1.0f;
                        store.Speed[index] = 0f;
                        return TransitionResult.Returned;
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

            // Wrong-lane hold: the planned turn exists — but only from another lane of this
            // edge (e.g. a 2-lane one-way necking into 1 lane generates its merge arc from
            // the outer lane only). Direct-transferring here would teleport the vehicle's
            // logical position onto the outgoing edge while its body stays at the end of the
            // terminating lane — an off-lane ghost that wedges the whole merge (the Bug-B
            // overlap deadlock). Hold at the line instead; LaneChangeLogic's turn preparation
            // (which allows standstill merges at critical urgency) moves the vehicle into the
            // arc-bearing lane when a gap opens.
            if (arcIdx < 0 && TurnServedByOtherLane(arcCache, graph, edgeIdx, nextEdge, inLane))
            {
                store.EdgeProgress[index] = stopAtT;
                store.Throttle[index] = 0f;
                store.Brake[index] = 1.0f;
                store.Speed[index] = 0f;
                return TransitionResult.Returned;
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
    /// True when the planned turn (edgeIdx → nextEdge) is drivable from SOME lane of the
    /// current edge other than <paramref name="inLane"/> — i.e. the vehicle is merely in
    /// the wrong lane for its turn (e.g. the through lane of a 2-lane road whose merge
    /// arc exists only from the outer lane) and must merge over rather than transfer.
    /// </summary>
    private static bool TurnServedByOtherLane(IntersectionArcCache arcCache, RoadGraph graph,
        int edgeIdx, int nextEdge, byte inLane)
    {
        byte laneCount = graph.Edges[edgeIdx].LaneCount;
        for (byte lane = 0; lane < laneCount; lane++)
        {
            if (lane == inLane) continue;
            var reachable = arcCache.GetReachableFromLane(edgeIdx, lane);
            if (reachable == null) continue;
            foreach (var (outEdge, _, _) in reachable)
                if (outEdge == nextEdge) return true;
        }
        return false;
    }

    /// <summary>
    /// Resolves the intersection arc for a lane transition: tries the lane-preserving
    /// guesses first, then ANY arc from the current lane to the planned next edge (the
    /// arc's own exit lane is authoritative — turn arcs may exit onto a different lane
    /// index than they enter from), and only reroutes when no arc reaches the planned
    /// edge at all. May update path, pathIdx, nextEdge, and nextEdgeData if a reroute occurs.
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

        // The generated arc's exit lane follows the geometry/restriction pairing (e.g. a
        // right turn exits onto the OUTERMOST lane of the target road), which the guesses
        // above cannot always predict — a right turn from a 1-lane road onto a 2-lane road
        // is keyed (0 -> 1), not (0 -> 0). Before discarding the planned path and
        // rerouting, accept ANY arc from this lane to the planned next edge; the arc
        // carries the correct exit lane itself.
        if (arcIdx < 0)
        {
            var toNext = arcCache.GetReachableFromLane(edgeIdx, inLane);
            if (toNext != null)
            {
                foreach (var (outEdge, _, aIdx) in toNext)
                {
                    if (outEdge == nextEdge)
                    {
                        arcIdx = aIdx;
                        break;
                    }
                }
            }
        }

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
            float pivotOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index, graph, nextEdge);
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
        float newLaneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index, graph, nextEdge);
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
        halfVehT = HalfLen(store, index) / edgeLength;
        // Same entry-stop-line clamp as Update(): a long vehicle on a short edge must not
        // get a stop target before the edge start (see the stopAtT comment there).
        stopAtT = MathF.Max(stopT - halfVehT, nextStartT);

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
        float laneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index, graph, edgeIdx);
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
        if (distFromEntry > Len(store, index) * 2f)
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

        // PD control + lateral correction (capped so heading control always dominates —
        // see LatCorrectionCapFraction for the convergence argument)
        float prevError = store.PrevHeadingError[index];
        float errorDerivative = (headingError - prevError) / dt;
        store.PrevHeadingError[index] = headingError;

        float sharpness = store.SteeringSharpness[index];
        float latCorrection = Klat * lateralError;
        float latCap = LatCorrectionCapFraction * Kp * sharpness;
        latCorrection = MathF.Max(-latCap, MathF.Min(latCap, latCorrection));
        float steer = ((Kp * sharpness) * headingError + (Kd * sharpness) * errorDerivative - latCorrection)
            * SpeedGainCompensation(store, index);
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
        SpatialGrid grid, StopLineCache stopLines, IntersectionArcCache arcCache, int edgeIdx, float edgeLength,
        float speed, float targetSpeed, float progress, float stopAtT,
        SignalState signal, SignalState yieldSignal)
    {
        float comfortDecel = store.BrakingComfort[index];
        float timeHeadway = MapAggressivenessToTimeHeadway(store.Aggressiveness[index]);
        bool breakerFront = BreakerFront(index);

        var (aheadDist, leaderSpeed, leaderIdx) = FindNearbyThreats(store, index, grid, graph);

        // Center-to-center leader distance → bumper-to-bumper gap: subtract BOTH
        // half-lengths (mine and the leader's actual type — a bus body reaches 6 m
        // back from its center, a motorcycle barely 1 m).
        float gap = aheadDist - HalfLen(store, index)
            - (leaderIdx >= 0 ? HalfLen(store, leaderIdx) : 0f);
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

        // Merge yield: don't drive into a merge another car is already crossing — brake to the stop line
        // and wait there, so the two don't collide in the single throat and get shoved off-lane into a
        // permanent overlap. Independent of stop-sign right-of-way (applies on exempt approaches too).
        // Skipped for a breaker front, which is being freed from an existing tangle and must not re-yield.
        if (!breakerFront && ApproachingOccupiedMerge(store, index, graph, arcCache, edgeIdx))
        {
            float distToStop = (stopAtT - progress) * edgeLength;
            if (distToStop > 0f && distToStop < gap)
            {
                gap = distToStop;
                deltaV = speed;
            }
        }

        SetLastLeader(index, leaderIdx);
        if (ApplyHardOverlapBrake(store, index, gap)) return;

        float maxAccel = EffectiveMaxAccel(store, index);
        float idmAccel = ComputeIdmAcceleration(speed, targetSpeed, gap, deltaV, timeHeadway, comfortDecel, maxAccel);
        var (idmThrottle, idmBrake) = MapIdmToControls(idmAccel, maxAccel);

        store.Brake[index] = idmBrake;
        store.Throttle[index] = MathF.Max(0f, idmThrottle);
    }

    /// <summary>Records the threat-scan leader for this tick (see <see cref="_lastLeader"/>).</summary>
    private static void SetLastLeader(int index, int leaderIdx)
    {
        if (_lastLeader.Length <= index) Array.Resize(ref _lastLeader, index + 64);
        _lastLeader[index] = leaderIdx;
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
        float myHalfLen = HalfLen(store, index);
        if (distToEnd < distToProj || distToEnd < myHalfLen)
            remainingDist = 0f; // force arc completion

        LogDiag(store, index, $"TICK_ARC proj={arcProgress:F4} distEnd={distToEnd:F2} distProj={distToProj:F2}");

        // Check for arc completion (distance-based, at half the vehicle's own body length)
        if (remainingDist < myHalfLen)
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
                float newLaneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index, graph, nextEdge);
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
                        if (progDist < (myHalfLen + HalfLen(store, j)) * 2f)
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
        float steer = ((Kp * arcSharpness) * headingError + (Kd * arcSharpness) * errorDerivative)
            * SpeedGainCompensation(store, index);
        steer = MathF.Max(-MaxSteer, MathF.Min(MaxSteer, steer));
        store.SteeringAngle[index] = steer;

        // IDM car-following (world-space, works regardless of edge/arc)
        float arcBaseSpeed = arc.SpeedLimit > 0f ? arc.SpeedLimit : TargetSpeed;
        float targetSpeed = arcBaseSpeed * store.SpeedBias[index];
        var (aheadDist, leaderSpeed, leaderIndex) = FindVehicleAhead(store, index, grid, graph);

        // Same-arc look-ahead: if another vehicle is ahead on this arc, use path distance.
        // The occupancy index holds exactly the vehicles whose CurrentArc == arcIdx, so this is
        // an exact replacement for the former full-vehicle scan.
        int sameArcCount = _arcOccupancy.OccupantCount(arcIdx);
        for (int k = 0; k < sameArcCount; k++)
        {
            int j = _arcOccupancy.OccupantAt(arcIdx, k);
            if (j == index || store.State[j] != VehicleState.Driving) continue;
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

                // The binding leader on the outgoing edge is near its start (small edgeDist), so
                // query the grid around the arc exit point (P3) rather than scanning every vehicle.
                // Re-apply the exact original filters; a leader beyond the radius would have a gap
                // too large to affect IDM, so the result is behaviorally equivalent.
                var arcExit = arc.P3;
                var arcLeaders = _arcLeaderBuffer ??= new List<int>();
                arcLeaders.Clear();
                grid.QueryFiltered(arcExit.X, arcExit.Y, ArcLeaderSearchRadius, store.PosX, store.PosY, arcLeaders);
                for (int bi = 0; bi < arcLeaders.Count; bi++)
                {
                    int j = arcLeaders[bi];
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

        // Bumper-to-bumper gap from center-to-center distance (both half-lengths, per type)
        float gap = aheadDist - myHalfLen
            - (leaderIndex >= 0 ? HalfLen(store, leaderIndex) : 0f);
        float deltaV = speed - leaderSpeed;

        SetLastLeader(index, leaderIndex);
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
        float arcMaxAccel = EffectiveMaxAccel(store, index);
        float idmAccel = ComputeIdmAcceleration(speed, targetSpeed, gap, deltaV, arcTimeHeadway, arcComfortDecel, arcMaxAccel);
        var (idmThrottle, idmBrake) = MapIdmToControls(idmAccel, arcMaxAccel);

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

    /// <summary>
    /// Effective IDM maximum acceleration (the 'a' parameter, m/s^2) for a vehicle: the
    /// driver's desired launch rate — mapped from Aggressiveness onto 1.4–3.2 m/s^2, matching
    /// field studies of drivers pulling away from stops (typical 1.5–2.5, aggressive ~3) —
    /// capped by the vehicle type's full-throttle capability (e.g. a loaded truck cannot
    /// launch like a sedan no matter who drives it). Used both by the IDM controller and by
    /// <see cref="VehiclePhysics"/> to convert the throttle input back to acceleration, so
    /// the two must never diverge.
    /// </summary>
    public static float EffectiveMaxAccel(VehicleStore store, int index)
    {
        float desired = DriverAccelBase + store.Aggressiveness[index] * DriverAccelAggrRange;
        float capability = VehicleTypeDynamics.GetMaxAccel((VehicleType)store.PreferredVehicle[index]);
        return MathF.Min(desired, capability);
    }

    /// <param name="desiredSpeed">Target speed in m/s.</param>
    /// <param name="gap">Bumper-to-bumper gap to leader in meters.</param>
    /// <param name="deltaV">Speed difference (positive = closing on leader).</param>
    /// <param name="timeHeadway">Desired time headway in seconds (IDM T parameter).</param>
    /// <param name="comfortDecel">Comfortable deceleration in m/s^2 (IDM b parameter).</param>
    /// <param name="maxAccel">Maximum acceleration in m/s^2 (IDM a parameter) — the vehicle's
    /// <see cref="EffectiveMaxAccel"/>, per-driver and per-type.</param>
    /// <returns>Acceleration in m/s^2 (positive = accelerate, negative = brake).</returns>
    private static float ComputeIdmAcceleration(float speed, float desiredSpeed, float gap, float deltaV,
        float timeHeadway, float comfortDecel, float maxAccel)
    {
        // Free-road term: (v/v0)^4
        float vRatio = desiredSpeed > 0.01f ? speed / desiredSpeed : 0f;
        float vRatio2 = vRatio * vRatio;
        float freeRoadTerm = vRatio2 * vRatio2;

        // Desired dynamic gap: s* = s0 + max(0, v*T + v*Δv / (2*sqrt(a*b)))
        float interaction = speed * deltaV / (2f * MathF.Sqrt(maxAccel * comfortDecel));
        float sStar = IdmMinGap + MathF.Max(0f, speed * timeHeadway + interaction);

        // Gap term: (s*/gap)^2
        float gapTerm = gap > 0.01f ? (sStar / gap) * (sStar / gap) : 100f;

        return Math.Clamp(maxAccel * (1f - freeRoadTerm - gapTerm), -MaxBrakeDecel, maxAccel);
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
        float laneOffset = LaneChangeLogic.ComputeCurrentLaneOffset(store, index, graph, edgeIdx);

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
    /// Maps IDM acceleration to throttle/brake control inputs. Throttle is normalized by the
    /// vehicle's <paramref name="maxAccel"/> — the same value <see cref="VehiclePhysics"/>
    /// multiplies back in — so the round trip preserves the IDM acceleration exactly.
    /// </summary>
    private static (float throttle, float brake) MapIdmToControls(float idmAccel, float maxAccel)
    {
        if (idmAccel >= 0f)
            return (MathF.Min(1f, idmAccel / maxAccel), 0f);
        return (0f, MathF.Min(1f, -idmAccel / MaxBrakeDecel));
    }

    /// <summary>
    /// Combined nearby vehicle scan: performs a single spatial query and computes the
    /// lane-aware leader — its center-to-center ahead distance, speed, and vehicle index
    /// (-1 when nothing is ahead). Callers convert the distance to a bumper-to-bumper gap
    /// by subtracting both vehicles' half-lengths, using the index for the leader's type.
    /// </summary>
    private static (float aheadDist, float leaderSpeed, int leaderIdx) FindNearbyThreats(
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
                    return (float.MaxValue, 0f, -1);
            }
        }

        var nearby = _nearbyBuffer ??= new List<int>();
        nearby.Clear();
        grid.QueryFiltered(vx, vy, CollisionSearchRadius, store.PosX, store.PosY, nearby);

        float minAheadDist = float.MaxValue;
        float leaderSpeed = 0f;
        int leaderIdx = -1;

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
                    float myCurrentOff = GeometryUtil.LaneLateralOffset(graph, myEdge, store.CurrentLane[index]);
                    float myTargetOff = GeometryUtil.LaneLateralOffset(graph, myEdge, store.TargetLane[index]);
                    float otherCurrentOff = GeometryUtil.LaneLateralOffset(graph, myEdge, store.CurrentLane[other]);
                    float otherTargetOff = GeometryUtil.LaneLateralOffset(graph, myEdge, store.TargetLane[other]);
                    float threshold = LaneWidth * 0.7f;
                    bool inLane = MathF.Abs(myCurrentOff - otherCurrentOff) < threshold ||
                                  MathF.Abs(myCurrentOff - otherTargetOff) < threshold ||
                                  MathF.Abs(myTargetOff - otherCurrentOff) < threshold ||
                                  MathF.Abs(myTargetOff - otherTargetOff) < threshold;
                    if (inLane && pathDist < minAheadDist)
                    {
                        minAheadDist = pathDist;
                        leaderSpeed = store.Speed[other];
                        leaderIdx = other;
                    }
                }
            }
            // Cross-edge / arc vehicles: use road-tangent-based projection. A leader must be
            // within a ~45° cone of the nose in the near field (forward >= |lateral|), not just
            // inside the lateral corridor: where two roads/arcs meet at an angle, a car BEHIND
            // in path order (waiting at the feeding edge's line, or on a parallel same-turn arc)
            // projects marginally "ahead" (forward < 1 m) at lateral ~2.3 m — accepting it as a
            // leader hard-brakes both cars against each other in a permanent mutual wedge that
            // seeded map-wide gridlocks. Beyond ~2.45 m forward the cone is wider than the
            // corridor, so far-field behavior is unchanged; genuine leaders (dead ahead on the
            // next edge) and crossed post-merge pairs (lateral < forward) still match.
            else if (forward > 0f)
            {
                bool diverging = headingDot < 0.85f;
                if (!diverging)
                {
                    bool inLane = MathF.Abs(lateral) <= LaneWidth * 0.7f
                        && forward >= MathF.Abs(lateral);
                    if (inLane && forward < minAheadDist)
                    {
                        minAheadDist = forward;
                        leaderSpeed = store.Speed[other];
                        leaderIdx = other;
                    }
                }
            }

        }

        return (minAheadDist, leaderSpeed, leaderIdx);
    }

    /// <summary>
    /// Finds the center distance, speed, and index of the nearest vehicle ahead in the
    /// forward cone. Delegates to <see cref="FindNearbyThreats"/>.
    /// </summary>
    private static (float distance, float leaderSpeed, int leaderIdx) FindVehicleAhead(VehicleStore store, int index, SpatialGrid grid, RoadGraph graph)
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

        // Per-pass on-edge occupancy (for the shared-lane entry gate).
        RebuildEdgeOccupancy(store, graph.Edges.Count);

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

        // Break any merge/overlap deadlock that survived the per-vehicle pass (after all controls are set,
        // so the breaker's creep override is the final word before physics integrates this tick).
        RunDeadlockBreaker(store, grid, graph, arcCache);

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
    /// <summary>
    /// Finds the parametric t on the Bézier closest to (px, py), searching in a window near currentT.
    /// </summary>
    private static float FindNearestT(RoadGraph graph, int edgeIdx, float px, float py, float currentT)
    {
        // Cache the four control points once (instead of re-fetching the edge + both node structs
        // 21x via EvaluateBezier), then run the window search via the shared Horner evaluator.
        var edge = graph.Edges[edgeIdx];
        var p0 = graph.Nodes[edge.FromNode].Position;
        var p3 = graph.Nodes[edge.ToNode].Position;
        return NearestTOnBezier(p0, edge.ControlPoint1, edge.ControlPoint2, p3, px, py, currentT, 0.05f, 0.15f);
    }

    /// <summary>
    /// Window nearest-point search on a cubic Bézier, evaluated via cached monomial coefficients
    /// (Horner form) — no delegate, no per-step control-point fetch. Searches [t-back, t+fwd]
    /// clamped to [0,1] in 21 steps; returns the t with the closest curve point to (px, py).
    /// </summary>
    private static float NearestTOnBezier(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3,
        float px, float py, float currentT, float windowBack, float windowForward)
    {
        // B(t) = a + b t + c t^2 + d t^3
        Vector2 a = p0;
        Vector2 b = 3f * (p1 - p0);
        Vector2 c = 3f * (p0 - 2f * p1 + p2);
        Vector2 d = p3 - p0 + 3f * (p1 - p2);

        float searchMin = MathF.Max(0f, currentT - windowBack);
        float searchMax = MathF.Min(1f, currentT + windowForward);
        const int steps = 20;
        float bestT = currentT, bestDist = float.MaxValue;
        for (int i = 0; i <= steps; i++)
        {
            float t = searchMin + (searchMax - searchMin) * i / steps;
            float bx = a.X + t * (b.X + t * (c.X + t * d.X));
            float by = a.Y + t * (b.Y + t * (c.Y + t * d.Y));
            float dx = bx - px, dy = by - py;
            float dist = dx * dx + dy * dy;
            if (dist < bestDist) { bestDist = dist; bestT = t; }
        }
        return bestT;
    }

    /// <summary>
    /// Finds the parametric t on an intersection arc Bezier closest to (px, py).
    /// Uses a wider search window than edges since arcs are short and vehicle may lag behind.
    /// </summary>
    private static float FindNearestTOnArc(IntersectionArcCache arcCache, int arcIdx, float px, float py, float currentT)
    {
        // Cache the arc's control points once, then search via the shared Horner evaluator. Wider
        // window than edges since arcs are short and the vehicle may lag behind.
        var arc = arcCache.GetArc(arcIdx);
        return NearestTOnBezier(arc.P0, arc.P1, arc.P2, arc.P3, px, py, currentT, 0.1f, 0.25f);
    }

    /// <summary>
    /// Speed-adaptive gain compensation for the PD steering command. The bicycle model
    /// yaws at speed/wheelbase rad/s per radian of steer, so the steering loop's plant
    /// gain grows linearly with speed (and driver sharpness, and inversely with
    /// wheelbase) while Kp/Kd stay fixed. Discretized at the 30 Hz sim tick, the loop
    /// crosses into a barely-damped ~15 Hz steering flip-flop once that gain reaches ≈8
    /// (a default sedan at ≈20 m/s = 45 mph; short-wheelbase motorcycles and
    /// sharp-steering drivers hit it at city speeds) — the derivative term dominates the
    /// crossing because its discrete gain is Kd/dt, as large as Kp itself. Scaling the
    /// WHOLE command by MaxYawGain/gain once the gain exceeds <see cref="MaxYawGain"/>
    /// makes the closed-loop dynamics speed-, wheelbase-, and sharpness-invariant above
    /// the ceiling (well-damped by construction) and leaves handling below it untouched.
    /// Both the edge and arc steering paths apply it just before the MaxSteer clamp.
    /// </summary>
    private static float SpeedGainCompensation(VehicleStore store, int index)
    {
        float wheelbase = VehicleTypeDimensions.GetWheelbase(store.PreferredVehicle[index]);
        float gain = store.Speed[index] / wheelbase * store.SteeringSharpness[index];
        return gain > MaxYawGain ? MaxYawGain / gain : 1f;
    }

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
