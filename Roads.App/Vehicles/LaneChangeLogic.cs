using System.Numerics;
using Roads.App;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Lane change decision logic. Evaluates whether vehicles should change lanes for
/// turn preparation (move to the correct lane before an upcoming turn) or congestion
/// avoidance (pass slow traffic). Performs gap safety checks before initiating changes
/// and interpolates lateral position during the transition.
/// </summary>
public static class LaneChangeLogic
{
    /// <summary>Duration in seconds for a full lane change maneuver.</summary>
    private const float LaneChangeDuration = 2.0f;
    /// <summary>Lane width in meters.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;
    /// <summary>Cooldown in seconds after completing a turn-prep lane change.</summary>
    private const float CooldownAfterChange = 3.0f;
    /// <summary>Cooldown in seconds after completing a congestion lane change (longer to prevent oscillation).</summary>
    private const float CongestionCooldownAfterChange = 6.0f;
    /// <summary>Base distance in meters ahead at which turn preparation lane changes begin.</summary>
    private const float TurnPrepBaseDistance = 150f;
    /// <summary>Additional prep distance in meters per lane that must be crossed.</summary>
    private const float TurnPrepPerLane = 60f;
    /// <summary>Maximum urgency reduction factor for gap thresholds (40% smaller at max urgency).</summary>
    private const float MaxUrgencyGapReduction = 0.4f;
    /// <summary>Absolute minimum gap ahead in meters, even at maximum merge urgency.</summary>
    private const float AbsoluteMinGapAhead = 4f;
    /// <summary>Absolute minimum gap behind in meters, even at maximum merge urgency.</summary>
    private const float AbsoluteMinGapBehind = 3f;
    /// <summary>Minimum distance component of the safe-gap threshold for lane changes (meters).</summary>
    private const float SafeGapMinDistance = 2.0f;
    /// <summary>Time-based component of the safe-gap threshold for lane changes (seconds).</summary>
    private const float SafeGapTimeBuffer = 1.0f;
    /// <summary>Vehicle body length in meters (for gap calculations).</summary>
    private const float VehicleLength = SimConstants.VehicleLength;
    /// <summary>Search radius in meters for nearby vehicle queries during gap checks.</summary>
    private const float CollisionSearchRadius = 40f;
    /// <summary>If speed drops below this fraction of the speed limit, consider a passing lane change.</summary>
    private const float CongestionSpeedRatio = 0.6f;

    /// <summary>Thread-local buffer for spatial grid query results.</summary>
    [ThreadStatic] private static List<int>? _nearbyBuffer;

    /// <summary>
    /// Updates lane change state for all active vehicles: ticks cooldowns,
    /// advances in-progress changes, and evaluates new lane change decisions.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="graph">Road graph for edge lane counts and geometry.</param>
    /// <param name="grid">Spatial grid for gap safety checks.</param>
    /// <param name="dt">Delta time in seconds.</param>
    public static void UpdateAll(VehicleStore store, RoadGraph graph, SpatialGrid grid, IntersectionArcCache arcCache, StopLineCache stopLines, float dt)
    {
        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            if (store.CurrentArc[i] >= 0) continue; // no lane changes on intersection arcs

            int edgeIdx = store.CurrentEdge[i];
            if (edgeIdx < 0 || edgeIdx >= graph.Edges.Count) continue;
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;

            // Tick cooldown
            if (store.LaneChangeCooldown[i] > 0f)
                store.LaneChangeCooldown[i] -= dt;

            // Single-lane road: nothing to do
            if (edge.LaneCount <= 1)
            {
                store.CurrentLane[i] = 0;
                store.TargetLane[i] = 0;
                store.LaneChangeProgress[i] = 0f;
                continue;
            }

            // Clamp lane to valid range (edge may have changed lane count)
            byte maxLane = (byte)(edge.LaneCount - 1);
            if (store.CurrentLane[i] > maxLane)
            {
                store.CurrentLane[i] = maxLane;
                store.TargetLane[i] = maxLane;
                store.LaneChangeProgress[i] = 0f;
            }

            // If currently executing a lane change, advance progress
            if (store.CurrentLane[i] != store.TargetLane[i])
            {
                store.LaneChangeProgress[i] += dt / LaneChangeDuration;
                if (store.LaneChangeProgress[i] >= 1f)
                {
                    // Complete the lane change
                    store.CurrentLane[i] = store.TargetLane[i];
                    store.LaneChangeProgress[i] = 0f;
                    store.LaneChangeCooldown[i] = store.MergeUrgency[i] > 0f
                        ? CooldownAfterChange
                        : CongestionCooldownAfterChange;
                }
                continue; // don't evaluate new changes during an active one
            }

            // Don't evaluate if on cooldown
            if (store.LaneChangeCooldown[i] > 0f) continue;

            // Evaluate lane change decisions (also computes and stores MergeUrgency)
            byte desiredLane = EvaluateDesiredLane(store, i, graph, grid);
            store.DesiredLane[i] = desiredLane;

            if (desiredLane != store.CurrentLane[i])
            {
                // Don't attempt lane changes when nearly stopped (queued at a light).
                // Vehicles creeping in a queue can't safely merge without overlap.
                float speed = store.Speed[i];
                if (speed < 1.5f)
                {
                    store.LaneChangeCooldown[i] = 0.5f;
                    continue;
                }

                // Move one lane at a time toward desired lane
                byte targetLane = desiredLane > store.CurrentLane[i]
                    ? (byte)(store.CurrentLane[i] + 1)
                    : (byte)(store.CurrentLane[i] - 1);

                // Only apply urgency gap reduction when vehicle has enough speed for
                // a safe merge. At low speeds (in a queue), use full gap thresholds.
                float urgency = store.MergeUrgency[i];
                float speedRatio = Math.Clamp((speed - 1.5f) / 5f, 0f, 1f);
                float effectiveUrgency = urgency * speedRatio;

                // Check gap safety before initiating (urgency relaxes thresholds)
                if (IsLaneChangeSafe(store, i, targetLane, graph, grid, arcCache, stopLines, effectiveUrgency))
                {
                    store.TargetLane[i] = targetLane;
                    store.LaneChangeProgress[i] = 0f;
                }
                else
                {
                    // Gap not safe — cooldown scales with urgency (high urgency = retry faster)
                    store.LaneChangeCooldown[i] = MathF.Max(0.2f, 1.0f - urgency * 0.7f);
                }
            }
            else
            {
                // No lane change needed — clear merge urgency and speed bias
                store.MergeUrgency[i] = 0f;
                store.MergeSpeedBias[i] = 0f;
            }
        }
    }

    /// <summary>
    /// Determines the ideal lane based on upcoming turn preparation and congestion avoidance.
    /// Turn preparation takes priority over congestion. Congestion merges are gated by
    /// lane density comparison to avoid oscillation between equally-congested lanes.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Index of the vehicle.</param>
    /// <param name="graph">Road graph for path and geometry lookups.</param>
    /// <param name="grid">Spatial grid for lane density queries.</param>
    /// <returns>Desired lane index.</returns>
    private static byte EvaluateDesiredLane(VehicleStore store, int index, RoadGraph graph, SpatialGrid grid)
    {
        int edgeIdx = store.CurrentEdge[index];
        var edge = graph.Edges[edgeIdx];
        byte currentLane = store.CurrentLane[index];
        byte maxLane = (byte)(edge.LaneCount - 1);

        // 1. Turn preparation: check if upcoming turn needs a specific lane
        byte turnLane = GetTurnPreparationLane(store, index, graph, out float urgency);
        store.MergeUrgency[index] = urgency;
        if (turnLane != byte.MaxValue)
            return Math.Clamp(turnLane, (byte)0, maxLane);

        // 2. Congestion-based: if stuck behind slow traffic, try adjacent lane
        // (only if no turn prep needed AND target lane is measurably better)
        float speed = store.Speed[index];
        float desiredSpeed = edge.SpeedLimit > 0f ? edge.SpeedLimit : 13.4f;
        if (speed < desiredSpeed * CongestionSpeedRatio && speed > 0.5f)
        {
            // Prefer passing lane (lower index), but verify it's actually better
            if (currentLane > 0 && IsLaneBetter(store, index, (byte)(currentLane - 1), grid))
                return (byte)(currentLane - 1);
            if (currentLane < maxLane && IsLaneBetter(store, index, (byte)(currentLane + 1), grid))
                return (byte)(currentLane + 1);
        }

        return currentLane; // stay in current lane
    }

    /// <summary>
    /// Compares traffic conditions between the vehicle's current lane and a candidate lane
    /// on the same edge. Returns true if the candidate lane has meaningfully higher average
    /// speed or lower vehicle density, preventing oscillation between equally-congested lanes.
    /// </summary>
    private static bool IsLaneBetter(VehicleStore store, int index, byte candidateLane, SpatialGrid grid)
    {
        float vx = store.PosX[index];
        float vy = store.PosY[index];
        int myEdge = store.CurrentEdge[index];
        byte currentLane = store.CurrentLane[index];

        var nearby = _nearbyBuffer ??= new List<int>();
        nearby.Clear();
        grid.QueryFiltered(vx, vy, CollisionSearchRadius, store.PosX, store.PosY, nearby);

        float currentLaneSpeedSum = 0f;
        int currentLaneCount = 0;
        float candidateLaneSpeedSum = 0f;
        int candidateLaneCount = 0;

        foreach (int other in nearby)
        {
            if (other == index) continue;
            if (store.State[other] != VehicleState.Driving) continue;
            if (store.CurrentEdge[other] != myEdge) continue;

            byte otherLane = store.CurrentLane[other];
            if (otherLane == currentLane)
            {
                currentLaneSpeedSum += store.Speed[other];
                currentLaneCount++;
            }
            else if (otherLane == candidateLane)
            {
                candidateLaneSpeedSum += store.Speed[other];
                candidateLaneCount++;
            }
        }

        // If candidate lane has no vehicles nearby, it's clearly better
        if (candidateLaneCount == 0 && currentLaneCount > 0)
            return true;

        // If both are empty, no benefit to changing
        if (candidateLaneCount == 0 && currentLaneCount == 0)
            return false;

        // Compare average speeds — candidate must be meaningfully better (>20% faster)
        float currentAvg = currentLaneCount > 0
            ? currentLaneSpeedSum / currentLaneCount
            : store.Speed[index];
        float candidateAvg = candidateLaneCount > 0
            ? candidateLaneSpeedSum / candidateLaneCount
            : float.MaxValue;

        return candidateAvg > currentAvg * 1.2f
            || candidateLaneCount < currentLaneCount - 1;
    }

    /// <summary>
    /// Looks ahead in the vehicle's path (scanning across multiple edges) to determine if
    /// a specific lane is needed for an upcoming turn. Uses explicit lane restrictions when
    /// available, otherwise falls back to cross-product geometry heuristic.
    /// Also computes merge urgency based on distance remaining and lanes to cross.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Index of the vehicle.</param>
    /// <param name="graph">Road graph for tangent evaluation.</param>
    /// <param name="urgency">Output merge urgency (0 = no urgency, 1 = critical). Set to 0 when no turn prep needed.</param>
    /// <returns>Required lane index, or <c>byte.MaxValue</c> if no turn preparation is needed.</returns>
    private static byte GetTurnPreparationLane(VehicleStore store, int index, RoadGraph graph, out float urgency)
    {
        urgency = 0f;

        var path = store.Path[index];
        int pathIdx = store.PathIndex[index];
        if (path == null || pathIdx + 1 >= path.Count) return byte.MaxValue;

        int currentEdgeIdx = store.CurrentEdge[index];
        var currentEdge = graph.Edges[currentEdgeIdx];
        if (currentEdge.FromNode < 0) return byte.MaxValue;

        float progress = store.EdgeProgress[index];
        float edgeLength = currentEdge.Length;
        if (edgeLength < 0.01f) edgeLength = 0.01f;

        byte currentLane = store.CurrentLane[index];
        byte maxLane = (byte)(currentEdge.LaneCount - 1);

        // Maximum prep distance for scanning (use max lanes for upper bound)
        float maxPrepDistance = TurnPrepBaseDistance + TurnPrepPerLane * maxLane;

        // Accumulate distance from current position, scanning ahead through path edges
        float accumulatedDist = (1f - progress) * edgeLength;

        // Scan forward through path to find the first upcoming turn or restriction
        for (int scan = pathIdx; scan + 1 < path.Count; scan++)
        {
            int scanEdgeIdx = path[scan];
            int nextEdgeIdx = path[scan + 1];
            if (scanEdgeIdx < 0 || scanEdgeIdx >= graph.Edges.Count) break;
            if (nextEdgeIdx < 0 || nextEdgeIdx >= graph.Edges.Count) break;

            var scanEdge = graph.Edges[scanEdgeIdx];
            var nextEdge = graph.Edges[nextEdgeIdx];
            if (scanEdge.FromNode < 0 || nextEdge.FromNode < 0) break;

            // Only check lane restrictions / turn direction on the CURRENT edge
            // (the one the vehicle is actually on), since that's where lane changes happen
            if (scan == pathIdx)
            {
                byte requiredLane = byte.MaxValue;

                // Check explicit lane restrictions on current edge
                if (graph.HasLaneRestrictions(currentEdgeIdx))
                {
                    byte bestLane = byte.MaxValue;
                    int bestDist = int.MaxValue;
                    for (byte lane = 0; lane <= maxLane; lane++)
                    {
                        var restrictions = graph.GetLaneRestrictions(currentEdgeIdx, lane);
                        if (restrictions == null)
                        {
                            continue;
                        }
                        else
                        {
                            foreach (var (outEdge, _) in restrictions)
                            {
                                if (outEdge == nextEdgeIdx)
                                {
                                    int dist = Math.Abs(lane - currentLane);
                                    if (dist < bestDist) { bestDist = dist; bestLane = lane; }
                                    break;
                                }
                            }
                        }
                    }
                    requiredLane = bestLane;
                }
                else
                {
                    // No restrictions — use geometry-based heuristic
                    requiredLane = ClassifyTurnLane(graph, currentEdgeIdx, nextEdgeIdx, maxLane);
                }

                if (requiredLane != byte.MaxValue)
                {
                    int lanesToCross = Math.Abs(requiredLane - currentLane);
                    float prepDistance = TurnPrepBaseDistance + TurnPrepPerLane * lanesToCross;
                    if (accumulatedDist <= prepDistance)
                    {
                        urgency = 1f - Math.Clamp(accumulatedDist / prepDistance, 0f, 1f);
                        urgency = Math.Clamp(urgency * (1f + 0.3f * lanesToCross), 0f, 1f);
                        return requiredLane;
                    }
                }
            }
            else
            {
                // For future edges, just check if a turn exists (geometry-based).
                // If a turn is detected within lookahead range, the vehicle needs
                // to be in the correct lane on the CURRENT edge to handle it.
                byte turnLane = ClassifyTurnLane(graph, scanEdgeIdx, nextEdgeIdx, maxLane);
                if (turnLane != byte.MaxValue)
                {
                    // A turn is coming on a future edge — get into the right lane now.
                    // Use same lane index (left=0, right=maxLane) for current edge.
                    byte requiredLane = Math.Min(turnLane, maxLane);

                    // Also check explicit restrictions on current edge if available
                    if (graph.HasLaneRestrictions(currentEdgeIdx))
                    {
                        // Restrictions already encode which lane can reach which exit,
                        // but for multi-hop they may not directly apply.
                        // Still prefer the geometry-based direction as a guide.
                    }

                    int lanesToCross = Math.Abs(requiredLane - currentLane);
                    float prepDistance = TurnPrepBaseDistance + TurnPrepPerLane * lanesToCross;
                    if (accumulatedDist <= prepDistance)
                    {
                        urgency = 1f - Math.Clamp(accumulatedDist / prepDistance, 0f, 1f);
                        urgency = Math.Clamp(urgency * (1f + 0.3f * lanesToCross), 0f, 1f);
                        return requiredLane;
                    }
                }
            }

            // Stop scanning if we've looked far enough ahead
            if (accumulatedDist > maxPrepDistance) break;

            // Add the next edge's length to accumulated distance
            accumulatedDist += nextEdge.Length;
        }

        return byte.MaxValue; // no upcoming turn within lookahead range
    }

    /// <summary>
    /// Classifies the turn direction from one edge to the next and returns the required
    /// lane index (0 for left, maxLane for right), or byte.MaxValue for straight.
    /// Delegates to <see cref="GeometryUtil.ClassifyTurn"/> for the direction classification.
    /// </summary>
    private static byte ClassifyTurnLane(RoadGraph graph, int fromEdgeIdx, int toEdgeIdx, byte maxLane)
    {
        var turn = GeometryUtil.ClassifyTurn(graph, fromEdgeIdx, toEdgeIdx);
        return turn switch
        {
            GeometryUtil.TurnDirection.Right => maxLane,
            GeometryUtil.TurnDirection.Left => 0,
            _ => byte.MaxValue,
        };
    }

    /// <summary>
    /// Scans nearby vehicles to find the nearest ahead and behind in a target lane.
    /// Shared by IsLaneChangeSafe and ApplyMergeSpeedBias.
    /// </summary>
    private static (float nearestAhead, float nearestBehind) ScanTargetLaneGaps(
        VehicleStore store, int index, byte targetLane, RoadGraph graph, SpatialGrid grid,
        IntersectionArcCache? arcCache = null, StopLineCache? stopLines = null)
    {
        float vx = store.PosX[index];
        float vy = store.PosY[index];
        float cosH = MathF.Cos(store.Heading[index]);
        float sinH = MathF.Sin(store.Heading[index]);
        int myEdge = store.CurrentEdge[index];

        // Use road tangent for forward projection (immune to heading lag on curves),
        // consistent with FindNearbyThreats in SteeringController.
        float fwdCos = cosH, fwdSin = sinH;
        float myProgress = store.EdgeProgress[index];
        float edgeLength = 0f;
        if (myEdge >= 0)
        {
            edgeLength = MathF.Max(graph.Edges[myEdge].Length, 0.01f);
            var tangent = graph.EvaluateBezierTangent(myEdge, myProgress);
            float tLen = tangent.Length();
            if (tLen > 0.001f)
            {
                fwdCos = tangent.X / tLen;
                fwdSin = tangent.Y / tLen;
            }
        }

        var nearby = _nearbyBuffer ??= new List<int>();
        nearby.Clear();
        grid.QueryFiltered(vx, vy, CollisionSearchRadius, store.PosX, store.PosY, nearby);

        float targetLaneOffset = LaneWidth * (0.5f + targetLane);
        float myLaneOffset = ComputeCurrentLaneOffset(store, index);

        float nearestAhead = float.MaxValue;
        float nearestBehind = float.MaxValue;

        foreach (int other in nearby)
        {
            if (other == index) continue;
            if (store.State[other] != VehicleState.Driving) continue;

            float dx = store.PosX[other] - vx;
            float dy = store.PosY[other] - vy;

            bool inTargetLane;
            float dist;

            // Same-edge vehicles: use path distance along edge
            if (store.CurrentEdge[other] == myEdge && store.CurrentArc[other] < 0)
            {
                dist = (store.EdgeProgress[other] - myProgress) * edgeLength;
                float otherCurrentOff = LaneWidth * (0.5f + store.CurrentLane[other]);
                float otherTargetOff = LaneWidth * (0.5f + store.TargetLane[other]);
                float threshold = LaneWidth * 0.7f;
                inTargetLane = MathF.Abs(targetLaneOffset - otherCurrentOff) < threshold
                            || MathF.Abs(targetLaneOffset - otherTargetOff) < threshold;
            }
            // Cross-edge / arc vehicles: use road-tangent-based Euclidean projection
            else
            {
                dist = dx * fwdCos + dy * fwdSin;
                float lateral = -dx * fwdSin + dy * fwdCos;
                float expectedLateral = targetLaneOffset - myLaneOffset;
                inTargetLane = MathF.Abs(lateral - expectedLateral) < LaneWidth * 0.7f;
            }

            if (!inTargetLane) continue;

            if (dist > 0f && dist < nearestAhead)
                nearestAhead = dist;
            else if (dist <= 0f && -dist < nearestBehind)
                nearestBehind = -dist;
        }

        // Check vehicles on arcs that will exit onto our edge in the target lane.
        // They count as approaching from behind (they'll arrive at the edge start).
        if (arcCache != null && stopLines != null)
        {
            float fromStopT = stopLines.GetStopTAtFromNode(myEdge);
            float distFromStart = (myProgress - fromStopT) * edgeLength;

            for (int j = 0; j < store.Count; j++)
            {
                if (j == index || store.State[j] != VehicleState.Driving) continue;
                int otherArc = store.CurrentArc[j];
                if (otherArc < 0) continue;
                var otherArcData = arcCache.GetArc(otherArc);
                if (otherArcData.OutgoingEdge != myEdge) continue;
                if (otherArcData.OutgoingLane != targetLane) continue;

                float otherRemaining = (1f - store.ArcProgress[j]) * otherArcData.Length;
                float behindDist = distFromStart + otherRemaining;
                if (behindDist < nearestBehind)
                    nearestBehind = behindDist;
            }
        }

        return (nearestAhead, nearestBehind);
    }

    /// <summary>
    /// Checks whether changing to the target lane is safe by verifying sufficient gaps
    /// ahead and behind in the target lane using spatial grid queries.
    /// </summary>
    private static bool IsLaneChangeSafe(VehicleStore store, int index, byte targetLane, RoadGraph graph, SpatialGrid grid, IntersectionArcCache arcCache, StopLineCache stopLines, float urgency = 0f)
    {
        var (nearestAhead, nearestBehind) = ScanTargetLaneGaps(store, index, targetLane, graph, grid, arcCache, stopLines);

        float speed = store.Speed[index];
        float urgencyFactor = 1f - urgency * MaxUrgencyGapReduction;
        float safeGapAhead = (SafeGapMinDistance + speed * SafeGapTimeBuffer) * urgencyFactor;
        float safeGapBehind = (SafeGapMinDistance + speed * 0.8f) * urgencyFactor;

        safeGapAhead = MathF.Max(safeGapAhead, AbsoluteMinGapAhead);
        safeGapBehind = MathF.Max(safeGapBehind, AbsoluteMinGapBehind);

        return (nearestAhead - VehicleLength) >= safeGapAhead
            && (nearestBehind - VehicleLength) >= safeGapBehind;
    }

    /// <summary>
    /// Applies a speed bias to vehicles that want to change lanes for turn preparation but
    /// cannot find a gap. Scans the target lane for the nearest gap and biases the vehicle
    /// to speed up (gap ahead) or slow down (gap behind) to align with it.
    /// Must be called after <see cref="UpdateAll"/>.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="graph">Road graph for edge data.</param>
    /// <param name="grid">Spatial grid for nearby vehicle queries.</param>
    public static void ApplyMergeSpeedBias(VehicleStore store, RoadGraph graph, SpatialGrid grid, IntersectionArcCache arcCache, StopLineCache stopLines)
    {
        for (int i = 0; i < store.Count; i++)
        {
            // Only apply when: driving, on edge, wants a different lane, not mid-change, has urgency
            if (store.State[i] != VehicleState.Driving) continue;
            if (store.CurrentArc[i] >= 0) continue;
            if (store.MergeUrgency[i] < 0.1f) { store.MergeSpeedBias[i] = 0f; continue; }
            if (store.CurrentLane[i] != store.TargetLane[i]) { store.MergeSpeedBias[i] = 0f; continue; }

            int edgeIdx = store.CurrentEdge[i];
            if (edgeIdx < 0 || edgeIdx >= graph.Edges.Count) { store.MergeSpeedBias[i] = 0f; continue; }
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0 || edge.LaneCount <= 1) { store.MergeSpeedBias[i] = 0f; continue; }

            // Figure out which lane we want (one step toward desired)
            byte desiredLane = store.MergeUrgency[i] > 0f ? FindDesiredAdjacentLane(store, i) : store.CurrentLane[i];
            if (desiredLane == store.CurrentLane[i]) { store.MergeSpeedBias[i] = 0f; continue; }

            var (nearestAhead, nearestBehind) = ScanTargetLaneGaps(store, i, desiredLane, graph, grid, arcCache, stopLines);

            // Find the gap center and compute bias
            float urgency = store.MergeUrgency[i];
            float maxBias = 2f * urgency; // up to +/- 2 m/s at full urgency

            if (nearestAhead < float.MaxValue && nearestBehind < float.MaxValue)
            {
                // Only bias toward gap if it's large enough for the vehicle to fit
                float totalGap = nearestAhead + nearestBehind - VehicleLength;
                if (totalGap < VehicleLength * 0.5f)
                {
                    // Gap too small — don't waste effort centering in it
                    store.MergeSpeedBias[i] = 0f;
                }
                else
                {
                    // Gap exists between two vehicles — bias toward its center
                    float gapCenter = (nearestAhead - nearestBehind) * 0.5f;
                    if (gapCenter > 2f)
                        store.MergeSpeedBias[i] = Math.Clamp(gapCenter * 0.15f, 0f, maxBias);
                    else if (gapCenter < -2f)
                        store.MergeSpeedBias[i] = Math.Clamp(gapCenter * 0.15f, -maxBias, 0f);
                    else
                        store.MergeSpeedBias[i] = 0f;
                }
            }
            else if (nearestAhead < 20f && nearestBehind == float.MaxValue)
            {
                // Vehicle ahead blocking, nothing behind — slow down to let gap pass
                store.MergeSpeedBias[i] = -maxBias * 0.5f;
            }
            else if (nearestBehind < 20f && nearestAhead == float.MaxValue)
            {
                // Vehicle behind, open ahead — speed up to pull ahead of it
                store.MergeSpeedBias[i] = maxBias * 0.5f;
            }
            else
            {
                store.MergeSpeedBias[i] = 0f;
            }
        }
    }

    /// <summary>
    /// Returns the adjacent lane one step toward the vehicle's desired merge target.
    /// Uses <see cref="VehicleStore.DesiredLane"/> set during <see cref="UpdateAll"/>.
    /// </summary>
    private static byte FindDesiredAdjacentLane(VehicleStore store, int index)
    {
        byte desired = store.DesiredLane[index];
        byte current = store.CurrentLane[index];
        if (desired == byte.MaxValue || desired == current) return current;
        return desired > current ? (byte)(current + 1) : (byte)(current - 1);
    }

    /// <summary>
    /// Computes the current effective lateral offset in meters for a vehicle's lane position,
    /// smoothly interpolating during in-progress lane changes using smoothstep.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Index of the vehicle.</param>
    /// <returns>Rightward offset from the road edge in meters.</returns>
    public static float ComputeCurrentLaneOffset(VehicleStore store, int index)
    {
        byte currentLane = store.CurrentLane[index];
        byte targetLane = store.TargetLane[index];
        float progress = store.LaneChangeProgress[index];

        float currentOffset = LaneWidth * (0.5f + currentLane);
        if (currentLane == targetLane || progress <= 0f)
            return currentOffset;

        float targetOffset = LaneWidth * (0.5f + targetLane);
        // Smoothstep interpolation for natural motion
        float t = progress * progress * (3f - 2f * progress);
        return currentOffset + (targetOffset - currentOffset) * t;
    }
}
