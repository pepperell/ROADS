using System.Numerics;
using System.Text;
using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Diagnostics;

/// <summary>
/// Shared deadlock / stuck-vehicle diagnostics, used by BOTH the live in-app capture
/// (MainForm 'D' key) AND the headless --simtest harness, so the two stay in sync.
/// All state is passed in via <see cref="Deps"/> — the class holds no simulation state of
/// its own. The per-vehicle dump, nearest-vehicle-ahead probe and blocking-chain walk are
/// faithful ports of the original MainForm helpers (DumpVehicleDiag / NearestVehicleAhead /
/// BuildBlockingChain / VehiclesOnArc); MainForm now delegates to these so there is a single
/// implementation. The full node/cluster report is the automated equivalent of the manual
/// 'D' capture and additionally walks every jam cluster and emits a machine-readable summary.
/// </summary>
public static class DeadlockReport
{
    /// <summary>Speed (m/s) below which a vehicle counts as "stopped" for stuck tracking.</summary>
    public const float StuckSpeedThreshold = 0.1f;

    /// <summary>
    /// Read-only handle to all the simulation systems the diagnostics need. Bundled so the
    /// dump helpers take one parameter instead of a dozen, and so the live and headless callers
    /// build the exact same view of the world.
    /// </summary>
    public sealed class Deps
    {
        public required RoadGraph Graph;
        public required VehicleStore Vehicles;
        public required SpatialGrid VehicleGrid;
        public required StopLineCache StopLines;
        public required IntersectionArcCache Arcs;
        public required TrafficSignalSystem TrafficSignals;
        public required StopSignSystem StopSigns;
        public required YieldSignSystem YieldSigns;
    }

    // ── Geometry / neighbour probes (ports of MainForm helpers) ────────────

    /// <summary>
    /// Nearest other driving vehicle ahead of <paramref name="v"/> within a 30 m forward cone
    /// (±5 m lateral so cross-traffic at an intersection is caught). Returns its index and the
    /// bumper gap, or -1 if nothing is ahead (then the vehicle is held by a signal/stop, not a car).
    /// </summary>
    public static int NearestVehicleAhead(Deps d, int v, out float gap, out float lateral)
    {
        var veh = d.Vehicles;
        gap = float.MaxValue; lateral = 0f;
        float vx = veh.PosX[v], vy = veh.PosY[v];
        float fx, fy;
        int e = veh.CurrentEdge[v];
        if (veh.CurrentArc[v] < 0 && e >= 0 && e < d.Graph.Edges.Count && d.Graph.Edges[e].FromNode >= 0)
        {
            var t = d.Graph.EvaluateBezierTangent(e, veh.EdgeProgress[v]);
            float tl = t.Length();
            if (tl > 0.001f) { fx = t.X / tl; fy = t.Y / tl; }
            else { fx = MathF.Cos(veh.Heading[v]); fy = MathF.Sin(veh.Heading[v]); }
        }
        else { fx = MathF.Cos(veh.Heading[v]); fy = MathF.Sin(veh.Heading[v]); }

        var buf = new List<int>();
        d.VehicleGrid.QueryFiltered(vx, vy, 30f, veh.PosX, veh.PosY, buf);
        int best = -1; float bestFwd = float.MaxValue;
        foreach (int o in buf)
        {
            if (o == v || veh.State[o] != VehicleState.Driving) continue;
            float dx = veh.PosX[o] - vx, dy = veh.PosY[o] - vy;
            float fwd = dx * fx + dy * fy;
            if (fwd <= 0f) continue;
            float la = -dx * fy + dy * fx;
            if (MathF.Abs(la) > 5f) continue;
            if (fwd < bestFwd) { bestFwd = fwd; best = o; lateral = la; }
        }
        if (best >= 0)
            gap = bestFwd - VehicleTypeDimensions.GetHalfLength(veh.PreferredVehicle[v])
                - VehicleTypeDimensions.GetHalfLength(veh.PreferredVehicle[best]);
        return best;
    }

    /// <summary>Comma-separated indices of driving vehicles currently on the given arc, or "(empty)".</summary>
    public static string VehiclesOnArc(Deps d, int arcIdx)
    {
        var list = new List<int>();
        var veh = d.Vehicles;
        for (int o = 0; o < veh.Count; o++)
            if (veh.State[o] == VehicleState.Driving && veh.CurrentArc[o] == arcIdx)
                list.Add(o);
        return list.Count == 0 ? "(empty)" : string.Join(",", list);
    }

    /// <summary>
    /// Walks "nearest vehicle ahead" from <paramref name="v"/> to expose the head of the queue or a
    /// deadlock cycle. Stops at a vehicle with no car ahead (signal/stop-held), a moving non-blocker,
    /// or a revisited vehicle (a true deadlock loop, flagged explicitly).
    /// </summary>
    public static string BuildBlockingChain(Deps d, int v)
    {
        var sb = new StringBuilder();
        var seen = new HashSet<int>();
        int cur = v;
        for (int hop = 0; hop < 16; hop++)
        {
            if (!seen.Add(cur)) { sb.Append($"{cur} <== DEADLOCK CYCLE"); break; }
            sb.Append(cur);
            int next = NearestVehicleAhead(d, cur, out float g, out _);
            if (next < 0) { sb.Append(" -> [held by signal/stop or clear]"); break; }
            if (d.Vehicles.Speed[next] > 1.0f && g > 8f) { sb.Append($" -> {next} (moving, not blocking)"); break; }
            sb.Append($" -[{g:F1}m]-> ");
            cur = next;
        }
        return sb.ToString();
    }

    /// <summary>True if the blocking chain from <paramref name="v"/> closes into a cycle.</summary>
    public static bool ChainHasCycle(Deps d, int v)
    {
        var seen = new HashSet<int>();
        int cur = v;
        for (int hop = 0; hop < 16; hop++)
        {
            if (!seen.Add(cur)) return true;
            int next = NearestVehicleAhead(d, cur, out float g, out _);
            if (next < 0) return false;
            if (d.Vehicles.Speed[next] > 1.0f && g > 8f) return false;
            cur = next;
        }
        return false;
    }

    // ── Per-vehicle dump (port of MainForm.DumpVehicleDiag) ────────────────

    /// <summary>
    /// Appends a full per-vehicle diagnostic dump to <paramref name="sb"/>: pose/control,
    /// current edge + signals, current arc, lane state, path, intent/off-road, and the
    /// blocker analysis (stuck time, nearest ahead, stop-sign FCFS, intended turn + conflicting
    /// arc occupants, arcs at the approached node, and the blocking chain). Faithful port of the
    /// in-app dump so live and headless output match.
    /// </summary>
    public static void DumpVehicle(StringBuilder sb, Deps d, int i, int stuckSimTicks)
    {
        var veh = d.Vehicles;
        var graph = d.Graph;

        sb.AppendLine($"=== Vehicle {i} dump ===");
        sb.AppendLine($"  Pos: ({veh.PosX[i]:F2}, {veh.PosY[i]:F2})");
        sb.AppendLine($"  Heading: {veh.Heading[i] * 180f / MathF.PI:F1} deg");
        sb.AppendLine($"  Speed: {veh.Speed[i]:F3} m/s");
        sb.AppendLine($"  State: {veh.State[i]}");
        sb.AppendLine($"  Throttle: {veh.Throttle[i]:F3}  Brake: {veh.Brake[i]:F3}");
        sb.AppendLine($"  SteeringAngle: {veh.SteeringAngle[i] * 180f / MathF.PI:F1} deg");

        int edgeIdx = veh.CurrentEdge[i];
        sb.AppendLine($"  CurrentEdge: {edgeIdx}  EdgeProgress: {veh.EdgeProgress[i]:F4}");
        if (edgeIdx >= 0 && edgeIdx < graph.Edges.Count && graph.Edges[edgeIdx].FromNode >= 0)
        {
            var edge = graph.Edges[edgeIdx];
            sb.AppendLine($"    Edge: {edge.FromNode} -> {edge.ToNode}  Length: {edge.Length:F2}m  SpeedLimit: {edge.SpeedLimit:F1} m/s  Lanes: {edge.LaneCount}");
            float stopT = d.StopLines.GetStopTAtToNode(edgeIdx);
            float fromT = d.StopLines.GetStopTAtFromNode(edgeIdx);
            sb.AppendLine($"    StopT: fromNode={fromT:F4} toNode={stopT:F4}");

            var signal = d.TrafficSignals.GetSignal(edgeIdx);
            if (d.StopSigns.CanQuery(graph) && d.YieldSigns.CanQuery(graph))
            {
                var stopSignal = d.StopSigns.GetSignal(edgeIdx, graph, i);
                var yieldSignal = d.YieldSigns.GetSignal(edgeIdx, graph);
                sb.AppendLine($"    Signals: traffic={signal} stopSign={stopSignal} yield={yieldSignal}");
            }
            else
            {
                sb.AppendLine($"    Signals: traffic={signal} stopSign=n/a yield=n/a (right-of-way tracking stale)");
            }

            var node = graph.Nodes[edge.ToNode];
            sb.AppendLine($"    ToNode {edge.ToNode}: flags={node.Flags}");
        }

        sb.AppendLine($"  CurrentArc: {veh.CurrentArc[i]}  ArcProgress: {veh.ArcProgress[i]:F4}");
        if (veh.CurrentArc[i] >= 0)
        {
            var arc = d.Arcs.GetArc(veh.CurrentArc[i]);
            sb.AppendLine($"    Arc: node={arc.NodeIndex} inEdge={arc.IncomingEdge} outEdge={arc.OutgoingEdge} inLane={arc.IncomingLane} outLane={arc.OutgoingLane} length={arc.Length:F2}m speedLimit={arc.SpeedLimit:F1}");
        }

        sb.AppendLine($"  CurrentLane: {veh.CurrentLane[i]}  TargetLane: {veh.TargetLane[i]}  LaneChangeProgress: {veh.LaneChangeProgress[i]:F3}");
        sb.AppendLine($"  PrevHeadingError: {veh.PrevHeadingError[i]:F4}");

        var path = veh.Path[i];
        int pathIdx = veh.PathIndex[i];
        sb.AppendLine($"  PathIndex: {pathIdx}/{path?.Count ?? 0}");
        if (path != null)
        {
            for (int p = 0; p < path.Count; p++)
            {
                int pe = path[p];
                string marker = p == pathIdx ? " <-- current" : "";
                if (pe >= 0 && pe < graph.Edges.Count && graph.Edges[pe].FromNode >= 0)
                {
                    var e = graph.Edges[pe];
                    sb.AppendLine($"    [{p}] edge {pe}: {e.FromNode}->{e.ToNode} len={e.Length:F1}m{marker}");
                }
                else
                {
                    sb.AppendLine($"    [{p}] edge {pe}: DEFUNCT{marker}");
                }
            }
        }

        sb.AppendLine($"  ResidentId: {veh.ResidentId[i]}  DistToRoadSq: {veh.DistToRoadSq[i]:F2}"
            + (veh.DistToRoadSq[i] > 4f ? "  [OFF-LANE]" : ""));
        int destNode = veh.DestinationNode[i];
        if (destNode >= 0 && destNode < graph.Nodes.Count)
            sb.AppendLine($"  Destination: node {destNode} (POI {graph.Nodes[destNode].PointOfInterest}, flags {graph.Nodes[destNode].Flags})");

        sb.AppendLine("  --- BLOCKER ANALYSIS ---");
        sb.AppendLine($"  Stuck for: {stuckSimTicks} sim-ticks (~{stuckSimTicks / 30f:F1}s of sim time continuously stopped)");

        int leader = NearestVehicleAhead(d, i, out float gap, out float lat);
        if (leader >= 0)
            sb.AppendLine($"  Nearest ahead: vehicle {leader} gap={gap:F2}m lateral={lat:F2}m " +
                $"speed={veh.Speed[leader]:F2} edge={veh.CurrentEdge[leader]} arc={veh.CurrentArc[leader]} prog={veh.EdgeProgress[leader]:F3}");
        else
            sb.AppendLine("  Nearest ahead: none within 30m (so it is held by a signal/stop, not a car)");

        if (edgeIdx >= 0 && edgeIdx < graph.Edges.Count && graph.Edges[edgeIdx].FromNode >= 0)
        {
            string stale = d.StopSigns.CanQuery(graph) ? "" : " [STALE: not updated since last cache rebuild]";
            sb.AppendLine($"  StopSign FCFS: {d.StopSigns.DescribeStopState(graph, edgeIdx)}{stale}");
            int toNode = graph.Edges[edgeIdx].ToNode;
            if (toNode >= 0 && d.StopSigns.IsStopSign(toNode))
                sb.AppendLine(d.StopSigns.DescribeNodeFull(graph, toNode));
        }

        if (veh.CurrentArc[i] < 0 && edgeIdx >= 0 && path != null && pathIdx + 1 < path.Count)
        {
            int nextEdge = path[pathIdx + 1];
            if (nextEdge >= 0 && nextEdge < graph.Edges.Count && graph.Edges[nextEdge].FromNode >= 0)
            {
                byte inLane = veh.CurrentLane[i];
                byte outLane = (byte)Math.Min(inLane, graph.Edges[nextEdge].LaneCount - 1);
                int arcIdx = d.Arcs.GetArcIndex(edgeIdx, nextEdge, inLane, outLane);
                sb.AppendLine($"  Intended turn: edge {edgeIdx} -> {nextEdge}, arc={arcIdx}");
                if (arcIdx >= 0)
                {
                    foreach (int c in d.Arcs.GetConflictingArcs(arcIdx))
                    {
                        var ca = d.Arcs.GetArc(c);
                        sb.AppendLine($"    conflictArc {c} ({ca.IncomingEdge}->{ca.OutgoingEdge}) occupants: {VehiclesOnArc(d, c)}");
                    }
                }
            }
        }

        int approachNode = veh.CurrentArc[i] >= 0
            ? d.Arcs.GetArc(veh.CurrentArc[i]).NodeIndex
            : (edgeIdx >= 0 && edgeIdx < graph.Edges.Count ? graph.Edges[edgeIdx].ToNode : -1);
        if (approachNode >= 0)
        {
            sb.AppendLine($"  Arcs at node {approachNode}:");
            foreach (int a in d.Arcs.GetArcsAtNode(approachNode))
            {
                var ad = d.Arcs.GetArc(a);
                string occ = VehiclesOnArc(d, a);
                if (occ != "(empty)")
                    sb.AppendLine($"    arc {a} ({ad.IncomingEdge}->{ad.OutgoingEdge}) occupants: {occ}");
            }
        }

        sb.AppendLine($"  Blocking chain: {BuildBlockingChain(d, i)}");
        sb.AppendLine();
    }

    // ── Node-level helpers for the cluster report ──────────────────────────

    /// <summary>Best-effort node "type" label from its traffic-control systems.</summary>
    public static string NodeTypeLabel(Deps d, int node)
    {
        if (d.TrafficSignals.IsTrafficLight(node)) return "TrafficLight";
        if (d.StopSigns.IsStopSign(node)) return "StopSign";
        if (d.YieldSigns.IsYield(node)) return "Yield";
        return "uncontrolled";
    }

    /// <summary>
    /// The node a stuck vehicle is contending for: its current arc's node if mid-intersection,
    /// otherwise its current edge's ToNode. -1 when neither is valid.
    /// </summary>
    public static int ContendedNode(Deps d, int v)
    {
        var veh = d.Vehicles;
        if (veh.CurrentArc[v] >= 0)
            return d.Arcs.GetArc(veh.CurrentArc[v]).NodeIndex;
        int e = veh.CurrentEdge[v];
        if (e >= 0 && e < d.Graph.Edges.Count && d.Graph.Edges[e].FromNode >= 0)
            return d.Graph.Edges[e].ToNode;
        return -1;
    }
}
