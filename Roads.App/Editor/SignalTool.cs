using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that cycles intersection signal types on click.
/// Left-click cycles the node-level signal type. Right-click toggles per-edge stop/yield
/// exemptions or rotates traffic light phase groupings.
/// </summary>
public class SignalTool
{
    /// <summary>
    /// Cycles the signal type of the nearest intersection node:
    /// None → Traffic Light → Stop Sign → Yield → None.
    /// Sets <see cref="NodeFlags.ManualSignal"/> to prevent auto-assignment from overriding the choice.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph containing the nodes.</param>
    /// <returns><c>true</c> if a node was found and its signal type was changed; otherwise <c>false</c>.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0) return false;

        // Only allow signals at real intersections (incl. one-way merges, which have only
        // 2 incoming edges but 3 distinct neighbors — see RoadGraph.IsTrafficControlJunction).
        if (!graph.IsTrafficControlJunction(node)) return false;

        var flags = graph.Nodes[node].Flags;

        if (flags.HasFlag(NodeFlags.TrafficLight))
        {
            // Traffic Light → Stop Sign
            flags = (flags & ~NodeFlags.TrafficLight) | NodeFlags.StopSign | NodeFlags.ManualSignal;
        }
        else if (flags.HasFlag(NodeFlags.StopSign))
        {
            // Stop Sign → Yield
            flags = (flags & ~NodeFlags.StopSign) | NodeFlags.Yield | NodeFlags.ManualSignal;
        }
        else if (flags.HasFlag(NodeFlags.Yield))
        {
            // Yield → None
            flags = (flags & ~NodeFlags.Yield) | NodeFlags.ManualSignal;
        }
        else
        {
            // None → Traffic Light
            flags = flags | NodeFlags.TrafficLight | NodeFlags.ManualSignal;
        }

        graph.SetNodeFlags(node, flags);
        return true;
    }

    /// <summary>
    /// Right-click handler: toggles per-edge stop/yield exemptions, or rotates traffic light
    /// phase groupings. For stop/yield nodes, finds the nearest incoming edge at the node and
    /// toggles its exempt status. For traffic light nodes, rotates the phase pairing.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph containing the nodes and edges.</param>
    /// <param name="edgeGrid">Spatial grid for nearest-edge lookups.</param>
    /// <param name="signals">Traffic signal system (for phase rotation).</param>
    /// <param name="stopSigns">Stop sign system (for edge exemptions).</param>
    /// <param name="yieldSigns">Yield sign system (for edge exemptions).</param>
    /// <returns><c>true</c> if a signal property was changed; otherwise <c>false</c>.</returns>
    public bool OnRightClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid,
        TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns)
    {
        // First, try to find a signal node near the click
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0) return false;

        var flags = graph.Nodes[node].Flags;

        // Traffic light node: rotate phase grouping
        if (flags.HasFlag(NodeFlags.TrafficLight))
        {
            signals.RotatePhase(node);
            return true;
        }

        // Stop sign or yield node: toggle the nearest incoming edge's exemption
        bool isStop = flags.HasFlag(NodeFlags.StopSign);
        bool isYield = flags.HasFlag(NodeFlags.Yield);
        if (!isStop && !isYield) return false;

        int bestEdge = FindNearestIncomingEdge(worldPos, graph, node);
        if (bestEdge < 0) return false;

        if (isStop)
        {
            bool current = stopSigns.IsEdgeExempt(bestEdge);
            stopSigns.SetEdgeExempt(bestEdge, !current);
        }
        else
        {
            bool current = yieldSigns.IsEdgeExempt(bestEdge);
            yieldSigns.SetEdgeExempt(bestEdge, !current);
        }
        return true;
    }

    /// <summary>
    /// Finds the incoming edge at a node whose approach end is closest to the click position.
    /// Evaluates each incoming edge at t=0.9 (near the node) and picks the nearest.
    /// </summary>
    private static int FindNearestIncomingEdge(Vector2 worldPos, RoadGraph graph, int nodeIndex)
    {
        var incoming = graph.GetIncomingEdges(nodeIndex);
        int bestEdge = -1;
        float bestDist = float.MaxValue;

        foreach (int edgeIdx in incoming)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;

            var pos = graph.EvaluateBezier(edgeIdx, 0.9f);
            float dist = Vector2.DistanceSquared(worldPos, pos);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestEdge = edgeIdx;
            }
        }
        return bestEdge;
    }
}
