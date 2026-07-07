using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool backing the signals submenu: <see cref="OnClick"/> (the Change Type tool)
/// cycles a node's signal type; <see cref="RotatePhase"/> (the Rotate tool) rotates a
/// traffic light's phase grouping; <see cref="ToggleExemption"/> (the Exempt tool)
/// toggles a stop/yield approach's exemption.
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
            // Traffic Light → Stop Sign (the actuated bit goes with the light, so a
            // future re-light starts at the fixed-time default)
            flags = (flags & ~(NodeFlags.TrafficLight | NodeFlags.ActuatedSignal))
                | NodeFlags.StopSign | NodeFlags.ManualSignal;
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
    /// The Rotate tool's click handler: rotates the phase grouping of the nearest
    /// traffic-light node (no-op on any other node).
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph containing the nodes.</param>
    /// <param name="signals">Traffic signal system (owns the phase rotation).</param>
    /// <returns><c>true</c> if a light's phases were rotated; otherwise <c>false</c>.</returns>
    public bool RotatePhase(Vector2 worldPos, RoadGraph graph, TrafficSignalSystem signals)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.Nodes[node].Flags.HasFlag(NodeFlags.TrafficLight)) return false;

        signals.RotatePhase(node);
        return true;
    }

    /// <summary>
    /// The Exempt tool's click handler: at the nearest stop/yield node, toggles the
    /// exemption of the incoming approach closest to the click (an exempt approach does
    /// not stop/yield at the node's sign). No-op on nodes without a stop or yield sign.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph containing the nodes and edges.</param>
    /// <param name="stopSigns">Stop sign system (for edge exemptions).</param>
    /// <param name="yieldSigns">Yield sign system (for edge exemptions).</param>
    /// <returns><c>true</c> if an approach's exemption was toggled; otherwise <c>false</c>.</returns>
    public bool ToggleExemption(Vector2 worldPos, RoadGraph graph,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0) return false;

        var flags = graph.Nodes[node].Flags;
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
    /// Public so the Exempt tool's hover highlight can target the same approach a click would.
    /// </summary>
    public static int FindNearestIncomingEdge(Vector2 worldPos, RoadGraph graph, int nodeIndex)
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
