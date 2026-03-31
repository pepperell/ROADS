using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that toggles the Destination flag on road nodes.
/// Only nodes with ≤ 2 outgoing edges (dead-ends or mid-road) can be flagged.
/// </summary>
public class DestinationTool
{
    /// <summary>
    /// Toggles the Destination flag on the nearest node within snap distance.
    /// </summary>
    /// <returns>True if a flag was toggled.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.CanPlaceMarker(node)) return false;

        var flags = graph.Nodes[node].Flags;
        flags ^= NodeFlags.Destination;
        graph.SetNodeFlags(node, flags);
        return true;
    }
}
