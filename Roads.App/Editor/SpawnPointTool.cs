using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that toggles the <see cref="NodeFlags.Spawn"/> marker on road nodes. Only nodes
/// with ≤ 2 outgoing edges (dead-ends or mid-road) can be flagged.
/// </summary>
public class SpawnPointTool
{
    /// <summary>
    /// Toggles <see cref="NodeFlags.Spawn"/> on the nearest node within snap distance.
    /// </summary>
    /// <returns>True if a flag was toggled.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.CanPlaceMarker(node)) return false;

        var flags = graph.Nodes[node].Flags;
        graph.SetNodeFlags(node, flags ^ NodeFlags.Spawn);
        return true;
    }
}
