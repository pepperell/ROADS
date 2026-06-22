using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that toggles a spawn marker on road nodes. The active <see cref="SpawnKind"/>
/// (from the tool's sub-menu) selects whether an ordinary <see cref="NodeFlags.Spawn"/> or a
/// <see cref="NodeFlags.RegionSpawn"/> is toggled. The two spawn kinds are mutually exclusive on
/// a node. Only nodes with ≤ 2 outgoing edges (dead-ends or mid-road) can be flagged.
/// </summary>
public class SpawnPointTool
{
    /// <summary>
    /// Toggles the selected spawn kind on the nearest node within snap distance, clearing the
    /// other spawn kind so a node is only ever one kind.
    /// </summary>
    /// <returns>True if a flag was toggled.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, SpawnKind kind)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.CanPlaceMarker(node)) return false;

        var target = kind == SpawnKind.RegionSpawn ? NodeFlags.RegionSpawn : NodeFlags.Spawn;
        var other = kind == SpawnKind.RegionSpawn ? NodeFlags.Spawn : NodeFlags.RegionSpawn;

        var flags = graph.Nodes[node].Flags;
        if ((flags & target) != 0)
            flags &= ~target;                    // toggle the selected kind off
        else
            flags = (flags & ~other) | target;   // set this kind, clear the other spawn kind
        graph.SetNodeFlags(node, flags);
        return true;
    }
}
