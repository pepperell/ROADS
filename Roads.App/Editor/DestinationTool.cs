using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that places or removes destination POI markers on road nodes.
/// Clicking a node with no destination sets the flag and POI type.
/// Clicking a destination node with the same type toggles it off.
/// Clicking a destination node with a different type changes the type.
/// Only nodes with ≤ 2 outgoing edges (dead-ends or mid-road) can be flagged.
/// </summary>
public class DestinationTool
{
    /// <summary>
    /// Places or modifies a destination POI on the nearest node within snap distance.
    /// </summary>
    /// <returns>True if a change was made.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, POIType poiType)
    {
        int node = graph.FindNearestNode(worldPos, EditorState.SnapDistance);
        if (node < 0 || !graph.CanPlaceMarker(node)) return false;

        var flags = graph.Nodes[node].Flags;
        var currentPOI = graph.Nodes[node].PointOfInterest;

        if (flags.HasFlag(NodeFlags.Destination))
        {
            if (currentPOI == poiType)
            {
                // Same type — toggle off
                graph.SetNodeFlags(node, flags & ~NodeFlags.Destination);
                graph.SetNodePOIType(node, POIType.None);
            }
            else
            {
                // Different type — re-type without removing
                graph.SetNodePOIType(node, poiType);
            }
        }
        else
        {
            // No destination — set flag and type
            graph.SetNodeFlags(node, flags | NodeFlags.Destination);
            graph.SetNodePOIType(node, poiType);
        }

        return true;
    }
}
