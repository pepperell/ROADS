using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that removes the nearest road edge (and its reverse pair) on click.
/// </summary>
public class DeleteTool
{
    /// <summary>
    /// Deletes the nearest edge within snap distance, including its reverse edge.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph to remove edges from.</param>
    /// <param name="edgeGrid">Spatial grid used to find the nearest edge.</param>
    /// <returns><c>true</c> if an edge was deleted; otherwise <c>false</c>.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid)
    {
        int nearestEdge = edgeGrid.FindNearestEdge(graph, worldPos, EditorState.SnapDistance);

        if (nearestEdge >= 0)
        {
            // Also remove the reverse edge so the whole road disappears
            int reverse = graph.FindReverseEdge(nearestEdge);
            graph.RemoveEdge(nearestEdge);
            if (reverse >= 0)
                graph.RemoveEdge(reverse);
            return true;
        }

        return false;
    }
}
