using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Shared edge-snapping logic for editor tools that place points on road edges.
/// Returns the snapped position, edge index, and parametric t.
/// Used by both spawn point and destination point placement.
/// </summary>
public static class EdgeSnapTool
{
    /// <summary>
    /// Finds the nearest point on a road edge to the given world position.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph to evaluate edge geometry.</param>
    /// <param name="edgeGrid">Spatial grid used to find the nearest edge.</param>
    /// <returns>The snapped position, edge index, and edge t, or null if no edge is within snap distance.</returns>
    public static (Vector2 position, int edgeIndex, float edgeT)? Snap(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid)
    {
        var (bestEdge, bestT) = edgeGrid.FindNearestEdgeWithT(graph, worldPos, EditorState.SnapDistance * 2f);

        if (bestEdge < 0) return null;

        var bestPoint = graph.EvaluateBezier(bestEdge, bestT);
        return (bestPoint, bestEdge, bestT);
    }
}
