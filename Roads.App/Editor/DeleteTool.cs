using System.Numerics;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that removes the nearest road edge (and its reverse pair) on click.
/// </summary>
public class DeleteTool
{
    /// <summary>
    /// Requests deletion of the nearest edge within snap distance. The request is routed through
    /// the population coordinator, which removes the edge and its reverse — immediately if neither
    /// endpoint is populated, or via a graceful drain that drives people out first if removing the
    /// road would orphan a populated endpoint.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph used to locate the nearest edge.</param>
    /// <param name="edgeGrid">Spatial grid used to find the nearest edge.</param>
    /// <param name="population">
    /// Population coordinator that owns the delete lifecycle (reverse-edge removal and the
    /// drain-or-immediate decision are handled internally by <see cref="PopulationManager.RequestDeleteEdge"/>).
    /// </param>
    /// <returns><c>true</c> if an edge was found and its deletion requested; otherwise <c>false</c>.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid, PopulationManager population)
    {
        int nearestEdge = edgeGrid.FindNearestEdge(graph, worldPos, EditorState.SnapDistance);

        if (nearestEdge >= 0)
        {
            // The coordinator removes the reverse edge too and decides immediate-vs-drain internally.
            population.RequestDeleteEdge(nearestEdge);
            return true;
        }

        return false;
    }
}
