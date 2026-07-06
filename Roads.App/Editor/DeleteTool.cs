using System.Numerics;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that removes what the cursor targets on click: a node (deleted together
/// with every attached segment), or otherwise the nearest road edge (and its reverse pair).
/// </summary>
public class DeleteTool
{
    /// <summary>
    /// Deletes the click target. A node within <see cref="EditorState.NodePickDistance"/>
    /// (the same radius the hover highlight uses) wins over edges and is deleted along with
    /// ALL its attached segments; otherwise the nearest edge within snap distance is deleted
    /// (with its reverse pair). Both paths route through the population coordinator, which
    /// deletes immediately when no population depends on the target, or begins a graceful
    /// drain that drives people out first (holding the roads open until they have left).
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph used to locate the click target.</param>
    /// <param name="edgeGrid">Spatial grid used to find the nearest edge.</param>
    /// <param name="population">
    /// Population coordinator that owns the delete lifecycle
    /// (<see cref="PopulationManager.RequestDeleteNode"/> /
    /// <see cref="PopulationManager.RequestDeleteEdge"/> decide immediate-vs-drain internally).
    /// </param>
    /// <returns><c>true</c> if a node or edge was found and its deletion requested.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid, PopulationManager population)
    {
        int nearNode = graph.FindNearestNode(worldPos, EditorState.NodePickDistance);
        if (nearNode >= 0)
        {
            // The coordinator removes the node and every attached segment — immediately, or
            // after draining any population living/parked at it.
            population.RequestDeleteNode(nearNode);
            return true;
        }

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
