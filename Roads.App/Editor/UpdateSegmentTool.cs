using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that retypes the clicked segment to the road-toolbar's sticky options:
/// road type (which also resets the speed limit to the type default), per-direction lane
/// count, one-way vs two-way topology, and the shared-lane flag. All the underlying
/// <see cref="RoadGraph"/> mutators mirror to the reverse edge and bump the graph
/// version, so caches and vehicles fix up through the normal GraphChangeHandler path —
/// exactly as the Select tool's keyboard shortcuts (R, +/-, O, J) do.
/// </summary>
public class UpdateSegmentTool
{
    /// <summary>
    /// Applies the sticky road options to the nearest segment within snap distance.
    /// A two-way pair is resolved to its primary (lower-index) edge first so the
    /// direction that survives a one-way conversion is deterministic — the O key on the
    /// Select tool remains the precise direction-flip. The shared-lane flag is cleared
    /// BEFORE a one-way conversion (SetSharedLane is a no-op once the reverse edge is
    /// gone), and applied after a two-way conversion, where it forces the lane count
    /// to 1.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph containing the segment.</param>
    /// <param name="state">Editor state carrying the sticky road options.</param>
    /// <param name="edgeSpatialGrid">Spatial grid used to find the nearest edge.</param>
    /// <returns><c>true</c> if a segment was found and updated.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, EditorState state, EdgeSpatialGrid edgeSpatialGrid)
    {
        edgeSpatialGrid.RebuildIfNeeded(graph);
        int edge = edgeSpatialGrid.FindNearestEdge(graph, worldPos, EditorState.SnapDistance);
        if (edge < 0) return false;

        int reverse = graph.FindReverseEdge(edge);
        if (reverse >= 0) edge = Math.Min(edge, reverse);

        graph.SetEdgeRoadType(edge, state.SelectedRoadType);
        graph.SetLaneCount(edge, state.SelectedLaneCount);
        if (state.SelectedOneWay)
        {
            graph.SetSharedLane(edge, false);
            graph.MakeOneWay(edge);
        }
        else
        {
            graph.MakeTwoWay(edge);
            graph.SetSharedLane(edge, state.SelectedSharedLane);
        }
        return true;
    }
}
