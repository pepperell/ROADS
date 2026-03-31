using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that creates vehicle destination points snapped to road edges.
/// </summary>
public class DestinationTool
{
    /// <summary>
    /// Places a destination point snapped to the nearest road edge.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph to evaluate edge geometry.</param>
    /// <param name="edgeGrid">Spatial grid used to find the nearest edge.</param>
    /// <returns>The new <see cref="DestinationPoint"/>, or <c>null</c> if no edge is within snap distance.</returns>
    public DestinationPoint? OnClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeGrid)
    {
        var snap = EdgeSnapTool.Snap(worldPos, graph, edgeGrid);
        if (snap == null) return null;

        var (pos, edge, t) = snap.Value;
        return new DestinationPoint { Position = pos, EdgeIndex = edge, EdgeT = t };
    }
}
