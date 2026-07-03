using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that adds a standalone node: a click splits the nearest road within snap
/// distance (creating an on-road node) or places a free node in empty space. Clicks
/// within snap distance of an existing node do nothing (nodes never stack). The hover
/// ghost comes from <see cref="ComputeGhost"/>, which applies the SAME distance-clamped
/// split position the click will use, so the preview and the committed node always agree.
/// </summary>
public class NodeTool
{
    /// <summary>
    /// Adds a node at the click position: splits the nearest edge within snap distance at
    /// the distance-clamped parameter, or adds a free node in empty space. No-op within
    /// snap distance of an existing node.
    /// </summary>
    /// <returns>True if a node was created.</returns>
    public bool OnClick(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeSpatialGrid)
    {
        if (graph.FindNearestNode(worldPos, EditorState.SnapDistance) >= 0) return false;

        edgeSpatialGrid.RebuildIfNeeded(graph);
        var (nearEdge, nearT) = edgeSpatialGrid.FindNearestEdgeWithT(graph, worldPos, EditorState.SnapDistance);
        if (nearEdge >= 0)
        {
            graph.SplitEdge(nearEdge, ClampSplitT(graph, nearEdge, nearT));
            return true;
        }

        graph.AddNode(worldPos);
        return true;
    }

    /// <summary>
    /// Ghost position for the hover preview: the clamped on-road split position when a
    /// road is within snap distance, the raw cursor position in empty space, or null over
    /// an existing node (where a click adds nothing).
    /// </summary>
    public static Vector2? ComputeGhost(Vector2 worldPos, RoadGraph graph, EdgeSpatialGrid edgeSpatialGrid)
    {
        if (graph.FindNearestNode(worldPos, EditorState.SnapDistance) >= 0) return null;

        edgeSpatialGrid.RebuildIfNeeded(graph);
        var (nearEdge, nearT) = edgeSpatialGrid.FindNearestEdgeWithT(graph, worldPos, EditorState.SnapDistance);
        if (nearEdge >= 0)
            return graph.EvaluateBezier(nearEdge, ClampSplitT(graph, nearEdge, nearT));
        return worldPos;
    }

    /// <summary>
    /// Distance-based split clamp: keeps the split <see cref="SimConstants.MinSplitSetback"/>
    /// meters from each endpoint (capped at t = 0.45 on very short edges) — the same policy
    /// as the Road tool's T-intersection snap, so the un-splittable zone near a node does
    /// not scale with road length.
    /// </summary>
    private static float ClampSplitT(RoadGraph graph, int edgeIndex, float t)
    {
        float len = MathF.Max(graph.Edges[edgeIndex].Length, 0.01f);
        float margin = MathF.Min(0.45f, SimConstants.MinSplitSetback / len);
        return Math.Clamp(t, margin, 1f - margin);
    }
}
