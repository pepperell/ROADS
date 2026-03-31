using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that builds two-way road segments by clicking to place nodes.
/// Each click either snaps to an existing node, splits a nearby edge to create a T-intersection,
/// or adds a new node. If a road start node exists in <see cref="EditorState"/>,
/// a two-way edge pair is created between the start and the new node.
/// </summary>
public class RoadTool
{
    /// <summary>
    /// Handles a click to place or extend a road segment. Snaps to existing nodes or edges
    /// when within <see cref="EditorState.SnapDistance"/>, otherwise creates a new node.
    /// If <see cref="EditorState.RoadStartNode"/> is set, creates a two-way edge pair and
    /// splits any crossed edges to form intersections.
    /// </summary>
    /// <param name="worldPos">Click position in world space.</param>
    /// <param name="graph">Road graph to add nodes and edges to.</param>
    /// <param name="state">Editor state tracking the in-progress road start node.</param>
    /// <param name="edgeSpatialGrid">Optional spatial grid for edge-snap and T-intersection support.</param>
    public void OnClick(Vector2 worldPos, RoadGraph graph, EditorState state, EdgeSpatialGrid? edgeSpatialGrid = null)
    {
        // Try to snap to an existing node first
        int existingNode = graph.FindNearestNode(worldPos, EditorState.SnapDistance);

        int nodeIndex;
        if (existingNode >= 0)
        {
            nodeIndex = existingNode;
        }
        else if (edgeSpatialGrid != null)
        {
            // Try to snap to the nearest edge — split it to create a T-intersection
            var (nearEdge, nearT) = edgeSpatialGrid.FindNearestEdgeWithT(graph, worldPos, EditorState.SnapDistance);
            if (nearEdge >= 0)
            {
                // Clamp t away from endpoints to avoid degenerate splits
                nearT = Math.Clamp(nearT, 0.05f, 0.95f);
                var (midNode, _, _) = graph.SplitEdge(nearEdge, nearT);
                nodeIndex = midNode;
            }
            else
            {
                nodeIndex = graph.AddNode(worldPos);
            }
        }
        else
        {
            nodeIndex = graph.AddNode(worldPos);
        }

        // If we have a start node, create edges in both directions (two-way road)
        if (state.RoadStartNode.HasValue && state.RoadStartNode.Value != nodeIndex)
        {
            int forwardEdge = graph.AddEdge(state.RoadStartNode.Value, nodeIndex);
            graph.AddEdge(nodeIndex, state.RoadStartNode.Value);
            // Only split forward edge; SplitEdge handles reverse automatically
            SplitAtCrossings(graph, forwardEdge);
        }

        // This node becomes the start of the next segment
        state.RoadStartNode = nodeIndex;
    }

    /// <summary>
    /// Detects where the new edge crosses existing edges and splits both at each crossing.
    /// Crossings are processed in ascending parametric order so that earlier splits don't
    /// invalidate later split positions. Called by <see cref="OnClick"/> after creating a new edge.
    /// </summary>
    /// <param name="graph">Road graph containing the edges.</param>
    /// <param name="newEdgeIndex">Index of the newly created forward edge to check for crossings.</param>
    private static void SplitAtCrossings(RoadGraph graph, int newEdgeIndex)
    {
        var crossings = graph.FindEdgeCrossings(newEdgeIndex);
        if (crossings.Count == 0) return;

        // Sort by tSelf ascending so we can split the new edge from start to end
        crossings.Sort((a, b) => a.tSelf.CompareTo(b.tSelf));

        // Step 1: Split all crossed "other" edges at their crossing points.
        // Track the midNode created at each crossing so we can reuse it in Step 2.
        var midNodes = new int[crossings.Count];
        for (int i = 0; i < crossings.Count; i++)
        {
            var (otherEdge, _, tOther) = crossings[i];
            if (graph.Edges[otherEdge].FromNode < 0)
            {
                midNodes[i] = -1; // already split/defunct
                continue;
            }
            var (midNode, _, _) = graph.SplitEdge(otherEdge, tOther);
            midNodes[i] = midNode;
        }

        // Step 2: Split the new edge at each tSelf, ascending.
        // Reuse the midNodes from Step 1 so both roads share the same intersection node.
        int currentEdge = newEdgeIndex;
        float consumedT = 0f;

        for (int i = 0; i < crossings.Count; i++)
        {
            if (midNodes[i] < 0) continue; // skipped crossing
            if (graph.Edges[currentEdge].FromNode < 0) break; // edge already defunct

            float tSelf = crossings[i].tSelf;

            // Rescale t into the remaining portion of the curve
            float localT = (tSelf - consumedT) / (1f - consumedT);
            localT = Math.Clamp(localT, 0.05f, 0.95f);

            var (_, _, secondHalf) = graph.SplitEdge(currentEdge, localT, midNodes[i]);

            // The second half (higher t) continues to be split
            currentEdge = secondHalf;
            consumedT = tSelf;
        }
    }

    /// <summary>
    /// Cancels the in-progress road by resetting all tool state.
    /// </summary>
    /// <param name="state">Editor state to reset.</param>
    public void OnCancel(EditorState state)
    {
        state.ResetToolState();
    }
}
