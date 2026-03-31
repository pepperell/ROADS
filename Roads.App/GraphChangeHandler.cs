using System.Numerics;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App;

/// <summary>
/// Central handler for all graph mutations. Idempotent — safe to call multiple times;
/// only runs fix-ups when the graph version has actually changed. Fixes stale state in:
/// editor selection, node marker flags, and active vehicles.
/// </summary>
public class GraphChangeHandler
{
    private readonly RoadGraph _graph;
    private readonly EditorState _editorState;
    private readonly VehicleStore _vehicles;
    private readonly EdgeSpatialGrid _edgeSpatialGrid;
    private readonly VehicleSpawner _spawner;
    private int _lastHandledGraphVersion;

    public GraphChangeHandler(RoadGraph graph, EditorState editorState,
        VehicleStore vehicles, EdgeSpatialGrid edgeSpatialGrid,
        VehicleSpawner spawner)
    {
        _graph = graph;
        _editorState = editorState;
        _vehicles = vehicles;
        _edgeSpatialGrid = edgeSpatialGrid;
        _spawner = spawner;
    }

    /// <summary>
    /// Checks if the graph version has changed and, if so, fixes stale editor selections,
    /// strips marker flags from nodes that now have too many edges, and reroutes vehicles
    /// on defunct edges.
    /// </summary>
    public void HandleIfNeeded()
    {
        if (_graph.Version == _lastHandledGraphVersion) return;
        _lastHandledGraphVersion = _graph.Version;

        // 1. Rebuild spatial index first (needed by re-snap operations below)
        _edgeSpatialGrid.RebuildIfNeeded(_graph);

        // 2. Clear stale editor selections
        if (_editorState.SelectedEdge >= 0 &&
            (_editorState.SelectedEdge >= _graph.Edges.Count ||
             _graph.Edges[_editorState.SelectedEdge].FromNode < 0))
            _editorState.SelectedEdge = -1;

        if (_editorState.SelectedNode >= 0 &&
            (_editorState.SelectedNode >= _graph.Nodes.Count ||
             float.IsNaN(_graph.Nodes[_editorState.SelectedNode].Position.X)))
            _editorState.SelectedNode = -1;

        // 3. Strip Spawn/Destination flags from nodes that now have 3+ outgoing edges
        _graph.StripMarkerFlagsFromIntersections();

        // 4. Fix stale vehicle edges and paths
        for (int i = _vehicles.Count - 1; i >= 0; i--)
        {
            int curEdge = _vehicles.CurrentEdge[i];
            bool edgeStale = curEdge < 0 || curEdge >= _graph.Edges.Count
                || _graph.Edges[curEdge].FromNode < 0;

            bool pathStale = false;
            if (!edgeStale)
            {
                var path = _vehicles.Path[i];
                int pathIdx = _vehicles.PathIndex[i];
                if (path != null)
                {
                    for (int j = pathIdx; j < path.Count; j++)
                    {
                        int pe = path[j];
                        if (pe < 0 || pe >= _graph.Edges.Count || _graph.Edges[pe].FromNode < 0)
                        {
                            pathStale = true;
                            break;
                        }
                    }
                }
            }

            if (!edgeStale && !pathStale) continue;

            var pos = new Vector2(_vehicles.PosX[i], _vehicles.PosY[i]);
            float heading = _vehicles.Heading[i];
            var headingVec = new Vector2(MathF.Cos(heading), MathF.Sin(heading));
            var (newEdge, newT) = FindNearestEdgeDirectional(pos, headingVec);
            if (newEdge < 0)
            {
                _vehicles.Remove(i);
                continue;
            }

            _vehicles.CurrentEdge[i] = newEdge;
            _vehicles.EdgeProgress[i] = newT;
            _vehicles.PrevHeadingError[i] = 0f;
            _vehicles.CurrentArc[i] = -1;
            _vehicles.ArcProgress[i] = 0f;

            var newPath = _spawner.FindNewPath(newEdge, newT);
            if (newPath == null)
            {
                _vehicles.Remove(i);
                continue;
            }

            _vehicles.Path[i] = newPath;
            _vehicles.PathIndex[i] = 0;
        }
    }

    /// <summary>
    /// Finds the nearest edge to a position, preferring edges whose tangent aligns with
    /// the given direction. If direction is zero, falls back to pure distance matching.
    /// </summary>
    private (int edgeIndex, float t) FindNearestEdgeDirectional(Vector2 position, Vector2 direction)
    {
        var (bestEdge, bestT) = _edgeSpatialGrid.FindNearestEdgeWithT(_graph, position, float.MaxValue);
        if (bestEdge < 0 || direction == Vector2.Zero) return (bestEdge, bestT);

        // Check if the matched edge's tangent aligns with our direction
        var tangent = _graph.EvaluateBezierTangent(bestEdge, bestT);
        float dot = Vector2.Dot(tangent, direction);
        if (dot >= 0) return (bestEdge, bestT); // same direction, good

        // Wrong direction — try the reverse edge
        int reverse = _graph.FindReverseEdge(bestEdge);
        if (reverse >= 0 && _graph.Edges[reverse].FromNode >= 0)
        {
            float reverseT = 1f - bestT;
            return (reverse, reverseT);
        }

        return (bestEdge, bestT); // no reverse available, keep what we have
    }
}
