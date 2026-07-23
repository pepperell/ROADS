using System.Numerics;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App;

/// <summary>
/// Central handler for all graph mutations. Invoked automatically once per frame at the
/// top of SimulationLoop.Tick, in both paused and active modes — no editor call site
/// needs to invoke it manually (manual calls are harmless: idempotent, version-keyed
/// early-out). Fixes stale state in: editor selection, node marker flags, and active
/// vehicles. Fix-ups therefore land at most one frame (~16 ms) after a mutation, and
/// always before any cache rebuild or vehicle update within the tick.
/// </summary>
public class GraphChangeHandler
{
    /// <summary>
    /// Maximum distance a vehicle whose edge went defunct may be re-snapped onto a
    /// surviving edge. Must exceed the largest legitimate on-road distance from a
    /// vehicle to its edge's centerline (the outer lane of a 3-lane two-way edge sits
    /// 2.5 lane widths ≈ 8.75 m out), so splits and in-place redraws keep their
    /// traffic. A vehicle farther out has no plausible continuous drive onto the new
    /// geometry — teleport-snapping it would strand it visibly off-road — so it is
    /// removed instead, consistent with the no-edge/no-path removal below.
    /// </summary>
    private const float MaxResnapDistance = 4f * SimConstants.LaneWidth;

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
    /// strips marker flags from nodes that now have too many edges, and repairs vehicles
    /// on defunct edges/paths — re-snapping them and re-pathing to their ORIGINAL
    /// destination (random fallback only when that is gone; removal when stranded farther
    /// than <see cref="MaxResnapDistance"/> from every surviving edge or nothing is
    /// reachable). Called once per tick by SimulationLoop; O(1) when unchanged.
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

        // 3. Strip Destination flags from nodes that now have 3+ outgoing edges
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

            // Repair the route to the vehicle's ORIGINAL destination first — an edit near
            // a car must never hijack its journey (placing the first Home used to pull
            // every passing car into its driveway via the random fallback). A fresh
            // random destination is chosen only when the original is gone or unreachable
            // from the re-snapped edge; the path may start on the reverse edge (turn
            // around), so CurrentEdge follows the path's actual start.
            int dest = _vehicles.DestinationNode[i];
            int startEdge = newEdge;
            float startT = newT;
            List<int>? newPath = null;
            if (dest >= 0 && dest < _graph.Nodes.Count
                && !float.IsNaN(_graph.Nodes[dest].Position.X))
                newPath = _spawner.FindPathToNode(newEdge, newT, dest, out startEdge, out startT);
            if (newPath == null)
            {
                newPath = _spawner.FindNewPath(newEdge, newT, out startEdge, out startT, out int newDest);
                if (newPath != null)
                    _vehicles.DestinationNode[i] = newDest;
            }
            if (newPath == null)
            {
                _vehicles.Remove(i);
                continue;
            }

            _vehicles.CurrentEdge[i] = startEdge;
            _vehicles.EdgeProgress[i] = startT;
            _vehicles.PrevHeadingError[i] = 0f;
            _vehicles.CurrentArc[i] = -1;
            _vehicles.ArcProgress[i] = 0f;
            _vehicles.ClearingArc[i] = -1;
            _vehicles.Path[i] = newPath;
            _vehicles.PathIndex[i] = 0;
        }
    }

    /// <summary>
    /// Finds the nearest edge within <see cref="MaxResnapDistance"/> of a position,
    /// preferring edges whose tangent aligns with the given direction. If direction is
    /// zero, falls back to pure distance matching. Returns -1 when no edge is in range.
    /// </summary>
    private (int edgeIndex, float t) FindNearestEdgeDirectional(Vector2 position, Vector2 direction)
    {
        var (bestEdge, bestT) = _edgeSpatialGrid.FindNearestEdgeWithT(_graph, position, MaxResnapDistance);
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
