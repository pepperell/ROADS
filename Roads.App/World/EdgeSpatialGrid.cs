using System.Numerics;
using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Spatial grid for fast road edge lookup by world position.
/// Indexes edge sample points into cells using linked-list chaining (flat arrays)
/// for minimal GC pressure. Rebuilds when the graph changes.
/// </summary>
public class EdgeSpatialGrid
{
    /// <summary>Side length in meters of each grid cell.</summary>
    private const float CellSize = 50f;
    /// <summary>Number of sample points along each edge used for spatial indexing (t = 0.0, 0.1, ..., 1.0).</summary>
    private const int SamplesPerEdge = 11;

    /// <summary>Per-entry edge index.</summary>
    private int[] _edgeIndex = Array.Empty<int>();
    /// <summary>Per-entry next-pointer for the cell's linked list (or -1 for end).</summary>
    private int[] _next = Array.Empty<int>();
    /// <summary>Maps cell hash to the head entry index of that cell's linked list.</summary>
    private readonly Dictionary<int, int> _heads = new();
    /// <summary>Current number of entries in the flat arrays.</summary>
    private int _entryCount;
    /// <summary>Current capacity of the flat arrays.</summary>
    private int _capacity;

    /// <summary>Graph version when the grid was last rebuilt.</summary>
    private int _cachedVersion = -1;

    /// <summary>
    /// Rebuilds the spatial index if the graph has changed since the last rebuild.
    /// Must be called before any edge queries each frame.
    /// </summary>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_cachedVersion == graph.Version) return;
        Rebuild(graph);
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Clears and repopulates the grid by sampling each active edge at regular t intervals.
    /// Uses linked-list chaining into flat arrays for cache-friendly iteration.
    /// </summary>
    private void Rebuild(RoadGraph graph)
    {
        _heads.Clear();
        _entryCount = 0;

        // Estimate capacity: active edges * samples per edge
        int estimatedEntries = graph.Edges.Count * SamplesPerEdge;
        if (estimatedEntries > _capacity)
        {
            _capacity = Math.Max(estimatedEntries, _capacity * 2);
            _edgeIndex = new int[_capacity];
            _next = new int[_capacity];
        }

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;

            for (int s = 0; s < SamplesPerEdge; s++)
            {
                float t = s / (float)(SamplesPerEdge - 1);
                var pt = graph.EvaluateBezier(i, t);
                int cell = PackCell(
                    (int)MathF.Floor(pt.X / CellSize),
                    (int)MathF.Floor(pt.Y / CellSize));

                int entryIdx = _entryCount++;
                if (entryIdx >= _capacity)
                {
                    _capacity = Math.Max(_capacity * 2, entryIdx + 1);
                    Array.Resize(ref _edgeIndex, _capacity);
                    Array.Resize(ref _next, _capacity);
                }

                _edgeIndex[entryIdx] = i;
                _next[entryIdx] = _heads.TryGetValue(cell, out int existingHead) ? existingHead : -1;
                _heads[cell] = entryIdx;
            }
        }
    }

    /// <summary>
    /// Finds the nearest active edge to a world position.
    /// </summary>
    public int FindNearestEdge(RoadGraph graph, Vector2 position, float maxDistance)
    {
        var (edgeIndex, _) = FindNearestEdgeWithT(graph, position, maxDistance);
        return edgeIndex;
    }

    /// <summary>
    /// Finds the nearest active edge and the parametric t of the closest point on that edge.
    /// </summary>
    public (int edgeIndex, float t) FindNearestEdgeWithT(RoadGraph graph, Vector2 position, float maxDistance)
    {
        int bestEdge = -1;
        float bestT = 0f;
        float bestDistSq = maxDistance < float.MaxValue ? maxDistance * maxDistance : float.MaxValue;

        bool searchAll = maxDistance > CellSize * int.MaxValue;

        if (searchAll)
        {
            // Walk all heads and iterate all entries
            var evaluated = new HashSet<int>();
            foreach (var headIdx in _heads.Values)
            {
                int idx = headIdx;
                while (idx >= 0)
                {
                    if (evaluated.Add(_edgeIndex[idx]))
                        SearchEdge(graph, position, _edgeIndex[idx], ref bestEdge, ref bestT, ref bestDistSq);
                    idx = _next[idx];
                }
            }
        }
        else
        {
            int minCellX = (int)MathF.Floor((position.X - maxDistance) / CellSize);
            int maxCellX = (int)MathF.Floor((position.X + maxDistance) / CellSize);
            int minCellY = (int)MathF.Floor((position.Y - maxDistance) / CellSize);
            int maxCellY = (int)MathF.Floor((position.Y + maxDistance) / CellSize);

            var evaluated = new HashSet<int>();

            for (int gx = minCellX; gx <= maxCellX; gx++)
            {
                for (int gy = minCellY; gy <= maxCellY; gy++)
                {
                    int cell = PackCell(gx, gy);
                    if (!_heads.TryGetValue(cell, out int idx)) continue;

                    while (idx >= 0)
                    {
                        int edgeIdx = _edgeIndex[idx];
                        if (evaluated.Add(edgeIdx))
                            SearchEdge(graph, position, edgeIdx, ref bestEdge, ref bestT, ref bestDistSq);
                        idx = _next[idx];
                    }
                }
            }
        }

        return (bestEdge, bestT);
    }

    /// <summary>Evaluates a single edge at fine sample points, updating the best match.</summary>
    private static void SearchEdge(RoadGraph graph, Vector2 position, int edgeIdx,
        ref int bestEdge, ref float bestT, ref float bestDistSq)
    {
        var edge = graph.Edges[edgeIdx];
        if (edge.FromNode < 0) return;

        const int fineSteps = 20;
        for (int s = 0; s <= fineSteps; s++)
        {
            float t = s / (float)fineSteps;
            var pt = graph.EvaluateBezier(edgeIdx, t);
            float distSq = Vector2.DistanceSquared(position, pt);
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                bestEdge = edgeIdx;
                bestT = t;
            }
        }
    }

    /// <summary>Packs integer cell coordinates into a single hash key using bit spreading.</summary>
    private static int PackCell(int cx, int cy) => GeometryUtil.PackCell(cx, cy);
}
