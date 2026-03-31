using Roads.App;

namespace Roads.App.World;

/// <summary>
/// Uniform spatial grid for fast neighbor queries.
/// Uses index chaining (linked list per cell) with flat arrays.
/// </summary>
public class SpatialGrid
{
    /// <summary>Side length in meters of each grid cell.</summary>
    public const float CellSize = 50f;

    /// <summary>Per-entity next-pointer for the cell's linked list (or -1 for end).</summary>
    private int[] _next = Array.Empty<int>();

    /// <summary>Maps cell hash to the head entity index of that cell's linked list.</summary>
    private readonly Dictionary<int, int> _heads = new();

    /// <summary>Current capacity of the per-entity arrays.</summary>
    private int _capacity;

    /// <summary>
    /// Rebuilds the grid from vehicle positions.
    /// Must be called once per tick before any queries.
    /// </summary>
    /// <param name="posX">Array of X positions for all entities.</param>
    /// <param name="posY">Array of Y positions for all entities.</param>
    /// <param name="count">Number of active entities to index.</param>
    public void Rebuild(float[] posX, float[] posY, int count)
    {
        if (count > _capacity)
        {
            _capacity = Math.Max(count, _capacity * 2);
            _next = new int[_capacity];
        }

        _heads.Clear();

        for (int i = 0; i < count; i++)
        {
            int cell = CellKey(posX[i], posY[i]);

            if (_heads.TryGetValue(cell, out int existingHead))
            {
                _next[i] = existingHead;
            }
            else
            {
                _next[i] = -1;
            }
            _heads[cell] = i;
        }
    }

    /// <summary>
    /// Queries entities near a point, filtering by exact Euclidean distance.
    /// Only returns entities within the actual radius (not just in neighboring cells).
    /// </summary>
    /// <param name="cx">Center X coordinate of the search area.</param>
    /// <param name="cy">Center Y coordinate of the search area.</param>
    /// <param name="radius">Search radius in meters.</param>
    /// <param name="posX">Array of X positions for distance filtering.</param>
    /// <param name="posY">Array of Y positions for distance filtering.</param>
    /// <param name="results">List to append matching entity indices to (not cleared).</param>
    public void QueryFiltered(float cx, float cy, float radius, float[] posX, float[] posY, List<int> results)
    {
        int minCellX = (int)MathF.Floor((cx - radius) / CellSize);
        int maxCellX = (int)MathF.Floor((cx + radius) / CellSize);
        int minCellY = (int)MathF.Floor((cy - radius) / CellSize);
        int maxCellY = (int)MathF.Floor((cy + radius) / CellSize);

        float radiusSq = radius * radius;

        for (int gx = minCellX; gx <= maxCellX; gx++)
        {
            for (int gy = minCellY; gy <= maxCellY; gy++)
            {
                int cell = PackCell(gx, gy);
                if (!_heads.TryGetValue(cell, out int idx)) continue;

                while (idx >= 0)
                {
                    float dx = posX[idx] - cx;
                    float dy = posY[idx] - cy;
                    if (dx * dx + dy * dy <= radiusSq)
                        results.Add(idx);
                    idx = _next[idx];
                }
            }
        }
    }

    /// <summary>Computes the cell hash for a world-space position.</summary>
    private static int CellKey(float x, float y)
    {
        int cx = (int)MathF.Floor(x / CellSize);
        int cy = (int)MathF.Floor(y / CellSize);
        return PackCell(cx, cy);
    }

    /// <summary>Packs integer cell coordinates into a single hash key using bit spreading.</summary>
    private static int PackCell(int cx, int cy) => GeometryUtil.PackCell(cx, cy);
}
