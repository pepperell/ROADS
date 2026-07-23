using System.Numerics;

namespace Roads.App.World;

/// <summary>A filled water circle (one brush dab) in world meters.</summary>
public struct WaterCircle
{
    /// <summary>World-space center in meters.</summary>
    public Vector2 Center;
    /// <summary>Radius in meters.</summary>
    public float Radius;
}

/// <summary>
/// A freestanding stream segment: a cubic Bezier strip of the given total width.
/// Not part of the road graph — chains are just consecutive segments sharing an
/// endpoint, and round stroke caps make the joints (and mid-chain width changes)
/// render seamlessly.
/// </summary>
public struct WaterSegment
{
    /// <summary>Start point of the cubic.</summary>
    public Vector2 P0;
    /// <summary>First control point (near <see cref="P0"/>).</summary>
    public Vector2 C1;
    /// <summary>Second control point (near <see cref="P3"/>).</summary>
    public Vector2 C2;
    /// <summary>End point of the cubic.</summary>
    public Vector2 P3;
    /// <summary>Total stroke width in meters.</summary>
    public float Width;
}

/// <summary>
/// The painted water layer: brush circles plus stream segments. Purely visual —
/// nothing in the simulation reads it; it never touches the road graph.
/// <see cref="Version"/> is the layer's ONLY change signal (the water analog of
/// <c>RoadGraph.Version</c>): the minimap picture, prop placement, and the scenery
/// settle-gate key on it, so every mutation must bump it.
/// </summary>
public class WaterLayer
{
    /// <summary>
    /// Shore band width in meters rendered around every primitive (the tan ring).
    /// Lives here rather than in <c>WaterRenderer</c> because bridge detection also
    /// counts the band as water contact — a road touching the tan gets guard rails.
    /// </summary>
    public const float ShoreBand = 2.5f;

    private const float CellSize = 50f; // matches SpatialGrid/EdgeSpatialGrid cells
    private const int SegmentSamples = 16; // polyline resolution for distance tests

    private readonly List<WaterCircle> _circles = new();
    private readonly List<WaterSegment> _segments = new();

    /// <summary>Bumped on every mutation; consumers rebuild derived caches when it changes.</summary>
    public int Version { get; private set; }

    /// <summary>All brush circles (index-stable between mutations).</summary>
    public IReadOnlyList<WaterCircle> Circles => _circles;

    /// <summary>All stream segments (index-stable between mutations).</summary>
    public IReadOnlyList<WaterSegment> Segments => _segments;

    /// <summary>True when the layer holds no water at all.</summary>
    public bool IsEmpty => _circles.Count == 0 && _segments.Count == 0;

    // ── Derived state, rebuilt lazily when Version moves ─────────────────────────
    private struct Aabb { public float MinX, MinY, MaxX, MaxY; }

    private readonly List<Aabb> _circleBounds = new();
    private readonly List<Aabb> _segmentBounds = new();
    private readonly Dictionary<int, List<int>> _circleCells = new();
    private readonly Dictionary<int, List<int>> _segmentCells = new();
    private int _derivedVersion = -1;

    /// <summary>Adds one brush dab.</summary>
    public void AddCircle(Vector2 center, float radius)
    {
        _circles.Add(new WaterCircle { Center = center, Radius = radius });
        Version++;
    }

    /// <summary>Adds one stream segment (cubic Bezier strip).</summary>
    public void AddSegment(Vector2 p0, Vector2 c1, Vector2 c2, Vector2 p3, float width)
    {
        _segments.Add(new WaterSegment { P0 = p0, C1 = c1, C2 = c2, P3 = p3, Width = width });
        Version++;
    }

    /// <summary>
    /// Removes every primitive whose water area intersects the eraser circle at
    /// <paramref name="pos"/> with the given <paramref name="radius"/>. Whole
    /// primitives are removed (a dab, one stream segment) — there is no partial
    /// subtraction. Returns the number of primitives removed; bumps
    /// <see cref="Version"/> only when something was removed.
    /// </summary>
    public int EraseAt(Vector2 pos, float radius)
    {
        int before = _circles.Count + _segments.Count;
        _circles.RemoveAll(c => Vector2.DistanceSquared(c.Center, pos) <= Sq(c.Radius + radius));
        _segments.RemoveAll(s => DistSqToSegmentSpine(in s, pos) <= Sq(s.Width * 0.5f + radius));
        int removed = before - (_circles.Count + _segments.Count);
        if (removed > 0) Version++;
        return removed;
    }

    /// <summary>Removes all water. Always bumps the version (used by New/stress-scene resets).</summary>
    public void Clear()
    {
        _circles.Clear();
        _segments.Clear();
        Version++;
    }

    /// <summary>
    /// Wholesale replace from a loaded map (the <c>RoadGraph.LoadFromData</c> idiom):
    /// clears both lists, adopts the given primitives, and bumps the version once.
    /// </summary>
    public void LoadFromData(List<WaterCircle> circles, List<WaterSegment> segments)
    {
        _circles.Clear();
        _segments.Clear();
        _circles.AddRange(circles);
        _segments.AddRange(segments);
        Version++;
    }

    /// <summary>
    /// True when the point lies inside any water primitive (used by prop placement
    /// to keep trees/street lights out of rivers and ponds). O(1) cell lookup.
    /// </summary>
    public bool IsWater(Vector2 p) => IsWater(p, 0f);

    /// <summary>
    /// True when the point lies within <paramref name="expand"/> meters of any water
    /// primitive's fill. Bridge detection calls this with <see cref="ShoreBand"/> plus
    /// the road half-width so "the blue/tan touches the roadway" counts as contact.
    /// Scans every grid cell the inflated point box overlaps, so <paramref name="expand"/>
    /// may exceed the cell size (primitives are indexed by their un-inflated bounds).
    /// </summary>
    public bool IsWater(Vector2 p, float expand)
    {
        if (IsEmpty) return false;
        RebuildDerivedIfNeeded();

        int cx0 = (int)MathF.Floor((p.X - expand) / CellSize), cx1 = (int)MathF.Floor((p.X + expand) / CellSize);
        int cy0 = (int)MathF.Floor((p.Y - expand) / CellSize), cy1 = (int)MathF.Floor((p.Y + expand) / CellSize);
        for (int cx = cx0; cx <= cx1; cx++)
        for (int cy = cy0; cy <= cy1; cy++)
        {
            int key = GeometryUtil.PackCell(cx, cy);
            if (_circleCells.TryGetValue(key, out var circles))
            {
                foreach (int i in circles)
                {
                    var c = _circles[i];
                    if (Vector2.DistanceSquared(c.Center, p) <= Sq(c.Radius + expand)) return true;
                }
            }
            if (_segmentCells.TryGetValue(key, out var segs))
            {
                foreach (int i in segs)
                {
                    var s = _segments[i];
                    if (DistSqToSegmentSpine(in s, p) <= Sq(s.Width * 0.5f + expand)) return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Collects the indices of primitives whose AABB intersects the given world rect
    /// (render culling). Linear bounds scan — cheaper and simpler than a cell walk for
    /// whole-viewport queries. Output lists are cleared first.
    /// </summary>
    public void QueryVisible(float minX, float minY, float maxX, float maxY,
        List<int> circleResults, List<int> segmentResults)
    {
        circleResults.Clear();
        segmentResults.Clear();
        if (IsEmpty) return;
        RebuildDerivedIfNeeded();

        for (int i = 0; i < _circleBounds.Count; i++)
        {
            var b = _circleBounds[i];
            if (b.MaxX >= minX && b.MinX <= maxX && b.MaxY >= minY && b.MinY <= maxY)
                circleResults.Add(i);
        }
        for (int i = 0; i < _segmentBounds.Count; i++)
        {
            var b = _segmentBounds[i];
            if (b.MaxX >= minX && b.MinX <= maxX && b.MaxY >= minY && b.MinY <= maxY)
                segmentResults.Add(i);
        }
    }

    /// <summary>Evaluates a segment's cubic Bezier spine at parameter <paramref name="t"/>.</summary>
    public static Vector2 EvaluateSegment(in WaterSegment s, float t)
    {
        float u = 1f - t;
        float a = u * u * u, b = 3f * u * u * t, c = 3f * u * t * t, d = t * t * t;
        return new Vector2(
            a * s.P0.X + b * s.C1.X + c * s.C2.X + d * s.P3.X,
            a * s.P0.Y + b * s.C1.Y + c * s.C2.Y + d * s.P3.Y);
    }

    /// <summary>
    /// Squared distance from a point to the segment's sampled Bezier spine (projection
    /// onto each of the <see cref="SegmentSamples"/> polyline pieces, not just sample
    /// points, so narrow streams test accurately).
    /// </summary>
    private static float DistSqToSegmentSpine(in WaterSegment s, Vector2 p)
    {
        float best = float.MaxValue;
        var prev = s.P0;
        for (int k = 1; k <= SegmentSamples; k++)
        {
            var cur = EvaluateSegment(in s, k / (float)SegmentSamples);
            best = MathF.Min(best, DistSqPointToLineSegment(p, prev, cur));
            prev = cur;
        }
        return best;
    }

    private static float DistSqPointToLineSegment(Vector2 p, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        float lenSq = ab.LengthSquared();
        if (lenSq < 1e-12f) return Vector2.DistanceSquared(p, a);
        float t = Math.Clamp(Vector2.Dot(p - a, ab) / lenSq, 0f, 1f);
        return Vector2.DistanceSquared(p, a + ab * t);
    }

    private static float Sq(float v) => v * v;

    private void RebuildDerivedIfNeeded()
    {
        if (_derivedVersion == Version) return;
        _derivedVersion = Version;

        _circleBounds.Clear();
        _segmentBounds.Clear();
        _circleCells.Clear();
        _segmentCells.Clear();

        for (int i = 0; i < _circles.Count; i++)
        {
            var c = _circles[i];
            var b = new Aabb
            {
                MinX = c.Center.X - c.Radius, MinY = c.Center.Y - c.Radius,
                MaxX = c.Center.X + c.Radius, MaxY = c.Center.Y + c.Radius,
            };
            _circleBounds.Add(b);
            InsertIntoCells(_circleCells, i, in b);
        }
        for (int i = 0; i < _segments.Count; i++)
        {
            var s = _segments[i];
            // Control-polygon AABB contains the curve (convex-hull property); inflate by half width.
            float half = s.Width * 0.5f;
            var b = new Aabb
            {
                MinX = Min4(s.P0.X, s.C1.X, s.C2.X, s.P3.X) - half,
                MinY = Min4(s.P0.Y, s.C1.Y, s.C2.Y, s.P3.Y) - half,
                MaxX = Max4(s.P0.X, s.C1.X, s.C2.X, s.P3.X) + half,
                MaxY = Max4(s.P0.Y, s.C1.Y, s.C2.Y, s.P3.Y) + half,
            };
            _segmentBounds.Add(b);
            InsertIntoCells(_segmentCells, i, in b);
        }
    }

    private static void InsertIntoCells(Dictionary<int, List<int>> cells, int index, in Aabb b)
    {
        int cx0 = (int)MathF.Floor(b.MinX / CellSize), cx1 = (int)MathF.Floor(b.MaxX / CellSize);
        int cy0 = (int)MathF.Floor(b.MinY / CellSize), cy1 = (int)MathF.Floor(b.MaxY / CellSize);
        for (int cx = cx0; cx <= cx1; cx++)
        for (int cy = cy0; cy <= cy1; cy++)
        {
            int key = GeometryUtil.PackCell(cx, cy);
            if (!cells.TryGetValue(key, out var list))
            {
                list = new List<int>();
                cells[key] = list;
            }
            list.Add(index);
        }
    }

    private static float Min4(float a, float b, float c, float d) => MathF.Min(MathF.Min(a, b), MathF.Min(c, d));
    private static float Max4(float a, float b, float c, float d) => MathF.Max(MathF.Max(a, b), MathF.Max(c, d));
}
