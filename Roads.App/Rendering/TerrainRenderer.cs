using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Draws the grass terrain background under the road network: a flat base color (via
/// <see cref="GetBaseColor"/>, used for <c>canvas.Clear</c> before the camera matrix is set)
/// plus zoom-gated positional detail drawn by <see cref="Draw"/> as the first world-space
/// pass. Detail has three tiers: 512m supertile mottle blobs (zoom ≥ 0.15), 128m detail-tile
/// grass/dirt patches (zoom ≥ 0.6), and per-frame fine grass speckle on a jittered lattice
/// (zoom ≥ 2.0, capped at ~1500 flecks per frame). All placement is a pure deterministic hash
/// of tile coordinates — the terrain never depends on the road graph and never reshuffles.
/// Tile geometry is cached per tile (SKPath built once, drawn every frame with day/night
/// dimmed colors); each tier's cache is fully cleared (paths disposed) when it exceeds
/// <see cref="MaxCachedTiles"/> entries. All colors run through the same darkness/dawn-dusk
/// formula as <see cref="GetBaseColor"/> so terrain participates in the day/night cycle.
/// Rendering is read-only; no per-frame heap allocation in the draw loops.
/// </summary>
public sealed class TerrainRenderer
{
    private const float SuperTileSize = 512f;
    private const float DetailTileSize = 128f;
    /// <summary>Supertile blobs have a 300m max nominal radius; per-point jitter stays within it.</summary>
    private const float SuperInflate = 320f;
    /// <summary>Detail patches have a 55m max nominal radius.</summary>
    private const float DetailInflate = 64f;
    private const float SpeckleSpacing = 24f;
    private const int MaxSpecklesPerFrame = 1500;
    private const int MaxCachedTiles = 2048;

    // Tier salts keep the three hash streams independent of each other.
    private const uint SuperSalt = 0xB5297A4Du;
    private const uint DetailSalt = 0x68E31DA4u;
    private const uint SpeckleSalt = 0x1B56C4E9u;

    /// <summary>One cached organic patch: geometry, its conservative bounds for culling, and
    /// its undimmed base color (dimmed per frame by the current darkness).</summary>
    private struct Patch
    {
        public SKPath Path;
        public SKRect Bounds;
        public SKColor Color;
    }

    private readonly Dictionary<long, Patch[]> _superTiles = new();
    private readonly Dictionary<long, Patch[]> _detailTiles = new();

    private readonly SKPaint _patchPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _speckPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeWidth = 0.3f,
        StrokeCap = SKStrokeCap.Round,
        IsAntialias = true,
    };

    /// <summary>
    /// Grass base color for the full-canvas clear: day RGB(86,105,64), lerped toward night
    /// RGB(15,17,26) by <c>darkness * 0.85</c>, with a warm dawn/dusk bias that peaks
    /// mid-transition and vanishes at full day or full night.
    /// </summary>
    public static SKColor GetBaseColor(float darkness) => DimColor(new SKColor(86, 105, 64), darkness);

    /// <summary>
    /// Draws the terrain detail tiers for the visible world rect. The canvas must already be
    /// under the camera world matrix, and the flat base must already have been cleared to
    /// <see cref="GetBaseColor"/>. Draws nothing below zoom 0.15 (schematic view). Only tiles
    /// intersecting <paramref name="viewRect"/> are visited; missing tiles are generated
    /// lazily and cached.
    /// </summary>
    public void Draw(SKCanvas canvas, SKRect viewRect, float zoom, float darkness)
    {
        if (zoom < 0.15f) return;

        DrawPatchTier(canvas, viewRect, darkness, _superTiles, SuperTileSize, SuperInflate, superTier: true);
        if (zoom >= 0.6f)
            DrawPatchTier(canvas, viewRect, darkness, _detailTiles, DetailTileSize, DetailInflate, superTier: false);
        if (zoom >= 2.0f)
            DrawSpeckle(canvas, viewRect, darkness);
    }

    /// <summary>
    /// Draws one cached patch tier: iterates the integer tile range covering
    /// <paramref name="viewRect"/> inflated by the tier's maximum patch overhang, builds and
    /// caches any missing tile, and fills each patch whose bounds intersect the view with its
    /// darkness-dimmed color.
    /// </summary>
    private void DrawPatchTier(SKCanvas canvas, SKRect viewRect, float darkness,
        Dictionary<long, Patch[]> cache, float tileSize, float inflate, bool superTier)
    {
        int x0 = (int)MathF.Floor((viewRect.Left - inflate) / tileSize);
        int x1 = (int)MathF.Floor((viewRect.Right + inflate) / tileSize);
        int y0 = (int)MathF.Floor((viewRect.Top - inflate) / tileSize);
        int y1 = (int)MathF.Floor((viewRect.Bottom + inflate) / tileSize);

        for (int ty = y0; ty <= y1; ty++)
        {
            for (int tx = x0; tx <= x1; tx++)
            {
                long key = TileKey(tx, ty);
                if (!cache.TryGetValue(key, out var patches))
                {
                    if (cache.Count >= MaxCachedTiles) EvictAll(cache);
                    patches = superTier ? BuildSuperTile(tx, ty) : BuildDetailTile(tx, ty);
                    cache[key] = patches;
                }

                for (int i = 0; i < patches.Length; i++)
                {
                    ref var p = ref patches[i];
                    if (!p.Bounds.IntersectsWith(viewRect)) continue;
                    _patchPaint.Color = DimColor(p.Color, darkness);
                    canvas.DrawPath(p.Path, _patchPaint);
                }
            }
        }
    }

    /// <summary>Builds the 1–2 very large opaque mottle blobs (radius 120–300m) for one 512m
    /// supertile, in slightly darker or lighter grass than the base.</summary>
    private static Patch[] BuildSuperTile(int tx, int ty)
    {
        uint seed = TileSeed(tx, ty, SuperSalt);
        int count = 1 + (int)(Hash01(seed, 0) * 2f); // 1-2
        var patches = new Patch[count];
        for (int k = 0; k < count; k++)
        {
            uint salt = (uint)(100 * (k + 1));
            float cx = (tx + Hash01(seed, salt + 1)) * SuperTileSize;
            float cy = (ty + Hash01(seed, salt + 2)) * SuperTileSize;
            float radius = 120f + Hash01(seed, salt + 3) * 180f;
            var color = Hash01(seed, salt + 4) < 0.5f
                ? new SKColor(78, 96, 57)   // slightly darker grass
                : new SKColor(94, 114, 71); // slightly lighter grass
            patches[k] = BuildBlobPatch(cx, cy, radius, color, seed, salt + 5);
        }
        return patches;
    }

    /// <summary>Builds the 2–5 opaque grass/dirt patches (radius 15–55m) for one 128m detail
    /// tile, choosing colors from a small muted palette (dirt at ~10% probability).</summary>
    private static Patch[] BuildDetailTile(int tx, int ty)
    {
        uint seed = TileSeed(tx, ty, DetailSalt);
        int count = 2 + (int)(Hash01(seed, 0) * 4f); // 2-5
        var patches = new Patch[count];
        for (int k = 0; k < count; k++)
        {
            uint salt = (uint)(100 * (k + 1));
            float cx = (tx + Hash01(seed, salt + 1)) * DetailTileSize;
            float cy = (ty + Hash01(seed, salt + 2)) * DetailTileSize;
            float radius = 15f + Hash01(seed, salt + 3) * 40f;
            float roll = Hash01(seed, salt + 4);
            var color =
                roll < 0.10f ? new SKColor(109, 94, 66) :   // dirt
                roll < 0.40f ? new SKColor(76, 94, 56) :    // dark grass
                roll < 0.70f ? new SKColor(101, 112, 68) :  // light dry grass
                               new SKColor(98, 110, 60);    // meadow
            patches[k] = BuildBlobPatch(cx, cy, radius, color, seed, salt + 5);
        }
        return patches;
    }

    /// <summary>
    /// Builds one irregular organic closed blob: 6–9 vertices at jittered angles and radii
    /// (0.6–1.0 × <paramref name="maxRadius"/>, so the blob never exceeds the nominal radius),
    /// smoothed with quadratic segments through edge midpoints.
    /// </summary>
    private static Patch BuildBlobPatch(float cx, float cy, float maxRadius, SKColor color, uint seed, uint salt)
    {
        int n = 6 + (int)(Hash01(seed, salt) * 4f); // 6-9 points
        Span<SKPoint> pts = stackalloc SKPoint[9];
        float step = MathF.Tau / n;
        for (int i = 0; i < n; i++)
        {
            float ang = i * step + (Hash01(seed, salt + 10u + (uint)i) - 0.5f) * step * 0.7f;
            float r = maxRadius * (0.6f + 0.4f * Hash01(seed, salt + 40u + (uint)i));
            pts[i] = new SKPoint(cx + MathF.Cos(ang) * r, cy + MathF.Sin(ang) * r);
        }

        var path = new SKPath();
        path.MoveTo(Mid(pts[n - 1], pts[0]));
        for (int i = 0; i < n; i++)
            path.QuadTo(pts[i], Mid(pts[i], pts[(i + 1) % n]));
        path.Close();

        return new Patch { Path = path, Bounds = path.Bounds, Color = color };
    }

    /// <summary>
    /// Draws sparse ~1m grass flecks on a jittered lattice covering <paramref name="viewRect"/>,
    /// per frame (not cached — placement is deterministic per lattice cell). The lattice spacing
    /// starts at 24m and is widened when the view area would otherwise exceed
    /// <see cref="MaxSpecklesPerFrame"/> flecks.
    /// </summary>
    private void DrawSpeckle(SKCanvas canvas, SKRect viewRect, float darkness)
    {
        float spacing = SpeckleSpacing;
        float area = viewRect.Width * viewRect.Height;
        if (area / (spacing * spacing) > MaxSpecklesPerFrame)
            spacing = MathF.Sqrt(area / MaxSpecklesPerFrame);

        SKColor light = DimColor(new SKColor(118, 132, 82), darkness).WithAlpha(30);
        SKColor dark = DimColor(new SKColor(66, 82, 48), darkness).WithAlpha(30);

        int x0 = (int)MathF.Floor(viewRect.Left / spacing);
        int x1 = (int)MathF.Floor(viewRect.Right / spacing);
        int y0 = (int)MathF.Floor(viewRect.Top / spacing);
        int y1 = (int)MathF.Floor(viewRect.Bottom / spacing);

        for (int iy = y0; iy <= y1; iy++)
        {
            for (int ix = x0; ix <= x1; ix++)
            {
                uint seed = TileSeed(ix, iy, SpeckleSalt);
                if (Hash01(seed, 0) < 0.25f) continue; // keep the lattice sparse
                float px = (ix + Hash01(seed, 1)) * spacing;
                float py = (iy + Hash01(seed, 2)) * spacing;
                float ang = Hash01(seed, 3) * MathF.Tau;
                float len = 0.7f + Hash01(seed, 4) * 0.6f;
                _speckPaint.Color = Hash01(seed, 5) < 0.5f ? light : dark;
                canvas.DrawLine(px, py, px + MathF.Cos(ang) * len, py + MathF.Sin(ang) * len, _speckPaint);
            }
        }
    }

    /// <summary>Disposes every cached patch path in the tier and clears the dictionary.
    /// Regeneration on the next frame is cheap.</summary>
    private static void EvictAll(Dictionary<long, Patch[]> cache)
    {
        foreach (var patches in cache.Values)
            for (int i = 0; i < patches.Length; i++)
                patches[i].Path.Dispose();
        cache.Clear();
    }

    /// <summary>
    /// Applies the shared day/night formula to an undimmed base color: lerp toward night
    /// RGB(15,17,26) by <c>darkness * 0.85</c>, plus a warm dawn/dusk bias (+12R, −3G, −10B)
    /// scaled by a factor that peaks mid-transition (mirrors SceneRenderer.GetEnvironmentColors).
    /// </summary>
    private static SKColor DimColor(SKColor c, float darkness)
    {
        darkness = Math.Clamp(darkness, 0f, 1f);
        float t = darkness * 0.85f;
        float warmth = (darkness > 0f && darkness < 1f)
            ? 1f - MathF.Abs(darkness * 2f - 1f)
            : 0f;
        float r = c.Red + (15f - c.Red) * t + warmth * 12f;
        float g = c.Green + (17f - c.Green) * t - warmth * 3f;
        float b = c.Blue + (26f - c.Blue) * t - warmth * 10f;
        return new SKColor(ClampByte(r), ClampByte(g), ClampByte(b));
    }

    private static byte ClampByte(float v) => (byte)Math.Clamp((int)v, 0, 255);

    private static SKPoint Mid(SKPoint a, SKPoint b) => new((a.X + b.X) * 0.5f, (a.Y + b.Y) * 0.5f);

    private static long TileKey(int tx, int ty) => ((long)tx << 32) ^ (uint)ty;

    /// <summary>Per-tile hash seed combining both tile coordinates with a tier salt so the
    /// three tiers draw from independent deterministic streams.</summary>
    private static uint TileSeed(int tx, int ty, uint tierSalt)
        => Hash(Hash(unchecked((uint)tx), unchecked((uint)ty)), tierSalt);

    private static uint Hash(uint x)
    {
        unchecked { x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16; return x; }
    }

    private static uint Hash(uint a, uint b)
    {
        unchecked { return Hash(a * 0x9E3779B9u ^ b); }
    }

    private static float Hash01(uint a, uint b) => Hash(a, b) * (1f / 4294967296f);
}
