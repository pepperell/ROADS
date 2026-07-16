using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Draws the painted water layer (brush circles + stream segments) as part of the
/// ground plane: called right after the terrain pass and before roads, so roads
/// crossing water read as bridges/culverts. Two opaque passes — a shore band
/// (every primitive expanded a few meters in a sand color) then the water fill —
/// so overlapping primitives merge into one seamless body with a coherent
/// shoreline, no layer compositing needed. Both colors run through
/// <see cref="TerrainRenderer.DimColor"/> so water tints with the land day/night.
/// Unlike the terrain detail tiers there is no zoom floor: water is primary map
/// content and only drops sub-pixel primitives.
/// </summary>
public class WaterRenderer
{
    /// <summary>Shore band width in meters added around every primitive in pass 0.</summary>
    private const float ShoreBand = 2.5f;

    /// <summary>Primitives smaller than this on screen (pixels) are skipped.</summary>
    private const float MinPixelSize = 0.5f;

    private static readonly SKColor WaterBase = new(62, 102, 134);
    private static readonly SKColor ShoreBase = new(126, 116, 86);

    private readonly WaterLayer _water;

    private readonly SKPaint _fillPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _strokePaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true,
    };
    private readonly SKPath _segmentPath = new();
    private readonly List<int> _visibleCircles = new();
    private readonly List<int> _visibleSegments = new();

    public WaterRenderer(WaterLayer water)
    {
        _water = water;
    }

    /// <summary>
    /// Draws the visible water primitives. The canvas must already be under the camera
    /// world matrix; <paramref name="zoom"/> is pixels per world meter.
    /// </summary>
    public void Draw(SKCanvas canvas, SKRect viewRect, float zoom, float darkness)
    {
        if (_water.IsEmpty) return;

        _water.QueryVisible(viewRect.Left, viewRect.Top, viewRect.Right, viewRect.Bottom,
            _visibleCircles, _visibleSegments);
        if (_visibleCircles.Count == 0 && _visibleSegments.Count == 0) return;

        DrawPass(canvas, zoom, TerrainRenderer.DimColor(ShoreBase, darkness), ShoreBand);
        DrawPass(canvas, zoom, TerrainRenderer.DimColor(WaterBase, darkness), 0f);
    }

    /// <summary>One opaque pass over the visible primitives, expanded by <paramref name="expand"/> meters.</summary>
    private void DrawPass(SKCanvas canvas, float zoom, SKColor color, float expand)
    {
        _fillPaint.Color = color;
        _strokePaint.Color = color;

        var circles = _water.Circles;
        foreach (int i in _visibleCircles)
        {
            var c = circles[i];
            float r = c.Radius + expand;
            if (2f * r * zoom < MinPixelSize) continue;
            canvas.DrawCircle(c.Center.X, c.Center.Y, r, _fillPaint);
        }

        var segments = _water.Segments;
        foreach (int i in _visibleSegments)
        {
            var s = segments[i];
            float w = s.Width + 2f * expand;
            if (w * zoom < MinPixelSize) continue;
            _strokePaint.StrokeWidth = w;
            _segmentPath.Reset();
            _segmentPath.MoveTo(s.P0.X, s.P0.Y);
            _segmentPath.CubicTo(s.C1.X, s.C1.Y, s.C2.X, s.C2.Y, s.P3.X, s.P3.Y);
            canvas.DrawPath(_segmentPath, _strokePaint);
        }
    }
}
