using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Draws the large "ROADS" title-screen logo: each letter is a hand-authored road
/// skeleton — thick asphalt strokes with a dark casing rim, a dashed yellow centerline,
/// and small paved-intersection patches with white dots where strokes meet (the O is a
/// ring road, the S a winding street). Letters are authored as centerline polylines in a
/// normalized space (y 0 = top to 1 = baseline, per-letter widths, laid side by side into
/// one word ≈ 4.04 × 1.0) and built once into a single combined path; <see cref="Draw"/>
/// fits that word into the given rect with a uniform canvas transform, so all stroke
/// widths and dash intervals are normalized constants that scale automatically.
/// Colors echo the road renderer (residential asphalt, centerline yellow). All paints are
/// process-lifetime and used only on the single render thread (the UiTheme discipline).
/// </summary>
public static class RoadsLogo
{
    private const float TotalWidth = 4.04f; // sum of letter widths + 4 × 0.22 spacing
    private const float CasingWidth = 0.30f;
    private const float AsphaltWidth = 0.24f;
    private const float CenterlineWidth = 0.035f;
    private const float CasingPatchRadius = 0.17f;
    private const float AsphaltPatchRadius = 0.14f;
    private const float DotRadius = 0.05f;

    /// <summary>Every letter's centerline skeleton, pre-translated into word space.</summary>
    private static readonly SKPath Skeleton = new();
    /// <summary>Points where 3+ strokes meet (plus the O's compass points) — drawn as
    /// paved intersection patches with a white dot.</summary>
    private static readonly SKPoint[] Junctions;

    private static readonly SKPaint CasingStroke = new()
    {
        Style = SKPaintStyle.Stroke, StrokeWidth = CasingWidth, Color = new SKColor(30, 32, 36),
        StrokeCap = SKStrokeCap.Round, StrokeJoin = SKStrokeJoin.Round, IsAntialias = true,
    };
    private static readonly SKPaint AsphaltStroke = new()
    {
        Style = SKPaintStyle.Stroke, StrokeWidth = AsphaltWidth, Color = new SKColor(82, 84, 88),
        StrokeCap = SKStrokeCap.Round, StrokeJoin = SKStrokeJoin.Round, IsAntialias = true,
    };
    private static readonly SKPaint CenterlineStroke = new()
    {
        Style = SKPaintStyle.Stroke, StrokeWidth = CenterlineWidth, Color = new SKColor(200, 166, 60),
        PathEffect = SKPathEffect.CreateDash(new[] { 0.10f, 0.075f }, 0f), IsAntialias = true,
    };
    private static readonly SKPaint CasingFill = new()
    { Style = SKPaintStyle.Fill, Color = new SKColor(30, 32, 36), IsAntialias = true };
    private static readonly SKPaint AsphaltFill = new()
    { Style = SKPaintStyle.Fill, Color = new SKColor(82, 84, 88), IsAntialias = true };
    private static readonly SKPaint DotFill = new()
    { Style = SKPaintStyle.Fill, Color = new SKColor(200, 200, 200), IsAntialias = true };

    static RoadsLogo()
    {
        var junctions = new List<SKPoint>();

        void Polyline(float xOff, params (float x, float y)[] pts)
        {
            Skeleton.MoveTo(xOff + pts[0].x, pts[0].y);
            for (int i = 1; i < pts.Length; i++)
                Skeleton.LineTo(xOff + pts[i].x, pts[i].y);
        }
        void Junction(float xOff, float x, float y) => junctions.Add(new SKPoint(xOff + x, y));

        // R (width 0.62): spine, bowl, leg.
        float x0 = 0f;
        Polyline(x0, (0f, 0f), (0f, 1f));
        Polyline(x0, (0f, 0f), (0.42f, 0f), (0.58f, 0.10f), (0.58f, 0.34f), (0.42f, 0.46f), (0f, 0.46f));
        Polyline(x0, (0.30f, 0.46f), (0.62f, 1f));
        Junction(x0, 0f, 0f);
        Junction(x0, 0f, 0.46f);
        Junction(x0, 0.30f, 0.46f);

        // O (width 0.66): a ring road with side-street stubs marked at the compass points.
        x0 = 0.84f;
        Skeleton.AddOval(SKRect.Create(x0, 0.02f, 0.66f, 0.96f));
        Junction(x0, 0.33f, 0.02f);
        Junction(x0, 0.66f, 0.50f);
        Junction(x0, 0.33f, 0.98f);
        Junction(x0, 0f, 0.50f);

        // A (width 0.66): two diagonals meeting at the apex, plus the crossbar.
        x0 = 1.72f;
        Polyline(x0, (0f, 1f), (0.33f, 0f));
        Polyline(x0, (0.33f, 0f), (0.66f, 1f));
        Polyline(x0, (0.13f, 0.62f), (0.53f, 0.62f));
        Junction(x0, 0.33f, 0f);
        Junction(x0, 0.13f, 0.62f);
        Junction(x0, 0.53f, 0.62f);

        // D (width 0.62): spine and bowl.
        x0 = 2.60f;
        Polyline(x0, (0f, 0f), (0f, 1f));
        Polyline(x0, (0f, 0f), (0.34f, 0f), (0.56f, 0.14f), (0.62f, 0.50f), (0.56f, 0.86f), (0.34f, 1f), (0f, 1f));
        Junction(x0, 0f, 0f);
        Junction(x0, 0f, 1f);

        // S (width 0.60): one winding street, no junctions.
        x0 = 3.44f;
        Polyline(x0, (0.58f, 0.10f), (0.20f, 0.10f), (0.02f, 0.22f), (0.02f, 0.38f), (0.20f, 0.50f),
            (0.40f, 0.50f), (0.58f, 0.62f), (0.58f, 0.78f), (0.40f, 0.90f), (0.02f, 0.90f));

        Junctions = junctions.ToArray();
    }

    /// <summary>Draws the logo fitted (uniform scale, centered) into <paramref name="area"/>.</summary>
    public static void Draw(SKCanvas canvas, SKRect area)
    {
        // The casing overflows the skeleton box by half its width on every side.
        float scale = MathF.Min(area.Width / (TotalWidth + CasingWidth), area.Height / (1f + CasingWidth));
        if (scale <= 0f) return;

        canvas.Save();
        canvas.Translate(area.MidX - TotalWidth * scale / 2f, area.MidY - scale / 2f);
        canvas.Scale(scale);

        canvas.DrawPath(Skeleton, CasingStroke);
        foreach (var j in Junctions) canvas.DrawCircle(j, CasingPatchRadius, CasingFill);
        canvas.DrawPath(Skeleton, AsphaltStroke);
        foreach (var j in Junctions) canvas.DrawCircle(j, AsphaltPatchRadius, AsphaltFill);
        canvas.DrawPath(Skeleton, CenterlineStroke);
        foreach (var j in Junctions) canvas.DrawCircle(j, DotRadius, DotFill);

        canvas.Restore();
    }
}
