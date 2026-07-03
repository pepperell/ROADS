using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The filled/stacked frame-time bar inside the performance HUD: sim (blue), draw
/// (orange), and idle (gray) segments proportional to the current frame's breakdown,
/// clipped to a rounded bar with an outline. Reads live values from
/// <see cref="PerfTelemetry"/> each draw.
/// </summary>
public class PerformanceBar : Panel
{
    private readonly PerfTelemetry _telemetry;

    private readonly SKPaint _simPaint = new() { Color = new SKColor(80, 140, 220), Style = SKPaintStyle.Fill };
    private readonly SKPaint _drawPaint = new() { Color = new SKColor(220, 160, 50), Style = SKPaintStyle.Fill };
    private readonly SKPaint _idlePaint = new() { Color = new SKColor(60, 62, 68), Style = SKPaintStyle.Fill };
    private readonly SKPaint _outlinePaint = new() { Color = new SKColor(80, 82, 88), Style = SKPaintStyle.Stroke, StrokeWidth = 1f };

    public PerformanceBar(PerfTelemetry telemetry)
    {
        _telemetry = telemetry;
        HitTestSelf = false; // decoration inside the HUD; the HUD panel consumes input
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        double frame = _telemetry.LastFrameMs;
        double sim = _telemetry.AvgSimMs;
        double draw = _telemetry.AvgDrawMs;
        double idle = Math.Max(0, frame - sim - draw);

        if (frame > 0)
        {
            float simFrac = (float)(sim / frame);
            float drawFrac = (float)(draw / frame);
            float idleFrac = (float)(idle / frame);
            float total = simFrac + drawFrac + idleFrac;
            if (total > 0)
            {
                simFrac /= total;
                drawFrac /= total;
                idleFrac /= total;
            }

            float simW = simFrac * Bounds.Width;
            float drawW = drawFrac * Bounds.Width;
            float idleW = idleFrac * Bounds.Width;

            canvas.Save();
            canvas.ClipRoundRect(new SKRoundRect(Bounds, 3f));
            canvas.DrawRect(Bounds.Left, Bounds.Top, simW, Bounds.Height, _simPaint);
            canvas.DrawRect(Bounds.Left + simW, Bounds.Top, drawW, Bounds.Height, _drawPaint);
            canvas.DrawRect(Bounds.Left + simW + drawW, Bounds.Top, idleW, Bounds.Height, _idlePaint);
            canvas.Restore();
        }
        else
        {
            canvas.DrawRoundRect(Bounds, 3f, 3f, _idlePaint);
        }

        canvas.DrawRoundRect(Bounds, 3f, 3f, _outlinePaint);
    }
}
