using SkiaSharp;
using Roads.App.Vehicles;

namespace Roads.App.Rendering;

/// <summary>
/// Displays a live statistics panel showing vehicle count, average speed, and congestion
/// (percentage of vehicles travelling below <see cref="CongestionThresholdMs"/> m/s).
/// Stats are computed read-only from <see cref="VehicleStore"/> on every draw call;
/// no simulation-tick work is added. Positioned above the <see cref="PerformanceHud"/>
/// at the bottom-left so the two panels do not overlap.
/// </summary>
public class StatisticsPanel
{
    /// <summary>
    /// Speed threshold in m/s below which a vehicle is counted as congested.
    /// 2 m/s (~4.5 mph) represents near-standstill / heavy stop-and-go traffic.
    /// </summary>
    public const float CongestionThresholdMs = 2f;

    /// <summary>Whether the statistics panel is visible.</summary>
    public bool Visible { get; set; }

    // Reusable paints — same palette as PerformanceHud for visual consistency.
    private readonly SKPaint _bgPaint       = new() { Color = new SKColor(20, 22, 28, 200), Style = SKPaintStyle.Fill };
    private readonly SKPaint _outlinePaint  = new() { Color = new SKColor(80, 82, 88),      Style = SKPaintStyle.Stroke, StrokeWidth = 1f };
    private readonly SKPaint _headerPaint   = new() { Color = new SKColor(200, 200, 200),   IsAntialias = true };
    private readonly SKPaint _valuePaint    = new() { Color = new SKColor(100, 200, 255),   IsAntialias = true };
    private readonly SKPaint _warnPaint     = new() { Color = new SKColor(220, 160, 50),    IsAntialias = true };
    private readonly SKPaint _alertPaint    = new() { Color = new SKColor(220, 80, 60),     IsAntialias = true };
    private readonly SKFont  _font          = new() { Size = 11 };

    /// <summary>
    /// Draws the statistics panel in screen space, positioned directly above the
    /// <see cref="PerformanceHud"/> (which occupies the bottom-left at y = canvasHeight - 78).
    /// Computes vehicle count, average speed, and congestion ratio from
    /// <paramref name="vehicles"/> without mutating any state.
    /// </summary>
    /// <param name="canvas">Skia canvas; must be in screen (identity) space.</param>
    /// <param name="vehicles">Vehicle store — read only.</param>
    /// <param name="canvasWidth">Canvas width in pixels.</param>
    /// <param name="canvasHeight">Canvas height in pixels.</param>
    public void Draw(SKCanvas canvas, VehicleStore vehicles, float canvasWidth, float canvasHeight)
    {
        if (!Visible) return;

        // ── Compute stats (read-only) ─────────────────────────────────────────────
        int count = vehicles.Count;
        float avgSpeed = 0f;
        int congested = 0;

        if (count > 0)
        {
            float sum = 0f;
            var speeds = vehicles.Speed;
            for (int i = 0; i < count; i++)
            {
                float s = speeds[i];
                sum += s;
                if (s < CongestionThresholdMs)
                    congested++;
            }
            avgSpeed = sum / count;
        }

        float congestionPct = count > 0 ? (congested * 100f / count) : 0f;

        // Convert speed from m/s to mph for display
        float avgSpeedMph = avgSpeed * 2.23694f;

        // ── Layout constants ──────────────────────────────────────────────────────
        // PerformanceHud sits at canvasHeight - 68 - 10 (height=68, margin=10).
        // Place this panel directly above it with a 4px gap.
        const float panelWidth  = 256f;
        const float panelHeight = 66f;
        const float padding     = 8f;
        const float perfHudHeight = 68f;
        const float margin      = 10f;
        const float gap         = 4f;

        float px = margin;
        float py = canvasHeight - perfHudHeight - panelHeight - margin - gap;

        // ── Background panel ──────────────────────────────────────────────────────
        canvas.DrawRoundRect(px, py, panelWidth, panelHeight, 4f, 4f, _bgPaint);
        canvas.DrawRoundRect(px, py, panelWidth, panelHeight, 4f, 4f, _outlinePaint);

        // ── Row 1: header label ───────────────────────────────────────────────────
        float textY = py + padding + 10f;
        canvas.DrawText("STATISTICS", px + padding, textY, SKTextAlign.Left, _font, _headerPaint);

        // ── Row 2: vehicle count + avg speed ─────────────────────────────────────
        textY += 16f;
        canvas.DrawText($"Vehicles: ", px + padding, textY, SKTextAlign.Left, _font, _headerPaint);
        canvas.DrawText($"{count}", px + padding + 60f, textY, SKTextAlign.Left, _font, _valuePaint);

        canvas.DrawText($"Avg Speed: ", px + padding + 100f, textY, SKTextAlign.Left, _font, _headerPaint);
        canvas.DrawText($"{avgSpeedMph:F1} mph", px + padding + 166f, textY, SKTextAlign.Left, _font, _valuePaint);

        // ── Row 3: congestion ─────────────────────────────────────────────────────
        textY += 16f;
        canvas.DrawText("Congestion: ", px + padding, textY, SKTextAlign.Left, _font, _headerPaint);

        // Color the congestion value: blue (<25%), amber (25–60%), red (>60%)
        var congPaint = congestionPct >= 60f ? _alertPaint
                      : congestionPct >= 25f ? _warnPaint
                      : _valuePaint;
        canvas.DrawText($"{congestionPct:F1}%", px + padding + 72f, textY, SKTextAlign.Left, _font, congPaint);

        canvas.DrawText($"({congested} slow)", px + padding + 118f, textY, SKTextAlign.Left, _font, _headerPaint);
    }
}
