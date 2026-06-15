using System.Diagnostics;
using Roads.App.World;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Displays a performance HUD with FPS, vehicle count, and a linear stacked bar graph
/// showing frame time breakdown: simulation (blue), drawing (orange), and idle (gray).
/// Values are instantaneous per-frame (no averaging), so the readout reflects the current
/// frame immediately — it does not ramp while a buffer refills after an unpause.
/// Also reads pathfinding timing from <see cref="Pathfinder.ReadPathfindStatsAndReset"/>
/// once per HUD update (the HUD is the single consumer/resetter of those accumulators).
/// </summary>
public class PerformanceHud
{
    // Instantaneous per-frame metrics — deliberately NOT a rolling average, so the HUD reflects
    // the true current cost immediately after an unpause instead of ramping while a buffer refills.
    private double _lastSim;
    private double _lastDraw;
    private double _lastFrame;
    private long _lastFrameTick;

    // Last-read pathfind snapshot (consumed once per Draw call).
    private double _lastPathfindMs;
    private int _lastPathfindCalls;

    private readonly Stopwatch _stopwatch = Stopwatch.StartNew();

    // Reusable paints
    private readonly SKPaint _bgPaint = new() { Color = new SKColor(20, 22, 28, 200), Style = SKPaintStyle.Fill };
    private readonly SKPaint _simPaint = new() { Color = new SKColor(80, 140, 220), Style = SKPaintStyle.Fill };
    private readonly SKPaint _drawPaint = new() { Color = new SKColor(220, 160, 50), Style = SKPaintStyle.Fill };
    private readonly SKPaint _idlePaint = new() { Color = new SKColor(60, 62, 68), Style = SKPaintStyle.Fill };
    private readonly SKPaint _outlinePaint = new() { Color = new SKColor(80, 82, 88), Style = SKPaintStyle.Stroke, StrokeWidth = 1f };
    private readonly SKPaint _headerPaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _simTextPaint = new() { Color = new SKColor(80, 140, 220), IsAntialias = true };
    private readonly SKPaint _drawTextPaint = new() { Color = new SKColor(220, 160, 50), IsAntialias = true };
    private readonly SKPaint _idleTextPaint = new() { Color = new SKColor(120, 120, 120), IsAntialias = true };
    private readonly SKPaint _pathfindTextPaint = new() { Color = new SKColor(160, 220, 140), IsAntialias = true };
    private readonly SKFont _font = new() { Size = 11 };

    /// <summary>Whether the performance HUD is visible.</summary>
    public bool Visible { get; set; }

    // ---------------------------------------------------------------------------
    // Public read-only accessors — expose the same instantaneous per-frame values
    // shown in the HUD. Callers (e.g. BenchmarkCapture) read these after Draw() has
    // run for the frame so the values are current. (Named "Avg*" for source
    // compatibility; they are last-frame values, not averages.)
    // ---------------------------------------------------------------------------

    /// <summary>Most recent frame's FPS (instantaneous, not averaged).</summary>
    public double AvgFps { get; private set; }

    /// <summary>Most recent frame's simulation time in milliseconds (instantaneous).</summary>
    public double AvgSimMs { get; private set; }

    /// <summary>Most recent frame's draw time in milliseconds (instantaneous).</summary>
    public double AvgDrawMs { get; private set; }

    /// <summary>
    /// Pathfinding time in milliseconds for the last frame, as read by <see cref="Draw"/>.
    /// The HUD is the single consumer/resetter of the pathfind accumulators; callers must
    /// read this after <see cref="Draw"/> has been called for the frame.
    /// </summary>
    public double LastPathfindMs => _lastPathfindMs;

    /// <summary>
    /// Number of pathfinder calls in the last frame, as read by <see cref="Draw"/>.
    /// The HUD is the single consumer/resetter of the pathfind accumulators; callers must
    /// read this after <see cref="Draw"/> has been called for the frame.
    /// </summary>
    public int LastPathfindCalls => _lastPathfindCalls;

    /// <summary>Records the simulation time for the current frame in milliseconds.</summary>
    public void RecordSimTime(double milliseconds)
    {
        _lastSim = milliseconds;
    }

    /// <summary>
    /// Records the draw time for the current frame in milliseconds and computes the
    /// total frame time from wall clock. No averaging — values are per-frame.
    /// </summary>
    public void RecordDrawTime(double milliseconds)
    {
        _lastDraw = milliseconds;

        long now = _stopwatch.ElapsedTicks;
        if (_lastFrameTick > 0)
            _lastFrame = (now - _lastFrameTick) * 1000.0 / Stopwatch.Frequency;
        _lastFrameTick = now;
    }

    /// <summary>
    /// Draws the performance HUD in screen space at the bottom-left corner.
    /// Shows FPS, vehicle count, per-category millisecond values, pathfinding stats,
    /// GC collection counts, and a single stacked bar colored by sim/draw/idle proportions.
    /// Reads pathfinding stats from <see cref="Pathfinder.ReadPathfindStatsAndReset"/> once
    /// per call — the HUD is the single consumer/resetter of those accumulators.
    /// Also updates <see cref="AvgFps"/>, <see cref="AvgSimMs"/>, <see cref="AvgDrawMs"/>
    /// so callers may read them after this method returns.
    /// </summary>
    public void Draw(SKCanvas canvas, int vehicleCount, float canvasWidth, float canvasHeight)
    {
        // Always consume pathfind stats each frame to keep the accumulators from growing
        // unboundedly, even when the HUD is hidden.
        var (pathfindMs, pathfindCalls) = Pathfinder.ReadPathfindStatsAndReset();
        _lastPathfindMs = pathfindMs;
        _lastPathfindCalls = pathfindCalls;

        // Instantaneous per-frame values (no averaging — reflects the current frame immediately).
        double avgSim = _lastSim;
        double avgDraw = _lastDraw;
        double avgFrame = _lastFrame;

        double fps = avgFrame > 0 ? 1000.0 / avgFrame : 0;

        // Publish current-frame values for external readers (e.g. BenchmarkCapture).
        AvgFps = fps;
        AvgSimMs = avgSim;
        AvgDrawMs = avgDraw;

        if (!Visible) return;

        double idleMs = Math.Max(0, avgFrame - avgSim - avgDraw);

        // Read GC collection counts
        int gc0 = GC.CollectionCount(0);
        int gc1 = GC.CollectionCount(1);
        int gc2 = GC.CollectionCount(2);

        // Layout constants
        const float barWidth = 240f;
        const float barHeight = 14f;
        const float padding = 8f;
        const float panelWidth = barWidth + padding * 2;
        const float panelHeight = 100f;  // extra height for pathfind + GC lines
        float px = 10f;
        float py = canvasHeight - panelHeight - 10f;

        // Background panel
        canvas.DrawRoundRect(px, py, panelWidth, panelHeight, 4f, 4f, _bgPaint);

        // Header: FPS | Vehicles | Frame time
        float textY = py + padding + 10f;
        canvas.DrawText($"FPS: {fps:F0}  |  Vehicles: {vehicleCount}  |  Frame: {avgFrame:F1}ms",
            px + padding, textY, SKTextAlign.Left, _font, _headerPaint);

        // Category labels with colored text
        textY += 16f;
        float labelX = px + padding;
        canvas.DrawText($"Sim: {avgSim:F1}ms", labelX, textY, SKTextAlign.Left, _font, _simTextPaint);
        canvas.DrawText($"Draw: {avgDraw:F1}ms", labelX + 80, textY, SKTextAlign.Left, _font, _drawTextPaint);
        canvas.DrawText($"Idle: {idleMs:F1}ms", labelX + 170, textY, SKTextAlign.Left, _font, _idleTextPaint);

        // Pathfind stats line (green-ish text)
        textY += 16f;
        canvas.DrawText($"Path: {_lastPathfindMs:F2}ms  calls: {_lastPathfindCalls}",
            labelX, textY, SKTextAlign.Left, _font, _pathfindTextPaint);

        // GC collection counts line
        textY += 14f;
        canvas.DrawText($"GC  gen0: {gc0}  gen1: {gc1}  gen2: {gc2}",
            labelX, textY, SKTextAlign.Left, _font, _idleTextPaint);

        // Stacked bar graph
        float barX = px + padding;
        float barY = textY + 6f;

        if (avgFrame > 0)
        {
            float simFrac = (float)(avgSim / avgFrame);
            float drawFrac = (float)(avgDraw / avgFrame);
            float idleFrac = (float)(idleMs / avgFrame);

            // Normalize so fractions sum to 1
            float total = simFrac + drawFrac + idleFrac;
            if (total > 0)
            {
                simFrac /= total;
                drawFrac /= total;
                idleFrac /= total;
            }

            float simW = simFrac * barWidth;
            float drawW = drawFrac * barWidth;
            float idleW = idleFrac * barWidth;

            // Clip to rounded bar shape
            canvas.Save();
            canvas.ClipRoundRect(new SKRoundRect(new SKRect(barX, barY, barX + barWidth, barY + barHeight), 3f));

            canvas.DrawRect(barX, barY, simW, barHeight, _simPaint);
            canvas.DrawRect(barX + simW, barY, drawW, barHeight, _drawPaint);
            canvas.DrawRect(barX + simW + drawW, barY, idleW, barHeight, _idlePaint);

            canvas.Restore();
        }
        else
        {
            canvas.DrawRoundRect(barX, barY, barWidth, barHeight, 3f, 3f, _idlePaint);
        }

        // Bar outline
        canvas.DrawRoundRect(barX, barY, barWidth, barHeight, 3f, 3f, _outlinePaint);
    }
}
