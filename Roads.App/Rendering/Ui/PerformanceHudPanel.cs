using SkiaSharp;
using Roads.App.Vehicles;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The performance HUD: FPS / vehicle count / frame time, per-category millisecond rows,
/// pathfind stats, GC collection counts, and the stacked <see cref="PerformanceBar"/>,
/// under a PERFORMANCE title row (matching the statistics panel). Shown by default; the
/// P key toggles it. Purely a VIEW over <see cref="PerfTelemetry"/> — all
/// measurement and the pathfind-accumulator drain live there and run whether or not this
/// panel is visible. The panel is <see cref="Panel.ExternallyDrawn"/>: the UiRoot lays it
/// out (inside the bottom-left stack) and it consumes clicks like any panel, but MainForm
/// draws it after <see cref="PerfTelemetry.RecordDrawTime"/> so (a) it renders above every
/// other overlay, as before the migration, and (b) its own draw cost stays outside the
/// measured draw window, keeping benchmark.log comparable across the refactor.
/// </summary>
public class PerformanceHudPanel : Panel
{
    private const float Pad = 8f;
    private const float BarWidth = 240f;
    private const float BarHeight = 14f;

    private readonly PerfTelemetry _telemetry;
    private readonly VehicleStore _vehicles;
    private readonly PerformanceBar _bar;

    private readonly SKPaint _headerPaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _simTextPaint = new() { Color = new SKColor(80, 140, 220), IsAntialias = true };
    private readonly SKPaint _drawTextPaint = new() { Color = new SKColor(220, 160, 50), IsAntialias = true };
    private readonly SKPaint _idleTextPaint = new() { Color = new SKColor(120, 120, 120), IsAntialias = true };
    private readonly SKPaint _pathfindTextPaint = new() { Color = new SKColor(160, 220, 140), IsAntialias = true };

    public PerformanceHudPanel(PerfTelemetry telemetry, VehicleStore vehicles)
    {
        _telemetry = telemetry;
        _vehicles = vehicles;
        ExternallyDrawn = true;
        Size = new SKSize(BarWidth + Pad * 2f, 116f);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;

        _bar = new PerformanceBar(telemetry)
        {
            Size = new SKSize(BarWidth, BarHeight),
            Offset = new SKPoint(Pad, 86f),
        };
        Add(_bar);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        double fps = _telemetry.AvgFps;
        double sim = _telemetry.AvgSimMs;
        double draw = _telemetry.AvgDrawMs;
        double frame = _telemetry.LastFrameMs;
        double idle = Math.Max(0, frame - sim - draw);

        float labelX = Bounds.Left + Pad;
        float textY = Bounds.Top + Pad + 10f;
        canvas.DrawText("PERFORMANCE", labelX, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);

        textY += 16f;
        canvas.DrawText($"FPS: {fps:F0}  |  Vehicles: {_vehicles.Count}  |  Frame: {frame:F1}ms",
            labelX, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);

        textY += 16f;
        canvas.DrawText($"Sim: {sim:F1}ms", labelX, textY, SKTextAlign.Left, UiTheme.Font11, _simTextPaint);
        canvas.DrawText($"Draw: {draw:F1}ms", labelX + 80, textY, SKTextAlign.Left, UiTheme.Font11, _drawTextPaint);
        canvas.DrawText($"Idle: {idle:F1}ms", labelX + 170, textY, SKTextAlign.Left, UiTheme.Font11, _idleTextPaint);

        textY += 16f;
        canvas.DrawText($"Path: {_telemetry.LastPathfindMs:F2}ms  calls: {_telemetry.LastPathfindCalls}",
            labelX, textY, SKTextAlign.Left, UiTheme.Font11, _pathfindTextPaint);

        textY += 14f;
        canvas.DrawText($"GC  gen0: {GC.CollectionCount(0)}  gen1: {GC.CollectionCount(1)}  gen2: {GC.CollectionCount(2)}",
            labelX, textY, SKTextAlign.Left, UiTheme.Font11, _idleTextPaint);
    }
}
