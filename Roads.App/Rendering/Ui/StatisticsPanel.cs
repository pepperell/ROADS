using SkiaSharp;
using Roads.App.Vehicles;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Live traffic statistics: vehicle count, average speed, congestion percentage
/// (share of vehicles below <see cref="CongestionThresholdMs"/> m/s, color-graded
/// blue/amber/red), employment, active edge count, and resident totals. Shown by
/// default; the N key toggles it. Stats are computed read-only from the vehicle store,
/// road graph, and population manager on every draw; no simulation-tick work is added.
/// Positioned by the bottom-left stack above the performance HUD.
/// </summary>
public class StatisticsPanel : Panel
{
    /// <summary>
    /// Speed threshold in m/s below which a vehicle is counted as congested.
    /// 2 m/s (~4.5 mph) represents near-standstill / heavy stop-and-go traffic.
    /// </summary>
    public const float CongestionThresholdMs = 2f;

    private const float Pad = 8f;

    private readonly VehicleStore _vehicles;
    private readonly SimulationLoop _simLoop;
    private readonly World.RoadGraph _graph;

    private readonly SKPaint _headerPaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _valuePaint = new() { Color = new SKColor(100, 200, 255), IsAntialias = true };
    private readonly SKPaint _warnPaint = new() { Color = new SKColor(220, 160, 50), IsAntialias = true };
    private readonly SKPaint _alertPaint = new() { Color = new SKColor(220, 80, 60), IsAntialias = true };

    public StatisticsPanel(VehicleStore vehicles, SimulationLoop simLoop, World.RoadGraph graph)
    {
        _vehicles = vehicles;
        _simLoop = simLoop;
        _graph = graph;
        Size = new SKSize(256f, 98f);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        int count = _vehicles.Count;
        float avgSpeed = 0f;
        int congested = 0;
        if (count > 0)
        {
            float sum = 0f;
            var speeds = _vehicles.Speed;
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
        float avgSpeedMph = avgSpeed * 2.23694f;

        float px = Bounds.Left, py = Bounds.Top;
        float textY = py + Pad + 10f;
        canvas.DrawText("STATISTICS", px + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);

        textY += 16f;
        canvas.DrawText("Vehicles: ", px + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        canvas.DrawText($"{count}", px + Pad + 60f, textY, SKTextAlign.Left, UiTheme.Font11, _valuePaint);
        canvas.DrawText("Avg Speed: ", px + Pad + 100f, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        canvas.DrawText($"{avgSpeedMph:F1} mph", px + Pad + 166f, textY, SKTextAlign.Left, UiTheme.Font11, _valuePaint);

        textY += 16f;
        canvas.DrawText("Congestion: ", px + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        var congPaint = congestionPct >= 60f ? _alertPaint
                      : congestionPct >= 25f ? _warnPaint
                      : _valuePaint;
        canvas.DrawText($"{congestionPct:F1}%", px + Pad + 72f, textY, SKTextAlign.Left, UiTheme.Font11, congPaint);
        canvas.DrawText($"({congested} slow)", px + Pad + 118f, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);

        textY += 16f;
        var population = _simLoop.Population;
        canvas.DrawText("Jobs: ", px + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        canvas.DrawText($"{population.EmployedWorkers} emp", px + Pad + 36f, textY, SKTextAlign.Left, UiTheme.Font11, _valuePaint);
        canvas.DrawText($"{population.UnemployedWorkers} jobless", px + Pad + 92f, textY, SKTextAlign.Left, UiTheme.Font11,
            population.UnemployedWorkers > 0 ? _warnPaint : _valuePaint);
        canvas.DrawText($"{population.JobOpenings} open", px + Pad + 170f, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);

        textY += 16f;
        canvas.DrawText("Edges: ", px + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        canvas.DrawText($"{_graph.ActiveEdgeCount}", px + Pad + 40f, textY, SKTextAlign.Left, UiTheme.Font11, _valuePaint);
        canvas.DrawText("Residents: ", px + Pad + 84f, textY, SKTextAlign.Left, UiTheme.Font11, _headerPaint);
        canvas.DrawText($"{population.TotalResidents} ({population.ActiveDrivers} active)",
            px + Pad + 148f, textY, SKTextAlign.Left, UiTheme.Font11, _valuePaint);
    }
}
