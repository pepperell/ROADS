using Roads.App;
using Roads.App.Vehicles;
using Roads.App.World;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Renders a read-only info panel in the bottom-left corner showing detailed data
/// about the currently selected vehicle: position, speed, steering, edge, lane, and path.
/// </summary>
public class VehicleInfoPanel
{
    private const float PanelWidth = 220f;
    private const float Padding = 10f;
    private const float LineHeight = 16f;

    /// <summary>
    /// Draws the vehicle info panel for the given vehicle index.
    /// </summary>
    /// <param name="canvas">SkiaSharp canvas to draw on.</param>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="vehicleIndex">Index of the selected vehicle.</param>
    /// <param name="graph">Road graph for edge/node data.</param>
    /// <param name="canvasHeight">Canvas height for bottom-left positioning.</param>
    public void Draw(SKCanvas canvas, VehicleStore store, int vehicleIndex, RoadGraph graph, float canvasHeight,
        IntersectionArcCache? arcCache = null)
    {
        if (vehicleIndex < 0 || vehicleIndex >= store.Count) return;

        var lines = new List<string>(16);

        // Position & Motion
        float speed = store.Speed[vehicleIndex];
        float mph = speed * 2.23694f;
        float headingDeg = store.Heading[vehicleIndex] * (180f / MathF.PI);
        float steerDeg = store.SteeringAngle[vehicleIndex] * (180f / MathF.PI);

        lines.Add($"Vehicle #{vehicleIndex}");
        lines.Add($"Pos: ({store.PosX[vehicleIndex]:F1}, {store.PosY[vehicleIndex]:F1})");
        lines.Add($"Speed: {speed:F1} m/s ({mph:F0} mph)");
        lines.Add($"Heading: {headingDeg:F1}\u00b0");
        lines.Add($"Steer: {steerDeg:F1}\u00b0");
        lines.Add($"Throttle: {store.Throttle[vehicleIndex]:F2}  Brake: {store.Brake[vehicleIndex]:F2}");

        // Road info
        int edgeIdx = store.CurrentEdge[vehicleIndex];
        if (edgeIdx >= 0 && edgeIdx < graph.Edges.Count)
        {
            var edge = graph.Edges[edgeIdx];
            float progress = store.EdgeProgress[vehicleIndex];
            float limitMph = edge.SpeedLimit * 2.23694f;
            lines.Add($"Edge: #{edgeIdx} ({edge.FromNode}\u2192{edge.ToNode})");
            lines.Add($"Progress: {progress * 100f:F1}%");
            lines.Add($"Limit: {limitMph:F0} mph");
            lines.Add($"Lane: {store.CurrentLane[vehicleIndex] + 1} of {edge.LaneCount}");

            if (store.TargetLane[vehicleIndex] != store.CurrentLane[vehicleIndex])
                lines.Add($"Lane Change: \u2192{store.TargetLane[vehicleIndex] + 1} ({store.LaneChangeProgress[vehicleIndex] * 100f:F0}%)");
        }

        // Arc info (when on arc)
        int currentArc = store.CurrentArc[vehicleIndex];
        if (currentArc >= 0 && arcCache != null)
        {
            var arc = arcCache.GetArc(currentArc);
            lines.Add($"Arc: #{currentArc} at node {arc.NodeIndex}");
            lines.Add($"  In: edge {arc.IncomingEdge} L{arc.IncomingLane}");
            lines.Add($"  Out: edge {arc.OutgoingEdge} L{arc.OutgoingLane}");
            lines.Add($"  Progress: {store.ArcProgress[vehicleIndex] * 100f:F1}%");
            if (store.Brake[vehicleIndex] >= 0.99f && store.Speed[vehicleIndex] < 0.5f)
                lines.Add("  *** STUCK ***");
        }

        // Path info
        var path = store.Path[vehicleIndex];
        int pathIdx = store.PathIndex[vehicleIndex];
        if (path != null)
        {
            int remaining = path.Count - pathIdx - 1;
            lines.Add($"Path: edge {pathIdx + 1} of {path.Count}  ({remaining} left)");
        }

        // Driver personality
        var archetypeName = ((DriverArchetype)store.Archetype[vehicleIndex]).ToString();
        lines.Add($"Driver: {archetypeName}");
        lines.Add($"  Spd: {store.SpeedBias[vehicleIndex]:F2}x  Aggr: {store.Aggressiveness[vehicleIndex]:F2}  Brake: {store.BrakingComfort[vehicleIndex]:F1}");
        lines.Add($"  React: {store.ReactionTime[vehicleIndex]:F2}s  Steer: {store.SteeringSharpness[vehicleIndex]:F1}x  LnChg: {store.LaneChangeBias[vehicleIndex]:F2}");

        // Draw panel
        float panelHeight = lines.Count * LineHeight + Padding * 2;
        float px = Padding;
        float py = canvasHeight - panelHeight - Padding;

        using var bgPaint = new SKPaint
        {
            Color = new SKColor(30, 32, 38, 220),
            Style = SKPaintStyle.Fill,
        };
        canvas.DrawRoundRect(px, py, PanelWidth, panelHeight, 4f, 4f, bgPaint);

        using var font = new SKFont { Size = 12 };
        using var labelPaint = new SKPaint
        {
            Color = new SKColor(200, 200, 200),
            IsAntialias = true,
        };
        using var headerPaint = new SKPaint
        {
            Color = new SKColor(100, 200, 255),
            IsAntialias = true,
        };

        for (int i = 0; i < lines.Count; i++)
        {
            var paint = i == 0 ? headerPaint : labelPaint;
            canvas.DrawText(lines[i], px + Padding, py + Padding + (i + 1) * LineHeight - 2f, SKTextAlign.Left, font, paint);
        }
    }
}
