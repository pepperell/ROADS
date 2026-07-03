using SkiaSharp;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Read-only detail panel for the currently selected vehicle: position, speed, steering,
/// edge/lane/arc state, path progress, and driver personality. Visibility follows the
/// selection live (<see cref="Panel.VisibleWhen"/>); height follows the line count
/// (<see cref="Measure"/> rebuilds a REUSED line list — no per-frame allocation — and the
/// bottom-left stack positions siblings from the measured size).
/// </summary>
public class VehicleInfoPanel : Panel
{
    // Same width as the other bottom-left stack panels (statistics/performance/selection).
    private const float PanelWidth = 256f;
    private const float Pad = 10f;
    private const float LineHeight = 16f;

    private readonly VehicleStore _store;
    private readonly RoadGraph _graph;
    private readonly EditorState _editorState;
    private readonly IntersectionArcCache _arcCache;
    private readonly List<string> _lines = new(20);

    private readonly SKPaint _labelPaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _headerPaint = new() { Color = new SKColor(100, 200, 255), IsAntialias = true };

    public VehicleInfoPanel(VehicleStore store, RoadGraph graph, EditorState editorState,
        IntersectionArcCache arcCache)
    {
        _store = store;
        _graph = graph;
        _editorState = editorState;
        _arcCache = arcCache;
        VisibleWhen = () => editorState.SelectedVehicle >= 0 && editorState.SelectedVehicle < store.Count;
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
        Size = new SKSize(PanelWidth, Pad * 2f);
    }

    public override void Measure(float canvasWidth, float canvasHeight)
    {
        if (!IsEffectivelyVisible) return;
        BuildLines(_editorState.SelectedVehicle);
        Size = new SKSize(PanelWidth, _lines.Count * LineHeight + Pad * 2f);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        for (int i = 0; i < _lines.Count; i++)
        {
            var paint = i == 0 ? _headerPaint : _labelPaint;
            canvas.DrawText(_lines[i], Bounds.Left + Pad,
                Bounds.Top + Pad + (i + 1) * LineHeight - 2f, SKTextAlign.Left, UiTheme.Font12, paint);
        }
    }

    private void BuildLines(int vehicleIndex)
    {
        var store = _store;
        var graph = _graph;
        var lines = _lines;
        lines.Clear();

        float speed = store.Speed[vehicleIndex];
        float mph = speed * 2.23694f;
        float headingDeg = store.Heading[vehicleIndex] * (180f / MathF.PI);
        float steerDeg = store.SteeringAngle[vehicleIndex] * (180f / MathF.PI);

        lines.Add($"Vehicle #{vehicleIndex}");
        lines.Add($"Pos: ({store.PosX[vehicleIndex]:F1}, {store.PosY[vehicleIndex]:F1})");
        lines.Add($"Speed: {speed:F1} m/s ({mph:F0} mph)");
        lines.Add($"Heading: {headingDeg:F1}°");
        lines.Add($"Steer: {steerDeg:F1}°");
        lines.Add($"Throttle: {store.Throttle[vehicleIndex]:F2}  Brake: {store.Brake[vehicleIndex]:F2}");

        int edgeIdx = store.CurrentEdge[vehicleIndex];
        if (edgeIdx >= 0 && edgeIdx < graph.Edges.Count)
        {
            var edge = graph.Edges[edgeIdx];
            float progress = store.EdgeProgress[vehicleIndex];
            float limitMph = edge.SpeedLimit * 2.23694f;
            lines.Add($"Edge: #{edgeIdx} ({edge.FromNode}→{edge.ToNode})");
            lines.Add($"Progress: {progress * 100f:F1}%");
            lines.Add($"Limit: {limitMph:F0} mph");
            lines.Add($"Lane: {store.CurrentLane[vehicleIndex] + 1} of {edge.LaneCount}");

            if (store.TargetLane[vehicleIndex] != store.CurrentLane[vehicleIndex])
                lines.Add($"Lane Change: →{store.TargetLane[vehicleIndex] + 1} ({store.LaneChangeProgress[vehicleIndex] * 100f:F0}%)");
        }

        int currentArc = store.CurrentArc[vehicleIndex];
        if (currentArc >= 0)
        {
            var arc = _arcCache.GetArc(currentArc);
            lines.Add($"Arc: #{currentArc} at node {arc.NodeIndex}");
            lines.Add($"  In: edge {arc.IncomingEdge} L{arc.IncomingLane}");
            lines.Add($"  Out: edge {arc.OutgoingEdge} L{arc.OutgoingLane}");
            lines.Add($"  Progress: {store.ArcProgress[vehicleIndex] * 100f:F1}%");
            if (store.Brake[vehicleIndex] >= 0.99f && store.Speed[vehicleIndex] < 0.5f)
                lines.Add("  *** STUCK ***");
        }

        var path = store.Path[vehicleIndex];
        int pathIdx = store.PathIndex[vehicleIndex];
        if (path != null)
        {
            int remaining = path.Count - pathIdx - 1;
            lines.Add($"Path: edge {pathIdx + 1} of {path.Count}  ({remaining} left)");
        }

        var archetypeName = ((DriverArchetype)store.Archetype[vehicleIndex]).ToString();
        lines.Add($"Driver: {archetypeName}");
        lines.Add($"  Spd: {store.SpeedBias[vehicleIndex]:F2}x  Aggr: {store.Aggressiveness[vehicleIndex]:F2}  Brake: {store.BrakingComfort[vehicleIndex]:F1}");
        lines.Add($"  React: {store.ReactionTime[vehicleIndex]:F2}s  Steer: {store.SteeringSharpness[vehicleIndex]:F1}x  LnChg: {store.LaneChangeBias[vehicleIndex]:F2}");
    }
}
