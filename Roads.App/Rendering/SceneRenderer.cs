using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Composes all renderers to draw the complete scene: background grid, roads,
/// vehicles, markers, editor overlays, and screen-space UI. All state is read-only
/// within the render pass.
/// </summary>
public class SceneRenderer
{
    private readonly RoadRenderer _roadRenderer;
    private readonly VehicleRenderer _vehicleRenderer;
    private readonly MarkerRenderer _spawnPointRenderer;
    private readonly MarkerRenderer _destinationRenderer;
    private readonly UIRenderer _uiRenderer;
    private readonly SliderPanel _sliderPanel;
    private readonly VehicleInfoPanel _vehicleInfoPanel;
    private readonly LaneRestrictionTool _laneRestrictionTool;

    public SceneRenderer(RoadRenderer roadRenderer, VehicleRenderer vehicleRenderer,
        MarkerRenderer spawnPointRenderer, MarkerRenderer destinationRenderer,
        UIRenderer uiRenderer, SliderPanel sliderPanel, VehicleInfoPanel vehicleInfoPanel,
        LaneRestrictionTool laneRestrictionTool)
    {
        _roadRenderer = roadRenderer;
        _vehicleRenderer = vehicleRenderer;
        _spawnPointRenderer = spawnPointRenderer;
        _destinationRenderer = destinationRenderer;
        _uiRenderer = uiRenderer;
        _sliderPanel = sliderPanel;
        _vehicleInfoPanel = vehicleInfoPanel;
        _laneRestrictionTool = laneRestrictionTool;
    }

    /// <summary>
    /// Renders the entire scene: grid, roads, vehicles, markers, editor overlays, and UI.
    /// </summary>
    public void Render(SKCanvas canvas, SKImageInfo info, Camera camera,
        RoadGraph graph, VehicleStore vehicles, EditorState editorState,
        List<SpawnPoint> spawnPoints, List<DestinationPoint> destinations,
        StopLineCache stopLineCache, IntersectionArcCache intersectionArcs,
        TrafficSignalSystem trafficSignals, StopSignSystem stopSigns,
        YieldSignSystem yieldSigns, SimulationLoop simLoop,
        Point currentMousePos)
    {
        canvas.Clear(new SKColor(40, 42, 48));

        var transform = camera.GetTransformMatrix(info.Width, info.Height);
        canvas.SetMatrix(transform);

        // Draw grid
        DrawGrid(canvas, camera, info);

        // Draw roads
        _roadRenderer.Draw(canvas, graph, stopLineCache, camera.Zoom);
        _roadRenderer.DrawSignals(canvas, graph, trafficSignals, stopLineCache, camera.Zoom);
        _roadRenderer.DrawStopSigns(canvas, graph, stopSigns, stopLineCache, camera.Zoom);
        _roadRenderer.DrawYieldSigns(canvas, graph, yieldSigns, stopLineCache, camera.Zoom);
        _roadRenderer.DrawSpeedLimitSigns(canvas, graph, camera.Zoom);

        // Draw selection highlights
        DrawEdgeHighlight(canvas, graph, editorState);
        DrawNodeHighlight(canvas, graph, editorState, intersectionArcs, camera);

        // Draw lane restriction overlay
        if (editorState.LaneRestrictionMode && editorState.SelectedNode >= 0)
            _laneRestrictionTool.DrawOverlay(canvas, editorState.SelectedNode, graph, editorState, stopLineCache, camera.Zoom);

        // Draw drag crossing previews
        DrawCrossingPreviews(canvas, editorState, camera);

        // Draw vehicles
        _vehicleRenderer.Draw(canvas, vehicles, camera.Zoom);
        _vehicleRenderer.DrawArcConflictOverlay(canvas, vehicles, intersectionArcs);
        if (editorState.SelectedVehicle >= 0)
            _vehicleRenderer.DrawSelectionOverlay(canvas, vehicles, editorState.SelectedVehicle, graph, stopLineCache, intersectionArcs);

        // Draw spawn points and destinations
        _spawnPointRenderer.Draw(canvas, spawnPoints, sp => sp.Position, camera.Zoom);
        _destinationRenderer.Draw(canvas, destinations, dp => dp.Position, camera.Zoom);

        // Draw control point handles in Select mode
        DrawControlPointHandles(canvas, graph, editorState, camera);

        // Draw road tool preview and snap indicator
        DrawRoadPreview(canvas, graph, editorState, camera, currentMousePos, info);
        DrawSnapIndicator(canvas, graph, editorState, camera, currentMousePos, info);

        // Reset transform for UI overlay
        canvas.ResetMatrix();

        string statusText = BuildStatusText(graph, vehicles, editorState, spawnPoints, simLoop, camera);
        _uiRenderer.Draw(canvas, editorState, statusText, info.Width, info.Height);
        _sliderPanel.Draw(canvas, info.Width);
        if (editorState.SelectedVehicle >= 0)
            _vehicleInfoPanel.Draw(canvas, vehicles, editorState.SelectedVehicle, graph, info.Height, intersectionArcs);
    }

    private static void DrawGrid(SKCanvas canvas, Camera camera, SKImageInfo info)
    {
        using var gridPaint = new SKPaint
        {
            Color = new SKColor(60, 62, 68),
            StrokeWidth = 1f / camera.Zoom,
            IsAntialias = true
        };

        float gridSize = 100f;
        float worldLeft = -camera.CenterX / camera.Zoom - info.Width / (2f * camera.Zoom);
        float worldRight = -camera.CenterX / camera.Zoom + info.Width / (2f * camera.Zoom);
        float worldTop = -camera.CenterY / camera.Zoom - info.Height / (2f * camera.Zoom);
        float worldBottom = -camera.CenterY / camera.Zoom + info.Height / (2f * camera.Zoom);

        float startX = MathF.Floor(worldLeft / gridSize) * gridSize;
        float startY = MathF.Floor(worldTop / gridSize) * gridSize;

        for (float x = startX; x <= worldRight; x += gridSize)
            canvas.DrawLine(x, worldTop, x, worldBottom, gridPaint);
        for (float y = startY; y <= worldBottom; y += gridSize)
            canvas.DrawLine(worldLeft, y, worldRight, y, gridPaint);
    }

    private static void DrawEdgeHighlight(SKCanvas canvas, RoadGraph graph, EditorState editorState)
    {
        if (editorState.ActiveTool != EditorTool.Select || editorState.SelectedEdge < 0
            || editorState.SelectedEdge >= graph.Edges.Count
            || graph.Edges[editorState.SelectedEdge].FromNode < 0)
            return;

        using var highlightPaint = new SKPaint
        {
            Color = new SKColor(100, 200, 255, 80),
            StrokeWidth = graph.Edges[editorState.SelectedEdge].LaneCount * 2 * 3.5f + 2f,
            Style = SKPaintStyle.Stroke,
            StrokeCap = SKStrokeCap.Round,
            IsAntialias = true,
        };
        using var path = new SKPath();
        var p0 = graph.EvaluateBezier(editorState.SelectedEdge, 0f);
        path.MoveTo(p0.X, p0.Y);
        for (int s = 1; s <= 20; s++)
        {
            var pt = graph.EvaluateBezier(editorState.SelectedEdge, s / 20f);
            path.LineTo(pt.X, pt.Y);
        }
        canvas.DrawPath(path, highlightPaint);
    }

    private static void DrawNodeHighlight(SKCanvas canvas, RoadGraph graph, EditorState editorState,
        IntersectionArcCache intersectionArcs, Camera camera)
    {
        if (editorState.ActiveTool != EditorTool.Select || editorState.SelectedNode < 0
            || editorState.SelectedNode >= graph.Nodes.Count
            || float.IsNaN(graph.Nodes[editorState.SelectedNode].Position.X))
            return;

        var nodePos = graph.Nodes[editorState.SelectedNode].Position;
        float nodeRadius = Math.Max(6f, 10f / camera.Zoom);
        using var nodeHighlightFill = new SKPaint
        {
            Color = new SKColor(100, 200, 255, 100),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var nodeHighlightStroke = new SKPaint
        {
            Color = new SKColor(100, 200, 255, 220),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 2f / camera.Zoom),
            IsAntialias = true,
        };
        canvas.DrawCircle(nodePos.X, nodePos.Y, nodeRadius, nodeHighlightFill);
        canvas.DrawCircle(nodePos.X, nodePos.Y, nodeRadius, nodeHighlightStroke);

        // Draw intersection arcs at the selected node
        var arcSpan = intersectionArcs.GetArcsAtNode(editorState.SelectedNode);
        if (arcSpan.Length > 0)
        {
            using var arcPaint = new SKPaint
            {
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1f, 2f / camera.Zoom),
                IsAntialias = true,
            };
            const int arcSegments = 12;

            for (int ai = 0; ai < arcSpan.Length; ai++)
            {
                var arc = intersectionArcs.GetArc(arcSpan[ai]);

                // Color by turn direction: green=straight, yellow=moderate, orange=sharp
                float dot = 0f;
                {
                    var inT = new Vector2(arc.P1.X - arc.P0.X, arc.P1.Y - arc.P0.Y);
                    var outT = new Vector2(arc.P3.X - arc.P2.X, arc.P3.Y - arc.P2.Y);
                    float inLen = inT.Length();
                    float outLen = outT.Length();
                    if (inLen > 0.001f && outLen > 0.001f)
                        dot = (inT.X * outT.X + inT.Y * outT.Y) / (inLen * outLen);
                }

                if (dot >= 0.7f)
                    arcPaint.Color = new SKColor(100, 220, 100, 160); // straight: green
                else if (dot >= 0.0f)
                    arcPaint.Color = new SKColor(220, 200, 60, 160);  // moderate: yellow
                else
                    arcPaint.Color = new SKColor(220, 130, 50, 160);  // sharp: orange

                using var arcPath = new SKPath();
                var first = intersectionArcs.EvaluateArc(arcSpan[ai], 0f);
                arcPath.MoveTo(first.X, first.Y);
                for (int s = 1; s <= arcSegments; s++)
                {
                    float t = (float)s / arcSegments;
                    var pt = intersectionArcs.EvaluateArc(arcSpan[ai], t);
                    arcPath.LineTo(pt.X, pt.Y);
                }
                canvas.DrawPath(arcPath, arcPaint);
            }
        }
    }

    private static void DrawCrossingPreviews(SKCanvas canvas, EditorState editorState, Camera camera)
    {
        if (editorState.DragCrossingPreviews.Count == 0) return;

        float previewRadius = Math.Max(5f, 8f / camera.Zoom);
        using var previewFill = new SKPaint
        {
            Color = new SKColor(255, 200, 0, 180),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var previewStroke = new SKPaint
        {
            Color = new SKColor(255, 160, 0, 255),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 2f / camera.Zoom),
            IsAntialias = true,
        };
        foreach (var pos in editorState.DragCrossingPreviews)
        {
            canvas.DrawCircle(pos.X, pos.Y, previewRadius, previewFill);
            canvas.DrawCircle(pos.X, pos.Y, previewRadius, previewStroke);
        }
    }

    private static void DrawControlPointHandles(SKCanvas canvas, RoadGraph graph,
        EditorState editorState, Camera camera)
    {
        if (editorState.ActiveTool != EditorTool.Select) return;

        float handleRadius = Math.Max(3f, 5f / camera.Zoom);
        using var handlePaint = new SKPaint
        {
            Color = new SKColor(255, 140, 40, 200),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var linePaint = new SKPaint
        {
            Color = new SKColor(255, 140, 40, 100),
            StrokeWidth = Math.Max(0.5f, 1f / camera.Zoom),
            Style = SKPaintStyle.Stroke,
            IsAntialias = true,
        };

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            // Skip reverse edge of a pair — only show primary handles
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            var fromPos = graph.Nodes[edge.FromNode].Position;
            var toPos = graph.Nodes[edge.ToNode].Position;

            // Draw lines from node to control point
            canvas.DrawLine(fromPos.X, fromPos.Y, edge.ControlPoint1.X, edge.ControlPoint1.Y, linePaint);
            canvas.DrawLine(toPos.X, toPos.Y, edge.ControlPoint2.X, edge.ControlPoint2.Y, linePaint);

            // Draw control point handles
            canvas.DrawCircle(edge.ControlPoint1.X, edge.ControlPoint1.Y, handleRadius, handlePaint);
            canvas.DrawCircle(edge.ControlPoint2.X, edge.ControlPoint2.Y, handleRadius, handlePaint);
        }
    }

    private static void DrawRoadPreview(SKCanvas canvas, RoadGraph graph, EditorState editorState,
        Camera camera, Point currentMousePos, SKImageInfo info)
    {
        if (editorState.ActiveTool != EditorTool.Road || !editorState.IsDrawingRoad) return;

        var startNode = graph.Nodes[editorState.RoadStartNode!.Value];
        var mouseWorld = camera.ScreenToWorld(currentMousePos.X, currentMousePos.Y, info.Width, info.Height);

        using var previewPaint = new SKPaint
        {
            Color = new SKColor(100, 180, 255, 128),
            StrokeWidth = 3.5f,
            Style = SKPaintStyle.Stroke,
            IsAntialias = true,
            PathEffect = SKPathEffect.CreateDash(new[] { 4f, 4f }, 0),
        };
        canvas.DrawLine(startNode.Position.X, startNode.Position.Y, mouseWorld.X, mouseWorld.Y, previewPaint);
    }

    private static void DrawSnapIndicator(SKCanvas canvas, RoadGraph graph, EditorState editorState,
        Camera camera, Point currentMousePos, SKImageInfo info)
    {
        if (editorState.ActiveTool != EditorTool.Road) return;

        var mouseWorld = camera.ScreenToWorld(currentMousePos.X, currentMousePos.Y, info.Width, info.Height);
        int nearNode = graph.FindNearestNode(new Vector2(mouseWorld.X, mouseWorld.Y), EditorState.SnapDistance);
        if (nearNode < 0) return;

        var nodePos = graph.Nodes[nearNode].Position;
        using var snapPaint = new SKPaint
        {
            Color = new SKColor(100, 180, 255, 180),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 1.5f / camera.Zoom),
            IsAntialias = true,
        };
        canvas.DrawCircle(nodePos.X, nodePos.Y, Math.Max(4f, 6f / camera.Zoom), snapPaint);
    }

    private static string BuildStatusText(RoadGraph graph, VehicleStore vehicles,
        EditorState editorState, List<SpawnPoint> spawnPoints,
        SimulationLoop simLoop, Camera camera)
    {
        string status = editorState.IsDrawingRoad ? " [drawing]" : "";
        string selInfo = "";
        if (editorState.SelectedNode >= 0 && editorState.SelectedNode < graph.Nodes.Count
            && !float.IsNaN(graph.Nodes[editorState.SelectedNode].Position.X))
        {
            var selNode = graph.Nodes[editorState.SelectedNode];
            string flags = selNode.Flags == NodeFlags.None ? "none" : selNode.Flags.ToString();
            if (editorState.LaneRestrictionMode)
            {
                string laneInfo = editorState.LaneRestrictionEdge >= 0
                    ? $"Input Edge {editorState.LaneRestrictionEdge} Lane {editorState.LaneRestrictionLane}"
                    : "click input lane";
                selInfo = $"  |  LANE RESTRICT: {laneInfo}  [click lanes, C=clear, Esc=exit]";
            }
            else
                selInfo = $"  |  Node #{editorState.SelectedNode}  Flags: {flags}  [L=lane restrict, Del=delete, drag to move]";
        }
        else if (editorState.SelectedEdge >= 0 && editorState.SelectedEdge < graph.Edges.Count
            && graph.Edges[editorState.SelectedEdge].FromNode >= 0)
        {
            var sel = graph.Edges[editorState.SelectedEdge];
            float mph = sel.SpeedLimit * 2.23694f;
            selInfo = $"  |  Selected: {sel.LaneCount} lane(s) [+/- lanes]  Speed: {mph:F0} mph [ [ / ] to change]";
        }
        string spawnInfo = spawnPoints.Count > 0 ? $"Spawn Pts: {spawnPoints.Count}" : "V=spawn, or place Spawn Pts";
        string timeInfo = simLoop.Paused ? "PAUSED" : $"{simLoop.TimeScale}x";
        string debugInfo = VehicleRenderer.ShowArcConflicts ? "  |  [G] ARC DEBUG" : "";
        return $"Zoom: {camera.Zoom:F2}x  |  Speed: {timeInfo}  |  Edges: {graph.ActiveEdgeCount}  |  Vehicles: {vehicles.Count}  |  {spawnInfo}{status}{selInfo}{debugInfo}";
    }
}
