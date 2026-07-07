using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Composes all renderers to draw the complete scene: terrain background, faint editor
/// grid, roads, buildings, props, vehicles, traffic-control signs, editor
/// overlays, and screen-space UI. All state is read-only within the render pass.
/// The terrain/building/prop/sign renderers are owned privately here (like the heat
/// map) since nothing outside the scene needs them.
/// </summary>
public class SceneRenderer
{
    private readonly RoadRenderer _roadRenderer;
    private readonly VehicleRenderer _vehicleRenderer;
    private readonly LaneRestrictionTool _laneRestrictionTool;

    /// <summary>Retained-mode UI tree (owned by MainForm, which also routes input to it).
    /// Drawn as the screen-space pass; legacy immediate-mode panels draw around it until
    /// their migration lands.</summary>
    private readonly Ui.UiRoot _uiRoot;

    /// <summary>
    /// Congestion heat-map overlay. Recomputed once per frame in <see cref="Render"/>
    /// before the road draw pass and forwarded to the road renderer.
    /// </summary>
    private readonly CongestionHeatMap _heatMap = new();

    /// <summary>Procedural terrain background (grass mottling); replaces the old flat clear color.</summary>
    private readonly TerrainRenderer _terrain = new();

    /// <summary>Deterministic building placement derived from destination nodes; rebuilt on graph version change.</summary>
    private readonly BuildingLayer _buildingLayer = new();

    /// <summary>Draws the buildings computed by <see cref="_buildingLayer"/> (and far-zoom POI dots).</summary>
    private readonly BuildingRenderer _buildingRenderer = new();

    /// <summary>Street lights, trees, and bushes. Rebuilt after the building layer so props avoid footprints.</summary>
    private readonly PropRenderer _propRenderer = new();

    /// <summary>Realistic traffic signals, stop/yield signs, and change-only speed-limit signs.</summary>
    private readonly SignRenderer _signRenderer = new();

    /// <summary>Building world AABBs handed to the prop renderer; refreshed when scenery rebuilds.</summary>
    private readonly List<SKRect> _buildingBounds = new();
    private int _buildingBoundsVersion = -1;

    /// <summary>
    /// Frames the graph version must hold still before scenery (buildings, props, speed-sign
    /// placements) rebuilds. Node/control-point drags bump the version every mouse-move frame;
    /// without this gate the full placement pass (~tens of ms at city scale) would re-run per
    /// frame for the whole drag. Stale scenery draws safely in the interim — positions are
    /// baked and node indices are stable. Roads/markings still rebuild live for drag feedback.
    /// </summary>
    private const int SceneryRebuildDelayFrames = 8;
    private int _lastSeenGraphVersion = -1;
    private int _graphStableFrames;

    /// <summary>Reusable buffer of visible edge indices, refilled each frame from the edge grid.</summary>
    private readonly List<int> _visibleEdges = new();

    /// <summary>
    /// Enables or disables the congestion heat-map overlay. Forwards to
    /// <see cref="CongestionHeatMap.Enabled"/> so the director can wire a keyboard
    /// toggle without touching MainForm.cs.
    /// </summary>
    public bool HeatMapEnabled
    {
        get => _heatMap.Enabled;
        set => _heatMap.Enabled = value;
    }

    /// <summary>
    /// Notifies the scene that the graph was REPLACED wholesale (New / Load / stress scene)
    /// rather than edited. Cached scenery must NOT survive the settle-gate window here: the
    /// gate's stale-draw safety argument assumes node indices are stable, which holds for
    /// edits (nodes go defunct in place) but not for a replaced — possibly smaller — node
    /// list, where an old footprint's node index can be out of range (and the old city
    /// would draw over the new map). Resets the gate so the next frame rebuilds scenery
    /// immediately, like a first build. Call after the new graph is fully loaded.
    /// </summary>
    public void OnMapReplaced()
    {
        _lastSeenGraphVersion = -1;
        _graphStableFrames = 0;
        _buildingBoundsVersion = -1;
    }

    /// <summary>
    /// Constructs the SceneRenderer and wires all sub-renderers and UI panels.
    /// Call order dependency: <see cref="Render"/> must be called only after all
    /// sub-renderers are fully initialized.
    /// </summary>
    public SceneRenderer(RoadRenderer roadRenderer, VehicleRenderer vehicleRenderer,
        LaneRestrictionTool laneRestrictionTool, Ui.UiRoot uiRoot)
    {
        _roadRenderer = roadRenderer;
        _vehicleRenderer = vehicleRenderer;
        _laneRestrictionTool = laneRestrictionTool;
        _uiRoot = uiRoot;
    }

    /// <summary>
    /// Renders the entire scene: grid, roads, vehicles, editor overlays, and UI.
    /// </summary>
    public void Render(SKCanvas canvas, SKImageInfo info, Camera camera,
        RoadGraph graph, VehicleStore vehicles, EditorState editorState,
        StopLineCache stopLineCache, IntersectionArcCache intersectionArcs,
        TrafficSignalSystem trafficSignals, StopSignSystem stopSigns,
        YieldSignSystem yieldSigns, SimulationLoop simLoop,
        Point currentMousePos)
    {
        float darkness = simLoop.Clock.Darkness;
        canvas.Clear(TerrainRenderer.GetBaseColor(darkness));

        var transform = camera.GetTransformMatrix(info.Width, info.Height);
        canvas.SetMatrix(transform);

        // Compute the visible world-space rectangle once; forwarded to renderers for
        // frustum culling so they can skip off-screen geometry cheaply.
        var viewRect = camera.GetVisibleWorldRect(info.Width, info.Height);

        // Terrain detail (grass mottling) over the base clear color, then a faint
        // 100 m reference grid kept as an editor alignment aid.
        _terrain.Draw(canvas, viewRect, camera.Zoom, darkness);
        DrawGrid(canvas, camera, info, GetGridColor(darkness));

        // Update congestion heat-map before the road draw pass so values are current
        // (cheap no-op when the overlay is disabled).
        _heatMap.Update(vehicles, graph);

        // Cull roads to the visible viewport via the edge spatial grid, so the surface/marking
        // passes iterate only on-screen edges instead of the whole network (big win when zoomed in).
        simLoop.EdgeGrid.QueryVisible(graph.Edges.Count, viewRect.Left, viewRect.Top, viewRect.Right, viewRect.Bottom, _visibleEdges);

        // Draw roads (heat-map forwarded so the renderer can tint surfaces; traffic
        // signals forwarded so light-controlled approaches get crosswalks).
        _roadRenderer.Draw(canvas, graph, stopLineCache, camera.Zoom, darkness, _heatMap, viewRect, _visibleEdges, stopSigns, trafficSignals);

        // Buildings and props sit under vehicles. Placement rebuilds are deferred until the
        // graph version has held still for SceneryRebuildDelayFrames (drags bump it every
        // frame); the very first build runs immediately. Rebuild order matters: the building
        // layer first, then bounds collection, then props so they avoid footprints.
        if (graph.Version != _lastSeenGraphVersion)
        {
            _lastSeenGraphVersion = graph.Version;
            _graphStableFrames = 0;
        }
        else if (_graphStableFrames <= SceneryRebuildDelayFrames)
        {
            _graphStableFrames++;
        }
        bool scenerySettled = _graphStableFrames >= SceneryRebuildDelayFrames || _buildingBoundsVersion < 0;
        if (scenerySettled && graph.Version != _buildingBoundsVersion)
        {
            _buildingLayer.RebuildIfNeeded(graph, simLoop.EdgeGrid);
            _buildingLayer.CollectBounds(_buildingBounds);
            _buildingBoundsVersion = graph.Version;
            _propRenderer.Rebuild(graph, simLoop.EdgeGrid, _buildingBounds, graph.Version);
        }
        _buildingRenderer.Draw(canvas, _buildingLayer, graph, viewRect, camera.Zoom, darkness);
        _propRenderer.Draw(canvas, viewRect, camera.Zoom, darkness);

        // Draw hover and selection highlights
        DrawEdgeHoverHighlight(canvas, graph, editorState);
        DrawNodeHoverHighlight(canvas, graph, editorState, camera);
        DrawEdgeHighlight(canvas, graph, editorState);
        DrawNodeHighlight(canvas, graph, editorState, intersectionArcs, camera);

        // Draw lane restriction overlay
        if (editorState.LaneRestrictionMode && editorState.SelectedNode >= 0)
            _laneRestrictionTool.DrawOverlay(canvas, editorState.SelectedNode, graph, editorState, stopLineCache, camera.Zoom);

        // Draw drag crossing previews
        DrawCrossingPreviews(canvas, editorState, camera);

        // Draw vehicles
        _vehicleRenderer.Draw(canvas, vehicles, camera.Zoom, darkness, viewRect);
        _vehicleRenderer.DrawArcConflictOverlay(canvas, vehicles, intersectionArcs);
        if (editorState.HoveredVehicle >= 0 && editorState.HoveredVehicle != editorState.SelectedVehicle)
            _vehicleRenderer.DrawHoverOverlay(canvas, vehicles, editorState.HoveredVehicle);
        if (editorState.SelectedVehicle >= 0)
            _vehicleRenderer.DrawSelectionOverlay(canvas, vehicles, editorState.SelectedVehicle, graph, stopLineCache, intersectionArcs);

        // Traffic-control furniture draws above vehicles so signal state stays readable.
        // Speed-sign placement (which reads the StopLineCache) rebuilds only once the graph
        // has settled, so it never caches positions derived from a not-yet-rebuilt cache.
        _signRenderer.Draw(canvas, graph, stopLineCache, trafficSignals, stopSigns, yieldSigns,
            camera.Zoom, viewRect, darkness, allowRebuild: scenerySettled);

        // Destination placement ghost (translucent preview of dest node + connector + foot node)
        DrawDestinationPlacementGhost(canvas, editorState, camera);

        // Node tool placement ghost (translucent preview at the exact click-result position)
        DrawNodeGhost(canvas, editorState, camera);

        // Control-type badges (F/A over every traffic light) while the Ctrl Type tool is active
        DrawSignalControlIcons(canvas, graph, editorState, camera, viewRect);

        // Draw control point handles in Select mode
        DrawControlPointHandles(canvas, graph, editorState, camera);

        // Draw road tool preview (anchor ghost, dashed segment line, crossing ghosts)
        DrawRoadPreview(canvas, graph, editorState, camera, currentMousePos, info);

        // Reset transform, then draw the entire screen-space UI as one retained-mode pass
        // (status bar, menu bar, POI submenu, legend, sliders, bottom-left stack, minimap;
        // the performance HUD is laid out here but drawn by MainForm.OnPaintSurface).
        canvas.ResetMatrix();
        _uiRoot.Draw(canvas, info.Width, info.Height);
    }

    private static void DrawGrid(SKCanvas canvas, Camera camera, SKImageInfo info, SKColor gridColor)
    {
        using var gridPaint = new SKPaint
        {
            Color = gridColor,
            StrokeWidth = 1f / camera.Zoom,
            IsAntialias = true
        };

        float gridSize = 100f;
        var view = camera.GetVisibleWorldRect(info.Width, info.Height);
        float worldLeft = view.Left;
        float worldRight = view.Right;
        float worldTop = view.Top;
        float worldBottom = view.Bottom;

        float startX = MathF.Floor(worldLeft / gridSize) * gridSize;
        float startY = MathF.Floor(worldTop / gridSize) * gridSize;

        for (float x = startX; x <= worldRight; x += gridSize)
            canvas.DrawLine(x, worldTop, x, worldBottom, gridPaint);
        for (float y = startY; y <= worldBottom; y += gridSize)
            canvas.DrawLine(worldLeft, y, worldRight, y, gridPaint);
    }

    /// <summary>Returns the hover highlight color (fill alpha, stroke alpha) for the active tool.</summary>
    private static (SKColor fill, SKColor stroke) GetHoverColors(EditorState editorState)
    {
        return editorState.ActiveTool switch
        {
            EditorTool.Delete     => (new SKColor(220, 60, 60, 50), new SKColor(220, 60, 60, 120)),
            EditorTool.Destination => GetDestinationHoverColors(editorState),
            EditorTool.Signal     => (new SKColor(220, 200, 40, 50), new SKColor(220, 200, 40, 120)),
            EditorTool.SignalControl => (new SKColor(60, 190, 170, 50), new SKColor(60, 190, 170, 120)),
            EditorTool.SignalRotate => (new SKColor(235, 140, 50, 50), new SKColor(235, 140, 50, 120)),
            EditorTool.SignalExempt => (new SKColor(200, 90, 210, 50), new SKColor(200, 90, 210, 120)),
            EditorTool.UpdateSegment => (new SKColor(170, 110, 240, 50), new SKColor(170, 110, 240, 120)),
            _                     => (new SKColor(100, 200, 255, 50), new SKColor(100, 200, 255, 120)),
        };
    }

    private static (SKColor fill, SKColor stroke) GetDestinationHoverColors(EditorState editorState)
    {
        var c = Ui.UiTheme.PoiColor(editorState.SelectedPOIType);
        return (new SKColor(c.Red, c.Green, c.Blue, 50), new SKColor(c.Red, c.Green, c.Blue, 120));
    }

    private static void DrawNodeHoverHighlight(SKCanvas canvas, RoadGraph graph,
        EditorState editorState, Camera camera)
    {
        int idx = editorState.HoveredNode;
        if (idx < 0 || idx >= graph.Nodes.Count || float.IsNaN(graph.Nodes[idx].Position.X)
            || idx == editorState.SelectedNode)
            return;

        var (fillColor, strokeColor) = GetHoverColors(editorState);
        var pos = graph.Nodes[idx].Position;
        // Junction footprint (widest incident road), floored at the bare node-dot size.
        float radius = MathF.Max(RenderDetail.NodeDotRadius(camera.Zoom),
            GeometryUtil.NodeJunctionRadius(graph, idx));
        using var fillPaint = new SKPaint
        {
            Color = fillColor,
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var strokePaint = new SKPaint
        {
            Color = strokeColor,
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 2f / camera.Zoom),
            IsAntialias = true,
        };
        canvas.DrawCircle(pos.X, pos.Y, radius, fillPaint);
        canvas.DrawCircle(pos.X, pos.Y, radius, strokePaint);
    }

    private static void DrawEdgeHoverHighlight(SKCanvas canvas, RoadGraph graph, EditorState editorState)
    {
        int idx = editorState.HoveredEdge;
        if (idx < 0 || idx >= graph.Edges.Count || graph.Edges[idx].FromNode < 0
            || idx == editorState.SelectedEdge)
            return;

        // The Update Segment tool previews the width the click will PRODUCE (from the
        // sticky road options); every other tool highlights the segment's current width.
        float surfaceWidth = editorState.ActiveTool == EditorTool.UpdateSegment
            ? GeometryUtil.RoadSurfaceWidthFor(editorState.SelectedLaneCount,
                editorState.SelectedOneWay, editorState.SelectedSharedLane)
            : GeometryUtil.RoadSurfaceWidth(graph, idx);

        var (fillColor, _) = GetHoverColors(editorState);
        using var hoverPaint = new SKPaint
        {
            Color = fillColor,
            StrokeWidth = surfaceWidth + 2f,
            Style = SKPaintStyle.Stroke,
            StrokeCap = SKStrokeCap.Round,
            IsAntialias = true,
        };
        using var path = new SKPath();
        var p0 = graph.EvaluateBezier(idx, 0f);
        path.MoveTo(p0.X, p0.Y);
        for (int s = 1; s <= 20; s++)
        {
            var pt = graph.EvaluateBezier(idx, s / 20f);
            path.LineTo(pt.X, pt.Y);
        }
        canvas.DrawPath(path, hoverPaint);
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
            StrokeWidth = GeometryUtil.RoadSurfaceWidth(graph, editorState.SelectedEdge) + 2f,
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
        // Junction footprint (widest incident road), floored at the bare node-dot size.
        float nodeRadius = MathF.Max(RenderDetail.NodeDotRadius(camera.Zoom),
            GeometryUtil.NodeJunctionRadius(graph, editorState.SelectedNode));
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

    /// <summary>
    /// Draws the Destination placement ghost: a translucent preview of the new destination dot at
    /// the cursor, a dashed connector to the perpendicular foot on the nearest road, and a small
    /// foot dot on the road. Drawn in world coordinates (camera transform still applied), so all
    /// radii/strokes are divided by zoom to match the real markers. The guard short-circuits when
    /// the tool is not Destination or the ghost fields are null (cursor over an eligible node, over
    /// UI, or no nearby road), giving automatic cleanup. Uses its own paints (does not touch shared).
    /// </summary>
    private static void DrawDestinationPlacementGhost(SKCanvas canvas, EditorState editorState, Camera camera)
    {
        if (editorState.ActiveTool != EditorTool.Destination) return;
        if (editorState.GhostDestPos is not { } dest || editorState.GhostFootPos is not { } foot)
            return;

        // Ghost POI color: same palette as the submenu / hover tints, low alpha.
        var c = Ui.UiTheme.PoiColor(editorState.SelectedPOIType);
        byte ghostAlpha = 110; // ~43%, within the 50-180 range used by existing previews

        float zoom = camera.Zoom;
        float radius = Math.Max(4f, 6f / zoom);          // match the historical marker dot
        float innerRadius = radius * 0.5f;
        float footRadius = Math.Max(3f, 4f / zoom);      // smaller foot node

        // --- Connector road (dashed translucent line cursor -> foot), mimic DrawRoadPreview ---
        using var connectorPaint = new SKPaint
        {
            Color = new SKColor(c.Red, c.Green, c.Blue, ghostAlpha),
            StrokeWidth = Math.Max(2f, 3.5f / zoom),
            Style = SKPaintStyle.Stroke,
            StrokeCap = SKStrokeCap.Round,
            IsAntialias = true,
            PathEffect = SKPathEffect.CreateDash(new[] { 4f, 4f }, 0),
        };
        canvas.DrawLine(dest.X, dest.Y, foot.X, foot.Y, connectorPaint);

        // --- Foot node (small translucent dot on the road) ---
        using var footFill = new SKPaint
        {
            Color = new SKColor(c.Red, c.Green, c.Blue, ghostAlpha),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var footStroke = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 120),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 1.5f / zoom),
            IsAntialias = true,
        };
        canvas.DrawCircle(foot.X, foot.Y, footRadius, footFill);
        canvas.DrawCircle(foot.X, foot.Y, footRadius, footStroke);

        // --- Destination dot (translucent, mimics the historical marker composition) ---
        using var destFill = new SKPaint
        {
            Color = new SKColor(c.Red, c.Green, c.Blue, ghostAlpha),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var destStroke = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 120),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 1.5f / zoom),
            IsAntialias = true,
        };
        using var destInner = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 110),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        canvas.DrawCircle(dest.X, dest.Y, radius, destFill);
        canvas.DrawCircle(dest.X, dest.Y, radius, destStroke);
        canvas.DrawCircle(dest.X, dest.Y, innerRadius, destInner);

        // --- Building footprint preview (mirrors BuildingLayer's base placement: default
        // size for the POI type, front facing the foot node, 3 m base setback). The real
        // placement may nudge/shrink to avoid collisions; the ghost shows intent, not the
        // final result. EntryExit gets no footprint (it renders as a gateway, not a building).
        if (editorState.SelectedPOIType != POIType.EntryExit)
        {
            var (halfW, halfD) = BuildingLayer.GetDefaultFootprint(editorState.SelectedPOIType);
            var facing = new Vector2(foot.X - dest.X, foot.Y - dest.Y);
            float facingLen = facing.Length();
            if (halfW > 0f && halfD > 0f && facingLen > 0.001f)
            {
                facing /= facingLen;
                float cx = dest.X - facing.X * (3f + halfD);
                float cy = dest.Y - facing.Y * (3f + halfD);

                using var fpFill = new SKPaint
                {
                    Color = new SKColor(c.Red, c.Green, c.Blue, 55),
                    Style = SKPaintStyle.Fill,
                    IsAntialias = true,
                };
                using var fpStroke = new SKPaint
                {
                    Color = new SKColor(c.Red, c.Green, c.Blue, ghostAlpha),
                    Style = SKPaintStyle.Stroke,
                    StrokeWidth = Math.Max(0.4f, 1.5f / zoom),
                    IsAntialias = true,
                };

                canvas.Save();
                canvas.Translate(cx, cy);
                canvas.RotateDegrees(MathF.Atan2(facing.Y, facing.X) * (180f / MathF.PI));
                // Same local axes as BuildingRenderer: X = depth (front at +X), Y = facade width.
                var fpRect = new SKRect(-halfD, -halfW, halfD, halfW);
                canvas.DrawRect(fpRect, fpFill);
                canvas.DrawRect(fpRect, fpStroke);
                canvas.Restore();
            }
        }
    }

    /// <summary>
    /// Draws the Node tool's placement ghost at the exact point a click would create the
    /// node — snapped onto the nearest road (a split) or free at the cursor. A null ghost
    /// (cursor over an existing node, over UI, or mid-pan — recomputed every mouse-move)
    /// draws nothing.
    /// </summary>
    private static void DrawNodeGhost(SKCanvas canvas, EditorState editorState, Camera camera)
    {
        if (editorState.ActiveTool != EditorTool.Node) return;
        if (editorState.NodeGhostPos is not { } pos) return;
        // Splitting a road: the ghost spans that road's half-width; free node: bare dot.
        float radius = MathF.Max(RenderDetail.NodeDotRadius(camera.Zoom), editorState.NodeGhostRadius);
        DrawGhostNode(canvas, pos, camera, radius);
    }

    /// <summary>
    /// Shared ghost-node visual (translucent blue fill, dashed white ring) used by the
    /// Node tool's placement ghost and the Road tool's anchors — a node that will exist
    /// only after the operation commits. The caller supplies the radius so the ghost
    /// matches the footprint the commit will produce (the target road's half-width for
    /// the Road tool / an on-road split, the bare node-dot size for a free node); the
    /// dash interval scales with the circumference so the ring reads as a dashed circle
    /// at every size.
    /// </summary>
    private static void DrawGhostNode(SKCanvas canvas, Vector2 pos, Camera camera, float radius)
    {
        using var fill = new SKPaint
        {
            Color = new SKColor(100, 200, 255, 90),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        float dash = MathF.Max(0.8f, radius * MathF.Tau / 16f); // 8 dashes around the ring
        using var stroke = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 150),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 1.5f / camera.Zoom),
            IsAntialias = true,
            PathEffect = SKPathEffect.CreateDash(new[] { dash, dash }, 0),
        };
        canvas.DrawCircle(pos.X, pos.Y, radius, fill);
        canvas.DrawCircle(pos.X, pos.Y, radius, stroke);
    }

    /// <summary>
    /// Draws a control-type badge above every traffic-light node while the Signal-Control
    /// (Ctrl Type) tool is active: a blue "F" disc for fixed-time, an orange "A" disc for
    /// actuated (<see cref="NodeFlags.ActuatedSignal"/>). Badges are constant screen size
    /// (scaled by 1/zoom), lifted above the node so the intersection stays visible, and
    /// culled to the viewport. Reads node flags directly, so a click's flag flip shows on
    /// the very next frame.
    /// </summary>
    private static void DrawSignalControlIcons(SKCanvas canvas, RoadGraph graph,
        EditorState editorState, Camera camera, SKRect viewRect)
    {
        if (editorState.ActiveTool != EditorTool.SignalControl) return;

        float zoom = camera.Zoom;
        float radius = 9f / zoom;
        float liftY = 16f / zoom;
        float pad = radius + liftY;

        using var fill = new SKPaint { Style = SKPaintStyle.Fill, IsAntialias = true };
        using var ring = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 230),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 1.5f / zoom,
            IsAntialias = true,
        };
        using var textPaint = new SKPaint { Color = SKColors.White, IsAntialias = true };
        using var font = new SKFont(SKTypeface.Default, 11f / zoom) { Embolden = true };

        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            if (!node.Flags.HasFlag(NodeFlags.TrafficLight)) continue;
            if (node.Position.X < viewRect.Left - pad || node.Position.X > viewRect.Right + pad
                || node.Position.Y < viewRect.Top - pad || node.Position.Y > viewRect.Bottom + pad)
                continue;

            bool actuated = node.Flags.HasFlag(NodeFlags.ActuatedSignal);
            float bx = node.Position.X;
            float by = node.Position.Y - liftY;
            fill.Color = actuated ? new SKColor(215, 130, 30, 235) : new SKColor(45, 110, 190, 235);
            canvas.DrawCircle(bx, by, radius, fill);
            canvas.DrawCircle(bx, by, radius, ring);
            canvas.DrawText(actuated ? "A" : "F", bx, by + 4f / zoom, SKTextAlign.Center, font, textPaint);
        }
    }

    private static void DrawControlPointHandles(SKCanvas canvas, RoadGraph graph,
        EditorState editorState, Camera camera)
    {
        if (editorState.ActiveTool != EditorTool.Select) return;

        int selNode = editorState.SelectedNode;
        int selEdge = editorState.SelectedEdge;
        bool hasSelectedEdge = selEdge >= 0 && selEdge < graph.Edges.Count
            && graph.Edges[selEdge].FromNode >= 0;
        bool hasSelectedNode = selNode >= 0 && selNode < graph.Nodes.Count;

        if (!hasSelectedEdge && !hasSelectedNode) return;

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

        // Selected edge: show both control points
        if (hasSelectedEdge)
        {
            int primaryIdx = selEdge;
            int rev = graph.FindReverseEdge(primaryIdx);
            if (rev >= 0 && rev < primaryIdx) primaryIdx = rev;

            var edge = graph.Edges[primaryIdx];
            var fromPos = graph.Nodes[edge.FromNode].Position;
            var toPos = graph.Nodes[edge.ToNode].Position;

            canvas.DrawLine(fromPos.X, fromPos.Y, edge.ControlPoint1.X, edge.ControlPoint1.Y, linePaint);
            canvas.DrawLine(toPos.X, toPos.Y, edge.ControlPoint2.X, edge.ControlPoint2.Y, linePaint);
            canvas.DrawCircle(edge.ControlPoint1.X, edge.ControlPoint1.Y, handleRadius, handlePaint);
            canvas.DrawCircle(edge.ControlPoint2.X, edge.ControlPoint2.Y, handleRadius, handlePaint);
        }

        // Selected node: show only the adjacent CP on each connected edge
        if (hasSelectedNode)
        {
            var drawn = new HashSet<(int, int)>(); // (primaryEdge, cpIndex) to avoid duplicates
            for (int i = 0; i < graph.Edges.Count; i++)
            {
                var e = graph.Edges[i];
                if (e.FromNode < 0) continue;

                int cpSide; // which CP is adjacent to the selected node
                if (e.FromNode == selNode) cpSide = 1;
                else if (e.ToNode == selNode) cpSide = 2;
                else continue;

                // Deduplicate via primary edge
                int primary = i;
                int rev = graph.FindReverseEdge(i);
                if (rev >= 0 && rev < i) { primary = rev; cpSide = cpSide == 1 ? 2 : 1; }
                if (!drawn.Add((primary, cpSide))) continue;

                var edge = graph.Edges[primary];
                var nodePos = graph.Nodes[cpSide == 1 ? edge.FromNode : edge.ToNode].Position;
                var cpPos = cpSide == 1 ? edge.ControlPoint1 : edge.ControlPoint2;

                canvas.DrawLine(nodePos.X, nodePos.Y, cpPos.X, cpPos.Y, linePaint);
                canvas.DrawCircle(cpPos.X, cpPos.Y, handleRadius, handlePaint);
            }
        }
    }

    /// <summary>
    /// Draws the road tool's preview. At ALL times a ghost node marks the anchor a click
    /// would use (the snapped existing node, the clamped on-road split point, or the raw
    /// cursor in empty space) — the chain start before the first click, the segment end
    /// while drawing (this replaced the old snap-indicator ring). While drawing it adds a
    /// translucent round-capped band at the committed road's width (covering the node
    /// space at each end) with a thin dashed centerline, from the start anchor to the
    /// snapped end anchor; a ghost node when the start is still a PENDING anchor (nothing
    /// commits until the second click); and a ghost node at every crossing where the
    /// segment will split an existing road.
    /// </summary>
    private static void DrawRoadPreview(SKCanvas canvas, RoadGraph graph, EditorState editorState,
        Camera camera, Point currentMousePos, SKImageInfo info)
    {
        if (editorState.ActiveTool != EditorTool.Road) return;

        // Every ghost node of this tool marks an endpoint/intersection of the road being
        // drawn, so all of them scale with the selected road's half-width.
        float ghostRadius = MathF.Max(RenderDetail.NodeDotRadius(camera.Zoom),
            GeometryUtil.RoadSurfaceWidthFor(editorState.SelectedLaneCount,
                editorState.SelectedOneWay, editorState.SelectedSharedLane) * 0.5f);

        if (editorState.RoadAnchorGhostPos is { } anchorGhost)
            DrawGhostNode(canvas, anchorGhost, camera, ghostRadius);

        if (!editorState.IsDrawingRoad) return;

        Vector2 startPos;
        if (editorState.RoadStartNode is { } startNode)
        {
            startPos = graph.Nodes[startNode].Position;
            if (float.IsNaN(startPos.X)) return; // start node went defunct mid-draw
        }
        else
        {
            startPos = editorState.RoadStartAnchorPos!.Value;
            DrawGhostNode(canvas, startPos, camera, ghostRadius);
        }

        // The preview line ends at the SNAPPED anchor (the edge the commit will create);
        // raw cursor as fallback when no ghost has been computed yet this frame.
        float endX, endY;
        if (editorState.RoadAnchorGhostPos is { } end)
        {
            endX = end.X; endY = end.Y;
        }
        else
        {
            var mouseWorld = camera.ScreenToWorld(currentMousePos.X, currentMousePos.Y, info.Width, info.Height);
            endX = mouseWorld.X; endY = mouseWorld.Y;
        }

        // Two-layer ghost: a solid translucent band at the width the committed road will
        // have (from the sticky road options) with ROUND caps, so each end bulges out to
        // cover the space of the node the commit creates there — plus the classic thin
        // dashed centerline on top as the drawing guide. (Dashing the wide band itself
        // would not work: round caps extend each dash by half the road width, fusing the
        // gaps shut.)
        using var bandPaint = new SKPaint
        {
            Color = new SKColor(100, 180, 255, 60),
            StrokeWidth = GeometryUtil.RoadSurfaceWidthFor(editorState.SelectedLaneCount,
                editorState.SelectedOneWay, editorState.SelectedSharedLane),
            Style = SKPaintStyle.Stroke,
            StrokeCap = SKStrokeCap.Round,
            IsAntialias = true,
        };
        canvas.DrawLine(startPos.X, startPos.Y, endX, endY, bandPaint);

        using var previewPaint = new SKPaint
        {
            Color = new SKColor(100, 180, 255, 160),
            StrokeWidth = 1.5f,
            Style = SKPaintStyle.Stroke,
            IsAntialias = true,
            PathEffect = SKPathEffect.CreateDash(new[] { 4f, 4f }, 0),
        };
        canvas.DrawLine(startPos.X, startPos.Y, endX, endY, previewPaint);

        // Intersection nodes the commit will create where the segment crosses roads.
        foreach (var crossing in editorState.RoadCrossingPreviews)
            DrawGhostNode(canvas, crossing, camera, ghostRadius);
    }

    /// <summary>
    /// Faint editor-grid color: a lightened, translucent variant of the terrain base so the
    /// 100 m alignment grid stays visible over grass without competing with the scenery.
    /// </summary>
    private static SKColor GetGridColor(float darkness)
    {
        var b = TerrainRenderer.GetBaseColor(darkness);
        return new SKColor(
            (byte)Math.Min(b.Red + 20, 255),
            (byte)Math.Min(b.Green + 20, 255),
            (byte)Math.Min(b.Blue + 20, 255),
            70);
    }
}
