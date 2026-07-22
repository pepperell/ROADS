using System.Numerics;
using SkiaSharp;
using Roads.App;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Renders the road network with a distinct visual identity per <see cref="RoadType"/>:
/// shoulder/sidewalk/verge bands under the asphalt, per-type surface tones and edge
/// treatment (dark curbs on residential/arterial, white edge lines on highway), per-type
/// center-line policy (none on residential, double solid yellow on arterial, median band
/// plus yellow lines on highway, worn tire tracks on dirt), lane markings, stop lines,
/// continental crosswalks at signalized approaches, intersection fills, and node dots.
/// Traffic-control furniture (signal heads, stop/yield/speed signs) is drawn by
/// <see cref="SignRenderer"/>. Caches per-edge Bezier offset paths and invalidates when
/// the graph version changes.
/// </summary>
public class RoadRenderer
{
    /// <summary>Lane width in meters.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;
    /// <summary>Number of line segments per Bézier curve for rendering.</summary>
    private const int BezierSegments = 20;
    /// <summary>Zoom below which shoulders/sidewalks are skipped (sub-pixel at that scale).</summary>
    private const float ShoulderMinZoom = 0.3f;
    /// <summary>Lateral offset (m) of each line of the arterial double-yellow from the center path.</summary>
    private const float ArterialYellowOffset = 0.22f;
    /// <summary>Lateral offset (m) of each highway yellow line from the median center.</summary>
    private const float HighwayYellowOffset = 0.9f;
    /// <summary>Stroke width (m) of the highway median band.</summary>
    private const float HighwayMedianWidth = 1.6f;
    /// <summary>Lateral offset (m) of each dirt tire track from its lane center.</summary>
    private const float TireTrackSpread = 0.85f;
    /// <summary>Stroke width (m) of a dirt tire track.</summary>
    private const float TireTrackWidth = 0.45f;
    /// <summary>Zoom at or above which crosswalks are drawn (needs TrafficSignalSystem supplied to Draw).</summary>
    private const float CrosswalkMinZoom = 0.8f;
    /// <summary>Lateral width (m) of each continental crosswalk bar (MUTCD ~24 in).</summary>
    private const float CrosswalkBarWidth = 0.6f;
    /// <summary>Lateral gap (m) between continental bars (equal-width gaps; ~1.2 m pitch
    /// puts about three bars per 3.5 m lane).</summary>
    private const float CrosswalkBarGap = 0.6f;
    /// <summary>Length (m) of each bar along the travel direction — the crossing depth
    /// pedestrians walk within (MUTCD typical ~8 ft).</summary>
    private const float CrosswalkDepth = 2.4f;
    /// <summary>Distance (m) from the stop line to the near edge of the crosswalk band.
    /// The whole band (this + <see cref="CrosswalkDepth"/>) must fit inside the strip
    /// StopLineCache reserves at signalized approaches
    /// (<see cref="SimConstants.SignalCrosswalkSetback"/>) or the bars extend into the junction.</summary>
    private const float CrosswalkStartOffset = 1.3f;
    /// <summary>Zoom below which editor node dots are hidden (they are an aid, not scenery).</summary>
    private const float NodeDotMinZoom = 0.5f;

    /// <summary>Cached offset paths per edge, invalidated when graph version changes.</summary>
    private readonly List<CachedEdgePaths> _cache = new();
    /// <summary>Graph version when the cache was last rebuilt.</summary>
    private int _cachedVersion = -1;
    /// <summary>Ambient lighting factor for the current frame (1.0 = full day). Set at the start of Draw.</summary>
    private float _ambient = 1f;
    /// <summary>
    /// Heat-map overlay active for the current frame, or null when the overlay is disabled.
    /// Set at the start of <see cref="Draw"/> and consumed by <see cref="DrawRoadSurface"/>.
    /// </summary>
    private CongestionHeatMap? _heatMap;

    // Reusable paints (avoid per-frame allocation)
    private readonly SKPaint _surfacePaint = new()
    {
        Color = new SKColor(70, 72, 78),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true
    };

    private readonly SKPaint _intersectionFillPaint = new()
    {
        Color = new SKColor(70, 72, 78),
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };

    private readonly SKPaint _nodePaint = new()
    {
        Color = new SKColor(120, 130, 150),
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };

    // Shared-lane dash effect (also used for dashed edge lines) — immutable and shared
    // across all edges/frames (created once to honor the per-frame no-allocation
    // discipline of the paints above).
    private readonly SKPathEffect _centerLineDash = SKPathEffect.CreateDash(new[] { 2f, 2f }, 0);

    // Reusable paints for DrawRoadLines (StrokeWidth updated per frame based on zoom)
    private readonly SKPaint _edgeLinePaint = new()
    {
        Color = new SKColor(200, 200, 200),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
    // Solid yellow center paint: arterial double-yellow and highway median edge lines.
    private readonly SKPaint _centerLinePaint = new()
    {
        Color = new SKColor(200, 166, 60),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
    // Dark curb line at the asphalt edge of residential/arterial roads.
    private readonly SKPaint _curbPaint = new()
    {
        Color = new SKColor(45, 46, 50),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
    // Highway median band stroked along the center path.
    private readonly SKPaint _medianPaint = new()
    {
        Color = new SKColor(96, 100, 92),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
    // Shoulder/sidewalk/verge band stroked under the asphalt (color set per edge type).
    private readonly SKPaint _shoulderPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true
    };
    // Worn tire tracks on dirt roads (long ragged dashes).
    private readonly SKPaint _tireTrackPaint = new()
    {
        Color = new SKColor(96, 76, 48),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        PathEffect = SKPathEffect.CreateDash(new[] { 7f, 2f }, 0),
        IsAntialias = true
    };
    // Continental crosswalk bars at signalized approaches.
    private readonly SKPaint _crosswalkPaint = new()
    {
        Color = new SKColor(255, 255, 255, 200),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Butt,
        IsAntialias = true
    };
    // Stop bars at light/stop approaches (width set per frame from zoom).
    private readonly SKPaint _stopLinePaint = new()
    {
        Color = new SKColor(255, 255, 255),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Butt,
        IsAntialias = true
    };
    // Intersection corner boundary lines (color set per node type, width per frame).
    private readonly SKPaint _cornerPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Butt,
        IsAntialias = true
    };

    // Per-node scratch state for DrawIntersectionFills — reused across nodes/frames so the
    // per-visible-node fill pass never allocates.
    private readonly List<ApproachInfo> _approaches = new();
    private readonly HashSet<int> _seenPairs = new();
    private readonly SKPath _fillPath = new();
    private readonly SKPath _cornerPath = new();
    private readonly SKPaint _laneDividerPaint = new()
    {
        Color = new SKColor(180, 180, 180, 160),
        Style = SKPaintStyle.Stroke,
        PathEffect = SKPathEffect.CreateDash(new[] { 1.5f, 1.5f }, 0),
        IsAntialias = true
    };
    // White chevrons drawn down each lane of a one-way road to show travel direction.
    private readonly SKPaint _arrowPaint = new()
    {
        Color = new SKColor(210, 210, 210, 220),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true
    };

    /// <summary>
    /// Draws all active road edges in three passes (shoulders/sidewalks first, then asphalt
    /// surfaces, then markings) so that connecting roads don't overlap each other's lines
    /// and junction interiors stay clean. Also draws stop lines, crosswalks, and nodes.
    /// For paired edges (forward/reverse), only the lower-index edge is drawn to avoid overdraw.
    /// When <paramref name="heatMap"/> is non-null and <see cref="CongestionHeatMap.Enabled"/>
    /// is true, a congestion tint is alpha-blended over each road surface after the base
    /// color is applied, without affecting lane markings or the road type styling.
    /// When <paramref name="trafficSignals"/> is non-null and zoom is at least
    /// <see cref="CrosswalkMinZoom"/>, continental crosswalks are drawn at each signalized approach.
    ///
    /// Frustum culling: edges whose endpoint AABB does not intersect <paramref name="viewRect"/>
    /// are skipped in all passes. A generous margin is added to the AABB so that Bézier
    /// curves bowing outside their endpoint bounding box are never incorrectly culled.
    ///
    /// Level-of-Detail: when <paramref name="zoom"/> is below
    /// <see cref="RenderDetail.RoadSimpleThreshold"/>, roads are drawn as brightened
    /// center-line strokes whose width grades by road type, and all other passes are
    /// skipped. Shoulders are skipped below <see cref="ShoulderMinZoom"/>.
    /// </summary>
    /// <param name="canvas">SkiaSharp canvas in world-space coordinates.</param>
    /// <param name="graph">Road graph to render.</param>
    /// <param name="stopLines">Stop-line cache for marking trim t-values.</param>
    /// <param name="zoom">Current camera zoom (world units per screen pixel).</param>
    /// <param name="darkness">Ambient darkness factor (0 = full day, 1 = full night).</param>
    /// <param name="heatMap">Optional congestion heat-map overlay; null disables it.</param>
    /// <param name="viewRect">Visible world-space rectangle for frustum culling.</param>
    /// <param name="visibleEdges">Optional pre-culled edge index list from the edge spatial grid.</param>
    /// <param name="stopSigns">Optional stop-sign system for stop-line exemption checks.</param>
    /// <param name="trafficSignals">Optional signal system; enables crosswalk rendering at lights.</param>
    public void Draw(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom,
        float darkness = 0f, CongestionHeatMap? heatMap = null,
        SKRect viewRect = default, IReadOnlyList<int>? visibleEdges = null,
        StopSignSystem? stopSigns = null, TrafficSignalSystem? trafficSignals = null)
    {
        // An edge-less graph still flows through every pass: each pass iterates the (empty)
        // edge list and skips naturally, while DrawNodes at the end still renders any
        // stand-alone nodes placed on an otherwise empty map.

        // When the caller supplies a pre-culled visible-edge list (from the edge spatial grid),
        // the surface/marking passes iterate only those instead of the whole edge list. The
        // per-edge IsVisible check below still runs as a precise refinement of the grid's superset.
        int visCount = visibleEdges?.Count ?? graph.Edges.Count;

        bool lodSimple = zoom < RenderDetail.RoadSimpleThreshold;

        // Store heat-map reference for use inside DrawRoadSurface (cleared after the pass)
        _heatMap = (!lodSimple && heatMap != null && heatMap.Enabled) ? heatMap : null;

        // Dim road colors at night
        _ambient = 1f - darkness * 0.45f;
        // Default surface paint color (intersection fills use this; per-edge surfaces override it)
        _surfacePaint.Color = RoadTypeVisuals.GetSurfaceColor(RoadType.Residential, _ambient);
        _intersectionFillPaint.Color = _surfacePaint.Color;
        _nodePaint.Color = new SKColor(Dim(120, _ambient), Dim(130, _ambient), Dim(150, _ambient), 120);
        _edgeLinePaint.Color = new SKColor(Dim(200, _ambient), Dim(200, _ambient), Dim(200, _ambient));
        _centerLinePaint.Color = new SKColor(Dim(200, _ambient), Dim(166, _ambient), Dim(60, _ambient));
        // RGB dims; alpha stays constant (scaling both would double-dim the markings).
        _laneDividerPaint.Color = new SKColor(Dim(180, _ambient), Dim(180, _ambient), Dim(180, _ambient), 160);
        _arrowPaint.Color = new SKColor(Dim(210, _ambient), Dim(210, _ambient), Dim(210, _ambient), 220);
        _curbPaint.Color = new SKColor(Dim(45, _ambient), Dim(46, _ambient), Dim(50, _ambient));
        _medianPaint.Color = new SKColor(Dim(96, _ambient), Dim(100, _ambient), Dim(92, _ambient));
        _tireTrackPaint.Color = new SKColor(Dim(96, _ambient), Dim(76, _ambient), Dim(48, _ambient));
        _crosswalkPaint.Color = new SKColor(Dim(255, _ambient), Dim(255, _ambient), Dim(255, _ambient), 200);
        _stopLinePaint.Color = new SKColor(Dim(255, _ambient), Dim(255, _ambient), Dim(255, _ambient));

        RebuildCacheIfNeeded(graph, stopLines);

        // viewRect is only used for culling; a zero/empty rect means no culling (backward-compat
        // with callers that do not supply it, e.g. signal/sign helper methods).
        bool cull = viewRect.Width > 0f || viewRect.Height > 0f;

        // LOD simple: draw roads as plain center-line strokes at city-overview zoom.
        // Stroke width and (brightened) color grade by road type so the network hierarchy
        // stays legible when roads collapse to lines.
        if (lodSimple)
        {
            for (int i = 0; i < graph.Edges.Count; i++)
            {
                if (i >= _cache.Count) break;
                var edge = graph.Edges[i];
                if (edge.FromNode < 0) continue;
                int reverse = graph.FindReverseEdge(i);
                if (reverse >= 0 && reverse < i) continue;

                if (cull)
                {
                    var from = graph.Nodes[edge.FromNode].Position;
                    var to   = graph.Nodes[edge.ToNode].Position;
                    float halfW = GeometryUtil.RoadHalfWidth(graph, i);
                    if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to,
                            edge.ControlPoint1, edge.ControlPoint2, halfW), viewRect))
                        continue;
                }

                _surfacePaint.StrokeWidth = RoadTypeVisuals.GetSchematicStrokeWidth(edge.RoadType) / zoom;
                _surfacePaint.Color = RoadTypeVisuals.GetSchematicColor(edge.RoadType, _ambient);
                canvas.DrawPath(_cache[i].CenterPath, _surfacePaint);
            }
            return; // skip all detail passes
        }

        // Pass 0: shoulder/sidewalk/verge bands under everything — ALL visible edges'
        // shoulders are drawn before any asphalt so junction interiors stay clean once
        // the asphalt and intersection fills go on top. Sub-pixel at low zoom, so skipped.
        if (zoom >= ShoulderMinZoom)
        {
            for (int k = 0; k < visCount; k++)
            {
                int i = visibleEdges != null ? visibleEdges[k] : k;
                if (i >= _cache.Count) continue;
                var edge = graph.Edges[i];
                if (edge.FromNode < 0) continue;
                int reverse = graph.FindReverseEdge(i);
                if (reverse >= 0 && reverse < i) continue;

                if (cull)
                {
                    var from = graph.Nodes[edge.FromNode].Position;
                    var to   = graph.Nodes[edge.ToNode].Position;
                    float halfW = GeometryUtil.RoadHalfWidth(graph, i);
                    if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to,
                            edge.ControlPoint1, edge.ControlPoint2, halfW), viewRect))
                        continue;
                }

                var cached = _cache[i];
                float asphaltWidth = cached.TotalWidth * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType);
                _shoulderPaint.Color = RoadTypeVisuals.GetShoulderColor(edge.RoadType, _ambient);
                _shoulderPaint.StrokeWidth = asphaltWidth + 2f * RoadTypeVisuals.GetShoulderWidth(edge.RoadType);
                canvas.DrawPath(cached.CenterPath, _shoulderPaint);
            }
        }

        // Pass 1: draw all asphalt surfaces first so overlapping roads blend seamlessly
        for (int k = 0; k < visCount; k++)
        {
            int i = visibleEdges != null ? visibleEdges[k] : k;
            if (i >= _cache.Count) continue;
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            if (cull)
            {
                var from = graph.Nodes[edge.FromNode].Position;
                var to   = graph.Nodes[edge.ToNode].Position;
                float halfW = GeometryUtil.RoadHalfWidth(graph, i);
                if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to,
                        edge.ControlPoint1, edge.ControlPoint2, halfW), viewRect))
                    continue;
            }

            DrawRoadSurface(canvas, edge, i);
        }

        // Heat-map field is only valid during Pass 1; clear it so stale data is never
        // accidentally accessed by intersection-fill or marking passes.
        _heatMap = null;

        // Pass 1.5: fill intersection interiors and draw corner curves
        DrawIntersectionFills(canvas, graph, stopLines, zoom, viewRect);

        // Pass 2: draw all lane markings and boundary lines on top
        for (int k = 0; k < visCount; k++)
        {
            int i = visibleEdges != null ? visibleEdges[k] : k;
            if (i >= _cache.Count) continue;
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            if (cull)
            {
                var from = graph.Nodes[edge.FromNode].Position;
                var to   = graph.Nodes[edge.ToNode].Position;
                float halfW = GeometryUtil.RoadHalfWidth(graph, i);
                if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to,
                        edge.ControlPoint1, edge.ControlPoint2, halfW), viewRect))
                    continue;
            }

            DrawRoadLines(canvas, edge, i, zoom);

            // One-way roads (no reverse edge) get direction chevrons down each lane.
            if (reverse < 0 && RoadTypeVisuals.HasPaintedLines(edge.RoadType))
                DrawOneWayArrows(canvas, graph, i, zoom);
        }

        DrawStopLines(canvas, graph, stopLines, zoom, viewRect, stopSigns);
        if (trafficSignals != null && zoom >= CrosswalkMinZoom)
            DrawCrosswalks(canvas, graph, stopLines, trafficSignals, viewRect);
        DrawNodes(canvas, graph, zoom, viewRect);
    }

    /// <summary>
    /// Draws white direction chevrons along each lane of a one-way road so the travel
    /// direction is visible. Chevrons are placed at a couple of points between the edge's
    /// stop-line trims (so they never sit inside an intersection) and oriented along the
    /// curve tangent. Lane positions come from <see cref="GeometryUtil.LaneLateralOffset"/>,
    /// so they sit on the centered one-way lanes.
    /// </summary>
    private void DrawOneWayArrows(SKCanvas canvas, RoadGraph graph, int edgeIndex, float zoom)
    {
        var edge = graph.Edges[edgeIndex];
        float size = MathF.Max(2.5f, 4f / zoom);
        _arrowPaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);

        // Keep chevrons clear of the intersection by sampling inside the trimmed range.
        // (Two markers per lane reads clearly without cluttering short driveways.)
        ReadOnlySpan<float> ts = stackalloc float[] { 0.4f, 0.7f };
        foreach (float t in ts)
        {
            var pos = graph.EvaluateBezier(edgeIndex, t);
            var tan = graph.EvaluateBezierTangent(edgeIndex, t);
            float len = tan.Length();
            if (len < 0.001f) continue;
            float dx = tan.X / len, dy = tan.Y / len;
            float rx = -dy, ry = dx; // right normal (Y-down)

            for (int lane = 0; lane < edge.LaneCount; lane++)
            {
                float off = GeometryUtil.LaneLateralOffset(graph, edgeIndex, lane);
                float cx = pos.X + rx * off, cy = pos.Y + ry * off;
                // Chevron ">" pointing in travel direction.
                float tipX = cx + dx * size * 0.5f, tipY = cy + dy * size * 0.5f;
                float baseX = cx - dx * size * 0.5f, baseY = cy - dy * size * 0.5f;
                float halfBarb = size * 0.45f;
                canvas.DrawLine(tipX, tipY, baseX + rx * halfBarb, baseY + ry * halfBarb, _arrowPaint);
                canvas.DrawLine(tipX, tipY, baseX - rx * halfBarb, baseY - ry * halfBarb, _arrowPaint);
            }
        }
    }

    /// <summary>Rebuilds the per-edge path cache if the graph version has changed.</summary>
    private void RebuildCacheIfNeeded(RoadGraph graph, StopLineCache stopLines)
    {
        if (_cachedVersion == graph.Version) return;

        // Dispose old SKPath objects before clearing to prevent memory leak
        foreach (var entry in _cache)
            entry?.Dispose();
        _cache.Clear();
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0)
            {
                _cache.Add(null!); // placeholder for defunct edge
                continue;
            }
            _cache.Add(BuildEdgePaths(graph, i, stopLines));
        }
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Builds the cached Bezier paths for a single edge: center path (full length for surface),
    /// left/right boundary paths, lane dividers, and per-type marking paths (arterial
    /// double-yellow, highway median yellows, dirt tire tracks) — all marking paths trimmed
    /// to stop line t-values so they don't extend into intersections.
    /// </summary>
    private CachedEdgePaths BuildEdgePaths(RoadGraph graph, int edgeIndex, StopLineCache stopLines)
    {
        var edge = graph.Edges[edgeIndex];
        float totalWidth = GeometryUtil.RoadSurfaceWidth(graph, edgeIndex);
        // Boundary lines (curbs / white edge lines) sit at the DRAWN asphalt edge, which is
        // the geometric half-width scaled by the per-type visual multiplier — otherwise the
        // curb floats inside the roadway on arterials/highways (drawn wider than geometric).
        float halfWidth = totalWidth * 0.5f * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType);
        bool hasCenterLine = GeometryUtil.HasCenterDivider(graph, edgeIndex);
        bool dashedEdges = (edge.Flags & EdgeFlags.SharedLane) != 0;

        // Build center path (full length — surface needs to extend into intersection)
        var centerPath = BuildBezierPath(graph, edgeIndex);

        // Trim markings at intersection boundaries using stop line t-values.
        // Center line and lane dividers use the overall (max) stop-T.
        float tMin = stopLines.GetStopTAtFromNode(edgeIndex);
        float tMax = stopLines.GetStopTAtToNode(edgeIndex);

        // Per-side trims for boundary lines (asymmetric at acute-angle intersections)
        float tMinLeft = stopLines.GetLeftTrimAtFromNode(edgeIndex);
        float tMaxLeft = stopLines.GetLeftTrimAtToNode(edgeIndex);
        float tMinRight = stopLines.GetRightTrimAtFromNode(edgeIndex);
        float tMaxRight = stopLines.GetRightTrimAtToNode(edgeIndex);

        // The trimmed center path is only stroked for the highway median band. Two-way
        // arterials mark the center with the double-yellow pair below; residential
        // two-ways carry no center marking at all. Keep an empty path otherwise so the
        // cache shape is uniform.
        var centerLinePath = hasCenterLine && edge.RoadType == RoadType.Highway
            ? BuildOffsetPath(graph, edgeIndex, 0f, tMin, tMax)
            : new SKPath();

        // Build edge boundary paths with per-side trims
        var leftPath = BuildOffsetPath(graph, edgeIndex, -halfWidth, tMinLeft, tMaxLeft);
        var rightPath = BuildOffsetPath(graph, edgeIndex, halfWidth, tMinRight, tMaxRight);

        // Build lane divider paths (trimmed to overall stop-T, multi-lane only).
        var lanePaths = new List<SKPath>();
        if (edge.LaneCount > 1)
        {
            if (hasCenterLine)
            {
                // Two-way: dividers mirrored on each side of the center divider (the lanes
                // of this direction and the opposing direction).
                for (int lane = 1; lane < edge.LaneCount; lane++)
                {
                    float offset = lane * LaneWidth;
                    lanePaths.Add(BuildOffsetPath(graph, edgeIndex, offset, tMin, tMax));
                    lanePaths.Add(BuildOffsetPath(graph, edgeIndex, -offset, tMin, tMax));
                }
            }
            else
            {
                // One-way: dividers at the interior boundaries of the centered lane set.
                for (int lane = 1; lane < edge.LaneCount; lane++)
                {
                    float offset = (lane - edge.LaneCount * 0.5f) * LaneWidth;
                    lanePaths.Add(BuildOffsetPath(graph, edgeIndex, offset, tMin, tMax));
                }
            }
        }

        // Per-type marking paths. Contents depend on the road type:
        //   Dirt      — a tire-track pair (±TireTrackSpread) around each lane center; on a
        //               two-way road the opposing direction's lanes are mirrored across the
        //               center path (only the lower-index edge of a pair is drawn).
        //   Arterial  — double solid yellow at ±ArterialYellowOffset (two-way only).
        //   Highway   — solid yellow at ±HighwayYellowOffset flanking the median (two-way only).
        var typeMarkingPaths = new List<SKPath>();
        if (edge.RoadType == RoadType.Dirt)
        {
            for (int lane = 0; lane < edge.LaneCount; lane++)
            {
                float laneOff = GeometryUtil.LaneLateralOffset(graph, edgeIndex, lane);
                typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, laneOff - TireTrackSpread, tMin, tMax));
                typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, laneOff + TireTrackSpread, tMin, tMax));
                if (hasCenterLine)
                {
                    // Opposing-direction lanes mirrored across the center divider.
                    typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, -laneOff - TireTrackSpread, tMin, tMax));
                    typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, -laneOff + TireTrackSpread, tMin, tMax));
                }
            }
        }
        else if (hasCenterLine && edge.RoadType == RoadType.Arterial)
        {
            typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, -ArterialYellowOffset, tMin, tMax));
            typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, ArterialYellowOffset, tMin, tMax));
        }
        else if (hasCenterLine && edge.RoadType == RoadType.Highway)
        {
            typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, -HighwayYellowOffset, tMin, tMax));
            typeMarkingPaths.Add(BuildOffsetPath(graph, edgeIndex, HighwayYellowOffset, tMin, tMax));
        }

        return new CachedEdgePaths(centerPath, centerLinePath, leftPath, rightPath, lanePaths,
            typeMarkingPaths, totalWidth, hasCenterLine, dashedEdges);
    }

    /// <summary>
    /// Builds an SKPath by sampling the edge's cubic Bezier curve at BezierSegments intervals.
    /// </summary>
    private static SKPath BuildBezierPath(RoadGraph graph, int edgeIndex)
    {
        var path = new SKPath();
        var p0 = graph.EvaluateBezier(edgeIndex, 0f);
        path.MoveTo(p0.X, p0.Y);

        for (int s = 1; s <= BezierSegments; s++)
        {
            float t = s / (float)BezierSegments;
            var pt = graph.EvaluateBezier(edgeIndex, t);
            path.LineTo(pt.X, pt.Y);
        }

        return path;
    }

    /// <summary>
    /// Builds an SKPath offset perpendicular to the Bezier curve by the given distance,
    /// trimmed to the parametric range [tMin, tMax] so markings don't extend into intersections.
    /// Positive offset shifts right of travel direction; negative shifts left.
    /// </summary>
    private static SKPath BuildOffsetPath(RoadGraph graph, int edgeIndex, float offset,
        float tMin = 0f, float tMax = 1f)
    {
        var path = new SKPath();
        bool first = true;

        void AddPoint(float t)
        {
            var pos = graph.EvaluateBezier(edgeIndex, t);
            var tangent = graph.EvaluateBezierTangent(edgeIndex, t);
            float len = tangent.Length();
            if (len < 0.001f) return;
            var normal = new Vector2(-tangent.Y / len, tangent.X / len);
            var offsetPos = pos + normal * offset;

            if (first)
            {
                path.MoveTo(offsetPos.X, offsetPos.Y);
                first = false;
            }
            else
            {
                path.LineTo(offsetPos.X, offsetPos.Y);
            }
        }

        // Start at exact tMin so the boundary begins precisely at the stop line
        if (tMin > 0f) AddPoint(tMin);

        // Strict inequalities so the true endpoints (t=0 when tMin=0, t=1 when tMax=1) are
        // included. Using <=/>= dropped the first and last grid segment, leaving an
        // un-marked dead zone of length/BezierSegments at each untrimmed end — which grows
        // with road length. (Trimmed ends, tMin>0 / tMax<1, are still anchored by the
        // explicit AddPoint(tMin)/(tMax) calls above and below.)
        for (int s = 0; s <= BezierSegments; s++)
        {
            float t = s / (float)BezierSegments;
            if (t < tMin || t > tMax) continue;
            AddPoint(t);
        }

        // End at exact tMax so the boundary ends precisely at the stop line
        if (tMax < 1f) AddPoint(tMax);

        return path;
    }

    /// <summary>
    /// Draws the asphalt surface for a single road edge. Surface color and stroke width
    /// are derived from the edge's <see cref="RoadType"/> via <see cref="RoadTypeVisuals"/>
    /// so each classification is visually distinct; lane geometry is not affected.
    /// When a <see cref="CongestionHeatMap"/> is active, a semi-transparent congestion tint
    /// is drawn over the base surface on the same path. At zero congestion the tint is fully
    /// transparent so the road looks exactly as it does without the heat-map.
    /// </summary>
    private void DrawRoadSurface(SKCanvas canvas, RoadEdge edge, int edgeIndex)
    {
        var cached = _cache[edgeIndex];
        float strokeWidth = cached.TotalWidth * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType);
        _surfacePaint.Color = RoadTypeVisuals.GetSurfaceColor(edge.RoadType, _ambient);
        _surfacePaint.StrokeWidth = strokeWidth;
        canvas.DrawPath(cached.CenterPath, _surfacePaint);

        // Blend congestion tint over the base surface (skipped when overlay is off or
        // congestion is zero/near-zero so alpha is effectively 0).
        if (_heatMap != null)
        {
            float congestion = _heatMap.GetCongestion(edgeIndex);
            if (congestion > 0.005f)
            {
                var overlayPaint = _heatMap.GetOverlayPaint(congestion, strokeWidth);
                canvas.DrawPath(cached.CenterPath, overlayPaint);
            }
        }
    }

    /// <summary>
    /// Draws boundary treatment, center marking, and lane dividers for a single road edge,
    /// following the per-type policy: dirt shows only worn tire tracks (no paint);
    /// residential/arterial get a thin dark curb line at the asphalt edge (white solid edge
    /// lines remain highway-only); the center of a two-way road is unmarked on residential,
    /// double solid yellow on arterial, and a median band flanked by yellow lines on highway.
    /// White dashed lane dividers apply to all multi-lane paved types.
    /// All marking paths are trimmed at intersection boundaries.
    /// </summary>
    private void DrawRoadLines(SKCanvas canvas, RoadEdge edge, int edgeIndex, float zoom)
    {
        var cached = _cache[edgeIndex];

        // Unpaved roads (dirt) have no painted markings — worn tire tracks instead.
        // Tracks are sub-pixel below mid zoom, so skip them there.
        if (!RoadTypeVisuals.HasPaintedLines(edge.RoadType))
        {
            if (zoom >= ShoulderMinZoom)
            {
                _tireTrackPaint.StrokeWidth = TireTrackWidth;
                foreach (var trackPath in cached.TypeMarkingPaths)
                    canvas.DrawPath(trackPath, _tireTrackPaint);
            }
            return;
        }

        // Edge boundary treatment: bright white solid lines are a highway signature;
        // residential/arterial get a thin dark curb line at the asphalt edge instead.
        // Single-lane two-way roads dash both edges to signal a shared single track.
        SKPaint boundaryPaint;
        if (edge.RoadType == RoadType.Highway)
        {
            boundaryPaint = _edgeLinePaint;
            boundaryPaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);
        }
        else
        {
            boundaryPaint = _curbPaint;
            boundaryPaint.StrokeWidth = Math.Max(0.25f, 0.4f / zoom);
        }
        boundaryPaint.PathEffect = cached.DashedEdges ? _centerLineDash : null;
        canvas.DrawPath(cached.LeftEdgePath, boundaryPaint);
        canvas.DrawPath(cached.RightEdgePath, boundaryPaint);
        boundaryPaint.PathEffect = null;

        // Center marking policy — only two-way roads have a center divider to mark.
        if (cached.HasCenterLine)
        {
            switch (edge.RoadType)
            {
                case RoadType.Arterial:
                    // Double solid yellow.
                    _centerLinePaint.StrokeWidth = MathF.Max(0.15f, 0.2f / zoom);
                    foreach (var yellowPath in cached.TypeMarkingPaths)
                        canvas.DrawPath(yellowPath, _centerLinePaint);
                    break;
                case RoadType.Highway:
                    // Median band with a solid yellow line along each side.
                    _medianPaint.StrokeWidth = HighwayMedianWidth;
                    canvas.DrawPath(cached.CenterLinePath, _medianPaint);
                    _centerLinePaint.StrokeWidth = MathF.Max(0.15f, 0.2f / zoom);
                    foreach (var yellowPath in cached.TypeMarkingPaths)
                        canvas.DrawPath(yellowPath, _centerLinePaint);
                    break;
                // Residential: real residential streets carry no center line.
            }
        }

        // Lane dividers (white dashed, multi-lane only)
        if (cached.LanePaths.Count > 0)
        {
            _laneDividerPaint.StrokeWidth = Math.Max(0.2f, 0.3f / zoom);
            foreach (var lanePath in cached.LanePaths)
            {
                canvas.DrawPath(lanePath, _laneDividerPaint);
            }
        }
    }

    /// <summary>True if (x,y) is within the cull rect (expanded by <paramref name="margin"/>), or
    /// culling is off (an empty rect means "draw everything"). Used to skip off-screen node/edge work.</summary>
    private static bool InView(float x, float y, SKRect cull, float margin)
        => (cull.Width <= 0f && cull.Height <= 0f)
           || (x >= cull.Left - margin && x <= cull.Right + margin
               && y >= cull.Top - margin && y <= cull.Bottom + margin);

    /// <summary>
    /// Draws white stop lines wherever a vehicle actually stops (traffic-light approaches
    /// and non-exempt stop-sign approaches; never on dirt).
    /// </summary>
    private void DrawStopLines(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom,
        SKRect cullRect = default, StopSignSystem? stopSigns = null)
    {
        _stopLinePaint.StrokeWidth = Math.Max(0.4f, 0.6f / zoom);

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            // Dirt roads carry no painted markings, so no stop line (the stop still applies).
            if (edge.RoadType == RoadType.Dirt) continue;

            var toPos = graph.Nodes[edge.ToNode].Position;
            if (!InView(toPos.X, toPos.Y, cullRect, 40f)) continue;

            // A stop line is painted only where a vehicle actually stops: a traffic-light approach
            // (stops at red), or a stop-sign approach that is NOT exempt. Yield and uncontrolled
            // nodes get none — as does the exempt main-road approach at a minor-road stop (e.g. a
            // home driveway), which has neither a stop sign nor a light of its own.
            var toFlags = graph.Nodes[edge.ToNode].Flags;
            bool stops = (toFlags & NodeFlags.TrafficLight) != 0
                      || ((toFlags & NodeFlags.StopSign) != 0 && (stopSigns == null || !stopSigns.IsEdgeExempt(i)));
            if (stops)
            {
                float stopT = stopLines.GetStopTAtToNode(i);
                if (stopT < 0.999f)
                    DrawStopLine(canvas, graph, i, edge, stopT, _stopLinePaint);
            }
        }
    }

    /// <summary>
    /// Draws a single stop line perpendicular to the edge at parameter t,
    /// spanning the right side (travel lanes) only.
    /// </summary>
    private void DrawStopLine(SKCanvas canvas, RoadGraph graph, int edgeIndex, RoadEdge edge, float t, SKPaint paint)
    {
        var center = graph.EvaluateBezier(edgeIndex, t);
        var tangent = graph.EvaluateBezierTangent(edgeIndex, t);
        float len = tangent.Length();
        if (len < 0.001f) return;

        // Normal perpendicular to tangent (right side in Y-down coords)
        float nx = -tangent.Y / len;
        float ny = tangent.X / len;

        // Span exactly this edge's approaching lanes: two-way is the right half (0..+halfWidth);
        // one-way / single-lane two-way span the full centered width.
        var (spanMin, spanMax) = GeometryUtil.LaneSpan(graph, edgeIndex);
        canvas.DrawLine(center.X + nx * spanMin, center.Y + ny * spanMin,
            center.X + nx * spanMax, center.Y + ny * spanMax, paint);
    }

    /// <summary>
    /// Draws a US continental crosswalk just downstream of the stop line at every paved
    /// approach whose destination node has a traffic light: white bars ELONGATED ALONG
    /// THE TRAVEL DIRECTION (<see cref="CrosswalkDepth"/> long, <see cref="CrosswalkBarWidth"/>
    /// wide — the realistic orientation, perpendicular to the stop line), stacked evenly
    /// across the full roadway. The bar count is derived from the roadway span at a fixed
    /// bar+gap pitch and the pattern is centered, so wide arterials get proportionally
    /// more bars than a narrow residential street (~three per 3.5 m lane). Dirt approaches
    /// are skipped. Caller gates on zoom (drawn at <see cref="CrosswalkMinZoom"/> and
    /// above) and only calls this when a <see cref="TrafficSignalSystem"/> was supplied to Draw.
    /// </summary>
    private void DrawCrosswalks(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines,
        TrafficSignalSystem trafficSignals, SKRect cullRect = default)
    {
        _crosswalkPaint.StrokeWidth = CrosswalkBarWidth;

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.RoadType == RoadType.Dirt) continue; // no paint on dirt
            if (!trafficSignals.IsTrafficLight(edge.ToNode)) continue;

            var toPos = graph.Nodes[edge.ToNode].Position;
            if (!InView(toPos.X, toPos.Y, cullRect, 40f)) continue;

            float stopT = stopLines.GetStopTAtToNode(i);
            if (stopT >= 0.999f) continue; // no stop line inset — no room for a crosswalk

            var center = graph.EvaluateBezier(i, stopT);
            var tangent = graph.EvaluateBezierTangent(i, stopT);
            float len = tangent.Length();
            if (len < 0.001f) continue;

            // Travel direction points toward the node, so "downstream of the stop line"
            // (between the stop line and the intersection) is +tangent.
            float dx = tangent.X / len, dy = tangent.Y / len;
            float nx = -dy, ny = dx; // right normal (Y-down) = across the road

            // A crosswalk crosses the FULL roadway (unlike stop lines, which span only the
            // approaching lanes): on two-way roads the path is the center divider and
            // LaneSpan covers just the right half, so mirror it across the center.
            var (spanMin, spanMax) = GeometryUtil.LaneSpan(graph, i);
            if (GeometryUtil.HasCenterDivider(graph, i))
                spanMin = -spanMax;

            // Fit as many bars as the roadway width allows at the fixed pitch, centered
            // laterally so the pattern is symmetric with no partial bar at either curb.
            float pitch = CrosswalkBarWidth + CrosswalkBarGap;
            float span = spanMax - spanMin;
            int bars = Math.Max(2, (int)((span + CrosswalkBarGap) / pitch));
            float used = bars * pitch - CrosswalkBarGap;
            float s0 = spanMin + (span - used) * 0.5f + CrosswalkBarWidth * 0.5f;

            float near = CrosswalkStartOffset;
            float far = CrosswalkStartOffset + CrosswalkDepth;
            for (int bar = 0; bar < bars; bar++)
            {
                float s = s0 + bar * pitch;
                float bx = center.X + nx * s, by = center.Y + ny * s;
                canvas.DrawLine(bx + dx * near, by + dy * near,
                    bx + dx * far, by + dy * far, _crosswalkPaint);
            }
        }
    }

    /// <summary>
    /// Draws small translucent circles at each active intersection/endpoint node. Node dots
    /// are an editor aid rather than scenery, so they are hidden below
    /// <see cref="NodeDotMinZoom"/> and kept small and faint at all zooms.
    /// </summary>
    private void DrawNodes(SKCanvas canvas, RoadGraph graph, float zoom, SKRect cullRect = default)
    {
        if (zoom < NodeDotMinZoom) return;
        float nodeRadius = RenderDetail.NodeDotRadius(zoom);

        for (int i = 0; i < graph.Nodes.Count; i++)
        {
            var node = graph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue; // skip defunct nodes
            if (!InView(node.Position.X, node.Position.Y, cullRect, nodeRadius + 10f)) continue;
            canvas.DrawCircle(node.Position.X, node.Position.Y, nodeRadius, _nodePaint);
        }
    }

    // ── Intersection fills ──────────────────────────────────────────────

    /// <summary>
    /// Per-road approach data at an intersection node: boundary endpoints and direction,
    /// used to build the intersection fill polygon with curved corners.
    /// Left/right are defined looking away from the node.
    /// </summary>
    private struct ApproachInfo
    {
        public float Angle;
        public Vector2 LeftBound;
        public Vector2 RightBound;
        public Vector2 AwayDir;      // overall direction (for sorting)
        public Vector2 LeftAwayDir;   // tangent at left boundary's trim-t
        public Vector2 RightAwayDir;  // tangent at right boundary's trim-t
    }

    /// <summary>
    /// Fills intersection interiors with asphalt and draws curved boundary lines connecting
    /// adjacent roads at each intersection node. All per-node scratch state (approach list,
    /// pair set, fill/corner paths, corner paint) lives in reusable fields — this runs for
    /// every visible node every frame, so it must not allocate.
    /// </summary>
    private void DrawIntersectionFills(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom, SKRect cullRect = default)
    {
        _cornerPaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);

        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            // Cull before the expensive CollectApproaches (Bezier evals + sort) for off-screen nodes.
            if (!InView(node.Position.X, node.Position.Y, cullRect, 60f)) continue;

            CollectApproaches(graph, stopLines, n);
            var approaches = _approaches;
            if (approaches.Count < 2) continue;

            approaches.Sort(static (a, b) => a.Angle.CompareTo(b.Angle));

            int count = approaches.Count;

            // The intersection takes on the appearance of its highest-ranked incident road
            // (e.g. a highway+arterial node looks like highway; an all-dirt node looks like dirt).
            RoadType nodeType = HighestRoadTypeAtNode(graph, n);
            _intersectionFillPaint.Color = RoadTypeVisuals.GetSurfaceColor(nodeType, _ambient);

            // Build intersection fill polygon: for each road, a straight segment along the
            // stop line (left→right boundary), then a curve to the next road's left boundary.
            _fillPath.Reset();
            _fillPath.MoveTo(approaches[0].LeftBound.X, approaches[0].LeftBound.Y);

            for (int i = 0; i < count; i++)
            {
                var curr = approaches[i];
                var next = approaches[(i + 1) % count];

                _fillPath.LineTo(curr.RightBound.X, curr.RightBound.Y);
                AddCornerCurve(_fillPath, curr.RightBound, curr.RightAwayDir,
                    next.LeftBound, next.LeftAwayDir);
            }

            _fillPath.Close();
            canvas.DrawPath(_fillPath, _intersectionFillPaint);

            // Corner boundary lines — unpaved (dirt) intersections carry no paint, matching
            // the per-edge rule, so skip them when the intersection's appearance is dirt.
            // The color matches the per-type edge treatment: white lines at highway
            // intersections, dark curbs at residential/arterial ones.
            if (!RoadTypeVisuals.HasPaintedLines(nodeType)) continue;

            _cornerPaint.Color = nodeType == RoadType.Highway ? _edgeLinePaint.Color : _curbPaint.Color;

            for (int i = 0; i < count; i++)
            {
                var curr = approaches[i];
                var next = approaches[(i + 1) % count];

                _cornerPath.Reset();
                _cornerPath.MoveTo(curr.RightBound.X, curr.RightBound.Y);
                AddCornerCurve(_cornerPath, curr.RightBound, curr.RightAwayDir,
                    next.LeftBound, next.LeftAwayDir);
                canvas.DrawPath(_cornerPath, _cornerPaint);
            }
        }
    }

    /// <summary>
    /// The highest-ranked road type among all edges incident to a node (see
    /// <see cref="RoadTypeVisuals.GetRank"/>). Determines the intersection's fill appearance.
    /// </summary>
    private static RoadType HighestRoadTypeAtNode(RoadGraph graph, int nodeIndex)
    {
        RoadType best = RoadType.Dirt;
        int bestRank = -1;

        foreach (int e in graph.GetIncomingEdges(nodeIndex))
        {
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            int r = RoadTypeVisuals.GetRank(edge.RoadType);
            if (r > bestRank) { bestRank = r; best = edge.RoadType; }
        }
        foreach (int e in graph.GetOutgoingEdges(nodeIndex))
        {
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            int r = RoadTypeVisuals.GetRank(edge.RoadType);
            if (r > bestRank) { bestRank = r; best = edge.RoadType; }
        }
        return best;
    }

    /// <summary>
    /// Collects boundary endpoint info for each unique road meeting at a node into the
    /// reusable <see cref="_approaches"/> list (cleared first; result valid until the next
    /// call). Deduplicates forward/reverse pairs so each physical road appears once.
    /// </summary>
    private void CollectApproaches(RoadGraph graph, StopLineCache stopLines, int nodeIndex)
    {
        var approaches = _approaches;
        approaches.Clear();
        var seen = _seenPairs; // track processed road pairs by min edge index
        seen.Clear();

        // Incoming edges: ToNode == nodeIndex
        foreach (int edgeIdx in graph.GetIncomingEdges(nodeIndex))
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;

            int rev = graph.FindReverseEdge(edgeIdx);
            int pairKey = rev >= 0 ? Math.Min(edgeIdx, rev) : edgeIdx;
            if (!seen.Add(pairKey)) continue;

            float leftT = stopLines.GetLeftTrimAtToNode(edgeIdx);
            float rightT = stopLines.GetRightTrimAtToNode(edgeIdx);
            float stopT = stopLines.GetStopTAtToNode(edgeIdx);
            if (ComputeApproach(graph, edgeIdx, leftT, rightT, stopT, atToNode: true) is { } info)
                approaches.Add(info);
        }

        // Outgoing edges not already covered (one-way roads leaving the node)
        foreach (int edgeIdx in graph.GetOutgoingEdges(nodeIndex))
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;

            int rev = graph.FindReverseEdge(edgeIdx);
            int pairKey = rev >= 0 ? Math.Min(edgeIdx, rev) : edgeIdx;
            if (!seen.Add(pairKey)) continue;

            float leftT = stopLines.GetLeftTrimAtFromNode(edgeIdx);
            float rightT = stopLines.GetRightTrimAtFromNode(edgeIdx);
            float stopT = stopLines.GetStopTAtFromNode(edgeIdx);
            if (ComputeApproach(graph, edgeIdx, leftT, rightT, stopT, atToNode: false) is { } info)
                approaches.Add(info);
        }
    }

    /// <summary>
    /// Computes boundary endpoints and direction for a single edge at a node.
    /// Each boundary is evaluated at its own per-side trim t-value so that acute-angle
    /// intersections have asymmetric boundary positions. Left/right are defined looking
    /// away from the node, using the standard right-normal convention (-ty, tx) for Y-down.
    /// </summary>
    private static ApproachInfo? ComputeApproach(RoadGraph graph, int edgeIdx,
        float leftTrimT, float rightTrimT, float stopT, bool atToNode)
    {
        // Use the overall stopT for direction/angle (stable for sorting)
        var tangent = graph.EvaluateBezierTangent(edgeIdx, stopT);
        float len = tangent.Length();
        if (len < 0.001f) return null;

        // Boundary endpoints sit at the DRAWN asphalt edge (geometric half-width × visual
        // multiplier) so fills and corner curves meet the surface strokes and curb lines.
        float halfWidth = GeometryUtil.RoadHalfWidth(graph, edgeIdx)
            * RoadTypeVisuals.GetWidthMultiplier(graph.Edges[edgeIdx].RoadType);

        // Compute left boundary point at its own trim-t (negative offset in tangent frame)
        var posL = graph.EvaluateBezier(edgeIdx, leftTrimT);
        var tanL = graph.EvaluateBezierTangent(edgeIdx, leftTrimT);
        float lenL = tanL.Length();
        if (lenL < 0.001f) return null;
        float nxL = -tanL.Y / lenL, nyL = tanL.X / lenL;
        var leftOfTangent = new Vector2(posL.X - nxL * halfWidth, posL.Y - nyL * halfWidth);

        // Compute right boundary point at its own trim-t (positive offset in tangent frame)
        var posR = graph.EvaluateBezier(edgeIdx, rightTrimT);
        var tanR = graph.EvaluateBezierTangent(edgeIdx, rightTrimT);
        float lenR = tanR.Length();
        if (lenR < 0.001f) return null;
        float nxR = -tanR.Y / lenR, nyR = tanR.X / lenR;
        var rightOfTangent = new Vector2(posR.X + nxR * halfWidth, posR.Y + nyR * halfWidth);

        // Per-boundary away directions (tangent at each boundary's actual trim-t)
        Vector2 leftAway, rightAway;
        if (atToNode)
        {
            leftAway = new Vector2(-tanL.X / lenL, -tanL.Y / lenL);
            rightAway = new Vector2(-tanR.X / lenR, -tanR.Y / lenR);
        }
        else
        {
            leftAway = new Vector2(tanL.X / lenL, tanL.Y / lenL);
            rightAway = new Vector2(tanR.X / lenR, tanR.Y / lenR);
        }

        // Convert to consistent "looking away from node" frame
        Vector2 awayDir, leftBound, rightBound;
        Vector2 leftAwayDir, rightAwayDir;
        if (atToNode)
        {
            // Tangent points INTO node; away = negated; left/right swap
            awayDir = new Vector2(-tangent.X / len, -tangent.Y / len);
            rightBound = leftOfTangent;
            leftBound = rightOfTangent;
            // Swap per-boundary dirs too (left trim → right boundary and vice versa)
            leftAwayDir = rightAway;
            rightAwayDir = leftAway;
        }
        else
        {
            // Tangent points AWAY from node; no swap needed
            awayDir = new Vector2(tangent.X / len, tangent.Y / len);
            rightBound = rightOfTangent;
            leftBound = leftOfTangent;
            leftAwayDir = leftAway;
            rightAwayDir = rightAway;
        }

        return new ApproachInfo
        {
            Angle = MathF.Atan2(awayDir.Y, awayDir.X),
            LeftBound = leftBound,
            RightBound = rightBound,
            AwayDir = awayDir,
            LeftAwayDir = leftAwayDir,
            RightAwayDir = rightAwayDir
        };
    }

    /// <summary>
    /// Adds a smooth quadratic Bézier curve from P0 to P2, tangent to the boundary lines
    /// at both endpoints. The control point is the intersection of rays extending from each
    /// boundary endpoint toward the node interior, producing a natural curb-return shape.
    /// Falls back to a straight line if the rays are near-parallel.
    /// </summary>
    private static void AddCornerCurve(SKPath path, Vector2 p0, Vector2 awayDir0,
        Vector2 p2, Vector2 awayDir2)
    {
        // At P0: curve should be tangent to the boundary line, heading toward the node
        // At P2: curve should be tangent to the next boundary line, heading away from the node
        var d0 = new Vector2(-awayDir0.X, -awayDir0.Y); // toward node
        var d2 = new Vector2(-awayDir2.X, -awayDir2.Y); // toward node

        // Find intersection: P0 + s*d0 = P2 + t*d2
        float dx = p2.X - p0.X;
        float dy = p2.Y - p0.Y;
        float det = d0.X * d2.Y - d0.Y * d2.X;

        if (MathF.Abs(det) > 0.001f)
        {
            float s = (dx * d2.Y - dy * d2.X) / det;

            if (s > 0.01f)
            {
                // Clamp to prevent degenerate control points at very acute angles
                float maxDist = (p2 - p0).Length() * 2f;
                if (s > maxDist) s = maxDist;

                float p1x = p0.X + s * d0.X;
                float p1y = p0.Y + s * d0.Y;
                path.QuadTo(p1x, p1y, p2.X, p2.Y);
                return;
            }
        }

        // Parallel or behind: straight line
        path.LineTo(p2.X, p2.Y);
    }

    private static byte Dim(int baseValue, float ambient) =>
        (byte)Math.Clamp((int)(baseValue * ambient), 0, 255);

    /// <summary>
    /// Pre-computed SkiaSharp paths for rendering a single road edge: center path, trimmed
    /// center line (highway median), left/right boundaries, lane dividers, per-road-type
    /// marking paths, and total road width.
    /// </summary>
    private sealed class CachedEdgePaths : IDisposable
    {
        /// <summary>Path along the center of the road, full length (used for surface + shoulder rendering).</summary>
        public readonly SKPath CenterPath;
        /// <summary>Trimmed center path — non-empty only for two-way highways, stroked as the median band.</summary>
        public readonly SKPath CenterLinePath;
        /// <summary>Path along the left boundary of the road, trimmed at intersections.</summary>
        public readonly SKPath LeftEdgePath;
        /// <summary>Path along the right boundary of the road, trimmed at intersections.</summary>
        public readonly SKPath RightEdgePath;
        /// <summary>Paths for lane divider markings, trimmed at intersections (empty for single-lane roads).</summary>
        public readonly List<SKPath> LanePaths;
        /// <summary>
        /// Per-road-type marking paths, trimmed at intersections. Dirt: tire-track pairs per
        /// lane (both directions on two-way roads); arterial: double-yellow pair; highway:
        /// yellow lines flanking the median. Empty for other type/topology combinations.
        /// </summary>
        public readonly List<SKPath> TypeMarkingPaths;
        /// <summary>Total road width in meters (2 × <see cref="GeometryUtil.RoadHalfWidth"/>).</summary>
        public readonly float TotalWidth;
        /// <summary>True if the road has a center divider to mark (two-way roads only).</summary>
        public readonly bool HasCenterLine;
        /// <summary>True if the outer edge lines should be dashed (single-lane two-way roads).</summary>
        public readonly bool DashedEdges;

        public CachedEdgePaths(SKPath center, SKPath centerLine, SKPath left, SKPath right, List<SKPath> lanes,
            List<SKPath> typeMarkings, float totalWidth, bool hasCenterLine, bool dashedEdges)
        {
            CenterPath = center;
            CenterLinePath = centerLine;
            LeftEdgePath = left;
            RightEdgePath = right;
            LanePaths = lanes;
            TypeMarkingPaths = typeMarkings;
            TotalWidth = totalWidth;
            HasCenterLine = hasCenterLine;
            DashedEdges = dashedEdges;
        }

        public void Dispose()
        {
            CenterPath.Dispose();
            CenterLinePath.Dispose();
            LeftEdgePath.Dispose();
            RightEdgePath.Dispose();
            foreach (var p in LanePaths) p.Dispose();
            foreach (var p in TypeMarkingPaths) p.Dispose();
        }
    }
}
