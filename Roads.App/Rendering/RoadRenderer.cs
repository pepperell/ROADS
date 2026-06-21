using System.Numerics;
using SkiaSharp;
using Roads.App;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Renders the road network: asphalt surfaces, lane markings, edge boundaries, stop lines,
/// intersection nodes, and traffic control indicators (signals, stop signs, yield signs).
/// Caches per-edge Bezier offset paths and invalidates when the graph version changes.
/// </summary>
public class RoadRenderer
{
    /// <summary>Lane width in meters.</summary>
    private const float LaneWidth = SimConstants.LaneWidth;
    /// <summary>Number of line segments per Bézier curve for rendering.</summary>
    private const int BezierSegments = 20;

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

    // Center-line dash effect — immutable and shared across all edges/frames (created once
    // to honor the per-frame no-allocation discipline of the paints above).
    private readonly SKPathEffect _centerLineDash = SKPathEffect.CreateDash(new[] { 2f, 2f }, 0);

    // Reusable paints for DrawRoadLines (StrokeWidth updated per frame based on zoom)
    private readonly SKPaint _edgeLinePaint = new()
    {
        Color = new SKColor(200, 200, 200),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
    private readonly SKPaint _centerLinePaint = new()
    {
        Color = new SKColor(220, 180, 40),
        Style = SKPaintStyle.Stroke,
        IsAntialias = true
    };
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
    /// Draws all active road edges in two passes (surfaces first, then markings) so that
    /// connecting roads don't overlap each other's lines. Also draws stop lines and nodes.
    /// For paired edges (forward/reverse), only the lower-index edge is drawn to avoid overdraw.
    /// When <paramref name="heatMap"/> is non-null and <see cref="CongestionHeatMap.Enabled"/>
    /// is true, a congestion tint is alpha-blended over each road surface after the base
    /// color is applied, without affecting lane markings or the road type styling.
    ///
    /// Frustum culling: edges whose endpoint AABB does not intersect <paramref name="viewRect"/>
    /// are skipped in both passes. A generous margin is added to the AABB so that Bézier
    /// curves bowing outside their endpoint bounding box are never incorrectly culled.
    ///
    /// Level-of-Detail: when <paramref name="zoom"/> is below
    /// <see cref="RenderDetail.RoadSimpleThreshold"/>, Pass 1 draws plain center-line
    /// strokes only (no surface fill or markings) and Pass 2 / intersections / stop lines
    /// / nodes are all skipped. The simplified view is always legible at city-overview scale.
    /// </summary>
    /// <param name="canvas">SkiaSharp canvas in world-space coordinates.</param>
    /// <param name="graph">Road graph to render.</param>
    /// <param name="stopLines">Stop-line cache for marking trim t-values.</param>
    /// <param name="zoom">Current camera zoom (world units per screen pixel).</param>
    /// <param name="darkness">Ambient darkness factor (0 = full day, 1 = full night).</param>
    /// <param name="heatMap">Optional congestion heat-map overlay; null disables it.</param>
    /// <param name="viewRect">Visible world-space rectangle for frustum culling.</param>
    public void Draw(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom,
        float darkness = 0f, CongestionHeatMap? heatMap = null,
        SKRect viewRect = default, IReadOnlyList<int>? visibleEdges = null,
        StopSignSystem? stopSigns = null)
    {
        if (graph.Edges.Count == 0) return;

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
        _surfacePaint.Color = new SKColor(Dim(70, _ambient), Dim(72, _ambient), Dim(78, _ambient));
        _intersectionFillPaint.Color = _surfacePaint.Color;
        _nodePaint.Color = new SKColor(Dim(120, _ambient), Dim(130, _ambient), Dim(150, _ambient));
        _edgeLinePaint.Color = new SKColor(Dim(200, _ambient), Dim(200, _ambient), Dim(200, _ambient));
        _centerLinePaint.Color = new SKColor(Dim(220, _ambient), Dim(180, _ambient), Dim(40, _ambient));
        _laneDividerPaint.Color = new SKColor(Dim(180, _ambient), Dim(180, _ambient), Dim(180, _ambient), (byte)(160 * _ambient));

        RebuildCacheIfNeeded(graph, stopLines);

        // viewRect is only used for culling; a zero/empty rect means no culling (backward-compat
        // with callers that do not supply it, e.g. signal/sign helper methods).
        bool cull = viewRect.Width > 0f || viewRect.Height > 0f;

        // LOD simple: draw roads as plain center-line strokes at city-overview zoom.
        // This replaces both surface pass and marking pass with a single cheap draw.
        if (lodSimple)
        {
            _surfacePaint.StrokeWidth = 1.5f / zoom; // thin but visible at all scales
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
                    if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to, halfW), viewRect))
                        continue;
                }

                _surfacePaint.Color = RoadTypeVisuals.GetSurfaceColor(edge.RoadType, _ambient);
                canvas.DrawPath(_cache[i].CenterPath, _surfacePaint);
            }
            return; // skip all detail passes
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
                if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to, halfW), viewRect))
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
                if (!RenderDetail.IsVisible(RenderDetail.EdgeBounds(from, to, halfW), viewRect))
                    continue;
            }

            DrawRoadLines(canvas, edge, i, zoom);

            // One-way roads (no reverse edge) get direction chevrons down each lane.
            if (reverse < 0 && RoadTypeVisuals.HasPaintedLines(edge.RoadType))
                DrawOneWayArrows(canvas, graph, i, zoom);
        }

        DrawStopLines(canvas, graph, stopLines, zoom, viewRect, stopSigns);
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
    /// left/right boundary paths and lane dividers trimmed to stop line t-values so markings
    /// don't extend into intersections.
    /// </summary>
    private CachedEdgePaths BuildEdgePaths(RoadGraph graph, int edgeIndex, StopLineCache stopLines)
    {
        var edge = graph.Edges[edgeIndex];
        float totalWidth = GeometryUtil.RoadSurfaceWidth(graph, edgeIndex);
        float halfWidth = totalWidth * 0.5f;
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

        // Yellow center line exists only on a two-way road, where the edge path is the
        // center divider. One-way and single-lane two-way roads have no divider — keep an
        // empty path so the cache shape is uniform.
        var centerLinePath = hasCenterLine
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

        return new CachedEdgePaths(centerPath, centerLinePath, leftPath, rightPath, lanePaths,
            totalWidth, hasCenterLine, dashedEdges);
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
    /// Draws boundary lines, center line, and lane dividers for a single road edge.
    /// All marking paths are trimmed at intersection boundaries. Unpaved road types
    /// (those where <see cref="RoadTypeVisuals.HasPaintedLines"/> is false, i.e. dirt)
    /// carry no paint and are skipped entirely.
    /// </summary>
    private void DrawRoadLines(SKCanvas canvas, RoadEdge edge, int edgeIndex, float zoom)
    {
        var cached = _cache[edgeIndex];

        // Unpaved roads (dirt) have no painted edge lines, center line, or lane dividers.
        if (!RoadTypeVisuals.HasPaintedLines(edge.RoadType)) return;

        // Edge boundary lines (white) — update zoom-dependent stroke width. Single-lane
        // two-way roads dash both edges to signal a shared single track.
        _edgeLinePaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);
        _edgeLinePaint.PathEffect = cached.DashedEdges ? _centerLineDash : null;
        canvas.DrawPath(cached.LeftEdgePath, _edgeLinePaint);
        canvas.DrawPath(cached.RightEdgePath, _edgeLinePaint);
        _edgeLinePaint.PathEffect = null;

        // Center line (yellow dashed) — only on two-way roads, where the edge path is the
        // center divider. Shared effect, no per-frame allocation.
        if (cached.HasCenterLine)
        {
            _centerLinePaint.StrokeWidth = Math.Max(0.3f, 0.4f / zoom);
            _centerLinePaint.PathEffect = _centerLineDash;
            canvas.DrawPath(cached.CenterLinePath, _centerLinePaint);
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

    /// <summary>
    /// Draws white stop lines at both ends of each edge where StopLineCache indicates a line
    /// is needed (i.e., where the stop-T is inset from the edge endpoint).
    /// </summary>
    /// <summary>True if (x,y) is within the cull rect (expanded by <paramref name="margin"/>), or
    /// culling is off (an empty rect means "draw everything"). Used to skip off-screen node/edge work.</summary>
    private static bool InView(float x, float y, SKRect cull, float margin)
        => (cull.Width <= 0f && cull.Height <= 0f)
           || (x >= cull.Left - margin && x <= cull.Right + margin
               && y >= cull.Top - margin && y <= cull.Bottom + margin);

    private void DrawStopLines(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom,
        SKRect cullRect = default, StopSignSystem? stopSigns = null)
    {
        using var stopLinePaint = new SKPaint
        {
            Color = new SKColor(255, 255, 255),
            StrokeWidth = Math.Max(0.4f, 0.6f / zoom),
            Style = SKPaintStyle.Stroke,
            StrokeCap = SKStrokeCap.Butt,
            IsAntialias = true
        };

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
                    DrawStopLine(canvas, graph, i, edge, stopT, stopLinePaint);
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

    /// <summary>Draws small circles at each active intersection/endpoint node.</summary>
    private void DrawNodes(SKCanvas canvas, RoadGraph graph, float zoom, SKRect cullRect = default)
    {
        float nodeRadius = Math.Max(2f, 3f / zoom);

        for (int i = 0; i < graph.Nodes.Count; i++)
        {
            var node = graph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue; // skip defunct nodes
            if (!InView(node.Position.X, node.Position.Y, cullRect, nodeRadius + 10f)) continue;
            canvas.DrawCircle(node.Position.X, node.Position.Y, nodeRadius, _nodePaint);
        }
    }

    /// <summary>
    /// Iterates incoming edges at nodes matching a predicate and computes the sign position
    /// (stop line offset to the right). Calls the draw action for each valid position.
    /// Shared by DrawSignals, DrawStopSigns, and DrawYieldSigns.
    /// </summary>
    private static void ForEachSignPosition(
        RoadGraph graph, StopLineCache stopLines,
        Func<int, bool> isNodeActive,
        Func<int, bool>? isEdgeIncluded,
        Action<SKCanvas, int, float, float> drawSign,
        SKCanvas canvas, SKRect cullRect = default)
    {
        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            if (!isNodeActive(n)) continue;
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            // Skip the per-edge Bezier evaluations below for off-screen intersections.
            if (!InView(node.Position.X, node.Position.Y, cullRect, 40f)) continue;

            var incoming = graph.GetIncomingEdges(n);
            foreach (int edgeIdx in incoming)
            {
                var edge = graph.Edges[edgeIdx];
                if (edge.FromNode < 0) continue;
                if (isEdgeIncluded != null && !isEdgeIncluded(edgeIdx)) continue;

                float stopT = stopLines.GetStopTAtToNode(edgeIdx);
                var pos = graph.EvaluateBezier(edgeIdx, stopT);
                var tangent = graph.EvaluateBezierTangent(edgeIdx, stopT);
                float len = tangent.Length();
                if (len < 0.001f) continue;

                float rx = -tangent.Y / len;
                float ry = tangent.X / len;
                float offset = GeometryUtil.RoadHalfWidth(graph, edgeIdx) + 2f;
                float sx = pos.X + rx * offset;
                float sy = pos.Y + ry * offset;

                drawSign(canvas, edgeIdx, sx, sy);
            }
        }
    }

    /// <summary>
    /// Draws colored circles (green/yellow/red) for traffic light signals at each incoming
    /// edge's stop line, offset to the right of the travel direction.
    /// </summary>
    public void DrawSignals(SKCanvas canvas, RoadGraph graph, TrafficSignalSystem signals, StopLineCache stopLines, float zoom, SKRect cullRect = default)
    {
        if (zoom < 0.3f) return; // too far out to read — skip (declutter + draw cost at scale); matches speed-limit signs
        float radius = Math.Max(1.5f, 2.5f / zoom);

        using var greenPaint = new SKPaint { Color = new SKColor(0, 200, 0), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var yellowPaint = new SKPaint { Color = new SKColor(255, 200, 0), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var redPaint = new SKPaint { Color = new SKColor(220, 40, 40), Style = SKPaintStyle.Fill, IsAntialias = true };

        ForEachSignPosition(graph, stopLines,
            n => signals.IsTrafficLight(n),
            null,
            (c, edgeIdx, sx, sy) =>
            {
                var state = signals.GetSignal(edgeIdx);
                var paint = state switch
                {
                    SignalState.Green => greenPaint,
                    SignalState.Yellow => yellowPaint,
                    _ => redPaint,
                };
                c.DrawCircle(sx, sy, radius, paint);
            },
            canvas, cullRect);
    }

    /// <summary>
    /// Draws red squares for stop signs at each incoming edge's stop line, offset to the
    /// right of the travel direction.
    /// </summary>
    public void DrawStopSigns(SKCanvas canvas, RoadGraph graph, StopSignSystem stopSigns, StopLineCache stopLines, float zoom, SKRect cullRect = default)
    {
        if (zoom < 0.3f) return; // too far out to read — skip (declutter + draw cost at scale); matches speed-limit signs
        float size = Math.Max(1.5f, 2.5f / zoom);

        using var stopPaint = new SKPaint { Color = new SKColor(200, 30, 30), Style = SKPaintStyle.Fill, IsAntialias = true };

        ForEachSignPosition(graph, stopLines,
            n => stopSigns.IsStopSign(n),
            edgeIdx => !stopSigns.IsEdgeExempt(edgeIdx),
            (c, _, sx, sy) => c.DrawRect(sx - size, sy - size, size * 2, size * 2, stopPaint),
            canvas, cullRect);
    }

    /// <summary>
    /// Draws orange inverted triangles for yield signs at each incoming edge's stop line,
    /// offset to the right of the travel direction.
    /// </summary>
    public void DrawYieldSigns(SKCanvas canvas, RoadGraph graph, YieldSignSystem yieldSigns, StopLineCache stopLines, float zoom, SKRect cullRect = default)
    {
        if (zoom < 0.3f) return; // too far out to read — skip (declutter + draw cost at scale); matches speed-limit signs
        float size = Math.Max(1.5f, 2.5f / zoom);

        using var yieldPaint = new SKPaint { Color = new SKColor(230, 160, 0), Style = SKPaintStyle.Fill, IsAntialias = true };

        ForEachSignPosition(graph, stopLines,
            n => yieldSigns.IsYield(n),
            edgeIdx => !yieldSigns.IsEdgeExempt(edgeIdx),
            (c, _, sx, sy) =>
            {
                using var path = new SKPath();
                path.MoveTo(sx - size, sy - size);
                path.LineTo(sx + size, sy - size);
                path.LineTo(sx, sy + size);
                path.Close();
                c.DrawPath(path, yieldPaint);
            },
            canvas, cullRect);
    }

    /// <summary>
    /// Draws speed limit signs near the start of each road edge, offset to the right of
    /// the travel direction. Shows a white circle with red border and speed in mph.
    /// Only drawn at medium/close zoom to avoid visual clutter.
    /// </summary>
    public void DrawSpeedLimitSigns(SKCanvas canvas, RoadGraph graph, float zoom, SKRect cullRect = default)
    {
        if (zoom < 0.3f) return; // too far out — skip to avoid clutter

        float radius = Math.Max(2.5f, 4f / zoom);
        float fontSize = Math.Max(2f, 3f / zoom);

        using var bgPaint = new SKPaint
        {
            Color = SKColors.White,
            Style = SKPaintStyle.Fill,
            IsAntialias = true
        };
        using var borderPaint = new SKPaint
        {
            Color = new SKColor(200, 30, 30),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(0.3f, 0.5f / zoom),
            IsAntialias = true
        };
        using var textPaint = new SKPaint
        {
            Color = SKColors.Black,
            IsAntialias = true
        };
        using var font = new SKFont
        {
            Size = fontSize
        };

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            // For paired edges, only draw on the lower-index edge
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            var fromPos = graph.Nodes[edge.FromNode].Position;
            if (!InView(fromPos.X, fromPos.Y, cullRect, 60f)) continue;

            // Position at 20% of the edge length (past intersection, near start)
            float signT = 0.2f;
            var pos = graph.EvaluateBezier(i, signT);
            var tangent = graph.EvaluateBezierTangent(i, signT);
            float len = tangent.Length();
            if (len < 0.001f) continue;

            float rx = -tangent.Y / len;
            float ry = tangent.X / len;
            float offset = GeometryUtil.RoadHalfWidth(graph, i) + 3f;
            float sx = pos.X + rx * offset;
            float sy = pos.Y + ry * offset;

            // Draw white circle with red border
            canvas.DrawCircle(sx, sy, radius, bgPaint);
            canvas.DrawCircle(sx, sy, radius, borderPaint);

            // Draw speed text (mph, rounded to nearest 5)
            float mph = edge.SpeedLimit * 2.23694f;
            int roundedMph = ((int)MathF.Round(mph / 5f)) * 5;
            string text = roundedMph.ToString();

            using var textBlob = SKTextBlob.Create(text, font);
            if (textBlob != null)
            {
                float tw = font.MeasureText(text);
                canvas.DrawText(text, sx - tw * 0.5f, sy + fontSize * 0.35f, font, textPaint);
            }
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
    /// adjacent roads at each intersection node.
    /// </summary>
    private void DrawIntersectionFills(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom, SKRect cullRect = default)
    {
        float curveLineWidth = Math.Max(0.3f, 0.5f / zoom);

        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            // Cull before the expensive CollectApproaches (allocates + sorts) for off-screen nodes.
            if (!InView(node.Position.X, node.Position.Y, cullRect, 60f)) continue;

            var approaches = CollectApproaches(graph, stopLines, n);
            if (approaches.Count < 2) continue;

            approaches.Sort((a, b) => a.Angle.CompareTo(b.Angle));

            int count = approaches.Count;

            // The intersection takes on the appearance of its highest-ranked incident road
            // (e.g. a highway+arterial node looks like highway; an all-dirt node looks like dirt).
            RoadType nodeType = HighestRoadTypeAtNode(graph, n);
            _intersectionFillPaint.Color = RoadTypeVisuals.GetSurfaceColor(nodeType, _ambient);

            // Build intersection fill polygon: for each road, a straight segment along the
            // stop line (left→right boundary), then a curve to the next road's left boundary.
            using var fillPath = new SKPath();
            fillPath.MoveTo(approaches[0].LeftBound.X, approaches[0].LeftBound.Y);

            for (int i = 0; i < count; i++)
            {
                var curr = approaches[i];
                var next = approaches[(i + 1) % count];

                fillPath.LineTo(curr.RightBound.X, curr.RightBound.Y);
                AddCornerCurve(fillPath, curr.RightBound, curr.RightAwayDir,
                    next.LeftBound, next.LeftAwayDir);
            }

            fillPath.Close();
            canvas.DrawPath(fillPath, _intersectionFillPaint);

            // White corner boundary lines — unpaved (dirt) intersections carry no paint, matching
            // the per-edge rule, so skip them when the intersection's appearance is dirt.
            if (!RoadTypeVisuals.HasPaintedLines(nodeType)) continue;

            using var curvePaint = new SKPaint
            {
                Color = _edgeLinePaint.Color,
                StrokeWidth = curveLineWidth,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Butt,
                IsAntialias = true
            };

            for (int i = 0; i < count; i++)
            {
                var curr = approaches[i];
                var next = approaches[(i + 1) % count];

                using var curvePath = new SKPath();
                curvePath.MoveTo(curr.RightBound.X, curr.RightBound.Y);
                AddCornerCurve(curvePath, curr.RightBound, curr.RightAwayDir,
                    next.LeftBound, next.LeftAwayDir);
                canvas.DrawPath(curvePath, curvePaint);
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
    /// Collects boundary endpoint info for each unique road meeting at a node.
    /// Deduplicates forward/reverse pairs so each physical road appears once.
    /// </summary>
    private static List<ApproachInfo> CollectApproaches(RoadGraph graph, StopLineCache stopLines, int nodeIndex)
    {
        var approaches = new List<ApproachInfo>();
        var seen = new HashSet<int>(); // track processed road pairs by min edge index

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

        return approaches;
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

        float halfWidth = GeometryUtil.RoadHalfWidth(graph, edgeIdx);

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
    /// Pre-computed SkiaSharp paths for rendering a single road edge: center line, left/right
    /// boundaries, lane dividers, and total road width.
    /// </summary>
    private sealed class CachedEdgePaths : IDisposable
    {
        /// <summary>Path along the center of the road, full length (used for surface rendering).</summary>
        public readonly SKPath CenterPath;
        /// <summary>Center line path trimmed at intersection boundaries (used for yellow dashed marking).</summary>
        public readonly SKPath CenterLinePath;
        /// <summary>Path along the left boundary of the road, trimmed at intersections.</summary>
        public readonly SKPath LeftEdgePath;
        /// <summary>Path along the right boundary of the road, trimmed at intersections.</summary>
        public readonly SKPath RightEdgePath;
        /// <summary>Paths for lane divider markings, trimmed at intersections (empty for single-lane roads).</summary>
        public readonly List<SKPath> LanePaths;
        /// <summary>Total road width in meters (2 × <see cref="GeometryUtil.RoadHalfWidth"/>).</summary>
        public readonly float TotalWidth;
        /// <summary>True if a yellow center line should be drawn (two-way roads only).</summary>
        public readonly bool HasCenterLine;
        /// <summary>True if the outer edge lines should be dashed (single-lane two-way roads).</summary>
        public readonly bool DashedEdges;

        public CachedEdgePaths(SKPath center, SKPath centerLine, SKPath left, SKPath right, List<SKPath> lanes,
            float totalWidth, bool hasCenterLine, bool dashedEdges)
        {
            CenterPath = center;
            CenterLinePath = centerLine;
            LeftEdgePath = left;
            RightEdgePath = right;
            LanePaths = lanes;
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
        }
    }
}
