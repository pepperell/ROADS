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

    /// <summary>
    /// Draws all active road edges in two passes (surfaces first, then markings) so that
    /// connecting roads don't overlap each other's lines. Also draws stop lines and nodes.
    /// For paired edges (forward/reverse), only the lower-index edge is drawn to avoid overdraw.
    /// </summary>
    public void Draw(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom, float darkness = 0f)
    {
        if (graph.Edges.Count == 0) return;

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

        // Pass 1: draw all asphalt surfaces first so overlapping roads blend seamlessly
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            if (i >= _cache.Count) break;
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;
            DrawRoadSurface(canvas, edge, i);
        }

        // Pass 1.5: fill intersection interiors and draw corner curves
        DrawIntersectionFills(canvas, graph, stopLines, zoom);

        // Pass 2: draw all lane markings and boundary lines on top
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            if (i >= _cache.Count) break;
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;
            DrawRoadLines(canvas, edge, i, zoom);
        }

        DrawStopLines(canvas, graph, stopLines, zoom);
        DrawNodes(canvas, graph, zoom);
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
        float totalWidth = edge.LaneCount * 2 * LaneWidth;

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

        // Trimmed center line for markings (separate from surface center path)
        var centerLinePath = BuildOffsetPath(graph, edgeIndex, 0f, tMin, tMax);

        // Build edge boundary paths with per-side trims
        var leftPath = BuildOffsetPath(graph, edgeIndex, -totalWidth / 2f, tMinLeft, tMaxLeft);
        var rightPath = BuildOffsetPath(graph, edgeIndex, totalWidth / 2f, tMinRight, tMaxRight);

        // Build lane divider paths (trimmed to overall stop-T, multi-lane only)
        var lanePaths = new List<SKPath>();
        if (edge.LaneCount > 1)
        {
            for (int lane = 1; lane < edge.LaneCount; lane++)
            {
                float offset = lane * LaneWidth;
                lanePaths.Add(BuildOffsetPath(graph, edgeIndex, offset, tMin, tMax));
                lanePaths.Add(BuildOffsetPath(graph, edgeIndex, -offset, tMin, tMax));
            }
        }

        return new CachedEdgePaths(centerPath, centerLinePath, leftPath, rightPath, lanePaths, totalWidth);
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
    /// </summary>
    private void DrawRoadSurface(SKCanvas canvas, RoadEdge edge, int edgeIndex)
    {
        var cached = _cache[edgeIndex];
        _surfacePaint.Color = RoadTypeVisuals.GetSurfaceColor(edge.RoadType, _ambient);
        _surfacePaint.StrokeWidth = cached.TotalWidth * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType);
        canvas.DrawPath(cached.CenterPath, _surfacePaint);
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

        // Edge boundary lines (white) — update zoom-dependent stroke width
        _edgeLinePaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);
        canvas.DrawPath(cached.LeftEdgePath, _edgeLinePaint);
        canvas.DrawPath(cached.RightEdgePath, _edgeLinePaint);

        // Center line (yellow dashed) — shared effect, no per-frame allocation.
        _centerLinePaint.StrokeWidth = Math.Max(0.3f, 0.4f / zoom);
        _centerLinePaint.PathEffect = _centerLineDash;
        canvas.DrawPath(cached.CenterLinePath, _centerLinePaint);

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
    /// <summary>Mask of node flags that indicate a controlled intersection where stop lines should be drawn.</summary>
    private const NodeFlags ControlledMask = NodeFlags.TrafficLight | NodeFlags.StopSign | NodeFlags.Yield;

    private void DrawStopLines(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom)
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

            // Draw stop line at ToNode end only (the approach side of the intersection).
            // Outgoing edges (FromNode end) don't need stop lines.
            if ((graph.Nodes[edge.ToNode].Flags & ControlledMask) != 0)
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

        float halfWidth = edge.LaneCount * LaneWidth;

        // Draw across right half only (our travel lanes)
        canvas.DrawLine(center.X, center.Y,
            center.X + nx * halfWidth, center.Y + ny * halfWidth, paint);
    }

    /// <summary>Draws small circles at each active intersection/endpoint node.</summary>
    private void DrawNodes(SKCanvas canvas, RoadGraph graph, float zoom)
    {
        float nodeRadius = Math.Max(2f, 3f / zoom);

        for (int i = 0; i < graph.Nodes.Count; i++)
        {
            var node = graph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue; // skip defunct nodes
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
        SKCanvas canvas)
    {
        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            if (!isNodeActive(n)) continue;
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;

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
                float offset = edge.LaneCount * LaneWidth + 2f;
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
    public void DrawSignals(SKCanvas canvas, RoadGraph graph, TrafficSignalSystem signals, StopLineCache stopLines, float zoom)
    {
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
            canvas);
    }

    /// <summary>
    /// Draws red squares for stop signs at each incoming edge's stop line, offset to the
    /// right of the travel direction.
    /// </summary>
    public void DrawStopSigns(SKCanvas canvas, RoadGraph graph, StopSignSystem stopSigns, StopLineCache stopLines, float zoom)
    {
        float size = Math.Max(1.5f, 2.5f / zoom);

        using var stopPaint = new SKPaint { Color = new SKColor(200, 30, 30), Style = SKPaintStyle.Fill, IsAntialias = true };

        ForEachSignPosition(graph, stopLines,
            n => stopSigns.IsStopSign(n),
            edgeIdx => !stopSigns.IsEdgeExempt(edgeIdx),
            (c, _, sx, sy) => c.DrawRect(sx - size, sy - size, size * 2, size * 2, stopPaint),
            canvas);
    }

    /// <summary>
    /// Draws orange inverted triangles for yield signs at each incoming edge's stop line,
    /// offset to the right of the travel direction.
    /// </summary>
    public void DrawYieldSigns(SKCanvas canvas, RoadGraph graph, YieldSignSystem yieldSigns, StopLineCache stopLines, float zoom)
    {
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
            canvas);
    }

    /// <summary>
    /// Draws speed limit signs near the start of each road edge, offset to the right of
    /// the travel direction. Shows a white circle with red border and speed in mph.
    /// Only drawn at medium/close zoom to avoid visual clutter.
    /// </summary>
    public void DrawSpeedLimitSigns(SKCanvas canvas, RoadGraph graph, float zoom)
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

            // Position at 20% of the edge length (past intersection, near start)
            float signT = 0.2f;
            var pos = graph.EvaluateBezier(i, signT);
            var tangent = graph.EvaluateBezierTangent(i, signT);
            float len = tangent.Length();
            if (len < 0.001f) continue;

            float rx = -tangent.Y / len;
            float ry = tangent.X / len;
            float offset = edge.LaneCount * LaneWidth + 3f;
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
    private void DrawIntersectionFills(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom)
    {
        float curveLineWidth = Math.Max(0.3f, 0.5f / zoom);

        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;

            var approaches = CollectApproaches(graph, stopLines, n);
            if (approaches.Count < 2) continue;

            approaches.Sort((a, b) => a.Angle.CompareTo(b.Angle));

            int count = approaches.Count;

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

            // Draw corner curves as white boundary lines
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

        var edge = graph.Edges[edgeIdx];
        float halfWidth = edge.LaneCount * LaneWidth;

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
        /// <summary>Total road width in meters (LaneCount * 2 * LaneWidth for two-way).</summary>
        public readonly float TotalWidth;

        public CachedEdgePaths(SKPath center, SKPath centerLine, SKPath left, SKPath right, List<SKPath> lanes, float totalWidth)
        {
            CenterPath = center;
            CenterLinePath = centerLine;
            LeftEdgePath = left;
            RightEdgePath = right;
            LanePaths = lanes;
            TotalWidth = totalWidth;
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
