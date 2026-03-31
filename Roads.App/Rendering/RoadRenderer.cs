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

    // Reusable paints (avoid per-frame allocation)
    private readonly SKPaint _surfacePaint = new()
    {
        Color = new SKColor(70, 72, 78),
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true
    };

    private readonly SKPaint _nodePaint = new()
    {
        Color = new SKColor(120, 130, 150),
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };

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
        PathEffect = SKPathEffect.CreateDash(new[] { 2f, 2f }, 0),
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
    public void Draw(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines, float zoom)
    {
        if (graph.Edges.Count == 0) return;

        RebuildCacheIfNeeded(graph);

        // Pass 1: draw all asphalt surfaces first so overlapping roads blend seamlessly
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            if (i >= _cache.Count) break;
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;
            DrawRoadSurface(canvas, i);
        }

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
    private void RebuildCacheIfNeeded(RoadGraph graph)
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
            _cache.Add(BuildEdgePaths(graph, i));
        }
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Builds the cached Bezier paths for a single edge: center path, left/right boundary paths,
    /// and lane divider paths offset from center by multiples of LaneWidth.
    /// </summary>
    private CachedEdgePaths BuildEdgePaths(RoadGraph graph, int edgeIndex)
    {
        var edge = graph.Edges[edgeIndex];
        float totalWidth = edge.LaneCount * 2 * LaneWidth;

        // Build center path
        var centerPath = BuildBezierPath(graph, edgeIndex);

        // Build edge boundary paths
        var leftPath = BuildOffsetPath(graph, edgeIndex, -totalWidth / 2f);
        var rightPath = BuildOffsetPath(graph, edgeIndex, totalWidth / 2f);

        // Build lane divider paths (if multi-lane)
        var lanePaths = new List<SKPath>();
        if (edge.LaneCount > 1)
        {
            for (int lane = 1; lane < edge.LaneCount; lane++)
            {
                float offset = lane * LaneWidth;
                lanePaths.Add(BuildOffsetPath(graph, edgeIndex, offset));
                lanePaths.Add(BuildOffsetPath(graph, edgeIndex, -offset));
            }
        }

        return new CachedEdgePaths(centerPath, leftPath, rightPath, lanePaths, totalWidth);
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
    /// Builds an SKPath offset perpendicular to the Bezier curve by the given distance.
    /// Positive offset shifts right of travel direction; negative shifts left.
    /// </summary>
    private static SKPath BuildOffsetPath(RoadGraph graph, int edgeIndex, float offset)
    {
        var path = new SKPath();
        bool first = true;

        for (int s = 0; s <= BezierSegments; s++)
        {
            float t = s / (float)BezierSegments;
            var pos = graph.EvaluateBezier(edgeIndex, t);
            var tangent = graph.EvaluateBezierTangent(edgeIndex, t);

            float len = tangent.Length();
            if (len < 0.001f) continue;
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

        return path;
    }

    /// <summary>Draws the gray asphalt surface for a single road edge.</summary>
    private void DrawRoadSurface(SKCanvas canvas, int edgeIndex)
    {
        var cached = _cache[edgeIndex];
        _surfacePaint.StrokeWidth = cached.TotalWidth;
        canvas.DrawPath(cached.CenterPath, _surfacePaint);
    }

    /// <summary>
    /// Draws boundary lines, center line, and lane dividers for a single road edge.
    /// </summary>
    private void DrawRoadLines(SKCanvas canvas, RoadEdge edge, int edgeIndex, float zoom)
    {
        var cached = _cache[edgeIndex];

        // Edge boundary lines (white) — update zoom-dependent stroke width
        _edgeLinePaint.StrokeWidth = Math.Max(0.3f, 0.5f / zoom);
        canvas.DrawPath(cached.LeftEdgePath, _edgeLinePaint);
        canvas.DrawPath(cached.RightEdgePath, _edgeLinePaint);

        // Center line (yellow dashed)
        _centerLinePaint.StrokeWidth = Math.Max(0.3f, 0.4f / zoom);
        canvas.DrawPath(cached.CenterPath, _centerLinePaint);

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

            // Draw stop line at ToNode end
            float stopT = stopLines.GetStopTAtToNode(i);
            if (stopT < 0.999f)
                DrawStopLine(canvas, graph, i, edge, stopT, stopLinePaint, graph.FindReverseEdge(i) >= 0);

            // Draw stop line at FromNode end
            float startT = stopLines.GetStopTAtFromNode(i);
            if (startT > 0.001f)
                DrawStopLine(canvas, graph, i, edge, startT, stopLinePaint, graph.FindReverseEdge(i) >= 0);
        }
    }

    /// <summary>
    /// Draws a single stop line perpendicular to the edge at parameter t. On two-way roads
    /// (hasReverse true), only the right half is drawn; on one-way roads, the full width.
    /// </summary>
    private void DrawStopLine(SKCanvas canvas, RoadGraph graph, int edgeIndex, RoadEdge edge, float t, SKPaint paint, bool hasReverse)
    {
        var center = graph.EvaluateBezier(edgeIndex, t);
        var tangent = graph.EvaluateBezierTangent(edgeIndex, t);
        float len = tangent.Length();
        if (len < 0.001f) return;

        // Normal perpendicular to tangent (right side in Y-down coords)
        float nx = -tangent.Y / len;
        float ny = tangent.X / len;

        float halfWidth = edge.LaneCount * LaneWidth;

        if (hasReverse)
        {
            // Two-way road: draw line across right half only (our travel direction)
            canvas.DrawLine(center.X, center.Y,
                center.X + nx * halfWidth, center.Y + ny * halfWidth, paint);
        }
        else
        {
            // One-way: draw across full width
            canvas.DrawLine(center.X - nx * halfWidth, center.Y - ny * halfWidth,
                center.X + nx * halfWidth, center.Y + ny * halfWidth, paint);
        }
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

    /// <summary>
    /// Pre-computed SkiaSharp paths for rendering a single road edge: center line, left/right
    /// boundaries, lane dividers, and total road width.
    /// </summary>
    private sealed class CachedEdgePaths : IDisposable
    {
        /// <summary>Path along the center of the road (Bézier center line).</summary>
        public readonly SKPath CenterPath;
        /// <summary>Path along the left boundary of the road.</summary>
        public readonly SKPath LeftEdgePath;
        /// <summary>Path along the right boundary of the road.</summary>
        public readonly SKPath RightEdgePath;
        /// <summary>Paths for lane divider markings (empty for single-lane roads).</summary>
        public readonly List<SKPath> LanePaths;
        /// <summary>Total road width in meters (LaneCount * 2 * LaneWidth for two-way).</summary>
        public readonly float TotalWidth;

        public CachedEdgePaths(SKPath center, SKPath left, SKPath right, List<SKPath> lanes, float totalWidth)
        {
            CenterPath = center;
            LeftEdgePath = left;
            RightEdgePath = right;
            LanePaths = lanes;
            TotalWidth = totalWidth;
        }

        public void Dispose()
        {
            CenterPath.Dispose();
            LeftEdgePath.Dispose();
            RightEdgePath.Dispose();
            foreach (var p in LanePaths) p.Dispose();
        }
    }
}
