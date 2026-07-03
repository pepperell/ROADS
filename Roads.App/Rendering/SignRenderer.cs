using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Renders realistic top-down traffic-control furniture: per-approach traffic-signal heads
/// (dark housing, three lenses, emissive active lens with soft glow), stop-sign octagons,
/// yield triangles, and speed-limit boards. Speed-limit signs are drawn only where the limit
/// actually changes: at the start of an edge whose rounded-to-5 mph limit differs from its
/// upstream continuation (the incoming edge at the FromNode whose end tangent best aligns
/// with this edge's start tangent, within 30 degrees), or — when no continuation exists —
/// differs from the road type's default limit. Speed-sign placements are cached and rebuilt
/// lazily when <see cref="RoadGraph.Version"/> changes.
///
/// Call-order dependencies: <see cref="StopLineCache.RebuildIfNeeded"/> and
/// <see cref="TrafficSignalSystem.RebuildIfNeeded"/> must have run for the current graph
/// version before <see cref="Draw"/> each frame (signal queries assert at least one rebuild).
/// Rendering is strictly read-only over sim state.
///
/// Level of detail: below zoom 0.3 nothing is drawn; drop shadows and STOP / SPEED LIMIT
/// lettering appear at zoom &gt;= 0.8; YIELD lettering appears at zoom &gt;= 1.5. All furniture
/// uses the screen-constant sizing idiom (world scale = max(1, 2 / zoom)) so it stays legible
/// at mid zoom and reaches true size when zoomed in. Passive surfaces are dimmed by the
/// day/night ambient factor (1 - darkness * 0.45); active signal lenses and their glow are
/// emissive — full brightness always, with glow alpha and radius increasing at night.
/// Sign boards and their text render axis-aligned (upright in screen space); only signal-head
/// housings rotate to the approach tangent.
/// </summary>
public sealed class SignRenderer
{
    // ── LOD / layout constants ──────────────────────────────────────────

    /// <summary>Zoom below which nothing is drawn (matches vehicle-dot LOD).</summary>
    private const float MinZoom = 0.3f;
    /// <summary>Zoom at which drop shadows, STOP text, and the SPEED LIMIT header appear.</summary>
    private const float DetailZoom = 0.8f;
    /// <summary>Zoom at which YIELD lettering appears.</summary>
    private const float YieldTextZoom = 1.5f;
    /// <summary>cos(30°): minimum tangent alignment for an upstream edge to count as a continuation.</summary>
    private const float CosContinuation = 0.8660254f;
    /// <summary>Conversion factor from meters/second to miles/hour.</summary>
    private const float MphPerMs = 2.23694f;
    /// <summary>Circumradius (meters, before screen-constant scaling) of stop/yield sign faces.</summary>
    private const float SignFaceRadius = 1.1f;

    // ── Speed-sign placement cache (keyed on graph.Version) ────────────

    /// <summary>One cached speed-limit sign placement.</summary>
    private struct SpeedSign
    {
        /// <summary>World anchor position of the board center.</summary>
        public float X, Y;
        /// <summary>Road tangent angle (degrees) at the placement t. The board itself renders
        /// axis-aligned so its text stays upright in screen space; the angle is retained for
        /// callers/variants that want an oriented board.</summary>
        public float AngleDeg;
        /// <summary>Speed limit in mph, rounded to the nearest 5.</summary>
        public int Mph;
        /// <summary>Cached mph digits.</summary>
        public string Text;
        /// <summary>Half of the measured number width in board-local units (for centering).</summary>
        public float TextHalfWidth;
    }

    private readonly List<SpeedSign> _speedSigns = new();
    /// <summary>Graph version the speed-sign placement cache was built against.</summary>
    private int _speedCacheVersion = -1;

    // ── Reusable geometry (unit shapes, positioned via canvas transforms) ──

    /// <summary>Unit stop-sign octagon (circumradius 1, flat top).</summary>
    private readonly SKPath _octagonPath = CreateOctagon();
    /// <summary>Unit yield triangle (circumradius 1, apex pointing down-screen).</summary>
    private readonly SKPath _trianglePath = CreateTriangle();
    /// <summary>Signal-head housing in head-local space (+X = travel direction): 2.6 long, 1.1 wide.</summary>
    private readonly SKRoundRect _housingRect = new(new SKRect(-1.3f, -0.55f, 1.3f, 0.55f), 0.18f);
    /// <summary>Speed-limit board in board-local space: 1.2 wide, 1.6 tall.</summary>
    private readonly SKRoundRect _boardRect = new(new SKRect(-0.6f, -0.8f, 0.6f, 0.8f), 0.12f);

    // ── Reusable paints (colors refreshed once per Draw; no per-frame allocation) ──

    private readonly SKPaint _shadowPaint = new() { Color = new SKColor(0, 0, 0, 55), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _poleDotPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _housingPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _housingRimPaint = new() { Style = SKPaintStyle.Stroke, StrokeWidth = 0.15f, IsAntialias = true };
    private readonly SKPaint _lensPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _glowPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _stopFillPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _stopRimPaint = new() { Style = SKPaintStyle.Stroke, StrokeWidth = 0.12f, IsAntialias = true };
    private readonly SKPaint _stopTextPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _yieldFillPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _yieldBorderPaint = new() { Style = SKPaintStyle.Stroke, StrokeWidth = 0.28f, IsAntialias = true };
    private readonly SKPaint _yieldTextPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _boardFillPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _boardBorderPaint = new() { Style = SKPaintStyle.Stroke, StrokeWidth = 0.07f, IsAntialias = true };
    private readonly SKPaint _boardTextPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };

    // ── Emissive lens colors (never ambient-dimmed) ─────────────────────

    private static readonly SKColor RedActive = new(235, 60, 50);
    private static readonly SKColor YellowActive = new(250, 200, 60);
    private static readonly SKColor GreenActive = new(80, 220, 90);

    /// <summary>Inactive lens colors (~25% brightness of the active colors), ambient-dimmed per frame.</summary>
    private SKColor _redInactive, _yellowInactive, _greenInactive;
    /// <summary>Glow alphas for the inner/outer glow circles, scaled by (1 + darkness) per frame.</summary>
    private byte _glowAlpha1, _glowAlpha2;
    /// <summary>Glow radii (head-local units) for the inner/outer glow circles, enlarged at night.</summary>
    private float _glowR1, _glowR2;

    // ── Cached fonts and measured text extents (sized in unit-sign space) ──

    private static readonly SKTypeface BoldTypeface = SKTypeface.FromFamilyName(null, SKFontStyle.Bold);

    private readonly SKFont _stopFont;
    private readonly SKFont _yieldFont;
    private readonly SKFont _headerFont;
    private readonly SKFont _numberFont;
    private readonly float _stopTextHalfWidth;
    private readonly float _stopTextBaseline;
    private readonly float _yieldTextHalfWidth;
    private readonly float _yieldTextBaseline;
    private readonly float _speedHalfWidth;
    private readonly float _limitHalfWidth;

    /// <summary>
    /// Creates the renderer, sizing the cached fonts once so each label fits its unit-space
    /// sign face (text widths are measured a single time here, never per frame).
    /// </summary>
    public SignRenderer()
    {
        _stopFont = new SKFont(BoldTypeface, 1f) { Hinting = SKFontHinting.None, Subpixel = true };
        float w = MathF.Max(_stopFont.MeasureText("STOP"), 0.001f);
        _stopFont.Size = MathF.Min(0.8f, 1.55f / w);
        _stopTextHalfWidth = _stopFont.MeasureText("STOP") * 0.5f;
        _stopTextBaseline = 0.35f * _stopFont.Size;

        _yieldFont = new SKFont(BoldTypeface, 1f) { Hinting = SKFontHinting.None, Subpixel = true };
        w = MathF.Max(_yieldFont.MeasureText("YIELD"), 0.001f);
        _yieldFont.Size = MathF.Min(0.42f, 1.05f / w);
        _yieldTextHalfWidth = _yieldFont.MeasureText("YIELD") * 0.5f;
        _yieldTextBaseline = -0.15f + 0.35f * _yieldFont.Size;

        _headerFont = new SKFont(BoldTypeface, 1f) { Hinting = SKFontHinting.None, Subpixel = true };
        float wSpeed = _headerFont.MeasureText("SPEED");
        float wLimit = _headerFont.MeasureText("LIMIT");
        w = MathF.Max(MathF.Max(wSpeed, wLimit), 0.001f);
        _headerFont.Size = MathF.Min(0.3f, 0.95f / w);
        _speedHalfWidth = _headerFont.MeasureText("SPEED") * 0.5f;
        _limitHalfWidth = _headerFont.MeasureText("LIMIT") * 0.5f;

        _numberFont = new SKFont(BoldTypeface, 0.62f) { Hinting = SKFontHinting.None, Subpixel = true };
    }

    /// <summary>
    /// Draws all traffic-control furniture for the current frame: speed-limit boards first
    /// (bottom layer), then signal heads, stop signs, and yield signs. Everything is culled
    /// against <paramref name="cullRect"/> (an empty rect disables culling) and gated by zoom.
    /// </summary>
    /// <param name="canvas">Canvas in world-space coordinates.</param>
    /// <param name="graph">Road graph (read-only).</param>
    /// <param name="stopLines">Stop-line cache; must be rebuilt for the current graph version.</param>
    /// <param name="signals">Traffic-signal system; must be rebuilt at least once.</param>
    /// <param name="stopSigns">Stop-sign system (node predicate + per-edge exemptions only).</param>
    /// <param name="yieldSigns">Yield-sign system (node predicate + per-edge exemptions only).</param>
    /// <param name="zoom">Camera zoom in pixels per meter.</param>
    /// <param name="cullRect">Visible world rect; empty means draw everything.</param>
    /// <param name="darkness">Day/night factor in [0,1] (0 = day).</param>
    /// <param name="allowRebuild">When false, the speed-sign placement cache is not rebuilt
    /// even if the graph version changed (stale boards draw from baked positions). The caller
    /// passes false during continuous edits so placement work — which also reads
    /// <paramref name="stopLines"/> — runs only once the graph and its caches have settled.</param>
    public void Draw(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines,
                     TrafficSignalSystem signals, StopSignSystem stopSigns, YieldSignSystem yieldSigns,
                     float zoom, SKRect cullRect, float darkness, bool allowRebuild = true)
    {
        if (zoom < MinZoom) return;
        if (graph.Edges.Count == 0) return;

        float ambient = 1f - darkness * 0.45f;
        UpdateFrameColors(ambient, darkness);
        if (allowRebuild || _speedCacheVersion < 0)
            RebuildSpeedSignsIfNeeded(graph, stopLines);

        bool shadows = zoom >= DetailZoom;
        float scale = MathF.Max(1f, 2f / zoom);

        DrawSpeedSigns(canvas, zoom, cullRect, scale, shadows);
        DrawSignalHeads(canvas, graph, stopLines, signals, cullRect, scale, shadows);
        DrawStopSignFaces(canvas, graph, stopLines, stopSigns, zoom, cullRect, scale, shadows);
        DrawYieldSignFaces(canvas, graph, stopLines, yieldSigns, zoom, cullRect, scale, shadows);
    }

    // ── Speed-limit sign placement (sparse: only where the limit changes) ──

    /// <summary>Speed limit in mph rounded to the nearest 5. All limit comparisons use this
    /// rounded value so float noise cannot create phantom signs.</summary>
    private static int RoundMph(float metersPerSecond)
        => (int)MathF.Round(metersPerSecond * MphPerMs / 5f) * 5;

    /// <summary>
    /// Rebuilds the cached speed-sign placements when the graph version changes. Every active
    /// directed edge gets its own decision (no reverse-pair dedup — each travel direction posts
    /// its own signage). A sign is placed at the start of an edge iff its rounded mph differs
    /// from its upstream continuation's, or — with no continuation — from the road-type default.
    /// Placement t is 0.12, pushed past the FromNode-side stop trim when that is larger, and the
    /// board sits right of travel just off the visual asphalt.
    /// </summary>
    private void RebuildSpeedSignsIfNeeded(RoadGraph graph, StopLineCache stopLines)
    {
        if (_speedCacheVersion == graph.Version) return;
        _speedCacheVersion = graph.Version;
        _speedSigns.Clear();

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue; // defunct

            int mph = RoundMph(edge.SpeedLimit);
            int reverse = graph.FindReverseEdge(i);

            // Upstream continuation: the incoming edge at FromNode (excluding our own reverse)
            // whose arrival direction best aligns with our departure direction, within 30°.
            int continuation = -1;
            var startTan = graph.EvaluateBezierTangent(i, 0f);
            float startLen = startTan.Length();
            if (startLen >= 0.001f)
            {
                float dx = startTan.X / startLen, dy = startTan.Y / startLen;
                float bestDot = CosContinuation;
                foreach (int c in graph.GetIncomingEdges(edge.FromNode))
                {
                    if (c == reverse || c == i) continue;
                    if (graph.Edges[c].FromNode < 0) continue;
                    var endTan = graph.EvaluateBezierTangent(c, 1f);
                    float endLen = endTan.Length();
                    if (endLen < 0.001f) continue;
                    float dot = (endTan.X * dx + endTan.Y * dy) / endLen;
                    if (dot > bestDot)
                    {
                        bestDot = dot;
                        continuation = c;
                    }
                }
            }

            bool needSign;
            if (continuation >= 0)
            {
                needSign = RoundMph(graph.Edges[continuation].SpeedLimit) != mph;
            }
            else
            {
                // No aligned continuation (corridor start, sharp bend, or T-junction
                // departure). Post a sign only when the limit differs BOTH from the
                // road-type default (drivers assume the default for the road class) AND
                // from every incoming approach (a same-limit corridor arriving at a kink
                // or tee needs no re-posting) — keeps signage sparse without missing
                // genuine changes.
                needSign = RoundMph(RoadTypeDefaults.GetDefaultSpeedLimit(edge.RoadType)) != mph;
                if (needSign)
                {
                    foreach (int c in graph.GetIncomingEdges(edge.FromNode))
                    {
                        if (c == reverse || c == i) continue;
                        if (graph.Edges[c].FromNode < 0) continue;
                        if (RoundMph(graph.Edges[c].SpeedLimit) == mph) { needSign = false; break; }
                    }
                }
            }
            if (!needSign) continue;

            float t = MathF.Min(MathF.Max(0.12f, stopLines.GetStopTAtFromNode(i) + 0.03f), 0.9f);
            float offset = GeometryUtil.RoadHalfWidth(graph, i)
                           * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + 2f;
            var pos = GeometryUtil.OffsetRight(graph, i, t, offset);
            var tan = graph.EvaluateBezierTangent(i, t);
            string text = mph.ToString();

            _speedSigns.Add(new SpeedSign
            {
                X = pos.X,
                Y = pos.Y,
                AngleDeg = MathF.Atan2(tan.Y, tan.X) * (180f / MathF.PI),
                Mph = mph,
                Text = text,
                TextHalfWidth = _numberFont.MeasureText(text) * 0.5f,
            });
        }
    }

    /// <summary>
    /// Draws the cached speed-limit boards: white rounded rectangle with black border and pole
    /// dot, axis-aligned so the lettering stays upright. At zoom &gt;= 0.8 a tiny two-line
    /// SPEED LIMIT header renders above the number; below that only the bold number shows.
    /// </summary>
    private void DrawSpeedSigns(SKCanvas canvas, float zoom, SKRect cull, float scale, bool shadows)
    {
        float margin = 10f + 4f * scale;
        bool header = zoom >= DetailZoom;

        for (int k = 0; k < _speedSigns.Count; k++)
        {
            var sign = _speedSigns[k];
            if (!InView(sign.X, sign.Y, cull, margin)) continue;

            if (shadows)
                canvas.DrawCircle(sign.X + 0.5f, sign.Y + 0.5f, 0.6f * scale, _shadowPaint);

            canvas.Save();
            canvas.Translate(sign.X, sign.Y);
            canvas.Scale(scale);

            canvas.DrawCircle(0f, 0.95f, 0.16f, _poleDotPaint);
            canvas.DrawRoundRect(_boardRect, _boardFillPaint);
            canvas.DrawRoundRect(_boardRect, _boardBorderPaint);

            if (header)
            {
                canvas.DrawText("SPEED", -_speedHalfWidth, -0.4f, _headerFont, _boardTextPaint);
                canvas.DrawText("LIMIT", -_limitHalfWidth, -0.13f, _headerFont, _boardTextPaint);
                canvas.DrawText(sign.Text, -sign.TextHalfWidth, 0.55f, _numberFont, _boardTextPaint);
            }
            else
            {
                canvas.DrawText(sign.Text, -sign.TextHalfWidth, 0.22f, _numberFont, _boardTextPaint);
            }

            canvas.Restore();
        }
    }

    // ── Traffic-signal heads ────────────────────────────────────────────

    /// <summary>
    /// Draws one signal head per approach at every traffic-light node: pole dot, dark rounded
    /// housing with a lighter rim, and three lenses laid out along the travel direction
    /// (red first, then yellow, then green toward the intersection). The active lens renders
    /// emissive with two concentric glow circles; inactive lenses are near-dark. Heads rotate
    /// to the approach tangent and anchor at the stop line, offset right of the visual asphalt.
    /// </summary>
    private void DrawSignalHeads(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines,
        TrafficSignalSystem signals, SKRect cull, float scale, bool shadows)
    {
        float nodeMargin = 60f + 10f * scale;
        float signMargin = 10f + 4f * scale;
        int nodeCount = graph.Nodes.Count;

        for (int n = 0; n < nodeCount; n++)
        {
            if (!signals.IsTrafficLight(n)) continue;
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            if (!InView(node.Position.X, node.Position.Y, cull, nodeMargin)) continue;

            foreach (int edgeIdx in graph.GetIncomingEdges(n))
            {
                if (!TryGetApproachAnchor(graph, stopLines, edgeIdx, 1.2f,
                        out float sx, out float sy, out float tx, out float ty))
                    continue;
                if (!InView(sx, sy, cull, signMargin)) continue;

                if (shadows)
                    canvas.DrawCircle(sx + 0.5f, sy + 0.5f, 0.6f * scale, _shadowPaint);

                var state = signals.GetSignal(edgeIdx);

                canvas.Save();
                canvas.Translate(sx, sy);
                canvas.RotateDegrees(MathF.Atan2(ty, tx) * (180f / MathF.PI));
                canvas.Scale(scale);

                canvas.DrawCircle(0f, 1f, 0.175f, _poleDotPaint);
                canvas.DrawRoundRect(_housingRect, _housingPaint);
                canvas.DrawRoundRect(_housingRect, _housingRimPaint);

                DrawLens(canvas, -0.85f, RedActive, _redInactive, state == SignalState.Red);
                DrawLens(canvas, 0f, YellowActive, _yellowInactive, state == SignalState.Yellow);
                DrawLens(canvas, 0.85f, GreenActive, _greenInactive, state == SignalState.Green);

                canvas.Restore();
            }
        }
    }

    /// <summary>
    /// Draws one lens at head-local x. Active: full-brightness fill plus two concentric glow
    /// circles (alpha and radius grow with darkness). Inactive: near-dark ambient-dimmed fill.
    /// </summary>
    private void DrawLens(SKCanvas canvas, float lx, SKColor active, SKColor inactive, bool isActive)
    {
        _lensPaint.Color = isActive ? active : inactive;
        canvas.DrawCircle(lx, 0f, 0.275f, _lensPaint);
        if (!isActive) return;

        _glowPaint.Color = active.WithAlpha(_glowAlpha1);
        canvas.DrawCircle(lx, 0f, _glowR1, _glowPaint);
        _glowPaint.Color = active.WithAlpha(_glowAlpha2);
        canvas.DrawCircle(lx, 0f, _glowR2, _glowPaint);
    }

    // ── Stop signs ──────────────────────────────────────────────────────

    /// <summary>
    /// Draws a red octagon with white rim (plus pole dot) per non-exempt approach at every
    /// stop-sign node, anchored at the stop line and offset right of the visual asphalt.
    /// White STOP lettering renders at zoom &gt;= 0.8. Faces are axis-aligned so text stays
    /// upright in screen space.
    /// </summary>
    private void DrawStopSignFaces(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines,
        StopSignSystem stopSigns, float zoom, SKRect cull, float scale, bool shadows)
    {
        float radius = SignFaceRadius * scale; // == max(1.1, 2.2/zoom)
        float nodeMargin = 60f + 10f * scale;
        float signMargin = 10f + 2f * radius;
        bool text = zoom >= DetailZoom;
        int nodeCount = graph.Nodes.Count;

        for (int n = 0; n < nodeCount; n++)
        {
            if (!stopSigns.IsStopSign(n)) continue;
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            if (!InView(node.Position.X, node.Position.Y, cull, nodeMargin)) continue;

            foreach (int edgeIdx in graph.GetIncomingEdges(n))
            {
                if (stopSigns.IsEdgeExempt(edgeIdx)) continue;
                if (!TryGetApproachAnchor(graph, stopLines, edgeIdx, 2f,
                        out float sx, out float sy, out _, out _))
                    continue;
                if (!InView(sx, sy, cull, signMargin)) continue;

                if (shadows)
                    canvas.DrawCircle(sx + 0.5f, sy + 0.5f, 0.6f * scale, _shadowPaint);

                canvas.Save();
                canvas.Translate(sx, sy);
                canvas.Scale(radius);

                canvas.DrawCircle(0f, 1.05f, 0.16f, _poleDotPaint);
                canvas.DrawPath(_octagonPath, _stopFillPaint);
                canvas.DrawPath(_octagonPath, _stopRimPaint);
                if (text)
                    canvas.DrawText("STOP", -_stopTextHalfWidth, _stopTextBaseline, _stopFont, _stopTextPaint);

                canvas.Restore();
            }
        }
    }

    // ── Yield signs ─────────────────────────────────────────────────────

    /// <summary>
    /// Draws a white inverted triangle with a thick red border (plus pole dot) per non-exempt
    /// approach at every yield node, anchored like stop signs. Red YIELD lettering renders at
    /// zoom &gt;= 1.5. Faces are axis-aligned so text stays upright in screen space.
    /// </summary>
    private void DrawYieldSignFaces(SKCanvas canvas, RoadGraph graph, StopLineCache stopLines,
        YieldSignSystem yieldSigns, float zoom, SKRect cull, float scale, bool shadows)
    {
        float radius = SignFaceRadius * scale;
        float nodeMargin = 60f + 10f * scale;
        float signMargin = 10f + 2f * radius;
        bool text = zoom >= YieldTextZoom;
        int nodeCount = graph.Nodes.Count;

        for (int n = 0; n < nodeCount; n++)
        {
            if (!yieldSigns.IsYield(n)) continue;
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            if (!InView(node.Position.X, node.Position.Y, cull, nodeMargin)) continue;

            foreach (int edgeIdx in graph.GetIncomingEdges(n))
            {
                if (yieldSigns.IsEdgeExempt(edgeIdx)) continue;
                if (!TryGetApproachAnchor(graph, stopLines, edgeIdx, 2f,
                        out float sx, out float sy, out _, out _))
                    continue;
                if (!InView(sx, sy, cull, signMargin)) continue;

                if (shadows)
                    canvas.DrawCircle(sx + 0.5f, sy + 0.5f, 0.6f * scale, _shadowPaint);

                canvas.Save();
                canvas.Translate(sx, sy);
                canvas.Scale(radius);

                canvas.DrawCircle(0f, 1.05f, 0.16f, _poleDotPaint);
                canvas.DrawPath(_trianglePath, _yieldFillPaint);
                canvas.DrawPath(_trianglePath, _yieldBorderPaint);
                if (text)
                    canvas.DrawText("YIELD", -_yieldTextHalfWidth, _yieldTextBaseline, _yieldFont, _yieldTextPaint);

                canvas.Restore();
            }
        }
    }

    // ── Shared helpers ──────────────────────────────────────────────────

    /// <summary>
    /// Computes the world anchor for a sign serving one approach edge: the stop-line point
    /// (t = StopLineCache stop-T at the ToNode), offset right of travel by the visual asphalt
    /// half-width (geometric half-width × road-type width multiplier) plus
    /// <paramref name="extraOffset"/> meters. Also returns the normalized approach tangent.
    /// Returns false for defunct edges or degenerate tangents.
    /// </summary>
    private static bool TryGetApproachAnchor(RoadGraph graph, StopLineCache stopLines, int edgeIdx,
        float extraOffset, out float sx, out float sy, out float tx, out float ty)
    {
        sx = sy = tx = ty = 0f;
        var edge = graph.Edges[edgeIdx];
        if (edge.FromNode < 0) return false;

        float stopT = stopLines.GetStopTAtToNode(edgeIdx);
        var pos = graph.EvaluateBezier(edgeIdx, stopT);
        var tangent = graph.EvaluateBezierTangent(edgeIdx, stopT);
        float len = tangent.Length();
        if (len < 0.001f) return false;

        tx = tangent.X / len;
        ty = tangent.Y / len;
        float rx = -ty; // right-of-travel normal in Y-down coords
        float ry = tx;
        float offset = GeometryUtil.RoadHalfWidth(graph, edgeIdx)
                       * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + extraOffset;
        sx = pos.X + rx * offset;
        sy = pos.Y + ry * offset;
        return true;
    }

    /// <summary>True if (x,y) is within the cull rect expanded by <paramref name="margin"/>,
    /// or when culling is off (an empty rect means "draw everything").</summary>
    private static bool InView(float x, float y, SKRect cull, float margin)
        => (cull.Width <= 0f && cull.Height <= 0f)
           || (x >= cull.Left - margin && x <= cull.Right + margin
               && y >= cull.Top - margin && y <= cull.Bottom + margin);

    /// <summary>
    /// Refreshes all passive paint colors for the frame's ambient level and computes the
    /// emissive glow parameters (alpha × (1 + darkness), radius × (1 + 0.5 × darkness)).
    /// </summary>
    private void UpdateFrameColors(float ambient, float darkness)
    {
        _poleDotPaint.Color = new SKColor(Dim(50, ambient), Dim(52, ambient), Dim(55, ambient));
        _housingPaint.Color = new SKColor(Dim(30, ambient), Dim(32, ambient), Dim(34, ambient));
        _housingRimPaint.Color = new SKColor(Dim(78, ambient), Dim(82, ambient), Dim(86, ambient));

        var signRed = new SKColor(Dim(178, ambient), Dim(34, ambient), Dim(52, ambient));
        var signWhite = new SKColor(Dim(240, ambient), Dim(240, ambient), Dim(237, ambient));

        _stopFillPaint.Color = signRed;
        _stopRimPaint.Color = signWhite;
        _stopTextPaint.Color = signWhite;
        _yieldFillPaint.Color = signWhite;
        _yieldBorderPaint.Color = signRed;
        _yieldTextPaint.Color = signRed;
        _boardFillPaint.Color = signWhite;
        _boardBorderPaint.Color = new SKColor(Dim(25, ambient), Dim(26, ambient), Dim(28, ambient));
        _boardTextPaint.Color = new SKColor(Dim(15, ambient), Dim(15, ambient), Dim(17, ambient));

        // Inactive lenses: ~25% brightness of the active colors, then ambient-dimmed.
        _redInactive = new SKColor(Dim(59, ambient), Dim(15, ambient), Dim(13, ambient));
        _yellowInactive = new SKColor(Dim(63, ambient), Dim(50, ambient), Dim(15, ambient));
        _greenInactive = new SKColor(Dim(20, ambient), Dim(55, ambient), Dim(23, ambient));

        _glowAlpha1 = (byte)Math.Min(255, (int)(70f * (1f + darkness)));
        _glowAlpha2 = (byte)Math.Min(255, (int)(30f * (1f + darkness)));
        _glowR1 = 1.2f * (1f + 0.5f * darkness);
        _glowR2 = 2.2f * (1f + 0.5f * darkness);
    }

    /// <summary>Multiplies a color channel by the ambient factor, clamped to byte range.</summary>
    private static byte Dim(int baseValue, float ambient)
        => (byte)Math.Clamp((int)(baseValue * ambient), 0, 255);

    /// <summary>Builds the unit stop-sign octagon: circumradius 1, vertices at 22.5° + k·45°
    /// so the top and bottom edges are flat.</summary>
    private static SKPath CreateOctagon()
    {
        var path = new SKPath();
        for (int k = 0; k < 8; k++)
        {
            float a = (22.5f + 45f * k) * (MathF.PI / 180f);
            float x = MathF.Cos(a);
            float y = MathF.Sin(a);
            if (k == 0) path.MoveTo(x, y);
            else path.LineTo(x, y);
        }
        path.Close();
        return path;
    }

    /// <summary>Builds the unit yield triangle: circumradius 1, inverted (apex at +Y,
    /// which points down-screen in Y-down world coordinates).</summary>
    private static SKPath CreateTriangle()
    {
        var path = new SKPath();
        path.MoveTo(0f, 1f);
        path.LineTo(-0.866f, -0.5f);
        path.LineTo(0.866f, -0.5f);
        path.Close();
        return path;
    }
}
