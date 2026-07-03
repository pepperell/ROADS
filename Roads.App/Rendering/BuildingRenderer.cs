using System.Numerics;
using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Draws the procedural buildings placed by <see cref="BuildingLayer"/> plus the marker
/// fallbacks that keep every destination visible, replacing the old POI dot pass.
/// LOD tiers (zoom = pixels per meter):
/// <list type="bullet">
/// <item>zoom &lt; 0.3 — classic destination dot (POI color fill, white ring and inner dot),
/// identical to the retired MarkerRenderer pass, for every real-POI destination node.</item>
/// <item>0.3–0.8 — each building as a rotated flat rectangle in a muted wall color with a
/// subtle POI-color tint and a thin dark outline.</item>
/// <item>&gt; 0.8 — full per-type art in TWO passes over the visible set: first every
/// building's ground plane (lawns, school fields, walkways, aprons, parking pads/lots),
/// then every structure (roofs, walls, doors, windows) — so one building's ground margin
/// can never paint over a neighboring structure; &gt; 1.5 adds close-up trim (chimneys,
/// AC units).</item>
/// </list>
/// Destination nodes that received no footprint keep the dot at ALL zooms. EntryExit
/// nodes never have footprints: below 0.3 they keep the magenta dot, at 0.3+ they get a
/// highway-gateway marker (two posts and a green sign board across the road). At night
/// (darkness &gt; 0.3) roofed buildings show warm emissive window rects (never dimmed)
/// and Homes/Shops a faint warm halo; all other colors are dimmed by
/// ambient = 1 − darkness × 0.45.
///
/// Call order: <see cref="BuildingLayer.RebuildIfNeeded"/> must run before
/// <see cref="Draw"/> each frame. Rendering is read-only over the graph; all paints are
/// reusable fields (no per-frame allocation) and every building is culled against
/// <c>viewRect</c> via its precomputed AABB.
/// </summary>
public sealed class BuildingRenderer
{
    private const float RadToDeg = 180f / MathF.PI;

    // Reusable paints (colors/widths set per use; never allocated per frame).
    private readonly SKPaint _fill = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _stroke = new() { Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _line = new() { Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _glow = new() { Style = SKPaintStyle.Fill, IsAntialias = true };

    private static readonly SKColor OutlineColor = new(42, 44, 42);
    private static readonly SKColor LawnColor = new(100, 118, 72);
    private static readonly SKColor FieldColor = new(70, 100, 58);
    private static readonly SKColor WindowColor = new(255, 214, 150);
    private static readonly SKColor HaloColor = new(255, 190, 110);
    private static readonly SKColor SignBoardColor = new(46, 84, 52);

    /// <summary>Home roof palettes (light half, dark half) hash-picked per building.</summary>
    private static readonly (SKColor L, SKColor D)[] HomeRoofPalettes =
    {
        (new SKColor(139, 101, 84), new SKColor(121, 86, 72)),
        (new SKColor(112, 112, 116), new SKColor(96, 96, 100)),
        (new SKColor(125, 96, 62), new SKColor(107, 80, 52)),
        (new SKColor(100, 78, 70), new SKColor(85, 65, 58)),
    };

    /// <summary>Leisure roof palettes — warmer terracotta tones.</summary>
    private static readonly (SKColor L, SKColor D)[] LeisureRoofPalettes =
    {
        (new SKColor(154, 108, 84), new SKColor(134, 92, 70)),
        (new SKColor(148, 120, 82), new SKColor(128, 102, 68)),
    };

    /// <summary>Muted awning accent colors for shops, hash-picked per building.</summary>
    private static readonly SKColor[] AwningColors =
    {
        new(150, 84, 72), new(84, 104, 124), new(122, 110, 66), new(104, 86, 106),
    };

    /// <summary>
    /// Draws all buildings and destination markers. <paramref name="darkness"/> is the
    /// day/night factor (0 = day); non-emissive colors are dimmed by 1 − darkness × 0.45.
    /// </summary>
    public void Draw(SKCanvas canvas, BuildingLayer layer, RoadGraph graph,
        SKRect viewRect, float zoom, float darkness)
    {
        float ambient = 1f - darkness * 0.45f;
        float nightFactor = darkness <= 0.3f ? 0f : Math.Clamp((darkness - 0.3f) / 0.7f, 0f, 1f);

        if (zoom >= 0.3f)
        {
            // Halos extend past the footprint AABB; widen the cull test at night.
            var cullRect = viewRect;
            cullRect.Inflate(nightFactor > 0f ? 40f : 4f, nightFactor > 0f ? 40f : 4f);

            var buildings = layer.Buildings;
            if (zoom < 0.8f)
            {
                for (int i = 0; i < buildings.Count; i++)
                {
                    if (!cullRect.IntersectsWith(layer.GetBounds(i))) continue;
                    var b = buildings[i];
                    DrawFlat(canvas, in b, ambient, zoom);
                }
            }
            else
            {
                // Full-detail tier draws in two passes: ALL ground planes first, then ALL
                // structures. Ground art extends past the footprint (lawn margins, school
                // fields, aprons), so an interleaved per-building pass would let a later
                // building's ground paint over an already-drawn neighbor's roof.
                for (int i = 0; i < buildings.Count; i++)
                {
                    if (!cullRect.IntersectsWith(layer.GetBounds(i))) continue;
                    var b = buildings[i];
                    DrawGround(canvas, layer, i, in b, graph, zoom, ambient);
                }
                for (int i = 0; i < buildings.Count; i++)
                {
                    if (!cullRect.IntersectsWith(layer.GetBounds(i))) continue;
                    var b = buildings[i];

                    // Halos only at the full-detail tier: in the 0.3-0.8 overview band a
                    // whole city of homes is visible and thousands of blended circles would
                    // eat the frame budget for ~5 px halos that visually merge anyway.
                    if (nightFactor > 0f && (b.Type == POIType.Home || b.Type == POIType.Shop))
                        DrawHalo(canvas, in b, nightFactor);

                    DrawStructure(canvas, in b, graph, zoom, ambient, nightFactor);
                }
            }
        }

        DrawNodeMarkers(canvas, layer, graph, viewRect, zoom, ambient);
    }

    // ── Marker fallbacks (dots and gateways) ─────────────────────────────

    /// <summary>
    /// Per-node marker pass: classic dots for footprint-less destinations (all zooms) and
    /// for every destination below zoom 0.3; gateway markers (or dots below 0.3) for
    /// EntryExit nodes. Destination nodes with POIType.None (stress-map intersections)
    /// are skipped entirely.
    /// </summary>
    private void DrawNodeMarkers(SKCanvas canvas, BuildingLayer layer, RoadGraph graph,
        SKRect viewRect, float zoom, float ambient)
    {
        float dotRadius = MathF.Max(4f, 6f / zoom);
        var cullRect = viewRect;
        cullRect.Inflate(dotRadius + 30f, dotRadius + 30f);

        var nodes = graph.Nodes;
        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if ((node.Flags & NodeFlags.Destination) == 0) continue;
            var poi = node.PointOfInterest;
            if (poi == POIType.None) continue;
            if (node.Position.X < cullRect.Left || node.Position.X > cullRect.Right
                || node.Position.Y < cullRect.Top || node.Position.Y > cullRect.Bottom) continue;

            if (poi == POIType.EntryExit)
            {
                if (zoom >= 0.3f)
                    DrawGateway(canvas, graph, i, node.Position, zoom, ambient);
                else
                    DrawDot(canvas, node.Position, PoiColor(poi), zoom);
                continue;
            }

            if (zoom < 0.3f || !layer.HasFootprint(i))
                DrawDot(canvas, node.Position, PoiColor(poi), zoom);
        }
    }

    /// <summary>Classic destination dot: POI color fill, white ring, white inner dot.</summary>
    private void DrawDot(SKCanvas canvas, Vector2 pos, SKColor color, float zoom)
    {
        float radius = MathF.Max(4f, 6f / zoom);
        _fill.Color = color;
        canvas.DrawCircle(pos.X, pos.Y, radius, _fill);
        _stroke.Color = new SKColor(255, 255, 255, 220);
        _stroke.StrokeWidth = MathF.Max(1f, 1.5f / zoom);
        canvas.DrawCircle(pos.X, pos.Y, radius, _stroke);
        _fill.Color = new SKColor(255, 255, 255, 200);
        canvas.DrawCircle(pos.X, pos.Y, radius * 0.5f, _fill);
    }

    /// <summary>
    /// Highway-gateway marker for an EntryExit node: two gray post dots flanking the road
    /// and a green sign board with a white border spanning it, oriented perpendicular to
    /// the incident edge tangent.
    /// </summary>
    private void DrawGateway(SKCanvas canvas, RoadGraph graph, int nodeIndex, Vector2 pos,
        float zoom, float ambient)
    {
        int edge = -1;
        float tEnd = 0f;
        foreach (int e in graph.GetOutgoingEdges(nodeIndex))
        {
            if (graph.Edges[e].FromNode < 0) continue;
            edge = e; tEnd = 0f;
            break;
        }
        if (edge < 0)
        {
            foreach (int e in graph.GetIncomingEdges(nodeIndex))
            {
                if (graph.Edges[e].FromNode < 0) continue;
                edge = e; tEnd = 1f;
                break;
            }
        }
        if (edge < 0) return;

        var tangent = graph.EvaluateBezierTangent(edge, tEnd);
        float len = tangent.Length();
        if (len < 0.001f) return;
        var right = new Vector2(-tangent.Y / len, tangent.X / len);

        float halfSpan = GeometryUtil.RoadHalfWidth(graph, edge)
            * RoadTypeVisuals.GetWidthMultiplier(graph.Edges[edge].RoadType) + 1f;
        float halfThick = MathF.Max(0.9f, 1.25f / zoom);

        canvas.Save();
        canvas.Translate(pos.X, pos.Y);
        canvas.RotateDegrees(MathF.Atan2(right.Y, right.X) * RadToDeg);

        _fill.Color = Dim(SignBoardColor, ambient);
        var board = new SKRect(-halfSpan, -halfThick, halfSpan, halfThick);
        canvas.DrawRect(board, _fill);
        _stroke.Color = new SKColor(255, 255, 255, 230);
        _stroke.StrokeWidth = MathF.Max(0.25f, 0.8f / zoom);
        canvas.DrawRect(board, _stroke);

        float postR = MathF.Max(0.6f, 1.2f / zoom);
        _fill.Color = Dim(new SKColor(120, 122, 126), ambient);
        canvas.DrawCircle(-halfSpan, 0f, postR, _fill);
        canvas.DrawCircle(halfSpan, 0f, postR, _fill);

        canvas.Restore();
    }

    // ── Building tiers ───────────────────────────────────────────────────

    /// <summary>Mid-zoom tier: rotated flat rect, muted wall color with POI tint, dark outline.</summary>
    private void DrawFlat(SKCanvas canvas, in BuildingFootprint b, float ambient, float zoom)
    {
        canvas.Save();
        canvas.Translate(b.Center.X, b.Center.Y);
        canvas.RotateDegrees(b.FacingRadians * RadToDeg);

        var rect = new SKRect(-b.HalfDepth, -b.HalfWidth, b.HalfDepth, b.HalfWidth);
        _fill.Color = Dim(Blend(WallColor(b.Type), PoiColor(b.Type), 0.22f), ambient);
        canvas.DrawRect(rect, _fill);
        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = MathF.Max(0.4f, 1f / zoom);
        canvas.DrawRect(rect, _stroke);

        canvas.Restore();
    }

    /// <summary>
    /// Enters the building's local frame shared by both full-tier passes: computes the
    /// facing axis, right-lateral axis, and the destination node's local position, then
    /// pushes a canvas transform whose local +X axis is the facing direction (front/facade
    /// at x = +HalfDepth, the node beyond it) and +Y the lateral. Returns false (no
    /// transform pushed) for a stale footprint mid-edit; on true the caller must Restore.
    /// </summary>
    private static bool TryBeginLocalFrame(SKCanvas canvas, in BuildingFootprint b,
        RoadGraph graph, out Vector2 axis, out Vector2 lat, out Vector2 nodeLocal)
    {
        axis = default; lat = default; nodeLocal = default;
        var nodePos = graph.Nodes[b.NodeIndex].Position;
        if (float.IsNaN(nodePos.X)) return false; // stale footprint mid-edit; skip this frame

        axis = new Vector2(MathF.Cos(b.FacingRadians), MathF.Sin(b.FacingRadians));
        lat = new Vector2(-axis.Y, axis.X);
        var rel = nodePos - b.Center;
        nodeLocal = new Vector2(Vector2.Dot(rel, axis), Vector2.Dot(rel, lat));

        canvas.Save();
        canvas.Translate(b.Center.X, b.Center.Y);
        canvas.RotateDegrees(b.FacingRadians * RadToDeg);
        return true;
    }

    /// <summary>
    /// Full-tier pass 1: the building's ground plane — everything at grade that may extend
    /// past the footprint (lawn, school field, walkway, apron, parking pad, patio, the
    /// whole unroofed parking lot). Runs for ALL visible buildings before any structure.
    /// </summary>
    private void DrawGround(SKCanvas canvas, BuildingLayer layer, int buildingIndex,
        in BuildingFootprint b, RoadGraph graph, float zoom, float ambient)
    {
        if (!TryBeginLocalFrame(canvas, in b, graph, out var axis, out var lat, out var nodeLocal))
            return;

        float outlineW = MathF.Max(0.3f, 0.8f / zoom);
        switch (b.Type)
        {
            case POIType.Home:
                DrawHomeGround(canvas, in b, nodeLocal, ambient);
                break;
            case POIType.Work:
                DrawWorkGround(canvas, in b, nodeLocal, ambient);
                break;
            case POIType.Shop:
                DrawShopGround(canvas, in b, nodeLocal, ambient, zoom);
                break;
            case POIType.Leisure:
                DrawLeisureGround(canvas, in b, ambient, outlineW);
                break;
            case POIType.School:
                DrawSchoolGround(canvas, layer, buildingIndex, in b, axis, lat, ambient, zoom);
                break;
            case POIType.Parking:
                DrawParkingGround(canvas, in b, ambient, outlineW);
                break;
        }

        canvas.Restore();
    }

    /// <summary>
    /// Full-tier pass 2: the structure (roof, walls, door, trim, night windows). Runs after
    /// every visible building's ground plane so no lawn/field/apron overlaps a structure.
    /// Parking lots are entirely ground and draw nothing here.
    /// </summary>
    private void DrawStructure(SKCanvas canvas, in BuildingFootprint b, RoadGraph graph,
        float zoom, float ambient, float nightFactor)
    {
        if (!TryBeginLocalFrame(canvas, in b, graph, out var axis, out _, out _))
            return;

        float outlineW = MathF.Max(0.3f, 0.8f / zoom);
        switch (b.Type)
        {
            case POIType.Home:
                DrawHomeStructure(canvas, in b, axis, ambient, zoom, outlineW);
                break;
            case POIType.Work:
                DrawWorkStructure(canvas, in b, ambient, zoom, outlineW);
                break;
            case POIType.Shop:
                DrawShopStructure(canvas, in b, ambient, outlineW);
                break;
            case POIType.Leisure:
                DrawLeisureStructure(canvas, in b, axis, ambient, outlineW);
                break;
            case POIType.School:
                DrawSchoolStructure(canvas, in b, ambient, outlineW);
                break;
        }

        if (nightFactor > 0f && b.Type != POIType.Parking)
            DrawWindows(canvas, in b, nightFactor);

        canvas.Restore();
    }

    // ── Per-type art (local frame: +X toward front/node, +Y lateral) ─────

    /// <summary>Home ground: lawn under the house and the walkway toward the node.</summary>
    private void DrawHomeGround(SKCanvas canvas, in BuildingFootprint b, Vector2 nodeLocal,
        float ambient)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        // Lawn (slightly larger) under everything. Margin stays within the 1.5 m road
        // clearance the placement pass guarantees, so grass never paints over asphalt.
        _fill.Color = Dim(LawnColor, ambient);
        canvas.DrawRect(new SKRect(-hd - 1.4f, -hw - 1.4f, hd + 1.4f, hw + 1.4f), _fill);

        // Walkway from the front door toward the node, stopping 2 m short so it never
        // paints onto the road surface when the node sits on/next to asphalt.
        var walkDir = new Vector2(nodeLocal.X - hd, nodeLocal.Y);
        float walkLen = walkDir.Length();
        if (walkLen > 2.5f)
        {
            walkDir /= walkLen;
            _line.Color = Dim(new SKColor(168, 162, 148), ambient);
            _line.StrokeWidth = 1.1f;
            canvas.DrawLine(hd, 0f,
                hd + walkDir.X * (walkLen - 2f), walkDir.Y * (walkLen - 2f), _line);
        }
    }

    /// <summary>Home structure: two-tone gable roof, front door, chimney at close zoom.</summary>
    private void DrawHomeStructure(SKCanvas canvas, in BuildingFootprint b, Vector2 axis,
        float ambient, float zoom, float outlineW)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        var pal = HomeRoofPalettes[(int)(Hash(b.Seed, 11) & 3)];
        DrawGableRoof(canvas, hd, hw, axis, pal.L, pal.D, ambient, outlineW);

        // Front door on the facade.
        _fill.Color = Dim(new SKColor(88, 66, 50), ambient);
        canvas.DrawRect(new SKRect(hd - 1f, -0.8f, hd, 0.8f), _fill);

        if (zoom > 1.5f)
        {
            // Chimney square near the ridge.
            _fill.Color = Dim(new SKColor(105, 95, 90), ambient);
            var chimney = new SKRect(-hd * 0.35f - 0.55f, hw * 0.35f - 0.55f,
                                     -hd * 0.35f + 0.55f, hw * 0.35f + 0.55f);
            canvas.DrawRect(chimney, _fill);
            _stroke.Color = OutlineColor;
            _stroke.StrokeWidth = outlineW * 0.7f;
            canvas.DrawRect(chimney, _stroke);
        }
    }

    /// <summary>Work ground: concrete apron between the facade and the node.</summary>
    private void DrawWorkGround(SKCanvas canvas, in BuildingFootprint b, Vector2 nodeLocal,
        float ambient)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        // Apron stops 2.5 m short of the node so it never paints across the
        // sidewalk/asphalt when the node sits on a road (the driveway connector edge,
        // drawn as a road, visually fills the gap for off-road POIs).
        float apronHalfW = MathF.Min(hw, 5f);
        float apronEnd = MathF.Max(hd + 2f, nodeLocal.X - 2.5f);
        _fill.Color = Dim(new SKColor(150, 148, 142), ambient);
        canvas.DrawRect(new SKRect(hd, -apronHalfW, apronEnd, apronHalfW), _fill);
    }

    /// <summary>Work structure: flat roof with darker parapet inset, AC units at close zoom.</summary>
    private void DrawWorkStructure(SKCanvas canvas, in BuildingFootprint b,
        float ambient, float zoom, float outlineW)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        var body = new SKRect(-hd, -hw, hd, hw);
        _fill.Color = Dim(new SKColor(128, 126, 122), ambient);
        canvas.DrawRect(body, _fill);
        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW;
        canvas.DrawRect(body, _stroke);

        // Parapet: darker inset outline.
        _stroke.Color = Dim(new SKColor(100, 98, 94), ambient);
        _stroke.StrokeWidth = MathF.Max(0.3f, 0.6f / zoom);
        canvas.DrawRect(new SKRect(-hd + 1.2f, -hw + 1.2f, hd - 1.2f, hw - 1.2f), _stroke);

        if (zoom > 1.5f)
        {
            int count = 2 + (int)(Hash(b.Seed, 7) % 3u);
            _fill.Color = Dim(new SKColor(158, 156, 150), ambient);
            for (int k = 0; k < count; k++)
            {
                float x = -hd + 2f + Hash01(b.Seed, (uint)(20 + k)) * (2f * hd - 4f);
                float y = -hw + 2f + Hash01(b.Seed, (uint)(40 + k)) * (2f * hw - 4f);
                canvas.DrawRect(new SKRect(x - 0.7f, y - 0.7f, x + 0.7f, y + 0.7f), _fill);
            }
        }
    }

    /// <summary>Shop ground: front parking pad with white stall lines toward the node.</summary>
    private void DrawShopGround(SKCanvas canvas, in BuildingFootprint b, Vector2 nodeLocal,
        float ambient, float zoom)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        // Parking pad between the facade and the node, stopping 2.5 m short of the node so
        // it never paints across the sidewalk/asphalt when the node sits on a road.
        float padEnd = Math.Clamp(nodeLocal.X - 2.5f, hd + 2.5f, hd + 12f);
        _fill.Color = Dim(new SKColor(72, 72, 75), ambient);
        canvas.DrawRect(new SKRect(hd, -hw, padEnd, hw), _fill);

        // Stall lines dividing the pad.
        int stalls = 2 + (int)(Hash(b.Seed, 9) % 3u);
        _line.Color = new SKColor(255, 255, 255, (byte)(150 * ambient));
        _line.StrokeWidth = MathF.Max(0.25f, 0.5f / zoom);
        for (int k = 0; k < stalls; k++)
        {
            float y = -hw * 0.7f + (2f * hw * 0.7f) * ((k + 0.5f) / stalls);
            canvas.DrawLine(hd + 0.4f, y, padEnd - 0.4f, y, _line);
        }
    }

    /// <summary>Shop structure: flat roof, muted accent awning strip along the facade.</summary>
    private void DrawShopStructure(SKCanvas canvas, in BuildingFootprint b,
        float ambient, float outlineW)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        var body = new SKRect(-hd, -hw, hd, hw);
        _fill.Color = Dim(new SKColor(134, 130, 124), ambient);
        canvas.DrawRect(body, _fill);
        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW;
        canvas.DrawRect(body, _stroke);

        // Awning strip along the facade.
        _fill.Color = Dim(AwningColors[(int)(Hash(b.Seed, 5) & 3)], ambient);
        canvas.DrawRect(new SKRect(hd, -hw, hd + 1.3f, hw), _fill);
    }

    /// <summary>Leisure ground: small patio circle on a hash-picked side.</summary>
    private void DrawLeisureGround(SKCanvas canvas, in BuildingFootprint b,
        float ambient, float outlineW)
    {
        float hw = b.HalfWidth;

        float side = (Hash(b.Seed, 13) & 1) == 0 ? 1f : -1f;
        _fill.Color = Dim(new SKColor(152, 146, 136), ambient);
        canvas.DrawCircle(0f, side * (hw + 3f), 2.6f, _fill);
        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW * 0.7f;
        canvas.DrawCircle(0f, side * (hw + 3f), 2.6f, _stroke);
    }

    /// <summary>Leisure structure: warm-palette gable roof.</summary>
    private void DrawLeisureStructure(SKCanvas canvas, in BuildingFootprint b, Vector2 axis,
        float ambient, float outlineW)
    {
        var pal = LeisureRoofPalettes[(int)(Hash(b.Seed, 11) & 1)];
        DrawGableRoof(canvas, b.HalfDepth, b.HalfWidth, axis, pal.L, pal.D, ambient, outlineW);
    }

    /// <summary>School ground: green field beside the building (when placed).</summary>
    private void DrawSchoolGround(SKCanvas canvas, BuildingLayer layer, int buildingIndex,
        in BuildingFootprint b, Vector2 axis, Vector2 lat, float ambient, float zoom)
    {
        if (!layer.TryGetSchoolField(buildingIndex, out var fieldCenter, out float fhw, out float fhd))
            return;

        var relField = fieldCenter - b.Center;
        float fx = Vector2.Dot(relField, axis);
        float fy = Vector2.Dot(relField, lat);
        var fieldRect = new SKRect(fx - fhd, fy - fhw, fx + fhd, fy + fhw);
        _fill.Color = Dim(FieldColor, ambient);
        canvas.DrawRect(fieldRect, _fill);
        _stroke.Color = new SKColor(255, 255, 255, (byte)(150 * ambient));
        _stroke.StrokeWidth = MathF.Max(0.25f, 0.6f / zoom);
        canvas.DrawRect(fieldRect, _stroke);
    }

    /// <summary>School structure: large two-tone hip-style roof (dark base, lighter inset top).</summary>
    private void DrawSchoolStructure(SKCanvas canvas, in BuildingFootprint b,
        float ambient, float outlineW)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        var body = new SKRect(-hd, -hw, hd, hw);
        _fill.Color = Dim(new SKColor(108, 100, 90), ambient);
        canvas.DrawRect(body, _fill);

        // Hip-style top: lighter inset panel.
        float inset = MathF.Min(hd, hw) * 0.45f;
        _fill.Color = Dim(new SKColor(127, 119, 107), ambient);
        canvas.DrawRect(new SKRect(-hd + inset, -hw + inset, hd - inset, hw - inset), _fill);

        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW;
        canvas.DrawRect(body, _stroke);
    }

    /// <summary>Parking (all ground, nothing in the structure pass): unroofed dark asphalt
    /// lot with a white perimeter line and two banks of stall dividers along the long axis.</summary>
    private void DrawParkingGround(SKCanvas canvas, in BuildingFootprint b, float ambient, float outlineW)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;

        var body = new SKRect(-hd, -hw, hd, hw);
        _fill.Color = Dim(new SKColor(58, 60, 64), ambient);
        canvas.DrawRect(body, _fill);
        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW;
        canvas.DrawRect(body, _stroke);

        // White perimeter line inset from the lot edge.
        _stroke.Color = new SKColor(255, 255, 255, (byte)(170 * ambient));
        _stroke.StrokeWidth = 0.25f;
        canvas.DrawRect(new SKRect(-hd + 0.6f, -hw + 0.6f, hd - 0.6f, hw - 0.6f), _stroke);

        // Stall dividers: two banks at either end of the short axis.
        _line.Color = new SKColor(255, 255, 255, (byte)(130 * ambient));
        _line.StrokeWidth = 0.2f;
        if (hw >= hd)
        {
            // Lot is wide: stalls run along X, spaced along Y.
            float stallLen = hd * 0.55f;
            for (float y = -hw + 2f; y <= hw - 2f; y += 2.7f)
            {
                canvas.DrawLine(-hd + 0.6f, y, -hd + 0.6f + stallLen, y, _line);
                canvas.DrawLine(hd - 0.6f - stallLen, y, hd - 0.6f, y, _line);
            }
        }
        else
        {
            // Lot is deep: stalls run along Y, spaced along X.
            float stallLen = hw * 0.55f;
            for (float x = -hd + 2f; x <= hd - 2f; x += 2.7f)
            {
                canvas.DrawLine(x, -hw + 0.6f, x, -hw + 0.6f + stallLen, _line);
                canvas.DrawLine(x, hw - 0.6f - stallLen, x, hw - 0.6f, _line);
            }
        }
    }

    // ── Shared pieces ────────────────────────────────────────────────────

    /// <summary>
    /// Two-tone gable roof in the local frame: ridge along the long axis, halves in the
    /// palette's light/dark tones with the lighter half toward world −Y (fake sun), thin
    /// ridge highlight, dark outline.
    /// </summary>
    private void DrawGableRoof(SKCanvas canvas, float hd, float hw, Vector2 axis,
        SKColor light, SKColor dark, float ambient, float outlineW)
    {
        var l = Dim(light, ambient);
        var d = Dim(dark, ambient);
        var ridge = Dim(Lighten(light, 1.12f), ambient);

        if (hw >= hd)
        {
            // Ridge along the lateral (local Y) axis; halves split across local X.
            // World Y of the +X half's center is axis.Y * hd/2 above the center.
            bool posXLighter = axis.Y < 0f;
            _fill.Color = posXLighter ? d : l;
            canvas.DrawRect(new SKRect(-hd, -hw, 0f, hw), _fill);
            _fill.Color = posXLighter ? l : d;
            canvas.DrawRect(new SKRect(0f, -hw, hd, hw), _fill);
            _line.Color = ridge;
            _line.StrokeWidth = 0.25f;
            canvas.DrawLine(0f, -hw, 0f, hw, _line);
        }
        else
        {
            // Ridge along the depth (local X) axis; halves split across local Y
            // (lateral world Y component is axis.X).
            bool posYLighter = axis.X < 0f;
            _fill.Color = posYLighter ? d : l;
            canvas.DrawRect(new SKRect(-hd, -hw, hd, 0f), _fill);
            _fill.Color = posYLighter ? l : d;
            canvas.DrawRect(new SKRect(-hd, 0f, hd, hw), _fill);
            _line.Color = ridge;
            _line.StrokeWidth = 0.25f;
            canvas.DrawLine(-hd, 0f, hd, 0f, _line);
        }

        _stroke.Color = OutlineColor;
        _stroke.StrokeWidth = outlineW;
        canvas.DrawRect(new SKRect(-hd, -hw, hd, hw), _stroke);
    }

    /// <summary>Warm emissive window rects along the facade (never ambient-dimmed);
    /// alpha scales with the night factor. Local frame.</summary>
    private void DrawWindows(SKCanvas canvas, in BuildingFootprint b, float nightFactor)
    {
        float hd = b.HalfDepth, hw = b.HalfWidth;
        int count = 2 + (int)(Hash(b.Seed, 60) % 3u);
        _fill.Color = WindowColor.WithAlpha((byte)(235 * nightFactor));
        for (int k = 0; k < count; k++)
        {
            float y = count == 1 ? 0f : -hw * 0.75f + (2f * hw * 0.75f) * (k / (float)(count - 1));
            canvas.DrawRect(new SKRect(hd - 1.6f, y - 0.7f, hd - 0.4f, y + 0.7f), _fill);
        }
    }

    /// <summary>Faint warm night halo (single low-alpha circle) around Homes and Shops.
    /// Drawn in world space under the building body.</summary>
    private void DrawHalo(SKCanvas canvas, in BuildingFootprint b, float nightFactor)
    {
        float radius = MathF.Max(b.HalfWidth, b.HalfDepth) * 2.2f;
        _glow.Color = HaloColor.WithAlpha((byte)(20 * nightFactor));
        canvas.DrawCircle(b.Center.X, b.Center.Y, radius, _glow);
    }

    // ── Colors ───────────────────────────────────────────────────────────

    /// <summary>Muted wall color per POI type for the mid-zoom flat tier.</summary>
    private static SKColor WallColor(POIType type) => type switch
    {
        POIType.Home    => new SKColor(150, 140, 126),
        POIType.Work    => new SKColor(128, 126, 122),
        POIType.Shop    => new SKColor(134, 130, 124),
        POIType.Leisure => new SKColor(146, 120, 98),
        POIType.School  => new SKColor(140, 132, 118),
        POIType.Parking => new SKColor(58, 60, 64),
        _               => new SKColor(130, 128, 124),
    };

    /// <summary>POI palette color (UiTheme.PoiColors) with a red fallback for None.</summary>
    private static SKColor PoiColor(POIType type) => Ui.UiTheme.PoiColor(type);

    private static SKColor Dim(SKColor c, float ambient) => new(
        (byte)Math.Clamp((int)(c.Red * ambient), 0, 255),
        (byte)Math.Clamp((int)(c.Green * ambient), 0, 255),
        (byte)Math.Clamp((int)(c.Blue * ambient), 0, 255),
        c.Alpha);

    private static SKColor Blend(SKColor a, SKColor b, float k) => new(
        (byte)(a.Red + (b.Red - a.Red) * k),
        (byte)(a.Green + (b.Green - a.Green) * k),
        (byte)(a.Blue + (b.Blue - a.Blue) * k),
        a.Alpha);

    private static SKColor Lighten(SKColor c, float f) => new(
        (byte)Math.Clamp((int)(c.Red * f), 0, 255),
        (byte)Math.Clamp((int)(c.Green * f), 0, 255),
        (byte)Math.Clamp((int)(c.Blue * f), 0, 255),
        c.Alpha);

    private static uint Hash(uint x)
    {
        unchecked
        {
            x ^= x >> 16; x *= 0x7feb352d;
            x ^= x >> 15; x *= 0x846ca68b;
            x ^= x >> 16;
            return x;
        }
    }

    private static uint Hash(uint a, uint b) { unchecked { return Hash(a * 0x9E3779B9u ^ b); } }

    private static float Hash01(uint a, uint b) => Hash(a, b) * (1f / 4294967296f);
}
