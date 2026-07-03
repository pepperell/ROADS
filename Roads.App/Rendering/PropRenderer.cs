using System.Numerics;
using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Procedurally places and renders decorative props — street lights (emissive night glow),
/// trees, and bushes — around the road network. Placement is fully deterministic: every
/// candidate is seeded by a hash of its quantized world position (0.5 m grid), so props do
/// not reshuffle when an unrelated edge split renumbers edges. Every prop is rejected if it
/// encroaches on ANY road within range — distance to each nearby edge's centerline must be
/// at least that edge's visual half-width (<see cref="GeometryUtil.RoadHalfWidth"/> ×
/// <see cref="RoadTypeVisuals.GetWidthMultiplier"/>) plus 1 m — or if it falls inside any
/// building AABB inflated by 1.5 m (buildings are hashed into a 64 m cell grid once per
/// rebuild). Total props are capped at ~20,000: when exceeded, all spacings are doubled and
/// placement runs once more (graceful density degradation).
/// Call order: <see cref="Rebuild"/> must be called before <see cref="Draw"/> each frame; it
/// early-outs in O(1) while the graph version and building count are unchanged, and calls
/// <see cref="EdgeSpatialGrid.RebuildIfNeeded"/> itself so no external ordering is required
/// for the grid. The caller may defer Rebuild during continuous edits (stale props draw
/// safely — positions are baked, nothing dereferences the graph at draw time). Rendering is
/// strictly read-only over sim state, draws nothing below zoom 0.5, culls by 64 m bucket,
/// and reuses all paints (no per-frame allocation; vegetation colors are baked at rebuild).
/// Vegetation and pole colors are ambient-dimmed at night (ambient = 1 − darkness × 0.45);
/// lit lamps and their glow are emissive and never dimmed.
/// </summary>
public sealed class PropRenderer
{
    /// <summary>Prop kind stored in <see cref="Prop.Kind"/>: tree canopy.</summary>
    private const byte KindTree = 0;
    /// <summary>Prop kind stored in <see cref="Prop.Kind"/>: bush (smaller, yellower canopy).</summary>
    private const byte KindBush = 1;
    /// <summary>Prop kind stored in <see cref="Prop.Kind"/>: street light.</summary>
    private const byte KindLight = 2;

    /// <summary>Side length (meters) of the draw-culling / building-hash cells.</summary>
    private const float BucketCell = 64f;
    /// <summary>Soft cap on total placed props; exceeding it doubles all spacings once.</summary>
    private const int MaxProps = 20000;
    /// <summary>
    /// Half-extent (meters) of the road-clearance query box around a candidate: the widest
    /// possible visual half-width (4-lane two-way highway ≈ 17.5 m) + 1 m clearance + margin.
    /// Any edge whose clearance envelope could contain the candidate is inside this box.
    /// </summary>
    private const float MaxClearance = 20f;
    /// <summary>Clearance slack (meters) for street lights, which sit exactly at the required
    /// clearance from their own road and must not self-reject on sampling error.</summary>
    private const float LightRoadSlack = 0.35f;
    /// <summary>Clearance slack (meters) for vegetation (tolerates fine-sample chord error only).</summary>
    private const float VegRoadSlack = 0.05f;
    /// <summary>Extra world-space margin (meters) when culling draw buckets/props — covers the
    /// largest visual extent (9 m lamp glow, 5 m canopy + shadow offset).</summary>
    private const float CullMargin = 10f;

    /// <summary>Street light spacing (meters of arc length) on Arterial/Highway edges.</summary>
    private const float MajorLightSpacing = 45f;
    /// <summary>Street light spacing (meters of arc length) on Residential edges.</summary>
    private const float ResidentialLightSpacing = 85f;
    /// <summary>Roadside tree spacing (meters of arc length) on Residential edges.</summary>
    private const float RoadsideTreeSpacing = 30f;
    /// <summary>Side length (meters) of the open-field cluster cells.</summary>
    private const float FieldCellSize = 128f;

    /// <summary>A single placed prop, baked to a world position at rebuild time.</summary>
    private struct Prop
    {
        /// <summary>One of <see cref="KindTree"/>, <see cref="KindBush"/>, <see cref="KindLight"/>.</summary>
        public byte Kind;
        /// <summary>World-space position (meters).</summary>
        public Vector2 Pos;
        /// <summary>Canopy radius for vegetation; pole radius for lights.</summary>
        public float Size;
        /// <summary>Deterministic per-prop seed hashed from the quantized world position.</summary>
        public uint Seed;
        /// <summary>For lights: world angle (radians) from the pole toward the road center
        /// (the lamp arm direction). Unused for vegetation.</summary>
        public float SideAngle;
        /// <summary>Vegetation base color (pre-dim), baked at rebuild so drawing does no hashing.</summary>
        public byte ColR, ColG, ColB;
    }

    /// <summary>All placed props (cleared and refilled on rebuild; capacity is reused).</summary>
    private readonly List<Prop> _props = new();
    /// <summary>Prop indices bucketed by 64 m cell for draw culling.</summary>
    private readonly Dictionary<long, List<int>> _buckets = new();
    /// <summary>Building indices bucketed by 64 m cell for point-in-building rejection.</summary>
    private readonly Dictionary<long, List<int>> _buildingCells = new();
    /// <summary>Building AABBs inflated by 1.5 m, index-aligned with <see cref="_buildingCells"/> entries.</summary>
    private readonly List<SKRect> _inflatedBuildings = new();
    /// <summary>Pool of bucket lists so rebuilds do not re-allocate collections.</summary>
    private readonly Stack<List<int>> _listPool = new();

    /// <summary>Reusable edge-index buffer for road-clearance queries (no per-candidate allocation).</summary>
    private readonly List<int> _clearanceEdges = new();

    /// <summary>Graph version at the last rebuild (early-out key, paired with the building count).</summary>
    private int _cachedVersion = -1;
    /// <summary>Building count at the last rebuild (early-out key, paired with the graph version).</summary>
    private int _cachedBuildingCount = -1;

    // Reusable paints — colors are set per draw call; no per-frame allocation.
    private readonly SKPaint _shadowPaint = new()
    {
        Color = new SKColor(0, 0, 0, 55),
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };
    private readonly SKPaint _fillPaint = new()
    {
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };
    private readonly SKPaint _armPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeWidth = 0.16f,
        StrokeCap = SKStrokeCap.Round,
        IsAntialias = true
    };
    private readonly SKPaint _headPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeWidth = 0.5f,
        StrokeCap = SKStrokeCap.Butt,
        IsAntialias = true
    };
    private readonly SKPaint _glowPaint = new()
    {
        Style = SKPaintStyle.Fill,
        IsAntialias = true
    };

    /// <summary>
    /// Rebuilds prop placement when the graph or building set changes; O(1) early-out when
    /// <paramref name="graphVersion"/> equals the cached version AND
    /// <paramref name="buildingBounds"/>.Count equals the cached count. Call once per frame
    /// before <see cref="Draw"/>. <paramref name="buildingBounds"/> are rotation-inclusive
    /// world AABBs from the building layer; each is inflated by 1.5 m for rejection.
    /// </summary>
    public void Rebuild(RoadGraph graph, EdgeSpatialGrid edgeGrid,
        IReadOnlyList<SKRect> buildingBounds, int graphVersion)
    {
        if (graphVersion == _cachedVersion && buildingBounds.Count == _cachedBuildingCount)
            return;
        _cachedVersion = graphVersion;
        _cachedBuildingCount = buildingBounds.Count;

        edgeGrid.RebuildIfNeeded(graph);
        BuildBuildingHash(buildingBounds);

        BuildProps(graph, edgeGrid, 1f);
        if (_props.Count > MaxProps)
            BuildProps(graph, edgeGrid, 2f); // degrade gracefully: double all spacings, rebuild once
    }

    /// <summary>
    /// Draws all props inside <paramref name="viewRect"/> (64 m bucket culling; an empty rect
    /// disables culling). Nothing is drawn below zoom 0.5 (props are sub-pixel there and the
    /// whole map can be in view). Vegetation is drawn first, then street lights, so night
    /// glow renders above canopies. LOD: zoom 0.5–0.8 draws flat non-antialiased canopy
    /// circles (day lights skipped entirely; at night only the glow); ≥ 0.8 adds antialiasing,
    /// shadows, canopy highlights, poles and caps; ≥ 1.5 adds trunks, lamp arms, and lamp
    /// heads. At darkness &gt; 0.25 lamps gain an emissive warm dot plus a two-ring glow
    /// scaled by darkness (never ambient-dimmed).
    /// </summary>
    public void Draw(SKCanvas canvas, SKRect viewRect, float zoom, float darkness)
    {
        if (zoom < 0.5f || _props.Count == 0) return;

        float ambient = 1f - darkness * 0.45f;
        bool night = darkness > 0.25f;
        bool cull = viewRect.Width > 0f || viewRect.Height > 0f;

        // Antialiasing only at near zoom: in the 0.5-0.8 overview band props are a few
        // pixels wide and AA fills would dominate the frame cost at city scale.
        bool fine = zoom >= 0.8f;
        _fillPaint.IsAntialias = fine;
        _shadowPaint.IsAntialias = fine;

        // Day + far zoom: lights have no visible structure (pole dots are sub-pixel), so the
        // light pass is skipped entirely unless it is night (glow) or near zoom.
        int passCount = (fine || night) ? 2 : 1;

        // Pass 0 = vegetation, pass 1 = street lights (glow must draw over canopies).
        for (int pass = 0; pass < passCount; pass++)
        {
            if (cull)
            {
                int cx0 = (int)MathF.Floor((viewRect.Left - CullMargin) / BucketCell);
                int cx1 = (int)MathF.Floor((viewRect.Right + CullMargin) / BucketCell);
                int cy0 = (int)MathF.Floor((viewRect.Top - CullMargin) / BucketCell);
                int cy1 = (int)MathF.Floor((viewRect.Bottom + CullMargin) / BucketCell);
                for (int cx = cx0; cx <= cx1; cx++)
                for (int cy = cy0; cy <= cy1; cy++)
                {
                    if (!_buckets.TryGetValue(CellKey(cx, cy), out var list)) continue;
                    DrawBucket(canvas, list, pass, viewRect, cull: true, zoom, darkness, ambient, night);
                }
            }
            else
            {
                foreach (var list in _buckets.Values)
                    DrawBucket(canvas, list, pass, viewRect, cull: false, zoom, darkness, ambient, night);
            }
        }
    }

    // ── Drawing ─────────────────────────────────────────────────────────

    /// <summary>Draws the props of one 64 m bucket matching the given pass (0 = vegetation,
    /// 1 = lights), with a per-prop refinement cull against the view rect.</summary>
    private void DrawBucket(SKCanvas canvas, List<int> indices, int pass, SKRect viewRect,
        bool cull, float zoom, float darkness, float ambient, bool night)
    {
        for (int k = 0; k < indices.Count; k++)
        {
            var p = _props[indices[k]];
            bool isLight = p.Kind == KindLight;
            if (isLight != (pass == 1)) continue;
            if (cull && (p.Pos.X < viewRect.Left - CullMargin || p.Pos.X > viewRect.Right + CullMargin
                      || p.Pos.Y < viewRect.Top - CullMargin || p.Pos.Y > viewRect.Bottom + CullMargin))
                continue;

            if (isLight) DrawLight(canvas, in p, zoom, darkness, ambient, night);
            else DrawVegetation(canvas, in p, zoom, ambient);
        }
    }

    /// <summary>
    /// Draws a tree or bush: offset shadow ellipse + flat two-tone canopy (base circle plus a
    /// lighter highlight circle shifted up-left at zoom ≥ 0.8; single flat circle below), and
    /// a small trunk dot at zoom ≥ 1.5 (trees only). The canopy base color (±10 per channel
    /// position-hash variation) is baked into the prop at rebuild; all colors are ambient-dimmed.
    /// </summary>
    private void DrawVegetation(SKCanvas canvas, in Prop p, float zoom, float ambient)
    {
        float r = p.Size;

        if (zoom >= 0.8f)
            canvas.DrawOval(p.Pos.X + 0.9f, p.Pos.Y + 0.9f, r, r * 0.9f, _shadowPaint);

        _fillPaint.Color = new SKColor(Dim(p.ColR, ambient), Dim(p.ColG, ambient), Dim(p.ColB, ambient));
        canvas.DrawCircle(p.Pos.X, p.Pos.Y, r, _fillPaint);

        if (zoom >= 0.8f)
        {
            // Two-tone highlight: base (58,84,48) → (74,100,58), same delta applied to bushes.
            _fillPaint.Color = new SKColor(Dim(p.ColR + 16, ambient), Dim(p.ColG + 16, ambient), Dim(p.ColB + 10, ambient));
            canvas.DrawCircle(p.Pos.X - 0.25f * r, p.Pos.Y - 0.25f * r, r * 0.6f, _fillPaint);
        }

        if (p.Kind != KindBush && zoom >= 1.5f)
        {
            _fillPaint.Color = new SKColor(Dim(74, ambient), Dim(58, ambient), Dim(40, ambient));
            canvas.DrawCircle(p.Pos.X, p.Pos.Y, 0.3f, _fillPaint);
        }
    }

    /// <summary>
    /// Draws a street light: pole dot (screen-constant minimum size at far zoom), lighter cap
    /// at zoom ≥ 0.8, and at zoom ≥ 1.5 a 2.2 m arm toward the road (stored
    /// <see cref="Prop.SideAngle"/>) ending in a 0.5 × 0.9 m lamp head. At night
    /// (darkness &gt; 0.25) the lamp head gains an emissive warm dot and two concentric glow
    /// circles (4 m / 9 m, alpha scaled by darkness) that are never ambient-dimmed.
    /// </summary>
    private void DrawLight(SKCanvas canvas, in Prop p, float zoom, float darkness, float ambient, bool night)
    {
        float x = p.Pos.X, y = p.Pos.Y;
        float dirX = MathF.Cos(p.SideAngle), dirY = MathF.Sin(p.SideAngle);
        float headX = x + dirX * 2.2f, headY = y + dirY * 2.2f;

        // Structural geometry only at near zoom; in the 0.5-0.8 band lights render at night
        // only (glow + warm dot below), and the pass is skipped entirely during the day.
        if (zoom >= 0.8f)
        {
            _fillPaint.Color = new SKColor(Dim(58, ambient), Dim(60, ambient), Dim(64, ambient));
            canvas.DrawCircle(x, y, p.Size, _fillPaint);
            _fillPaint.Color = new SKColor(Dim(96, ambient), Dim(98, ambient), Dim(104, ambient));
            canvas.DrawCircle(x, y, 0.2f, _fillPaint);
        }

        if (zoom >= 1.5f)
        {
            _armPaint.Color = new SKColor(Dim(58, ambient), Dim(60, ambient), Dim(64, ambient));
            canvas.DrawLine(x, y, headX, headY, _armPaint);
            // Lamp head: 0.9 m long, 0.5 m wide (butt-capped stroke centered on the arm end).
            _headPaint.Color = new SKColor(Dim(48, ambient), Dim(50, ambient), Dim(54, ambient));
            canvas.DrawLine(headX - dirX * 0.45f, headY - dirY * 0.45f,
                headX + dirX * 0.45f, headY + dirY * 0.45f, _headPaint);
        }

        if (night)
        {
            // Emissive: not ambient-dimmed. Outer ring first so the inner ring stacks on it.
            _glowPaint.Color = new SKColor(255, 220, 150, (byte)(16f * darkness));
            canvas.DrawCircle(headX, headY, 9f, _glowPaint);
            _glowPaint.Color = new SKColor(255, 220, 150, (byte)(45f * darkness));
            canvas.DrawCircle(headX, headY, 4f, _glowPaint);
            _fillPaint.Color = new SKColor(255, 220, 150, 220);
            canvas.DrawCircle(headX, headY, 0.45f, _fillPaint);
        }
    }

    // ── Placement ───────────────────────────────────────────────────────

    /// <summary>Clears and refills the prop list and draw buckets at the given spacing scale
    /// (1 = normal density, 2 = degraded after exceeding the prop cap).</summary>
    private void BuildProps(RoadGraph graph, EdgeSpatialGrid edgeGrid, float scale)
    {
        _props.Clear();
        ReturnLists(_buckets);

        PlaceStreetLights(graph, edgeGrid, scale);
        PlaceRoadsideTrees(graph, edgeGrid, scale);
        PlaceFieldClusters(graph, edgeGrid, scale);
        PlacePoiAccents(graph, edgeGrid, scale);
    }

    /// <summary>
    /// Places street lights along paved, non-shared-lane edges (reverse pairs deduped):
    /// Arterial/Highway every ~45 m alternating sides, Residential every ~85 m right side
    /// only, t clamped to [0.08, 0.92]. Poles sit at visual half-width + 1 m; the stored
    /// side angle points from the pole toward the road for the lamp arm.
    /// </summary>
    private void PlaceStreetLights(RoadGraph graph, EdgeSpatialGrid edgeGrid, float scale)
    {
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.RoadType == RoadType.Dirt) continue;
            if ((edge.Flags & EdgeFlags.SharedLane) != 0) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            bool major = edge.RoadType == RoadType.Arterial || edge.RoadType == RoadType.Highway;
            float spacing = (major ? MajorLightSpacing : ResidentialLightSpacing) * scale;
            float len = edge.Length;
            if (len <= 0f) continue;
            float clearance = GeometryUtil.RoadHalfWidth(graph, i)
                * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + 1.0f;

            int count = (int)(len * 0.84f / spacing);
            for (int k = 0; k < count; k++)
            {
                float t = 0.08f + (k + 0.5f) * spacing / len;
                if (t > 0.92f) break;
                float side = major ? ((k & 1) == 0 ? 1f : -1f) : 1f;

                var pos = GeometryUtil.OffsetRight(graph, i, t, side * clearance);
                if (InsideBuilding(pos.X, pos.Y)) continue;
                if (TooCloseToRoad(graph, edgeGrid, pos, LightRoadSlack)) continue;

                var tan = graph.EvaluateBezierTangent(i, t);
                float tl = tan.Length();
                if (tl < 0.001f) continue;
                // Right normal (Y-down): (-ty, tx). Arm points back toward the road center.
                float dirX = -side * (-tan.Y / tl);
                float dirY = -side * (tan.X / tl);
                AddProp(KindLight, pos, 0.35f, MathF.Atan2(dirY, dirX));
            }
        }
    }

    /// <summary>
    /// Places roadside trees along Residential edges (reverse pairs deduped) every ~30 m with
    /// ±6 m arc jitter, on a hash-chosen side at visual half-width + 4.5 m, t clamped to
    /// [0.15, 0.85], canopy radius 2.5–4.5 m.
    /// </summary>
    private void PlaceRoadsideTrees(RoadGraph graph, EdgeSpatialGrid edgeGrid, float scale)
    {
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var edge = graph.Edges[i];
            if (edge.FromNode < 0) continue;
            if (edge.RoadType != RoadType.Residential) continue;
            int reverse = graph.FindReverseEdge(i);
            if (reverse >= 0 && reverse < i) continue;

            float spacing = RoadsideTreeSpacing * scale;
            float len = edge.Length;
            if (len <= 0f) continue;
            float offset = GeometryUtil.RoadHalfWidth(graph, i)
                * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + 4.5f;

            int count = (int)(len * 0.7f / spacing);
            for (int k = 0; k < count; k++)
            {
                float baseDist = (k + 0.5f) * spacing;
                float tBase = 0.15f + baseDist / len;
                if (tBase > 0.85f) break;

                // All per-candidate decisions hash off the quantized base sample position, so
                // they survive edge renumbering (position-anchored, not index-anchored).
                uint seed = HashPos(graph.EvaluateBezier(i, tBase));
                float jitter = (Hash01(seed, 11) - 0.5f) * 12f; // ±6 m
                float t = Math.Clamp(0.15f + (baseDist + jitter) / len, 0.15f, 0.85f);
                float side = Hash01(seed, 12) < 0.5f ? -1f : 1f;

                var pos = GeometryUtil.OffsetRight(graph, i, t, side * offset);
                if (InsideBuilding(pos.X, pos.Y)) continue;
                if (TooCloseToRoad(graph, edgeGrid, pos, VegRoadSlack)) continue;

                float size = 2.5f + Hash01(seed, 13) * 2.0f;
                AddProp(KindTree, pos, size, 0f);
            }
        }
    }

    /// <summary>
    /// Scatters open-field tree clusters: iterates 128 m cells (world-origin aligned, so
    /// clusters never move with graph edits) covering the active-node AABB inflated by 150 m.
    /// Each cell hashes to 0–2 clusters of 2–6 trees within 25 m of a hashed center, canopy
    /// radius 2.5–5 m, subject to the standard road/building rejection.
    /// </summary>
    private void PlaceFieldClusters(RoadGraph graph, EdgeSpatialGrid edgeGrid, float scale)
    {
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        bool any = false;
        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            any = true;
            if (node.Position.X < minX) minX = node.Position.X;
            if (node.Position.X > maxX) maxX = node.Position.X;
            if (node.Position.Y < minY) minY = node.Position.Y;
            if (node.Position.Y > maxY) maxY = node.Position.Y;
        }
        if (!any) return;
        minX -= 150f; minY -= 150f; maxX += 150f; maxY += 150f;

        float cell = FieldCellSize * scale;
        int cx0 = (int)MathF.Floor(minX / cell);
        int cx1 = (int)MathF.Floor(maxX / cell);
        int cy0 = (int)MathF.Floor(minY / cell);
        int cy1 = (int)MathF.Floor(maxY / cell);

        for (int cx = cx0; cx <= cx1; cx++)
        for (int cy = cy0; cy <= cy1; cy++)
        {
            uint cellHash = Hash((uint)cx, (uint)cy);
            int clusters = (int)(cellHash % 3u); // 0–2 clusters per cell
            for (int c = 0; c < clusters; c++)
            {
                float centerX = (cx + Hash01(cellHash, (uint)(20 + c * 2))) * cell;
                float centerY = (cy + Hash01(cellHash, (uint)(21 + c * 2))) * cell;
                uint clusterSeed = HashPos(new Vector2(centerX, centerY));
                int trees = 2 + (int)(Hash01(clusterSeed, 1) * 5f); // 2–6

                for (int k = 0; k < trees; k++)
                {
                    float ang = Hash01(clusterSeed, (uint)(2 + k * 2)) * MathF.Tau;
                    float dist = MathF.Sqrt(Hash01(clusterSeed, (uint)(3 + k * 2))) * 25f;
                    var pos = new Vector2(centerX + MathF.Cos(ang) * dist,
                                          centerY + MathF.Sin(ang) * dist);
                    if (InsideBuilding(pos.X, pos.Y)) continue;
                    if (TooCloseToRoad(graph, edgeGrid, pos, VegRoadSlack)) continue;

                    float size = 2.5f + Hash01(HashPos(pos), 5) * 2.5f;
                    AddProp(KindTree, pos, size, 0f);
                }
            }
        }
    }

    /// <summary>
    /// Adds POI accents around Destination nodes with a Leisure or School point of interest:
    /// 4–7 extra trees scattered 10–20 m from the node; Leisure additionally gets 2–3 bushes
    /// (canopy radius 1.0–1.6 m) at 8–18 m. Stress-map artifacts (Destination with
    /// <see cref="POIType.None"/>) are excluded by the POI filter.
    /// </summary>
    private void PlacePoiAccents(RoadGraph graph, EdgeSpatialGrid edgeGrid, float scale)
    {
        for (int n = 0; n < graph.Nodes.Count; n++)
        {
            var node = graph.Nodes[n];
            if (float.IsNaN(node.Position.X)) continue;
            if ((node.Flags & NodeFlags.Destination) == 0) continue;
            if (node.PointOfInterest != POIType.Leisure && node.PointOfInterest != POIType.School)
                continue;

            uint seed = HashPos(node.Position);
            int trees = 4 + (int)(Hash01(seed, 31) * 4f); // 4–7
            if (scale > 1f) trees = Math.Max(2, trees / 2); // degraded density
            for (int k = 0; k < trees; k++)
            {
                float ang = Hash01(seed, (uint)(32 + k * 2)) * MathF.Tau;
                float dist = 10f + Hash01(seed, (uint)(33 + k * 2)) * 10f;
                var pos = new Vector2(node.Position.X + MathF.Cos(ang) * dist,
                                      node.Position.Y + MathF.Sin(ang) * dist);
                if (InsideBuilding(pos.X, pos.Y)) continue;
                if (TooCloseToRoad(graph, edgeGrid, pos, VegRoadSlack)) continue;

                float size = 2.5f + Hash01(HashPos(pos), 6) * 2.0f;
                AddProp(KindTree, pos, size, 0f);
            }

            if (node.PointOfInterest == POIType.Leisure)
            {
                int bushes = 2 + (int)(Hash01(seed, 60) * 2f); // 2–3
                for (int k = 0; k < bushes; k++)
                {
                    float ang = Hash01(seed, (uint)(61 + k * 2)) * MathF.Tau;
                    float dist = 8f + Hash01(seed, (uint)(62 + k * 2)) * 10f;
                    var pos = new Vector2(node.Position.X + MathF.Cos(ang) * dist,
                                          node.Position.Y + MathF.Sin(ang) * dist);
                    if (InsideBuilding(pos.X, pos.Y)) continue;
                    if (TooCloseToRoad(graph, edgeGrid, pos, VegRoadSlack)) continue;

                    float size = 1.0f + Hash01(HashPos(pos), 7) * 0.6f;
                    AddProp(KindBush, pos, size, 0f);
                }
            }
        }
    }

    // ── Rejection helpers ───────────────────────────────────────────────

    /// <summary>
    /// True when the position encroaches on ANY road within range: for every edge whose
    /// clearance envelope could contain the point (queried via the edge grid into a reusable
    /// buffer, reverse pairs deduped), the distance to that edge's centerline must be at least
    /// its visual half-width + 1 m clearance (minus <paramref name="slack"/>, which absorbs
    /// sampling error for props placed exactly at the clearance boundary). Checking every
    /// nearby edge — not just the nearest — is required because required clearance is
    /// per-edge: a point closer to a NARROW road's centerline can still sit inside a WIDE
    /// road's asphalt.
    /// </summary>
    private bool TooCloseToRoad(RoadGraph graph, EdgeSpatialGrid edgeGrid, Vector2 pos, float slack)
    {
        edgeGrid.QueryVisible(graph.Edges.Count,
            pos.X - MaxClearance, pos.Y - MaxClearance,
            pos.X + MaxClearance, pos.Y + MaxClearance, _clearanceEdges);

        for (int q = 0; q < _clearanceEdges.Count; q++)
        {
            int e = _clearanceEdges[q];
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            // Both directions of a pair share centerline geometry; the grid always returns
            // both together, so testing only the lower-index edge halves the sampling cost.
            int reverse = graph.FindReverseEdge(e);
            if (reverse >= 0 && reverse < e) continue;

            float required = GeometryUtil.RoadHalfWidth(graph, e)
                * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + 1.0f - slack;
            if (required <= 0f) continue;

            // Cheap reject: a cubic Bezier lies inside the convex hull of its four control
            // points, so a point farther than `required` from the hull AABB cannot violate.
            var p0 = graph.Nodes[edge.FromNode].Position;
            var p3 = graph.Nodes[edge.ToNode].Position;
            var c1 = edge.ControlPoint1;
            var c2 = edge.ControlPoint2;
            float minX = MathF.Min(MathF.Min(p0.X, p3.X), MathF.Min(c1.X, c2.X)) - required;
            float maxX = MathF.Max(MathF.Max(p0.X, p3.X), MathF.Max(c1.X, c2.X)) + required;
            float minY = MathF.Min(MathF.Min(p0.Y, p3.Y), MathF.Min(c1.Y, c2.Y)) - required;
            float maxY = MathF.Max(MathF.Max(p0.Y, p3.Y), MathF.Max(c1.Y, c2.Y)) + required;
            if (pos.X < minX || pos.X > maxX || pos.Y < minY || pos.Y > maxY) continue;

            if (DistanceToEdgeBelow(graph, e, edge.Length, pos, required)) return true;
        }
        return false;
    }

    /// <summary>
    /// True when the point's distance to the edge's centerline is below the threshold.
    /// Coarse scan (~4 m steps) followed by a local ±1-step refinement at 1/8 resolution
    /// keeps the measured-distance error under ~0.25 m (below the smallest slack in use).
    /// </summary>
    private static bool DistanceToEdgeBelow(RoadGraph graph, int edgeIdx, float length, Vector2 pos, float threshold)
    {
        int steps = Math.Clamp((int)(length / 4f), 4, 96);
        float bestT = 0f, bestD2 = float.MaxValue;
        for (int s = 0; s <= steps; s++)
        {
            float t = (float)s / steps;
            float d2 = Vector2.DistanceSquared(pos, graph.EvaluateBezier(edgeIdx, t));
            if (d2 < bestD2) { bestD2 = d2; bestT = t; }
        }

        float span = 1f / steps;
        for (int s = -7; s <= 7; s++)
        {
            if (s == 0) continue;
            float t = Math.Clamp(bestT + s * span / 8f, 0f, 1f);
            float d2 = Vector2.DistanceSquared(pos, graph.EvaluateBezier(edgeIdx, t));
            if (d2 < bestD2) bestD2 = d2;
        }
        return bestD2 < threshold * threshold;
    }

    /// <summary>Hashes the inflated building AABBs into 64 m cells (built once per rebuild).</summary>
    private void BuildBuildingHash(IReadOnlyList<SKRect> buildingBounds)
    {
        ReturnLists(_buildingCells);
        _inflatedBuildings.Clear();
        for (int i = 0; i < buildingBounds.Count; i++)
        {
            var r = buildingBounds[i];
            r.Inflate(1.5f, 1.5f);
            _inflatedBuildings.Add(r);

            int cx0 = (int)MathF.Floor(r.Left / BucketCell);
            int cx1 = (int)MathF.Floor(r.Right / BucketCell);
            int cy0 = (int)MathF.Floor(r.Top / BucketCell);
            int cy1 = (int)MathF.Floor(r.Bottom / BucketCell);
            for (int cx = cx0; cx <= cx1; cx++)
            for (int cy = cy0; cy <= cy1; cy++)
            {
                long key = CellKey(cx, cy);
                if (!_buildingCells.TryGetValue(key, out var list))
                {
                    list = RentList();
                    _buildingCells[key] = list;
                }
                list.Add(i);
            }
        }
    }

    /// <summary>True when the point lies inside any building AABB (already inflated by 1.5 m).</summary>
    private bool InsideBuilding(float x, float y)
    {
        long key = CellKey((int)MathF.Floor(x / BucketCell), (int)MathF.Floor(y / BucketCell));
        if (!_buildingCells.TryGetValue(key, out var list)) return false;
        for (int k = 0; k < list.Count; k++)
        {
            var r = _inflatedBuildings[list[k]];
            if (x >= r.Left && x <= r.Right && y >= r.Top && y <= r.Bottom) return true;
        }
        return false;
    }

    // ── Storage helpers ─────────────────────────────────────────────────

    /// <summary>Appends a prop (seeded from its quantized final position, vegetation color
    /// baked from that seed) and buckets it by 64 m cell.</summary>
    private void AddProp(byte kind, Vector2 pos, float size, float sideAngle)
    {
        uint seed = HashPos(pos);
        byte cr = 0, cg = 0, cb = 0;
        if (kind != KindLight)
        {
            bool bush = kind == KindBush;
            int vr = (int)(Hash01(seed, 101) * 21f) - 10;
            int vg = (int)(Hash01(seed, 102) * 21f) - 10;
            int vb = (int)(Hash01(seed, 103) * 21f) - 10;
            cr = (byte)Math.Clamp((bush ? 72 : 58) + vr, 0, 255);
            cg = (byte)Math.Clamp((bush ? 96 : 84) + vg, 0, 255);
            cb = (byte)Math.Clamp((bush ? 52 : 48) + vb, 0, 255);
        }

        int index = _props.Count;
        _props.Add(new Prop
        {
            Kind = kind, Pos = pos, Size = size, Seed = seed, SideAngle = sideAngle,
            ColR = cr, ColG = cg, ColB = cb,
        });

        long key = CellKey((int)MathF.Floor(pos.X / BucketCell), (int)MathF.Floor(pos.Y / BucketCell));
        if (!_buckets.TryGetValue(key, out var list))
        {
            list = RentList();
            _buckets[key] = list;
        }
        list.Add(index);
    }

    /// <summary>Returns all of a bucket map's lists to the pool and clears the map.</summary>
    private void ReturnLists(Dictionary<long, List<int>> map)
    {
        foreach (var list in map.Values)
        {
            list.Clear();
            _listPool.Push(list);
        }
        map.Clear();
    }

    /// <summary>Takes a list from the pool, or allocates one when the pool is empty.</summary>
    private List<int> RentList() => _listPool.Count > 0 ? _listPool.Pop() : new List<int>();

    /// <summary>Packs signed cell coordinates into a collision-free 64-bit key.</summary>
    private static long CellKey(int cx, int cy) => ((long)cx << 32) | (uint)cy;

    /// <summary>Ambient-dims a color channel (multiplicative), clamped to byte range.</summary>
    private static byte Dim(int baseValue, float ambient)
        => (byte)Math.Clamp((int)(baseValue * ambient), 0, 255);

    // ── Deterministic hashing (never SimRandom/Random — visuals must not perturb the sim RNG) ──

    /// <summary>Seed hash of a world position quantized to a 0.5 m grid — stable across
    /// edge renumbering because it depends only on where the prop is.</summary>
    private static uint HashPos(Vector2 p)
        => Hash((uint)(int)MathF.Round(p.X * 2f), (uint)(int)MathF.Round(p.Y * 2f));

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
