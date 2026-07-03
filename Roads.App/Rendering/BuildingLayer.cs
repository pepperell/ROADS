using System.Numerics;
using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// One placed procedural building. World units are meters, Y-down. The building is an
/// oriented box (OBB): its local depth axis points along <see cref="FacingRadians"/>
/// (the direction the FRONT faces, i.e. toward the road / foot node) and its local
/// width axis is the right normal of that direction. <see cref="Seed"/> is a stable
/// per-building hash used for all visual variation (palettes, window counts, jitter).
/// </summary>
public readonly struct BuildingFootprint
{
    /// <summary>Index of the destination node this building belongs to (stable identity).</summary>
    public readonly int NodeIndex;
    /// <summary>POI type of the destination node (never None or EntryExit).</summary>
    public readonly POIType Type;
    /// <summary>World-space center of the footprint in meters.</summary>
    public readonly Vector2 Center;
    /// <summary>Half-extent across the facade (perpendicular to the facing direction), meters.</summary>
    public readonly float HalfWidth;
    /// <summary>Half-extent front-to-back (along the facing direction), meters.</summary>
    public readonly float HalfDepth;
    /// <summary>Y-down atan2 angle of the direction the front faces (toward the road/foot node).</summary>
    public readonly float FacingRadians;
    /// <summary>Deterministic per-building hash seed (from node index and POI type).</summary>
    public readonly uint Seed;

    public BuildingFootprint(int nodeIndex, POIType type, Vector2 center,
        float halfWidth, float halfDepth, float facingRadians, uint seed)
    {
        NodeIndex = nodeIndex;
        Type = type;
        Center = center;
        HalfWidth = halfWidth;
        HalfDepth = halfDepth;
        FacingRadians = facingRadians;
        Seed = seed;
    }
}

/// <summary>
/// Deterministic procedural building placement for destination POI nodes. For every
/// non-defunct node with <see cref="NodeFlags.Destination"/> and a real POI type
/// (not None, not EntryExit) it tries to place a non-overlapping oriented footprint
/// near the node: behind an off-road destination node (front toward its road foot),
/// or beside the road for a legacy on-road node (right side first, then left).
/// Candidates are searched first-fit in a fixed deterministic order — scale
/// {1.0, 0.85, 0.7, 0.55, 0.4} outermost (all positions are tried at full size before
/// shrinking), then setback {3, 7, 11} m, then lateral shift {0, +4, -4, +8, -8} m —
/// and a candidate is accepted only when it clears every nearby road surface
/// (sampled every ~2 m, clearance = visual road half-width + 1.5 m; the building's own
/// connector edges are exempt so a driveway may touch the front) and every previously
/// accepted footprint (OBB-vs-OBB SAT, prefiltered by a 64 m spatial hash).
/// If nothing fits at minimum scale the node gets NO footprint and the renderer falls
/// back to the classic destination dot. Schools additionally attempt a green field
/// beside the building; when placed it participates in collision and is folded into
/// that building's bounds.
///
/// Call order: <see cref="RebuildIfNeeded"/> must be called before reading
/// <see cref="Buildings"/> / <see cref="CollectBounds"/> or drawing each frame; it is
/// version-keyed on <see cref="RoadGraph.Version"/> and rebuilds lazily. Placement is
/// purely read-only over the graph and uses stateless hashing only (never SimRandom),
/// iterating nodes in index order, so results are reproducible for a given graph.
/// </summary>
public sealed class BuildingLayer
{
    /// <summary>Setback distances (m) from the node to the front face, tried in order.</summary>
    private static readonly float[] Setbacks = { 3f, 7f, 11f };
    /// <summary>Lateral shifts (m) perpendicular to facing, tried in order.</summary>
    private static readonly float[] Laterals = { 0f, 4f, -4f, 8f, -8f };
    /// <summary>Footprint scales tried in order (outermost loop — size is preserved first).</summary>
    private static readonly float[] Scales = { 1f, 0.85f, 0.7f, 0.55f, 0.4f };

    /// <summary>Extra clearance (m) required between a footprint and the road asphalt edge.</summary>
    private const float RoadClearance = 1.5f;
    /// <summary>Spacing (m) between road-surface collision samples along each edge Bézier.</summary>
    private const float RoadSampleSpacing = 2f;
    /// <summary>Cell size (m) of the accepted-footprint spatial hash used for OBB prefiltering.</summary>
    private const float AcceptCellSize = 64f;
    /// <summary>Margin (m) added to the per-node edge query / sample-keep rectangle. Covers the
    /// 16 m query inflation plus the largest road clearance (~14 m half-width × 1.25 + 1.5).</summary>
    private const float QueryMargin = 20f;

    /// <summary>An oriented box: Axis is the unit facing direction (local depth axis).</summary>
    private struct Obb
    {
        public Vector2 Center;
        public Vector2 Axis;
        public float HalfDepth;
        public float HalfWidth;
    }

    /// <summary>One road-surface collision sample: a point on an edge Bézier and the
    /// minimum distance any footprint must keep from it.</summary>
    private struct RoadSample
    {
        public Vector2 Pos;
        public float Clearance;
    }

    /// <summary>Optional green field attached to a School building (same axes as the building).</summary>
    private struct SchoolField
    {
        public bool Present;
        public Vector2 Center;
        public float HalfWidth;
        public float HalfDepth;
    }

    private readonly List<BuildingFootprint> _buildings = new();
    /// <summary>Rotation-inclusive world AABB per building, index-aligned with <see cref="_buildings"/>
    /// (includes the school field when present).</summary>
    private readonly List<SKRect> _bounds = new();
    /// <summary>Per-building school field record, index-aligned with <see cref="_buildings"/>.</summary>
    private readonly List<SchoolField> _schoolFields = new();
    /// <summary>Node indices that received a footprint (for the renderer's dot fallback test).</summary>
    private readonly HashSet<int> _footprintNodes = new();

    /// <summary>All accepted collision OBBs (buildings AND school fields).</summary>
    private readonly List<Obb> _accepted = new();
    /// <summary>64 m spatial hash: cell key → indices into <see cref="_accepted"/>.</summary>
    private readonly Dictionary<int, List<int>> _acceptCells = new();
    /// <summary>Per-query dedupe set for spatial-hash lookups (reused).</summary>
    private readonly HashSet<int> _acceptDedup = new();

    /// <summary>Reused edge-query result list.</summary>
    private readonly List<int> _edgeQuery = new();
    /// <summary>Reused per-node road collision samples.</summary>
    private readonly List<RoadSample> _roadSamples = new();

    /// <summary>Graph version at the last rebuild (-1 = never built).</summary>
    private int _cachedVersion = -1;

    /// <summary>Placed buildings in node-index order. Valid after <see cref="RebuildIfNeeded"/>.</summary>
    public IReadOnlyList<BuildingFootprint> Buildings => _buildings;

    /// <summary>
    /// Rebuilds all footprints if the graph changed since the last rebuild (version-keyed).
    /// Must be called once per frame before <see cref="Buildings"/>, <see cref="CollectBounds"/>,
    /// or <see cref="BuildingRenderer.Draw"/>. Also brings <paramref name="edgeGrid"/> up to
    /// date (its own rebuild is version-keyed and a no-op when current).
    /// </summary>
    public void RebuildIfNeeded(RoadGraph graph, EdgeSpatialGrid edgeGrid)
    {
        if (_cachedVersion == graph.Version) return;
        edgeGrid.RebuildIfNeeded(graph);
        Rebuild(graph, edgeGrid);
        _cachedVersion = graph.Version;
    }

    /// <summary>
    /// Fills <paramref name="results"/> (cleared first) with one rotation-inclusive world AABB
    /// per building, index-aligned with <see cref="Buildings"/>. A School's AABB includes its
    /// field when one was placed. Consumed by prop placement to keep props off buildings.
    /// </summary>
    public void CollectBounds(List<SKRect> results)
    {
        results.Clear();
        for (int i = 0; i < _bounds.Count; i++)
            results.Add(_bounds[i]);
    }

    /// <summary>Rotation-inclusive world AABB of one building (same rect CollectBounds emits).</summary>
    public SKRect GetBounds(int buildingIndex) => _bounds[buildingIndex];

    /// <summary>True when the node received a footprint in the last rebuild. Nodes without a
    /// footprint keep the classic destination dot at all zoom levels.</summary>
    public bool HasFootprint(int nodeIndex) => _footprintNodes.Contains(nodeIndex);

    /// <summary>
    /// School field placed beside building <paramref name="buildingIndex"/>, if any. The field
    /// shares the building's axes: <paramref name="halfWidth"/> is across the facade direction,
    /// <paramref name="halfDepth"/> along it.
    /// </summary>
    public bool TryGetSchoolField(int buildingIndex, out Vector2 center, out float halfWidth, out float halfDepth)
    {
        var f = _schoolFields[buildingIndex];
        center = f.Center;
        halfWidth = f.HalfWidth;
        halfDepth = f.HalfDepth;
        return f.Present;
    }

    /// <summary>
    /// Un-jittered (midpoint) footprint half-extents for a POI type, for the editor's
    /// placement ghost. Returns (across-facade, front-to-back) half sizes in meters.
    /// </summary>
    public static (float halfW, float halfD) GetDefaultFootprint(POIType type)
    {
        var (wMin, wMax, dMin, dMax) = GetExtentRange(type);
        return ((wMin + wMax) * 0.5f, (dMin + dMax) * 0.5f);
    }

    /// <summary>Half-extent ranges (m) per POI type: width across facade, depth front-to-back.</summary>
    private static (float wMin, float wMax, float dMin, float dMax) GetExtentRange(POIType type) => type switch
    {
        POIType.Home    => (5f, 7f, 4f, 6f),
        POIType.Work    => (10f, 16f, 8f, 14f),
        POIType.Shop    => (8f, 12f, 6f, 10f),
        POIType.Leisure => (8f, 14f, 6f, 10f),
        POIType.School  => (14f, 20f, 10f, 14f),
        POIType.Parking => (12f, 18f, 10f, 14f),
        _               => (5f, 7f, 4f, 6f),
    };

    /// <summary>Full placement pass: iterates nodes in index order and places footprints.</summary>
    private void Rebuild(RoadGraph graph, EdgeSpatialGrid edgeGrid)
    {
        _buildings.Clear();
        _bounds.Clear();
        _schoolFields.Clear();
        _footprintNodes.Clear();
        _accepted.Clear();
        foreach (var list in _acceptCells.Values)
            list.Clear();

        var nodes = graph.Nodes;
        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if ((node.Flags & NodeFlags.Destination) == 0) continue;
            var poi = node.PointOfInterest;
            if (poi == POIType.None || poi == POIType.EntryExit) continue;

            TryPlaceBuilding(graph, edgeGrid, i, node.Position, poi);
        }
    }

    /// <summary>
    /// Attempts to place one building for a destination node; appends to the building lists
    /// on success. Facing comes from the connector direction (off-road node with exactly one
    /// distinct neighbor) or from an incident edge's right normal (on-road node, right side
    /// tried before left).
    /// </summary>
    private void TryPlaceBuilding(RoadGraph graph, EdgeSpatialGrid edgeGrid,
        int nodeIndex, Vector2 nodePos, POIType poi)
    {
        uint seed = Hash((uint)nodeIndex, (uint)poi);
        var (wMin, wMax, dMin, dMax) = GetExtentRange(poi);
        float baseHalfW = wMin + (wMax - wMin) * Hash01(seed, 1);
        float baseHalfD = dMin + (dMax - dMin) * Hash01(seed, 2);

        // Facing candidates: one for an off-road node (front toward the foot), two for an
        // on-road node (building on the road's right side first, then the left side).
        Span<Vector2> facings = stackalloc Vector2[2];
        int facingCount = ComputeFacings(graph, nodeIndex, nodePos, seed, facings);
        if (facingCount == 0) return;
        bool offRoad = facingCount == 1;

        // Gather road collision samples once per node, over a rect covering every candidate
        // position (max setback + depth + lateral + width, plus the school field reach),
        // inflated by QueryMargin for the edge query and clearance.
        float reach = Setbacks[^1] + 2f * dMax + 8f + wMax + QueryMargin;
        if (poi == POIType.School) reach += 2f + 2f * 13f; // field lateral extent
        float minX = nodePos.X - reach, minY = nodePos.Y - reach;
        float maxX = nodePos.X + reach, maxY = nodePos.Y + reach;
        GatherRoadSamples(graph, edgeGrid, nodeIndex, offRoad, minX, minY, maxX, maxY);

        for (int f = 0; f < facingCount; f++)
        {
            var facing = facings[f];
            var lat = new Vector2(-facing.Y, facing.X);
            foreach (float scale in Scales)
            {
                float hw = baseHalfW * scale;
                float hd = baseHalfD * scale;
                foreach (float setback in Setbacks)
                {
                    foreach (float lateral in Laterals)
                    {
                        var obb = new Obb
                        {
                            Center = nodePos - facing * (setback + hd) + lat * lateral,
                            Axis = facing,
                            HalfDepth = hd,
                            HalfWidth = hw,
                        };
                        if (!Fits(in obb)) continue;

                        Accept(graph, nodeIndex, poi, seed, in obb);
                        return;
                    }
                }
            }
        }
        // Nothing fit at minimum scale: no footprint (renderer keeps the classic dot).
    }

    /// <summary>
    /// Computes candidate facing directions for a node into <paramref name="facings"/> and
    /// returns how many are valid (0 when no usable geometry exists). Exactly one distinct
    /// neighbor = off-road destination (facing toward that foot node, single candidate);
    /// otherwise on-road (right normal candidates of the first incident edge tangent).
    /// </summary>
    private static int ComputeFacings(RoadGraph graph, int nodeIndex, Vector2 nodePos,
        uint seed, Span<Vector2> facings)
    {
        // Count distinct neighbors across outgoing and incoming edges.
        int firstNbr = -1;
        bool multiple = false;
        foreach (int e in graph.GetOutgoingEdges(nodeIndex))
        {
            int to = graph.Edges[e].ToNode;
            if (to < 0) continue;
            if (firstNbr < 0) firstNbr = to;
            else if (to != firstNbr) { multiple = true; break; }
        }
        if (!multiple)
        {
            foreach (int e in graph.GetIncomingEdges(nodeIndex))
            {
                int from = graph.Edges[e].FromNode;
                if (from < 0) continue;
                if (firstNbr < 0) firstNbr = from;
                else if (from != firstNbr) { multiple = true; break; }
            }
        }
        if (firstNbr < 0) return 0; // isolated node: no roads, no placement

        if (!multiple)
        {
            // Off-road destination: front faces the road foot node.
            var dir = graph.Nodes[firstNbr].Position - nodePos;
            float len = dir.Length();
            if (len < 0.1f)
            {
                float a = Hash01(seed, 3) * (2f * MathF.PI);
                facings[0] = new Vector2(MathF.Cos(a), MathF.Sin(a));
            }
            else
            {
                facings[0] = dir / len;
            }
            return 1;
        }

        // On-road node: use the right normal of the first incident edge tangent.
        Vector2 tangent = default;
        bool found = false;
        foreach (int e in graph.GetOutgoingEdges(nodeIndex))
        {
            if (graph.Edges[e].FromNode < 0) continue;
            tangent = graph.EvaluateBezierTangent(e, 0f);
            found = true;
            break;
        }
        if (!found)
        {
            foreach (int e in graph.GetIncomingEdges(nodeIndex))
            {
                if (graph.Edges[e].FromNode < 0) continue;
                tangent = graph.EvaluateBezierTangent(e, 1f);
                found = true;
                break;
            }
        }
        float tLen = tangent.Length();
        if (!found || tLen < 0.001f) return 0;

        var right = new Vector2(-tangent.Y / tLen, tangent.X / tLen);
        // Building on the RIGHT side of the road faces back toward the node: facing = -right.
        facings[0] = -right;
        facings[1] = right;
        return 2;
    }

    /// <summary>
    /// Collects road-surface samples (every ~2 m along each nearby edge Bézier) into
    /// <see cref="_roadSamples"/>, restricted to the given rect. Each sample carries the
    /// clearance a footprint must keep: visual asphalt half-width + <see cref="RoadClearance"/>.
    /// For an off-road node the node's own connector edges are skipped (a driveway may touch
    /// the building front). Two-way pairs are sampled once (reverse edge skipped).
    /// </summary>
    private void GatherRoadSamples(RoadGraph graph, EdgeSpatialGrid edgeGrid, int nodeIndex,
        bool offRoad, float minX, float minY, float maxX, float maxY)
    {
        _roadSamples.Clear();
        edgeGrid.QueryVisible(graph.Edges.Count, minX, minY, maxX, maxY, _edgeQuery);

        for (int q = 0; q < _edgeQuery.Count; q++)
        {
            int e = _edgeQuery[q];
            var edge = graph.Edges[e];
            if (edge.FromNode < 0) continue;
            // Two-way pair: process only the lower-indexed edge (both share the asphalt).
            int reverse = graph.FindReverseEdge(e);
            if (reverse >= 0 && reverse < e) continue;
            // Own connector exemption (off-road destinations only).
            if (offRoad && (edge.FromNode == nodeIndex || edge.ToNode == nodeIndex)) continue;

            float clearance = GeometryUtil.RoadHalfWidth(graph, e)
                * RoadTypeVisuals.GetWidthMultiplier(edge.RoadType) + RoadClearance;

            // 2048 cap keeps ~2 m sampling up to ~4 km edges; beyond that the effective
            // clearance erodes by half the widened spacing (acceptable on such outliers).
            int steps = Math.Clamp((int)MathF.Ceiling(edge.Length / RoadSampleSpacing), 1, 2048);
            for (int s = 0; s <= steps; s++)
            {
                float t = s / (float)steps;
                var p = graph.EvaluateBezier(e, t);
                if (p.X < minX || p.X > maxX || p.Y < minY || p.Y > maxY) continue;
                _roadSamples.Add(new RoadSample { Pos = p, Clearance = clearance });
            }
        }
    }

    /// <summary>True when the candidate clears all gathered road samples and every
    /// previously accepted footprint/field.</summary>
    private bool Fits(in Obb obb)
    {
        for (int i = 0; i < _roadSamples.Count; i++)
        {
            var s = _roadSamples[i];
            if (DistSqPointToObb(s.Pos, in obb) < s.Clearance * s.Clearance)
                return false;
        }

        var aabb = ObbAabb(in obb);
        _acceptDedup.Clear();
        int cx0 = (int)MathF.Floor(aabb.Left / AcceptCellSize);
        int cx1 = (int)MathF.Floor(aabb.Right / AcceptCellSize);
        int cy0 = (int)MathF.Floor(aabb.Top / AcceptCellSize);
        int cy1 = (int)MathF.Floor(aabb.Bottom / AcceptCellSize);
        for (int cx = cx0; cx <= cx1; cx++)
        for (int cy = cy0; cy <= cy1; cy++)
        {
            if (!_acceptCells.TryGetValue(GeometryUtil.PackCell(cx, cy), out var list)) continue;
            for (int k = 0; k < list.Count; k++)
            {
                int idx = list[k];
                if (!_acceptDedup.Add(idx)) continue;
                var other = _accepted[idx];
                if (ObbsOverlap(in obb, in other)) return false;
            }
        }
        return true;
    }

    /// <summary>Records an accepted footprint: building list entry, bounds, collision OBB,
    /// footprint-node set, and (for Schools) an optional field beside the building.</summary>
    private void Accept(RoadGraph graph, int nodeIndex, POIType poi, uint seed, in Obb obb)
    {
        InsertAccepted(in obb);

        var bounds = ObbAabb(in obb);
        var field = default(SchoolField);
        if (poi == POIType.School)
        {
            if (TryPlaceSchoolField(seed, in obb, out var fieldObb))
            {
                InsertAccepted(in fieldObb);
                field = new SchoolField
                {
                    Present = true,
                    Center = fieldObb.Center,
                    HalfWidth = fieldObb.HalfWidth,
                    HalfDepth = fieldObb.HalfDepth,
                };
                var fieldAabb = ObbAabb(in fieldObb);
                bounds = new SKRect(
                    MathF.Min(bounds.Left, fieldAabb.Left),
                    MathF.Min(bounds.Top, fieldAabb.Top),
                    MathF.Max(bounds.Right, fieldAabb.Right),
                    MathF.Max(bounds.Bottom, fieldAabb.Bottom));
            }
        }

        _buildings.Add(new BuildingFootprint(nodeIndex, poi, obb.Center,
            obb.HalfWidth, obb.HalfDepth, MathF.Atan2(obb.Axis.Y, obb.Axis.X), seed));
        _bounds.Add(bounds);
        _schoolFields.Add(field);
        _footprintNodes.Add(nodeIndex);
    }

    /// <summary>
    /// Tries to place a school's green field beside the accepted building (same axes),
    /// first on the building's local-right side then the left. The field must clear roads
    /// and accepted footprints like any building; on failure no field is recorded.
    /// </summary>
    private bool TryPlaceSchoolField(uint seed, in Obb building, out Obb fieldObb)
    {
        float fieldHalfW = 9f + 4f * Hash01(seed, 4);
        float fieldHalfD = 6f + 3f * Hash01(seed, 5);
        var lat = new Vector2(-building.Axis.Y, building.Axis.X);
        for (int side = 0; side < 2; side++)
        {
            float sign = side == 0 ? 1f : -1f;
            fieldObb = new Obb
            {
                Center = building.Center + lat * (sign * (building.HalfWidth + fieldHalfW + 2f)),
                Axis = building.Axis,
                HalfDepth = fieldHalfD,
                HalfWidth = fieldHalfW,
            };
            if (Fits(in fieldObb)) return true;
        }
        fieldObb = default;
        return false;
    }

    /// <summary>Adds an OBB to the accepted collision set and its 64 m spatial hash.</summary>
    private void InsertAccepted(in Obb obb)
    {
        int idx = _accepted.Count;
        _accepted.Add(obb);
        var aabb = ObbAabb(in obb);
        int cx0 = (int)MathF.Floor(aabb.Left / AcceptCellSize);
        int cx1 = (int)MathF.Floor(aabb.Right / AcceptCellSize);
        int cy0 = (int)MathF.Floor(aabb.Top / AcceptCellSize);
        int cy1 = (int)MathF.Floor(aabb.Bottom / AcceptCellSize);
        for (int cx = cx0; cx <= cx1; cx++)
        for (int cy = cy0; cy <= cy1; cy++)
        {
            int key = GeometryUtil.PackCell(cx, cy);
            if (!_acceptCells.TryGetValue(key, out var list))
            {
                list = new List<int>(4);
                _acceptCells[key] = list;
            }
            list.Add(idx);
        }
    }

    /// <summary>Squared distance from a point to an OBB (0 when inside).</summary>
    private static float DistSqPointToObb(Vector2 p, in Obb o)
    {
        var d = p - o.Center;
        float along = d.X * o.Axis.X + d.Y * o.Axis.Y;
        float across = -d.X * o.Axis.Y + d.Y * o.Axis.X;
        float dx = MathF.Max(MathF.Abs(along) - o.HalfDepth, 0f);
        float dy = MathF.Max(MathF.Abs(across) - o.HalfWidth, 0f);
        return dx * dx + dy * dy;
    }

    /// <summary>OBB-vs-OBB overlap via separating-axis test on the four box axes.</summary>
    private static bool ObbsOverlap(in Obb a, in Obb b)
    {
        var aLat = new Vector2(-a.Axis.Y, a.Axis.X);
        var bLat = new Vector2(-b.Axis.Y, b.Axis.X);
        var d = b.Center - a.Center;

        Span<Vector2> axes = stackalloc Vector2[4];
        axes[0] = a.Axis;
        axes[1] = aLat;
        axes[2] = b.Axis;
        axes[3] = bLat;

        for (int i = 0; i < 4; i++)
        {
            var n = axes[i];
            float ra = a.HalfDepth * MathF.Abs(Vector2.Dot(a.Axis, n))
                     + a.HalfWidth * MathF.Abs(Vector2.Dot(aLat, n));
            float rb = b.HalfDepth * MathF.Abs(Vector2.Dot(b.Axis, n))
                     + b.HalfWidth * MathF.Abs(Vector2.Dot(bLat, n));
            if (MathF.Abs(Vector2.Dot(d, n)) > ra + rb) return false;
        }
        return true;
    }

    /// <summary>Rotation-inclusive world AABB of an OBB.</summary>
    private static SKRect ObbAabb(in Obb o)
    {
        float ex = MathF.Abs(o.Axis.X) * o.HalfDepth + MathF.Abs(o.Axis.Y) * o.HalfWidth;
        float ey = MathF.Abs(o.Axis.Y) * o.HalfDepth + MathF.Abs(o.Axis.X) * o.HalfWidth;
        return new SKRect(o.Center.X - ex, o.Center.Y - ey, o.Center.X + ex, o.Center.Y + ey);
    }

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
