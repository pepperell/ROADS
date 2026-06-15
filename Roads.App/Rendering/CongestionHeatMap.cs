using SkiaSharp;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Computes and stores per-edge congestion values (0–1) by counting vehicles on each edge
/// and normalising against a capacity estimate derived from the edge's length and lane count.
/// <para>
/// Capacity heuristic: each lane can hold one vehicle per <see cref="MetersPerVehicle"/>
/// metres of road length. An edge with <c>LaneCount</c> lanes and length <c>L</c> can hold
/// approximately <c>LaneCount * L / MetersPerVehicle</c> vehicles. Congestion = actual count
/// divided by that capacity, clamped to [0, 1].
/// </para>
/// <para>
/// Allocation discipline: <see cref="Update"/> uses only pre-allocated fields; it does not
/// allocate on the heap per frame. The counts array is grown lazily when the edge list grows.
/// </para>
/// </summary>
public class CongestionHeatMap
{
    /// <summary>
    /// Assumed road spacing between vehicles at capacity (metres per vehicle per lane).
    /// Chosen as roughly twice the vehicle length plus a comfortable headway, giving a
    /// realistic jam density of ~1 vehicle every 10 m per lane.
    /// </summary>
    private const float MetersPerVehicle = 10f;

    /// <summary>Whether the heat-map overlay is drawn. Defaults to false; toggled at
    /// runtime via the H key (see MainForm.OnCanvasKeyDown).</summary>
    public bool Enabled { get; set; } = false;

    /// <summary>Per-edge vehicle count, indexed by edge index. Length grows to match the
    /// graph edge list; never shrunk to avoid repeated allocations.</summary>
    private int[] _counts = Array.Empty<int>();

    /// <summary>Per-edge normalised congestion in [0, 1]. Parallel to <see cref="_counts"/>.</summary>
    private float[] _congestion = Array.Empty<float>();

    /// <summary>
    /// Reusable paint for blending the heat-map tint over the road surface.
    /// Color is updated per edge in the draw path; no per-frame allocation needed.
    /// </summary>
    private readonly SKPaint _overlayPaint = new()
    {
        Style = SKPaintStyle.Stroke,
        StrokeCap = SKStrokeCap.Round,
        StrokeJoin = SKStrokeJoin.Round,
        IsAntialias = true,
    };

    /// <summary>
    /// Recomputes per-edge congestion values from the current vehicle snapshot.
    /// Iterates every active vehicle once, increments its edge's count, then normalises
    /// each count against the edge's capacity estimate. Called once per rendered frame,
    /// before the road draw pass.
    /// <para>
    /// Vehicles traversing an intersection arc (<see cref="VehicleStore.CurrentArc"/> ≥ 0)
    /// are excluded because they are not on an edge at that moment.
    /// </para>
    /// </summary>
    /// <param name="vehicles">Live vehicle store (read-only).</param>
    /// <param name="graph">Road graph providing edge lengths and lane counts.</param>
    public void Update(VehicleStore vehicles, RoadGraph graph)
    {
        int edgeCount = graph.Edges.Count;

        // Grow buffers if the graph has grown (never shrink to avoid churn)
        if (_counts.Length < edgeCount)
        {
            Array.Resize(ref _counts, edgeCount);
            Array.Resize(ref _congestion, edgeCount);
        }

        // Zero the counts for active edges only
        for (int i = 0; i < edgeCount; i++)
            _counts[i] = 0;

        // Tally vehicles onto their current edge (skip arc-traversing vehicles)
        int vehicleCount = vehicles.Count;
        int[] currentEdge = vehicles.CurrentEdge;
        int[] currentArc  = vehicles.CurrentArc;

        for (int v = 0; v < vehicleCount; v++)
        {
            if (currentArc[v] >= 0) continue; // in intersection arc — not on an edge
            int e = currentEdge[v];
            if ((uint)e < (uint)edgeCount)
                _counts[e]++;
        }

        // Normalise against capacity. A bidirectional road is rendered as ONE surface —
        // RoadRenderer draws only the lower-index edge of each forward/reverse pair — so a
        // road's congestion must aggregate BOTH directed edges. Otherwise return-direction
        // traffic (e.g. the evening home-bound commute, which rides the reverse edge) is
        // counted but never displayed. Both edges of a pair receive the same combined value,
        // so whichever one is drawn reflects the road's total congestion.
        var edges = graph.Edges;
        for (int i = 0; i < edgeCount; i++)
        {
            var edge = edges[i];
            if (edge.FromNode < 0)
            {
                _congestion[i] = 0f;
                continue;
            }

            int count = _counts[i];
            float capacity = edge.LaneCount * edge.Length / MetersPerVehicle;

            int reverse = graph.FindReverseEdge(i);
            if ((uint)reverse < (uint)edgeCount && edges[reverse].FromNode >= 0)
            {
                var rev = edges[reverse];
                count += _counts[reverse];
                capacity += rev.LaneCount * rev.Length / MetersPerVehicle;
            }

            if (capacity < 0.5f) capacity = 0.5f; // guard against zero-length edges
            _congestion[i] = Math.Min(1f, count / capacity);
        }
    }

    /// <summary>
    /// Returns the congestion level for a given edge index, or 0 if the index is out of range.
    /// </summary>
    /// <param name="edgeIndex">Index of the road edge.</param>
    public float GetCongestion(int edgeIndex) =>
        (uint)edgeIndex < (uint)_congestion.Length ? _congestion[edgeIndex] : 0f;

    /// <summary>
    /// Returns a heat-map color for the given congestion level. The alpha encodes intensity
    /// so the road surface color shows through at low congestion:
    /// <list type="bullet">
    ///   <item><description>0.0 → near-transparent (alpha ≈ 0)</description></item>
    ///   <item><description>0.0–0.5 → green fading toward yellow (alpha rises)</description></item>
    ///   <item><description>0.5–1.0 → yellow fading toward red (alpha peaks)</description></item>
    /// </list>
    /// </summary>
    /// <param name="congestion">Normalised congestion value in [0, 1].</param>
    public static SKColor GetColor(float congestion)
    {
        // Alpha ramps from 0 at zero congestion to ~200 at full congestion so the road
        // type surface colour remains visible at low densities and is dominated by red
        // only when the road is genuinely jammed.
        byte alpha = (byte)(congestion * 200f);

        byte r, g, b;
        if (congestion <= 0.5f)
        {
            // Green (0,200,0) → Yellow (220,220,0) over lower half
            float t = congestion * 2f; // 0..1 within this band
            r = (byte)(t * 220f);
            g = 200;
            b = 0;
        }
        else
        {
            // Yellow (220,220,0) → Red (220,30,0) over upper half
            float t = (congestion - 0.5f) * 2f; // 0..1 within this band
            r = 220;
            g = (byte)(220f * (1f - t) + 30f * t);
            b = 0;
        }

        return new SKColor(r, g, b, alpha);
    }

    /// <summary>
    /// Returns the reusable overlay paint with its color and stroke width set for the given
    /// edge, ready for drawing over the road surface.  The caller must not dispose this paint.
    /// </summary>
    /// <param name="congestion">Normalised congestion level for this edge.</param>
    /// <param name="strokeWidth">Stroke width matching the road surface width.</param>
    internal SKPaint GetOverlayPaint(float congestion, float strokeWidth)
    {
        _overlayPaint.Color = GetColor(congestion);
        _overlayPaint.StrokeWidth = strokeWidth;
        return _overlayPaint;
    }
}
