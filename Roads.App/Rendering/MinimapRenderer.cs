using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Screen-space overview panel in the bottom-right corner showing the whole road network,
/// the current camera viewport as a rectangle, and supporting click/drag to jump the camera.
/// The panel background is a dark desaturated terrain green and road segments are colored
/// (and Highway/Arterial slightly thickened) by <see cref="RoadType"/>, drawn in ascending
/// visual importance so highways sit on top. The network geometry is recorded once into an
/// <see cref="SKPicture"/> (in panel-local pixels) and re-recorded only when the graph
/// changes, so per-frame cost is one picture draw plus one rectangle. The world→panel
/// transform (<see cref="_scale"/>, <see cref="_offsetX"/>, <see cref="_offsetY"/>,
/// <see cref="_worldMin"/>) and the last panel rect are kept so input hit-testing and
/// screen→world mapping stay consistent with what was drawn.
/// </summary>
public class MinimapRenderer
{
    /// <summary>Fixed side length of the (square) minimap panel in pixels. Constant so the panel
    /// never changes size as roads are added — the network is fit (letterboxed) inside it.</summary>
    private const float BoxSize = 200f;
    /// <summary>Gap in pixels between the panel and the canvas edges.</summary>
    private const float Margin = 10f;
    /// <summary>World-space padding added around the network bounds (meters), so roads near the
    /// edge of the map aren't drawn flush against the panel border. Also guarantees a non-zero
    /// extent for single-node / degenerate networks.</summary>
    private const float WorldPad = 12f;

    /// <summary>Whether the minimap is shown and accepts input.</summary>
    public bool Visible { get; set; } = true;

    private readonly SKPaint _bgPaint = new() { Color = new SKColor(30, 38, 25, 200), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _borderPaint = new() { Color = new SKColor(80, 82, 88), Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true };
    /// <summary>Reusable road stroke; color and width are set per road type while recording.</summary>
    private readonly SKPaint _roadPaint = new() { Style = SKPaintStyle.Stroke, IsAntialias = true, StrokeCap = SKStrokeCap.Round };
    private readonly SKPaint _viewportPaint = new() { Color = new SKColor(120, 200, 255, 230), Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true };

    /// <summary>Road-type record order, ascending visual importance so highways draw on top.</summary>
    private static readonly RoadType[] DrawOrder =
        { RoadType.Dirt, RoadType.Residential, RoadType.Arterial, RoadType.Highway };

    // Cached network geometry (panel-local pixels) and the transform it was built with.
    private SKPicture? _networkPicture;
    private int _cachedVersion = -1;
    private bool _hasContent;
    private float _scale, _offsetX, _offsetY;
    private Vector2 _worldMin;

    // Last drawn panel rect (screen pixels) — used by input between frames.
    private SKRect _panelRect = SKRect.Empty;

    /// <summary>
    /// Draws the minimap panel when enabled: the fixed-size background box always, plus the
    /// cached road network and live viewport rectangle when there are roads. On a blank map the
    /// box is still shown, just empty. Rebuilds the cached network/transform when the graph changes.
    /// </summary>
    public void Draw(SKCanvas canvas, Camera camera, RoadGraph graph, int canvasWidth, int canvasHeight)
    {
        if (!Visible) { _panelRect = SKRect.Empty; return; }
        EnsureCache(graph);

        float left = canvasWidth - Margin - BoxSize;
        float top = canvasHeight - Margin - BoxSize;
        _panelRect = new SKRect(left, top, left + BoxSize, top + BoxSize);

        canvas.DrawRoundRect(_panelRect, 4f, 4f, _bgPaint);

        if (_hasContent)
        {
            canvas.Save();
            canvas.Translate(left, top);
            canvas.ClipRect(new SKRect(0, 0, BoxSize, BoxSize));

            if (_networkPicture != null) canvas.DrawPicture(_networkPicture);

            // Viewport rectangle: the visible world rect mapped into panel-local pixels.
            var view = camera.GetVisibleWorldRect(canvasWidth, canvasHeight);
            var tl = WorldToPanel(view.Left, view.Top);
            var br = WorldToPanel(view.Right, view.Bottom);
            canvas.DrawRect(new SKRect(tl.X, tl.Y, br.X, br.Y), _viewportPaint);

            canvas.Restore();
        }

        canvas.DrawRoundRect(_panelRect, 4f, 4f, _borderPaint);
    }

    /// <summary>True if a screen-space point lies on the visible minimap panel (even when blank,
    /// so clicks on the box are swallowed rather than placing a road behind it).</summary>
    public bool HitTest(float x, float y) => Visible && _panelRect.Contains(x, y);

    /// <summary>
    /// Maps a screen-space point on the panel back to the world position it represents.
    /// Returns false when the minimap is hidden, off the panel, or blank (no transform yet).
    /// </summary>
    public bool TryScreenToWorld(float x, float y, out Vector2 world)
    {
        world = default;
        if (!HitTest(x, y) || !_hasContent || _scale <= 0f) return false;
        float localX = x - _panelRect.Left;
        float localY = y - _panelRect.Top;
        world = new Vector2(
            (localX - _offsetX) / _scale + _worldMin.X,
            (localY - _offsetY) / _scale + _worldMin.Y);
        return true;
    }

    /// <summary>Maps a world position to panel-local pixels using the current cached transform.</summary>
    private SKPoint WorldToPanel(float wx, float wy)
        => new(_offsetX + (wx - _worldMin.X) * _scale, _offsetY + (wy - _worldMin.Y) * _scale);

    /// <summary>
    /// Recomputes the network bounds, world→panel transform, and cached picture when the graph
    /// version changes. Sets <see cref="_hasContent"/> false when there are no roads, so the panel
    /// is drawn blank (no network, no viewport rect). Bounds come from live edge endpoints — a
    /// stray node with no edges (e.g. an in-progress road start) does not count.
    /// </summary>
    private void EnsureCache(RoadGraph graph)
    {
        if (graph.Version == _cachedVersion) return;
        _cachedVersion = graph.Version;

        // World bounds over the endpoints of live (non-defunct) edges.
        float minX = float.MaxValue, minY = float.MaxValue, maxX = float.MinValue, maxY = float.MinValue;
        bool any = false;
        var nodes = graph.Nodes;
        var edges = graph.Edges;
        for (int i = 0; i < edges.Count; i++)
        {
            var e = edges[i];
            if (e.FromNode < 0) continue;
            var a = nodes[e.FromNode].Position;
            var b = nodes[e.ToNode].Position;
            if (float.IsNaN(a.X) || float.IsNaN(b.X)) continue;
            any = true;
            minX = MathF.Min(minX, MathF.Min(a.X, b.X));
            minY = MathF.Min(minY, MathF.Min(a.Y, b.Y));
            maxX = MathF.Max(maxX, MathF.Max(a.X, b.X));
            maxY = MathF.Max(maxY, MathF.Max(a.Y, b.Y));
        }

        if (!any)
        {
            _hasContent = false;
            _networkPicture?.Dispose();
            _networkPicture = null;
            return;
        }
        _hasContent = true;

        minX -= WorldPad; minY -= WorldPad; maxX += WorldPad; maxY += WorldPad;
        _worldMin = new Vector2(minX, minY);
        float worldW = maxX - minX;
        float worldH = maxY - minY;

        // Fixed square panel; the network is fit inside it (uniform scale, centered/letterboxed).
        _scale = MathF.Min(BoxSize / worldW, BoxSize / worldH);
        _offsetX = (BoxSize - worldW * _scale) / 2f;
        _offsetY = (BoxSize - worldH * _scale) / 2f;

        RecordNetwork(graph);
    }

    /// <summary>
    /// Records the road network as straight node-to-node segments in panel-local pixels,
    /// bucketed into one path per <see cref="RoadType"/> and stroked in <see cref="DrawOrder"/>
    /// with per-type color and width (Highway/Arterial slightly thicker).
    /// </summary>
    private void RecordNetwork(RoadGraph graph)
    {
        using var recorder = new SKPictureRecorder();
        var rec = recorder.BeginRecording(new SKRect(0, 0, BoxSize, BoxSize));

        var paths = new SKPath[4];
        for (int t = 0; t < paths.Length; t++) paths[t] = new SKPath();

        var nodes = graph.Nodes;
        var edges = graph.Edges;
        for (int i = 0; i < edges.Count; i++)
        {
            var e = edges[i];
            if (e.FromNode < 0) continue; // defunct
            var a = nodes[e.FromNode].Position;
            var b = nodes[e.ToNode].Position;
            if (float.IsNaN(a.X) || float.IsNaN(b.X)) continue;
            int t = (int)e.RoadType;
            if (t < 0 || t >= paths.Length) t = (int)RoadType.Residential;
            var pa = WorldToPanel(a.X, a.Y);
            var pb = WorldToPanel(b.X, b.Y);
            paths[t].MoveTo(pa);
            paths[t].LineTo(pb);
        }

        foreach (var type in DrawOrder)
        {
            var path = paths[(int)type];
            if (!path.IsEmpty)
            {
                _roadPaint.Color = GetRoadColor(type);
                _roadPaint.StrokeWidth = GetRoadWidth(type);
                rec.DrawPath(path, _roadPaint);
            }
            path.Dispose();
        }

        _networkPicture?.Dispose();
        _networkPicture = recorder.EndRecording();
    }

    /// <summary>Minimap stroke color for a road type — muted grays by importance, brown for dirt.</summary>
    private static SKColor GetRoadColor(RoadType type) => type switch
    {
        RoadType.Highway  => new SKColor(170, 175, 185),
        RoadType.Arterial => new SKColor(148, 150, 156),
        RoadType.Dirt     => new SKColor(122, 100, 68),
        _                 => new SKColor(118, 120, 124),
    };

    /// <summary>Minimap stroke width (panel pixels) — Highway/Arterial slightly thicker.</summary>
    private static float GetRoadWidth(RoadType type) => type switch
    {
        RoadType.Highway  => 1.7f,
        RoadType.Arterial => 1.4f,
        _                 => 1.1f,
    };
}
