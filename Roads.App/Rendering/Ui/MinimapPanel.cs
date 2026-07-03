using System.Numerics;
using SkiaSharp;
using Roads.App.Core;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Bottom-right overview panel showing the whole road network, the current camera viewport
/// as a rectangle, and click/drag camera control: mouse-down jumps the camera to the
/// clicked world point and begins a scrub-drag (no dead zone — the jump happens on the
/// down), captured moves keep scrubbing while the cursor stays on the panel and freeze the
/// camera when it leaves (returning resumes). A blank map still shows (and swallows clicks
/// on) the empty box. A chip in the box's top-left corner shows the current camera zoom.
/// Road segments are colored/thickened by <see cref="RoadType"/> and
/// recorded once into an <see cref="SKPicture"/> (panel-local pixels), re-recorded only
/// when the graph version changes; the world→panel transform is kept alongside so
/// screen→world mapping stays consistent with what was drawn.
/// </summary>
public class MinimapPanel : Panel
{
    /// <summary>Fixed side length of the (square) panel — the network is fit (letterboxed) inside.</summary>
    private const float BoxSize = 200f;
    /// <summary>World-space padding (meters) around the network bounds so edge roads aren't
    /// flush against the border; also guarantees a non-zero extent for degenerate networks.</summary>
    private const float WorldPad = 12f;

    private readonly Camera _camera;
    private readonly RoadGraph _graph;

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

    // Canvas size captured at layout, needed for the camera's visible-rect query at draw.
    private float _canvasWidth, _canvasHeight;

    public MinimapPanel(Camera camera, RoadGraph graph)
    {
        _camera = camera;
        _graph = graph;
        Anchor = UiAnchor.BottomRight;
        Margin = new SKPoint(10f, 10f);
        Size = new SKSize(BoxSize, BoxSize);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
    }

    public override void Layout(float canvasWidth, float canvasHeight)
    {
        _canvasWidth = canvasWidth;
        _canvasHeight = canvasHeight;
        base.Layout(canvasWidth, canvasHeight);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        EnsureCache(_graph);
        if (_hasContent)
        {
            canvas.Save();
            canvas.Translate(Bounds.Left, Bounds.Top);
            canvas.ClipRect(new SKRect(0, 0, BoxSize, BoxSize));

            if (_networkPicture != null) canvas.DrawPicture(_networkPicture);

            // Viewport rectangle: the visible world rect mapped into panel-local pixels.
            var view = _camera.GetVisibleWorldRect((int)_canvasWidth, (int)_canvasHeight);
            var tl = WorldToPanel(view.Left, view.Top);
            var br = WorldToPanel(view.Right, view.Bottom);
            canvas.DrawRect(new SKRect(tl.X, tl.Y, br.X, br.Y), _viewportPaint);

            canvas.Restore();
        }

        // Zoom chip (top-left corner of the box): a translucent backdrop keeps the value
        // readable over roads. Lives here since the status bar was dissolved.
        var chip = SKRect.Create(Bounds.Left + 4f, Bounds.Top + 4f, 52f, 16f);
        UiTheme.FillScratch.Color = UiTheme.HudBackground;
        canvas.DrawRoundRect(chip, 3f, 3f, UiTheme.FillScratch);
        UiTheme.TextScratch.Color = UiTheme.TextPrimary;
        canvas.DrawText($"{_camera.Zoom:F2}x", chip.MidX, chip.Bottom - 4f,
            SKTextAlign.Center, UiTheme.Font11, UiTheme.TextScratch);
    }

    /// <summary>Click-to-jump plus scrub start. Always consumes (a blank box swallows the
    /// click rather than placing a road behind it); the camera moves only when the map has
    /// content. Capture then routes all moves to <see cref="OnMouseMove"/> for scrubbing.</summary>
    public override bool OnMouseDown(float x, float y)
    {
        if (TryScreenToWorld(x, y, out var world))
            _camera.CenterOnWorld(world.X, world.Y);
        RaiseClick();
        return true;
    }

    /// <summary>Captured scrub: keeps centering the camera while the cursor is on the panel;
    /// leaving the panel freezes the camera until the cursor returns (historical behavior).</summary>
    public override void OnMouseMove(float x, float y)
    {
        if (!IsPressed) return;
        if (TryScreenToWorld(x, y, out var world))
            _camera.CenterOnWorld(world.X, world.Y);
    }

    /// <summary>
    /// Maps a screen-space point on the panel back to the world position it represents.
    /// False when off the panel or the map is blank (no transform yet).
    /// </summary>
    private bool TryScreenToWorld(float x, float y, out Vector2 world)
    {
        world = default;
        if (!Bounds.Contains(x, y) || !_hasContent || _scale <= 0f) return false;
        float localX = x - Bounds.Left;
        float localY = y - Bounds.Top;
        world = new Vector2(
            (localX - _offsetX) / _scale + _worldMin.X,
            (localY - _offsetY) / _scale + _worldMin.Y);
        return true;
    }

    /// <summary>Maps a world position to panel-local pixels using the current cached transform.</summary>
    private SKPoint WorldToPanel(float wx, float wy)
        => new(_offsetX + (wx - _worldMin.X) * _scale, _offsetY + (wy - _worldMin.Y) * _scale);

    /// <summary>
    /// Recomputes the network bounds, world→panel transform, and cached picture when the
    /// graph version changes. Bounds come from live edge endpoints — a stray node with no
    /// edges (an in-progress road start) does not count as content, and a blank map draws
    /// an empty box (no network, no viewport rect).
    /// </summary>
    private void EnsureCache(RoadGraph graph)
    {
        if (graph.Version == _cachedVersion) return;
        _cachedVersion = graph.Version;

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
