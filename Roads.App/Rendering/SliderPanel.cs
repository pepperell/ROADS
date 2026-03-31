using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Definition for a single slider: label, value range, and getter/setter delegates.
/// Layout bounds are computed each frame by SliderPanel.Layout.
/// </summary>
public class SliderDef
{
    /// <summary>Display label for the slider.</summary>
    public string Label = "";
    /// <summary>Minimum slider value.</summary>
    public float Min;
    /// <summary>Maximum slider value.</summary>
    public float Max;
    /// <summary>Delegate to read the current value.</summary>
    public Func<float> Get = null!;
    /// <summary>Delegate to write a new value.</summary>
    public Action<float> Set = null!;
    /// <summary>Screen-space bounds of the slider track (computed by Layout).</summary>
    public SKRect TrackBounds;
    /// <summary>Screen-space bounds of the thumb handle (computed by Draw).</summary>
    public SKRect ThumbBounds;
}

/// <summary>
/// An overlay panel of labeled parameter sliders rendered in screen space.
/// Positioned at the top-right of the canvas. Supports click-and-drag interaction
/// and is toggled on/off with the T key.
/// </summary>
public class SliderPanel
{
    private const float PanelWidth = 200f;
    private const float SliderHeight = 16f;
    private const float RowHeight = 36f;
    private const float Padding = 10f;
    private const float LabelHeight = 14f;
    private const float ThumbWidth = 8f;

    /// <summary>All registered slider definitions.</summary>
    private readonly List<SliderDef> _sliders = new();
    /// <summary>Index of the slider currently being dragged, or -1.</summary>
    private int _draggingIndex = -1;
    /// <summary>Screen X position of the panel's left edge.</summary>
    private float _panelX;
    /// <summary>Screen Y position of the panel's top edge.</summary>
    private float _panelY;

    /// <summary>Whether the slider panel is visible. Toggled with the T key.</summary>
    public bool Visible { get; set; } = true;

    /// <summary>
    /// Registers a new slider with the given label, range, and getter/setter delegates.
    /// </summary>
    /// <param name="label">Display label.</param>
    /// <param name="min">Minimum value.</param>
    /// <param name="max">Maximum value.</param>
    /// <param name="get">Delegate to read the current value.</param>
    /// <param name="set">Delegate to write a new value.</param>
    public void AddSlider(string label, float min, float max, Func<float> get, Action<float> set)
    {
        _sliders.Add(new SliderDef { Label = label, Min = min, Max = max, Get = get, Set = set });
    }

    /// <summary>Recomputes track bounds based on the current screen width.</summary>
    public void Layout(float screenWidth)
    {
        _panelX = screenWidth - PanelWidth - Padding;
        _panelY = 40f;

        for (int i = 0; i < _sliders.Count; i++)
        {
            float y = _panelY + Padding + i * RowHeight + LabelHeight + 2f;
            float trackX = _panelX + Padding;
            float trackW = PanelWidth - 2 * Padding;
            _sliders[i].TrackBounds = new SKRect(trackX, y, trackX + trackW, y + SliderHeight);
        }
    }

    /// <summary>Draws the slider panel with all sliders, labels, and current values.</summary>
    public void Draw(SKCanvas canvas, float screenWidth)
    {
        if (!Visible || _sliders.Count == 0) return;

        Layout(screenWidth);

        float panelH = Padding + _sliders.Count * RowHeight + Padding;

        // Panel background
        using var bgPaint = new SKPaint
        {
            Color = new SKColor(25, 27, 33, 230),
            Style = SKPaintStyle.Fill,
        };
        canvas.DrawRoundRect(_panelX, _panelY, PanelWidth, panelH, 4f, 4f, bgPaint);

        using var labelFont = new SKFont { Size = 11 };
        using var labelPaint = new SKPaint { Color = new SKColor(170, 175, 185), IsAntialias = true };
        using var valuePaint = new SKPaint { Color = new SKColor(120, 200, 255), IsAntialias = true };
        using var trackPaint = new SKPaint { Color = new SKColor(50, 53, 60), Style = SKPaintStyle.Fill };
        using var fillPaint = new SKPaint { Color = new SKColor(60, 130, 200), Style = SKPaintStyle.Fill };
        using var thumbPaint = new SKPaint { Color = SKColors.White, Style = SKPaintStyle.Fill, IsAntialias = true };

        for (int i = 0; i < _sliders.Count; i++)
        {
            var s = _sliders[i];
            var track = s.TrackBounds;
            float labelY = _panelY + Padding + i * RowHeight + LabelHeight;
            float val = s.Get();

            // Label + value
            canvas.DrawText(s.Label, track.Left, labelY, SKTextAlign.Left, labelFont, labelPaint);
            canvas.DrawText(val.ToString("F2"), track.Right, labelY, SKTextAlign.Right, labelFont, valuePaint);

            // Track
            canvas.DrawRoundRect(track, 3f, 3f, trackPaint);

            // Fill
            float t = Math.Clamp((val - s.Min) / (s.Max - s.Min), 0f, 1f);
            float fillW = t * track.Width;
            canvas.DrawRoundRect(track.Left, track.Top, fillW, track.Height, 3f, 3f, fillPaint);

            // Thumb
            float thumbX = track.Left + t * track.Width - ThumbWidth / 2f;
            var thumbRect = new SKRect(thumbX, track.Top - 1f, thumbX + ThumbWidth, track.Bottom + 1f);
            canvas.DrawRoundRect(thumbRect, 2f, 2f, thumbPaint);
            s.ThumbBounds = thumbRect;
        }
    }

    /// <summary>Handles mouse down: begins dragging if a slider track was hit.</summary>
    /// <returns><c>true</c> if a slider was hit and the event was consumed.</returns>
    public bool OnMouseDown(float x, float y)
    {
        if (!Visible) return false;

        for (int i = 0; i < _sliders.Count; i++)
        {
            var track = _sliders[i].TrackBounds;
            // Hit test on track area (generous vertical)
            if (x >= track.Left - 4f && x <= track.Right + 4f &&
                y >= track.Top - 6f && y <= track.Bottom + 6f)
            {
                _draggingIndex = i;
                UpdateSliderValue(i, x);
                return true;
            }
        }
        return false;
    }

    /// <summary>Handles mouse move: updates the dragged slider value.</summary>
    /// <returns><c>true</c> if a slider is being dragged and the event was consumed.</returns>
    public bool OnMouseMove(float x, float y)
    {
        if (_draggingIndex < 0) return false;
        UpdateSliderValue(_draggingIndex, x);
        return true;
    }

    /// <summary>Handles mouse up: ends any active slider drag.</summary>
    public void OnMouseUp()
    {
        _draggingIndex = -1;
    }

    /// <summary>Whether a slider is currently being dragged.</summary>
    public bool IsDragging => _draggingIndex >= 0;

    /// <summary>Maps screen X to a value and applies it to the slider at the given index.</summary>
    private void UpdateSliderValue(int index, float screenX)
    {
        var s = _sliders[index];
        var track = s.TrackBounds;
        float t = Math.Clamp((screenX - track.Left) / track.Width, 0f, 1f);
        float val = s.Min + t * (s.Max - s.Min);
        s.Set(val);
    }
}
