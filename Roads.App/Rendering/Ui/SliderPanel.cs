using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Top-right overlay of labeled parameter <see cref="Slider"/> rows (steering tunables),
/// hidden by default and toggled with the T key. Rows are registered once via
/// <see cref="AddSlider"/> (same signature as the retired immediate-mode panel) and laid
/// out on a fixed 36 px row grid. The panel background consumes clicks, so clicking
/// between rows never edits the map.
/// </summary>
public class SliderPanel : Panel
{
    private const float PanelWidth = 200f;
    private const float SliderHeight = 16f;
    private const float RowHeight = 36f;
    private const float Padding = 10f;
    private const float LabelHeight = 14f;
    private const float HitPadX = 4f;
    private const float HitPadY = 6f;

    private readonly List<Slider> _rows = new();

    public SliderPanel()
    {
        Visible = false;
        Anchor = UiAnchor.TopRight;
        // Below the clock panel in the top-right column.
        Margin = new SKPoint(Padding, 10f + ClockPanel.PanelHeight + 8f);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
        Size = new SKSize(PanelWidth, Padding * 2f);
    }

    /// <summary>Registers a new slider row with the given label, range, and accessors.</summary>
    public void AddSlider(string label, float min, float max, Func<float> get, Action<float> set)
    {
        var row = new Slider(label, min, max, get, set)
        {
            // Bounds are the generous hit box surrounding the track (±4 h / ±6 v).
            Size = new SKSize(PanelWidth - 2f * Padding + 2f * HitPadX, SliderHeight + 2f * HitPadY),
        };
        _rows.Add(row);
        Add(row);
        Size = new SKSize(PanelWidth, Padding * 2f + _rows.Count * RowHeight);
    }

    protected override void LayoutChildren(float canvasWidth, float canvasHeight)
    {
        for (int i = 0; i < _rows.Count; i++)
        {
            // Track top within the row sits below the label line; the hit box extends
            // HitPadY above it (matching the retired panel's grab zone exactly).
            float trackTop = Padding + i * RowHeight + LabelHeight + 2f;
            _rows[i].Offset = new SKPoint(Padding - HitPadX, trackTop - HitPadY);
        }
        base.LayoutChildren(canvasWidth, canvasHeight);
    }
}
