using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// One labeled slider row inside a <see cref="SliderPanel"/>: label above, current value
/// right-aligned, a filled track with a white thumb. The panel positions this row so its
/// <see cref="Panel.Bounds"/> are the generous HIT BOX (track ±4 px horizontally, ±6 px
/// vertically — the historical grab feel); the visible track is inset from it. Mouse-down
/// jumps the value to the clicked position and consumes the event, so <see cref="UiRoot"/>
/// capture then routes every move here — drags keep working with the cursor far outside
/// the panel (the value is clamped to the range).
/// </summary>
public class Slider : Panel
{
    private const float HitPadX = 4f;
    private const float HitPadY = 6f;

    public readonly string LabelText;
    public readonly float Min;
    public readonly float Max;
    private readonly Func<float> _get;
    private readonly Action<float> _set;

    public Slider(string label, float min, float max, Func<float> get, Action<float> set)
    {
        LabelText = label;
        Min = min;
        Max = max;
        _get = get;
        _set = set;
    }

    /// <summary>Visible track rect (bounds minus the hit padding).</summary>
    private SKRect Track => new(
        Bounds.Left + HitPadX, Bounds.Top + HitPadY,
        Bounds.Right - HitPadX, Bounds.Bottom - HitPadY);

    protected override void OnDraw(SKCanvas canvas)
    {
        var track = Track;
        float labelY = track.Top - 2f;
        float val = _get();

        UiTheme.TextScratch.Color = new SKColor(170, 175, 185);
        canvas.DrawText(LabelText, track.Left, labelY, SKTextAlign.Left, UiTheme.Font11, UiTheme.TextScratch);
        UiTheme.TextScratch.Color = new SKColor(120, 200, 255);
        canvas.DrawText(val.ToString("F2"), track.Right, labelY, SKTextAlign.Right, UiTheme.Font11, UiTheme.TextScratch);

        UiTheme.FillScratch.Color = new SKColor(50, 53, 60);
        canvas.DrawRoundRect(track, 3f, 3f, UiTheme.FillScratch);

        float t = Math.Clamp((val - Min) / (Max - Min), 0f, 1f);
        UiTheme.FillScratch.Color = new SKColor(60, 130, 200);
        canvas.DrawRoundRect(track.Left, track.Top, t * track.Width, track.Height, 3f, 3f, UiTheme.FillScratch);

        const float thumbWidth = 8f;
        float thumbX = track.Left + t * track.Width - thumbWidth / 2f;
        UiTheme.FillScratch.Color = SKColors.White;
        canvas.DrawRoundRect(new SKRect(thumbX, track.Top - 1f, thumbX + thumbWidth, track.Bottom + 1f),
            2f, 2f, UiTheme.FillScratch);
    }

    public override bool OnMouseDown(float x, float y)
    {
        SetFromX(x);
        return true;
    }

    public override void OnMouseMove(float x, float y)
    {
        if (!IsPressed) return;
        SetFromX(x);
    }

    private void SetFromX(float screenX)
    {
        var track = Track;
        float t = Math.Clamp((screenX - track.Left) / track.Width, 0f, 1f);
        _set(Min + t * (Max - Min));
    }
}
