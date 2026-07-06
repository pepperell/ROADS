using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Labeled checkbox bound to external state through get/set accessors (the same live
/// pattern as <see cref="Slider"/>), so the checked visual always reflects the current
/// value with no per-frame sync — including when another control flips the bound state
/// (e.g. mutually exclusive options). A click anywhere on the control (box or label)
/// toggles the value and consumes the event.
/// </summary>
public class Checkbox : Panel
{
    private const float BoxSize = 12f;
    private const float BoxCorner = 2.5f;
    private const float LabelGap = 6f;

    private readonly string _label;
    private readonly Func<bool> _get;
    private readonly Action<bool> _set;

    private static readonly SKColor CheckedFill = new(60, 130, 200);
    private static readonly SKColor UncheckedFill = new(50, 53, 60);

    public Checkbox(string label, Func<bool> get, Action<bool> set)
    {
        _label = label;
        _get = get;
        _set = set;
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        bool isChecked = _get();
        float boxLeft = Bounds.Left;
        float boxTop = Bounds.MidY - BoxSize / 2f;
        var box = SKRect.Create(boxLeft, boxTop, BoxSize, BoxSize);

        UiTheme.FillScratch.Color = isChecked ? CheckedFill : UncheckedFill;
        canvas.DrawRoundRect(box, BoxCorner, BoxCorner, UiTheme.FillScratch);
        UiTheme.StrokeScratch.Color = isChecked ? CheckedFill : UiTheme.Outline;
        UiTheme.StrokeScratch.StrokeWidth = 1f;
        canvas.DrawRoundRect(box, BoxCorner, BoxCorner, UiTheme.StrokeScratch);

        if (isChecked)
        {
            UiTheme.StrokeScratch.Color = SKColors.White;
            UiTheme.StrokeScratch.StrokeWidth = 1.8f;
            using var check = new SKPath();
            check.MoveTo(box.Left + 2.5f, box.MidY + 0.5f);
            check.LineTo(box.Left + 5f, box.Bottom - 3f);
            check.LineTo(box.Right - 2.5f, box.Top + 3f);
            canvas.DrawPath(check, UiTheme.StrokeScratch);
        }

        UiTheme.TextScratch.Color = IsHovered ? SKColors.White : UiTheme.TextPrimary;
        canvas.DrawText(_label, boxLeft + BoxSize + LabelGap, Bounds.MidY + 4f,
            SKTextAlign.Left, UiTheme.Font11, UiTheme.TextScratch);
    }

    public override bool OnMouseDown(float x, float y)
    {
        _set(!_get());
        RaiseClick();
        return true;
    }
}
