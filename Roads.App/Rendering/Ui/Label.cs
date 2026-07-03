using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// A <see cref="Panel"/> with single-line text: fixed <see cref="Text"/> or a live
/// <see cref="TextSource"/> (evaluated at draw time — status bar, Pause/Play). The font is
/// a shared <see cref="UiTheme"/> instance and is never disposed by the label. Text is
/// positioned by <see cref="TextAlign"/> against the left/center/right of the bounds, with
/// the baseline at vertical center plus <see cref="TextOffset"/>.Y (buttons use +4.5 for
/// optical centering). Panels needing multi-line or custom text layout override
/// <see cref="Panel.OnDraw"/> instead.
/// </summary>
public class Label : Panel
{
    public string Text = "";

    /// <summary>Live text source; when set, wins over <see cref="Text"/>.</summary>
    public Func<string>? TextSource;

    /// <summary>Shared font from <see cref="UiTheme"/> — never disposed here.</summary>
    public SKFont Font = UiTheme.Font13;

    public SKColor TextColor = SKColors.White;
    public SKTextAlign TextAlign = SKTextAlign.Left;

    /// <summary>Offset applied to the text anchor: X from the alignment edge, Y from the
    /// vertical center to the baseline.</summary>
    public SKPoint TextOffset;

    protected override void OnDraw(SKCanvas canvas)
    {
        string text = TextSource?.Invoke() ?? Text;
        if (string.IsNullOrEmpty(text)) return;

        float x = TextAlign switch
        {
            SKTextAlign.Center => Bounds.MidX,
            SKTextAlign.Right => Bounds.Right,
            _ => Bounds.Left,
        } + TextOffset.X;
        float y = Bounds.MidY + TextOffset.Y;

        UiTheme.TextScratch.Color = TextColor;
        canvas.DrawText(text, x, y, TextAlign, Font, UiTheme.TextScratch);
    }
}
