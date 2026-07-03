using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>Background + text color pair for one button state.</summary>
public readonly struct ButtonColors
{
    public readonly SKColor Background;
    public readonly SKColor Text;
    public ButtonColors(SKColor background, SKColor text)
    {
        Background = background;
        Text = text;
    }
}

/// <summary>
/// A <see cref="Label"/> with a button-like rounded background whose colors change with
/// state. Resolution order: disabled → active (+hover) → hover → idle; text is centered.
/// <see cref="Panel.Click"/> fires on MOUSE-DOWN (deliberate parity with the app's
/// historical immediate-mode buttons — tool switches, actions, and POI selection all fire
/// on the down; a documented divergence from stock WinForms click-on-up-inside).
/// State sources are live delegates so buttons need no per-frame sync:
/// <see cref="IsActive"/> (e.g. active tool, paused) and <see cref="IsEnabled"/>
/// (e.g. speed clamps) are evaluated at draw and click time.
/// </summary>
public class Button : Label
{
    public ButtonColors Idle = new(new SKColor(55, 58, 65), new SKColor(180, 180, 180));
    public ButtonColors Hover = new(new SKColor(75, 80, 90), SKColors.White);

    /// <summary>Colors while active (falls back to <see cref="Idle"/> when null).</summary>
    public ButtonColors? Active;

    /// <summary>Colors while active AND hovered (falls back to <see cref="Active"/>).</summary>
    public ButtonColors? ActiveHover;

    /// <summary>Colors while disabled (falls back to <see cref="Idle"/>).</summary>
    public ButtonColors? Disabled;

    /// <summary>Live active state (active tool, paused, selected POI); null = never active.</summary>
    public Func<bool>? IsActive;

    /// <summary>Live enabled state; a disabled button draws dimmed and swallows clicks
    /// without raising <see cref="Panel.Click"/>. Null = always enabled.</summary>
    public Func<bool>? IsEnabled;

    public Button()
    {
        CornerRadius = 3f;
        TextAlign = SKTextAlign.Center;
        TextOffset = new SKPoint(0f, 4.5f);
    }

    private ButtonColors ResolveColors()
    {
        if (IsEnabled != null && !IsEnabled()) return Disabled ?? Idle;
        bool active = IsActive?.Invoke() ?? false;
        if (active) return IsHovered ? (ActiveHover ?? Active ?? Idle) : (Active ?? Idle);
        return IsHovered ? Hover : Idle;
    }

    protected override void OnDrawBackground(SKCanvas canvas)
    {
        var colors = ResolveColors();
        TextColor = colors.Text;
        UiTheme.FillScratch.Color = colors.Background;
        canvas.DrawRoundRect(Bounds, CornerRadius, CornerRadius, UiTheme.FillScratch);
    }

    public override bool OnMouseDown(float x, float y)
    {
        if (IsEnabled == null || IsEnabled())
            RaiseClick();
        return true; // always consume — a disabled button still blocks the map behind it
    }
}
