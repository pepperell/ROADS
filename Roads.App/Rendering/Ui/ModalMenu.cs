using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Shared base for the full-screen modal menus (<see cref="TitleScreen"/> and
/// <see cref="PauseMenu"/>): the panel itself is a full-canvas translucent scrim (the
/// SettingsDialog idiom — it consumes every left click so nothing behind it receives
/// input) holding a centered window of large vertical buttons added via
/// <see cref="AddMenuButton"/> (the window grows to fit).
///
/// Must be added to the <see cref="UiRoot"/> after the in-game panels and BEFORE the
/// <see cref="SettingsDialog"/> (which stays topmost so Settings opens in front of either
/// menu), and — because it is <see cref="Panel.ExternallyDrawn"/> — painted by
/// MainForm.OnPaintSurface after the performance HUD and before the settings dialog.
/// The owner gates keyboard, wheel, and middle/right mouse input while a menu is open
/// and drives all pause semantics; <see cref="Open"/>/<see cref="Close"/> only toggle
/// visibility.
/// </summary>
public abstract class ModalMenu : Panel
{
    protected const float MenuButtonWidth = 260f;
    protected const float MenuButtonHeight = 48f;
    protected const float MenuButtonSpacing = 14f;
    protected const float WindowPad = 24f;

    /// <summary>The centered dark window holding the button column.</summary>
    protected readonly Panel Window;
    private int _buttonCount;

    protected ModalMenu()
    {
        // The menu itself is the scrim: full-canvas (see Layout), swallows every click.
        Visible = false;
        ExternallyDrawn = true;
        BackgroundColor = new SKColor(0, 0, 0, 130);
        CornerRadius = 0f;

        Window = new Panel
        {
            Size = new SKSize(MenuButtonWidth + 2f * WindowPad, 2f * WindowPad),
            BackgroundColor = UiTheme.PanelBackground,
            BorderColor = UiTheme.Outline,
        };
        Add(Window);
    }

    /// <summary>Appends a large menu button to the vertical list; the window grows to fit.</summary>
    protected Button AddMenuButton(string label, Action onClick)
    {
        var button = new Button
        {
            Text = label,
            Font = UiTheme.FontMenu,
            TextOffset = new SKPoint(0f, 7f), // baseline tuning for the 20 px font
            Size = new SKSize(MenuButtonWidth, MenuButtonHeight),
            Offset = new SKPoint(WindowPad, WindowPad + _buttonCount * (MenuButtonHeight + MenuButtonSpacing)),
            Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
            Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
        };
        button.Click += onClick;
        Window.Add(button);
        _buttonCount++;
        Window.Size = new SKSize(Window.Size.Width,
            2f * WindowPad + _buttonCount * MenuButtonHeight + (_buttonCount - 1) * MenuButtonSpacing);
        return button;
    }

    public void Open() => Visible = true;
    public void Close() => Visible = false;

    /// <summary>Downward shift of the centered window from the canvas center — the title
    /// screen shifts its window down so the logo + window block centers as a unit.</summary>
    protected virtual float WindowCenterYOffset => 0f;

    /// <summary>Full-canvas scrim with the window centered — the framework has no center
    /// anchor, so bounds are computed here from the canvas size each frame.</summary>
    public override void Layout(float canvasWidth, float canvasHeight)
    {
        Bounds = SKRect.Create(0f, 0f, canvasWidth, canvasHeight);
        Window.Offset = new SKPoint(
            MathF.Round((canvasWidth - Window.Size.Width) / 2f),
            MathF.Round((canvasHeight - Window.Size.Height) / 2f + WindowCenterYOffset));
        LayoutChildren(canvasWidth, canvasHeight);
    }
}
