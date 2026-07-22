using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The in-game pause menu (Menu button, or Escape with nothing left to cancel): a
/// <see cref="ModalMenu"/> scrim over the paused simulation with Return to Game / Save /
/// Save As / Settings / Return to Title / Exit to Desktop (button window centered), plus
/// the keyboard-shortcut <see cref="LegendPanel"/> pinned to the left edge and centered
/// vertically. A dumb view — the owner (MainForm) supplies every action, owns the pause
/// capture/restore around Open/Close, and hosts the confirmation dialogs (Return to Title
/// and Exit both confirm). Save and Save As leave the menu open; the SettingsDialog sits
/// above this panel, so Settings opens in front with the menu dimmed behind its scrim.
/// </summary>
public class PauseMenu : ModalMenu
{
    /// <summary>Inset of the legend from the left edge of the canvas.</summary>
    private const float LegendLeftInset = 20f;

    private readonly LegendPanel _legend = new();

    public PauseMenu(Action onReturnToGame, Action onSave, Action onSaveAs,
        Action onSettings, Action onReturnToTitle, Action onExit)
    {
        AddMenuButton("Return to Game", onReturnToGame);
        AddMenuButton("Save", onSave);
        AddMenuButton("Save As", onSaveAs);
        AddMenuButton("Settings", onSettings);
        AddMenuButton("Return to Title", onReturnToTitle);
        AddMenuButton("Exit to Desktop", onExit);
        Add(_legend);
    }

    public override void Layout(float canvasWidth, float canvasHeight)
    {
        base.Layout(canvasWidth, canvasHeight);

        // Legend pinned to the left edge, centered vertically; the button window stays
        // canvas-centered (base.Layout).
        _legend.Offset = new SKPoint(
            LegendLeftInset,
            MathF.Round((canvasHeight - _legend.Size.Height) / 2f));
        _legend.Layout(canvasWidth, canvasHeight);
    }
}
