namespace Roads.App.Rendering.Ui;

/// <summary>
/// The in-game pause menu (Menu button, or Escape with nothing left to cancel): a
/// <see cref="ModalMenu"/> scrim over the paused simulation with Return to Game / Save /
/// Save As / Settings / Return to Title / Exit to Desktop. A dumb view — the owner
/// (MainForm) supplies every action, owns the pause capture/restore around Open/Close,
/// and hosts the confirmation dialogs (Return to Title and Exit both confirm). Save and
/// Save As leave the menu open; the SettingsDialog sits above this panel, so Settings
/// opens in front with the menu dimmed behind its scrim.
/// </summary>
public class PauseMenu : ModalMenu
{
    public PauseMenu(Action onReturnToGame, Action onSave, Action onSaveAs,
        Action onSettings, Action onReturnToTitle, Action onExit)
    {
        AddMenuButton("Return to Game", onReturnToGame);
        AddMenuButton("Save", onSave);
        AddMenuButton("Save As", onSaveAs);
        AddMenuButton("Settings", onSettings);
        AddMenuButton("Return to Title", onReturnToTitle);
        AddMenuButton("Exit to Desktop", onExit);
    }
}
