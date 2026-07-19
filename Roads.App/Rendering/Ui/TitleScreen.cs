using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The title screen shown over the live menu-map backdrop at startup (and after Return
/// to Title): the <see cref="ModalMenu"/> scrim dims the running simulation, the
/// <see cref="RoadsLogo"/> draws above a centered window of New / Load / Settings / Exit
/// buttons with a small credit line beneath it and the assembly version bottom-center
/// of the screen, and the window is shifted down so the logo + window block centers as
/// a unit.
/// The owner (MainForm) supplies the four actions, keeps every in-game panel hidden and
/// all editor input gated while this is visible, and closes it via EnterGame; the
/// SettingsDialog sits above this panel in both hit-testing and paint order, so Settings
/// opens in front with the title menu dimmed behind its scrim.
/// </summary>
public class TitleScreen : ModalMenu
{
    private const float LogoHeight = 160f;
    private const float LogoMaxWidth = 600f;
    private const float LogoGap = 28f;
    private const string Subtext = "Pepperell Games - http://pepperell.net/games/";
    /// <summary>Baseline distance of the credit line below the window's bottom edge.</summary>
    private const float SubtextGap = 26f;
    /// <summary>Assembly version (0.1.0.&lt;auto-incremented build number&gt; — see the
    /// csproj IncrementBuildNumber target), shown bottom-center of the screen.</summary>
    private static readonly string VersionText =
        $"v{typeof(TitleScreen).Assembly.GetName().Version?.ToString() ?? "?"}";

    public TitleScreen(Action onNew, Action onLoad, Action onSettings, Action onExit)
    {
        AddMenuButton("New", onNew);
        AddMenuButton("Load", onLoad);
        AddMenuButton("Settings", onSettings);
        AddMenuButton("Exit", onExit);
    }

    protected override float WindowCenterYOffset => (LogoHeight + LogoGap) / 2f;

    /// <summary>Draws the logo in the band above the window and the credit line beneath
    /// it. Window.Bounds is valid here: UiRoot lays out every panel (ExternallyDrawn
    /// included) before MainForm paints this one, and OnDraw runs after this panel's own
    /// Layout in the same pass.</summary>
    protected override void OnDraw(SKCanvas canvas)
    {
        float bottom = Window.Bounds.Top - LogoGap;
        float top = MathF.Max(8f, bottom - LogoHeight);
        if (bottom > top)
        {
            float width = MathF.Min(LogoMaxWidth, Bounds.Width - 40f);
            RoadsLogo.Draw(canvas, SKRect.Create(Bounds.MidX - width / 2f, top, width, bottom - top));
        }

        UiTheme.TextScratch.Color = UiTheme.TextDim;
        canvas.DrawText(Subtext, Bounds.MidX, Window.Bounds.Bottom + SubtextGap,
            SKTextAlign.Center, UiTheme.Font13, UiTheme.TextScratch);

        canvas.DrawText(VersionText, Bounds.MidX, Bounds.Bottom - 12f,
            SKTextAlign.Center, UiTheme.Font11, UiTheme.TextScratch);
    }
}
