using SkiaSharp;
using Roads.App.Core;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The modal Settings dialog: a full-canvas translucent scrim (this panel itself — it
/// consumes every left click so nothing behind it receives input; middle/right/wheel are
/// gated in MainForm, which also pauses the simulation while the dialog is open in-game —
/// opened from the title screen the backdrop sim keeps running) with a centered window of
/// tabbed pages (Graphics / Simulation / Driving / Audio / Music / Debug) built from the
/// standard retained-mode controls.
///
/// Edits go to a STAGED copy of <see cref="AppSettings"/> taken on <see cref="Open"/>;
/// nothing touches the live systems until Apply/OK. The Apply button's IsEnabled closure
/// compares the staged record against the owner's last-applied record by value every
/// frame, so it enables the moment any setting on any page diverges and dims again when
/// everything matches — no change events needed. OK applies (if dirty) and closes;
/// Cancel (button or Escape, routed by MainForm) discards the staged copy and closes.
///
/// Must be added to the <see cref="UiRoot"/> LAST (topmost hit-testing — above the title
/// screen and pause menu, so Settings opens in front of either with its scrim dimming
/// them) and, because it is <see cref="Panel.ExternallyDrawn"/>, painted by
/// MainForm.OnPaintSurface AFTER the performance HUD and both menus so it draws above
/// every other overlay.
/// </summary>
public class SettingsDialog : Panel
{
    private const float WindowWidth = 520f;
    private const float WindowHeight = 430f;
    private const float Pad = 20f;
    private const float TitleY = 14f;
    private const float TabRowY = 44f;
    private const float TabWidth = 74f; // six tabs must fit inside the window padding
    private const float TabHeight = 26f;
    private const float TabSpacing = 4f;
    private const float PageY = 82f;
    private const float ActionButtonWidth = 70f;
    private const float ActionButtonHeight = 26f;
    private const float ActionSpacing = 8f;

    // Slider row geometry (the SliderPanel's historical grab feel).
    private const float RowHeight = 36f;
    private const float CheckRowHeight = 28f;
    private const float SliderHeight = 16f;
    private const float LabelHeight = 14f;
    private const float HitPadX = 4f;
    private const float HitPadY = 6f;

    private enum Tab { Graphics, Simulation, Driving, Audio, Music, Debug }

    private Tab _tab = Tab.Graphics;
    private AppSettings _staged = new();
    private readonly Func<AppSettings> _getApplied;
    private readonly Action<AppSettings> _applyAndSave;
    private readonly Action _onClosed;
    private readonly Panel _window;

    /// <summary>True when the staged copy differs (by value) from the last-applied settings.</summary>
    private bool IsDirty => !_staged.Equals(_getApplied());

    /// <param name="getApplied">Live accessor for the owner's last-APPLIED settings record.</param>
    /// <param name="applyAndSave">Adopts the given record as applied: pushes it to the live
    /// systems and persists it (MainForm.ApplyStagedSettings).</param>
    /// <param name="onClosed">Invoked whenever the dialog closes (any path) — the owner
    /// restores the pre-dialog pause state here.</param>
    public SettingsDialog(Func<AppSettings> getApplied, Action<AppSettings> applyAndSave, Action onClosed)
    {
        _getApplied = getApplied;
        _applyAndSave = applyAndSave;
        _onClosed = onClosed;

        // The dialog itself is the scrim: full-canvas (see Layout), swallows every click.
        Visible = false;
        ExternallyDrawn = true;
        BackgroundColor = new SKColor(0, 0, 0, 130);
        CornerRadius = 0f;

        _window = new Panel
        {
            Size = new SKSize(WindowWidth, WindowHeight),
            BackgroundColor = UiTheme.PanelBackground,
            BorderColor = UiTheme.Outline,
        };
        Add(_window);

        _window.Add(new Label
        {
            Text = "Settings",
            Font = UiTheme.Font14,
            TextColor = SKColors.White,
            Offset = new SKPoint(Pad, TitleY),
            Size = new SKSize(200f, 18f),
        });

        BuildTabRow();

        var graphics = AddPage(Tab.Graphics);
        AddCheckRow(graphics, 0, "Fullscreen (borderless)",
            () => _staged.Fullscreen, v => _staged.Fullscreen = v);
        AddCheckRow(graphics, 1, "Show alignment grid",
            () => _staged.ShowGrid, v => _staged.ShowGrid = v);
        AddCheckRow(graphics, 2, "Congestion heat-map",
            () => _staged.HeatMapEnabled, v => _staged.HeatMapEnabled = v);
        AddCheckRow(graphics, 3, "Performance HUD",
            () => _staged.ShowPerformanceHud, v => _staged.ShowPerformanceHud = v);
        AddCheckRow(graphics, 4, "Minimap",
            () => _staged.ShowMinimap, v => _staged.ShowMinimap = v);
        AddCheckRow(graphics, 5, "Statistics panel",
            () => _staged.ShowStatistics, v => _staged.ShowStatistics = v);
        AddCheckRow(graphics, 6, "Shortcut legend",
            () => _staged.ShowLegend, v => _staged.ShowLegend = v);

        var simulation = AddPage(Tab.Simulation);
        AddSliderRow(simulation, 0, "Max vehicles", 50f, 2000f,
            () => _staged.MaxVehicles, v => _staged.MaxVehicles = (int)MathF.Round(v),
            format: "F0", step: 10f);
        AddSliderRow(simulation, 1, "Game seconds per real second (at 1x)", 1f, 60f,
            () => (float)_staged.GameSecondsPerRealSecond, v => _staged.GameSecondsPerRealSecond = MathF.Round(v),
            format: "F0", step: 1f);
        AddSliderRow(simulation, 2, "Autosave interval (s)", 30f, 1800f,
            () => (float)_staged.AutosaveIntervalSeconds, v => _staged.AutosaveIntervalSeconds = MathF.Round(v),
            format: "F0", step: 30f);
        AddSliderRow(simulation, 3, "Autosave backups kept", 1f, 20f,
            () => _staged.AutosaveMaxBackups, v => _staged.AutosaveMaxBackups = (int)MathF.Round(v),
            format: "F0", step: 1f);

        var driving = AddPage(Tab.Driving);
        AddSliderRow(driving, 0, "Kp", 0.5f, 10f, () => _staged.Kp, v => _staged.Kp = v);
        AddSliderRow(driving, 1, "Kd", 0f, 5f, () => _staged.Kd, v => _staged.Kd = v);
        AddSliderRow(driving, 2, "Max Steer", 0.1f, 1.5f, () => _staged.MaxSteer, v => _staged.MaxSteer = v);
        AddSliderRow(driving, 3, "Target Speed", 1f, 30f, () => _staged.TargetSpeed, v => _staged.TargetSpeed = v);
        AddSliderRow(driving, 4, "Lookahead Base", 0.5f, 15f, () => _staged.LookaheadBase, v => _staged.LookaheadBase = v);
        AddSliderRow(driving, 5, "Lookahead/Speed", 0f, 2f, () => _staged.LookaheadPerSpeed, v => _staged.LookaheadPerSpeed = v);
        AddSliderRow(driving, 6, "Lateral Gain", 0f, 3f, () => _staged.Klat, v => _staged.Klat = v);

        var audio = AddPage(Tab.Audio);
        AddCheckRow(audio, 0, "Sound enabled",
            () => _staged.SoundEnabled, v => _staged.SoundEnabled = v);
        AddCheckRow(audio, 1, "Ambient traffic hum",
            () => _staged.AmbientHumEnabled, v => _staged.AmbientHumEnabled = v);
        AddCheckRow(audio, 2, "Engine sounds (zoomed in)",
            () => _staged.EngineSoundsEnabled, v => _staged.EngineSoundsEnabled = v);
        AddCheckRow(audio, 3, "Event sounds (horns, screeches, signal ticks)",
            () => _staged.EventSoundsEnabled, v => _staged.EventSoundsEnabled = v);
        // Slider row 3's track top (3*36+16=124) clears the last check row (4*28=112).
        AddSliderRow(audio, 3, "Master volume", 0f, 1f,
            () => _staged.MasterVolume, v => _staged.MasterVolume = v,
            format: "F2", step: 0.05f);

        // Music: the generative-jazz tuning page. Check row 0 ends at y 24; slider rows
        // start at index 1 (track top 52) so the check row clears the first slider label.
        var music = AddPage(Tab.Music);
        AddCheckRow(music, 0, "Background music (generative jazz)",
            () => _staged.MusicEnabled, v => _staged.MusicEnabled = v);
        AddSliderRow(music, 1, "Music volume", 0f, 1f,
            () => _staged.MusicVolume, v => _staged.MusicVolume = v,
            format: "F2", step: 0.05f);
        AddSliderRow(music, 2, "Tempo (BPM)", 72f, 132f,
            () => _staged.MusicTempoBpm, v => _staged.MusicTempoBpm = MathF.Round(v),
            format: "F0", step: 2f);
        AddSliderRow(music, 3, "Swing feel (straight - triplet)", 0f, 1f,
            () => _staged.MusicSwing, v => _staged.MusicSwing = v,
            format: "F2", step: 0.05f);
        AddSliderRow(music, 4, "Traffic drives band energy", 0f, 1f,
            () => _staged.MusicTrafficResponse, v => _staged.MusicTrafficResponse = v,
            format: "F2", step: 0.05f);
        AddSliderRow(music, 5, "Night mellows the band", 0f, 1f,
            () => _staged.MusicNightResponse, v => _staged.MusicNightResponse = v,
            format: "F2", step: 0.05f);
        AddSliderRow(music, 6, "Congestion adds tension", 0f, 1f,
            () => _staged.MusicTensionResponse, v => _staged.MusicTensionResponse = v,
            format: "F2", step: 0.05f);

        var debug = AddPage(Tab.Debug);
        AddCheckRow(debug, 0, "Arc-conflict overlay",
            () => _staged.ShowArcConflicts, v => _staged.ShowArcConflicts = v);
        AddCheckRow(debug, 1, "Steering debug logging",
            () => _staged.DebugLogging, v => _staged.DebugLogging = v);

        BuildActionRow();
    }

    /// <summary>Stages a fresh copy of the applied settings and shows the dialog.
    /// When opened in-game the owner pauses the simulation BEFORE calling this; from the
    /// title screen it deliberately does not (the backdrop keeps running).</summary>
    public void Open()
    {
        _staged = _getApplied() with { };
        _tab = Tab.Graphics;
        Visible = true;
    }

    /// <summary>Discards the staged copy and closes (Cancel button, and Escape via MainForm).</summary>
    public void Cancel()
    {
        Visible = false;
        _onClosed();
    }

    /// <summary>Commits the staged settings (a copy, so the dialog keeps editing its own
    /// record) — the owner pushes them live and persists. Apply disables next frame
    /// because applied now equals staged.</summary>
    private void Apply() => _applyAndSave(_staged with { });

    private void Ok()
    {
        if (IsDirty) Apply();
        Cancel();
    }

    /// <summary>Full-canvas scrim with the window centered — the framework has no center
    /// anchor, so bounds are computed here from the canvas size each frame.</summary>
    public override void Layout(float canvasWidth, float canvasHeight)
    {
        Bounds = SKRect.Create(0f, 0f, canvasWidth, canvasHeight);
        _window.Offset = new SKPoint(
            MathF.Round((canvasWidth - WindowWidth) / 2f),
            MathF.Round((canvasHeight - WindowHeight) / 2f));
        LayoutChildren(canvasWidth, canvasHeight);
    }

    private void BuildTabRow()
    {
        var tabs = new[]
        {
            (label: "Graphics", tab: Tab.Graphics),
            (label: "Simulation", tab: Tab.Simulation),
            (label: "Driving", tab: Tab.Driving),
            (label: "Audio", tab: Tab.Audio),
            (label: "Music", tab: Tab.Music),
            (label: "Debug", tab: Tab.Debug),
        };
        for (int i = 0; i < tabs.Length; i++)
        {
            var tab = tabs[i].tab;
            var button = new Button
            {
                Text = tabs[i].label,
                Font = UiTheme.Font12,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(TabWidth, TabHeight),
                Offset = new SKPoint(Pad + i * (TabWidth + TabSpacing), TabRowY),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = () => _tab == tab,
            };
            button.Click += () => _tab = tab;
            _window.Add(button);
        }
    }

    /// <summary>Adds an (initially empty) page container shown only while its tab is
    /// current. All pages share the same area below the tab row; the tree is build-once,
    /// so switching tabs is purely a visibility change.</summary>
    private Panel AddPage(Tab tab)
    {
        var page = new Panel
        {
            Offset = new SKPoint(Pad, PageY),
            Size = new SKSize(WindowWidth - 2f * Pad, WindowHeight - PageY - ActionButtonHeight - 2f * Pad),
            VisibleWhen = () => _tab == tab,
        };
        _window.Add(page);
        return page;
    }

    private static void AddCheckRow(Panel page, int row, string label, Func<bool> get, Action<bool> set)
    {
        page.Add(new Checkbox(label, get, set)
        {
            Offset = new SKPoint(0f, row * CheckRowHeight),
            Size = new SKSize(page.Size.Width, 24f),
        });
    }

    private static void AddSliderRow(Panel page, int row, string label, float min, float max,
        Func<float> get, Action<float> set, string format = "F2", float step = 0f)
    {
        // Same row geometry as the retired SliderPanel: label line above the track, the
        // slider's Bounds being the generous hit box around it.
        float trackTop = row * RowHeight + LabelHeight + 2f;
        page.Add(new Slider(label, min, max, get, set)
        {
            Offset = new SKPoint(-HitPadX, trackTop - HitPadY),
            Size = new SKSize(page.Size.Width + 2f * HitPadX, SliderHeight + 2f * HitPadY),
            ValueFormat = format,
            Step = step,
        });
    }

    private void BuildActionRow()
    {
        var idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180));
        var hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White);
        var disabled = new ButtonColors(new SKColor(40, 42, 48), new SKColor(110, 110, 110));
        float y = WindowHeight - ActionButtonHeight - Pad * 0.8f;

        var buttons = new (string label, Action onClick, Func<bool>? isEnabled)[]
        {
            ("OK", Ok, null),
            ("Cancel", Cancel, null),
            ("Apply", Apply, () => IsDirty),
        };
        for (int i = 0; i < buttons.Length; i++)
        {
            var (label, onClick, isEnabled) = buttons[i];
            var button = new Button
            {
                Text = label,
                Font = UiTheme.Font12,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ActionButtonWidth, ActionButtonHeight),
                Offset = new SKPoint(
                    WindowWidth - Pad - (buttons.Length - i) * (ActionButtonWidth + ActionSpacing) + ActionSpacing,
                    y),
                Idle = idle,
                Hover = hover,
                Disabled = disabled,
                IsEnabled = isEnabled,
            };
            button.Click += onClick;
            _window.Add(button);
        }
    }
}
