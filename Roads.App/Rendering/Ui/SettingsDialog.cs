using SkiaSharp;
using Roads.App.Core;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The modal Settings dialog: a full-canvas translucent scrim (this panel itself — it
/// consumes every left click so nothing behind it receives input; middle/right/wheel are
/// gated in MainForm, which also pauses the simulation while the dialog is open in-game —
/// opened from the title screen the backdrop sim keeps running) with a centered window of
/// tabbed pages (Graphics / Simulation / Driving / Audio / Music / Debug) built from the
/// standard retained-mode controls. Every page is a <see cref="ScrollablePanel"/>, so a
/// page may hold more rows than fit: a scrollbar appears on overflow (mouse wheel works
/// because MainForm routes the wheel through UiRoot before gating it on modals).
///
/// Edits go to a STAGED copy of <see cref="AppSettings"/> taken on <see cref="Open"/>;
/// nothing touches the live systems until Apply/OK — EXCEPT the Music page, whose every
/// control live-previews the staged record through the owner's previewLive callback
/// (audio engine only; the change lands at the next bar). Previews never move the
/// applied record or persist anything, and <see cref="Cancel"/> re-previews the applied
/// record so Cancel/Escape audibly restore. The Apply button's IsEnabled closure
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
    private readonly Action<AppSettings> _previewLive;
    private readonly Panel _window;

    /// <summary>True when the staged copy differs (by value) from the last-applied settings.
    /// Live previews never move the applied record, so this stays exactly "Apply would
    /// change something" even while the Music page is audibly previewing.</summary>
    private bool IsDirty => !_staged.Equals(_getApplied());

    /// <param name="getApplied">Live accessor for the owner's last-APPLIED settings record.</param>
    /// <param name="applyAndSave">Adopts the given record as applied: pushes it to the live
    /// systems and persists it (MainForm.ApplyStagedSettings).</param>
    /// <param name="onClosed">Invoked whenever the dialog closes (any path) — the owner
    /// restores the pre-dialog pause state here.</param>
    /// <param name="previewLive">Live-preview push of a record to the AUDIO ENGINE only —
    /// cheap, idempotent, no adoption, no persistence (MainForm.PreviewMusicSettings).
    /// Every Music-page setter calls it with the staged record; Cancel calls it with the
    /// applied record to restore.</param>
    public SettingsDialog(Func<AppSettings> getApplied, Action<AppSettings> applyAndSave, Action onClosed,
        Action<AppSettings> previewLive)
    {
        _getApplied = getApplied;
        _applyAndSave = applyAndSave;
        _onClosed = onClosed;
        _previewLive = previewLive;

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

        // Music: the generative-jazz page — fully LIVE (every setter is Live-wrapped, so
        // edits preview through the audio engine at the next bar with no Apply; Cancel
        // restores). A tall scrollable column: the feel sliders, the auto/manual mode,
        // the manual style/instrumentation section (inert while Auto — it stages fields
        // the auto path ignores), then the hierarchical mixer (categories + indented
        // per-instrument sub-strips; active in BOTH modes).
        var music = AddPage(Tab.Music);
        {
            float y = 0f;
            AddCheckRowAt(music, y, "Background music (generative jazz)",
                () => _staged.MusicEnabled, Live<bool>(v => _staged.MusicEnabled = v));
            y = 36f;
            AddSliderRowAt(music, y, "Music volume", 0f, 1f,
                () => _staged.MusicVolume, Live<float>(v => _staged.MusicVolume = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Tempo (BPM)", 72f, 132f,
                () => _staged.MusicTempoBpm, Live<float>(v => _staged.MusicTempoBpm = MathF.Round(v)),
                format: "F0", step: 2f);
            y += 36f;
            AddSliderRowAt(music, y, "Swing feel (straight - triplet)", 0f, 1f,
                () => _staged.MusicSwing, Live<float>(v => _staged.MusicSwing = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Traffic drives band energy", 0f, 1f,
                () => _staged.MusicTrafficResponse, Live<float>(v => _staged.MusicTrafficResponse = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Night mellows the band", 0f, 1f,
                () => _staged.MusicNightResponse, Live<float>(v => _staged.MusicNightResponse = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Congestion adds tension", 0f, 1f,
                () => _staged.MusicTensionResponse, Live<float>(v => _staged.MusicTensionResponse = v),
                format: "F2", step: 0.05f);
            y += 44f;

            AddSectionLabel(music, y, "Mode");
            y += 24f;
            AddSegmentedRowAt(music, y, null, new[] { "Auto-compose", "Manual" },
                () => _staged.MusicManualMode ? 1 : 0,
                Live<int>(v => _staged.MusicManualMode = v == 1));
            y += 30f;

            AddSectionLabel(music, y, "Manual style & instrumentation (Manual mode)");
            y += 24f;
            AddSliderRowAt(music, y, "Energy", 0f, 1f,
                () => _staged.MusicManualIntensity, Live<float>(v => _staged.MusicManualIntensity = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Night mood", 0f, 1f,
                () => _staged.MusicManualNight, Live<float>(v => _staged.MusicManualNight = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSliderRowAt(music, y, "Tension", 0f, 1f,
                () => _staged.MusicManualTension, Live<float>(v => _staged.MusicManualTension = v),
                format: "F2", step: 0.05f);
            y += 36f;
            AddSegmentedRowAt(music, y, "Form",
                new[] { "Blues", "M.Blues", "AABA", "Bossa", "Vamp", "Noct.", "Waltz" },
                () => (int)_staged.MusicManualForm,
                Live<int>(v => _staged.MusicManualForm = (MusicFormChoice)v));
            y += 42f;
            AddSegmentedRowAt(music, y, "Key",
                new[] { "Bb", "Eb", "F", "C", "Ab" },
                () => (int)_staged.MusicManualKey,
                Live<int>(v => _staged.MusicManualKey = (MusicKeyChoice)v));
            y += 42f;
            AddSegmentedRowAt(music, y, "Lead",
                new[] { "Alto", "Tenor", "Tpt", "Harm", "Vibes", "Clar", "Sop", "Flute" },
                () => (int)_staged.MusicManualLead,
                Live<int>(v => _staged.MusicManualLead = (MusicLeadChoice)v));
            y += 42f;
            AddSegmentedRowAt(music, y, "Comping",
                new[] { "E.Piano", "Organ", "Guitar" },
                () => (int)_staged.MusicManualComp,
                Live<int>(v => _staged.MusicManualComp = (MusicCompChoice)v));
            y += 42f;
            AddSegmentedRowAt(music, y, "Drum kit",
                new[] { "Standard", "Brushes" },
                () => (int)_staged.MusicManualKit,
                Live<int>(v => _staged.MusicManualKit = (MusicKitChoice)v));
            y += 50f;

            AddSectionLabel(music, y, "Mixer — M mute, S solo (both modes)");
            y += 24f;
            var strips = new (string Name, bool Indent, Func<MixerStrip> Get, Action<MixerStrip> Set)[]
            {
                ("Comping", false, () => _staged.MixComp, v => _staged.MixComp = v),
                ("E.Piano", true, () => _staged.MixCompEPiano, v => _staged.MixCompEPiano = v),
                ("Organ", true, () => _staged.MixCompOrgan, v => _staged.MixCompOrgan = v),
                ("Guitar", true, () => _staged.MixCompGuitar, v => _staged.MixCompGuitar = v),
                ("Bass", false, () => _staged.MixBass, v => _staged.MixBass = v),
                ("Acoustic", true, () => _staged.MixBassAcoustic, v => _staged.MixBassAcoustic = v),
                ("Finger", true, () => _staged.MixBassFinger, v => _staged.MixBassFinger = v),
                ("Lead", false, () => _staged.MixLead, v => _staged.MixLead = v),
                ("Alto sax", true, () => _staged.MixLeadAlto, v => _staged.MixLeadAlto = v),
                ("Tenor sax", true, () => _staged.MixLeadTenor, v => _staged.MixLeadTenor = v),
                ("Muted trumpet", true, () => _staged.MixLeadTrumpet, v => _staged.MixLeadTrumpet = v),
                ("Harmonica", true, () => _staged.MixLeadHarmonica, v => _staged.MixLeadHarmonica = v),
                ("Vibraphone", true, () => _staged.MixLeadVibes, v => _staged.MixLeadVibes = v),
                ("Clarinet", true, () => _staged.MixLeadClarinet, v => _staged.MixLeadClarinet = v),
                ("Soprano sax", true, () => _staged.MixLeadSoprano, v => _staged.MixLeadSoprano = v),
                ("Flute", true, () => _staged.MixLeadFlute, v => _staged.MixLeadFlute = v),
                ("Pad", false, () => _staged.MixPad, v => _staged.MixPad = v),
                ("Piano", false, () => _staged.MixPiano, v => _staged.MixPiano = v),
                ("Horns", false, () => _staged.MixHorns, v => _staged.MixHorns = v),
                ("Drums", false, () => _staged.MixDrums, v => _staged.MixDrums = v),
                ("Kick", true, () => _staged.MixDrumKick, v => _staged.MixDrumKick = v),
                ("Snare", true, () => _staged.MixDrumSnare, v => _staged.MixDrumSnare = v),
                ("Hi-hat", true, () => _staged.MixDrumHat, v => _staged.MixDrumHat = v),
                ("Ride", true, () => _staged.MixDrumRide, v => _staged.MixDrumRide = v),
                ("Crash", true, () => _staged.MixDrumCrash, v => _staged.MixDrumCrash = v),
                ("Toms", true, () => _staged.MixDrumToms, v => _staged.MixDrumToms = v),
                ("Shaker", true, () => _staged.MixDrumShaker, v => _staged.MixDrumShaker = v),
            };
            foreach (var (name, indent, get, set) in strips)
            {
                AddMixerStrip(music, y, name, indent, get, set);
                y += 34f;
            }
        }

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

    /// <summary>Discards the staged copy and closes (Cancel button, and Escape via
    /// MainForm). Because the Music page previews its edits live, Cancel first re-previews
    /// the APPLIED record, audibly restoring the pre-dialog state (idempotent — harmless
    /// when nothing was previewed).</summary>
    public void Cancel()
    {
        _previewLive(_getApplied());
        Visible = false;
        _onClosed();
    }

    /// <summary>Wraps a Music-page setter so every edit also live-previews the staged
    /// record through the audio engine (the change lands at the next bar).</summary>
    private Action<T> Live<T>(Action<T> set) => v =>
    {
        set(v);
        _previewLive(_staged);
    };

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

    /// <summary>Adds an (initially empty) scrollable page shown only while its tab is
    /// current. All pages share the same area below the tab row; the tree is build-once,
    /// so switching tabs is purely a visibility change. Rows go into the page's
    /// <see cref="ScrollablePanel.Content"/> sized to <see cref="PageContentWidth"/>, so
    /// a page can hold arbitrarily many rows — a scrollbar appears once they overflow.</summary>
    private ScrollablePanel AddPage(Tab tab)
    {
        var page = new ScrollablePanel
        {
            Offset = new SKPoint(Pad, PageY),
            Size = new SKSize(WindowWidth - 2f * Pad, WindowHeight - PageY - ActionButtonHeight - 2f * Pad),
            VisibleWhen = () => _tab == tab,
        };
        _window.Add(page);
        return page;
    }

    /// <summary>Row width inside a page: the page minus its always-reserved scrollbar lane.</summary>
    private static float PageContentWidth(ScrollablePanel page)
        => page.Size.Width - ScrollablePanel.GutterWidth;

    private static void AddCheckRow(ScrollablePanel page, int row, string label, Func<bool> get, Action<bool> set)
        => AddCheckRowAt(page, row * CheckRowHeight, label, get, set);

    private static void AddCheckRowAt(ScrollablePanel page, float y, string label, Func<bool> get, Action<bool> set)
    {
        page.Content.Add(new Checkbox(label, get, set)
        {
            Offset = new SKPoint(0f, y),
            Size = new SKSize(PageContentWidth(page), 24f),
        });
    }

    private static void AddSliderRow(ScrollablePanel page, int row, string label, float min, float max,
        Func<float> get, Action<float> set, string format = "F2", float step = 0f)
        => AddSliderRowAt(page, row * RowHeight, label, min, max, get, set, format, step);

    private static void AddSliderRowAt(ScrollablePanel page, float y, string label, float min, float max,
        Func<float> get, Action<float> set, string format = "F2", float step = 0f)
    {
        // Same row geometry as the retired SliderPanel: label line above the track, the
        // slider's Bounds being the generous hit box around it. The right-hand hit pad is
        // trimmed (width + 1×HitPadX, not 2×) so the hit slop never counts as horizontal
        // content overflow and summons a phantom scrollbar.
        float trackTop = y + LabelHeight + 2f;
        page.Content.Add(new Slider(label, min, max, get, set)
        {
            Offset = new SKPoint(-HitPadX, trackTop - HitPadY),
            Size = new SKSize(PageContentWidth(page) + HitPadX, SliderHeight + 2f * HitPadY),
            ValueFormat = format,
            Step = step,
        });
    }

    /// <summary>A dim section heading inside a page column (24 px of pitch).</summary>
    private static void AddSectionLabel(ScrollablePanel page, float y, string text)
    {
        page.Content.Add(new Label
        {
            Text = text,
            Font = UiTheme.Font12,
            TextColor = UiTheme.TextDim,
            TextOffset = new SKPoint(0f, 4f),
            Offset = new SKPoint(0f, y + 4f),
            Size = new SKSize(PageContentWidth(page), 16f),
        });
    }

    /// <summary>A segmented enum picker (the tab-row pattern): an optional 14 px label
    /// line, then one 20 px button per option sharing the row's width; exactly the
    /// option whose index <paramref name="get"/> returns highlights. The caller wraps
    /// <paramref name="set"/> with <see cref="Live{T}"/> when the row previews.</summary>
    private static void AddSegmentedRowAt(ScrollablePanel page, float y, string? label,
        string[] options, Func<int> get, Action<int> set)
    {
        float buttonsY = y;
        if (label != null)
        {
            page.Content.Add(new Label
            {
                Text = label,
                Font = UiTheme.Font11,
                TextColor = UiTheme.TextDim,
                TextOffset = new SKPoint(0f, 3f),
                Offset = new SKPoint(0f, y),
                Size = new SKSize(PageContentWidth(page), 12f),
            });
            buttonsY = y + 16f;
        }
        int n = options.Length;
        float bw = MathF.Floor((PageContentWidth(page) - 4f * (n - 1)) / n);
        for (int i = 0; i < n; i++)
        {
            int idx = i;
            var button = new Button
            {
                Text = options[i],
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 3.5f),
                Size = new SKSize(bw, 20f),
                Offset = new SKPoint(i * (bw + 4f), buttonsY),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = () => get() == idx,
            };
            button.Click += () => set(idx);
            page.Content.Add(button);
        }
    }

    /// <summary>One mixer strip (34 px pitch): a compact volume slider plus M(ute) and
    /// S(olo) toggle buttons at the right edge. Sub-strips indent under their category.
    /// All three controls live-preview on change.</summary>
    private void AddMixerStrip(ScrollablePanel page, float y, string name, bool indent,
        Func<MixerStrip> get, Action<MixerStrip> set)
    {
        float x = indent ? 16f : 0f;
        float sliderRight = PageContentWidth(page) - 56f;
        page.Content.Add(new Slider(name, 0f, 1f,
            () => get().Volume, Live<float>(v => set(get() with { Volume = v })))
        {
            Offset = new SKPoint(x - HitPadX, y + 16f - HitPadY),
            Size = new SKSize(sliderRight - x + HitPadX, SliderHeight + 2f * HitPadY),
            ValueFormat = "F2",
            Step = 0.05f,
        });
        AddMixToggle(page, new SKPoint(PageContentWidth(page) - 52f, y + 15f), "M",
            new SKColor(190, 80, 80), () => get().Mute, () => set(get() with { Mute = !get().Mute }));
        AddMixToggle(page, new SKPoint(PageContentWidth(page) - 26f, y + 15f), "S",
            new SKColor(210, 170, 70), () => get().Solo, () => set(get() with { Solo = !get().Solo }));
    }

    private void AddMixToggle(ScrollablePanel page, SKPoint offset, string text, SKColor activeColor,
        Func<bool> isOn, Action toggle)
    {
        var button = new Button
        {
            Text = text,
            Font = UiTheme.Font11,
            TextOffset = new SKPoint(0f, 3.5f),
            Size = new SKSize(24f, 18f),
            Offset = offset,
            Idle = new ButtonColors(new SKColor(50, 53, 60), new SKColor(150, 150, 150)),
            Hover = new ButtonColors(new SKColor(70, 74, 84), SKColors.White),
            Active = new ButtonColors(activeColor, SKColors.White),
            IsActive = isOn,
        };
        button.Click += () =>
        {
            toggle();
            _previewLive(_staged);
        };
        page.Content.Add(button);
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
