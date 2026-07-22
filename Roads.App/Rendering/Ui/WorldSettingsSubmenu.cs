using SkiaSharp;
using Roads.App.Editor;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The World Settings panel shown under the menu bar's World button while
/// <see cref="EditorState.WorldSettingsMenuOpen"/> is true: per-world settings that save
/// INSIDE the .roads file (unlike the Settings dialog's per-user settings). Currently the
/// automatic through-traffic spawning at entry/exit nodes — an on/off switch, the
/// population-rate multiplier, a population-independent base rate, and the rush-hour
/// curve switch. Controls bind live to the shared <see cref="WorldSettings"/> instance
/// (the same reference PopulationManager reads every tick and MapSerializer saves/loads),
/// so edits apply immediately and persist with the next save. Visibility is a live
/// <see cref="Panel.VisibleWhen"/> gate; the background pad consumes clicks.
/// </summary>
public class WorldSettingsSubmenu : Panel
{
    private const float PanelWidth = 240f;
    private const float Pad = 8f;
    private const float TitleHeight = 20f;
    private const float CheckRowHeight = 24f;
    private const float SliderRowHeight = 36f;
    /// <summary>Label baseline sits this far above the slider track (SettingsDialog geometry).</summary>
    private const float SliderLabelHeight = 14f;
    // Slider's own hit padding (see Slider.HitPadX/HitPadY): bounds extend this far
    // beyond the visible track, so offsets below compensate to keep tracks aligned.
    private const float SliderHitPadX = 4f;
    private const float SliderHitPadY = 6f;
    private const float SliderTrackHeight = 10f;

    public WorldSettingsSubmenu(EditorState editorState, WorldSettings world)
    {
        VisibleWhen = () => editorState.WorldSettingsMenuOpen;

        float contentWidth = PanelWidth - 2f * Pad;
        float y = Pad;

        Add(new Label
        {
            Text = "WORLD SETTINGS",
            Font = UiTheme.Font12,
            TextColor = UiTheme.TextPrimary,
            Offset = new SKPoint(Pad, y),
            Size = new SKSize(contentWidth, TitleHeight),
            TextOffset = new SKPoint(0f, 4f),
        });
        y += TitleHeight;

        Add(new Checkbox("Through traffic (entry/exit)",
            () => world.ThroughTrafficEnabled, v => world.ThroughTrafficEnabled = v)
        {
            Offset = new SKPoint(Pad, y),
            Size = new SKSize(contentWidth, CheckRowHeight),
        });
        y += CheckRowHeight;

        AddSlider(y, contentWidth, new Slider("Population rate multiplier",
            0f, WorldSettings.MaxMultiplier,
            () => world.ThroughTrafficMultiplier, v => world.ThroughTrafficMultiplier = v)
        { ValueFormat = "F1", Step = 0.1f });
        y += SliderRowHeight;

        AddSlider(y, contentWidth, new Slider("Base rate (cars/min)",
            0f, WorldSettings.MaxBaseCarsPerMin,
            () => world.ThroughTrafficBaseCarsPerMin, v => world.ThroughTrafficBaseCarsPerMin = v)
        { ValueFormat = "F0", Step = 1f });
        y += SliderRowHeight;

        Add(new Checkbox("Rush-hour variation",
            () => world.RushHourVariation, v => world.RushHourVariation = v)
        {
            Offset = new SKPoint(Pad, y),
            Size = new SKSize(contentWidth, CheckRowHeight),
        });
        y += CheckRowHeight;

        // Anchored under the World action button (index 2), padded like the tool submenus.
        Margin = new SKPoint(MenuBar.ActionButtonLeft(2) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(PanelWidth, y + Pad);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
    }

    /// <summary>Places a slider row: label line above the track, the slider's bounds
    /// being the generous hit box around the track (the SettingsDialog row geometry).</summary>
    private void AddSlider(float rowY, float contentWidth, Slider slider)
    {
        float trackTop = rowY + SliderLabelHeight + 2f;
        slider.Offset = new SKPoint(Pad - SliderHitPadX, trackTop - SliderHitPadY);
        slider.Size = new SKSize(contentWidth + SliderHitPadX, SliderTrackHeight + 2f * SliderHitPadY);
        Add(slider);
    }
}
