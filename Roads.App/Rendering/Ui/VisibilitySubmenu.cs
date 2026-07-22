using SkiaSharp;
using Roads.App.Core;
using Roads.App.Editor;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The view-toggle submenu shown under the menu bar's Visibility button while
/// <see cref="EditorState.VisibilityMenuOpen"/> is true: a vertical list of toggle
/// <see cref="Button"/>s for the panels (Perf HUD / Minimap / Statistics), the congestion
/// heat-map overlay, and the alignment grid — highlighted while the corresponding setting
/// is on. Each click routes through MainForm's MutateSettings, so a toggle here applies
/// live AND persists to settings.json, staying in sync with the equivalent hotkeys
/// (P / M / N / H). Visibility is a live <see cref="Panel.VisibleWhen"/> gate, which
/// covers both drawing and hit-testing; the background pad consumes clicks.
/// </summary>
public class VisibilitySubmenu : Panel
{
    private const float ButtonWidth = 110f;
    private const float ButtonHeight = 24f;
    private const float ButtonSpacing = 3f;
    private const float Pad = 4f;

    public VisibilitySubmenu(EditorState editorState,
        Func<AppSettings> settings, Action<Action<AppSettings>> mutateSettings)
    {
        VisibleWhen = () => editorState.VisibilityMenuOpen;

        var toggles = new (string Label, Func<bool> Get, Action<AppSettings, bool> Set)[]
        {
            ("Perf HUD",   () => settings().ShowPerformanceHud, (s, v) => s.ShowPerformanceHud = v),
            ("Minimap",    () => settings().ShowMinimap,        (s, v) => s.ShowMinimap = v),
            ("Statistics", () => settings().ShowStatistics,     (s, v) => s.ShowStatistics = v),
            ("Heat-map",   () => settings().HeatMapEnabled,     (s, v) => s.HeatMapEnabled = v),
            ("Grid",       () => settings().ShowGrid,           (s, v) => s.ShowGrid = v),
        };

        for (int i = 0; i < toggles.Length; i++)
        {
            var (label, get, set) = toggles[i];
            var button = new Button
            {
                Text = label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ButtonWidth, ButtonHeight),
                Offset = new SKPoint(Pad, Pad + i * (ButtonHeight + ButtonSpacing)),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = get,
            };
            button.Click += () => mutateSettings(s => set(s, !get()));
            Add(button);
        }

        // Anchored under the Visibility action button (index 1), padded like the tool submenus.
        Margin = new SKPoint(MenuBar.ActionButtonLeft(1) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(
            ButtonWidth + 2f * Pad,
            toggles.Length * ButtonHeight + (toggles.Length - 1) * ButtonSpacing + 2f * Pad);
        BackgroundColor = UiTheme.PanelBackground;
    }
}
