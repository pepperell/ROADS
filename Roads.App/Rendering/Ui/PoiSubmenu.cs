using SkiaSharp;
using Roads.App.Editor;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The POI-type submenu shown under the Dest Pt toolbar button while the Destination tool
/// is active: seven <see cref="Button"/>s (Home / Work / Shop / Leisure / School /
/// Parking / Ent/Exit) colored from <see cref="UiTheme.PoiColors"/> — full-saturation when
/// selected, half-tone otherwise. Visibility is a live <see cref="Panel.VisibleWhen"/>
/// gate on the active tool, which covers both drawing and hit-testing (no per-frame sync,
/// no manual gate at the input call site). The background pad consumes clicks.
/// </summary>
public class PoiSubmenu : Panel
{
    private const float ButtonWidth = 55f;
    private const float ButtonHeight = 24f;
    private const float ButtonSpacing = 3f;
    private const float Pad = 4f;

    private static readonly string[] PoiLabels =
        { "Home", "Work", "Shop", "Leisure", "School", "Parking", "Ent/Exit" };

    public PoiSubmenu(EditorState editorState)
    {
        VisibleWhen = () => editorState.ActiveTool == EditorTool.Destination;

        for (int i = 0; i < PoiLabels.Length; i++)
        {
            var poiType = (POIType)(i + 1); // POIType.Home = 1, Work = 2, ...
            var c = UiTheme.PoiColors[i];
            var button = new Button
            {
                Text = PoiLabels[i],
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ButtonWidth, ButtonHeight),
                Offset = new SKPoint(Pad + i * (ButtonWidth + ButtonSpacing), Pad),
                Idle = new ButtonColors(
                    new SKColor((byte)(c.Red / 2), (byte)(c.Green / 2), (byte)(c.Blue / 2), 200),
                    new SKColor(160, 160, 160)),
                Active = new ButtonColors(new SKColor(c.Red, c.Green, c.Blue, 255), SKColors.White),
                IsActive = () => editorState.SelectedPOIType == poiType,
            };
            // The retired submenu had no hover tint distinct from idle; keep parity.
            button.Hover = button.Idle;
            button.Click += () => editorState.SelectedPOIType = poiType;
            Add(button);
        }

        // Anchored under the Dest Pt tool button (index 3), padded like the old backdrop.
        Margin = new SKPoint(MenuBar.ToolButtonLeft(3) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(
            PoiLabels.Length * ButtonWidth + (PoiLabels.Length - 1) * ButtonSpacing + 2f * Pad,
            ButtonHeight + 2f * Pad);
        BackgroundColor = UiTheme.PanelBackground;
    }
}
