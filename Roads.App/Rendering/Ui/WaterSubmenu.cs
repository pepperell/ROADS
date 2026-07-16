using SkiaSharp;
using Roads.App.Editor;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The water-tools submenu shown under the Water toolbar button while the Water tool is
/// active. Row 1 holds the mode group (<b>Brush</b> paints dabs on click/drag,
/// <b>Stream</b> draws segment chains like the Road tool, <b>Erase</b> removes
/// intersecting primitives) — switching modes clears any in-progress stroke or pending
/// stream anchor so a stale chain never carries across. Row 2 holds the sticky options:
/// the size group (S/M/L/XL = 4/8/16/32 m brush radius; streams commit at 2× that
/// width, so mid-chain size changes give a varying-width stream) and the
/// Straight/Curved drawing-mode group (curved = each new segment leaves its start
/// tangent to the previous one; meaningful for Stream mode only, so it is disabled
/// otherwise). Visibility is a live <see cref="Panel.VisibleWhen"/> gate on the active
/// tool, covering both drawing and hit-testing. The background pad consumes clicks.
/// </summary>
public class WaterSubmenu : Panel
{
    private const float ModeButtonWidth = 70f;
    private const float SizeButtonWidth = 30f;
    private const float CurveButtonWidth = 55f;
    private const float ButtonHeight = 24f;
    private const float ButtonSpacing = 3f;
    private const float GroupGap = 12f;
    private const float RowSpacing = 4f;
    private const float Pad = 4f;

    public WaterSubmenu(EditorState editorState)
    {
        VisibleWhen = () => editorState.ActiveTool == EditorTool.Water;

        // ── Row 1: mode group ────────────────────────────────────────────
        var modes = new[]
        {
            (label: "Brush", mode: WaterMode.Brush),
            (label: "Stream", mode: WaterMode.Stream),
            (label: "Erase", mode: WaterMode.Erase),
        };
        for (int i = 0; i < modes.Length; i++)
        {
            var mode = modes[i].mode;
            var button = new Button
            {
                Text = modes[i].label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ModeButtonWidth, ButtonHeight),
                Offset = new SKPoint(Pad + i * (ModeButtonWidth + ButtonSpacing), Pad),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = () => editorState.WaterMode == mode,
            };
            button.Click += () =>
            {
                editorState.WaterMode = mode;
                // Never carry an in-progress stroke or chain into another mode.
                editorState.IsPaintingWater = false;
                editorState.WaterLastDabPos = null;
                editorState.WaterStreamAnchor = null;
                editorState.WaterStreamPrevDir = null;
            };
            Add(button);
        }
        float row1Width = modes.Length * ModeButtonWidth + (modes.Length - 1) * ButtonSpacing;

        // ── Row 2: sticky size + drawing-mode options ────────────────────
        float rowY = Pad + ButtonHeight + RowSpacing;
        float x = Pad;

        var sizes = new[]
        {
            (label: "S", radius: 4f),
            (label: "M", radius: 8f),
            (label: "L", radius: 16f),
            (label: "XL", radius: 32f),
        };
        foreach (var (label, radius) in sizes)
        {
            float r = radius;
            var button = new Button
            {
                Text = label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(SizeButtonWidth, ButtonHeight),
                Offset = new SKPoint(x, rowY),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = () => editorState.WaterBrushRadius == r,
            };
            button.Click += () => editorState.WaterBrushRadius = r;
            Add(button);
            x += SizeButtonWidth + ButtonSpacing;
        }
        x += GroupGap - ButtonSpacing;

        // Drawing-mode group (Stream mode only; the EditorState field is independent of
        // the road tool's SelectedCurved).
        var curveModes = new[] { (label: "Straight", curved: false), (label: "Curved", curved: true) };
        foreach (var (label, curved) in curveModes)
        {
            bool isCurved = curved;
            var button = new Button
            {
                Text = label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(CurveButtonWidth, ButtonHeight),
                Offset = new SKPoint(x, rowY),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                Disabled = new ButtonColors(new SKColor(40, 42, 48), new SKColor(110, 110, 110)),
                IsActive = () => editorState.WaterCurved == isCurved,
                IsEnabled = () => editorState.WaterMode == WaterMode.Stream,
            };
            button.Click += () => editorState.WaterCurved = isCurved;
            Add(button);
            x += CurveButtonWidth + ButtonSpacing;
        }
        float row2Width = x - ButtonSpacing - Pad;

        // Anchored under the Water tool button (index 4), padded like the other submenus.
        Margin = new SKPoint(MenuBar.ToolButtonLeft(4) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(
            MathF.Max(row1Width, row2Width) + 2f * Pad,
            2f * ButtonHeight + RowSpacing + 2f * Pad);
        BackgroundColor = UiTheme.PanelBackground;
    }
}
