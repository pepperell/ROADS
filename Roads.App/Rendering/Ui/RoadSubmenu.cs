using SkiaSharp;
using Roads.App.Editor;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The road-tools submenu shown under the Road toolbar button while any road-family tool
/// (Road / Node / Delete / UpdateSegment) is active. Row 1 holds the tool buttons
/// (Add Road / Add Node / Delete / Update Seg — the latter three live ONLY here, not on
/// the main toolbar). Row 2 holds the sticky road options applied by the Road and Update
/// Segment tools: road-type group, per-direction width group (1x–3x; disabled beyond 1x
/// while shared-lane is checked, since a shared lane is single-lane by definition), and
/// <see cref="Checkbox"/>es for one-way and single-lane two-way (mutually exclusive —
/// the <see cref="EditorState"/> setters enforce it; the live accessors make the visuals
/// follow). Visibility is a live <see cref="Panel.VisibleWhen"/> gate on the active tool,
/// covering both drawing and hit-testing. The background pad consumes clicks.
/// </summary>
public class RoadSubmenu : Panel
{
    private const float ToolButtonWidth = 70f;
    private const float SizeButtonWidth = 55f;
    private const float WidthButtonWidth = 30f;
    private const float ButtonHeight = 24f;
    private const float ButtonSpacing = 3f;
    private const float GroupGap = 12f;
    private const float RowSpacing = 4f;
    private const float Pad = 4f;
    private const float OneWayCheckboxWidth = 78f;
    private const float SharedCheckboxWidth = 122f;

    public RoadSubmenu(EditorState editorState)
    {
        VisibleWhen = () => editorState.ActiveTool
            is EditorTool.Road or EditorTool.Node or EditorTool.Delete or EditorTool.UpdateSegment;

        // ── Row 1: tool buttons ──────────────────────────────────────────
        var tools = new[]
        {
            (label: "Add Road", tool: EditorTool.Road),
            (label: "Add Node", tool: EditorTool.Node),
            (label: "Delete", tool: EditorTool.Delete),
            (label: "Update Seg", tool: EditorTool.UpdateSegment),
        };
        for (int i = 0; i < tools.Length; i++)
        {
            var tool = tools[i].tool;
            var button = new Button
            {
                Text = tools[i].label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ToolButtonWidth, ButtonHeight),
                Offset = new SKPoint(Pad + i * (ToolButtonWidth + ButtonSpacing), Pad),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                IsActive = () => editorState.ActiveTool == tool,
            };
            button.Click += () =>
            {
                editorState.ResetToolState();
                editorState.ActiveTool = tool;
            };
            Add(button);
        }
        float row1Width = tools.Length * ToolButtonWidth + (tools.Length - 1) * ButtonSpacing;

        // ── Row 2: sticky road options ───────────────────────────────────
        float rowY = Pad + ButtonHeight + RowSpacing;
        float x = Pad;

        // Road-type group.
        var sizes = new[]
        {
            (label: "Resid", type: RoadType.Residential),
            (label: "Arterial", type: RoadType.Arterial),
            (label: "Hwy", type: RoadType.Highway),
            (label: "Dirt", type: RoadType.Dirt),
        };
        foreach (var (label, type) in sizes)
        {
            var roadType = type;
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
                IsActive = () => editorState.SelectedRoadType == roadType,
            };
            button.Click += () => editorState.SelectedRoadType = roadType;
            Add(button);
            x += SizeButtonWidth + ButtonSpacing;
        }
        x += GroupGap - ButtonSpacing;

        // Width group (per-direction lane count). Disabled beyond 1x while shared-lane is
        // checked; the EditorState setter pins the count to 1 in that state.
        for (int n = 1; n <= 3; n++)
        {
            int lanes = n;
            var button = new Button
            {
                Text = $"{n}x",
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(WidthButtonWidth, ButtonHeight),
                Offset = new SKPoint(x, rowY),
                Idle = new ButtonColors(new SKColor(55, 58, 65), new SKColor(180, 180, 180)),
                Hover = new ButtonColors(new SKColor(75, 80, 90), SKColors.White),
                Active = new ButtonColors(new SKColor(60, 130, 200), SKColors.White),
                Disabled = new ButtonColors(new SKColor(40, 42, 48), new SKColor(110, 110, 110)),
                IsActive = () => editorState.SelectedLaneCount == lanes,
                IsEnabled = () => lanes == 1 || !editorState.SelectedSharedLane,
            };
            button.Click += () => editorState.SelectedLaneCount = (byte)lanes;
            Add(button);
            x += WidthButtonWidth + ButtonSpacing;
        }
        x += GroupGap - ButtonSpacing;

        // Checkboxes (mutually exclusive via the EditorState setters).
        Add(new Checkbox("One-way",
            () => editorState.SelectedOneWay, v => editorState.SelectedOneWay = v)
        {
            Size = new SKSize(OneWayCheckboxWidth, ButtonHeight),
            Offset = new SKPoint(x, rowY),
        });
        x += OneWayCheckboxWidth + ButtonSpacing;
        Add(new Checkbox("Single-lane 2-way",
            () => editorState.SelectedSharedLane, v => editorState.SelectedSharedLane = v)
        {
            Size = new SKSize(SharedCheckboxWidth, ButtonHeight),
            Offset = new SKPoint(x, rowY),
        });
        float row2Width = x + SharedCheckboxWidth - Pad;

        // Anchored under the Road tool button (index 1), padded like the POI submenu.
        Margin = new SKPoint(MenuBar.ToolButtonLeft(1) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(
            MathF.Max(row1Width, row2Width) + 2f * Pad,
            2f * ButtonHeight + RowSpacing + 2f * Pad);
        BackgroundColor = UiTheme.PanelBackground;
    }
}
