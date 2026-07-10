using SkiaSharp;
using Roads.App.Editor;

namespace Roads.App.Rendering.Ui;

/// <summary>Identifies a UI action triggered by an action button (not a tool toggle).
/// New/Save/Load/Settings are raised by the <see cref="MenuBar"/>; Pause/SpeedDown/SpeedUp
/// by the <see cref="ClockPanel"/>'s transport buttons. All route to the same owner
/// callback (MainForm holds the dialogs and the sim loop).</summary>
public enum UIAction { New, Save, Load, Settings, Pause, SpeedDown, SpeedUp }

/// <summary>
/// The top-left menu bar: four action <see cref="Button"/>s (New / Save / Load / Settings)
/// at the far left, then four editor-tool toggle buttons (Select / Road / Dest Pt / Signal).
/// The Node / Delete / Update Seg tools live on the <see cref="RoadSubmenu"/> opened by
/// the Road button, and the Change Type / Control Type / Rotate tools on the
/// <see cref="SignalSubmenu"/> opened by the Signal button — not here. Tool buttons set
/// <see cref="EditorState.ActiveTool"/> directly (resetting transient tool state first)
/// and highlight via live IsActive closures; actions are routed to the owner through the
/// callback given at construction. Sits flush under the top edge (the row freed by the
/// retired status bar): 70×30 buttons at y=14 from x=10, 4 px spacing, a 16 px gap
/// between the action and tool groups, on a rounded background strip that consumes clicks.
/// </summary>
public class MenuBar : Panel
{
    private const float ButtonWidth = 70f;
    private const float ButtonHeight = 30f;
    private const float ButtonSpacing = 4f;
    private const float ToolbarX = 10f;
    private const float ToolbarY = 14f;
    private const float GroupGap = 16f;
    private const int ActionCount = 4;

    /// <summary>Left edge (canvas px) of the first tool button (after the action group).</summary>
    private static float ToolStartX
        => ToolbarX + ActionCount * (ButtonWidth + ButtonSpacing) + GroupGap;

    /// <summary>Left edge (canvas px) of the tool button at the given index — the road
    /// submenu anchors under the Road button (index 1), the POI submenu under the Dest Pt
    /// button (index 2), the signals submenu under the Signal button (index 3). Constant
    /// geometry.</summary>
    public static float ToolButtonLeft(int index) => ToolStartX + index * (ButtonWidth + ButtonSpacing);

    /// <summary>Y of the row below the toolbar where submenus anchor.</summary>
    public const float SubmenuY = ToolbarY + ButtonHeight + 8f;

    public MenuBar(EditorState editorState, Action<UIAction> onAction)
    {
        var actions = new[]
        {
            (label: "New", action: UIAction.New),
            (label: "Save", action: UIAction.Save),
            (label: "Load", action: UIAction.Load),
            (label: "Settings", action: UIAction.Settings),
        };

        for (int i = 0; i < actions.Length; i++)
        {
            var action = actions[i].action;
            var button = new Button
            {
                Text = actions[i].label,
                Size = new SKSize(ButtonWidth, ButtonHeight),
                Idle = new ButtonColors(new SKColor(50, 75, 85), new SKColor(170, 200, 210)),
                Hover = new ButtonColors(new SKColor(65, 100, 115), SKColors.White),
            };
            button.Click += () => onAction(action);
            Add(button);
        }

        var tools = new[]
        {
            (label: "Select", tool: EditorTool.Select),
            (label: "Road", tool: EditorTool.Road),
            (label: "Dest Pt", tool: EditorTool.Destination),
            (label: "Signal", tool: EditorTool.Signal),
        };

        for (int i = 0; i < tools.Length; i++)
        {
            var tool = tools[i].tool;
            var button = new Button
            {
                Text = tools[i].label,
                Size = new SKSize(ButtonWidth, ButtonHeight),
                IsActive = () => editorState.ActiveTool == tool,
            };
            (button.Idle, button.Hover, button.Active) = ToolColors(tool);
            button.Click += () =>
            {
                editorState.ResetToolState();
                editorState.ActiveTool = tool;
            };
            Add(button);
        }

        // Background strip enclosing both groups.
        float lastToolRight = ToolButtonLeft(tools.Length - 1) + ButtonWidth;
        Margin = new SKPoint(ToolbarX - 6f, ToolbarY - 4f);
        Size = new SKSize(lastToolRight - ToolbarX + 12f, ButtonHeight + 8f);
        BackgroundColor = UiTheme.PanelBackground;
    }

    protected override void LayoutChildren(float canvasWidth, float canvasHeight)
    {
        // Children are the 4 action buttons then the 4 tool buttons, in add order.
        // Offsets are panel-relative: the panel's left edge sits at ToolbarX - 6.
        for (int i = 0; i < Children.Count; i++)
        {
            float x = i < ActionCount
                ? ToolbarX + i * (ButtonWidth + ButtonSpacing)
                : ToolButtonLeft(i - ActionCount);
            Children[i].Offset = new SKPoint(x - (ToolbarX - 6f), 4f);
        }
        base.LayoutChildren(canvasWidth, canvasHeight);
    }

    /// <summary>Per-tool button color sets (verbatim from the retired toolbar).</summary>
    private static (ButtonColors idle, ButtonColors hover, ButtonColors active) ToolColors(EditorTool tool)
    {
        var white = SKColors.White;
        var idleText = new SKColor(180, 180, 180);
        return tool switch
        {
            EditorTool.Destination => (
                new ButtonColors(new SKColor(90, 35, 30), idleText),
                new ButtonColors(new SKColor(130, 50, 40), white),
                new ButtonColors(new SKColor(180, 50, 40), white)),
            _ => (
                new ButtonColors(new SKColor(55, 58, 65), idleText),
                new ButtonColors(new SKColor(75, 80, 90), white),
                new ButtonColors(new SKColor(60, 130, 200), white)),
        };
    }
}
