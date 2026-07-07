using SkiaSharp;
using Roads.App.Editor;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The signals submenu shown under the Signal toolbar button while any signal-family tool
/// is active: <b>Change Type</b> (the classic Signal tool — click cycles a node through
/// Light / Stop / Yield / None), <b>Control Type</b> (toggles a light between fixed-time
/// and actuated; F/A badges show over every light), <b>Rotate</b> (rotates a light's
/// phase grouping), and <b>Exempt</b> (toggles whether a stop/yield approach must stop —
/// the hover highlights the exact approach a click would toggle).
/// Visibility is a live <see cref="Panel.VisibleWhen"/> gate on the
/// active tool, covering both drawing and hit-testing. The background pad consumes clicks.
/// </summary>
public class SignalSubmenu : Panel
{
    private const float ButtonWidth = 80f;
    private const float ButtonHeight = 24f;
    private const float ButtonSpacing = 3f;
    private const float Pad = 4f;

    public SignalSubmenu(EditorState editorState)
    {
        VisibleWhen = () => editorState.ActiveTool
            is EditorTool.Signal or EditorTool.SignalControl
            or EditorTool.SignalRotate or EditorTool.SignalExempt;

        var tools = new[]
        {
            (label: "Change Type", tool: EditorTool.Signal),
            (label: "Control Type", tool: EditorTool.SignalControl),
            (label: "Rotate", tool: EditorTool.SignalRotate),
            (label: "Exempt", tool: EditorTool.SignalExempt),
        };
        for (int i = 0; i < tools.Length; i++)
        {
            var tool = tools[i].tool;
            var button = new Button
            {
                Text = tools[i].label,
                Font = UiTheme.Font11,
                TextOffset = new SKPoint(0f, 4f),
                Size = new SKSize(ButtonWidth, ButtonHeight),
                Offset = new SKPoint(Pad + i * (ButtonWidth + ButtonSpacing), Pad),
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

        // Anchored under the Signal tool button (index 3), padded like the POI submenu.
        Margin = new SKPoint(MenuBar.ToolButtonLeft(3) - Pad, MenuBar.SubmenuY - Pad);
        Size = new SKSize(
            tools.Length * ButtonWidth + (tools.Length - 1) * ButtonSpacing + 2f * Pad,
            ButtonHeight + 2f * Pad);
        BackgroundColor = UiTheme.PanelBackground;
    }
}
