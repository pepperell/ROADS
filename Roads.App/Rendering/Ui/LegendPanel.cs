using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The keyboard-shortcut legend in the bottom-left corner. Static text on a translucent
/// backdrop; the panel consumes clicks (like every opaque panel in the retained UI) so a
/// click on the legend never edits the map behind it.
/// </summary>
public class LegendPanel : Panel
{
    private const float LineHeight = 16f;
    private const float Pad = 8f;

    private static readonly string[] ShortcutLines =
    {
        "Space   Pause / Resume",
        "<  >    Sim Speed - / +",
        "V       Spawn Vehicle",
        "+  -    Lane Count",
        "[  ]    Speed Limit",
        "R       Cycle Road Type",
        "O       One-Way (cycle)",
        "J       Single-Lane 2-Way",
        "Del     Delete Node",
        "L       Lane-Restrict Mode",
        "C       Default Restrictions",
        "1-4     Select Lane",
        "Esc     Exit Lane Mode",
        "T       Sliders",
        "P       Perf HUD",
        "M       Minimap",
        "N       Statistics Panel",
        "H       Heat-map",
        "G       Arc-Conflict Debug",
        "D       Dump Vehicle Diag",
        "F       Frame Diag Log",
        "Ctrl+S  Save Map",
        "Ctrl+O  Load Map",
        "K       Stress: Grid + 10K",
        "B       Capture Baseline",
    };

    public LegendPanel()
    {
        Anchor = UiAnchor.BottomLeft;
        Margin = new SKPoint(10f, 10f);
        Size = new SKSize(240f, ShortcutLines.Length * LineHeight + Pad * 2f);
        BackgroundColor = new SKColor(30, 32, 38, 180);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        UiTheme.TextScratch.Color = UiTheme.TextDim;
        for (int i = 0; i < ShortcutLines.Length; i++)
        {
            canvas.DrawText(ShortcutLines[i], Bounds.Left + Pad,
                Bounds.Top + Pad + (i + 1) * LineHeight - 2f,
                SKTextAlign.Left, UiTheme.Font12, UiTheme.TextScratch);
        }
    }
}
