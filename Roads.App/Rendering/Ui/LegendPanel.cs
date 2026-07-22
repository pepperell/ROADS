using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// The keyboard-shortcut reference, grouped into categories (General, Tools, Selected
/// Segment, Selected Node, Panels, Debug) with a brighter header line per group and a
/// title at the top. Lives inside the <see cref="PauseMenu"/> (positioned beside the
/// button window), so it is only visible while the game is paused. Static text on a
/// translucent backdrop; like every opaque panel it consumes clicks.
/// </summary>
public class LegendPanel : Panel
{
    private const float LineHeight = 16f;
    /// <summary>Extra vertical space above each group header (except the first group).</summary>
    private const float GroupGap = 6f;
    /// <summary>Gap between the title and the first group.</summary>
    private const float TitleGap = 8f;
    private const float Pad = 10f;

    private const string Title = "Keyboard Shortcuts";

    private static readonly (string Header, string[] Lines)[] Groups =
    {
        ("General", new[]
        {
            "Space   Pause / Resume",
            "<  >    Sim Speed - / +",
            "Ctrl+S  Save (quiet)",
            "Esc     Menu (when idle)",
        }),
        ("Tools", new[]
        {
            "W       Water Tool",
            "Esc,RMB Cancel / Select Tool",
        }),
        ("Selected Segment", new[]
        {
            "+  -    Lane Count",
            "R       Cycle Road Type",
            "O       One-Way (cycle)",
            "J       Single-Lane 2-Way",
        }),
        ("Selected Node", new[]
        {
            "L       Toggle Lane-Restrict Mode",
            "C       Default Restrictions",
            "Del     Delete Node",
            "1-4     Select Lane",
        }),
        ("Panels", new[]
        {
            "P       Perf HUD",
            "M       Minimap",
            "N       Statistics Panel",
            "H       Heat-map",
        }),
        ("Debug", new[]
        {
            "G       Arc-Conflict Debug",
            "D       Dump Vehicle Diag",
            "F       Frame Diag Log",
            "K       Stress: Grid + 10K",
            "B       Capture Baseline",
        }),
    };

    public LegendPanel()
    {
        int lineCount = 1; // title
        foreach (var (_, lines) in Groups)
            lineCount += 1 + lines.Length; // header + entries
        Size = new SKSize(240f,
            lineCount * LineHeight + TitleGap + (Groups.Length - 1) * GroupGap + Pad * 2f);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        float x = Bounds.Left + Pad;
        float y = Bounds.Top + Pad + LineHeight - 2f;

        UiTheme.TextScratch.Color = UiTheme.TextPrimary;
        canvas.DrawText(Title, x, y, SKTextAlign.Left, UiTheme.Font13, UiTheme.TextScratch);
        y += LineHeight + TitleGap;

        for (int g = 0; g < Groups.Length; g++)
        {
            if (g > 0) y += GroupGap;

            UiTheme.TextScratch.Color = UiTheme.Value;
            canvas.DrawText(Groups[g].Header, x, y,
                SKTextAlign.Left, UiTheme.Font12, UiTheme.TextScratch);
            y += LineHeight;

            UiTheme.TextScratch.Color = UiTheme.TextDim;
            foreach (string line in Groups[g].Lines)
            {
                canvas.DrawText(line, x, y,
                    SKTextAlign.Left, UiTheme.Font12, UiTheme.TextScratch);
                y += LineHeight;
            }
        }
    }
}
