using SkiaSharp;
using Roads.App.Editor;

namespace Roads.App.Rendering;

/// <summary>
/// Layout data for a single toolbar button: label text, associated tool, and screen bounds.
/// </summary>
public struct ToolbarButton
{
    /// <summary>Display text on the button.</summary>
    public string Label;
    /// <summary>Editor tool this button activates.</summary>
    public EditorTool Tool;
    /// <summary>Screen-space bounds of the button for hit testing and rendering.</summary>
    public SKRect Bounds;
}

/// <summary>
/// Renders the screen-space UI overlay: a status bar showing zoom/edge/vehicle counts,
/// and a horizontal toolbar of editor tool buttons with active-state highlighting.
/// </summary>
public class UIRenderer
{
    private const float ButtonWidth = 70f;
    private const float ButtonHeight = 30f;
    private const float ButtonSpacing = 4f;
    private const float ToolbarX = 10f;
    private const float ToolbarY = 36f;

    private readonly ToolbarButton[] _buttons;

    // Reusable paints for button rendering (Color updated per button per frame)
    private readonly SKPaint _btnPaint = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _btnTextPaint = new() { IsAntialias = true };

    public UIRenderer()
    {
        var tools = new[] {
            (label: "Select", tool: EditorTool.Select),
            (label: "Road", tool: EditorTool.Road),
            (label: "Delete", tool: EditorTool.Delete),
            (label: "Spawn Pt", tool: EditorTool.SpawnPoint),
            (label: "Dest Pt", tool: EditorTool.Destination),
            (label: "Signal", tool: EditorTool.Signal),
        };

        _buttons = new ToolbarButton[tools.Length];
        for (int i = 0; i < tools.Length; i++)
        {
            float x = ToolbarX + i * (ButtonWidth + ButtonSpacing);
            _buttons[i] = new ToolbarButton
            {
                Label = tools[i].label,
                Tool = tools[i].tool,
                Bounds = new SKRect(x, ToolbarY, x + ButtonWidth, ToolbarY + ButtonHeight),
            };
        }
    }

    private static readonly string[] ShortcutLines = {
        "Space  Pause / Resume",
        "<  >   Speed Down / Up",
        "V      Spawn Vehicle",
        "+  -   Lane Count",
        "[  ]   Speed Limit",
        "T      Toggle Sliders",
        "Del    Delete Node",
    };

    /// <summary>
    /// Draws the status bar text, toolbar buttons (highlighting the active tool),
    /// and a keyboard shortcut legend in the bottom-right corner.
    /// </summary>
    public void Draw(SKCanvas canvas, EditorState editorState, string statusText, float canvasWidth = 0, float canvasHeight = 0)
    {
        // Status bar
        using var statusPaint = new SKPaint { Color = SKColors.White, IsAntialias = true };
        using var statusFont = new SKFont { Size = 14 };
        canvas.DrawText(statusText, 10, 20, SKTextAlign.Left, statusFont, statusPaint);

        // Toolbar background
        float lastRight = _buttons[_buttons.Length - 1].Bounds.Right;
        float totalWidth = lastRight - ToolbarX + 6f + 6f;
        using var bgPaint = new SKPaint
        {
            Color = new SKColor(30, 32, 38, 220),
            Style = SKPaintStyle.Fill,
        };
        canvas.DrawRoundRect(ToolbarX - 6f, ToolbarY - 4f, totalWidth, ButtonHeight + 8f, 4f, 4f, bgPaint);

        using var font = new SKFont { Size = 13 };

        for (int i = 0; i < _buttons.Length; i++)
        {
            ref var btn = ref _buttons[i];
            bool active = editorState.ActiveTool == btn.Tool;

            SKColor btnColor;
            if (btn.Tool == EditorTool.SpawnPoint)
                btnColor = active ? new SKColor(40, 150, 60) : new SKColor(35, 80, 45);
            else if (btn.Tool == EditorTool.Destination)
                btnColor = active ? new SKColor(180, 50, 40) : new SKColor(90, 35, 30);
            else
                btnColor = active ? new SKColor(60, 130, 200) : new SKColor(55, 58, 65);

            _btnPaint.Color = btnColor;
            canvas.DrawRoundRect(btn.Bounds, 3f, 3f, _btnPaint);

            _btnTextPaint.Color = active ? SKColors.White : new SKColor(180, 180, 180);
            float textX = btn.Bounds.MidX;
            float textY = btn.Bounds.MidY + 4.5f;
            canvas.DrawText(btn.Label, textX, textY, SKTextAlign.Center, font, _btnTextPaint);
        }

        // Keyboard shortcut legend (bottom-right)
        if (canvasWidth > 0 && canvasHeight > 0)
        {
            using var legendFont = new SKFont { Size = 12 };
            float lineHeight = 16f;
            float padding = 8f;
            float legendWidth = 190f;
            float legendHeight = ShortcutLines.Length * lineHeight + padding * 2;
            float lx = canvasWidth - legendWidth - 10f;
            float ly = canvasHeight - legendHeight - 10f;

            using var legendBg = new SKPaint
            {
                Color = new SKColor(30, 32, 38, 180),
                Style = SKPaintStyle.Fill,
            };
            canvas.DrawRoundRect(lx, ly, legendWidth, legendHeight, 4f, 4f, legendBg);

            using var legendText = new SKPaint
            {
                Color = new SKColor(170, 170, 170),
                IsAntialias = true,
            };
            for (int li = 0; li < ShortcutLines.Length; li++)
            {
                canvas.DrawText(ShortcutLines[li], lx + padding, ly + padding + (li + 1) * lineHeight - 2f, SKTextAlign.Left, legendFont, legendText);
            }
        }
    }

    /// <summary>
    /// Tests if a screen position hits a toolbar button.
    /// </summary>
    /// <param name="screenX">Screen X coordinate.</param>
    /// <param name="screenY">Screen Y coordinate.</param>
    /// <returns>The tool if a button was clicked; otherwise <c>null</c>.</returns>
    public EditorTool? HitTest(float screenX, float screenY)
    {
        for (int i = 0; i < _buttons.Length; i++)
        {
            if (_buttons[i].Bounds.Contains(screenX, screenY))
                return _buttons[i].Tool;
        }
        return null;
    }
}
