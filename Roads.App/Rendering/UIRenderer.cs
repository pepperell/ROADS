using SkiaSharp;
using Roads.App.Editor;
using Roads.App.World;

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

/// <summary>Identifies a UI action triggered by an action button (not a tool toggle).</summary>
public enum UIAction { New, Save, Load, Pause, SpeedDown, SpeedUp }

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

    // Action buttons (Save/Load) — separate from tool-toggle buttons
    private const float ActionGap = 16f;
    private readonly (string Label, UIAction Action, SKRect Bounds)[] _actionButtons;

    /// <summary>Index of the toolbar button currently hovered, or -1.</summary>
    private int _hoveredToolButton = -1;
    /// <summary>Index of the action button currently hovered, or -1.</summary>
    private int _hoveredActionButton = -1;

    // POI submenu constants and state
    private const float POIButtonWidth = 55f;
    private const float POIButtonHeight = 24f;
    private const float POIButtonSpacing = 3f;
    private readonly SKRect[] _poiButtons;
    private static readonly string[] POILabels = { "Home", "Work", "Shop", "Leisure", "School", "Parking", "Ent/Exit" };

    /// <summary>POI type colors, indexed by (POIType - 1). Used for both submenu buttons and map markers.</summary>
    public static readonly SKColor[] POIColors =
    {
        new SKColor(60, 130, 220, 200),   // Home — blue
        new SKColor(140, 140, 150, 200),  // Work — gray
        new SKColor(220, 150, 40, 200),   // Shop — orange
        new SKColor(60, 180, 80, 200),    // Leisure — green
        new SKColor(200, 190, 50, 200),   // School — yellow
        new SKColor(60, 180, 200, 200),   // Parking — cyan
        new SKColor(190, 80, 200, 200),   // EntryExit — magenta
    };

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

        // Action buttons (Save / Load) after a gap
        var actions = new[] {
            (label: "New", action: UIAction.New),
            (label: "Save", action: UIAction.Save),
            (label: "Load", action: UIAction.Load),
            (label: "Pause", action: UIAction.Pause),
            (label: "<<", action: UIAction.SpeedDown),
            (label: ">>", action: UIAction.SpeedUp),
        };
        float actionStartX = _buttons[_buttons.Length - 1].Bounds.Right + ActionGap;
        _actionButtons = new (string, UIAction, SKRect)[actions.Length];
        for (int i = 0; i < actions.Length; i++)
        {
            float x = actionStartX + i * (ButtonWidth + ButtonSpacing);
            _actionButtons[i] = (actions[i].label, actions[i].action,
                new SKRect(x, ToolbarY, x + ButtonWidth, ToolbarY + ButtonHeight));
        }

        // POI submenu positioned below the "Dest Pt" button (index 4)
        float poiStartX = _buttons[4].Bounds.Left;
        float poiY = ToolbarY + ButtonHeight + 8f;
        _poiButtons = new SKRect[POILabels.Length];
        for (int i = 0; i < POILabels.Length; i++)
        {
            float px = poiStartX + i * (POIButtonWidth + POIButtonSpacing);
            _poiButtons[i] = new SKRect(px, poiY, px + POIButtonWidth, poiY + POIButtonHeight);
        }
    }

    private static readonly string[] ShortcutLines = {
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

    /// <summary>
    /// Draws the status bar text, toolbar buttons (highlighting the active tool),
    /// and a keyboard shortcut legend in the bottom-left corner.
    /// </summary>
    public void Draw(SKCanvas canvas, EditorState editorState, string statusText,
        bool paused = false, int timeScaleExponent = 0,
        float canvasWidth = 0, float canvasHeight = 0)
    {
        // Status bar
        using var statusPaint = new SKPaint { Color = SKColors.White, IsAntialias = true };
        using var statusFont = new SKFont { Size = 14 };
        canvas.DrawText(statusText, 10, 20, SKTextAlign.Left, statusFont, statusPaint);

        // Toolbar background
        float lastRight = _actionButtons[_actionButtons.Length - 1].Bounds.Right;
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
            bool hovered = _hoveredToolButton == i;

            SKColor btnColor;
            if (btn.Tool == EditorTool.SpawnPoint)
                btnColor = active ? new SKColor(40, 150, 60) : hovered ? new SKColor(50, 110, 60) : new SKColor(35, 80, 45);
            else if (btn.Tool == EditorTool.Destination)
                btnColor = active ? new SKColor(180, 50, 40) : hovered ? new SKColor(130, 50, 40) : new SKColor(90, 35, 30);
            else
                btnColor = active ? new SKColor(60, 130, 200) : hovered ? new SKColor(75, 80, 90) : new SKColor(55, 58, 65);

            _btnPaint.Color = btnColor;
            canvas.DrawRoundRect(btn.Bounds, 3f, 3f, _btnPaint);

            _btnTextPaint.Color = (active || hovered) ? SKColors.White : new SKColor(180, 180, 180);
            float textX = btn.Bounds.MidX;
            float textY = btn.Bounds.MidY + 4.5f;
            canvas.DrawText(btn.Label, textX, textY, SKTextAlign.Center, font, _btnTextPaint);
        }

        // Action buttons — state-aware rendering
        for (int i = 0; i < _actionButtons.Length; i++)
        {
            ref var ab = ref _actionButtons[i];
            bool hovered = _hoveredActionButton == i;
            string label = ab.Label;
            SKColor bgColor, textColor;

            if (ab.Action == UIAction.Pause)
            {
                label = paused ? "Play" : "Pause";
                if (hovered)
                {
                    bgColor = paused ? new SKColor(55, 155, 65) : new SKColor(150, 105, 40);
                    textColor = SKColors.White;
                }
                else
                {
                    bgColor = paused ? new SKColor(40, 120, 50) : new SKColor(120, 80, 30);
                    textColor = SKColors.White;
                }
            }
            else if (ab.Action == UIAction.SpeedDown)
            {
                bool canDecrease = timeScaleExponent > 0;
                bgColor = !canDecrease ? new SKColor(40, 45, 50)
                    : hovered ? new SKColor(65, 100, 115) : new SKColor(50, 75, 85);
                textColor = !canDecrease ? new SKColor(90, 95, 100)
                    : hovered ? SKColors.White : new SKColor(170, 200, 210);
            }
            else if (ab.Action == UIAction.SpeedUp)
            {
                bool canIncrease = timeScaleExponent < 6;
                bgColor = !canIncrease ? new SKColor(40, 45, 50)
                    : hovered ? new SKColor(65, 100, 115) : new SKColor(50, 75, 85);
                textColor = !canIncrease ? new SKColor(90, 95, 100)
                    : hovered ? SKColors.White : new SKColor(170, 200, 210);
            }
            else
            {
                bgColor = hovered ? new SKColor(65, 100, 115) : new SKColor(50, 75, 85);
                textColor = hovered ? SKColors.White : new SKColor(170, 200, 210);
            }

            _btnPaint.Color = bgColor;
            canvas.DrawRoundRect(ab.Bounds, 3f, 3f, _btnPaint);

            _btnTextPaint.Color = textColor;
            canvas.DrawText(label, ab.Bounds.MidX, ab.Bounds.MidY + 4.5f,
                SKTextAlign.Center, font, _btnTextPaint);
        }

        // POI type submenu (only when Destination tool active)
        if (editorState.ActiveTool == EditorTool.Destination)
            DrawPOISubmenu(canvas, editorState);

        // Keyboard shortcut legend (bottom-left)
        if (canvasWidth > 0 && canvasHeight > 0)
        {
            using var legendFont = new SKFont { Size = 12 };
            float lineHeight = 16f;
            float padding = 8f;
            float legendWidth = 240f;
            float legendHeight = ShortcutLines.Length * lineHeight + padding * 2;
            float lx = 10f;
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
    /// Updates hover state for all buttons. Returns true if the mouse is over any button,
    /// signaling that map-level hover logic should be suppressed.
    /// </summary>
    public bool UpdateHover(float screenX, float screenY)
    {
        _hoveredToolButton = -1;
        _hoveredActionButton = -1;

        for (int i = 0; i < _buttons.Length; i++)
        {
            if (_buttons[i].Bounds.Contains(screenX, screenY))
            {
                _hoveredToolButton = i;
                return true;
            }
        }
        for (int i = 0; i < _actionButtons.Length; i++)
        {
            if (_actionButtons[i].Bounds.Contains(screenX, screenY))
            {
                _hoveredActionButton = i;
                return true;
            }
        }
        return false;
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

    /// <summary>
    /// Tests if a screen position hits an action button (Save/Load).
    /// </summary>
    public UIAction? HitTestAction(float screenX, float screenY)
    {
        for (int i = 0; i < _actionButtons.Length; i++)
        {
            if (_actionButtons[i].Bounds.Contains(screenX, screenY))
                return _actionButtons[i].Action;
        }
        return null;
    }

    /// <summary>
    /// Tests if a screen position hits a POI submenu button.
    /// </summary>
    /// <returns>The POI type if a button was clicked; otherwise <c>null</c>.</returns>
    public POIType? HitTestPOI(float screenX, float screenY)
    {
        for (int i = 0; i < _poiButtons.Length; i++)
        {
            if (_poiButtons[i].Contains(screenX, screenY))
                return (POIType)(i + 1); // POIType.Home = 1, Work = 2, etc.
        }
        return null;
    }

    private void DrawPOISubmenu(SKCanvas canvas, EditorState editorState)
    {
        // Background panel
        float bgLeft = _poiButtons[0].Left - 4f;
        float bgTop = _poiButtons[0].Top - 4f;
        float bgRight = _poiButtons[^1].Right + 4f;
        float bgBottom = _poiButtons[0].Bottom + 4f;

        using var bgPaint = new SKPaint
        {
            Color = new SKColor(30, 32, 38, 220),
            Style = SKPaintStyle.Fill,
        };
        canvas.DrawRoundRect(bgLeft, bgTop, bgRight - bgLeft, bgBottom - bgTop, 4f, 4f, bgPaint);

        using var font = new SKFont { Size = 11 };

        for (int i = 0; i < _poiButtons.Length; i++)
        {
            var poiType = (POIType)(i + 1);
            bool active = editorState.SelectedPOIType == poiType;

            var color = POIColors[i];
            _btnPaint.Color = active
                ? new SKColor(color.Red, color.Green, color.Blue, 255)
                : new SKColor((byte)(color.Red / 2), (byte)(color.Green / 2), (byte)(color.Blue / 2), 200);

            canvas.DrawRoundRect(_poiButtons[i], 3f, 3f, _btnPaint);

            _btnTextPaint.Color = active ? SKColors.White : new SKColor(160, 160, 160);
            float textX = _poiButtons[i].MidX;
            float textY = _poiButtons[i].MidY + 4f;
            canvas.DrawText(POILabels[i], textX, textY, SKTextAlign.Center, font, _btnTextPaint);
        }
    }
}
