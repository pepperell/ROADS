using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Owner of all top-level screen-space panels: z-order, layout, drawing, hover tracking,
/// input consumption, and mouse capture. Panels draw bottom→top in add order and hit-test
/// top→bottom (reverse), so the last-added panel is visually and interactively topmost.
///
/// Input contract (MainForm routes left-button events here before world tools):
/// - <see cref="OnMouseDown"/> finds the deepest hit panel and walks up its parent chain
///   until a panel's OnMouseDown returns true; that panel is the consumer and takes mouse
///   CAPTURE. Returns true when consumed (the map must not see the event).
/// - While captured, every <see cref="OnMouseMove"/> goes only to the captured panel (even
///   far outside its bounds — slider drags and minimap scrubs rely on this) and returns
///   true; <see cref="OnMouseUp"/> releases capture after notifying the panel.
/// - Uncaptured moves maintain the single hovered panel (Enter/Leave) and return true when
///   the pointer is over any interactive panel, so the caller suppresses map hover logic.
/// - <see cref="OnMouseWheel"/> offers the wheel to the TOPMOST hit chain only (deepest
///   panel, then up the parents); a declining chain does NOT fall through to panels
///   underneath — a modal scrim must never let the wheel scroll panels below it. The
///   caller applies camera zoom only when this returns false (and no modal is up).
/// - Stale capture (a modal dialog opened inside Click can swallow the matching mouse-up)
///   self-heals two ways: MainForm releases it on the first mouse-move with no button held
///   (so hover resumes immediately after the dialog closes), and the next mouse-down
///   clears any leftover capture before dispatching (backstop).
///
/// <see cref="Draw"/> also runs Measure+Layout for every panel INCLUDING
/// <see cref="Panel.ExternallyDrawn"/> ones (they are only excluded from painting; the
/// perf HUD is drawn by MainForm after the frame's draw-time measurement).
/// </summary>
public class UiRoot
{
    private readonly List<Panel> _panels = new();
    private Panel? _hovered;
    private Panel? _captured;

    /// <summary>True while a panel holds mouse capture (a UI drag is in progress).</summary>
    public bool HasCapture => _captured != null;

    /// <summary>Adds a top-level panel above all previously added panels.</summary>
    public void Add(Panel panel) => _panels.Add(panel);

    /// <summary>Lays out and draws the whole tree for this frame (bottom→top).</summary>
    public void Draw(SKCanvas canvas, float canvasWidth, float canvasHeight)
    {
        foreach (var panel in _panels)
        {
            panel.Measure(canvasWidth, canvasHeight);
            panel.Layout(canvasWidth, canvasHeight);
        }
        foreach (var panel in _panels)
        {
            if (panel.ExternallyDrawn) continue;
            panel.Draw(canvas);
        }
    }

    /// <summary>Left mouse-down. True when a panel consumed the event (and took capture).</summary>
    public bool OnMouseDown(float x, float y)
    {
        // Self-heal stale capture (modal dialog swallowed the matching mouse-up).
        if (_captured != null)
        {
            _captured.IsPressed = false;
            _captured = null;
        }

        for (int i = _panels.Count - 1; i >= 0; i--)
        {
            var hit = _panels[i].HitTest(x, y);
            if (hit == null) continue;

            for (var p = hit; p != null; p = p.Parent)
            {
                if (!p.OnMouseDown(x, y)) continue;
                _captured = p;
                p.IsPressed = true;
                return true;
            }
            // The hit chain declined consumption; keep searching lower panels.
        }
        return false;
    }

    /// <summary>Mouse move. True when the UI owns the pointer (captured drag, or hovering
    /// any interactive panel) — the caller should suppress map hover logic.</summary>
    public bool OnMouseMove(float x, float y)
    {
        if (_captured != null)
        {
            _captured.OnMouseMove(x, y);
            return true;
        }

        Panel? hit = null;
        for (int i = _panels.Count - 1; i >= 0 && hit == null; i--)
            hit = _panels[i].HitTest(x, y);

        if (!ReferenceEquals(hit, _hovered))
        {
            _hovered?.OnMouseLeave();
            _hovered = hit;
            _hovered?.OnMouseEnter();
        }
        _hovered?.OnMouseMove(x, y);
        return hit != null;
    }

    /// <summary>Mouse wheel. True when a panel in the topmost hit chain consumed it
    /// (see the class doc — no fall-through past the topmost hit panel).</summary>
    public bool OnMouseWheel(float x, float y, float delta)
    {
        for (int i = _panels.Count - 1; i >= 0; i--)
        {
            var hit = _panels[i].HitTest(x, y);
            if (hit == null) continue;

            for (var p = hit; p != null; p = p.Parent)
                if (p.OnMouseWheel(x, y, delta))
                    return true;
            return false;
        }
        return false;
    }

    /// <summary>Left mouse-up: notifies and releases the captured panel, if any.</summary>
    public void OnMouseUp(float x, float y)
    {
        if (_captured == null) return;
        _captured.OnMouseUp(x, y);
        _captured.IsPressed = false;
        _captured = null;
    }
}
