using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>Corner a top-level panel is anchored to; its <see cref="Panel.Margin"/> is the
/// offset from that corner.</summary>
public enum UiAnchor { TopLeft, TopRight, BottomLeft, BottomRight }

/// <summary>
/// The fundamental retained-mode UI object: a rectangle with an optional background and
/// border, mouse interaction (hover/press/click), and children. Coordinates are absolute
/// canvas pixels; <see cref="Bounds"/> is recomputed every frame by <see cref="Layout"/>
/// (top-level panels from <see cref="Anchor"/>+<see cref="Margin"/>+<see cref="Size"/>,
/// children from the parent's top-left plus <see cref="Offset"/>). Input between frames
/// hit-tests the last-laid-out bounds.
/// Call order per frame: <see cref="UiRoot.Draw"/> runs Measure → Layout → Draw for the
/// whole tree; input dispatch (hit-test top-down, draw bottom-up) is owned by
/// <see cref="UiRoot"/>. A panel whose <see cref="OnMouseDown"/> returns true consumes the
/// click and becomes the mouse-capture target until release. Default panels consume
/// background clicks (<see cref="HitTestSelf"/> true) so UI never leaks input to the map.
/// </summary>
public class Panel
{
    private readonly List<Panel> _children = new();

    /// <summary>Absolute canvas-pixel bounds, valid after the frame's <see cref="Layout"/>.</summary>
    public SKRect Bounds;

    /// <summary>Desired size; fixed at construction or updated in <see cref="Measure"/>.</summary>
    public SKSize Size;

    /// <summary>Anchored corner (top-level panels only; ignored for children).</summary>
    public UiAnchor Anchor = UiAnchor.TopLeft;

    /// <summary>Offset from the anchored corner (top-level panels only).</summary>
    public SKPoint Margin;

    /// <summary>Offset from the parent's top-left corner (child panels only).</summary>
    public SKPoint Offset;

    /// <summary>Manual visibility flag (keyboard toggles flip this).</summary>
    public bool Visible = true;

    /// <summary>Optional live visibility gate evaluated on both draw and hit-test, so
    /// condition-driven panels (POI submenu, vehicle info) need no per-frame sync code.</summary>
    public Func<bool>? VisibleWhen;

    /// <summary>Effective visibility: <see cref="Visible"/> AND <see cref="VisibleWhen"/>.</summary>
    public bool IsEffectivelyVisible => Visible && (VisibleWhen?.Invoke() ?? true);

    /// <summary>Background fill; null draws no background.</summary>
    public SKColor? BackgroundColor;

    /// <summary>Border stroke; null draws no border.</summary>
    public SKColor? BorderColor;
    public float BorderWidth = 1f;
    public float CornerRadius = 4f;

    /// <summary>False makes the panel input-transparent (children are still hit-testable).</summary>
    public bool HitTestSelf = true;

    /// <summary>Laid out by <see cref="UiRoot.Draw"/> but not drawn by it — the owner draws
    /// the panel manually at a custom z-position (the performance HUD draws above everything
    /// and outside the measured draw window).</summary>
    public bool ExternallyDrawn;

    public Panel? Parent { get; private set; }
    public IReadOnlyList<Panel> Children => _children;

    /// <summary>True while the pointer is over this panel (maintained by <see cref="UiRoot"/>).</summary>
    public bool IsHovered { get; internal set; }

    /// <summary>True while this panel holds mouse capture (between consumed down and up).</summary>
    public bool IsPressed { get; internal set; }

    /// <summary>Raised when a mouse-down is consumed by this panel (see <see cref="OnMouseDown"/>;
    /// clicks fire on the DOWN, matching the app's historical immediate-mode behavior).</summary>
    public event Action? Click;

    public void Add(Panel child)
    {
        child.Parent = this;
        _children.Add(child);
    }

    protected void RaiseClick() => Click?.Invoke();

    /// <summary>Updates <see cref="Size"/> for dynamically-sized panels. Runs before
    /// <see cref="Layout"/> each frame so containers can position by measured sizes.</summary>
    public virtual void Measure(float canvasWidth, float canvasHeight) { }

    /// <summary>Computes <see cref="Bounds"/> and lays out children. Containers override
    /// <see cref="LayoutChildren"/> (or set child <see cref="Offset"/>s first) to place rows/stacks.</summary>
    public virtual void Layout(float canvasWidth, float canvasHeight)
    {
        Bounds = ComputeOwnBounds(canvasWidth, canvasHeight);
        LayoutChildren(canvasWidth, canvasHeight);
    }

    protected SKRect ComputeOwnBounds(float canvasWidth, float canvasHeight)
    {
        if (Parent == null)
        {
            float x = Anchor is UiAnchor.TopRight or UiAnchor.BottomRight
                ? canvasWidth - Margin.X - Size.Width : Margin.X;
            float y = Anchor is UiAnchor.BottomLeft or UiAnchor.BottomRight
                ? canvasHeight - Margin.Y - Size.Height : Margin.Y;
            return SKRect.Create(x, y, Size.Width, Size.Height);
        }
        return SKRect.Create(Parent.Bounds.Left + Offset.X, Parent.Bounds.Top + Offset.Y,
            Size.Width, Size.Height);
    }

    protected virtual void LayoutChildren(float canvasWidth, float canvasHeight)
    {
        foreach (var child in _children)
        {
            child.Measure(canvasWidth, canvasHeight);
            child.Layout(canvasWidth, canvasHeight);
        }
    }

    /// <summary>Draws background → content (<see cref="OnDraw"/>) → children (in add order,
    /// skipping <see cref="ExternallyDrawn"/>) → border. No-op when not effectively visible.</summary>
    public void Draw(SKCanvas canvas)
    {
        if (!IsEffectivelyVisible) return;

        OnDrawBackground(canvas);
        OnDraw(canvas);
        foreach (var child in _children)
        {
            if (child.ExternallyDrawn) continue;
            child.Draw(canvas);
        }
        OnDrawBorder(canvas);
    }

    /// <summary>Fills <see cref="BackgroundColor"/> when set. Buttons override to apply
    /// state-dependent colors before the label text draws.</summary>
    protected virtual void OnDrawBackground(SKCanvas canvas)
    {
        if (BackgroundColor is not { } bg) return;
        UiTheme.FillScratch.Color = bg;
        canvas.DrawRoundRect(Bounds, CornerRadius, CornerRadius, UiTheme.FillScratch);
    }

    /// <summary>Panel-specific content drawn between the background and the children.</summary>
    protected virtual void OnDraw(SKCanvas canvas) { }

    protected virtual void OnDrawBorder(SKCanvas canvas)
    {
        if (BorderColor is not { } bc) return;
        UiTheme.StrokeScratch.Color = bc;
        UiTheme.StrokeScratch.StrokeWidth = BorderWidth;
        canvas.DrawRoundRect(Bounds, CornerRadius, CornerRadius, UiTheme.StrokeScratch);
    }

    /// <summary>Returns the deepest interactive panel at (x, y): children are tested
    /// last-added-first (topmost wins), then this panel if <see cref="HitTestSelf"/>.</summary>
    public virtual Panel? HitTest(float x, float y)
    {
        if (!IsEffectivelyVisible) return null;
        for (int i = _children.Count - 1; i >= 0; i--)
        {
            var hit = _children[i].HitTest(x, y);
            if (hit != null) return hit;
        }
        if (HitTestSelf && Bounds.Contains(x, y)) return this;
        return null;
    }

    /// <summary>Mouse-down on this panel. Returning true consumes the event and grants this
    /// panel mouse capture. The default raises <see cref="Click"/> and consumes when
    /// <see cref="HitTestSelf"/> (background clicks never reach the map).</summary>
    public virtual bool OnMouseDown(float x, float y)
    {
        RaiseClick();
        return HitTestSelf;
    }

    /// <summary>Mouse move: while captured, receives every move (even far outside
    /// <see cref="Bounds"/>); otherwise only while hovered.</summary>
    public virtual void OnMouseMove(float x, float y) { }

    /// <summary>Mouse release delivered to the captured panel just before capture clears.</summary>
    public virtual void OnMouseUp(float x, float y) { }

    public virtual void OnMouseEnter() => IsHovered = true;
    public virtual void OnMouseLeave() => IsHovered = false;
}
