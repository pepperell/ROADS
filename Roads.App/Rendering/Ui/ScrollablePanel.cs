using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// A viewport over oversized content: callers add rows to <see cref="Content"/> (an
/// input-transparent inner panel) using ordinary child Offsets, and this panel scrolls
/// them. The content extent is recomputed every Layout from the content children's
/// Offset + Size, so callers never declare it; scrollbars (vertical and/or horizontal)
/// appear only when the extent exceeds the viewport. Children draw clipped to this
/// panel's bounds, and hit-testing is clamped to them too, so scrolled-out rows can
/// neither draw over nor steal clicks from surrounding UI.
///
/// A <see cref="GutterWidth"/> lane is ALWAYS reserved on the right and bottom edges
/// (bars overlay it when visible) so content geometry never reflows when a bar appears —
/// size row widths to <c>Size.Width - GutterWidth</c>.
///
/// Input: the mouse wheel scrolls vertically (horizontally when only horizontal overflow
/// exists), routed via <see cref="UiRoot.OnMouseWheel"/>; a press anywhere in a bar's
/// gutter grabs the thumb (centering it first when the press is outside it) and drags
/// through the standard UiRoot mouse capture. Scroll state clamps every frame, so
/// content shrinking below the viewport just snaps back to the origin.
/// </summary>
public class ScrollablePanel : Panel
{
    /// <summary>Reserved scrollbar lane on the right and bottom edges (always deducted
    /// from the viewport — see class doc).</summary>
    public const float GutterWidth = 10f;

    private const float BarThickness = 6f;
    private const float BarMargin = 2f;
    private const float MinThumbLength = 24f;
    /// <summary>Pixels scrolled per wheel notch (raw delta 120).</summary>
    private const float WheelStep = 48f;

    private static readonly SKColor TrackColor = new(255, 255, 255, 18);
    private static readonly SKColor ThumbColor = new(150, 155, 165, 150);
    private static readonly SKColor ThumbActiveColor = new(190, 195, 205, 220);

    /// <summary>The inner panel callers add content to. Input-transparent; sized and
    /// positioned by this panel every Layout (Size = content extent, Offset = −scroll).</summary>
    public Panel Content { get; } = new() { HitTestSelf = false };

    private SKSize _extent;
    private float _scrollX, _scrollY;
    private bool _dragV, _dragH;
    /// <summary>Pointer offset within the thumb at drag start (keeps the grab point pinned).</summary>
    private float _dragGrab;

    public ScrollablePanel()
    {
        Add(Content);
    }

    private float ViewportWidth => Size.Width - GutterWidth;
    private float ViewportHeight => Size.Height - GutterWidth;
    private float MaxScrollX => MathF.Max(0f, _extent.Width - ViewportWidth);
    private float MaxScrollY => MathF.Max(0f, _extent.Height - ViewportHeight);
    private bool VBarVisible => MaxScrollY > 0f;
    private bool HBarVisible => MaxScrollX > 0f;

    /// <summary>Extent is derived from logical child geometry (Offset + Size), not laid-out
    /// bounds, so it is scroll-independent; dynamically Measured children pick up one frame
    /// late, which is invisible at the frame rate.</summary>
    public override void Layout(float canvasWidth, float canvasHeight)
    {
        Bounds = ComputeOwnBounds(canvasWidth, canvasHeight);

        float ex = 0f, ey = 0f;
        foreach (var child in Content.Children)
        {
            ex = MathF.Max(ex, child.Offset.X + child.Size.Width);
            ey = MathF.Max(ey, child.Offset.Y + child.Size.Height);
        }
        _extent = new SKSize(ex, ey);

        _scrollX = Math.Clamp(_scrollX, 0f, MaxScrollX);
        _scrollY = Math.Clamp(_scrollY, 0f, MaxScrollY);

        Content.Size = new SKSize(MathF.Max(ex, ViewportWidth), MathF.Max(ey, ViewportHeight));
        Content.Offset = new SKPoint(-_scrollX, -_scrollY);
        LayoutChildren(canvasWidth, canvasHeight);
    }

    /// <summary>Children clipped to the viewport; bars drawn above them, inside the border.</summary>
    public override void Draw(SKCanvas canvas)
    {
        if (!IsEffectivelyVisible) return;

        OnDrawBackground(canvas);
        OnDraw(canvas);
        canvas.Save();
        canvas.ClipRect(Bounds);
        foreach (var child in Children)
        {
            if (child.ExternallyDrawn) continue;
            child.Draw(canvas);
        }
        canvas.Restore();
        DrawScrollbars(canvas);
        OnDrawBorder(canvas);
    }

    /// <summary>Clamped to this panel's bounds so scrolled-out children are unreachable.</summary>
    public override Panel? HitTest(float x, float y)
    {
        if (!IsEffectivelyVisible || !Bounds.Contains(x, y)) return null;
        return base.HitTest(x, y);
    }

    public override bool OnMouseDown(float x, float y)
    {
        if (VBarVisible && VGutterRect.Contains(x, y))
        {
            var (top, len) = VThumb;
            // A press outside the thumb centers it on the pointer first, then drags.
            _dragGrab = y >= top && y <= top + len ? y - top : len / 2f;
            _dragV = true;
            OnMouseMove(x, y);
            return true;
        }
        if (HBarVisible && HGutterRect.Contains(x, y))
        {
            var (left, len) = HThumb;
            _dragGrab = x >= left && x <= left + len ? x - left : len / 2f;
            _dragH = true;
            OnMouseMove(x, y);
            return true;
        }
        return base.OnMouseDown(x, y);
    }

    public override void OnMouseMove(float x, float y)
    {
        if (_dragV)
        {
            var (_, len) = VThumb;
            float travel = VTrackLength - len;
            if (travel > 0f)
                _scrollY = Math.Clamp((y - _dragGrab - VTrackTop) / travel * MaxScrollY, 0f, MaxScrollY);
        }
        else if (_dragH)
        {
            var (_, len) = HThumb;
            float travel = HTrackLength - len;
            if (travel > 0f)
                _scrollX = Math.Clamp((x - _dragGrab - HTrackLeft) / travel * MaxScrollX, 0f, MaxScrollX);
        }
    }

    public override void OnMouseUp(float x, float y)
    {
        _dragV = false;
        _dragH = false;
    }

    public override bool OnMouseWheel(float x, float y, float delta)
    {
        float step = delta / 120f * WheelStep;
        if (VBarVisible)
        {
            _scrollY = Math.Clamp(_scrollY - step, 0f, MaxScrollY);
            return true;
        }
        if (HBarVisible)
        {
            _scrollX = Math.Clamp(_scrollX - step, 0f, MaxScrollX);
            return true;
        }
        return false;
    }

    // ── Bar geometry (tracks stop short of the shared corner) ──

    private float VTrackTop => Bounds.Top + BarMargin;
    private float VTrackLength => ViewportHeight - 2f * BarMargin;
    private float HTrackLeft => Bounds.Left + BarMargin;
    private float HTrackLength => ViewportWidth - 2f * BarMargin;

    private SKRect VGutterRect => SKRect.Create(
        Bounds.Right - GutterWidth, Bounds.Top, GutterWidth, ViewportHeight);
    private SKRect HGutterRect => SKRect.Create(
        Bounds.Left, Bounds.Bottom - GutterWidth, ViewportWidth, GutterWidth);

    /// <summary>(top, length) of the vertical thumb along its track.</summary>
    private (float pos, float len) VThumb
    {
        get
        {
            float len = Math.Clamp(VTrackLength * ViewportHeight / _extent.Height, MinThumbLength, VTrackLength);
            float top = VTrackTop + (VTrackLength - len) * (MaxScrollY > 0f ? _scrollY / MaxScrollY : 0f);
            return (top, len);
        }
    }

    /// <summary>(left, length) of the horizontal thumb along its track.</summary>
    private (float pos, float len) HThumb
    {
        get
        {
            float len = Math.Clamp(HTrackLength * ViewportWidth / _extent.Width, MinThumbLength, HTrackLength);
            float left = HTrackLeft + (HTrackLength - len) * (MaxScrollX > 0f ? _scrollX / MaxScrollX : 0f);
            return (left, len);
        }
    }

    private void DrawScrollbars(SKCanvas canvas)
    {
        float inset = (GutterWidth - BarThickness) / 2f;
        if (VBarVisible)
        {
            float barX = Bounds.Right - GutterWidth + inset;
            UiTheme.FillScratch.Color = TrackColor;
            canvas.DrawRoundRect(SKRect.Create(barX, VTrackTop, BarThickness, VTrackLength),
                BarThickness / 2f, BarThickness / 2f, UiTheme.FillScratch);
            var (top, len) = VThumb;
            UiTheme.FillScratch.Color = _dragV ? ThumbActiveColor : ThumbColor;
            canvas.DrawRoundRect(SKRect.Create(barX, top, BarThickness, len),
                BarThickness / 2f, BarThickness / 2f, UiTheme.FillScratch);
        }
        if (HBarVisible)
        {
            float barY = Bounds.Bottom - GutterWidth + inset;
            UiTheme.FillScratch.Color = TrackColor;
            canvas.DrawRoundRect(SKRect.Create(HTrackLeft, barY, HTrackLength, BarThickness),
                BarThickness / 2f, BarThickness / 2f, UiTheme.FillScratch);
            var (left, len) = HThumb;
            UiTheme.FillScratch.Color = _dragH ? ThumbActiveColor : ThumbColor;
            canvas.DrawRoundRect(SKRect.Create(left, barY, len, BarThickness),
                BarThickness / 2f, BarThickness / 2f, UiTheme.FillScratch);
        }
    }
}
