using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Invisible layout container that stacks its children upward with a fixed gap, skipping
/// children that are not effectively visible — so the performance HUD, statistics panel,
/// vehicle-info panel, and selection-info panel never overlap and always sit flush above
/// one another regardless of which combination is shown (this replaces the stale
/// hardcoded HUD-height constant that made the old panels overlap by 28 px).
/// Anchored as a second bottom-left column just right of the (240 px wide) shortcut
/// legend, since the stack's panels are shown by default and would otherwise cover it.
/// Children are stacked in ADD order, bottom-first. The container itself draws nothing
/// and is input-transparent; its children consume input normally.
/// </summary>
public class BottomLeftStack : Panel
{
    private const float Gap = 4f;

    public BottomLeftStack()
    {
        HitTestSelf = false;
        Anchor = UiAnchor.BottomLeft;
        Margin = new SKPoint(258f, 10f);
        Size = new SKSize(0f, 0f);
    }

    public override void Layout(float canvasWidth, float canvasHeight)
    {
        // The container's bounds are a zero-size anchor point at the bottom-left inset;
        // children position themselves above it via computed offsets.
        float anchorY = canvasHeight - Margin.Y;
        Bounds = SKRect.Create(Margin.X, anchorY, 0f, 0f);

        float nextBottom = anchorY;
        foreach (var child in Children)
        {
            child.Measure(canvasWidth, canvasHeight);
            if (!child.IsEffectivelyVisible) continue;
            child.Offset = new SKPoint(0f, nextBottom - child.Size.Height - anchorY);
            child.Layout(canvasWidth, canvasHeight);
            nextBottom = child.Bounds.Top - Gap;
        }
    }
}
