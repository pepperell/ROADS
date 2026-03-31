using System.Numerics;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Renders road markers (spawn points, destinations) as colored circles with a white
/// border and inner dot at their world-space positions. Parameterized by fill color
/// so a single renderer handles both spawn points and destinations.
/// </summary>
public class MarkerRenderer
{
    private readonly SKColor _fillColor;

    public MarkerRenderer(SKColor fillColor)
    {
        _fillColor = fillColor;
    }

    /// <summary>
    /// Draws all markers as colored circles with a white border and inner dot.
    /// </summary>
    /// <param name="canvas">SkiaSharp canvas in world-space coordinates.</param>
    /// <param name="items">List of items to render.</param>
    /// <param name="getPosition">Function to extract world position from each item.</param>
    /// <param name="zoom">Current camera zoom level for scaling circle sizes.</param>
    public void Draw<T>(SKCanvas canvas, List<T> items, Func<T, Vector2> getPosition, float zoom)
    {
        if (items.Count == 0) return;

        float radius = Math.Max(4f, 6f / zoom);
        float innerRadius = radius * 0.5f;

        using var fillPaint = new SKPaint
        {
            Color = _fillColor,
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };
        using var strokePaint = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 220),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = Math.Max(1f, 1.5f / zoom),
            IsAntialias = true,
        };
        using var innerPaint = new SKPaint
        {
            Color = new SKColor(255, 255, 255, 200),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };

        for (int i = 0; i < items.Count; i++)
        {
            var pos = getPosition(items[i]);
            canvas.DrawCircle(pos.X, pos.Y, radius, fillPaint);
            canvas.DrawCircle(pos.X, pos.Y, radius, strokePaint);
            canvas.DrawCircle(pos.X, pos.Y, innerRadius, innerPaint);
        }
    }
}
