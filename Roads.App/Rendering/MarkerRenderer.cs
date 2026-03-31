using System.Numerics;
using SkiaSharp;
using Roads.App.World;

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
    /// Draws markers at all non-defunct nodes that have the specified flag set.
    /// </summary>
    public void DrawForFlag(SKCanvas canvas, RoadGraph graph, NodeFlags flag, float zoom)
    {
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

        var nodes = graph.Nodes;
        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if (!node.Flags.HasFlag(flag)) continue;
            canvas.DrawCircle(node.Position.X, node.Position.Y, radius, fillPaint);
            canvas.DrawCircle(node.Position.X, node.Position.Y, radius, strokePaint);
            canvas.DrawCircle(node.Position.X, node.Position.Y, innerRadius, innerPaint);
        }
    }
}
