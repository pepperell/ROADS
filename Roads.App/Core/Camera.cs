using SkiaSharp;

namespace Roads.App.Core;

/// <summary>
/// 2D camera providing pan, zoom, and coordinate transforms between screen space and
/// world space. Zoom is clamped between MinZoom (city overview) and MaxZoom (street level).
/// World origin (0,0) maps to center of screen when CenterX/CenterY are zero.
/// </summary>
public class Camera
{
    /// <summary>Horizontal pan offset in screen pixels.</summary>
    public float CenterX { get; set; }
    /// <summary>Vertical pan offset in screen pixels.</summary>
    public float CenterY { get; set; }
    /// <summary>Current zoom level (world units per pixel). Higher = closer.</summary>
    public float Zoom { get; set; } = 5.0f;

    /// <summary>Minimum zoom for city overview.</summary>
    private const float MinZoom = 0.01f;
    /// <summary>Maximum zoom for street-level detail.</summary>
    private const float MaxZoom = 100f;

    /// <summary>Translates the camera by a screen-space delta (pixels).</summary>
    public void Pan(float screenDx, float screenDy)
    {
        CenterX += screenDx;
        CenterY += screenDy;
    }

    /// <summary>
    /// Zooms toward a screen-space point so the world position under the cursor stays fixed.
    /// </summary>
    /// <param name="factor">Zoom multiplier (>1 = zoom in, &lt;1 = zoom out).</param>
    /// <param name="screenX">Screen X coordinate of the zoom focus point.</param>
    /// <param name="screenY">Screen Y coordinate of the zoom focus point.</param>
    /// <param name="viewWidth">Width of the viewport in pixels.</param>
    /// <param name="viewHeight">Height of the viewport in pixels.</param>
    public void ZoomAt(float factor, int screenX, int screenY, int viewWidth, int viewHeight)
    {
        float newZoom = Math.Clamp(Zoom * factor, MinZoom, MaxZoom);
        float actualFactor = newZoom / Zoom;

        // Zoom toward mouse position
        float offsetX = screenX - viewWidth / 2f - CenterX;
        float offsetY = screenY - viewHeight / 2f - CenterY;
        CenterX -= offsetX * (actualFactor - 1f);
        CenterY -= offsetY * (actualFactor - 1f);

        Zoom = newZoom;
    }

    /// <summary>Returns the world-to-screen transformation matrix for SkiaSharp rendering.</summary>
    public SKMatrix GetTransformMatrix(int viewWidth, int viewHeight)
    {
        // Translate so that (0,0) world is at center of screen, then apply pan and zoom
        var matrix = SKMatrix.CreateTranslation(viewWidth / 2f + CenterX, viewHeight / 2f + CenterY);
        matrix = matrix.PreConcat(SKMatrix.CreateScale(Zoom, Zoom));
        return matrix;
    }

    /// <summary>Converts a screen-space pixel coordinate to world-space position.</summary>
    public SKPoint ScreenToWorld(int screenX, int screenY, int viewWidth, int viewHeight)
    {
        var matrix = GetTransformMatrix(viewWidth, viewHeight);
        if (!matrix.TryInvert(out var inverse))
            return new SKPoint(screenX, screenY); // singular matrix — return screen coords as fallback
        return inverse.MapPoint(screenX, screenY);
    }
}
