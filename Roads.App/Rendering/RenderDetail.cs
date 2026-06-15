using System.Numerics;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// Static helpers for Level-of-Detail (LOD) zoom thresholds and frustum culling.
/// Centralises the decision of when to switch from detailed to simplified rendering
/// so both <see cref="RoadRenderer"/> and <see cref="VehicleRenderer"/> use the same
/// policy without duplicating constants.
///
/// LOD thresholds (zoom values are world-units per screen pixel):
///   VehicleDotThreshold  — below this zoom, draw vehicles as small dots instead of
///                          full body / windshield / lights. Set to 0.3 (approximately
///                          the point where a sedan becomes &lt;3 pixels wide and
///                          interior detail is imperceptible).
///   RoadSimpleThreshold  — below this zoom, draw roads as plain center-line strokes
///                          instead of surface + markings. Set to 0.15 (city-overview
///                          level where individual lanes are &lt;2 pixels and markings
///                          add noise rather than information).
/// </summary>
public static class RenderDetail
{
    /// <summary>
    /// Zoom below which vehicles are rendered as single filled dots rather than
    /// full body/windshield/headlight geometry. At this zoom a sedan body is roughly
    /// 2–3 screen pixels wide — too small for detail to be legible.
    /// </summary>
    public const float VehicleDotThreshold = 0.3f;

    /// <summary>
    /// Zoom below which roads are rendered as plain center-line strokes rather than
    /// full asphalt surfaces with lane markings. At this zoom individual lane widths
    /// are sub-pixel and markings produce visual noise.
    /// </summary>
    public const float RoadSimpleThreshold = 0.15f;

    /// <summary>
    /// Returns true when <paramref name="worldBounds"/> overlaps (or touches) the
    /// visible <paramref name="viewRect"/>. Uses SKRect's built-in IntersectsWith so
    /// touching edges count as visible — this errs toward drawing, which is the
    /// correct policy for frustum culling.
    /// </summary>
    /// <param name="worldBounds">AABB of the object in world space.</param>
    /// <param name="viewRect">Visible world-space rectangle from <c>Camera.GetVisibleWorldRect</c>.</param>
    public static bool IsVisible(SKRect worldBounds, SKRect viewRect)
        => viewRect.IntersectsWith(worldBounds);

    /// <summary>
    /// Builds a conservative world-space AABB for a vehicle centred at
    /// (<paramref name="posX"/>, <paramref name="posY"/>). Because the vehicle can be
    /// at any heading angle, the AABB uses half the vehicle diagonal as the half-extent
    /// in both axes — this over-approximates but is always correct and allocation-free.
    /// </summary>
    /// <param name="posX">Vehicle centre X in world space.</param>
    /// <param name="posY">Vehicle centre Y in world space.</param>
    /// <param name="halfLength">Half the render length of the vehicle.</param>
    /// <param name="halfWidth">Half the render width of the vehicle.</param>
    public static SKRect VehicleBounds(float posX, float posY, float halfLength, float halfWidth)
    {
        // Diagonal half-extent ensures the AABB covers all rotations without trig.
        float halfDiag = MathF.Sqrt(halfLength * halfLength + halfWidth * halfWidth);
        return new SKRect(posX - halfDiag, posY - halfDiag, posX + halfDiag, posY + halfDiag);
    }

    /// <summary>
    /// Builds a conservative world-space AABB for a road edge, given the positions of
    /// its two endpoint nodes and the total road half-width. Because the edge is a cubic
    /// Bézier that may bow outside the endpoint bounding box, an additional
    /// <paramref name="margin"/> (default 20 m — typically larger than any road half-width)
    /// is added on all sides so no visible geometry is accidentally culled.
    /// The road half-width is included so that the asphalt surface is fully enclosed.
    /// </summary>
    /// <param name="from">Position of the FromNode in world space.</param>
    /// <param name="to">Position of the ToNode in world space.</param>
    /// <param name="roadHalfWidth">Half the total road surface width (LaneCount * LaneWidth).</param>
    /// <param name="margin">Additional outward margin to account for Bézier bowing.</param>
    public static SKRect EdgeBounds(Vector2 from, Vector2 to,
        float roadHalfWidth, float margin = 20f)
    {
        float expand = roadHalfWidth + margin;
        float left   = MathF.Min(from.X, to.X) - expand;
        float right  = MathF.Max(from.X, to.X) + expand;
        float top    = MathF.Min(from.Y, to.Y) - expand;
        float bottom = MathF.Max(from.Y, to.Y) + expand;
        return new SKRect(left, top, right, bottom);
    }
}
