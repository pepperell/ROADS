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
    /// World-space radius of a node dot at the given zoom — the single size used by the
    /// road renderer's node dots AND every editor node visual derived from them (hover
    /// and selection highlights, ghost nodes), so a highlighted or previewed node always
    /// matches the node it represents. Small and roughly screen-constant (1.8 px) with a
    /// 1.2 m floor so dots never vanish when zoomed far in.
    /// </summary>
    public static float NodeDotRadius(float zoom) => MathF.Max(1.2f, 1.8f / zoom);

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
    /// Builds a conservative world-space AABB for a road edge from all four Bézier
    /// control points. A cubic Bézier lies entirely within the convex hull of its
    /// control points, so this AABB genuinely contains the curve — an endpoint-only
    /// box with a fixed fudge margin did not: with handles ≈ chord/3 a curve bows up
    /// to ~chord/4 outside the endpoint box, so long sweeping curves (chord ≳ 90 m)
    /// were culled while their belly was still on-screen. The road half-width encloses
    /// the asphalt surface; <paramref name="margin"/> now only covers strokes drawn
    /// just beyond it (curbs, dashed edge lines).
    /// </summary>
    /// <param name="from">Position of the FromNode in world space.</param>
    /// <param name="to">Position of the ToNode in world space.</param>
    /// <param name="cp1">First Bézier control point.</param>
    /// <param name="cp2">Second Bézier control point.</param>
    /// <param name="roadHalfWidth">Half the total road surface width (LaneCount * LaneWidth).</param>
    /// <param name="margin">Additional outward margin for strokes beyond the surface.</param>
    public static SKRect EdgeBounds(Vector2 from, Vector2 to, Vector2 cp1, Vector2 cp2,
        float roadHalfWidth, float margin = 5f)
    {
        float expand = roadHalfWidth + margin;
        float left   = MathF.Min(MathF.Min(from.X, to.X), MathF.Min(cp1.X, cp2.X)) - expand;
        float right  = MathF.Max(MathF.Max(from.X, to.X), MathF.Max(cp1.X, cp2.X)) + expand;
        float top    = MathF.Min(MathF.Min(from.Y, to.Y), MathF.Min(cp1.Y, cp2.Y)) - expand;
        float bottom = MathF.Max(MathF.Max(from.Y, to.Y), MathF.Max(cp1.Y, cp2.Y)) + expand;
        return new SKRect(left, top, right, bottom);
    }
}
