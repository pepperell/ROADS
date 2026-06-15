using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Static visual style lookup for each <see cref="RoadType"/>. Provides the asphalt surface
/// color and a width multiplier applied to the lane-derived road width, so road types are
/// visually distinct without altering lane geometry or Bézier paths.
/// Unpaved types (dirt) carry no painted lane markings.
/// </summary>
public static class RoadTypeVisuals
{
    /// <summary>
    /// Returns the asphalt surface color for the given road type, dimmed by the ambient
    /// lighting factor (1.0 = full day, lower = night). Each type has a distinct hue so
    /// road classification is immediately legible on the map.
    /// </summary>
    /// <param name="type">Road classification.</param>
    /// <param name="ambient">Ambient lighting factor in [0, 1].</param>
    public static SKColor GetSurfaceColor(RoadType type, float ambient)
    {
        return type switch
        {
            // Highway — medium grey (blue-grey tint), widest
            RoadType.Highway    => new SKColor(Dim(90, ambient), Dim(95, ambient), Dim(105, ambient)),
            // Arterial — slightly warmer grey
            RoadType.Arterial   => new SKColor(Dim(75, ambient), Dim(77, ambient), Dim(84, ambient)),
            // Residential — original dark grey (unchanged baseline)
            RoadType.Residential => new SKColor(Dim(70, ambient), Dim(72, ambient), Dim(78, ambient)),
            // Dirt — brown/tan
            RoadType.Dirt       => new SKColor(Dim(110, ambient), Dim(88, ambient), Dim(55, ambient)),
            _                   => new SKColor(Dim(70, ambient), Dim(72, ambient), Dim(78, ambient)),
        };
    }

    /// <summary>
    /// Returns the stroke-width multiplier for the given road type. The multiplier is
    /// applied to the lane-derived width (<c>LaneCount * 2 * LaneWidth</c>) so that road
    /// type visually conveys importance without affecting lane geometry.
    /// Highway roads appear noticeably wider; dirt roads appear narrower.
    /// </summary>
    /// <param name="type">Road classification.</param>
    public static float GetWidthMultiplier(RoadType type)
    {
        return type switch
        {
            RoadType.Highway     => 1.25f,
            RoadType.Arterial    => 1.10f,
            RoadType.Residential => 1.00f,
            RoadType.Dirt        => 0.85f,
            _                   => 1.00f,
        };
    }

    /// <summary>
    /// Returns true when the road type carries painted lane markings (edge lines, center
    /// line, lane dividers). Unpaved types — currently <see cref="RoadType.Dirt"/> — have
    /// no paint, so callers should skip all marking rendering for them.
    /// </summary>
    public static bool HasPaintedLines(RoadType type) => type != RoadType.Dirt;

    private static byte Dim(int baseValue, float ambient) =>
        (byte)Math.Clamp((int)(baseValue * ambient), 0, 255);
}
