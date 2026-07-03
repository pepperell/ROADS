using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Static per-<see cref="RoadType"/> visual style table. Provides the asphalt surface color,
/// a width multiplier applied to the lane-derived road width, the shoulder/sidewalk/verge
/// band (width + color) drawn under and outside the asphalt, and the brightened schematic
/// color/stroke used at city-overview zoom. Color helpers take an ambient lighting factor
/// (1.0 = full day) and dim RGB channels for night; geometry helpers are zoom-independent.
/// Unpaved types (dirt) carry no painted lane markings.
/// </summary>
public static class RoadTypeVisuals
{
    /// <summary>
    /// Returns the asphalt surface color for the given road type, dimmed by the ambient
    /// lighting factor (1.0 = full day, lower = night). Residential reads as worn light
    /// asphalt, arterial mid grey, highway dark grey, dirt brown — so classification is
    /// legible from tone alone.
    /// </summary>
    /// <param name="type">Road classification.</param>
    /// <param name="ambient">Ambient lighting factor in [0, 1].</param>
    public static SKColor GetSurfaceColor(RoadType type, float ambient)
    {
        return type switch
        {
            // Highway — darkest asphalt (fresh, heavily trafficked)
            RoadType.Highway     => new SKColor(Dim(56, ambient), Dim(58, ambient), Dim(66, ambient)),
            // Arterial — mid grey
            RoadType.Arterial    => new SKColor(Dim(66, ambient), Dim(68, ambient), Dim(74, ambient)),
            // Residential — worn light asphalt
            RoadType.Residential => new SKColor(Dim(82, ambient), Dim(84, ambient), Dim(88, ambient)),
            // Dirt — packed earth brown
            RoadType.Dirt        => new SKColor(Dim(124, ambient), Dim(100, ambient), Dim(64, ambient)),
            _                    => new SKColor(Dim(82, ambient), Dim(84, ambient), Dim(88, ambient)),
        };
    }

    /// <summary>
    /// Returns the stroke-width multiplier for the given road type. The multiplier is
    /// applied to the lane-derived width (<see cref="GeometryUtil.RoadSurfaceWidth"/>)
    /// so that road type visually conveys importance without affecting lane geometry.
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
            _                    => 1.00f,
        };
    }

    /// <summary>
    /// Width in meters of the shoulder band drawn on EACH side of the asphalt (sidewalk for
    /// residential/arterial, gravel shoulder for highway, worn verge for dirt). The band is
    /// rendered by stroking the road center path at asphalt width plus twice this value,
    /// underneath the asphalt pass.
    /// </summary>
    public static float GetShoulderWidth(RoadType type)
    {
        return type switch
        {
            RoadType.Highway     => 2.5f,  // gravel shoulder
            RoadType.Arterial    => 1.5f,  // narrow sidewalk
            RoadType.Residential => 2.0f,  // concrete sidewalk
            RoadType.Dirt        => 1.2f,  // worn dirt verge
            _                    => 2.0f,
        };
    }

    /// <summary>
    /// Color of the shoulder/sidewalk/verge band for the given road type, dimmed by the
    /// ambient lighting factor. Concrete tones for sidewalks, gravel grey for highway
    /// shoulders, dark earth for dirt verges.
    /// </summary>
    public static SKColor GetShoulderColor(RoadType type, float ambient)
    {
        return type switch
        {
            RoadType.Highway     => new SKColor(Dim(98, ambient), Dim(94, ambient), Dim(86, ambient)),
            RoadType.Arterial    => new SKColor(Dim(128, ambient), Dim(126, ambient), Dim(120, ambient)),
            RoadType.Residential => new SKColor(Dim(152, ambient), Dim(150, ambient), Dim(142, ambient)),
            RoadType.Dirt        => new SKColor(Dim(105, ambient), Dim(86, ambient), Dim(58, ambient)),
            _                    => new SKColor(Dim(152, ambient), Dim(150, ambient), Dim(142, ambient)),
        };
    }

    /// <summary>
    /// Screen-space stroke width (pixels) used for the schematic center-line rendering at
    /// city-overview zoom (below <see cref="RenderDetail.RoadSimpleThreshold"/>). Callers
    /// divide by zoom to convert to world units. Widths grade by importance so the network
    /// hierarchy stays legible when roads collapse to lines.
    /// </summary>
    public static float GetSchematicStrokeWidth(RoadType type)
    {
        return type switch
        {
            RoadType.Highway     => 2.6f,
            RoadType.Arterial    => 2.0f,
            RoadType.Residential => 1.3f,
            RoadType.Dirt        => 1.0f,
            _                    => 1.3f,
        };
    }

    /// <summary>
    /// Surface color brightened ~1.4x for the schematic (LOD-simple) rendering, dimmed by
    /// the ambient lighting factor. Brightening keeps the thin center-line strokes visible
    /// against the terrain at city-overview zoom.
    /// </summary>
    public static SKColor GetSchematicColor(RoadType type, float ambient)
    {
        return type switch
        {
            RoadType.Highway     => new SKColor(Dim(78, ambient), Dim(81, ambient), Dim(92, ambient)),
            RoadType.Arterial    => new SKColor(Dim(92, ambient), Dim(95, ambient), Dim(104, ambient)),
            RoadType.Residential => new SKColor(Dim(115, ambient), Dim(118, ambient), Dim(123, ambient)),
            RoadType.Dirt        => new SKColor(Dim(174, ambient), Dim(140, ambient), Dim(90, ambient)),
            _                    => new SKColor(Dim(115, ambient), Dim(118, ambient), Dim(123, ambient)),
        };
    }

    /// <summary>
    /// Returns true when the road type carries painted lane markings (edge lines, center
    /// line, lane dividers). Unpaved types — currently <see cref="RoadType.Dirt"/> — have
    /// no paint, so callers should skip all marking rendering for them (dirt shows worn
    /// tire tracks instead).
    /// </summary>
    public static bool HasPaintedLines(RoadType type) => type != RoadType.Dirt;

    /// <summary>
    /// Visual importance rank — Highway &gt; Arterial &gt; Residential &gt; Dirt. Note this is NOT
    /// the <see cref="RoadType"/> enum order (Dirt has the highest enum value but the lowest rank).
    /// An intersection takes on the appearance of its highest-ranked incident road.
    /// </summary>
    public static int GetRank(RoadType type) => type switch
    {
        RoadType.Highway     => 3,
        RoadType.Arterial    => 2,
        RoadType.Residential => 1,
        RoadType.Dirt        => 0,
        _                    => 1,
    };

    private static byte Dim(int baseValue, float ambient) =>
        (byte)Math.Clamp((int)(baseValue * ambient), 0, 255);
}
