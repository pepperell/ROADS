using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Shared, process-lifetime UI resources: fonts, common colors, the POI palette, and
/// scratch paints. Everything here is created once and never disposed. The scratch
/// paints have their Color reassigned per draw call — safe only because all UI drawing
/// happens on the single render thread.
/// </summary>
public static class UiTheme
{
    // Shared fonts (never dispose; Labels reference these rather than owning fonts).
    public static readonly SKFont Font11 = new() { Size = 11 };
    public static readonly SKFont Font12 = new() { Size = 12 };
    public static readonly SKFont Font13 = new() { Size = 13 };
    public static readonly SKFont Font14 = new() { Size = 14 };
    /// <summary>Large bold font for the full-screen menu buttons (title screen / pause menu).</summary>
    public static readonly SKFont FontMenu = new(SKTypeface.FromFamilyName(null, SKFontStyle.Bold), 20);

    // Common chrome colors.
    public static readonly SKColor PanelBackground = new(30, 32, 38, 220);
    public static readonly SKColor HudBackground = new(20, 22, 28, 200);
    public static readonly SKColor Outline = new(80, 82, 88);
    public static readonly SKColor TextPrimary = new(200, 200, 200);
    public static readonly SKColor TextDim = new(170, 170, 170);
    public static readonly SKColor Value = new(100, 200, 255);

    /// <summary>POI type colors, indexed by (POIType - 1). Single source for the POI
    /// submenu buttons, destination hover/ghost tints, and building far-zoom dots.</summary>
    public static readonly SKColor[] PoiColors =
    {
        new SKColor(60, 130, 220, 200),   // Home — blue
        new SKColor(140, 140, 150, 200),  // Work — gray
        new SKColor(220, 150, 40, 200),   // Shop — orange
        new SKColor(60, 180, 80, 200),    // Leisure — green
        new SKColor(200, 190, 50, 200),   // School — yellow
        new SKColor(60, 180, 200, 200),   // Parking — cyan
        new SKColor(190, 80, 200, 200),   // EntryExit — magenta
    };

    /// <summary>Palette lookup with the fallback red used for POIType.None / out-of-range
    /// values (matches the historical destination-dot fallback).</summary>
    public static SKColor PoiColor(POIType type)
    {
        int idx = (int)type - 1;
        return idx >= 0 && idx < PoiColors.Length ? PoiColors[idx] : new SKColor(200, 60, 40, 200);
    }

    // Scratch paints for panel chrome and text. Color (and for StrokeScratch, width) are
    // set immediately before each use; never hold these across a draw call boundary.
    public static readonly SKPaint FillScratch = new() { Style = SKPaintStyle.Fill, IsAntialias = true };
    public static readonly SKPaint StrokeScratch = new() { Style = SKPaintStyle.Stroke, IsAntialias = true };
    public static readonly SKPaint TextScratch = new() { IsAntialias = true };
}
