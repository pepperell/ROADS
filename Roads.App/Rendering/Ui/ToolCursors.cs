using System.Runtime.InteropServices;
using SkiaSharp;
using Roads.App.Editor;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Per-tool mouse cursors, drawn procedurally with SkiaSharp (matching the codebase's
/// no-shipped-assets ethos) and converted to real Windows cursors with proper hotspots
/// via CreateIconIndirect. Cursors are built once on first use and cached for the
/// process lifetime (the OS reclaims the handles at exit — WinForms' Cursor does not
/// own handles passed to it, so there is deliberately no disposal). Any failure in the
/// Win32 conversion falls back to the system arrow: a cursor must never crash the app.
///
/// <see cref="ForTool"/> is the single mapping point — for now every tool (and the UI)
/// shares the finger pointer, so new tool cursors are one drawing method plus one
/// switch arm. MainForm applies the result once per frame (see MainForm.UpdateCursor).
/// </summary>
public static class ToolCursors
{
    private static Cursor? _fingerPointer;

    /// <summary>The classic pointing-hand cursor — the Select tool.</summary>
    public static Cursor FingerPointer => _fingerPointer ??= CreateFingerPointer();

    /// <summary>Cursor for the given editor tool. For now the finger pointer is the
    /// application-wide cursor — every tool falls through to it until it grows its own
    /// art (add a drawing method and a switch arm here).</summary>
    public static Cursor ForTool(EditorTool tool) => tool switch
    {
        _ => FingerPointer,
    };

    // ═══════════════════════ Cursor art ═══════════════════════

    /// <summary>Draws the pointing hand: extended index finger (hotspot at its tip),
    /// three curled knuckles, palm with a heel bump — white fill, dark outline, the
    /// standard cursor look so it reads on any map background.</summary>
    private static Cursor CreateFingerPointer()
    {
        const int size = 32;
        try
        {
            using var bitmap = new SKBitmap(size, size, SKColorType.Bgra8888, SKAlphaType.Premul);
            using (var canvas = new SKCanvas(bitmap))
            {
                canvas.Clear(SKColors.Transparent);

                // Hand silhouette: union of rounded rects (index, three knuckles, palm)
                // plus the heel-of-hand bump, so fill and outline see one closed shape.
                var shape = new SKPath();
                AddRoundRect(ref shape, SKRect.Create(12.5f, 1.5f, 5f, 16.5f), 2.5f);  // index finger
                AddRoundRect(ref shape, SKRect.Create(17.5f, 8f, 4f, 8f), 2f);          // middle knuckle
                AddRoundRect(ref shape, SKRect.Create(21.5f, 9.5f, 4f, 8f), 2f);        // ring knuckle
                AddRoundRect(ref shape, SKRect.Create(25.5f, 11f, 3.5f, 7.5f), 1.75f);  // pinky knuckle
                AddRoundRect(ref shape, SKRect.Create(12.5f, 14.5f, 16.5f, 14f), 4.5f); // palm
                using (var heel = new SKPath())
                {
                    heel.AddCircle(12.2f, 23f, 3.8f);
                    UnionInto(ref shape, heel);
                }

                using var fill = new SKPaint
                {
                    IsAntialias = true,
                    Style = SKPaintStyle.Fill,
                    Color = new SKColor(250, 250, 250),
                };
                canvas.DrawPath(shape, fill);

                using var stroke = new SKPaint
                {
                    IsAntialias = true,
                    Style = SKPaintStyle.Stroke,
                    StrokeWidth = 1.4f,
                    StrokeJoin = SKStrokeJoin.Round,
                    StrokeCap = SKStrokeCap.Round,
                    Color = new SKColor(20, 22, 26),
                };
                canvas.DrawPath(shape, stroke);

                // Knuckle separations — the creases that make it read as a hand.
                stroke.StrokeWidth = 1.1f;
                canvas.DrawLine(17.5f, 9.2f, 17.5f, 15.2f, stroke);
                canvas.DrawLine(21.5f, 10.7f, 21.5f, 16.5f, stroke);
                canvas.DrawLine(25.5f, 12.2f, 25.5f, 17.5f, stroke);

                shape.Dispose();
            }
            return FromSkBitmap(bitmap, hotX: 15, hotY: 2);
        }
        catch
        {
            return Cursors.Default;
        }
    }

    /// <summary>Unions a rounded rect into the running silhouette path.</summary>
    private static void AddRoundRect(ref SKPath shape, SKRect rect, float radius)
    {
        using var piece = new SKPath();
        piece.AddRoundRect(new SKRoundRect(rect, radius));
        UnionInto(ref shape, piece);
    }

    private static void UnionInto(ref SKPath shape, SKPath piece)
    {
        var merged = shape.Op(piece, SKPathOp.Union);
        if (merged == null) return; // op failure: keep what we have rather than lose the shape
        shape.Dispose();
        shape = merged;
    }

    // ═══════════════════════ SKBitmap → Windows cursor ═══════════════════════

    /// <summary>Converts rendered BGRA pixels to an HCURSOR with the given hotspot:
    /// wrap the pixels in a GDI+ bitmap (same memory, no copy), take an HICON, then
    /// rebuild it as a cursor via CreateIconIndirect. The intermediate GDI bitmaps
    /// returned by GetIconInfo are owned copies and must be deleted or they leak.</summary>
    private static Cursor FromSkBitmap(SKBitmap bitmap, int hotX, int hotY)
    {
        using var gdi = new System.Drawing.Bitmap(bitmap.Width, bitmap.Height, bitmap.RowBytes,
            System.Drawing.Imaging.PixelFormat.Format32bppPArgb, bitmap.GetPixels());
        IntPtr hIcon = gdi.GetHicon();
        try
        {
            if (!GetIconInfo(hIcon, out IconInfo info)) return Cursors.Default;
            info.fIcon = false;
            info.xHotspot = (uint)hotX;
            info.yHotspot = (uint)hotY;
            IntPtr hCursor = CreateIconIndirect(ref info);
            DeleteObject(info.hbmColor);
            DeleteObject(info.hbmMask);
            return hCursor == IntPtr.Zero ? Cursors.Default : new Cursor(hCursor);
        }
        finally
        {
            DestroyIcon(hIcon);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct IconInfo
    {
        public bool fIcon;
        public uint xHotspot;
        public uint yHotspot;
        public IntPtr hbmMask;
        public IntPtr hbmColor;
    }

    [DllImport("user32.dll")]
    private static extern bool GetIconInfo(IntPtr hIcon, out IconInfo info);

    [DllImport("user32.dll")]
    private static extern IntPtr CreateIconIndirect(ref IconInfo info);

    [DllImport("user32.dll")]
    private static extern bool DestroyIcon(IntPtr hIcon);

    [DllImport("gdi32.dll")]
    private static extern bool DeleteObject(IntPtr hObject);
}
