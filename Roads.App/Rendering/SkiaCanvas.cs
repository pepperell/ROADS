using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// WinForms control that renders via SkiaSharp. Each paint wraps the cached GDI+ bitmap's
/// locked pixel memory in an SKSurface (a small native wrapper — the pixel buffer belongs
/// to the bitmap), invokes the PaintSurface event to draw into it, then blits the bitmap
/// to the GDI+ Graphics context. The bitmap is cached across frames and reallocated only
/// on resize, so a paint performs no full-frame allocation and no pixel copy.
/// </summary>
public class SkiaCanvas : Control
{
    /// <summary>Raised each frame with the offscreen SKCanvas and image info for drawing.</summary>
    public event Action<object?, SKCanvas, SKImageInfo>? PaintSurface;

    /// <summary>Cached GDI+ bitmap reused across frames when size is unchanged.</summary>
    private Bitmap? _cachedBitmap;
    /// <summary>Width of the cached bitmap.</summary>
    private int _cachedWidth;
    /// <summary>Height of the cached bitmap.</summary>
    private int _cachedHeight;

    public SkiaCanvas()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        int width = Width;
        int height = Height;
        if (width <= 0 || height <= 0) return;

        // Reuse the cached bitmap if size hasn't changed; otherwise allocate a new one
        if (_cachedBitmap == null || _cachedWidth != width || _cachedHeight != height)
        {
            _cachedBitmap?.Dispose();
            _cachedBitmap = new Bitmap(width, height, PixelFormat.Format32bppPArgb);
            _cachedWidth = width;
            _cachedHeight = height;
        }

        // Draw straight into the bitmap's pixel memory: SKSurface.Create with an external
        // pointer allocates only a tiny native wrapper — the pixels are the bitmap's — so
        // there is no per-frame surface allocation and no Snapshot/PeekPixels/MemoryCopy.
        // (The previous per-paint SKSurface.Create(info) + snapshot + full-frame copy cost
        // ~15 MB of native alloc/free per frame at 1440p.) Bgra8888/Premul matches
        // Format32bppPArgb byte-for-byte; bits.Stride is passed so any GDI+ row padding
        // is respected.
        var bits = _cachedBitmap.LockBits(new Rectangle(0, 0, width, height),
            ImageLockMode.WriteOnly, PixelFormat.Format32bppPArgb);
        try
        {
            var info = new SKImageInfo(width, height, SKColorType.Bgra8888, SKAlphaType.Premul);
            using var surface = SKSurface.Create(info, bits.Scan0, bits.Stride);
            if (surface == null) return;
            PaintSurface?.Invoke(this, surface.Canvas, info);
        }
        finally
        {
            _cachedBitmap.UnlockBits(bits);
        }

        e.Graphics.DrawImageUnscaled(_cachedBitmap, 0, 0);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
            _cachedBitmap?.Dispose();
        base.Dispose(disposing);
    }
}
