using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using SkiaSharp;

namespace Roads.App.Rendering;

/// <summary>
/// WinForms control that renders via SkiaSharp. Creates an offscreen SKSurface each frame,
/// invokes the PaintSurface event for drawing, then blits the result to the GDI+ Graphics
/// context using a raw pixel copy. Caches the GDI+ Bitmap across frames to reduce allocations.
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

        var info = new SKImageInfo(width, height, SKColorType.Bgra8888, SKAlphaType.Premul);
        using var surface = SKSurface.Create(info);
        if (surface == null) return;

        var canvas = surface.Canvas;
        PaintSurface?.Invoke(this, canvas, info);

        using var image = surface.Snapshot();
        using var pixmap = image.PeekPixels();

        // Reuse the cached bitmap if size hasn't changed; otherwise allocate a new one
        if (_cachedBitmap == null || _cachedWidth != width || _cachedHeight != height)
        {
            _cachedBitmap?.Dispose();
            _cachedBitmap = new Bitmap(width, height, PixelFormat.Format32bppPArgb);
            _cachedWidth = width;
            _cachedHeight = height;
        }

        var bits = _cachedBitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format32bppPArgb);

        var srcPtr = pixmap.GetPixels();
        int byteCount = width * height * 4;

        unsafe
        {
            Buffer.MemoryCopy(srcPtr.ToPointer(), bits.Scan0.ToPointer(), byteCount, byteCount);
        }

        _cachedBitmap.UnlockBits(bits);
        e.Graphics.DrawImageUnscaled(_cachedBitmap, 0, 0);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
            _cachedBitmap?.Dispose();
        base.Dispose(disposing);
    }
}
