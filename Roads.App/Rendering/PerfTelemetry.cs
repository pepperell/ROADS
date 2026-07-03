using System.Diagnostics;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Frame-timing telemetry, split out of the performance HUD so measurement no longer
/// depends on a UI panel being drawn. Values are instantaneous per-frame (no averaging),
/// so readouts reflect the current frame immediately — they do not ramp while a buffer
/// refills after an unpause.
/// Call order per frame: <see cref="RecordSimTime"/> (from the tick), then
/// <see cref="RecordDrawTime"/> after the scene render, then <see cref="Sample"/> —
/// which MUST run exactly once per rendered frame regardless of HUD visibility: it is
/// the single consumer/resetter of the pathfind accumulators
/// (<see cref="Pathfinder.ReadPathfindStatsAndReset"/>) and the publisher of the
/// properties BenchmarkCapture reads.
/// </summary>
public class PerfTelemetry
{
    private double _lastSim;
    private double _lastDraw;
    private double _lastFrame;
    private long _lastFrameTick;
    private readonly Stopwatch _stopwatch = Stopwatch.StartNew();

    /// <summary>Most recent frame's FPS (instantaneous, not averaged; the "Avg" name is
    /// kept for source compatibility with historical BenchmarkCapture columns).</summary>
    public double AvgFps { get; private set; }

    /// <summary>Most recent frame's simulation time in milliseconds (instantaneous).</summary>
    public double AvgSimMs { get; private set; }

    /// <summary>Most recent frame's draw time in milliseconds (instantaneous).</summary>
    public double AvgDrawMs { get; private set; }

    /// <summary>Most recent frame's total wall time in milliseconds.</summary>
    public double LastFrameMs => _lastFrame;

    /// <summary>Pathfinding time (ms) consumed by the last <see cref="Sample"/>.</summary>
    public double LastPathfindMs { get; private set; }

    /// <summary>Pathfinder call count consumed by the last <see cref="Sample"/>.</summary>
    public int LastPathfindCalls { get; private set; }

    /// <summary>Records the simulation time for the current frame in milliseconds.</summary>
    public void RecordSimTime(double milliseconds) => _lastSim = milliseconds;

    /// <summary>
    /// Records the draw time for the current frame in milliseconds and computes the
    /// total frame time from wall clock. No averaging — values are per-frame.
    /// </summary>
    public void RecordDrawTime(double milliseconds)
    {
        _lastDraw = milliseconds;

        long now = _stopwatch.ElapsedTicks;
        if (_lastFrameTick > 0)
            _lastFrame = (now - _lastFrameTick) * 1000.0 / Stopwatch.Frequency;
        _lastFrameTick = now;
    }

    /// <summary>
    /// Publishes the frame's values and drains the pathfind accumulators. Must run once
    /// per rendered frame (MainForm.OnPaintSurface calls it unconditionally); skipping it
    /// would let the accumulators grow unboundedly, and calling it twice per frame would
    /// halve the reported pathfind numbers.
    /// </summary>
    public void Sample()
    {
        var (pathfindMs, pathfindCalls) = Pathfinder.ReadPathfindStatsAndReset();
        LastPathfindMs = pathfindMs;
        LastPathfindCalls = pathfindCalls;

        AvgFps = _lastFrame > 0 ? 1000.0 / _lastFrame : 0;
        AvgSimMs = _lastSim;
        AvgDrawMs = _lastDraw;
    }
}
