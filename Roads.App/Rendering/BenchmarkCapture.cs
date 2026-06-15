namespace Roads.App.Rendering;

/// <summary>
/// Static helper that appends a labeled one-line snapshot of performance metrics to
/// <c>benchmark.log</c> in the current working directory.
///
/// Intended to be called from a hotkey handler by the integration layer.  This class
/// only provides the API; the caller decides when to invoke it.
///
/// IO robustness: any exception during the file write is caught, logged to
/// <c>benchmark_error.log</c>, and then silently discarded — it never throws into the
/// caller.  This mirrors the IO-error handling pattern in
/// <see cref="Roads.App.Persistence.AutoSaveManager"/>.
/// </summary>
public static class BenchmarkCapture
{
    private const string LogFile = "benchmark.log";
    private const string ErrorLog = "benchmark_error.log";

    /// <summary>
    /// Appends a single labeled snapshot line to <c>benchmark.log</c>.
    /// </summary>
    /// <param name="fps">Current rolling-average frames per second.</param>
    /// <param name="simMs">Current rolling-average simulation time in milliseconds.</param>
    /// <param name="drawMs">Current rolling-average draw time in milliseconds.</param>
    /// <param name="pathfindMs">Total pathfinding time in milliseconds for the last frame.</param>
    /// <param name="pathfindCalls">Number of pathfinder calls in the last frame.</param>
    /// <param name="vehicleCount">Current number of vehicles in the simulation.</param>
    /// <param name="arcConflicts">DEBUG collision tripwire: pairs of vehicles on mutually-conflicting
    /// arcs after the last tick (should be 0 on the unsignalized stress grid; 0 in Release builds).</param>
    /// <param name="timing">Per-subsystem sim-tick breakdown from the most recent active tick.</param>
    /// <param name="steering">TEMP: per-vehicle steering sub-phase breakdown (diagnostic).</param>
    public static void Capture(double fps, double simMs, double drawMs,
                               double pathfindMs, int pathfindCalls, int vehicleCount, int arcConflicts,
                               Roads.App.SimulationLoop.SimTimingBreakdown timing,
                               Roads.App.Vehicles.SteeringController.SteeringProfile steering)
    {
        try
        {
            int gc0 = GC.CollectionCount(0);
            int gc1 = GC.CollectionCount(1);
            int gc2 = GC.CollectionCount(2);

            string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");

            string line = $"ts={timestamp}, fps={fps:F1}, sim={simMs:F2}ms, draw={drawMs:F2}ms, " +
                          $"pathfind={pathfindMs:F3}ms, pathfind_calls={pathfindCalls}, " +
                          $"vehicles={vehicleCount}, arc_conflicts={arcConflicts}, gc0={gc0}, gc1={gc1}, gc2={gc2}, " +
                          $"[sim breakdown] grid={timing.GridMs:F2}ms, caches={timing.CachesMs:F2}ms, " +
                          $"signals={timing.SignalsMs:F2}ms, lanechange={timing.LaneChangeMs:F2}ms, " +
                          $"steering={timing.SteeringMs:F2}ms, physics={timing.PhysicsMs:F2}ms, " +
                          $"reroute={timing.RerouteMs:F2}ms, population={timing.PopulationMs:F2}ms, " +
                          $"[steering] arc={steering.ArcMs:F2}ms, project={steering.ProjectMs:F2}ms, " +
                          $"signals={steering.SignalsMs:F2}ms, transition={steering.TransitionMs:F2}ms, " +
                          $"steer={steering.SteerMs:F2}ms, speed={steering.SpeedMs:F2}ms";

            File.AppendAllText(LogFile, line + Environment.NewLine);
        }
        catch (Exception ex)
        {
            try
            {
                File.AppendAllText(ErrorLog,
                    $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] BenchmarkCapture failed: {ex}\n");
            }
            catch
            {
                // If even the error log fails, swallow silently.
            }
        }
    }
}
