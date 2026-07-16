using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Persistence;

/// <summary>
/// Periodically writes timestamped backup files of the current map to a <c>backups/</c>
/// subfolder (relative to the working directory), pruning the oldest when the count
/// exceeds <see cref="MaxBackups"/>.
///
/// Default policy: save every <see cref="IntervalSeconds"/> seconds (300 s = 5 minutes),
/// keep the most recent <see cref="MaxBackups"/> files (5).
///
/// Save-while-paused: backups are written regardless of the simulation's paused state.
/// The map state is equally valid when paused and the user may pause for extended review
/// periods, so withholding backups during a pause would defeat the purpose.
///
/// IO robustness: any exception during backup is caught and written to
/// <c>autosave_error.log</c>, then silently discarded.  A failed backup never crashes
/// or stalls the simulation or UI.
///
/// Thread safety: intended to be called from the WinForms UI thread only (via a
/// <see cref="System.Windows.Forms.Timer"/> tick).
/// </summary>
public class AutoSaveManager
{
    /// <summary>Subfolder name (relative to working directory) for backup files.</summary>
    private const string BackupDir = "backups";

    /// <summary>Prefix used for all auto-save file names.</summary>
    private const string FilePrefix = "autosave_";

    /// <summary>How many seconds between automatic saves (default 5 minutes).</summary>
    public double IntervalSeconds { get; set; } = 300.0;

    /// <summary>Maximum number of backup files to retain; oldest beyond this are deleted.</summary>
    public int MaxBackups { get; set; } = 5;

    private double _elapsed;

    // Save-target fields injected at construction time.
    private readonly RoadGraph _graph;
    private readonly VehicleStore _vehicles;
    private readonly Camera _camera;
    private readonly SimulationClock _clock;
    private readonly StopSignSystem _stopSigns;
    private readonly YieldSignSystem _yieldSigns;
    private readonly TrafficSignalSystem _signals;
    private readonly PopulationManager _population;
    private readonly WaterLayer _water;

    /// <summary>
    /// Constructs an <see cref="AutoSaveManager"/> with references to every object
    /// required by <see cref="MapSerializer.Save"/>.
    /// Must be called after all the objects above are fully initialised.
    /// </summary>
    public AutoSaveManager(
        RoadGraph graph,
        VehicleStore vehicles,
        Camera camera,
        SimulationClock clock,
        StopSignSystem stopSigns,
        YieldSignSystem yieldSigns,
        TrafficSignalSystem signals,
        PopulationManager population,
        WaterLayer water)
    {
        _graph = graph;
        _vehicles = vehicles;
        _camera = camera;
        _clock = clock;
        _stopSigns = stopSigns;
        _yieldSigns = yieldSigns;
        _signals = signals;
        _population = population;
        _water = water;
    }

    /// <summary>
    /// Accumulates <paramref name="elapsedSeconds"/> of wall time and writes a backup
    /// when <see cref="IntervalSeconds"/> has elapsed.  Does not save if the graph
    /// contains no nodes (nothing meaningful to back up).
    /// Call from the WinForms render timer tick (approximately every 16 ms / ~60 FPS).
    /// </summary>
    public void MaybeSave(double elapsedSeconds)
    {
        _elapsed += elapsedSeconds;
        if (_elapsed < IntervalSeconds)
            return;

        _elapsed = 0;

        // Skip entirely if there is nothing on the map.
        if (_graph.Nodes.Count == 0)
            return;

        TrySave();
    }

    /// <summary>
    /// Performs the backup write unconditionally and resets the interval timer.
    /// Errors are logged to <c>autosave_error.log</c> and swallowed.
    /// </summary>
    private void TrySave()
    {
        try
        {
            string dir = BackupDir;
            Directory.CreateDirectory(dir);

            string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            string path = Path.Combine(dir, $"{FilePrefix}{timestamp}.roads");

            // Save without vehicles — backups are map-geometry snapshots;
            // vehicle state is transient and large.
            MapSerializer.Save(path, _graph, _vehicles, _camera, _clock,
                _stopSigns, _yieldSigns, _signals, _population, _water, includeVehicles: false);

            PruneOldBackups(dir);
        }
        catch (Exception ex)
        {
            try
            {
                File.AppendAllText("autosave_error.log",
                    $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] Auto-save failed: {ex}\n");
            }
            catch
            {
                // If even the error log fails, swallow silently.
            }
        }
    }

    /// <summary>
    /// Deletes the oldest auto-save files in <paramref name="dir"/> so that at most
    /// <see cref="MaxBackups"/> backup files remain.
    /// Only files matching the <c>autosave_*.roads</c> pattern are considered.
    /// </summary>
    private void PruneOldBackups(string dir)
    {
        var files = Directory.GetFiles(dir, $"{FilePrefix}*.roads")
            .OrderBy(f => f)   // lexicographic = chronological for the timestamp format
            .ToList();

        int toDelete = files.Count - MaxBackups;
        for (int i = 0; i < toDelete; i++)
            File.Delete(files[i]);
    }
}
