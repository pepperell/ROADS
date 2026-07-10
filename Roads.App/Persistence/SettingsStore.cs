using System.Text.Json;
using Roads.App.Core;

namespace Roads.App.Persistence;

/// <summary>
/// Loads and saves the application's <see cref="AppSettings"/> as indented JSON in
/// <c>settings.json</c> in the working directory (beside the map files and backups/).
/// Fault-tolerant in both directions: a missing or unreadable file yields defaults, and
/// a failed write is swallowed (the AutoSaveManager idiom) — persistence must never
/// crash or block the app.
/// </summary>
public static class SettingsStore
{
    private const string FileName = "settings.json";

    private static readonly JsonSerializerOptions _options = new() { WriteIndented = true };

    /// <summary>Reads settings from disk; any failure (missing file, corrupt JSON) returns
    /// defaults. Members absent from the file keep their <see cref="AppSettings"/> defaults.</summary>
    public static AppSettings Load()
    {
        try
        {
            if (!File.Exists(FileName)) return new AppSettings();
            return JsonSerializer.Deserialize<AppSettings>(File.ReadAllText(FileName), _options)
                ?? new AppSettings();
        }
        catch
        {
            return new AppSettings();
        }
    }

    /// <summary>Writes settings to disk; failures are swallowed.</summary>
    public static void Save(AppSettings settings)
    {
        try
        {
            File.WriteAllText(FileName, JsonSerializer.Serialize(settings, _options));
        }
        catch
        {
            // A failed settings write (locked file, read-only dir) must never crash the app.
        }
    }
}
