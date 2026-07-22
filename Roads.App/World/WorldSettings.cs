namespace Roads.App.World;

/// <summary>
/// Per-world simulation settings, saved inside the .roads file (MapSerializer section 9,
/// format v5) — unlike <see cref="Core.AppSettings"/>, which is per-user and lives in
/// settings.json. A single instance is shared BY REFERENCE between the owner (MainForm or
/// a headless harness), <see cref="Vehicles.PopulationManager"/> (which reads it every
/// tick, so edits apply live), and MapSerializer (which overwrites the fields in place on
/// load; files older than v5 load as defaults via <see cref="ResetToDefaults"/>).
/// Edited from the World Settings panel (menu bar World button).
///
/// Current contents: automatic through-traffic spawning at entry/exit nodes. Grow this
/// class (and the serializer section + unpack_roads.py) for future world-scoped settings.
/// </summary>
public class WorldSettings
{
    /// <summary>UI/load clamp for <see cref="ThroughTrafficMultiplier"/>.</summary>
    public const float MaxMultiplier = 5f;
    /// <summary>UI/load clamp for <see cref="ThroughTrafficBaseCarsPerMin"/>.</summary>
    public const float MaxBaseCarsPerMin = 30f;

    /// <summary>Master switch for automatic through-traffic (non-resident cars entering
    /// at one entry/exit node and leaving at another). Off = no such spawns at all.</summary>
    public bool ThroughTrafficEnabled { get; set; } = true;

    /// <summary>Scales the population-driven through-traffic rate (base rate × housed
    /// residents). 1 = the historical rate, 0 = population contributes no traffic.</summary>
    public float ThroughTrafficMultiplier { get; set; } = 1f;

    /// <summary>Population-INDEPENDENT through-traffic floor, in cars per minute (at the
    /// peak time-of-day factor). Lets a world with entry/exits but few or no residents
    /// still carry pass-through traffic. 0 (default) = the historical behavior, where
    /// through-traffic exists only with a housed population.</summary>
    public float ThroughTrafficBaseCarsPerMin { get; set; }

    /// <summary>Apply the diurnal traffic curve (morning/evening peaks, overnight
    /// trickle) to through-traffic. Off = a flat rate around the clock.</summary>
    public bool RushHourVariation { get; set; } = true;

    /// <summary>Back to the defaults a fresh world starts with — used by New/empty-world
    /// resets and when loading a pre-v5 file that carries no world-settings section.</summary>
    public void ResetToDefaults()
    {
        ThroughTrafficEnabled = true;
        ThroughTrafficMultiplier = 1f;
        ThroughTrafficBaseCarsPerMin = 0f;
        RushHourVariation = true;
    }
}
