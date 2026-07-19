namespace Roads.App.Core;

/// <summary>
/// Every user-adjustable application setting, grouped by the Settings dialog page that
/// edits it. A mutable record: the compiler-generated VALUE equality is the dialog's
/// dirty check (staged copy vs. last-applied), and <c>with { }</c> is its clone — both
/// stay correct only while every member is a value type (revisit Clone/equality before
/// adding a collection-typed setting). Property initializers are the defaults, which
/// deliberately match the app's historical behavior; they also serve as the fallback
/// when a member is missing from a loaded settings.json.
/// Settings reach the live systems ONLY through MainForm.ApplySettings — nothing binds
/// to this object directly.
/// </summary>
public sealed record AppSettings
{
    // ── Graphics ──
    /// <summary>Borderless fullscreen (true) vs. a normal sizable window (false).</summary>
    public bool Fullscreen { get; set; }
    /// <summary>Draw the faint 100 m alignment grid over the terrain.</summary>
    public bool ShowGrid { get; set; } = true;
    /// <summary>Congestion heat-map overlay (also toggled by the H key).</summary>
    public bool HeatMapEnabled { get; set; }
    /// <summary>Performance HUD panel (also toggled by the P key).</summary>
    public bool ShowPerformanceHud { get; set; } = true;
    /// <summary>Minimap panel (also toggled by the M key).</summary>
    public bool ShowMinimap { get; set; } = true;
    /// <summary>Statistics panel (also toggled by the N key).</summary>
    public bool ShowStatistics { get; set; } = true;
    /// <summary>Keyboard-shortcut legend panel.</summary>
    public bool ShowLegend { get; set; } = true;

    // ── Simulation ──
    /// <summary>Maximum simultaneously active vehicles (lowering never despawns —
    /// spawning just stops until attrition brings the count under the cap).</summary>
    public int MaxVehicles { get; set; } = 200;
    /// <summary>Game-clock seconds that pass per real second at 1x speed.</summary>
    public double GameSecondsPerRealSecond { get; set; } = 10.0;
    /// <summary>Wall-clock seconds between automatic backup saves.</summary>
    public double AutosaveIntervalSeconds { get; set; } = 300.0;
    /// <summary>Number of autosave backups kept before the oldest is pruned.</summary>
    public int AutosaveMaxBackups { get; set; } = 5;

    // ── Driving (SteeringController tunables) ──
    /// <summary>PD steering proportional gain.</summary>
    public float Kp { get; set; } = 2.4f;
    /// <summary>PD steering derivative gain.</summary>
    public float Kd { get; set; } = 0.08f;
    /// <summary>Maximum front-wheel steering angle in radians.</summary>
    public float MaxSteer { get; set; } = 0.7f;
    /// <summary>Default target speed (m/s) on edges with no speed limit.</summary>
    public float TargetSpeed { get; set; } = 10f;
    /// <summary>Base steering lookahead distance in meters at zero speed.</summary>
    public float LookaheadBase { get; set; } = 3f;
    /// <summary>Additional lookahead distance per m/s of speed.</summary>
    public float LookaheadPerSpeed { get; set; } = 0.3f;
    /// <summary>Lateral (cross-track) error correction gain.</summary>
    public float Klat { get; set; } = 0.5f;

    // ── Audio ──
    /// <summary>Master switch for all synthesized sound.</summary>
    public bool SoundEnabled { get; set; } = true;
    /// <summary>Master output volume (0–1).</summary>
    public float MasterVolume { get; set; } = 0.7f;
    /// <summary>Ambient traffic-hum bed (density-scaled, near the camera).</summary>
    public bool AmbientHumEnabled { get; set; } = true;
    /// <summary>Per-vehicle engine tones when zoomed in close.</summary>
    public bool EngineSoundsEnabled { get; set; } = true;
    /// <summary>One-shot events: horns, brake screeches, signal ticks.</summary>
    public bool EventSoundsEnabled { get; set; } = true;

    // ── Music (generative jazz engine — see Audio/Music/Composer) ──
    /// <summary>Background music master switch (also requires <see cref="SoundEnabled"/>).</summary>
    public bool MusicEnabled { get; set; } = true;
    /// <summary>Music bus volume (0–1), independent of the SFX layers.</summary>
    public float MusicVolume { get; set; } = 0.5f;
    /// <summary>Base tempo in BPM; night/tension modulate around it (±~10%).</summary>
    public float MusicTempoBpm { get; set; } = 96f;
    /// <summary>Swing feel: 0 = straight 8ths, 1 = full triplet swing.</summary>
    public float MusicSwing { get; set; } = 0.6f;
    /// <summary>How strongly traffic density drives band energy (0 = constant mid-energy).</summary>
    public float MusicTrafficResponse { get; set; } = 1f;
    /// <summary>How strongly night mellows the band toward the ambient float (0 = never).</summary>
    public float MusicNightResponse { get; set; } = 1f;
    /// <summary>How strongly map-wide congestion raises musical tension (0 = never).</summary>
    public float MusicTensionResponse { get; set; } = 1f;

    // ── Debug ──
    /// <summary>Arc-conflict debug overlay (also toggled by the G key, which additionally
    /// forces <see cref="DebugLogging"/> ON — never off — matching its historical behavior).</summary>
    public bool ShowArcConflicts { get; set; }
    /// <summary>Steering/conflict debug logging.</summary>
    public bool DebugLogging { get; set; }
}
