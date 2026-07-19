namespace Roads.App.Core;

/// <summary>Chart family for the manual music mode. Mirrors the composer's forms, with
/// the 3/4 waltz nocturne split out (auto rolls that chart variant randomly — manual
/// must be able to audition it explicitly).</summary>
public enum MusicFormChoice { Blues, MinorBlues, Aaba, Bossa, Vamp, Nocturne, NocturneWaltz }

/// <summary>Key center for the manual music mode (the composer's five canonical keys).</summary>
public enum MusicKeyChoice { Bb, Eb, F, C, Ab }

/// <summary>Lead instrument for the manual music mode (the composer's full lead pool).</summary>
public enum MusicLeadChoice { AltoSax, TenorSax, MutedTrumpet, Harmonica, Vibraphone, Clarinet, SopranoSax, Flute }

/// <summary>Comping patch for the manual music mode.</summary>
public enum MusicCompChoice { EPiano, Organ, JazzGuitar }

/// <summary>Drum kit for the manual music mode (Brush falls back to Standard when the
/// soundfont lacks the GS brush patch).</summary>
public enum MusicKitChoice { Standard, Brush }

/// <summary>
/// One music-mixer strip: volume scale plus mute/solo. A record STRUCT (value type), so
/// <see cref="AppSettings"/>' synthesized value equality and shallow with-clone stay
/// correct with these as members. JSON caveat: System.Text.Json builds structs from
/// <c>default</c>, so a hand-edited settings.json strip that omits Volume reads as 0 —
/// app-written files always contain every member, and a wholly-absent strip property
/// falls back to the initializer default (unity, unmuted).
/// </summary>
public record struct MixerStrip
{
    /// <summary>Strip gain 0–1 (1 = the composer's baked-in level).</summary>
    public float Volume { get; set; } = 1f;
    public bool Mute { get; set; }
    public bool Solo { get; set; }
    public MixerStrip() { }
}

/// <summary>
/// Every user-adjustable application setting, grouped by the Settings dialog page that
/// edits it. A mutable record: the compiler-generated VALUE equality is the dialog's
/// dirty check (staged copy vs. last-applied), and <c>with { }</c> is its clone — both
/// stay correct only while every member is a value type (revisit Clone/equality before
/// adding a collection-typed setting; enums and the <see cref="MixerStrip"/> record
/// struct are value types and safe). Property initializers are the defaults, which
/// deliberately match the app's historical behavior; they also serve as the fallback
/// when a member is missing from a loaded settings.json.
/// Settings reach the live systems ONLY through MainForm.ApplySettings — nothing binds
/// to this object directly. Exception to the apply flow: the Settings dialog's Music
/// page LIVE-PREVIEWS its staged record through the audio engine on every edit (see
/// SettingsDialog); the applied record itself still moves only via ApplySettings.
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

    // ── Music: manual mode (pins mood + tune so every combination can be auditioned
    //    without a live traffic scenario; inert while false) ──
    /// <summary>False = auto-compose from traffic/night/congestion (historical behavior);
    /// true = the manual fields below pin mood, form, key, and instrumentation.</summary>
    public bool MusicManualMode { get; set; }
    /// <summary>Manual band energy (replaces the traffic mapping), 0–1.</summary>
    public float MusicManualIntensity { get; set; } = 0.6f;
    /// <summary>Manual night mood (replaces darkness), 0–1.</summary>
    public float MusicManualNight { get; set; }
    /// <summary>Manual tension (replaces congestion), 0–1.</summary>
    public float MusicManualTension { get; set; }
    /// <summary>Manual chart family.</summary>
    public MusicFormChoice MusicManualForm { get; set; } = MusicFormChoice.Blues;
    /// <summary>Manual key center.</summary>
    public MusicKeyChoice MusicManualKey { get; set; } = MusicKeyChoice.Bb;
    /// <summary>Manual lead instrument.</summary>
    public MusicLeadChoice MusicManualLead { get; set; } = MusicLeadChoice.AltoSax;
    /// <summary>Manual comping patch.</summary>
    public MusicCompChoice MusicManualComp { get; set; } = MusicCompChoice.EPiano;
    /// <summary>Manual drum kit.</summary>
    public MusicKitChoice MusicManualKit { get; set; } = MusicKitChoice.Standard;

    // ── Music: mixer (both auto and manual). Category strips act per MIDI channel via
    //    CC7; sub-strips act per instrument at note emission (drums by note number,
    //    melodic categories by the channel's active program). Solo scope: category solos
    //    against categories, sub against subs of the same category. ──
    public MixerStrip MixComp { get; set; } = new();
    public MixerStrip MixBass { get; set; } = new();
    public MixerStrip MixLead { get; set; } = new();
    public MixerStrip MixPad { get; set; } = new();
    public MixerStrip MixPiano { get; set; } = new();
    public MixerStrip MixHorns { get; set; } = new();
    public MixerStrip MixDrums { get; set; } = new();
    // Lead sub-strips (keyed by the lead channel's active program).
    public MixerStrip MixLeadAlto { get; set; } = new();
    public MixerStrip MixLeadTenor { get; set; } = new();
    public MixerStrip MixLeadTrumpet { get; set; } = new();
    public MixerStrip MixLeadHarmonica { get; set; } = new();
    public MixerStrip MixLeadVibes { get; set; } = new();
    public MixerStrip MixLeadClarinet { get; set; } = new();
    public MixerStrip MixLeadSoprano { get; set; } = new();
    public MixerStrip MixLeadFlute { get; set; } = new();
    // Comp sub-strips.
    public MixerStrip MixCompEPiano { get; set; } = new();
    public MixerStrip MixCompOrgan { get; set; } = new();
    public MixerStrip MixCompGuitar { get; set; } = new();
    // Bass sub-strips.
    public MixerStrip MixBassAcoustic { get; set; } = new();
    public MixerStrip MixBassFinger { get; set; } = new();
    // Drum voice sub-strips (keyed by percussion note number).
    public MixerStrip MixDrumKick { get; set; } = new();
    public MixerStrip MixDrumSnare { get; set; } = new();
    public MixerStrip MixDrumHat { get; set; } = new();
    public MixerStrip MixDrumRide { get; set; } = new();
    public MixerStrip MixDrumCrash { get; set; } = new();
    public MixerStrip MixDrumToms { get; set; } = new();
    public MixerStrip MixDrumShaker { get; set; } = new();

    // ── Debug ──
    /// <summary>Arc-conflict debug overlay (also toggled by the G key, which additionally
    /// forces <see cref="DebugLogging"/> ON — never off — matching its historical behavior).</summary>
    public bool ShowArcConflicts { get; set; }
    /// <summary>Steering/conflict debug logging.</summary>
    public bool DebugLogging { get; set; }
}
