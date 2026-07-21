using NAudio.Wave;
using NAudio.Wave.SampleProviders;
using Roads.App.Audio.Music;
using Roads.App.Audio.Synth;
using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Audio;

/// <summary>
/// The procedural sound engine: an always-running NAudio graph (see the fixed topology
/// below) whose parameters are driven once per UI frame by <see cref="Update"/> reading
/// sim state — nothing here ever writes to the simulation. Layers:
/// ambient traffic hum (density/speed near the camera, quieter and duller at night),
/// a pool of per-vehicle engine tones that fade in as the camera zooms close, one-shot
/// events (horn on a deadlock-breaker release, brake screech on hard braking, soft tick
/// on visible signal changes), and the generative background music (MeltySynth +
/// bundled GM soundfont, composed on the fly by <see cref="Composer"/> and steered by
/// global sim mood — see <see cref="UpdateMusicMood"/>).
///
///   WaveOutEvent → MasterProvider (SFX×duck + music → master gain → tanh)
///     ├→ MixingSampleProvider (float 44.1 kHz stereo, ReadFully)
///     │    → HumProvider + EngineVoice×8 + OneShotVoice×6   (permanently connected)
///     └→ MusicProvider (music bus — NOT pause-ducked; the band plays through pause)
///
/// Threading: the UI thread writes plain float targets; the playback thread slews them
/// per sample (32-bit float writes are atomic; smoothing erases staleness). One-shot
/// triggers use volatile sequence counters. All callbacks (BreakerFreed, VehicleRemoving)
/// fire on the UI thread, so no locking exists anywhere.
///
/// DETERMINISM INVARIANT: nothing under Roads.App.Audio may touch <see cref="SimRandom"/>
/// (per-trigger randomness comes from a private Random) or mutate sim state — the
/// headless --simtest harness never constructs this class and must behave identically
/// whether or not audio exists in the build.
///
/// A missing/failed audio device disables the engine (<see cref="DeviceAvailable"/>
/// false, every method no-ops); the app is unaffected either way.
/// </summary>
public sealed class AudioEngine : IDisposable
{
    private const int EngineVoiceCount = 8;
    /// <summary>Engine voices actively assigned at once (2 pool slots kept as fade headroom).</summary>
    private const int MaxActiveEngineVoices = 6;
    private const int OneShotVoiceCount = 6;

    /// <summary>Camera zoom below which engine voices are silent (vehicles render as LOD
    /// dots below ~0.3); engines reach full mix by <see cref="EngineFullZoom"/>.</summary>
    private const float EngineFadeInZoom = 0.35f;
    private const float EngineFullZoom = 0.9f;

    /// <summary>One-shots are suppressed above this time scale — single diegetic events
    /// lose temporal correspondence at fast-forward and would machine-gun; the continuous
    /// hum/engine layers keep playing at any speed.</summary>
    private const int MaxOneShotTimeScale = 4;

    private readonly VehicleStore _vehicles;
    private readonly RoadGraph _graph;
    private readonly TrafficSignalSystem _signals;
    private readonly SimulationLoop _simLoop;
    private readonly Camera _camera;

    private WaveOutEvent? _waveOut;
    private MasterProvider? _master;
    private HumProvider _hum = null!;
    private EngineVoice[] _engineVoices = Array.Empty<EngineVoice>();
    private OneShotVoice[] _oneShots = Array.Empty<OneShotVoice>();
    private MusicProvider? _music;

    /// <summary>False when no output device could be opened (or it failed mid-run);
    /// the engine is inert and every public method no-ops.</summary>
    public bool DeviceAvailable { get; private set; }

    // ── Settings (written by ApplySettings) ──
    private bool _soundEnabled = true;
    private float _masterVolume = 0.7f;
    private bool _ambientEnabled = true;
    private bool _enginesEnabled = true;
    private bool _eventsEnabled = true;
    private bool _musicEnabled = true;
    private float _musicVolume = 0.5f;
    private float _musicTempo = 96f;
    private float _musicSwing = 0.6f;
    private float _musicTraffic = 1f;
    private float _musicNight = 1f;
    private float _musicTension = 1f;
    private int _maxVehicles = 200;

    // ── Music manual-mode mirrors (settings enums pre-mapped to the composer's
    //    domain here, so the provider/composer never see AppSettings types) ──
    private bool _musicManual;
    private float _mIntensity = 0.6f, _mNight, _mTension;
    private int _mForm, _mKeyPc = 10, _mLead = Theory.GmAltoSax, _mComp = Theory.GmEPiano1;
    private bool _mBrush;
    /// <summary>MusicKeyChoice → pitch class (Bb Eb F C Ab).</summary>
    private static readonly int[] KeyPcMap = { 10, 3, 5, 0, 8 };
    /// <summary>MusicLeadChoice → GM program.</summary>
    private static readonly int[] LeadProgramMap =
    {
        Theory.GmAltoSax, Theory.GmTenorSax, Theory.GmMutedTrumpet, Theory.GmHarmonica,
        Theory.GmVibraphone, Theory.GmClarinet, Theory.GmSopranoSax, Theory.GmFlute,
    };
    /// <summary>MusicCompChoice → GM program.</summary>
    private static readonly int[] CompProgramMap = { Theory.GmEPiano1, Theory.GmDrawbarOrgan, Theory.GmJazzGuitar };

    // ── Music mixer mirrors (category index order Comp/Bass/Lead/Pad/Piano/Horns/Drums;
    //    sub order lead 0–7, comp 8–10, bass 11–12, drum voices 13–19) ──
    private readonly float[] _mixVol = { 1f, 1f, 1f, 1f, 1f, 1f, 1f };
    private readonly bool[] _mixMute = new bool[7];
    private readonly bool[] _mixSolo = new bool[7];
    private readonly float[] _subVol =
        { 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f };
    private readonly bool[] _subMute = new bool[20];
    private readonly bool[] _subSolo = new bool[20];

    // ── Per-vehicle event state (index-aligned to VehicleStore, swap-and-pop fixed up) ──
    private float[] _prevBrake = Array.Empty<float>();
    private float[] _screechCooldown = Array.Empty<float>();

    // ── Per-node signal-tick state ──
    private byte[] _prevNodePhase = Array.Empty<byte>();
    private int[] _nodeLastSeenFrame = Array.Empty<int>();
    private int _frame;

    // ── Engine voice pool ──
    private readonly int[] _voiceVehicle = new int[EngineVoiceCount];
    private readonly int[] _candidateIdx = new int[EngineVoiceCount];
    private readonly float[] _candidateScore = new float[EngineVoiceCount];

    // ── One-shot budget & pending horns ──
    private float _oneShotBudget = 4f;
    private float _tickBudget = 2f;
    private readonly List<int> _pendingHorns = new(16);

    /// <summary>Audio-only randomness (horn pitch, screech length). NEVER SimRandom.</summary>
    private readonly Random _rng = new(12345);

    public AudioEngine(VehicleStore vehicles, RoadGraph graph, TrafficSignalSystem signals,
        SimulationLoop simLoop, Camera camera)
    {
        _vehicles = vehicles;
        _graph = graph;
        _signals = signals;
        _simLoop = simLoop;
        _camera = camera;
        for (int i = 0; i < EngineVoiceCount; i++) _voiceVehicle[i] = -1;

        try
        {
            var format = WaveFormat.CreateIeeeFloatWaveFormat(44100, 2);
            var mixer = new MixingSampleProvider(format) { ReadFully = true };

            _hum = new HumProvider(format);
            mixer.AddMixerInput(_hum);

            _engineVoices = new EngineVoice[EngineVoiceCount];
            for (int i = 0; i < EngineVoiceCount; i++)
            {
                _engineVoices[i] = new EngineVoice(format);
                mixer.AddMixerInput(_engineVoices[i]);
            }

            _oneShots = new OneShotVoice[OneShotVoiceCount];
            for (int i = 0; i < OneShotVoiceCount; i++)
            {
                _oneShots[i] = new OneShotVoice(format);
                mixer.AddMixerInput(_oneShots[i]);
            }

            // Generative music: MeltySynth + the bundled GM soundfont. A missing or
            // corrupt soundfont just means no music — never no app. TickCount seed:
            // a fresh opening chorus each launch (music variety is a feature; the
            // determinism invariant only concerns SimRandom).
            try
            {
                string soundFontPath = Path.Combine(AppContext.BaseDirectory, "Assets", "GeneralUser-GS.sf2");
                if (File.Exists(soundFontPath))
                    _music = new MusicProvider(soundFontPath, 44100, seed: Environment.TickCount);
            }
            catch
            {
                _music = null;
            }

            _master = new MasterProvider(mixer, _music);
            _waveOut = new WaveOutEvent { DesiredLatency = 100, NumberOfBuffers = 3 };
            _waveOut.PlaybackStopped += (_, e) =>
            {
                // Device unplugged / driver failure mid-run: self-disable, app unaffected.
                if (e.Exception != null) DeviceAvailable = false;
            };
            _waveOut.Init(_master);
            _waveOut.Play();
            DeviceAvailable = true;
        }
        catch
        {
            // No audio device (or init failure): run inert. Never crash the app for sound.
            DeviceAvailable = false;
            _waveOut?.Dispose();
            _waveOut = null;
        }
    }

    /// <summary>Pushes the audio + music settings (called from MainForm.ApplySettings —
    /// the single choke point through which every settings mutation flows).</summary>
    public void ApplySettings(AppSettings settings)
    {
        _soundEnabled = settings.SoundEnabled;
        _masterVolume = Math.Clamp(settings.MasterVolume, 0f, 1f);
        _ambientEnabled = settings.AmbientHumEnabled;
        _enginesEnabled = settings.EngineSoundsEnabled;
        _eventsEnabled = settings.EventSoundsEnabled;
        _musicEnabled = settings.MusicEnabled;
        _musicVolume = Math.Clamp(settings.MusicVolume, 0f, 1f);
        _musicTempo = settings.MusicTempoBpm;
        _musicSwing = settings.MusicSwing;
        _musicTraffic = settings.MusicTrafficResponse;
        _musicNight = settings.MusicNightResponse;
        _musicTension = settings.MusicTensionResponse;
        _maxVehicles = Math.Max(1, settings.MaxVehicles);

        _musicManual = settings.MusicManualMode;
        _mIntensity = Math.Clamp(settings.MusicManualIntensity, 0f, 1f);
        _mNight = Math.Clamp(settings.MusicManualNight, 0f, 1f);
        _mTension = Math.Clamp(settings.MusicManualTension, 0f, 1f);
        _mForm = (int)settings.MusicManualForm;
        _mKeyPc = MapOr(KeyPcMap, (int)settings.MusicManualKey, 10);
        _mLead = MapOr(LeadProgramMap, (int)settings.MusicManualLead, Theory.GmAltoSax);
        _mComp = MapOr(CompProgramMap, (int)settings.MusicManualComp, Theory.GmEPiano1);
        _mBrush = settings.MusicManualKit == MusicKitChoice.Brush;

        CopyStrip(settings.MixComp, 0, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixBass, 1, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixLead, 2, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixPad, 3, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixPiano, 4, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixHorns, 5, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixDrums, 6, _mixVol, _mixMute, _mixSolo);
        CopyStrip(settings.MixLeadAlto, 0, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadTenor, 1, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadTrumpet, 2, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadHarmonica, 3, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadVibes, 4, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadClarinet, 5, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadSoprano, 6, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixLeadFlute, 7, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixCompEPiano, 8, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixCompOrgan, 9, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixCompGuitar, 10, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixBassAcoustic, 11, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixBassFinger, 12, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumKick, 13, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumSnare, 14, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumHat, 15, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumRide, 16, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumCrash, 17, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumToms, 18, _subVol, _subMute, _subSolo);
        CopyStrip(settings.MixDrumShaker, 19, _subVol, _subMute, _subSolo);

        PushMusicSettings();
    }

    private static void CopyStrip(MixerStrip s, int i, float[] vol, bool[] mute, bool[] solo)
    {
        vol[i] = Math.Clamp(s.Volume, 0f, 1f);
        mute[i] = s.Mute;
        solo[i] = s.Solo;
    }

    /// <summary>Table lookup with an index guard for out-of-range enum values from a
    /// hand-edited settings.json.</summary>
    private static int MapOr(int[] table, int index, int fallback)
        => (uint)index < (uint)table.Length ? table[index] : fallback;

    /// <summary>Pushes every settings-derived music target — gain, tempo, swing, the
    /// manual-mode pin, and both mixer levels — so a change takes effect (at the next
    /// bar) even while the sim is paused or on the title screen. In manual mode the
    /// mood targets are ALSO written here from the manual sliders (UpdateMusicMood
    /// skips its sim mappings), pinning ambience to 0 for the full band.</summary>
    private void PushMusicSettings()
    {
        if (_music == null) return;
        _music.TargetGain = _soundEnabled && _musicEnabled ? _musicVolume : 0f;
        _music.TempoSetting = _musicTempo;
        _music.SwingSetting = _musicSwing;

        _music.ManualMode = _musicManual;
        _music.ManualForm = _mForm;
        _music.ManualKeyPc = _mKeyPc;
        _music.ManualLeadProgram = _mLead;
        _music.ManualCompProgram = _mComp;
        _music.ManualBrushKit = _mBrush;
        for (int i = 0; i < 7; i++)
        {
            _music.MixVolume[i] = _mixVol[i];
            _music.MixMute[i] = _mixMute[i] ? 1 : 0;
            _music.MixSolo[i] = _mixSolo[i] ? 1 : 0;
        }
        for (int i = 0; i < 20; i++)
        {
            _music.SubVolume[i] = _subVol[i];
            _music.SubMute[i] = _subMute[i] ? 1 : 0;
            _music.SubSolo[i] = _subSolo[i] ? 1 : 0;
        }

        if (_musicManual)
        {
            _music.TargetIntensity = _mIntensity;
            _music.TargetNight = _mNight;
            _music.TargetTension = _mTension;
            _music.TargetAmbience = 0f;
        }
    }

    /// <summary>
    /// Per-frame parameter drive (UI thread, after the sim tick): computes the hum bed
    /// from vehicle density near the camera, assigns/updates engine voices for the
    /// closest on-screen vehicles, and detects one-shot events. Single merged O(n) pass
    /// over the vehicle store; zero allocation.
    /// </summary>
    public void Update(int viewWidth, int viewHeight)
    {
        if (!DeviceAvailable || _master == null || viewWidth <= 0 || viewHeight <= 0) return;

        _frame++;
        EnsureArrays();

        int timeScale = _simLoop.TimeScale;
        _master.TargetMaster = _soundEnabled ? _masterVolume : 0f;
        _master.TargetDuck = timeScale == 0 ? 0f : 1f;

        var rect = _camera.GetVisibleWorldRect(viewWidth, viewHeight);
        float camX = rect.MidX, camY = rect.MidY;
        float zoom = _camera.Zoom;
        float dark = _simLoop.Clock.Darkness;
        float engineMix = _enginesEnabled ? DspUtil.SmoothStep(EngineFadeInZoom, EngineFullZoom, zoom) : 0f;

        // One-shot budgets refill in wall-clock frame time; cooldowns tick in SIM time so
        // pausing freezes them (LastTickSubsteps is 0 while paused).
        float wallDt = 1f / 60f;
        float simDt = _simLoop.LastTickSubsteps * SimulationLoop.SimDt;
        _oneShotBudget = MathF.Min(_oneShotBudget + 4f * wallDt, 4f);
        _tickBudget = MathF.Min(_tickBudget + 2f * wallDt, 2f);
        bool oneShotsAllowed = _eventsEnabled && timeScale >= 1 && timeScale <= MaxOneShotTimeScale;

        // Inflated rects: engines slightly beyond the view (smooth entry), events +10%.
        float inflW = rect.Width * 0.2f, inflH = rect.Height * 0.2f;
        var engineRect = SKRectInflate(rect, inflW, inflH);
        var eventRect = SKRectInflate(rect, rect.Width * 0.1f, rect.Height * 0.1f);

        // Density radius: half the view diagonal, clamped to a sane world range.
        float halfDiag = 0.5f * MathF.Sqrt(rect.Width * rect.Width + rect.Height * rect.Height);
        float densityRadius = Math.Clamp(halfDiag, 100f, 800f);

        // ── Single merged O(n) vehicle pass: hum density + speed, screech edges, engine candidates ──
        float density = 0f, speedSum = 0f;
        int speedCount = 0;
        int candidates = 0;
        int drivingCount = 0, congestedCount = 0; // global (map-wide) — feeds the music mood
        for (int i = 0; i < _vehicles.Count; i++)
        {
            float brake = _vehicles.Brake[i];
            float prevBrake = _prevBrake[i];
            _prevBrake[i] = brake; // unconditionally, for EVERY vehicle (stale state on pan = false triggers)
            if (_screechCooldown[i] > 0f) _screechCooldown[i] -= simDt;

            if (_vehicles.State[i] != VehicleState.Driving) continue;

            float x = _vehicles.PosX[i], y = _vehicles.PosY[i];
            float speed = _vehicles.Speed[i];
            drivingCount++;
            if (speed < 2f) congestedCount++; // StatisticsPanel.CongestionThresholdMs
            float dx = x - camX, dy = y - camY;
            float dist = MathF.Sqrt(dx * dx + dy * dy);

            // Hum density: smoothstep falloff from 60% to 100% of the radius, speed-weighted.
            if (dist < densityRadius)
            {
                float speedNorm = MathF.Min(speed / 20f, 1f);
                float falloff = 1f - DspUtil.SmoothStep(densityRadius * 0.6f, densityRadius, dist);
                density += falloff * (0.3f + 0.7f * speedNorm);
                speedSum += speed;
                speedCount++;
            }

            // Brake screech: rising edge of hard braking at speed, per-vehicle cooldown.
            if (oneShotsAllowed && brake >= 0.95f && prevBrake < 0.95f && speed > 8f
                && _screechCooldown[i] <= 0f && eventRect.Contains(x, y) && _oneShotBudget >= 1f)
            {
                if (TriggerOneShot(OneShotKind.Screech, x, y, camX, viewWidth, zoom, gainDb: -14f))
                {
                    _oneShotBudget -= 1f;
                    _screechCooldown[i] = 4f;
                }
            }

            // Engine candidates: top-K by proximity score inside the inflated view.
            if (engineMix > 0f && engineRect.Contains(x, y))
            {
                float typeWeight = (VehicleType)_vehicles.PreferredVehicle[i] switch
                {
                    VehicleType.Truck or VehicleType.Bus => 1.4f,
                    VehicleType.Motorcycle => 1.2f,
                    _ => 1f,
                };
                float score = typeWeight / (dist * dist + 3600f);
                InsertCandidate(ref candidates, i, score);
            }
        }

        // ── Hum targets ──
        float avgSpeedNorm = speedCount > 0 ? MathF.Min(speedSum / speedCount / 20f, 1f) : 0f;
        float humMix = (_ambientEnabled ? 1f : 0f) * (1f - 0.5f * engineMix);
        _hum.TargetGain = 0.14f * (1f - MathF.Exp(-density / 6f)) * (1f - 0.4f * dark) * humMix;
        _hum.TargetCutoffHz = (350f + 450f * avgSpeedNorm) * (1f - 0.35f * dark);

        UpdateEngineVoices(candidates, engineMix, camX, camY, viewWidth, zoom);
        DrainPendingHorns(oneShotsAllowed, eventRect, camX, viewWidth, zoom);
        UpdateSignalTicks(oneShotsAllowed, rect, zoom, camX, viewWidth);
        UpdateMusicMood(drivingCount, congestedCount, dark, zoom, wallDt);
    }

    // ── Music resolution-trigger state (jam-cleared detection) ──
    private float _musicTensionHighSecs;
    private float _musicResolutionCooldown;

    /// <summary>
    /// Maps global sim state to the music mood: traffic density (vehicles vs. half the
    /// population cap) drives arrangement energy, time-of-day darkness pulls the band
    /// toward the night float, the map-wide congestion share (vehicles below the
    /// StatisticsPanel stall threshold) raises harmonic tension, the in-game hour and
    /// day steer the setlist repertoire, and camera zoom sets the far-view ambience
    /// (pads up, drums down). A tension FALLING edge — congestion that held above 0.5
    /// for a few seconds then cleared — fires the one-shot resolution tag, with a
    /// cooldown so oscillating jams don't machine-gun cadences. The Music settings
    /// sliders scale each response; at zero a mapping is pinned to its neutral value.
    /// The composer consumes these at bar boundaries, so changes always land musically.
    /// MANUAL mode bypasses everything here: the mood targets come from the manual
    /// sliders (written by PushMusicSettings), hour/day freeze, ambience pins to 0, and
    /// the resolution trigger is suppressed (its edge timer resets so no stale cadence
    /// fires on return to auto).
    /// </summary>
    private void UpdateMusicMood(int driving, int congested, float dark, float zoom, float wallDt)
    {
        if (_music == null) return;
        if (!_musicManual)
        {
            float trafficNorm = Math.Clamp(driving / MathF.Max(1f, 0.5f * _maxVehicles), 0f, 1f);
            // Response slider lerps between a fixed mid-energy band (0) and the full mapping (1).
            float intensity = Math.Clamp(0.5f + _musicTraffic * (0.15f + 1.05f * trafficNorm - 0.5f), 0f, 1f);
            float congestionShare = driving > 0 ? (float)congested / driving : 0f;
            // Below ~15% slow vehicles is normal signal queuing, not a jam; scale by a small
            // fleet factor so five stopped cars on an empty map don't read as gridlock.
            float tension = Math.Clamp((congestionShare - 0.15f) / 0.45f, 0f, 1f)
                * MathF.Min(1f, driving / 25f) * _musicTension;

            _music.TargetIntensity = intensity;
            _music.TargetNight = dark * _musicNight;
            _music.TargetTension = tension;
            _music.TargetHour = (float)_simLoop.Clock.TimeOfDay;
            _music.TargetDayNumber = _simLoop.Clock.DayNumber;
            _music.TargetAmbience = 1f - DspUtil.SmoothStep(0.25f, 0.9f, zoom);

            _musicResolutionCooldown = MathF.Max(0f, _musicResolutionCooldown - wallDt);
            if (tension > 0.5f)
            {
                _musicTensionHighSecs += wallDt;
            }
            else if (tension < 0.2f)
            {
                if (_musicTensionHighSecs >= 3f && _musicResolutionCooldown <= 0f)
                {
                    _music.ResolutionSeq++;
                    _musicResolutionCooldown = 45f;
                }
                _musicTensionHighSecs = 0f;
            }
        }
        else
        {
            _musicTensionHighSecs = 0f;
        }

        PushMusicSettings();
    }

    // ═══════════════════════ Engine voice pool ═══════════════════════

    /// <summary>
    /// Assigns the top-scoring candidates to the voice pool with hysteresis: held vehicles
    /// keep their voice unless an unassigned candidate beats them by 1.3× (or they left
    /// the area/stopped driving), and freed voices are only re-tuned once fully quiet —
    /// pitch/timbre never change audibly, so passes can't click or flutter.
    /// </summary>
    private void UpdateEngineVoices(int candidates, float engineMix,
        float camX, float camY, int viewWidth, float zoom)
    {
        // Release phase: drop voices whose vehicle is gone, out of range, or decisively outscored.
        for (int v = 0; v < EngineVoiceCount; v++)
        {
            int veh = _voiceVehicle[v];
            if (veh < 0) continue;

            bool valid = veh < _vehicles.Count && _vehicles.State[veh] == VehicleState.Driving;
            float heldScore = 0f;
            bool inCandidates = false;
            if (valid)
            {
                for (int c = 0; c < candidates; c++)
                    if (_candidateIdx[c] == veh) { inCandidates = true; break; }
                // Same type-weighted formula as the candidate pass, so the 1.3x hysteresis
                // compares like with like.
                float dx = _vehicles.PosX[veh] - camX, dy = _vehicles.PosY[veh] - camY;
                float typeWeight = (VehicleType)_vehicles.PreferredVehicle[veh] switch
                {
                    VehicleType.Truck or VehicleType.Bus => 1.4f,
                    VehicleType.Motorcycle => 1.2f,
                    _ => 1f,
                };
                heldScore = typeWeight / (dx * dx + dy * dy + 3600f);
            }

            if (!valid || engineMix <= 0f)
            {
                ReleaseVoice(v);
                continue;
            }
            if (!inCandidates)
            {
                // Outside the top-K: release only if some unassigned candidate clearly wins.
                bool decisivelyBeaten = false;
                for (int c = 0; c < candidates; c++)
                {
                    if (IsAssigned(_candidateIdx[c])) continue;
                    if (_candidateScore[c] > heldScore * 1.3f) { decisivelyBeaten = true; break; }
                }
                if (decisivelyBeaten) { ReleaseVoice(v); continue; }
            }

            RefreshVoiceTargets(v, veh, engineMix, camX, viewWidth, zoom);
        }

        if (engineMix <= 0f) return;

        // Assign phase: give unassigned candidates a fully-quiet voice (skip otherwise —
        // the anti-click rule: timbre/pitch only rewritten while silent).
        int active = CountActiveVoices();
        for (int c = 0; c < candidates && active < MaxActiveEngineVoices; c++)
        {
            int veh = _candidateIdx[c];
            if (IsAssigned(veh)) continue;

            int free = -1;
            for (int v = 0; v < EngineVoiceCount; v++)
                if (_voiceVehicle[v] < 0 && _engineVoices[v].IsQuiet) { free = v; break; }
            if (free < 0) break;

            var voice = _engineVoices[free];
            _voiceVehicle[free] = veh;
            voice.TimbreType = _vehicles.PreferredVehicle[veh];
            voice.Detune = 1f + ((float)_rng.NextDouble() - 0.5f) * 0.06f;
            // Anti-click ordering: publish the new vehicle's pitch/cutoff/pan targets
            // while the gain TARGET is still 0, arm the retune (volatile — the writes
            // above are visible to whoever observes it), and only then raise the gain.
            // Whichever buffer boundary the callback lands on, it either stays in the
            // quiet fast path or hard-resets to the NEW state while still silent.
            // (Raising the gain before Retune let one ~33 ms buffer render with the
            // previous vehicle's smoother state, then reset pitch mid-fade — a click.)
            RefreshVoiceTargets(free, veh, engineMix, camX, viewWidth, zoom, holdSilent: true);
            voice.Retune = true;
            RefreshVoiceTargets(free, veh, engineMix, camX, viewWidth, zoom);
            active++;
        }
    }

    /// <summary>Streams the per-frame pitch/gain/pan/cutoff targets for an assigned voice.
    /// <paramref name="holdSilent"/> publishes pitch/cutoff/pan but writes the gain target
    /// as 0 — the assignment path's anti-click ordering (targets first, then Retune, then
    /// a second call raises the gain).</summary>
    private void RefreshVoiceTargets(int voiceIdx, int veh, float engineMix,
        float camX, int viewWidth, float zoom, bool holdSilent = false)
    {
        var voice = _engineVoices[voiceIdx];
        float speed = _vehicles.Speed[veh];
        float speedNorm = MathF.Min(speed / 20f, 1f);
        float throttle = _vehicles.SmoothedThrottle[veh];

        float basePitch = (VehicleType)_vehicles.PreferredVehicle[veh] switch
        {
            VehicleType.Bus => 45f,
            VehicleType.Truck => 55f,
            VehicleType.SUV => 75f,
            VehicleType.Motorcycle => 130f,
            _ => 85f,
        };
        float pitch = basePitch * (0.8f + 1.6f * speedNorm);
        float cutoffScale = (VehicleType)_vehicles.PreferredVehicle[veh] == VehicleType.Motorcycle ? 1.5f : 1f;

        float dist = Distance(veh, camX);
        float distAtten = 1f / (1f + (dist / 60f) * (dist / 60f));

        voice.TargetPitchHz = pitch;
        voice.TargetCutoffHz = MathF.Min(pitch * (4f + 8f * throttle) * cutoffScale, 6000f);
        voice.TargetGain = holdSilent
            ? 0f
            : 0.10f * (0.25f + 0.75f * (0.4f * speedNorm + 0.6f * throttle)) * distAtten * engineMix;
        voice.TargetPan = ScreenPan(_vehicles.PosX[veh], camX, viewWidth, zoom);
    }

    private float _camYCache;

    private float Distance(int veh, float camX)
    {
        float dx = _vehicles.PosX[veh] - camX;
        float dy = _vehicles.PosY[veh] - _camYCache;
        return MathF.Sqrt(dx * dx + dy * dy);
    }

    private void ReleaseVoice(int v)
    {
        _engineVoices[v].TargetGain = 0f;
        _voiceVehicle[v] = -1;
    }

    private bool IsAssigned(int veh)
    {
        for (int v = 0; v < EngineVoiceCount; v++)
            if (_voiceVehicle[v] == veh) return true;
        return false;
    }

    private int CountActiveVoices()
    {
        int n = 0;
        for (int v = 0; v < EngineVoiceCount; v++)
            if (_voiceVehicle[v] >= 0) n++;
        return n;
    }

    /// <summary>Insertion into the fixed top-K candidate scratch (descending score).</summary>
    private void InsertCandidate(ref int count, int veh, float score)
    {
        int pos = count;
        while (pos > 0 && _candidateScore[pos - 1] < score) pos--;
        if (pos >= EngineVoiceCount) return;
        int end = Math.Min(count, EngineVoiceCount - 1);
        for (int j = end; j > pos; j--)
        {
            _candidateIdx[j] = _candidateIdx[j - 1];
            _candidateScore[j] = _candidateScore[j - 1];
        }
        _candidateIdx[pos] = veh;
        _candidateScore[pos] = score;
        if (count < EngineVoiceCount) count++;
    }

    // ═══════════════════════ One-shot events ═══════════════════════

    /// <summary>UI-thread callback from <see cref="SteeringController.BreakerFreed"/> —
    /// fires during the sim tick; drained (and position-validated) in Update.</summary>
    public void OnBreakerFreed(int vehicleIndex)
    {
        if (_pendingHorns.Count < 16) _pendingHorns.Add(vehicleIndex);
    }

    private void DrainPendingHorns(bool allowed, SkiaSharp.SKRect eventRect,
        float camX, int viewWidth, float zoom)
    {
        for (int i = 0; i < _pendingHorns.Count; i++)
        {
            int veh = _pendingHorns[i];
            if (!allowed || _oneShotBudget < 1f) continue;
            if (veh < 0 || veh >= _vehicles.Count) continue;
            float x = _vehicles.PosX[veh], y = _vehicles.PosY[veh];
            if (!eventRect.Contains(x, y)) continue;
            if (TriggerOneShot(OneShotKind.Horn, x, y, camX, viewWidth, zoom, gainDb: -12f))
                _oneShotBudget -= 1f;
        }
        _pendingHorns.Clear();
    }

    private void UpdateSignalTicks(bool allowed, SkiaSharp.SKRect rect, float zoom,
        float camX, int viewWidth)
    {
        // Ticks only when signal heads are legible on screen.
        if (zoom < 0.3f)
            return;

        for (int n = 0; n < _graph.Nodes.Count; n++)
        {
            if (!_signals.IsTrafficLight(n)) continue;
            var pos = _graph.Nodes[n].Position;
            if (!rect.Contains(pos.X, pos.Y)) continue; // NaN (defunct) fails naturally

            byte phase = _signals.GetNodePhase(n);
            bool seenLastFrame = _nodeLastSeenFrame[n] == _frame - 1;
            byte prev = _prevNodePhase[n];
            _prevNodePhase[n] = phase;
            _nodeLastSeenFrame[n] = _frame;

            if (!seenLastFrame || phase == prev) continue; // stale cache after a pan = no tick
            // Green (0,3) and yellow (1,4) onsets only; all-red onsets (2,5) skipped.
            if (phase == 2 || phase == 5) continue;
            if (!allowed || _tickBudget < 1f) continue;

            if (TriggerOneShot(OneShotKind.Tick, pos.X, pos.Y, camX, viewWidth, zoom, gainDb: -30f, fixedGain: true))
                _tickBudget -= 1f;
        }
    }

    /// <summary>Fires a one-shot on a free voice (false when the pool is saturated).</summary>
    private bool TriggerOneShot(OneShotKind kind, float x, float y,
        float camX, int viewWidth, float zoom, float gainDb, bool fixedGain = false)
    {
        OneShotVoice? free = null;
        for (int i = 0; i < _oneShots.Length; i++)
            if (_oneShots[i].IsFree) { free = _oneShots[i]; break; }
        if (free == null) return false;

        float dx = x - camX, dy = y - _camYCache;
        float dist = MathF.Sqrt(dx * dx + dy * dy);
        float distAtten = fixedGain ? 1f : 1f / (1f + (dist / 60f) * (dist / 60f));

        free.Kind = kind;
        free.Gain = MathF.Pow(10f, gainDb / 20f) * distAtten;
        free.Pan = ScreenPan(x, camX, viewWidth, zoom);
        free.Param1 = (float)_rng.NextDouble();
        free.Param2 = (float)_rng.NextDouble();
        free.IsFree = false;
        free.TriggerSeq++;
        return true;
    }

    /// <summary>Stereo pan from a world X position's on-screen location (±0.8 max).</summary>
    private float ScreenPan(float worldX, float camX, int viewWidth, float zoom)
    {
        float screenOffset = (worldX - camX) * zoom; // px from screen center
        return Math.Clamp(2f * screenOffset / viewWidth, -1f, 1f) * 0.8f;
    }

    // ═══════════════════════ Store event fixups & lifecycle ═══════════════════════

    /// <summary>Swap-and-pop fixup (same index-aligned pattern as MainForm._stuckTicks):
    /// per-vehicle audio state follows the moved vehicle, and any voice bound to the
    /// removed/moved index retargets — index-reuse tolerance would false-trigger a
    /// screech (or transplant an engine tone) the instant a slot is recycled.</summary>
    public void OnVehicleRemoving(int removed, int swappedFrom)
    {
        for (int v = 0; v < EngineVoiceCount; v++)
        {
            if (_voiceVehicle[v] == removed) ReleaseVoice(v);
            else if (_voiceVehicle[v] == swappedFrom) _voiceVehicle[v] = removed;
        }
        if (swappedFrom >= 0 && removed < _prevBrake.Length && swappedFrom < _prevBrake.Length)
        {
            _prevBrake[removed] = _prevBrake[swappedFrom];
            _screechCooldown[removed] = _screechCooldown[swappedFrom];
        }
    }

    /// <summary>Bulk clear (new map / load): all voices release, per-vehicle state zeroes.</summary>
    public void OnVehiclesCleared()
    {
        for (int v = 0; v < EngineVoiceCount; v++) ReleaseVoice(v);
        Array.Clear(_prevBrake, 0, _prevBrake.Length);
        Array.Clear(_screechCooldown, 0, _screechCooldown.Length);
        _pendingHorns.Clear();
    }

    private void EnsureArrays()
    {
        if (_prevBrake.Length < _vehicles.Count)
        {
            int size = _vehicles.Count + 64;
            Array.Resize(ref _prevBrake, size);
            Array.Resize(ref _screechCooldown, size);
        }
        if (_prevNodePhase.Length < _graph.Nodes.Count)
        {
            int size = _graph.Nodes.Count + 64;
            Array.Resize(ref _prevNodePhase, size);
            Array.Resize(ref _nodeLastSeenFrame, size);
        }
        // Cache the camera world-center Y for the distance helpers this frame.
        _camYCache = -_camera.CenterY / _camera.Zoom;
    }

    private static SkiaSharp.SKRect SKRectInflate(SkiaSharp.SKRect r, float dx, float dy)
        => new(r.Left - dx, r.Top - dy, r.Right + dx, r.Bottom + dy);

    public void Dispose()
    {
        SteeringController.BreakerFreed -= OnBreakerFreed;
        _waveOut?.Stop();
        _waveOut?.Dispose();
        _waveOut = null;
        DeviceAvailable = false;
    }
}
