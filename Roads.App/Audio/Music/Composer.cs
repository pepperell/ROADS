using System.Diagnostics;
using Roads.App.Audio.Synth;

namespace Roads.App.Audio.Music;

/// <summary>One scheduled MIDI event on the music timeline (absolute sample time).
/// Commands: 0x90 note-on, 0x80 note-off, 0xB0 control change, 0xC0 program change.</summary>
public struct MidiEvent
{
    public long Time;
    public byte Channel;
    public byte Command;
    public byte Data1;
    public byte Data2;

    /// <summary>Sort rank for same-sample ties: program/CC changes must precede note-offs,
    /// which must precede note-ons, or a patch change lands after the note it was meant
    /// for and a repeated note gets chopped by its predecessor's note-off.</summary>
    public readonly int TieRank => Command switch { 0x90 => 2, 0x80 => 1, _ => 0 };
}

/// <summary>
/// One bar's worth of mood, assembled ON THE PLAYBACK THREAD by MusicProvider.ComposeBar
/// from its individually-atomic target fields (plus the resolution sequence-counter
/// comparison) — the struct itself is never shared across threads, so no torn reads.
/// </summary>
public readonly record struct MoodInputs(
    float Intensity, float Night, float Tension, float TempoBpm, float Swing,
    float Hour, float Ambience, int DayNumber, bool ResolutionTag);

/// <summary>
/// The symbolic music generator: a bar-at-a-time jazz composer in the SimCity 2000 /
/// Transport Tycoon lineage. Runs entirely on the NAudio playback thread —
/// <see cref="MusicProvider"/> calls <see cref="SetMood"/> + <see cref="GenerateBar"/>
/// whenever the playhead reaches the end of the composed material, so no locking exists
/// anywhere. Randomness comes from a private <see cref="Random"/>; nothing here may
/// ever touch SimRandom or sim state.
///
/// Structure, outermost first:
///
/// SETLIST — the band plays discrete "tunes" (~4–6 min): a main form + key + tempo +
/// lead/comping palette + drum kit + reverb room, chosen by <see cref="PickTune"/>
/// (the single place hour-of-day, night, and day-number weighting happens). Each tune
/// runs Playing → Ending (2-bar ii–V–I tag with ritardando) → Break (silent bar or
/// two) → CountIn (pedal-hat clicks; nocturnes skip it and fade in on pads) → next
/// tune. Day rollover, nightfall, and dawn arm an early rotation.
///
/// FORMS — 12-bar jazz-blues, 12-bar minor blues, 32-bar AABA, 16-bar bossa
/// (straight 8ths), 8-bar funk vamp, and two nocturne floats (one in 3/4). The form
/// persists for the whole tune; blues-family tunes drop into the vamp as an interlude
/// after a few choruses or under congestion tension.
///
/// CHORUSES — chorus 0 is the head (sparse riff melody, two-feel bass on swing tunes);
/// middle choruses are solos (dense lead, simplified comping, trading fours with the
/// drums, brass guide-tone pads at high intensity); the final chorus may take a
/// half-step gear-change lift.
///
/// MOOD (slewed half-way per bar, consumed at bar boundaries so changes land
/// musically): intensity gates layers and density; night mellows and steers tune
/// selection toward nocturnes; tension pushes vamp interludes and pedal bass; ambience
/// (camera zoom) fades drums down and pads up; a resolution trigger (jam cleared)
/// inserts the tag mid-tune and restarts the chorus.
/// </summary>
public sealed class Composer
{
    // ── MIDI channel plan ──
    public const int ChComp = 0;   // comping (EP / jazz guitar / organ, per tune)
    public const int ChBass = 1;   // acoustic/finger bass
    public const int ChLead = 2;   // rotating lead within the tune's palette
    public const int ChPad = 3;    // warm pad (night float / far-zoom ambience)
    public const int ChPiano = 4;  // boogie piano layer (blues solos, high intensity)
    public const int ChHorns = 5;  // brass-section guide tones (solo choruses)
    public const int ChDrums = 9;  // GM percussion

    private enum TunePhase { Playing, Ending, Break, CountIn }
    private enum MusicForm { Blues, MinorBlues, Aaba, Bossa, Vamp, Nocturne }
    private enum ChorusRole { Head, Solo }
    private enum DrumStyle { Swing, Funk, Bossa, NightWaltz }

    // ═══════════════════════ Charts ═══════════════════════
    // Home keys noted per chart; everything is transposed to the tune's key center via
    // the single choke point in GenerateBar. Pitch classes: C=0 … B=11.

    private sealed class ChartDef
    {
        public string Name = "";
        public BarChords[] Bars = Array.Empty<BarChords>();
        /// <summary>The chart's home key center (pc) — transposition is relative to this.</summary>
        public int KeyRootPc;
        /// <summary>Multiplier on the user swing setting: 1 = full swing, 0 = straight.</summary>
        public float SwingFactor = 1f;
        public int BeatsPerBar = 4;
    }

    private static Chord C(int root, ChordQuality q) => new(root, q);

    private static readonly ChartDef BluesChart = new()
    {
        Name = "blues", KeyRootPc = 10,
        Bars = new BarChords[]
        {
            new(C(10, ChordQuality.Dom13)),
            new(C(3, ChordQuality.Dom13)),
            new(C(10, ChordQuality.Dom13)),
            new(C(5, ChordQuality.Min9), C(10, ChordQuality.Dom13)),
            new(C(3, ChordQuality.Dom13)),
            new(C(4, ChordQuality.Dim7)),
            new(C(10, ChordQuality.Dom13)),
            new(C(2, ChordQuality.Min7b5), C(7, ChordQuality.Dom7Alt)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Dom13)),
            new(C(10, ChordQuality.Dom13), C(7, ChordQuality.Dom7Alt)),
            new(C(0, ChordQuality.Min9), C(5, ChordQuality.Dom13)),
        },
    };

    private static readonly ChartDef MinorBluesChart = new()
    {
        Name = "minor-blues", KeyRootPc = 0,
        Bars = new BarChords[]
        {
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Min9)),
            new(C(0, ChordQuality.Min9)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Min9)),
            new(C(5, ChordQuality.Min9)),
            new(C(0, ChordQuality.Min9)),
            new(C(0, ChordQuality.Min9)),
            new(C(8, ChordQuality.Dom13)),
            new(C(7, ChordQuality.Dom7Alt)),
            new(C(0, ChordQuality.Min9)),
            new(C(7, ChordQuality.Dom7Alt)),
        },
    };

    private static readonly ChartDef AabaChart = new()
    {
        Name = "aaba", KeyRootPc = 3,
        Bars = BuildAaba(),
    };

    private static BarChords[] BuildAaba()
    {
        var a = new BarChords[]
        {
            new(C(3, ChordQuality.Maj9)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Min9)),
            new(C(10, ChordQuality.Dom13)),
            new(C(3, ChordQuality.Maj9)),
            new(C(8, ChordQuality.Dom13)),
            new(C(3, ChordQuality.Maj9), C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Min9), C(10, ChordQuality.Dom13)),
        };
        var b = new BarChords[]
        {
            new(C(7, ChordQuality.Dom13)),
            new(C(7, ChordQuality.Dom13)),
            new(C(0, ChordQuality.Dom13)),
            new(C(0, ChordQuality.Dom13)),
            new(C(5, ChordQuality.Dom13)),
            new(C(5, ChordQuality.Dom13)),
            new(C(10, ChordQuality.Dom13)),
            new(C(10, ChordQuality.Dom13)),
        };
        return a.Concat(a).Concat(b).Concat(a).ToArray();
    }

    private static readonly ChartDef BossaChart = new()
    {
        Name = "bossa", KeyRootPc = 5, SwingFactor = 0f,
        Bars = new BarChords[]
        {
            new(C(5, ChordQuality.Maj9)),
            new(C(5, ChordQuality.Maj9)),
            new(C(7, ChordQuality.Dom13)),
            new(C(7, ChordQuality.Dom13)),
            new(C(7, ChordQuality.Min9)),
            new(C(0, ChordQuality.Dom13)),
            new(C(5, ChordQuality.Maj9)),
            new(C(0, ChordQuality.Dom7Sus)),
            new(C(7, ChordQuality.Min9)),
            new(C(0, ChordQuality.Dom13)),
            new(C(9, ChordQuality.Min9)),
            new(C(2, ChordQuality.Dom7Alt)),
            new(C(7, ChordQuality.Min9)),
            new(C(0, ChordQuality.Dom13)),
            new(C(5, ChordQuality.Maj9)),
            new(C(5, ChordQuality.Maj9)),
        },
    };

    // Cm9↔F13 is the ii–V of Bb — home key Bb so a Bb-centered tune plays it as-is.
    private static readonly ChartDef VampChart = new()
    {
        Name = "vamp", KeyRootPc = 10, SwingFactor = 0.45f,
        Bars = new BarChords[]
        {
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Dom13)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Dom13)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Dom13)),
            new(C(0, ChordQuality.Min9)),
            new(C(5, ChordQuality.Dom7Alt)),
        },
    };

    private static readonly ChartDef NocturneChart = new()
    {
        Name = "nocturne", KeyRootPc = 10, SwingFactor = 0.3f,
        Bars = new BarChords[]
        {
            new(C(10, ChordQuality.Maj9)),
            new(C(7, ChordQuality.Min11)),
            new(C(3, ChordQuality.Maj7Sh11)),
            new(C(5, ChordQuality.Dom7Sus)),
            new(C(10, ChordQuality.Maj9)),
            new(C(8, ChordQuality.Maj7Sh11)),
            new(C(7, ChordQuality.Min11)),
            new(C(5, ChordQuality.Dom7Sus)),
        },
    };

    private static readonly ChartDef Nocturne2Chart = new()
    {
        Name = "nocturne2", KeyRootPc = 3, SwingFactor = 0.3f,
        Bars = new BarChords[]
        {
            new(C(3, ChordQuality.Maj9)),
            new(C(0, ChordQuality.Min11)),
            new(C(8, ChordQuality.Maj7Sh11)),
            new(C(10, ChordQuality.Dom7Sus)),
            new(C(7, ChordQuality.Min11)),
            new(C(0, ChordQuality.Min11)),
            new(C(8, ChordQuality.Maj7Sh11)),
            new(C(10, ChordQuality.Dom7Sus)),
        },
    };

    /// <summary>Nocturne 2 in 3/4 — the jazz waltz. Waltz charts must be single-chord
    /// bars (ChordAt's half-bar split assumes 4/4; asserted in the constructor).</summary>
    private static readonly ChartDef WaltzNocturneChart = new()
    {
        Name = "waltz-nocturne", KeyRootPc = 3, SwingFactor = 0.25f, BeatsPerBar = 3,
        Bars = Nocturne2Chart.Bars,
    };

    // ── Comping rhythm patterns: (beat, duration-in-beats) pairs ──
    private static readonly (float Beat, float Dur)[][] CompPatterns =
    {
        new[] { (1.5f, 0.6f), (3.0f, 0.9f) },                       // Charleston, displaced
        new[] { (0.0f, 0.8f), (2.5f, 0.55f) },
        new[] { (1.5f, 0.5f), (2.5f, 0.4f), (3.5f, 0.6f) },
        new[] { (2.5f, 1.2f) },
        new[] { (0.0f, 0.35f), (1.0f, 0.3f), (2.0f, 0.35f), (3.0f, 0.3f) }, // staccato stabs (tension)
    };

    // ── Melody rhythm templates (onsets in beats; the last onset can become the
    //    phrase-final long note). The last template is the dense solo run. ──
    private static readonly float[][] MelodyTemplates =
    {
        new[] { 0.0f, 0.5f, 1.0f, 1.5f, 2.5f },
        new[] { 0.5f, 1.0f, 1.5f, 2.0f, 3.0f },
        new[] { 0.0f, 1.0f, 1.5f, 2.0f, 2.5f },
        new[] { 1.0f, 1.5f, 2.0f, 3.0f, 3.5f },
        new[] { 0.0f, 2.5f },                                       // sparse (night / heads)
        new[] { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f },        // dense (solo flights)
    };

    private sealed class TuneDef
    {
        public MusicForm Form;
        public ChartDef Chart = BluesChart;
        /// <summary>The tune's key center (pc) before any gear-change lift.</summary>
        public int KeyCenterPc;
        /// <summary>BPM offset from the user tempo setting (the setting stays live —
        /// effective tempo is recomputed from it every bar).</summary>
        public float TempoOffset;
        public int[] LeadPalette = Array.Empty<int>();
        public int CompProgram = Theory.GmEPiano1;
        public int DrumKit;             // 0 standard, Theory.GmBrushKit brushes
        public int ReverbDepth = 40;    // CC91 base, fanned out per channel
        public bool CountIn = true;
        public int BudgetBars = 96;
    }

    private readonly int _sampleRate;
    private readonly Random _rng;

    // ── Mood state (slewed toward SetMood targets once per bar) ──
    private float _intensity = 0.5f;
    private float _night;
    private float _tension;
    private float _ambience;
    private float _hour = 8f;
    private int _dayNumber;
    private float _tempoSetting = 96f;
    private float _swingSetting = 0.6f;
    private bool _tagPending;

    private float _tempo = 96f;      // slewed BPM (max ±2.5 per bar — no jarring lurches)
    private float _ritFactor = 1f;   // Ending-only tempo divisor; NEVER fold into _tempo
    private float _swing;            // effective swing for the current bar
    private float _spb;              // samples per beat at the current tempo

    // ── Tune / setlist state ──
    private TunePhase _phase = TunePhase.Break;
    private int _phaseBarsLeft;      // bars left in the current non-Playing phase
    private TuneDef? _tune;
    private int _tuneBarsLeft;
    private int _barsIntoTune;
    private bool _tuneEndArmed;
    private int _lastDayNumber = -1;
    private MusicForm _prevForm = MusicForm.Nocturne;
    private int _prevKeyCenter = -1;

    // ── Chart / chorus state ──
    private ChartDef _chart = BluesChart;
    private int _barInChart;
    private int _transpose;          // effective offset for the current chart
    private int _keyCenterPc = 10;   // tune key center incl. gear lift (blues-lick root)
    private int _gearLift;
    private ChorusRole _role = ChorusRole.Head;
    private bool _twoFeel;
    private int _chorusIndex;
    private int _mainRun;            // consecutive main-form choruses since an interlude
    private bool _inInterlude;
    private int _tagBarsLeft;        // resolution-tag bars in flight

    // ── Program / console caches (suppress redundant MIDI) ──
    private bool _setupEmitted;
    private int _leadProgram = -1;
    private int _bassProgram = -1;
    private int _compProgram = -1;
    private int _drumKit = -1;
    private int _lastReverbDepth = -1;

    // ── Part state (voice-leading memory) ──
    private int _prevBassKey = 34;
    private int _prevCompTop = 65;
    private int _prevLeadKey = 70;
    private int _leadDirection = 1;
    private int _phraseBars;
    private int _restBars = 1;

    /// <summary>Set by MusicProvider after scanning the soundfont for the brush kit
    /// (bank 128, program 40); when false, tunes fall back to the standard kit.</summary>
    public bool BrushKitAvailable { get; set; }

    // ── Diagnostics (written on the playback thread; read by the offline harness on
    //    the same thread after Read() calls — not for cross-thread UI use) ──
    public int TunesStarted { get; private set; }
    public string CurrentTuneName { get; private set; } = "(none)";

    public Composer(int sampleRate, int seed)
    {
        _sampleRate = sampleRate;
        _rng = new Random(seed);
        Debug.Assert(WaltzNocturneChart.Bars.All(b => b.B == null),
            "3/4 charts must be single-chord bars (ChordAt splits at beat 2)");
    }

    /// <summary>Slews the mood state half-way toward the given inputs — called once per
    /// bar boundary, which quantizes mood changes to musical time.</summary>
    public void SetMood(in MoodInputs m)
    {
        _intensity += (Math.Clamp(m.Intensity, 0f, 1f) - _intensity) * 0.5f;
        _night += (Math.Clamp(m.Night, 0f, 1f) - _night) * 0.5f;
        _tension += (Math.Clamp(m.Tension, 0f, 1f) - _tension) * 0.5f;
        _ambience += (Math.Clamp(m.Ambience, 0f, 1f) - _ambience) * 0.5f;
        _hour = m.Hour;
        _dayNumber = m.DayNumber;
        _tempoSetting = Math.Clamp(m.TempoBpm, 60f, 160f);
        _swingSetting = Math.Clamp(m.Swing, 0f, 1f);
        if (m.ResolutionTag) _tagPending = true;
    }

    /// <summary>
    /// Composes one bar starting at <paramref name="t0"/> (absolute samples), appending
    /// events to <paramref name="ev"/> (unsorted — the provider sorts), and returns the
    /// bar length in samples. Every phase honors the same return-length contract;
    /// Break bars simply emit no events.
    /// </summary>
    public long GenerateBar(List<MidiEvent> ev, long t0)
    {
        if (!_setupEmitted) { EmitSetup(ev, t0); _setupEmitted = true; }

        // ── Phase transitions (evaluated at the bar boundary) ──
        if (_phase == TunePhase.Break && _phaseBarsLeft <= 0)
        {
            BeginTune(ev, t0); // cold open lands here too (_phase initializes to Break/0)
        }
        else if (_phase == TunePhase.CountIn && _phaseBarsLeft <= 0)
        {
            _phase = TunePhase.Playing;
        }
        else if (_phase == TunePhase.Ending && _phaseBarsLeft <= 0)
        {
            _phase = TunePhase.Break;
            _phaseBarsLeft = 1 + _rng.Next(2);
        }

        var tune = _tune!;

        // ── Arm rotation: budget, day rollover, nightfall on a day tune, dawn on a nocturne ──
        if (_phase == TunePhase.Playing)
        {
            if (_dayNumber != _lastDayNumber) _tuneEndArmed = true;
            if (tune.Form != MusicForm.Nocturne && _night > 0.6f) _tuneEndArmed = true;
            if (tune.Form == MusicForm.Nocturne && _night < 0.3f) _tuneEndArmed = true;
            if (_tuneEndArmed && _barInChart == 0 && _tagBarsLeft == 0 && _barsIntoTune > 0)
            {
                _phase = TunePhase.Ending;
                _phaseBarsLeft = 2;
                _ritFactor = 1f;
            }
        }

        UpdateTempo();
        long barLen = (long)(_chart.BeatsPerBar * _spb);
        _swing = _swingSetting * _chart.SwingFactor * (1f - 0.7f * _night);

        switch (_phase)
        {
            case TunePhase.CountIn:
                GenCountIn(ev, t0);
                _phaseBarsLeft--;
                return barLen;

            case TunePhase.Break:
                _phaseBarsLeft--;
                return barLen; // silence; overhanging tails ring into it naturally

            case TunePhase.Ending:
                _ritFactor -= 0.03f; // next ending bar is slower still
                GenTagBar(ev, t0, 2 - _phaseBarsLeft, fillOut: false);
                _phaseBarsLeft--;
                return barLen;
        }

        // ── Playing ──
        if (_tagPending)
        {
            // Jam-cleared resolution: gate, then play the tag at this bar boundary.
            if (_tagBarsLeft == 0 && !_tuneEndArmed && _barsIntoTune >= 2)
                _tagBarsLeft = 2;
            _tagPending = false; // consumed or dropped — never fires stale
        }
        if (_tagBarsLeft > 0)
        {
            GenTagBar(ev, t0, 2 - _tagBarsLeft, fillOut: _tagBarsLeft == 1);
            _tagBarsLeft--;
            if (_tagBarsLeft == 0) _barInChart = 0; // tag is a turnaround into a fresh chorus
            _barsIntoTune++;
            return barLen;
        }

        if (_barInChart == 0) BeginChorus(ev, t0);

        var bar = TransposedBar(_barInChart);
        var nextBar = TransposedBar((_barInChart + 1) % _chart.Bars.Length);
        bool lastBar = _barInChart == _chart.Bars.Length - 1;

        // Layer levels: ambience (far zoom) fades drums down and pads up.
        float drumLevel = Math.Clamp((_intensity - 0.12f) / 0.35f, 0f, 1f)
            * (1f - DspUtil.SmoothStep(0.35f, 0.75f, _night))
            * (1f - 0.5f * _ambience);
        bool drumsOwnBar = _role == ChorusRole.Solo && _chart.Bars.Length >= 8
            && _intensity > 0.5f && _barInChart / 4 % 2 == 1;
        float melodyAct = DspUtil.SmoothStep(0.35f, 0.7f, _intensity)
            * (1f - 0.5f * _tension) * (1f - 0.35f * _night)
            * (_role == ChorusRole.Solo ? 1.2f : 0.55f);

        var style = DrumStyleFor();
        GenBass(ev, t0, bar, nextBar);
        GenComp(ev, t0, bar);
        GenPad(ev, t0, bar, barLen);
        if (drumLevel > 0.06f) GenDrums(ev, t0, style, drumLevel, lastBar, fillBoost: drumsOwnBar);
        if (drumsOwnBar)
        {
            // Trading fours: the drums own this bar; the melody sits out and re-enters fresh.
            _phraseBars = 0;
            _restBars = 1;
        }
        else
        {
            GenLead(ev, t0, bar, melodyAct);
        }
        if (_role == ChorusRole.Solo && _intensity > 0.7f
            && tune.Form is not (MusicForm.Bossa or MusicForm.Nocturne))
            GenHorns(ev, t0, bar, barLen);
        if (tune.Form is MusicForm.Blues or MusicForm.MinorBlues && !_twoFeel
            && _intensity > 0.72f && _night < 0.3f && !_inInterlude)
            GenBoogiePiano(ev, t0, bar);

        _barInChart = (_barInChart + 1) % _chart.Bars.Length;
        _barsIntoTune++;
        _tuneBarsLeft--;
        if (_tuneBarsLeft <= 0) _tuneEndArmed = true;
        return barLen;
    }

    // ═══════════════════════ Tune lifecycle ═══════════════════════

    /// <summary>Starts the next tune: picks it, hard-sets the tempo (the ±2.5 BPM/bar
    /// slew would otherwise glide for ~15 bars — the break hides the jump), refreshes
    /// the per-tune console, and enters CountIn (or straight into Playing for
    /// nocturnes, which fade in on pads instead of counting off).</summary>
    private void BeginTune(List<MidiEvent> ev, long t0)
    {
        PickTune();
        var tune = _tune!;
        _keyCenterPc = tune.KeyCenterPc;
        _gearLift = 0;
        SetChart(tune.Chart);
        _tempo = EffectiveTempoBase();
        _ritFactor = 1f;
        EmitTuneSetup(ev, t0);

        _barInChart = 0;
        _chorusIndex = 0;
        _mainRun = 0;
        _inInterlude = false;
        _tuneEndArmed = false;
        _barsIntoTune = 0;
        _tuneBarsLeft = tune.BudgetBars;
        _tagBarsLeft = 0;
        _tagPending = false;
        _phraseBars = 0;
        _restBars = 1;

        _phase = tune.CountIn ? TunePhase.CountIn : TunePhase.Playing;
        _phaseBarsLeft = 1;
    }

    /// <summary>The single place hour-of-day, night, and repetition-avoidance weighting
    /// happens. Everything downstream just plays the tune it is handed.</summary>
    private void PickTune()
    {
        _lastDayNumber = _dayNumber;

        // Form weights: hour buckets bias the daytime repertoire; night overrides.
        Span<(MusicForm Form, float W)> pool = stackalloc (MusicForm, float)[]
        {
            (MusicForm.Blues, 1.0f * (_hour is >= 6f and < 10f ? 1.5f : 1f)),
            (MusicForm.MinorBlues, 0.8f * (_hour is >= 15f and < 20f ? 1.3f : 1f)),
            (MusicForm.Aaba, 1.0f),
            (MusicForm.Bossa, 0.7f * (_hour is >= 10f and < 15f ? 1.8f : 1f)),
            (MusicForm.Vamp, 0.6f * (_hour is >= 15f and < 20f ? 1.6f : 1f)),
            (MusicForm.Nocturne, _night > 0.5f ? 6f : 0.05f),
        };
        float total = 0f;
        for (int i = 0; i < pool.Length; i++)
        {
            if (pool[i].Form == _prevForm) pool[i].W *= 0.15f; // don't repeat the set
            total += pool[i].W;
        }
        float roll = (float)_rng.NextDouble() * total;
        MusicForm form = pool[^1].Form;
        foreach (var (f, w) in pool)
        {
            if (roll < w) { form = f; break; }
            roll -= w;
        }

        ChartDef chart = form switch
        {
            MusicForm.MinorBlues => MinorBluesChart,
            MusicForm.Aaba => AabaChart,
            MusicForm.Bossa => BossaChart,
            MusicForm.Vamp => VampChart,
            MusicForm.Nocturne => _rng.NextDouble() switch
            {
                < 0.35 => WaltzNocturneChart,
                < 0.70 => Nocturne2Chart,
                _ => NocturneChart,
            },
            _ => BluesChart,
        };

        // Key centers rotate through the horn-friendly set; never repeat the previous tune's.
        Span<int> keys = stackalloc[] { 10, 3, 5, 0, 8 }; // Bb Eb F C Ab
        int keyIdx = _rng.Next(keys.Length);
        if (keys[keyIdx] == _prevKeyCenter)
            keyIdx = (keyIdx + 1 + _rng.Next(keys.Length - 1)) % keys.Length;
        int key = keys[keyIdx];

        int[] palette = form switch
        {
            MusicForm.Nocturne => new[] { Theory.GmFlute, Theory.GmVibraphone, Theory.GmClarinet },
            MusicForm.Bossa => new[] { Theory.GmVibraphone, Theory.GmFlute, Theory.GmMutedTrumpet },
            _ => PickDayPalette(),
        };

        int comp = form switch
        {
            MusicForm.Blues or MusicForm.MinorBlues => _rng.NextDouble() switch
            {
                < 0.5 => Theory.GmEPiano1,
                < 0.75 => Theory.GmDrawbarOrgan,
                _ => Theory.GmJazzGuitar,
            },
            MusicForm.Bossa => _rng.NextDouble() < 0.6 ? Theory.GmJazzGuitar : Theory.GmEPiano1,
            MusicForm.Vamp => _rng.NextDouble() < 0.35 ? Theory.GmDrawbarOrgan : Theory.GmEPiano1,
            MusicForm.Nocturne => Theory.GmEPiano1,
            _ => _rng.NextDouble() < 0.35 ? Theory.GmJazzGuitar : Theory.GmEPiano1,
        };

        bool brush = BrushKitAvailable
            && (form == MusicForm.Nocturne || (form == MusicForm.Bossa && _rng.NextDouble() < 0.2));

        float tempoOffset = _rng.Next(-10, 13) + (form == MusicForm.Nocturne ? -8f : 0f);
        var tune = new TuneDef
        {
            Form = form,
            Chart = chart,
            KeyCenterPc = key,
            TempoOffset = tempoOffset,
            LeadPalette = palette,
            CompProgram = comp,
            DrumKit = brush ? Theory.GmBrushKit : 0,
            ReverbDepth = form == MusicForm.Nocturne ? 55 + _rng.Next(26) : 30 + _rng.Next(41),
            CountIn = form != MusicForm.Nocturne,
        };
        _tune = tune;

        // Budget ≈ 4–6 minutes at the tune's tempo, rounded to whole chart cycles.
        float bpm = EffectiveTempoBase();
        int budgetBars = (int)((240 + _rng.Next(121)) * (bpm / 60f) / chart.BeatsPerBar);
        int cycle = chart.Bars.Length;
        tune.BudgetBars = Math.Max(cycle * 2, budgetBars / cycle * cycle);

        _prevForm = form;
        _prevKeyCenter = key;
        TunesStarted++;
        CurrentTuneName = $"{chart.Name}@{key}";
    }

    private int[] PickDayPalette()
    {
        Span<int> all = stackalloc[]
        {
            Theory.GmAltoSax, Theory.GmTenorSax, Theory.GmMutedTrumpet, Theory.GmHarmonica,
            Theory.GmVibraphone, Theory.GmClarinet, Theory.GmSopranoSax,
        };
        int first = _rng.Next(all.Length);
        int second = (first + 1 + _rng.Next(all.Length - 1)) % all.Length;
        int third = (second + 1 + _rng.Next(all.Length - 1)) % all.Length;
        return third == first
            ? new[] { all[first], all[second] }
            : new[] { all[first], all[second], all[third] };
    }

    /// <summary>Effective tempo base: the LIVE user setting plus the tune's offset —
    /// the Music-page tempo slider keeps working mid-tune.</summary>
    private float EffectiveTempoBase()
        => Math.Clamp(_tempoSetting + (_tune?.TempoOffset ?? 0f), 66f, 140f);

    private void UpdateTempo()
    {
        if (_phase == TunePhase.Playing)
        {
            float target = EffectiveTempoBase() * (1f - 0.10f * _night) * (1f + 0.05f * _tension);
            _tempo += Math.Clamp(target - _tempo, -2.5f, 2.5f);
        }
        // Ending: no slew — the ritardando factor owns the slowdown. It must never be
        // folded into _tempo/_tempoSetting: both are overwritten from the UI every bar.
        _spb = _sampleRate * 60f / (_tempo * _ritFactor);
    }

    private void SetChart(ChartDef chart)
    {
        _chart = chart;
        int center = (_keyCenterPc + _gearLift) % 12;
        _transpose = ((center - chart.KeyRootPc) % 12 + 12) % 12;
    }

    private BarChords TransposedBar(int index)
    {
        var bar = _chart.Bars[index];
        if (_transpose == 0) return bar;
        var a = bar.A with { Root = (bar.A.Root + _transpose) % 12 };
        return bar.B is Chord b ? new BarChords(a, b with { Root = (b.Root + _transpose) % 12 }) : new BarChords(a);
    }

    /// <summary>Chorus boundary within a Playing tune: interlude switching, head/solo
    /// role, two-feel, the gear-change lift, lead rotation, and the section crash.</summary>
    private void BeginChorus(List<MidiEvent> ev, long t0)
    {
        var tune = _tune!;
        bool first = _barsIntoTune == 0;
        if (!first) _chorusIndex++;

        bool interludeEligible = tune.Form is MusicForm.Blues or MusicForm.MinorBlues or MusicForm.Aaba;
        if (_inInterlude)
        {
            _inInterlude = false;
            _mainRun = 0;
            SetChart(tune.Chart);
        }
        else if (!first && interludeEligible && (_tension > 0.55f || _mainRun >= 3))
        {
            _inInterlude = true;
            SetChart(VampChart);
        }
        else
        {
            _mainRun++;
        }

        // Final chorus: this cycle will exhaust the budget. Decide the gear-change NOW
        // so the walking bass's chromatic approach targets the lifted key from bar one.
        bool finalChorus = _tuneBarsLeft <= _chart.Bars.Length || _tuneEndArmed;
        if (finalChorus)
        {
            _tuneEndArmed = true;
            if (_gearLift == 0 && _rng.NextDouble() < 0.25
                && tune.Form is not (MusicForm.Nocturne or MusicForm.Bossa))
            {
                _gearLift = 1;
                SetChart(_chart);
            }
        }

        _role = first || finalChorus ? ChorusRole.Head : ChorusRole.Solo;
        _twoFeel = first && tune.Form is MusicForm.Blues or MusicForm.MinorBlues or MusicForm.Aaba;

        // Lead rotation within the tune's palette.
        int pick = tune.LeadPalette[_rng.Next(tune.LeadPalette.Length)];
        if (pick == _leadProgram && tune.LeadPalette.Length > 1)
            pick = tune.LeadPalette[(Array.IndexOf(tune.LeadPalette, pick) + 1) % tune.LeadPalette.Length];
        if (pick != _leadProgram) { _leadProgram = pick; Program(ev, t0, ChLead, pick); }

        int bass = _chart == VampChart || tune.Form == MusicForm.Vamp
            ? Theory.GmFingerBass : Theory.GmAcousticBass;
        if (bass != _bassProgram) { _bassProgram = bass; Program(ev, t0, ChBass, bass); }

        if (tune.Form != MusicForm.Nocturne && _intensity > 0.3f)
            Note(ev, ChDrums, Theory.DrCrash, Vel(72, 6), t0, (long)(1.5f * _sampleRate * 60f / _tempo));
    }

    private DrumStyle DrumStyleFor()
    {
        if (_chart == VampChart || _tune!.Form == MusicForm.Vamp) return DrumStyle.Funk;
        if (_tune.Form == MusicForm.Bossa) return DrumStyle.Bossa;
        if (_tune.Form == MusicForm.Nocturne) return DrumStyle.NightWaltz;
        return DrumStyle.Swing;
    }

    // ═══════════════════════ Console setup ═══════════════════════

    /// <summary>One-time console state — volumes, pans, chorus sends, and the fixed
    /// bass/pad/piano/horn programs. Reverb (CC91) deliberately does NOT live here:
    /// EmitTuneSetup owns it per tune (single-owner rule).</summary>
    private void EmitSetup(List<MidiEvent> ev, long t)
    {
        Program(ev, t, ChPad, Theory.GmWarmPad);
        Program(ev, t, ChPiano, Theory.GmAcousticGrand);
        Program(ev, t, ChHorns, Theory.GmBrassSection);

        Cc(ev, t, ChComp, 7, 84); Cc(ev, t, ChComp, 10, 52); Cc(ev, t, ChComp, 93, 30);
        Cc(ev, t, ChBass, 7, 105); Cc(ev, t, ChBass, 10, 64);
        Cc(ev, t, ChLead, 7, 88); Cc(ev, t, ChLead, 10, 76);
        Cc(ev, t, ChPad, 7, 72); Cc(ev, t, ChPad, 10, 64); Cc(ev, t, ChPad, 93, 40);
        Cc(ev, t, ChPiano, 7, 62); Cc(ev, t, ChPiano, 10, 40);
        Cc(ev, t, ChHorns, 7, 58); Cc(ev, t, ChHorns, 10, 58);
        Cc(ev, t, ChDrums, 7, 96);
    }

    /// <summary>Per-tune console refresh: comping program, drum kit, and the reverb
    /// room fanned out per channel from the tune's depth. Guarded by cached-last-value
    /// checks so unchanged tunes emit nothing.</summary>
    private void EmitTuneSetup(List<MidiEvent> ev, long t)
    {
        var tune = _tune!;
        if (tune.CompProgram != _compProgram)
        {
            _compProgram = tune.CompProgram;
            Program(ev, t, ChComp, tune.CompProgram);
        }
        if (tune.DrumKit != _drumKit)
        {
            _drumKit = tune.DrumKit;
            Program(ev, t, ChDrums, tune.DrumKit);
        }
        if (tune.ReverbDepth != _lastReverbDepth)
        {
            _lastReverbDepth = tune.ReverbDepth;
            int d = tune.ReverbDepth;
            Cc(ev, t, ChComp, 91, (int)(d * 0.55f));
            Cc(ev, t, ChBass, 91, 12);
            Cc(ev, t, ChLead, 91, (int)(d * 0.9f));
            Cc(ev, t, ChPad, 91, Math.Min(d + 25, 90));
            Cc(ev, t, ChPiano, 91, (int)(d * 0.45f));
            Cc(ev, t, ChHorns, 91, (int)(d * 0.8f));
            Cc(ev, t, ChDrums, 91, (int)(d * 0.65f));
        }
    }

    /// <summary>The count-off: pedal-hat clicks on every beat of one bar at the new tempo.</summary>
    private void GenCountIn(List<MidiEvent> ev, long t0)
    {
        long tick = (long)(0.1f * _spb);
        for (int b = 0; b < _chart.BeatsPerBar; b++)
            Note(ev, ChDrums, Theory.DrHatPedal, Vel(b == 0 ? 66 : 56, 3), Beat(t0, b), tick);
    }

    /// <summary>A tag bar — the shared ii–V / I cadence used by tune Endings and the
    /// jam-cleared resolution. Bar 0: ii on beats 1–2, V on 3–4. Bar 1: the I chord,
    /// held. <paramref name="fillOut"/> adds the tom fill leading back into a chorus
    /// (resolution tags only; endings decay into the break instead).</summary>
    private void GenTagBar(List<MidiEvent> ev, long t0, int index, bool fillOut)
    {
        int center = (_keyCenterPc + _gearLift) % 12;
        ChordQuality resolveQuality = _tune!.Form is MusicForm.Blues or MusicForm.MinorBlues or MusicForm.Vamp
            ? ChordQuality.Dom13 : ChordQuality.Maj9;
        var ii = new Chord((center + 2) % 12, ChordQuality.Min9);
        var v = new Chord((center + 7) % 12, ChordQuality.Dom13);
        var i = new Chord(center, resolveQuality);

        long tick = (long)(0.1f * _spb);
        if (index == 0)
        {
            int iiKey = PlaceNear(ii.Root, _prevBassKey, 26, 50);
            int vKey = PlaceNear(v.Root, iiKey, 26, 50);
            Note(ev, ChBass, iiKey, Vel(76, 4), Beat(t0, 0f), (long)(1.8f * _spb));
            Note(ev, ChBass, vKey, Vel(76, 4), Beat(t0, 2f), (long)(1.8f * _spb));
            _prevBassKey = vKey;
            CompHit(ev, Beat(t0, 0f), ii, (long)(1.6f * _spb), Vel(56, 6));
            CompHit(ev, Beat(t0, 2f), v, (long)(1.6f * _spb), Vel(58, 6));
            if (_intensity > 0.25f)
            {
                Note(ev, ChDrums, Theory.DrRide, Vel(56, 4), Beat(t0, 0f), tick);
                Note(ev, ChDrums, Theory.DrRide, Vel(52, 4), Beat(t0, 2f), tick);
            }
            if (fillOut)
            {
                Span<int> toms = stackalloc[] { Theory.DrTomHi, Theory.DrTomMid, Theory.DrTomLow };
                float[] beats = { 2.5f, 3f, 3.5f };
                for (int k = 0; k < beats.Length; k++)
                    Note(ev, ChDrums, toms[k], Vel(66 + k * 6, 4), Beat(t0, Swing8(beats[k])), tick);
            }
        }
        else
        {
            int iKey = PlaceNear(i.Root, _prevBassKey, 26, 50);
            Note(ev, ChBass, iKey, Vel(80, 4), Beat(t0, 0f), (long)(3.6f * _spb));
            _prevBassKey = iKey;
            CompHit(ev, Beat(t0, 0f), i, (long)(3.5f * _spb), Vel(58, 4));
            if (_intensity > 0.25f)
            {
                Note(ev, ChDrums, Theory.DrCrash, Vel(74, 5), Beat(t0, 0f), (long)(2f * _spb));
                Note(ev, ChDrums, Theory.DrRide, Vel(50, 4), Beat(t0, 2f), tick);
            }
            if (fillOut)
            {
                Span<int> toms2 = stackalloc[] { Theory.DrTomHi, Theory.DrTomMid, Theory.DrTomLow };
                float[] beats2 = { 2.5f, 3f, 3.5f };
                for (int k = 0; k < beats2.Length; k++)
                    Note(ev, ChDrums, toms2[k], Vel(66 + k * 6, 4), Beat(t0, Swing8(beats2[k])), tick);
            }
        }
    }

    // ═══════════════════════ Bass ═══════════════════════

    private void GenBass(List<MidiEvent> ev, long t0, BarChords bar, BarChords nextBar)
    {
        var tune = _tune!;

        if (tune.Form == MusicForm.Nocturne)
        {
            int root = PlaceNear(bar.A.Root, _prevBassKey, 26, 45);
            Note(ev, ChBass, root, Vel(56, 4), Beat(t0, 0f), (long)((_chart.BeatsPerBar - 0.4f) * _spb));
            _prevBassKey = root;
            if (_chart.BeatsPerBar == 4 && _rng.NextDouble() < 0.4)
                Note(ev, ChBass, root + 7, Vel(48, 4), Beat(t0, 2f), (long)(1.7f * _spb));
            return;
        }

        if (tune.Form == MusicForm.Bossa)
        {
            // Tumbao-ish: dotted root / fifth push, straight 8ths.
            int root = PlaceNear(bar.A.Root, _prevBassKey, 26, 45);
            int approach = PlaceNear(nextBar.A.Root, root, 26, 45) + (_rng.Next(2) == 0 ? 1 : -1);
            Note(ev, ChBass, root, Vel(78, 4), Beat(t0, 0f), (long)(1.35f * _spb));
            Note(ev, ChBass, root + 7, Vel(62, 4), Beat(t0, 1.5f), (long)(0.4f * _spb));
            Note(ev, ChBass, root, Vel(74, 4), Beat(t0, 2f), (long)(1.35f * _spb));
            Note(ev, ChBass, approach, Vel(58, 4), Beat(t0, 3.5f), (long)(0.4f * _spb));
            _prevBassKey = root;
            return;
        }

        if (_chart == VampChart || tune.Form == MusicForm.Vamp)
        {
            // Fixed funk ostinato transposed to the bar's root; tension pushes toward
            // insistent root 8ths (the pedal that reads as "stuck traffic").
            int root = PlaceNear(bar.A.Root, 33, 26, 40);
            if (_tension > 0.7f)
            {
                for (int e = 0; e < 8; e++)
                    Note(ev, ChBass, root, Vel(e % 2 == 0 ? 86 : 68, 5), Beat(t0, Swing8(e * 0.5f)), (long)(0.4f * _spb));
            }
            else
            {
                (float BeatPos, int Offset, float Dur, int V)[] hits =
                {
                    (0.0f, 0, 0.7f, 96), (1.5f, 0, 0.2f, 70), (1.75f, 10, 0.2f, 70),
                    (2.0f, 12, 0.45f, 90), (3.0f, 7, 0.3f, 76), (3.5f, 10, 0.25f, 70),
                };
                foreach (var h in hits)
                    Note(ev, ChBass, root + h.Offset, Vel(h.V, 5), Beat(t0, h.BeatPos), (long)(h.Dur * _spb));
            }
            _prevBassKey = root;
            return;
        }

        if (_twoFeel)
        {
            // Head-chorus two-feel: half-note roots and fifths, kicking into four next chorus.
            int root = PlaceNear(bar.A.Root, _prevBassKey, 26, 50);
            int second = bar.B is Chord chB
                ? PlaceNear(chB.Root, root, 26, 50)
                : (_rng.NextDouble() < 0.5 ? root + 7 : PlaceNear(nextBar.A.Root, root, 26, 50) + 1);
            Note(ev, ChBass, root, Vel(76, 4), Beat(t0, 0f), (long)(1.85f * _spb));
            Note(ev, ChBass, second, Vel(70, 4), Beat(t0, 2f), (long)(1.85f * _spb));
            if (_rng.NextDouble() < 0.3)
                Note(ev, ChBass, second + (_rng.Next(2) == 0 ? 1 : -1), Vel(54, 4),
                    Beat(t0, Swing8(3.5f)), (long)(0.3f * _spb));
            _prevBassKey = second;
            return;
        }

        // Classic four-to-the-bar walking line.
        var tonesA = Theory.ChordTones(bar.A.Quality);
        int nextRootPc = nextBar.A.Root;

        int b0 = PlaceNear(bar.A.Root, _prevBassKey, 26, 50);
        int b1, b2;
        if (bar.B is Chord chordB)
        {
            b1 = PlaceNear((bar.A.Root + tonesA[1 + _rng.Next(2)]) % 12, b0, 26, 50);
            b2 = PlaceNear(chordB.Root, b1, 26, 50);
        }
        else
        {
            b1 = PlaceNear((bar.A.Root + tonesA[1 + _rng.Next(2)]) % 12, b0, 26, 50);
            var scale = Theory.Scale(bar.A.Quality);
            int approachTarget = PlaceNear(nextRootPc, b1, 26, 50);
            int mid = (b1 + approachTarget) / 2;
            b2 = PlaceNear((bar.A.Root + scale[_rng.Next(scale.Length)]) % 12, mid, 26, 50);
        }
        int nextRootKey = PlaceNear(nextRootPc, b2, 26, 50);
        int b3 = nextRootKey + (_rng.Next(2) == 0 ? 1 : -1);

        Span<int> keys = stackalloc int[] { b0, b1, b2, b3 };
        for (int beat = 0; beat < 4; beat++)
        {
            Note(ev, ChBass, keys[beat], Vel(beat == 0 ? 82 : 74, 5), Beat(t0, beat), (long)(0.92f * _spb));
            if (_rng.NextDouble() < 0.10 && beat < 3)
                Note(ev, ChBass, keys[beat], Vel(58, 5), Beat(t0, Swing8(beat + 0.5f)), (long)(0.3f * _spb));
        }
        _prevBassKey = b3;
    }

    // ═══════════════════════ Comping ═══════════════════════

    private void GenComp(List<MidiEvent> ev, long t0, BarChords bar)
    {
        if (_tune!.Form == MusicForm.Nocturne)
        {
            if (_rng.NextDouble() < 0.55)
                CompHit(ev, Beat(t0, 0f), bar.A, (long)((_chart.BeatsPerBar - 0.4f) * _spb), Vel(44, 4));
            return;
        }

        int patternIdx = _tension > 0.5f ? 4
            : _role == ChorusRole.Solo ? (_rng.NextDouble() < 0.5 ? 3 : _rng.Next(2)) // solos: stay out of the way
            : _chart == VampChart ? 2 + _rng.Next(3)
            : _rng.Next(4);
        var pattern = CompPatterns[patternIdx];
        float density = (0.45f + 0.55f * _intensity) * (1f - 0.35f * _ambience);
        float durScale = _tension > 0.5f ? 0.45f : 1f;

        foreach (var (beat, dur) in pattern)
        {
            if (_rng.NextDouble() > density && pattern.Length > 1) continue;
            var chord = ChordAt(bar, beat);
            bool offbeat = beat - MathF.Floor(beat) > 0.01f;
            CompHit(ev, Beat(t0, Swing8(beat)), chord, (long)(dur * durScale * _spb), Vel(offbeat ? 64 : 56, 8));
        }
    }

    /// <summary>One comping voicing: picks the A/B rootless shape (with octave shift)
    /// whose top note lands nearest the previous hit's top — nearest-neighbor voice leading.</summary>
    private void CompHit(List<MidiEvent> ev, long time, Chord chord, long dur, int vel)
    {
        var shapes = Theory.Voicings(chord.Quality);
        int rootBase = 48 + chord.Root % 12;
        int[]? best = null;
        int bestShift = 0, bestDist = int.MaxValue;
        foreach (var shape in shapes)
        {
            int top = rootBase + shape[^1];
            foreach (int shift in stackalloc[] { -12, 0, 12 })
            {
                int d = Math.Abs(top + shift - _prevCompTop);
                if (d < bestDist) { bestDist = d; best = shape; bestShift = shift; }
            }
        }
        if (best == null) return;
        foreach (int interval in best)
            Note(ev, ChComp, rootBase + interval + bestShift, vel, time, dur);
        _prevCompTop = rootBase + best[^1] + bestShift;
    }

    // ═══════════════════════ Pad ═══════════════════════

    private void GenPad(List<MidiEvent> ev, long t0, BarChords bar, long barLen)
    {
        // Night float, and the far-zoom "overview ambient" bed.
        float presence = MathF.Max(_night, 0.8f * _ambience);
        if (presence < 0.15f) return;
        var tones = Theory.ChordTones(bar.A.Quality);
        int root = 36 + bar.A.Root % 12;
        int vel = Vel((int)(28f + 36f * presence), 3);
        long dur = (long)(barLen * 1.02f);
        Note(ev, ChPad, root, vel, t0, dur);
        Note(ev, ChPad, root + 7, vel, t0, dur);
        Note(ev, ChPad, root + 12 + tones[1], vel, t0, dur);
        Note(ev, ChPad, root + 12 + tones[^1], vel, t0, dur);
        if (presence > 0.6f)
            Note(ev, ChPad, root + 26, Vel(vel - 6, 3), t0, dur);
    }

    // ═══════════════════════ Horns ═══════════════════════

    /// <summary>Brass-section guide tones behind solo choruses: the 3rd and 7th of the
    /// bar's chord as a sustained dyad — one voicing, instant big-band lift.</summary>
    private void GenHorns(List<MidiEvent> ev, long t0, BarChords bar, long barLen)
    {
        var tones = Theory.ChordTones(bar.A.Quality);
        int third = PlaceNear((bar.A.Root + tones[1]) % 12, 58, 52, 64);
        int seventh = PlaceNear((bar.A.Root + tones[^1]) % 12, 63, 56, 68);
        int vel = Vel((int)(38f + 14f * _intensity), 3);
        long dur = (long)(barLen * 0.96f);
        Note(ev, ChHorns, third, vel, t0, dur);
        Note(ev, ChHorns, seventh, vel, t0, dur);
    }

    // ═══════════════════════ Drums ═══════════════════════

    private void GenDrums(List<MidiEvent> ev, long t0, DrumStyle style, float level,
        bool lastBar, bool fillBoost)
    {
        int V(int v) => Vel(Math.Max(1, (int)(v * (0.45f + 0.55f * level))), 4);
        long tick = (long)(0.1f * _spb);

        switch (style)
        {
            case DrumStyle.Funk:
            {
                for (int s = 0; s < 16; s++)
                {
                    if (level < 0.5f && s % 2 == 1) continue;
                    bool open = s == 15 && _rng.NextDouble() < 0.25;
                    int vel = V(s % 4 == 0 ? 60 : s % 2 == 0 ? 46 : 38 + _rng.Next(10));
                    Note(ev, ChDrums, open ? Theory.DrHatOpen : Theory.DrHatClosed, vel, Beat(t0, s * 0.25f), tick);
                }
                foreach (float b in stackalloc[] { 0f, 0.75f, 2.5f })
                    Note(ev, ChDrums, Theory.DrKick, V(92), Beat(t0, b), tick);
                if (_rng.NextDouble() < 0.2)
                    Note(ev, ChDrums, Theory.DrKick, V(78), Beat(t0, 1.75f), tick);
                Note(ev, ChDrums, Theory.DrSnare, V(96), Beat(t0, 1f), tick);
                Note(ev, ChDrums, Theory.DrSnare, V(96), Beat(t0, 3f), tick);
                foreach (float b in stackalloc[] { 1.75f, 2.25f, 3.25f })
                    if (_rng.NextDouble() < (fillBoost ? 0.75 : 0.4))
                        Note(ev, ChDrums, Theory.DrSnare, V(fillBoost ? 40 : 26), Beat(t0, b), tick);
                return;
            }

            case DrumStyle.Bossa:
            {
                // Straight-8th shaker bed, 3-2 clave-ish rim clicks, surdo-style kick.
                for (int e = 0; e < 8; e++)
                    Note(ev, ChDrums, Theory.DrShaker, V(e % 2 == 0 ? 46 : 34), Beat(t0, e * 0.5f), tick);
                foreach (float b in stackalloc[] { 0f, 1.5f, 3f })
                    Note(ev, ChDrums, Theory.DrSideStick, V(52), Beat(t0, b), tick);
                foreach (float b in stackalloc[] { 0f, 1.75f, 2f, 3.75f })
                    Note(ev, ChDrums, Theory.DrKick, V(b is 0f or 2f ? 72 : 48), Beat(t0, b), tick);
                Note(ev, ChDrums, Theory.DrHatPedal, V(44), Beat(t0, 1f), tick);
                Note(ev, ChDrums, Theory.DrHatPedal, V(44), Beat(t0, 3f), tick);
                return;
            }

            case DrumStyle.NightWaltz:
            {
                // Barely-there brushwork; covers both 4/4 nocturnes and the 3/4 waltz.
                Note(ev, ChDrums, Theory.DrRide, V(44), Beat(t0, 0f), tick);
                for (int b = 1; b < _chart.BeatsPerBar; b++)
                    Note(ev, ChDrums, Theory.DrHatPedal, V(36), Beat(t0, b), tick);
                return;
            }
        }

        // Swing: ride pattern + pedal hat, feathered kick, ghost snare.
        foreach (float b in stackalloc[] { 0f, 1f, 1.5f, 2f, 3f, 3.5f })
        {
            bool skip = b - MathF.Floor(b) > 0.01f;
            Note(ev, ChDrums, Theory.DrRide, V(skip ? 46 : 62), Beat(t0, Swing8(b)), tick);
        }
        if (_intensity > 0.8f && _barInChart % 4 == 0)
            Note(ev, ChDrums, Theory.DrRideBell, V(58), Beat(t0, 0f), tick);
        Note(ev, ChDrums, Theory.DrHatPedal, V(52), Beat(t0, 1f), tick);
        Note(ev, ChDrums, Theory.DrHatPedal, V(52), Beat(t0, 3f), tick);
        if (_intensity > 0.55f)
            for (int b = 0; b < 4; b++)
                Note(ev, ChDrums, Theory.DrKick, V(30), Beat(t0, b), tick);
        if (_rng.NextDouble() < 0.08)
            Note(ev, ChDrums, Theory.DrKick, V(70), Beat(t0, Swing8(2.5f)), tick);
        float ghostChance = fillBoost ? 0.6f : 0.25f * _intensity;
        for (int b = 0; b < 4; b++)
            if (_rng.NextDouble() < ghostChance)
                Note(ev, ChDrums, Theory.DrSnare, V(fillBoost ? 38 + _rng.Next(14) : 24 + _rng.Next(10)),
                    Beat(t0, Swing8(b + 0.5f)), tick);
        if (fillBoost)
        {
            // Trading-fours bars: the kit answers — snare/tom accents on the back half.
            Note(ev, ChDrums, _rng.Next(2) == 0 ? Theory.DrTomMid : Theory.DrSnare, V(62),
                Beat(t0, Swing8(2.5f)), tick);
            Note(ev, ChDrums, Theory.DrTomHi, V(58), Beat(t0, Swing8(3.5f)), tick);
        }
        if (_intensity is > 0.3f and < 0.7f && _barInChart % 2 == 1 && !fillBoost)
            Note(ev, ChDrums, Theory.DrSideStick, V(55), Beat(t0, 3f), tick);

        if (lastBar && level > 0.4f)
        {
            Span<int> toms = stackalloc[] { Theory.DrTomHi, Theory.DrTomMid, Theory.DrTomLow };
            float[] fillBeats = { 2.5f, 3f, 3.5f };
            for (int i = 0; i < fillBeats.Length; i++)
                Note(ev, ChDrums, toms[i], V(70 + i * 6), Beat(t0, Swing8(fillBeats[i])), tick);
        }
    }

    // ═══════════════════════ Lead melody ═══════════════════════

    private void GenLead(List<MidiEvent> ev, long t0, BarChords bar, float activity)
    {
        if (_phraseBars <= 0)
        {
            _restBars--;
            if (_restBars > 0 || activity < 0.15f) return;
            _phraseBars = activity > 0.6f ? 2 : 1;
        }

        bool finalBar = _phraseBars == 1;
        _phraseBars--;
        if (_phraseBars <= 0)
            _restBars = 1 + _rng.Next(2) + (activity < 0.4f ? 2 : 0);

        bool solo = _role == ChorusRole.Solo;
        var template = _night > 0.5f || _chart.BeatsPerBar == 3 ? MelodyTemplates[4]
            : solo && activity > 0.7f && _rng.NextDouble() < 0.45 ? MelodyTemplates[5]
            : MelodyTemplates[_rng.Next(MelodyTemplates.Length - 2)];

        bool bluesLick = _tune!.Form is MusicForm.Blues or MusicForm.MinorBlues
            && _rng.NextDouble() < 0.35;
        int rangeHi = 79 + (solo ? 5 : 0);

        int baseVel = _night > 0.5f ? 52 : 66;
        for (int i = 0; i < template.Length; i++)
        {
            float beat = template[i];
            if (beat >= _chart.BeatsPerBar) continue;
            bool longNote = finalBar && i == template.Length - 1;
            var chord = ChordAt(bar, beat);

            int key;
            if (longNote)
            {
                var tones = Theory.ChordTones(chord.Quality);
                Span<int> colors = stackalloc[] { tones[1], 2, 9 };
                int bestKey = _prevLeadKey, bestDist = int.MaxValue;
                foreach (int c in colors)
                {
                    int k = PlaceNear((chord.Root + c) % 12, _prevLeadKey, 55, rangeHi);
                    int d = Math.Abs(k - _prevLeadKey);
                    if (d < bestDist) { bestDist = d; bestKey = k; }
                }
                key = bestKey;
                if (_rng.NextDouble() < 0.25)
                    Note(ev, ChLead, key - 1, Vel(baseVel - 20, 4), Beat(t0, Swing8(beat) - 0.09f), (long)(0.09f * _spb));
            }
            else
            {
                key = bluesLick
                    ? StepScale(_prevLeadKey, WalkStep(), (_keyCenterPc + _gearLift) % 12, Theory.BluesScale)
                    : StepScale(_prevLeadKey, WalkStep(), chord.Root, Theory.Scale(chord.Quality));
                if (key > rangeHi) key -= 12;
                if (key < 55) key += 12;
            }

            float arc = template.Length > 1 ? (float)i / (template.Length - 1) : 1f;
            int vel = Vel(baseVel + (int)(14f * arc) + (longNote ? 6 : 0), 6);
            float durBeats = longNote ? _chart.BeatsPerBar - beat + 0.4f
                : (i + 1 < template.Length ? (template[i + 1] - beat) * 0.85f : 0.6f);
            Note(ev, ChLead, key, vel, Beat(t0, Swing8(beat)), (long)(durBeats * _spb));
            _prevLeadKey = key;
        }
    }

    /// <summary>Melodic step: mostly continues the current direction, mostly by step —
    /// the constrained random walk that reads as intent rather than noise.</summary>
    private int WalkStep()
    {
        if (_rng.NextDouble() > 0.6) _leadDirection = -_leadDirection;
        int magnitude = _rng.NextDouble() < 0.75 ? 1 : 2;
        return _leadDirection * magnitude;
    }

    // ═══════════════════════ Boogie piano ═══════════════════════

    private void GenBoogiePiano(List<MidiEvent> ev, long t0, BarChords bar)
    {
        // Swung-8th boogie ostinato: R-3-5-6-b7-6-5-3 over the chord of the half-bar.
        for (int e = 0; e < 8; e++)
        {
            var chord = e >= 4 ? (bar.B ?? bar.A) : bar.A;
            int third = Theory.ChordTones(chord.Quality)[1];
            int degree = e switch { 0 => 0, 1 => third, 2 => 7, 3 => 9, 4 => 10, 5 => 9, 6 => 7, _ => third };
            int root = PlaceNear(chord.Root, 43, 36, 50);
            Note(ev, ChPiano, root + degree, Vel(46, 5), Beat(t0, Swing8(e * 0.5f)), (long)(0.42f * _spb));
        }
    }

    // ═══════════════════════ Helpers ═══════════════════════

    private static Chord ChordAt(BarChords bar, float beat)
        => beat >= 2f && bar.B is Chord b ? b : bar.A;

    /// <summary>Bar-relative beat → absolute sample time.</summary>
    private long Beat(long t0, float beat) => t0 + (long)(beat * _spb);

    /// <summary>Swing map: off-beat 8ths (x.5) shift late by up to 1/6 beat (full swing
    /// lands them on the triplet); everything else passes through.</summary>
    private float Swing8(float beat)
    {
        float frac = beat - MathF.Floor(beat);
        return MathF.Abs(frac - 0.5f) < 0.02f ? beat + _swing / 6f : beat;
    }

    /// <summary>Velocity with humanizing jitter, clamped to MIDI range.</summary>
    private int Vel(int v, int jitter) => Math.Clamp(v + _rng.Next(-jitter, jitter + 1), 1, 127);

    /// <summary>The key with the given pitch class nearest to <paramref name="near"/>,
    /// clamped into [lo, hi].</summary>
    private static int PlaceNear(int pitchClass, int near, int lo, int hi)
    {
        pitchClass = ((pitchClass % 12) + 12) % 12;
        int candidate = near - ((near - pitchClass) % 12 + 12) % 12;
        if (near - candidate > 6) candidate += 12;
        while (candidate < lo) candidate += 12;
        while (candidate > hi) candidate -= 12;
        return candidate;
    }

    /// <summary>Moves <paramref name="key"/> by <paramref name="steps"/> positions along
    /// the scale rooted at <paramref name="rootPc"/>, snapping to the scale first.</summary>
    private static int StepScale(int key, int steps, int rootPc, int[] scale)
    {
        int rel = ((key - rootPc) % 12 + 12) % 12;
        int idx = 0, bestDist = int.MaxValue;
        for (int i = 0; i < scale.Length; i++)
        {
            int d = Math.Abs(scale[i] - rel);
            if (d < bestDist) { bestDist = d; idx = i; }
        }
        int octave = (key - rootPc - rel) / 12;
        idx += steps;
        while (idx < 0) { idx += scale.Length; octave--; }
        while (idx >= scale.Length) { idx -= scale.Length; octave++; }
        return rootPc + octave * 12 + scale[idx];
    }

    /// <summary>Emits a note-on/note-off pair with ±4 ms timing humanization.</summary>
    private void Note(List<MidiEvent> ev, int channel, int key, int vel, long start, long dur)
    {
        if (key is < 0 or > 127) return;
        start += (long)((_rng.NextDouble() - 0.5) * 0.008 * _sampleRate);
        if (start < 0) start = 0;
        if (dur < 64) dur = 64;
        ev.Add(new MidiEvent { Time = start, Channel = (byte)channel, Command = 0x90, Data1 = (byte)key, Data2 = (byte)vel });
        ev.Add(new MidiEvent { Time = start + dur, Channel = (byte)channel, Command = 0x80, Data1 = (byte)key, Data2 = 0 });
    }

    private static void Cc(List<MidiEvent> ev, long t, int channel, int controller, int value)
        => ev.Add(new MidiEvent { Time = t, Channel = (byte)channel, Command = 0xB0, Data1 = (byte)controller, Data2 = (byte)value });

    private static void Program(List<MidiEvent> ev, long t, int channel, int program)
        => ev.Add(new MidiEvent { Time = t, Channel = (byte)channel, Command = 0xC0, Data1 = (byte)program, Data2 = 0 });
}
