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
/// The symbolic music generator: a bar-at-a-time jazz composer in the SimCity 2000 /
/// Transport Tycoon lineage (smooth-jazz harmony over jump-blues bones). Runs entirely
/// on the NAudio playback thread — <see cref="MusicProvider"/> calls
/// <see cref="SetMood"/> + <see cref="GenerateBar"/> whenever the playhead reaches the
/// end of the composed material, so no locking exists anywhere. Randomness comes from a
/// private <see cref="Random"/>; nothing here may ever touch SimRandom or sim state
/// (the mood floats arrive as copies via SetMood).
///
/// Form: a state machine over three sections, chosen at each section boundary from the
/// slewed mood —
///   Blues    12-bar jazz-blues in Bb, swung; walking bass, EP comping, lead melody,
///            boogie piano layer at high intensity (the Transport Tycoon inheritance).
///   Vamp     8-bar Cm9↔F13 funk vamp (straighter 16ths, finger bass) — the bridge
///            after a few blues choruses, and the "tension" form under heavy congestion.
///   Nocturne 8-bar sus-plateau float (the SimCity 2000 new-age inheritance) — drums
///            fade out, warm pad swells, sparse flute/vibes lead. Chosen at night.
///
/// Mood inputs (0–1, slewed half-way per bar so changes land musically):
///   intensity — arrangement energy: gates drums/melody/piano layers and note density.
///   night     — crossfades toward the Nocturne palette (and picks the Nocturne form).
///   tension   — congestion: staccato comping, pedal bass, prefers the Vamp form.
/// </summary>
public sealed class Composer
{
    // ── MIDI channel plan ──
    public const int ChComp = 0;   // electric piano comping
    public const int ChBass = 1;   // acoustic/finger bass
    public const int ChLead = 2;   // rotating lead (sax/trumpet/vibes/flute)
    public const int ChPad = 3;    // warm pad (night float)
    public const int ChPiano = 4;  // boogie piano layer (high intensity)
    public const int ChDrums = 9;  // GM percussion

    private enum Section { Blues, Vamp, Nocturne }

    // ── Charts (Bb blues key; pitch classes: C=0 … Bb=10) ──
    private const int KeyRoot = 10; // Bb — blues-scale licks are rooted here

    private static readonly BarChords[] BluesChart =
    {
        new(new Chord(10, ChordQuality.Dom13)),
        new(new Chord(3, ChordQuality.Dom13)),
        new(new Chord(10, ChordQuality.Dom13)),
        new(new Chord(5, ChordQuality.Min9), new Chord(10, ChordQuality.Dom13)),
        new(new Chord(3, ChordQuality.Dom13)),
        new(new Chord(4, ChordQuality.Dim7)),
        new(new Chord(10, ChordQuality.Dom13)),
        new(new Chord(2, ChordQuality.Min7b5), new Chord(7, ChordQuality.Dom7Alt)),
        new(new Chord(0, ChordQuality.Min9)),
        new(new Chord(5, ChordQuality.Dom13)),
        new(new Chord(10, ChordQuality.Dom13), new Chord(7, ChordQuality.Dom7Alt)),
        new(new Chord(0, ChordQuality.Min9), new Chord(5, ChordQuality.Dom13)),
    };

    private static readonly BarChords[] VampChart =
    {
        new(new Chord(0, ChordQuality.Min9)),
        new(new Chord(5, ChordQuality.Dom13)),
        new(new Chord(0, ChordQuality.Min9)),
        new(new Chord(5, ChordQuality.Dom13)),
        new(new Chord(0, ChordQuality.Min9)),
        new(new Chord(5, ChordQuality.Dom13)),
        new(new Chord(0, ChordQuality.Min9)),
        new(new Chord(5, ChordQuality.Dom7Alt)),
    };

    private static readonly BarChords[] NocturneChart =
    {
        new(new Chord(10, ChordQuality.Maj9)),
        new(new Chord(7, ChordQuality.Min11)),
        new(new Chord(3, ChordQuality.Maj7Sh11)),
        new(new Chord(5, ChordQuality.Dom7Sus)),
        new(new Chord(10, ChordQuality.Maj9)),
        new(new Chord(8, ChordQuality.Maj7Sh11)),
        new(new Chord(7, ChordQuality.Min11)),
        new(new Chord(5, ChordQuality.Dom7Sus)),
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
    //    phrase-final long note) ──
    private static readonly float[][] MelodyTemplates =
    {
        new[] { 0.0f, 0.5f, 1.0f, 1.5f, 2.5f },
        new[] { 0.5f, 1.0f, 1.5f, 2.0f, 3.0f },
        new[] { 0.0f, 1.0f, 1.5f, 2.0f, 2.5f },
        new[] { 1.0f, 1.5f, 2.0f, 3.0f, 3.5f },
        new[] { 0.0f, 2.5f },                                       // sparse (night)
    };

    private readonly int _sampleRate;
    private readonly Random _rng;

    // ── Mood state (slewed toward SetMood targets once per bar) ──
    private float _intensity = 0.5f;
    private float _night;
    private float _tension;
    private float _tempoSetting = 96f;
    private float _swingSetting = 0.6f;

    private float _tempo = 96f;      // slewed BPM (max ±2.5 per bar — no jarring lurches)
    private float _swing;            // effective swing for the current bar
    private float _spb;              // samples per beat at the current tempo

    // ── Form state ──
    private Section _section = Section.Blues;
    private int _barInSection;
    private int _bluesRun;           // consecutive blues choruses since the last bridge
    private bool _setupEmitted;
    private int _leadProgram = -1;
    private int _bassProgram = -1;

    // ── Part state (voice-leading memory) ──
    private int _prevBassKey = 34;   // Bb1
    private int _prevCompTop = 65;
    private int _prevLeadKey = 70;
    private int _leadDirection = 1;
    private int _phraseBars;
    private int _restBars = 1;
    private int _barsGenerated;

    public Composer(int sampleRate, int seed)
    {
        _sampleRate = sampleRate;
        _rng = new Random(seed);
    }

    /// <summary>Slews the mood state half-way toward the given targets — called once per
    /// bar boundary, which quantizes mood changes to musical time.</summary>
    public void SetMood(float intensity, float night, float tension, float tempoBpm, float swing)
    {
        _intensity += (Math.Clamp(intensity, 0f, 1f) - _intensity) * 0.5f;
        _night += (Math.Clamp(night, 0f, 1f) - _night) * 0.5f;
        _tension += (Math.Clamp(tension, 0f, 1f) - _tension) * 0.5f;
        _tempoSetting = Math.Clamp(tempoBpm, 60f, 160f);
        _swingSetting = Math.Clamp(swing, 0f, 1f);
    }

    /// <summary>
    /// Composes one bar of music starting at <paramref name="t0"/> (absolute samples),
    /// appending events to <paramref name="ev"/> (unsorted — the provider sorts), and
    /// returns the bar length in samples.
    /// </summary>
    public long GenerateBar(List<MidiEvent> ev, long t0)
    {
        if (!_setupEmitted) { EmitSetup(ev, t0); _setupEmitted = true; }
        if (_barInSection == 0) BeginSection(ev, t0);

        // Tempo: night relaxes it, tension pushes it, slew ±2.5 BPM per bar.
        float tempoTarget = _tempoSetting * (1f - 0.10f * _night) * (1f + 0.05f * _tension);
        _tempo += Math.Clamp(tempoTarget - _tempo, -2.5f, 2.5f);
        _spb = _sampleRate * 60f / _tempo;
        long barLen = (long)(4f * _spb);

        // Swing: full in the blues, straighter in the funk vamp, nearly straight at night.
        _swing = _swingSetting * (_section == Section.Vamp ? 0.45f : 1f) * (1f - 0.7f * _night);

        var chart = CurrentChart;
        var bar = chart[_barInSection];
        var nextBar = chart[(_barInSection + 1) % chart.Length];
        bool lastBar = _barInSection == chart.Length - 1;

        // Layer levels from the mood.
        float drumLevel = Math.Clamp((_intensity - 0.12f) / 0.35f, 0f, 1f)
            * (1f - DspUtil.SmoothStep(0.35f, 0.75f, _night));
        float melodyAct = DspUtil.SmoothStep(0.35f, 0.7f, _intensity)
            * (1f - 0.5f * _tension) * (1f - 0.35f * _night);

        GenBass(ev, t0, bar, nextBar);
        GenComp(ev, t0, bar);
        GenPad(ev, t0, bar, barLen);
        if (drumLevel > 0.06f) GenDrums(ev, t0, drumLevel, lastBar);
        GenLead(ev, t0, bar, melodyAct);
        if (_section == Section.Blues && _intensity > 0.72f && _night < 0.3f)
            GenBoogiePiano(ev, t0, bar);

        _barInSection = (_barInSection + 1) % chart.Length;
        _barsGenerated++;
        return barLen;
    }

    private BarChords[] CurrentChart => _section switch
    {
        Section.Vamp => VampChart,
        Section.Nocturne => NocturneChart,
        _ => BluesChart,
    };

    // ═══════════════════════ Setup & form ═══════════════════════

    /// <summary>One-time channel setup: programs, volumes, pans, reverb/chorus sends.</summary>
    private void EmitSetup(List<MidiEvent> ev, long t)
    {
        Program(ev, t, ChComp, Theory.GmEPiano1);
        Program(ev, t, ChPad, Theory.GmWarmPad);
        Program(ev, t, ChPiano, Theory.GmAcousticGrand);

        Cc(ev, t, ChComp, 7, 84); Cc(ev, t, ChComp, 10, 52); Cc(ev, t, ChComp, 91, 28); Cc(ev, t, ChComp, 93, 30);
        Cc(ev, t, ChBass, 7, 105); Cc(ev, t, ChBass, 10, 64); Cc(ev, t, ChBass, 91, 12);
        Cc(ev, t, ChLead, 7, 88); Cc(ev, t, ChLead, 10, 76); Cc(ev, t, ChLead, 91, 42);
        Cc(ev, t, ChPad, 7, 72); Cc(ev, t, ChPad, 10, 64); Cc(ev, t, ChPad, 91, 70); Cc(ev, t, ChPad, 93, 40);
        Cc(ev, t, ChPiano, 7, 62); Cc(ev, t, ChPiano, 10, 40); Cc(ev, t, ChPiano, 91, 20);
        Cc(ev, t, ChDrums, 7, 96); Cc(ev, t, ChDrums, 91, 30);
    }

    /// <summary>Section boundary: pick the next form from the mood and rotate the lead
    /// instrument / bass patch (program changes land at the bar-line, before any note-on
    /// at the same sample thanks to the tie-rank sort).</summary>
    private void BeginSection(List<MidiEvent> ev, long t0)
    {
        Section prev = _section;
        if (_night > 0.6f) _section = Section.Nocturne;
        else if (_tension > 0.55f) _section = Section.Vamp;
        else if (prev == Section.Blues && _bluesRun >= 3) _section = Section.Vamp;
        else _section = Section.Blues;
        _bluesRun = _section == Section.Blues ? (prev == Section.Blues ? _bluesRun + 1 : 1) : 0;

        int[] leads = _night > 0.5f
            ? new[] { Theory.GmFlute, Theory.GmVibraphone }
            : new[] { Theory.GmAltoSax, Theory.GmMutedTrumpet, Theory.GmVibraphone, Theory.GmTenorSax };
        int pick = leads[_rng.Next(leads.Length)];
        if (pick == _leadProgram && leads.Length > 1)
            pick = leads[(Array.IndexOf(leads, pick) + 1) % leads.Length];
        if (pick != _leadProgram) { _leadProgram = pick; Program(ev, t0, ChLead, pick); }

        int bass = _section == Section.Vamp ? Theory.GmFingerBass : Theory.GmAcousticBass;
        if (bass != _bassProgram) { _bassProgram = bass; Program(ev, t0, ChBass, bass); }

        // Crash into the new section (skipped at night and on the cold open).
        if (_barsGenerated > 0 && _section != Section.Nocturne && _intensity > 0.3f)
            Note(ev, ChDrums, Theory.DrCrash, Vel(72, 6), t0, (long)(1.5f * _sampleRate * 60f / _tempo));
    }

    // ═══════════════════════ Bass ═══════════════════════

    private void GenBass(List<MidiEvent> ev, long t0, BarChords bar, BarChords nextBar)
    {
        switch (_section)
        {
            case Section.Nocturne:
            {
                int root = PlaceNear(bar.A.Root, _prevBassKey, 26, 45);
                Note(ev, ChBass, root, Vel(56, 4), Beat(t0, 0f), (long)(3.6f * _spb));
                _prevBassKey = root;
                if (_rng.NextDouble() < 0.4)
                    Note(ev, ChBass, root + 7, Vel(48, 4), Beat(t0, 2f), (long)(1.7f * _spb));
                return;
            }
            case Section.Vamp:
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
            default: // Blues: classic four-to-the-bar walking line.
            {
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
                    // Beat 3: scale tone stepping toward the coming approach note.
                    var scale = Theory.Scale(bar.A.Quality);
                    int approachTarget = PlaceNear(nextRootPc, b1, 26, 50);
                    int mid = (b1 + approachTarget) / 2;
                    b2 = PlaceNear((bar.A.Root + scale[_rng.Next(scale.Length)]) % 12, mid, 26, 50);
                }
                // Beat 4: chromatic approach into the next bar's root.
                int nextRootKey = PlaceNear(nextRootPc, b2, 26, 50);
                int b3 = nextRootKey + (_rng.Next(2) == 0 ? 1 : -1);

                Span<int> keys = stackalloc int[] { b0, b1, b2, b3 };
                for (int beat = 0; beat < 4; beat++)
                {
                    Note(ev, ChBass, keys[beat], Vel(beat == 0 ? 82 : 74, 5), Beat(t0, beat), (long)(0.92f * _spb));
                    // Occasional swung skip-note doubling the beat (the walking "skip").
                    if (_rng.NextDouble() < 0.10 && beat < 3)
                        Note(ev, ChBass, keys[beat], Vel(58, 5), Beat(t0, Swing8(beat + 0.5f)), (long)(0.3f * _spb));
                }
                _prevBassKey = b3;
                return;
            }
        }
    }

    // ═══════════════════════ Comping (electric piano) ═══════════════════════

    private void GenComp(List<MidiEvent> ev, long t0, BarChords bar)
    {
        if (_section == Section.Nocturne)
        {
            // Sparse: one soft voicing at the bar-line, half the time.
            if (_rng.NextDouble() < 0.55)
                CompHit(ev, Beat(t0, 0f), bar.A, (long)(3.5f * _spb), Vel(44, 4));
            return;
        }

        int patternIdx = _tension > 0.5f ? 4
            : _section == Section.Vamp ? 2 + _rng.Next(3)
            : _rng.Next(4);
        var pattern = CompPatterns[patternIdx];
        float density = 0.45f + 0.55f * _intensity;
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
        int rootBase = 48 + chord.Root % 12; // root reference C3–B3; shapes stack above
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

    // ═══════════════════════ Pad (night float) ═══════════════════════

    private void GenPad(List<MidiEvent> ev, long t0, BarChords bar, long barLen)
    {
        if (_night < 0.15f) return;
        var tones = Theory.ChordTones(bar.A.Quality);
        int root = 36 + bar.A.Root % 12;
        int vel = Vel((int)(28f + 36f * _night), 3);
        long dur = (long)(barLen * 1.02f); // slight overlap — slow pad attack bridges bars
        Note(ev, ChPad, root, vel, t0, dur);
        Note(ev, ChPad, root + 7, vel, t0, dur);
        Note(ev, ChPad, root + 12 + tones[1], vel, t0, dur);
        Note(ev, ChPad, root + 12 + tones[^1], vel, t0, dur);
        if (_night > 0.6f)
            Note(ev, ChPad, root + 26, Vel(vel - 6, 3), t0, dur); // 9th on top, deep night
    }

    // ═══════════════════════ Drums ═══════════════════════

    private void GenDrums(List<MidiEvent> ev, long t0, float level, bool lastBar)
    {
        int V(int v) => Vel(Math.Max(1, (int)(v * (0.45f + 0.55f * level))), 4);
        long tick = (long)(0.1f * _spb);

        if (_section == Section.Vamp)
        {
            // Funk: 16th hats with accents, syncopated kick, backbeat + ghost snare.
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
                if (_rng.NextDouble() < 0.4)
                    Note(ev, ChDrums, Theory.DrSnare, V(26), Beat(t0, b), tick);
            return;
        }

        // Swing: ride pattern + pedal hat, feathered kick, ghost snare.
        foreach (float b in stackalloc[] { 0f, 1f, 1.5f, 2f, 3f, 3.5f })
        {
            bool skip = b - MathF.Floor(b) > 0.01f;
            Note(ev, ChDrums, Theory.DrRide, V(skip ? 46 : 62), Beat(t0, Swing8(b)), tick);
        }
        if (_intensity > 0.8f && _barInSection % 4 == 0)
            Note(ev, ChDrums, Theory.DrRideBell, V(58), Beat(t0, 0f), tick);
        Note(ev, ChDrums, Theory.DrHatPedal, V(52), Beat(t0, 1f), tick);
        Note(ev, ChDrums, Theory.DrHatPedal, V(52), Beat(t0, 3f), tick);
        if (_intensity > 0.55f)
            for (int b = 0; b < 4; b++)
                Note(ev, ChDrums, Theory.DrKick, V(30), Beat(t0, b), tick);
        if (_rng.NextDouble() < 0.08)
            Note(ev, ChDrums, Theory.DrKick, V(70), Beat(t0, Swing8(2.5f)), tick); // dropped bomb
        for (int b = 0; b < 4; b++)
            if (_rng.NextDouble() < 0.25 * _intensity)
                Note(ev, ChDrums, Theory.DrSnare, V(24 + _rng.Next(10)), Beat(t0, Swing8(b + 0.5f)), tick);
        if (_intensity is > 0.3f and < 0.7f && _barInSection % 2 == 1)
            Note(ev, ChDrums, Theory.DrSideStick, V(55), Beat(t0, 3f), tick);

        if (lastBar && level > 0.4f)
        {
            // Section-turnaround fill: descending swung tom run into the next bar-line.
            Span<int> toms = stackalloc[] { Theory.DrTomHi, Theory.DrTomMid, Theory.DrTomLow };
            float[] fillBeats = { 2.5f, 3f, 3.5f };
            for (int i = 0; i < fillBeats.Length; i++)
                Note(ev, ChDrums, toms[i], V(70 + i * 6), Beat(t0, Swing8(fillBeats[i])), tick);
        }
    }

    // ═══════════════════════ Lead melody ═══════════════════════

    private void GenLead(List<MidiEvent> ev, long t0, BarChords bar, float activity)
    {
        // Phrase state machine: rest N bars, play a 1–2 bar phrase, rest again.
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

        var template = _night > 0.5f ? MelodyTemplates[4]
            : MelodyTemplates[_rng.Next(MelodyTemplates.Length - 1)];

        // Blues bars lean on the key blues scale ~35% of the time — the idiomatic licks;
        // otherwise the walk follows the chord-scale of the moment.
        bool bluesLick = _section == Section.Blues && _rng.NextDouble() < 0.35;

        int baseVel = _night > 0.5f ? 52 : 66;
        for (int i = 0; i < template.Length; i++)
        {
            float beat = template[i];
            bool longNote = finalBar && i == template.Length - 1;
            var chord = ChordAt(bar, beat);

            int key;
            if (longNote)
            {
                // Land on a color tone: 3rd, 9th, or 13th of the chord of the moment.
                var tones = Theory.ChordTones(chord.Quality);
                Span<int> colors = stackalloc[] { tones[1], 2, 9 };
                int bestKey = _prevLeadKey, bestDist = int.MaxValue;
                foreach (int c in colors)
                {
                    int k = PlaceNear((chord.Root + c) % 12, _prevLeadKey, 55, 79);
                    int d = Math.Abs(k - _prevLeadKey);
                    if (d < bestDist) { bestDist = d; bestKey = k; }
                }
                key = bestKey;
                // Grace note: quick chromatic crush from below into the held note.
                if (_rng.NextDouble() < 0.25)
                    Note(ev, ChLead, key - 1, Vel(baseVel - 20, 4), Beat(t0, Swing8(beat) - 0.09f), (long)(0.09f * _spb));
            }
            else
            {
                key = bluesLick
                    ? StepScale(_prevLeadKey, WalkStep(), KeyRoot, Theory.BluesScale)
                    : StepScale(_prevLeadKey, WalkStep(), chord.Root, Theory.Scale(chord.Quality));
                if (key > 79) key -= 12;
                if (key < 55) key += 12;
            }

            float arc = template.Length > 1 ? (float)i / (template.Length - 1) : 1f;
            int vel = Vel(baseVel + (int)(14f * arc) + (longNote ? 6 : 0), 6);
            float durBeats = longNote ? 4f - beat + 0.4f
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

    // ═══════════════════════ Boogie piano (Transport Tycoon layer) ═══════════════════════

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
        int candidate = near - ((near - pitchClass) % 12 + 12) % 12; // ≤ near, right pc
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
