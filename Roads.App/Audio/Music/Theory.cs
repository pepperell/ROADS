namespace Roads.App.Audio.Music;

/// <summary>
/// Chord qualities used by the song charts. Each quality maps to a chord-scale (melody
/// and walking-bass material), a chord-tone set (bass/melody targets), and a pair of
/// rootless comping voicings (the Bill-Evans A/B forms — the comping generator picks
/// whichever form lands nearest the previous voicing's top note, which is all the voice
/// leading this style needs).
/// </summary>
public enum ChordQuality
{
    Maj9,
    Dom13,
    Dom7Alt,
    Min9,
    Min11,
    Min7b5,
    Dim7,
    Dom7Sus,
    Maj7Sh11,
}

/// <summary>A chord symbol: pitch-class root (0 = C … 11 = B) plus quality.</summary>
public readonly record struct Chord(int Root, ChordQuality Quality);

/// <summary>One bar of harmony: chord A for the whole bar, or A on beats 1–2 and B on 3–4.</summary>
public readonly record struct BarChords(Chord A, Chord? B = null);

/// <summary>
/// Static music-theory tables for the generative jazz engine: chord-scales, chord tones,
/// rootless voicings, General MIDI program numbers, and GM percussion notes. Pure data —
/// no state, no randomness.
/// </summary>
public static class Theory
{
    // ── Chord-scales (semitone offsets from the chord root, one octave) ──
    private static readonly int[] ScaleMajor = { 0, 2, 4, 5, 7, 9, 11 };
    private static readonly int[] ScaleLydian = { 0, 2, 4, 6, 7, 9, 11 };
    private static readonly int[] ScaleMixolydian = { 0, 2, 4, 5, 7, 9, 10 };
    private static readonly int[] ScaleDorian = { 0, 2, 3, 5, 7, 9, 10 };
    private static readonly int[] ScaleLocrianNat2 = { 0, 2, 3, 5, 6, 8, 10 };
    private static readonly int[] ScaleWholeHalf = { 0, 2, 3, 5, 6, 8, 9, 11 };
    private static readonly int[] ScaleAltered = { 0, 1, 3, 4, 6, 8, 10 };

    /// <summary>The blues scale (offsets from the KEY root, not the chord) — melody bars
    /// in the blues form draw from this for the idiomatic licks.</summary>
    public static readonly int[] BluesScale = { 0, 3, 5, 6, 7, 10 };

    public static int[] Scale(ChordQuality q) => q switch
    {
        ChordQuality.Maj9 => ScaleMajor,
        ChordQuality.Maj7Sh11 => ScaleLydian,
        ChordQuality.Dom13 or ChordQuality.Dom7Sus => ScaleMixolydian,
        ChordQuality.Min9 or ChordQuality.Min11 => ScaleDorian,
        ChordQuality.Min7b5 => ScaleLocrianNat2,
        ChordQuality.Dim7 => ScaleWholeHalf,
        ChordQuality.Dom7Alt => ScaleAltered,
        _ => ScaleMajor,
    };

    // ── Chord tones (bass beats 1–2, melody phrase targets) ──
    private static readonly int[] TonesMaj = { 0, 4, 7, 11 };
    private static readonly int[] TonesDom = { 0, 4, 7, 10 };
    private static readonly int[] TonesAlt = { 0, 4, 8, 10 };
    private static readonly int[] TonesMin = { 0, 3, 7, 10 };
    private static readonly int[] TonesHalfDim = { 0, 3, 6, 10 };
    private static readonly int[] TonesDim = { 0, 3, 6, 9 };
    private static readonly int[] TonesSus = { 0, 5, 7, 10 };

    public static int[] ChordTones(ChordQuality q) => q switch
    {
        ChordQuality.Maj9 or ChordQuality.Maj7Sh11 => TonesMaj,
        ChordQuality.Dom13 => TonesDom,
        ChordQuality.Dom7Alt => TonesAlt,
        ChordQuality.Min9 or ChordQuality.Min11 => TonesMin,
        ChordQuality.Min7b5 => TonesHalfDim,
        ChordQuality.Dim7 => TonesDim,
        ChordQuality.Dom7Sus => TonesSus,
        _ => TonesDom,
    };

    // ── Rootless comping voicings: two shapes per quality (A/B forms), semitone
    //    offsets from the root. The bass owns the root, so four upper-structure
    //    notes read as full 9th/13th chords. ──
    private static readonly int[][] VoiceMaj9 = { new[] { 4, 7, 11, 14 }, new[] { 11, 14, 16, 19 } };
    private static readonly int[][] VoiceDom13 = { new[] { 4, 10, 14, 21 }, new[] { 10, 14, 16, 21 } };
    private static readonly int[][] VoiceDom7Alt = { new[] { 4, 10, 13, 20 }, new[] { 10, 15, 16, 20 } };
    private static readonly int[][] VoiceMin9 = { new[] { 3, 7, 10, 14 }, new[] { 10, 14, 15, 19 } };
    private static readonly int[][] VoiceMin11 = { new[] { 3, 10, 14, 17 }, new[] { 10, 15, 17, 19 } };
    private static readonly int[][] VoiceMin7b5 = { new[] { 3, 6, 10, 12 }, new[] { 6, 10, 12, 15 } };
    private static readonly int[][] VoiceDim7 = { new[] { 3, 6, 9, 12 }, new[] { 6, 9, 12, 15 } };
    private static readonly int[][] VoiceDom7Sus = { new[] { 5, 10, 14, 19 }, new[] { 10, 14, 17, 19 } };
    private static readonly int[][] VoiceMaj7Sh11 = { new[] { 4, 7, 11, 18 }, new[] { 11, 16, 18, 21 } };

    public static int[][] Voicings(ChordQuality q) => q switch
    {
        ChordQuality.Maj9 => VoiceMaj9,
        ChordQuality.Dom13 => VoiceDom13,
        ChordQuality.Dom7Alt => VoiceDom7Alt,
        ChordQuality.Min9 => VoiceMin9,
        ChordQuality.Min11 => VoiceMin11,
        ChordQuality.Min7b5 => VoiceMin7b5,
        ChordQuality.Dim7 => VoiceDim7,
        ChordQuality.Dom7Sus => VoiceDom7Sus,
        ChordQuality.Maj7Sh11 => VoiceMaj7Sh11,
        _ => VoiceDom13,
    };

    // ── General MIDI program numbers (0-based) ──
    public const int GmAcousticGrand = 0;
    public const int GmEPiano1 = 4;       // the Rhodes patch — default comping
    public const int GmVibraphone = 11;
    public const int GmDrawbarOrgan = 16; // gospel-blues comping alternate
    public const int GmHarmonica = 22;    // the Transport Tycoon signature voice
    public const int GmJazzGuitar = 26;   // clean-electric comping alternate
    public const int GmAcousticBass = 32; // walking bass
    public const int GmFingerBass = 33;   // funk vamp bass
    public const int GmMutedTrumpet = 59;
    public const int GmBrassSection = 61; // solo-chorus guide-tone backgrounds
    public const int GmSopranoSax = 64;
    public const int GmAltoSax = 65;
    public const int GmTenorSax = 66;
    public const int GmClarinet = 71;
    public const int GmFlute = 73;
    public const int GmWarmPad = 89;      // night float bed

    /// <summary>GS drum-kit program on the percussion channel (bank 128): brushes.
    /// Presence in the loaded soundfont is verified at runtime (GeneralUser GS has it).</summary>
    public const int GmBrushKit = 40;

    // ── GM percussion notes (channel 9) ──
    public const int DrKick = 36;
    public const int DrSideStick = 37;
    public const int DrSnare = 38;
    public const int DrHatClosed = 42;
    public const int DrHatPedal = 44;
    public const int DrTomLow = 45;
    public const int DrHatOpen = 46;
    public const int DrTomMid = 47;
    public const int DrCrash = 49;
    public const int DrTomHi = 50;
    public const int DrRide = 51;
    public const int DrRideBell = 53;
    public const int DrShaker = 70;       // maracas — the bossa straight-8th bed
}
