using MeltySynth;
using NAudio.Wave;
using Roads.App.Audio.Synth;

namespace Roads.App.Audio.Music;

/// <summary>
/// The generative-music sample provider: bridges the bar-at-a-time <see cref="Composer"/>
/// to a MeltySynth SoundFont synthesizer, rendering stereo float audio for the NAudio
/// graph. Joins <see cref="MasterProvider"/> on its music bus — NOT the SFX mixer — so
/// the pause duck silences the diegetic world while the band plays on (the classic
/// city-builder behavior).
///
/// Sequencing: a sorted event queue with sample timestamps. Read() renders synth audio
/// up to the next due event (or bar boundary), dispatches it, and repeats; when the
/// playhead reaches the end of the composed material it asks the composer for one more
/// bar. Note-offs that overhang a bar merge with the next bar's events on the re-sort.
/// The composer therefore runs entirely on the playback thread; the UI thread only
/// writes the Target* floats (32-bit float writes are atomic; bar-boundary reads and
/// the gain slew erase staleness — the same idiom as the rest of the audio graph).
///
/// While faded fully out the timeline freezes (no composing, no rendering — zeros only),
/// so a disabled music setting costs nothing; on re-enable the band resumes mid-phrase.
///
/// DETERMINISM INVARIANT: same as the whole audio namespace — never touches SimRandom
/// or sim state. Mood arrives as plain floats from AudioEngine.Update.
/// </summary>
public sealed class MusicProvider : ISampleProvider
{
    private const int RenderChunk = 1024;

    private readonly Synthesizer _synth;
    private readonly Composer _composer;
    private readonly List<MidiEvent> _queue = new(512);
    private int _queueIdx;
    private long _pos;          // playhead, absolute samples on the music timeline
    private long _nextBarStart; // end of composed material
    private readonly float[] _left = new float[RenderChunk];
    private readonly float[] _right = new float[RenderChunk];
    private ParamSmoother _gain;
    private bool _silent = true;

    public WaveFormat WaveFormat { get; }

    // ── Targets (UI thread writes each frame; playback thread reads) ──
    /// <summary>Music bus gain: music-volume setting, or 0 when music/sound is disabled.</summary>
    public float TargetGain;
    /// <summary>Arrangement energy 0–1 (traffic density mapping).</summary>
    public float TargetIntensity = 0.5f;
    /// <summary>Night mood 0–1 (time-of-day darkness mapping).</summary>
    public float TargetNight;
    /// <summary>Congestion tension 0–1.</summary>
    public float TargetTension;
    /// <summary>Base tempo setting in BPM (night/tension modulate around it).</summary>
    public float TempoSetting = 96f;
    /// <summary>Swing feel setting 0–1 (0 = straight 8ths, 1 = full triplet swing).</summary>
    public float SwingSetting = 0.6f;

    /// <exception cref="Exception">Any SoundFont load/parse failure — the caller
    /// (AudioEngine) catches and runs without music.</exception>
    public MusicProvider(string soundFontPath, int sampleRate, int seed)
    {
        var settings = new SynthesizerSettings(sampleRate)
        {
            MaximumPolyphony = 64,
            EnableReverbAndChorus = true,
        };
        _synth = new Synthesizer(new SoundFont(soundFontPath), settings);
        _composer = new Composer(sampleRate, seed);
        WaveFormat = WaveFormat.CreateIeeeFloatWaveFormat(sampleRate, 2);
        _gain.Reset(0f);
    }

    public int Read(float[] buffer, int offset, int count)
    {
        Array.Clear(buffer, offset, count);
        float target = TargetGain;
        if (target <= 0f && _gain.Value < 1e-4f)
        {
            // Fully faded: kill hanging voices once, then freeze the timeline for free.
            if (!_silent) { _synth.NoteOffAll(false); _silent = true; }
            return count;
        }
        _silent = false;

        float k = DspUtil.SlewCoeff(0.15f, WaveFormat.SampleRate);
        int frames = count / 2;
        int done = 0;
        while (done < frames)
        {
            // Compose whenever the playhead reaches the end of the composed material —
            // note-offs overhanging the bar-line keep the queue non-empty, so gating
            // this on queue exhaustion would spin forever at the bar boundary.
            if (_pos >= _nextBarStart) ComposeBar();

            // Render up to the next due event, the bar boundary, or the buffer end.
            long limit = _pos + (frames - done);
            if (_queueIdx < _queue.Count) limit = Math.Min(limit, _queue[_queueIdx].Time);
            limit = Math.Min(limit, _nextBarStart);
            int n = (int)Math.Min(limit - _pos, RenderChunk);
            if (n > 0)
            {
                _synth.Render(_left.AsSpan(0, n), _right.AsSpan(0, n));
                int b = offset + done * 2;
                for (int i = 0; i < n; i++)
                {
                    float g = _gain.Next(target, k);
                    buffer[b + 2 * i] = _left[i] * g;
                    buffer[b + 2 * i + 1] = _right[i] * g;
                }
                _pos += n;
                done += n;
            }

            while (_queueIdx < _queue.Count && _queue[_queueIdx].Time <= _pos)
            {
                var e = _queue[_queueIdx++];
                _synth.ProcessMidiMessage(e.Channel, e.Command, e.Data1, e.Data2);
            }
        }
        return count;
    }

    /// <summary>Appends the next bar to the queue: compacts consumed events, snapshots
    /// the mood targets into the composer, and re-sorts (merging any note-offs that
    /// overhang from previous bars) with the tie-rank ordering program/CC → off → on.</summary>
    private void ComposeBar()
    {
        if (_queueIdx > 0)
        {
            _queue.RemoveRange(0, _queueIdx);
            _queueIdx = 0;
        }
        _composer.SetMood(TargetIntensity, TargetNight, TargetTension, TempoSetting, SwingSetting);
        long barLen = _composer.GenerateBar(_queue, _nextBarStart);
        _nextBarStart += Math.Max(barLen, 64);
        _queue.Sort(static (a, b) =>
        {
            int c = a.Time.CompareTo(b.Time);
            return c != 0 ? c : a.TieRank.CompareTo(b.TieRank);
        });
    }
}
