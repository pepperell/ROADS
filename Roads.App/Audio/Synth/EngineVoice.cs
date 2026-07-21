using NAudio.Wave;
using Roads.App.Vehicles;

namespace Roads.App.Audio.Synth;

/// <summary>
/// One pooled per-vehicle engine tone: two detuned PolyBLEP saws (plus a sub-octave
/// square at 0.35 mix for trucks/buses) through a cascaded one-pole lowpass, with a slow
/// amplitude wobble for a "firing" feel, constant-power panned. The AudioEngine assigns
/// a vehicle and streams gain/pitch/pan/cutoff targets each frame; all are slewed per
/// sample so passes sweep the stereo field without zipper noise.
///
/// Anti-click contract: <see cref="TimbreType"/> and the per-assignment detune are only
/// consumed when <see cref="Retune"/> is observed, and the AudioEngine only sets Retune
/// while <see cref="IsQuiet"/> is true (the voice fully faded) AND with the new vehicle's
/// pitch/cutoff/pan targets already published, raising the gain target only afterwards —
/// so any callback either takes the quiet fast path or resets to the NEW state while
/// still silent; pitch jumps never happen audibly. An unassigned voice has TargetGain 0
/// and self-reports IsQuiet.
/// </summary>
public class EngineVoice : ISampleProvider
{
    /// <summary>Gain target; 0 releases the voice (150 ms), assignment fades in (80 ms).</summary>
    public float TargetGain;
    /// <summary>Fundamental pitch target in Hz (type base × speed × per-assignment detune).</summary>
    public float TargetPitchHz = 80f;
    /// <summary>Stereo pan target, -1..1 (screen-X of the vehicle × 0.8).</summary>
    public float TargetPan;
    /// <summary>Lowpass cutoff target in Hz (brightness from throttle).</summary>
    public float TargetCutoffHz = 800f;

    /// <summary>Set by the AudioEngine (with <see cref="TimbreType"/>/<see cref="Detune"/>
    /// already written) to make the voice adopt a new timbre; only set while quiet.</summary>
    public volatile bool Retune;
    /// <summary>VehicleType byte of the assigned vehicle (timbre selection on Retune).</summary>
    public byte TimbreType;
    /// <summary>Per-assignment constant detune factor (~±3%), chosen by the engine's RNG.</summary>
    public float Detune = 1f;

    /// <summary>True when the smoothed gain has decayed below audibility — the engine's
    /// gate for reassignment. Written by the playback thread, read by the UI thread.</summary>
    public volatile bool IsQuiet = true;

    private readonly WaveFormat _format;
    private PolyBlepOsc _osc1, _osc2, _subOsc;
    private OnePoleLp _lp1, _lp2;
    private ParamSmoother _gain, _pitch, _pan, _cutoff;
    private double _wobblePhase;
    private bool _hasSub;
    private float _wobbleDepth = 0.12f;

    public WaveFormat WaveFormat => _format;

    public EngineVoice(WaveFormat format)
    {
        _format = format;
        _pitch.Reset(80f);
        _cutoff.Reset(800f);
    }

    public int Read(float[] buffer, int offset, int count)
    {
        if (Retune)
        {
            // Adopt the new timbre while silent (the engine guarantees IsQuiet here).
            var type = (VehicleType)TimbreType;
            _hasSub = type is VehicleType.Truck or VehicleType.Bus;
            _wobbleDepth = type switch
            {
                VehicleType.Bus => 0.15f,
                VehicleType.Motorcycle => 0.20f,
                _ => 0.12f,
            };
            _pitch.Reset(TargetPitchHz);
            _cutoff.Reset(TargetCutoffHz);
            _pan.Reset(TargetPan);
            Retune = false;
        }

        float fs = _format.SampleRate;
        float gainTarget = TargetGain;
        float kGain = DspUtil.SlewCoeff(gainTarget > _gain.Value ? 0.08f : 0.15f, fs);
        float kPitch = DspUtil.SlewCoeff(0.08f, fs);
        float kPan = DspUtil.SlewCoeff(0.05f, fs);
        float kCutoffSlew = DspUtil.SlewCoeff(0.08f, fs);
        float pitchTarget = TargetPitchHz;
        float panTarget = TargetPan;
        float cutoffTarget = TargetCutoffHz;

        // Fully idle fast path: keep buffers zeroed without running the oscillators.
        if (gainTarget <= 0f && _gain.Value < 0.0005f)
        {
            Array.Clear(buffer, offset, count);
            IsQuiet = true;
            return count;
        }

        for (int i = 0; i < count; i += 2)
        {
            float pitch = _pitch.Next(pitchTarget, kPitch) * Detune;
            float freqNorm = pitch / fs;

            float sample = _osc1.NextSaw(freqNorm) + _osc2.NextSaw(freqNorm * 1.006f);
            if (_hasSub)
                sample += _subOsc.NextSquare(freqNorm * 0.5f) * 0.35f;
            sample *= 0.4f;

            float kLp = OnePoleLp.Coeff(_cutoff.Next(cutoffTarget, kCutoffSlew), fs);
            sample = _lp2.Next(_lp1.Next(sample, kLp), kLp);

            // Low-rate amplitude wobble at half the fundamental — a cheap "firing" feel.
            _wobblePhase += 2.0 * Math.PI * (pitch * 0.5) / fs;
            if (_wobblePhase > 2.0 * Math.PI) _wobblePhase -= 2.0 * Math.PI;
            float wobble = 1f + _wobbleDepth * (float)Math.Sin(_wobblePhase);

            float g = _gain.Next(gainTarget, kGain) * wobble;
            var (pl, pr) = DspUtil.ConstantPowerPan(_pan.Next(panTarget, kPan));

            buffer[offset + i] = sample * g * pl;
            if (i + 1 < count)
                buffer[offset + i + 1] = sample * g * pr;
        }

        IsQuiet = _gain.Value < 0.001f && gainTarget <= 0f;
        return count;
    }
}
