using NAudio.Wave;

namespace Roads.App.Audio.Synth;

/// <summary>Which one-shot sound a <see cref="OneShotVoice"/> plays when triggered.</summary>
public enum OneShotKind : byte
{
    /// <summary>Two-tone car horn (deadlock-breaker "jam freed" event).</summary>
    Horn,
    /// <summary>Bandpass-swept noise brake screech (hard braking at speed).</summary>
    Screech,
    /// <summary>Very short quiet click (visible traffic-signal phase onset).</summary>
    Tick,
}

/// <summary>
/// One pooled one-shot voice. The AudioEngine writes <see cref="Kind"/>, gain, pan, and
/// the per-trigger randoms, THEN increments <see cref="TriggerSeq"/>; the playback thread
/// notices the sequence change and restarts the envelope FROM ITS CURRENT OUTPUT LEVEL
/// (never from zero), so retriggering a still-sounding voice cannot click.
/// <see cref="IsFree"/> reports when the envelope has fully decayed.
/// </summary>
public class OneShotVoice : ISampleProvider
{
    // ── Trigger parameters (written by the UI thread before bumping TriggerSeq) ──
    public OneShotKind Kind;
    /// <summary>Peak gain including distance attenuation and category mix.</summary>
    public float Gain;
    /// <summary>Stereo pan, -1..1.</summary>
    public float Pan;
    /// <summary>Per-trigger random 0..1 (horn pitch offset / screech duration).</summary>
    public float Param1;
    /// <summary>Second per-trigger random 0..1 (horn hold length / screech sweep rate).</summary>
    public float Param2;

    /// <summary>Trigger gate: UI thread increments after writing parameters.</summary>
    public volatile int TriggerSeq;

    /// <summary>True when the envelope has fully decayed (voice reusable).</summary>
    public volatile bool IsFree = true;

    private readonly WaveFormat _format;
    private int _lastSeq;

    // Envelope state: 0 attack, 1 hold, 2 release, 3 done.
    private int _stage = 3;
    private float _env;
    private float _stageSamplesLeft;
    private float _attackStep, _releaseCoeff;
    private float _holdSamples;

    // Kind-specific synth state.
    private OneShotKind _activeKind;
    private PolyBlepOsc _hornOsc1, _hornOsc2;
    private float _hornFreq1, _hornFreq2;
    private OnePoleLp _hornLp;
    private XorShiftNoise _noise = new(0xB5297A4Du);
    private BiquadBp _screechBp;
    private float _screechCenter, _screechRate;
    private int _coeffRefreshCounter;
    private double _flutterPhase, _tickPhase;
    private float _tickEnv;
    private ParamSmoother _panSmooth;
    private float _gain;

    public WaveFormat WaveFormat => _format;

    public OneShotVoice(WaveFormat format)
    {
        _format = format;
    }

    public int Read(float[] buffer, int offset, int count)
    {
        float fs = _format.SampleRate;

        int seq = TriggerSeq;
        if (seq != _lastSeq)
        {
            _lastSeq = seq;
            StartTrigger(fs);
        }

        if (_stage == 3)
        {
            Array.Clear(buffer, offset, count);
            IsFree = true;
            return count;
        }

        float kPan = DspUtil.SlewCoeff(0.05f, fs);
        for (int i = 0; i < count; i += 2)
        {
            AdvanceEnvelope();
            float sample = _activeKind switch
            {
                OneShotKind.Horn => NextHorn(fs),
                OneShotKind.Screech => NextScreech(fs),
                _ => NextTick(fs),
            };
            float value = sample * _env * _gain;
            var (pl, pr) = DspUtil.ConstantPowerPan(_panSmooth.Next(Pan, kPan));
            buffer[offset + i] = value * pl;
            if (i + 1 < count)
                buffer[offset + i + 1] = value * pr;

            if (_stage == 3)
            {
                // Envelope finished mid-buffer: zero the remainder and stop synthesis.
                for (int j = i + 2; j < count; j++)
                    buffer[offset + j] = 0f;
                break;
            }
        }

        IsFree = _stage == 3;
        return count;
    }

    /// <summary>Latches the trigger parameters into playback-thread state. The envelope
    /// restarts from the CURRENT level (attack ramps from wherever _env is).</summary>
    private void StartTrigger(float fs)
    {
        _activeKind = Kind;
        _gain = Gain;
        _panSmooth.Reset(Pan);
        IsFree = false;
        _stage = 0;

        switch (_activeKind)
        {
            case OneShotKind.Horn:
                // Two-tone squares, ±4% random pitch per trigger.
                float pitchScale = 1f + (Param1 - 0.5f) * 0.08f;
                _hornFreq1 = 420f * pitchScale;
                _hornFreq2 = 520f * pitchScale;
                _attackStep = 1f / (0.008f * fs);
                _holdSamples = (0.25f + 0.15f * Param2) * fs;
                _releaseCoeff = DspUtil.SlewCoeff(0.06f, fs);
                break;

            case OneShotKind.Screech:
                _screechCenter = 2400f;
                // Sweep 2.4 -> 1.2 kHz over the note (duration 400-700 ms via Param1).
                float duration = 0.4f + 0.3f * Param1;
                _screechRate = MathF.Pow(0.5f, 1f / (duration * fs)); // per-sample decay to half
                _attackStep = 1f / (0.015f * fs);
                _holdSamples = duration * fs;
                _releaseCoeff = DspUtil.SlewCoeff(0.15f, fs);
                _screechBp.SetBandpass(_screechCenter, 4f, fs);
                break;

            default: // Tick
                _tickPhase = 0.0;
                _tickEnv = 1f;
                _attackStep = 1f / (0.002f * fs);
                _holdSamples = 0.01f * fs;
                _releaseCoeff = DspUtil.SlewCoeff(0.025f, fs);
                break;
        }
        _stageSamplesLeft = _holdSamples;
    }

    private void AdvanceEnvelope()
    {
        switch (_stage)
        {
            case 0: // attack (ramps from current level — click-free retrigger)
                _env += _attackStep;
                if (_env >= 1f) { _env = 1f; _stage = 1; }
                break;
            case 1: // hold
                _stageSamplesLeft -= 1f;
                if (_stageSamplesLeft <= 0f) _stage = 2;
                break;
            case 2: // release
                _env += (0f - _env) * _releaseCoeff;
                if (_env < 0.0005f) { _env = 0f; _stage = 3; }
                break;
        }
    }

    private float NextHorn(float fs)
    {
        float sample = _hornOsc1.NextSquare(_hornFreq1 / fs) + _hornOsc2.NextSquare(_hornFreq2 / fs);
        float kLp = OnePoleLp.Coeff(1800f, fs);
        return _hornLp.Next(sample * 0.5f, kLp);
    }

    private float NextScreech(float fs)
    {
        // Exponential center sweep down to 1.2 kHz; coefficients refresh every 32 samples
        // (recomputing the biquad per sample would be needless trig).
        _screechCenter = MathF.Max(_screechCenter * _screechRate, 1200f);
        if (++_coeffRefreshCounter >= 32)
        {
            _coeffRefreshCounter = 0;
            _screechBp.SetBandpass(_screechCenter, 4f, fs);
        }

        _flutterPhase += 2.0 * Math.PI * 30.0 / fs;
        if (_flutterPhase > 2.0 * Math.PI) _flutterPhase -= 2.0 * Math.PI;
        float flutter = 1f + 0.3f * (float)Math.Sin(_flutterPhase);

        return _screechBp.Next(_noise.Next()) * flutter * 2.5f;
    }

    private float NextTick(float fs)
    {
        // 1 kHz sine ping with a fast-decaying noise transient on top.
        _tickPhase += 2.0 * Math.PI * 1000.0 / fs;
        if (_tickPhase > 2.0 * Math.PI) _tickPhase -= 2.0 * Math.PI;
        _tickEnv *= 0.998f;
        float noiseBurst = _env > 0.9f ? _noise.Next() * 0.4f : 0f;
        return ((float)Math.Sin(_tickPhase) * 0.8f + noiseBurst) * _tickEnv;
    }
}
