using NAudio.Wave;

namespace Roads.App.Audio.Synth;

/// <summary>
/// The ambient traffic-hum bed: independent white noise per channel through a cascaded
/// one-pole lowpass pair, with two slow amplitude LFOs so the wash never reads as a
/// static loop, and a 20% mono-sum blend for center focus. The AudioEngine drives
/// <see cref="TargetGain"/> from vehicle density near the camera (× night × category
/// mix) and <see cref="TargetCutoffHz"/> from average speed (× night darkening); both
/// slew slowly (250/400 ms) so traffic changes swell rather than step.
/// </summary>
public class HumProvider : ISampleProvider
{
    /// <summary>Gain target (0..~0.2), written by the UI thread each frame.</summary>
    public float TargetGain;

    /// <summary>Lowpass cutoff target in Hz (~300-800), written by the UI thread.</summary>
    public float TargetCutoffHz = 500f;

    private readonly WaveFormat _format;
    private XorShiftNoise _noiseL = new(0x9E3779B9u);
    private XorShiftNoise _noiseR = new(0x7F4A7C15u);
    private OnePoleLp _lpL1, _lpL2, _lpR1, _lpR2;
    private ParamSmoother _gain;
    private ParamSmoother _cutoff;
    private double _lfoPhase1, _lfoPhase2;

    public WaveFormat WaveFormat => _format;

    public HumProvider(WaveFormat format)
    {
        _format = format;
        _cutoff.Reset(500f);
    }

    public int Read(float[] buffer, int offset, int count)
    {
        float fs = _format.SampleRate;
        float gainTarget = TargetGain;
        float cutoffTarget = TargetCutoffHz;

        // Idle gate (the MusicProvider freeze idiom): silent with no swell pending →
        // skip the noise/filter/LFO synthesis entirely. After sound-off the smoothed
        // gain otherwise decays toward zero and the whole per-sample loop runs at
        // denormal-assist speed indefinitely; frozen LFO phase is irrelevant in a
        // random wash.
        if (gainTarget <= 0f && _gain.Value < 1e-4f)
        {
            Array.Clear(buffer, offset, count);
            return count;
        }
        // Gain swells slower than it releases the other way (250 ms up / 400 ms down).
        float kGain = DspUtil.SlewCoeff(gainTarget > _gain.Value ? 0.25f : 0.4f, fs);
        float kCutoffSlew = DspUtil.SlewCoeff(0.25f, fs);
        double lfoInc1 = 2.0 * Math.PI * 0.08 / fs;
        double lfoInc2 = 2.0 * Math.PI * 0.013 / fs;

        for (int i = 0; i < count; i += 2)
        {
            float cutoff = _cutoff.Next(cutoffTarget, kCutoffSlew);
            float kLp = OnePoleLp.Coeff(cutoff, fs);

            _lfoPhase1 += lfoInc1;
            _lfoPhase2 += lfoInc2;
            if (_lfoPhase1 > 2.0 * Math.PI) _lfoPhase1 -= 2.0 * Math.PI;
            if (_lfoPhase2 > 2.0 * Math.PI) _lfoPhase2 -= 2.0 * Math.PI;
            float lfo = (1f + 0.15f * (float)Math.Sin(_lfoPhase1))
                      * (1f + 0.10f * (float)Math.Sin(_lfoPhase2));

            float g = _gain.Next(gainTarget, kGain) * lfo;

            float l = _lpL2.Next(_lpL1.Next(_noiseL.Next(), kLp), kLp);
            float r = _lpR2.Next(_lpR1.Next(_noiseR.Next(), kLp), kLp);

            // 20% mono-sum blend keeps the wash from feeling detached hard-left/right.
            float mono = (l + r) * 0.5f;
            buffer[offset + i] = (l * 0.8f + mono * 0.2f) * g;
            if (i + 1 < count)
                buffer[offset + i + 1] = (r * 0.8f + mono * 0.2f) * g;
        }
        return count;
    }
}
