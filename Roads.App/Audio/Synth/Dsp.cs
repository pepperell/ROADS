namespace Roads.App.Audio.Synth;

/// <summary>
/// Allocation-free DSP primitives shared by the synth voices. All are small mutable
/// structs owned by a single voice and advanced per sample on the NAudio playback
/// thread; none touch shared state. Filters and the parameter smoother flush denormals
/// (+1e-20f) so near-silent tails never fall into denormal-speed float math.
/// </summary>
public static class DspUtil
{
    /// <summary>One-pole slew coefficient for a time constant, precomputed once per buffer:
    /// <c>s += (target - s) * k</c> reaches ~63% of a step in <paramref name="tauSeconds"/>.</summary>
    public static float SlewCoeff(float tauSeconds, float sampleRate)
        => 1f - MathF.Exp(-1f / (MathF.Max(tauSeconds, 1e-4f) * sampleRate));

    /// <summary>Constant-power stereo pan: -1 = hard left, 0 = center, +1 = hard right.</summary>
    public static (float l, float r) ConstantPowerPan(float pan)
    {
        float angle = (Math.Clamp(pan, -1f, 1f) + 1f) * (MathF.PI / 4f);
        return (MathF.Cos(angle), MathF.Sin(angle));
    }

    /// <summary>Hermite smoothstep of <paramref name="x"/> between <paramref name="a"/> and <paramref name="b"/>.</summary>
    public static float SmoothStep(float a, float b, float x)
    {
        float t = Math.Clamp((x - a) / (b - a), 0f, 1f);
        return t * t * (3f - 2f * t);
    }
}

/// <summary>One-pole parameter smoother (slew limiter) — the anti-zipper for every
/// live-controlled gain/pitch/pan/cutoff. Advance per sample with a coefficient from
/// <see cref="DspUtil.SlewCoeff"/>. Flushes denormals like the filters: decaying toward
/// a zero target (gain after mute/duck) settles near +1e-20/k — normal-range float —
/// instead of grinding through denormal territory at microcode-assist speed forever.</summary>
public struct ParamSmoother
{
    private float _s;

    /// <summary>Current smoothed value without advancing.</summary>
    public readonly float Value => _s;

    /// <summary>Jumps the state directly (initialization only — audible if used live).</summary>
    public void Reset(float value) => _s = value;

    public float Next(float target, float k)
    {
        _s += (target - _s) * k + 1e-20f;
        return _s;
    }
}

/// <summary>One-pole lowpass (6 dB/oct). Coefficient k = SlewCoeff(1/(2π·fc), fs).</summary>
public struct OnePoleLp
{
    private float _s;

    public float Next(float x, float k)
    {
        _s += (x - _s) * k + 1e-20f;
        return _s;
    }

    /// <summary>Coefficient for a cutoff frequency in Hz.</summary>
    public static float Coeff(float cutoffHz, float sampleRate)
        => 1f - MathF.Exp(-2f * MathF.PI * Math.Clamp(cutoffHz, 10f, sampleRate * 0.45f) / sampleRate);
}

/// <summary>RBJ biquad bandpass (constant skirt gain). Recompute coefficients only when
/// the center frequency moves meaningfully (per buffer is fine for sweeps).</summary>
public struct BiquadBp
{
    private float _b0, _b1, _b2, _a1, _a2;
    private float _x1, _x2, _y1, _y2;

    public void SetBandpass(float centerHz, float q, float sampleRate)
    {
        float w0 = 2f * MathF.PI * Math.Clamp(centerHz, 20f, sampleRate * 0.45f) / sampleRate;
        float alpha = MathF.Sin(w0) / (2f * MathF.Max(q, 0.1f));
        float cosW0 = MathF.Cos(w0);
        float a0 = 1f + alpha;
        _b0 = alpha / a0;
        _b1 = 0f;
        _b2 = -alpha / a0;
        _a1 = -2f * cosW0 / a0;
        _a2 = (1f - alpha) / a0;
    }

    public float Next(float x)
    {
        float y = _b0 * x + _b1 * _x1 + _b2 * _x2 - _a1 * _y1 - _a2 * _y2 + 1e-20f;
        _x2 = _x1; _x1 = x;
        _y2 = _y1; _y1 = y;
        return y;
    }
}

/// <summary>Xorshift white-noise generator (own seed per voice — the audio path must never
/// touch SimRandom, which would break headless determinism).</summary>
public struct XorShiftNoise
{
    private uint _state;

    public XorShiftNoise(uint seed) => _state = seed == 0 ? 2463534242u : seed;

    /// <summary>Uniform white noise in [-1, 1).</summary>
    public float Next()
    {
        _state ^= _state << 13;
        _state ^= _state >> 17;
        _state ^= _state << 5;
        return _state * (2f / 4294967296f) - 1f;
    }
}

/// <summary>Anti-aliased sawtooth/square oscillator using PolyBLEP edge correction —
/// keeps engine tones from aliasing harshly as pitch sweeps with vehicle speed.</summary>
public struct PolyBlepOsc
{
    private float _phase; // 0..1

    /// <param name="freqNorm">Frequency as a fraction of the sample rate (f / fs).</param>
    public float NextSaw(float freqNorm)
    {
        _phase += freqNorm;
        if (_phase >= 1f) _phase -= 1f;
        float value = 2f * _phase - 1f;
        return value - PolyBlep(_phase, freqNorm);
    }

    /// <param name="freqNorm">Frequency as a fraction of the sample rate (f / fs).</param>
    public float NextSquare(float freqNorm)
    {
        _phase += freqNorm;
        if (_phase >= 1f) _phase -= 1f;
        float value = _phase < 0.5f ? 1f : -1f;
        value += PolyBlep(_phase, freqNorm);
        float fallPhase = _phase + 0.5f;
        if (fallPhase >= 1f) fallPhase -= 1f;
        value -= PolyBlep(fallPhase, freqNorm);
        return value;
    }

    /// <summary>Two-sample polynomial band-limited step correction around a discontinuity.</summary>
    private static float PolyBlep(float phase, float dt)
    {
        if (dt <= 0f) return 0f;
        if (phase < dt)
        {
            float t = phase / dt;
            return t + t - t * t - 1f;
        }
        if (phase > 1f - dt)
        {
            float t = (phase - 1f) / dt;
            return t * t + t + t + 1f;
        }
        return 0f;
    }
}
