using NAudio.Wave;

namespace Roads.App.Audio.Synth;

/// <summary>
/// Final stage of the synth graph: pulls from the mixer, applies the smoothed master
/// gain (settings volume × sound-enabled) and the pause duck (glides everything to
/// silence while the sim is paused), then a per-sample tanh soft clip so a busy mix can
/// never hard-clip the device. Targets are plain floats written by the UI thread each
/// frame and slewed here per sample (the idiomatic NAudio live-control pattern —
/// 32-bit float writes are atomic; smoothing erases staleness).
/// </summary>
public class MasterProvider : ISampleProvider
{
    private readonly ISampleProvider _source;
    private ParamSmoother _master;
    private ParamSmoother _duck;

    /// <summary>Master gain target: settings volume, or 0 when sound is disabled.</summary>
    public float TargetMaster;

    /// <summary>Pause duck target: 0 while the sim is paused (TimeScale == 0), else 1.</summary>
    public float TargetDuck = 1f;

    public WaveFormat WaveFormat => _source.WaveFormat;

    public MasterProvider(ISampleProvider source)
    {
        _source = source;
        _master.Reset(0f);
        _duck.Reset(1f);
    }

    public int Read(float[] buffer, int offset, int count)
    {
        int read = _source.Read(buffer, offset, count);
        float fs = WaveFormat.SampleRate;
        float kMaster = DspUtil.SlewCoeff(0.05f, fs);
        // Duck slews slower down (release) than up (resume attack).
        float duckTarget = TargetDuck;
        float kDuck = DspUtil.SlewCoeff(duckTarget > _duck.Value ? 0.1f : 0.2f, fs);
        float masterTarget = TargetMaster;

        // Stereo interleaved: apply identical gain to the L/R pair so the image is stable.
        for (int i = 0; i < read; i += 2)
        {
            float g = _master.Next(masterTarget, kMaster) * _duck.Next(duckTarget, kDuck);
            buffer[offset + i] = MathF.Tanh(buffer[offset + i] * g);
            if (i + 1 < read)
                buffer[offset + i + 1] = MathF.Tanh(buffer[offset + i + 1] * g);
        }
        return read;
    }
}
