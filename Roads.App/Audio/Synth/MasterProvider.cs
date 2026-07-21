using NAudio.Wave;

namespace Roads.App.Audio.Synth;

/// <summary>
/// Final stage of the synth graph: pulls from the SFX mixer and (optionally) the music
/// bus, applies the smoothed master gain (settings volume × sound-enabled) and the pause
/// duck, then a per-sample tanh soft clip so a busy mix can never hard-clip the device.
/// The duck applies ONLY to the SFX bus — pausing freezes the diegetic world so its
/// sounds glide to silence, but the background band plays on (the classic city-builder
/// behavior); the master gain and soft clip cover both buses. Targets are plain floats
/// written by the UI thread each frame and slewed here per sample (the idiomatic NAudio
/// live-control pattern — 32-bit float writes are atomic; smoothing erases staleness).
/// </summary>
public class MasterProvider : ISampleProvider
{
    private readonly ISampleProvider _sfx;
    private readonly ISampleProvider? _music;
    private float[] _musicBuffer = Array.Empty<float>();
    private ParamSmoother _master;
    private ParamSmoother _duck;

    /// <summary>Master gain target: settings volume, or 0 when sound is disabled.</summary>
    public float TargetMaster;

    /// <summary>Pause duck target: 0 while the sim is paused (TimeScale == 0), else 1.</summary>
    public float TargetDuck = 1f;

    public WaveFormat WaveFormat => _sfx.WaveFormat;

    public MasterProvider(ISampleProvider sfx, ISampleProvider? music = null)
    {
        _sfx = sfx;
        _music = music;
        _master.Reset(0f);
        _duck.Reset(1f);
    }

    public int Read(float[] buffer, int offset, int count)
    {
        int read = _sfx.Read(buffer, offset, count);
        if (_music != null)
        {
            if (_musicBuffer.Length < count) _musicBuffer = new float[count];
            _music.Read(_musicBuffer, 0, count);
        }

        float fs = WaveFormat.SampleRate;
        float kMaster = DspUtil.SlewCoeff(0.05f, fs);
        // Duck slews slower down (release) than up (resume attack).
        float duckTarget = TargetDuck;
        float kDuck = DspUtil.SlewCoeff(duckTarget > _duck.Value ? 0.1f : 0.2f, fs);
        float masterTarget = TargetMaster;

        // Idle gate (the MusicProvider freeze idiom): fully muted → hand back silence
        // without the per-sample loop. Beyond skipping the loop, this is the denormal
        // guard for the SIGNAL path — at a ~1e-16 smoothed gain every product and tanh
        // intermediate below is denormal, paying assist penalties on every sample for
        // as long as sound stays off. Inputs were already read above, so upstream state
        // (and the music bus's own freeze) keeps advancing normally.
        if (masterTarget <= 0f && _master.Value < 1e-4f)
        {
            Array.Clear(buffer, offset, read);
            return read;
        }

        // Stereo interleaved: apply identical gain to the L/R pair so the image is stable.
        for (int i = 0; i < read; i += 2)
        {
            float g = _master.Next(masterTarget, kMaster);
            float d = _duck.Next(duckTarget, kDuck);
            float musL = _music != null ? _musicBuffer[i] : 0f;
            float musR = _music != null && i + 1 < read ? _musicBuffer[i + 1] : 0f;
            buffer[offset + i] = MathF.Tanh((buffer[offset + i] * d + musL) * g);
            if (i + 1 < read)
                buffer[offset + i + 1] = MathF.Tanh((buffer[offset + i + 1] * d + musR) * g);
        }
        return read;
    }
}
