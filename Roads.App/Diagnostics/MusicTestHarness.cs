using System.Text;
using NAudio.Wave;
using Roads.App.Audio.Music;

namespace Roads.App.Diagnostics;

/// <summary>
/// Headless music-engine verification (<c>--musictest[=seconds]</c>): renders the
/// generative jazz engine offline through the same <see cref="MusicProvider"/> path the
/// app plays live, sweeping four mood presets (calm day → busy day → gridlock → night),
/// and writes the result to a WAV (default <c>musictest.wav</c> — listen to audition the
/// band) plus a small report with per-phase RMS levels. Exit codes: 0 = every phase
/// produced audio, 1 = a phase rendered silent (composer or synth regression),
/// 2 = the bundled soundfont is missing. Fixed seed — reruns are byte-comparable.
/// </summary>
public static class MusicTestHarness
{
    private const int SampleRate = 44100;
    private const float SilenceRmsFloor = 1e-3f;

    public static int Run(float seconds, string outPath)
    {
        var report = new StringBuilder();
        string reportPath = Path.ChangeExtension(outPath, ".log");
        try
        {
            return RunCore(seconds, outPath, report, reportPath);
        }
        catch (Exception ex)
        {
            // A diagnostic harness must never die silently (WinExe has no console).
            report.AppendLine($"musictest CRASHED: {ex}");
            try { File.WriteAllText(reportPath, report.ToString()); } catch { }
            return 3;
        }
    }

    private static int RunCore(float seconds, string outPath, StringBuilder report, string reportPath)
    {
        string soundFontPath = Path.Combine(AppContext.BaseDirectory, "Assets", "GeneralUser-GS.sf2");
        if (!File.Exists(soundFontPath))
        {
            report.AppendLine($"musictest FAILED: soundfont not found at {soundFontPath}");
            File.WriteAllText(reportPath, report.ToString());
            return 2;
        }

        var music = new MusicProvider(soundFontPath, SampleRate, seed: 20260716)
        {
            TargetGain = 0.85f,
        };

        var phases = new (string Name, float Intensity, float Night, float Tension)[]
        {
            ("day-calm", 0.30f, 0f, 0f),
            ("day-busy", 0.95f, 0f, 0f),
            ("gridlock", 0.70f, 0f, 0.9f),
            ("night", 0.35f, 1f, 0f),
        };

        report.AppendLine($"musictest: {seconds:F0}s total, {phases.Length} mood phases -> {outPath}");
        bool ok = true;
        using (var writer = new WaveFileWriter(outPath, music.WaveFormat))
        {
            var buffer = new float[SampleRate / 10 * 2]; // 100 ms chunks
            int framesPerPhase = (int)(SampleRate * seconds / phases.Length);
            foreach (var phase in phases)
            {
                music.TargetIntensity = phase.Intensity;
                music.TargetNight = phase.Night;
                music.TargetTension = phase.Tension;

                double sumSquares = 0;
                long sampleCount = 0;
                int remaining = framesPerPhase;
                while (remaining > 0)
                {
                    int frames = Math.Min(remaining, buffer.Length / 2);
                    int samples = music.Read(buffer, 0, frames * 2);
                    writer.WriteSamples(buffer, 0, samples);
                    for (int i = 0; i < samples; i++) sumSquares += buffer[i] * buffer[i];
                    sampleCount += samples;
                    remaining -= frames;
                }

                double rms = Math.Sqrt(sumSquares / Math.Max(1, sampleCount));
                bool phaseOk = rms >= SilenceRmsFloor;
                ok &= phaseOk;
                report.AppendLine($"  {phase.Name,-9} RMS {rms:F4}  {(phaseOk ? "ok" : "SILENT")}");
            }
        }

        report.AppendLine(ok ? "musictest OK" : "musictest FAILED (silent phase)");
        File.WriteAllText(reportPath, report.ToString());
        Console.WriteLine(report.ToString());
        return ok ? 0 : 1;
    }
}
