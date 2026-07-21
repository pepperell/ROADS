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

    /// <summary>Default composer seed: fixed so plain <c>--musictest</c> reruns are
    /// byte-comparable. Pass <c>--musicseed=&lt;n&gt;</c> to sweep seeds (the GUI seeds
    /// per-launch with TickCount, so seed-dependent audio bugs need a sweep to catch).</summary>
    private const int DefaultSeed = 20260716;

    public static int Run(float seconds, string outPath, int seed = DefaultSeed)
    {
        var report = new StringBuilder();
        string reportPath = Path.ChangeExtension(outPath, ".log");
        try
        {
            return RunCore(seconds, outPath, report, reportPath, seed);
        }
        catch (Exception ex)
        {
            // A diagnostic harness must never die silently (WinExe has no console).
            report.AppendLine($"musictest CRASHED: {ex}");
            try { File.WriteAllText(reportPath, report.ToString()); } catch { }
            return 3;
        }
    }

    private static int RunCore(float seconds, string outPath, StringBuilder report, string reportPath, int seed)
    {
        string soundFontPath = Path.Combine(AppContext.BaseDirectory, "Assets", "GeneralUser-GS.sf2");
        if (!File.Exists(soundFontPath))
        {
            report.AppendLine($"musictest FAILED: soundfont not found at {soundFontPath}");
            File.WriteAllText(reportPath, report.ToString());
            return 2;
        }

        var music = new MusicProvider(soundFontPath, SampleRate, seed)
        {
            TargetGain = 0.85f,
        };

        // Mood sweep: hour steers the setlist repertoire, ambience simulates far zoom,
        // the resolution phase fires the jam-cleared cadence, and the night phase bumps
        // the day number to force a tune rotation on top of the darkness shift.
        var phases = new (string Name, float I, float N, float T, float Hour, float Amb, bool Resolve, int Day)[]
        {
            ("morning-rush", 0.90f, 0f, 0f, 8f, 0f, false, 0),
            ("midday-calm", 0.40f, 0f, 0f, 12f, 0.2f, false, 0),
            ("evening-jam", 0.70f, 0f, 0.9f, 17f, 0f, false, 0),
            ("resolution", 0.60f, 0f, 0f, 17f, 0f, true, 0),
            ("night", 0.35f, 1f, 0f, 23f, 0.4f, false, 1),
        };

        report.AppendLine($"musictest: {seconds:F0}s total, {phases.Length} mood phases -> {outPath}");
        report.AppendLine($"  brush kit (bank 128 patch 40): {(music.BrushKitAvailable ? "present" : "MISSING - standard kit fallback")}");
        bool ok = true;
        using (var writer = new WaveFileWriter(outPath, music.WaveFormat))
        {
            var buffer = new float[SampleRate / 10 * 2]; // 100 ms chunks
            int framesPerPhase = (int)(SampleRate * seconds / phases.Length);
            foreach (var phase in phases)
            {
                music.TargetIntensity = phase.I;
                music.TargetNight = phase.N;
                music.TargetTension = phase.T;
                music.TargetHour = phase.Hour;
                music.TargetAmbience = phase.Amb;
                music.TargetDayNumber = phase.Day;
                if (phase.Resolve) music.ResolutionSeq++;

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
                report.AppendLine($"  {phase.Name,-12} RMS {rms:F4}  tune {music.CurrentTune,-14} {(phaseOk ? "ok" : "SILENT")}");
            }
        }

        report.AppendLine($"  tunes started: {music.TunesStarted}");
        // Long renders must actually rotate the setlist (tune budgets are 4-6 min).
        if (seconds >= 360 && music.TunesStarted < 2)
        {
            ok = false;
            report.AppendLine("  FAILED: no tune rotation in a long render");
        }

        report.AppendLine(ok ? "musictest OK" : "musictest FAILED");
        File.WriteAllText(reportPath, report.ToString());
        Console.WriteLine(report.ToString());
        return ok ? 0 : 1;
    }
}
