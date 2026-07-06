using System;

namespace Roads.App.Core;

/// <summary>
/// Tracks in-game time of day (0.0–24.0 hours). Each sim tick advances game time
/// at a configurable ratio (default: game time runs at realtime at 1x speed).
/// </summary>
public class SimulationClock
{
    /// <summary>Game-seconds that pass per real second at 1x speed (1.0 = realtime).</summary>
    private const double GameSecondsPerRealSecond = 1.0;

    /// <summary>Current time of day in fractional hours (0.0 = midnight, 12.0 = noon).</summary>
    public double TimeOfDay { get; set; } = 8.0;

    /// <summary>Day counter, incremented each midnight rollover.</summary>
    public int DayNumber { get; set; }

    /// <summary>Advances the clock by one simulation timestep. Returns true on midnight rollover.</summary>
    public bool Advance(float simDt)
    {
        // Each tick advances by simDt real-seconds worth of game time.
        // GameSecondsPerRealSecond / 3600 converts to hours.
        TimeOfDay += simDt * GameSecondsPerRealSecond / 3600.0;
        bool rolledOver = false;
        if (TimeOfDay >= 24.0)
        {
            TimeOfDay -= 24.0;
            DayNumber++;
            rolledOver = true;
        }
        if (TimeOfDay < 0.0) TimeOfDay += 24.0;
        return rolledOver;
    }

    /// <summary>Returns "HH:MM:SS" formatted game time.</summary>
    public string GetDisplayTime()
    {
        int totalSeconds = (int)(TimeOfDay * 3600.0);
        int hours = totalSeconds / 3600;
        int minutes = totalSeconds / 60 % 60;
        int seconds = totalSeconds % 60;
        return $"{hours:D2}:{minutes:D2}:{seconds:D2}";
    }

    /// <summary>
    /// Darkness factor: 0 = full daylight, 1 = full night.
    /// Dawn (5–7): transitions 1→0. Day (7–18): 0. Dusk (18–20): transitions 0→1. Night (20–5): 1.
    /// </summary>
    public float Darkness
    {
        get
        {
            double t = TimeOfDay;
            if (t >= 7.0 && t <= 18.0) return 0f;
            if (t >= 20.0 || t <= 5.0) return 1f;
            if (t > 5.0 && t < 7.0) return (float)(1.0 - (t - 5.0) / 2.0);
            // 18 < t < 20: dusk
            return (float)((t - 18.0) / 2.0);
        }
    }
}
