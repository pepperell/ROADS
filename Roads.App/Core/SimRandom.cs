namespace Roads.App.Core;

/// <summary>
/// Process-wide pseudo-random source for ALL simulation-affecting draws (spawns, destinations,
/// reroutes, driver-personality traits, schedules, lane assignments, region-entry decisions,
/// signal phase offsets). Replaces direct use of <see cref="System.Random.Shared"/> so the entire
/// simulation can be made run-to-run reproducible by seeding once at startup.
///
/// Default behaviour matches the old <c>Random.Shared</c>: until <see cref="Seed"/> is called the
/// generator is time-seeded, so the GUI's traffic is non-repeating exactly as before. The headless
/// regression harness calls <see cref="Seed"/> with a fixed value so two identical
/// <c>--simtest</c> invocations draw the IDENTICAL sequence and therefore produce identical jam
/// clusters and exit codes — the property the harness's name/report depend on.
///
/// THREADING: the simulation is single-threaded (one substep loop). This wrapper is intentionally
/// NOT thread-safe, mirroring a per-thread <see cref="System.Random"/>; do not call it concurrently.
/// </summary>
public static class SimRandom
{
    private static Random _rng = new();

    /// <summary>
    /// Reseeds the simulation RNG with a fixed value, making every subsequent draw deterministic
    /// and reproducible. Call once, before any spawning/stepping, for a repeatable run. Calling
    /// this is what turns an otherwise timing-independent run into a fully reproducible one.
    /// </summary>
    public static void Seed(int seed) => _rng = new Random(seed);

    /// <summary>
    /// Reseeds the simulation RNG from fresh entropy (the parameterless <see cref="Random"/>
    /// constructor), discarding the current stream. The GUI calls this whenever a built-in
    /// world is loaded (title backdrop, New-game template) so each visit starts a genuinely
    /// new traffic sequence rather than continuing wherever the previous world left off.
    /// Never called by the headless harnesses — they need <see cref="Seed"/> determinism.
    /// </summary>
    public static void Reseed() => _rng = new Random();

    /// <summary>Returns a non-negative random integer less than <paramref name="maxExclusive"/>.</summary>
    public static int Next(int maxExclusive) => _rng.Next(maxExclusive);

    /// <summary>Returns a random integer in [<paramref name="minInclusive"/>, <paramref name="maxExclusive"/>).</summary>
    public static int Next(int minInclusive, int maxExclusive) => _rng.Next(minInclusive, maxExclusive);

    /// <summary>Returns a random double in [0.0, 1.0).</summary>
    public static double NextDouble() => _rng.NextDouble();

    /// <summary>Returns a random float in [0.0, 1.0).</summary>
    public static float NextSingle() => _rng.NextSingle();
}
