namespace Roads.App.Vehicles;

/// <summary>
/// Named driver archetypes with distinct trait profiles.
/// </summary>
public enum DriverArchetype : byte
{
    Commuter,
    SundayDriver,
    LeadFoot,
    NervousNellie,
    Trucker,
}

/// <summary>
/// Generated personality traits for a single driver.
/// </summary>
public readonly struct DriverTraits
{
    public readonly DriverArchetype Archetype;
    public readonly float Aggressiveness;
    public readonly float SpeedBias;
    public readonly float ReactionTime;
    public readonly float SteeringSharpness;
    public readonly float BrakingComfort;
    public readonly float LaneChangeBias;
    public readonly float PatienceTimer;
    public readonly byte PreferredVehicle;

    public DriverTraits(DriverArchetype archetype, float aggressiveness, float speedBias,
        float reactionTime, float steeringSharpness, float brakingComfort,
        float laneChangeBias, float patienceTimer, byte preferredVehicle)
    {
        Archetype = archetype;
        Aggressiveness = aggressiveness;
        SpeedBias = speedBias;
        ReactionTime = reactionTime;
        SteeringSharpness = steeringSharpness;
        BrakingComfort = brakingComfort;
        LaneChangeBias = laneChangeBias;
        PatienceTimer = patienceTimer;
        PreferredVehicle = preferredVehicle;
    }
}

/// <summary>
/// Generates driver personality traits from named archetypes with Gaussian noise.
/// Each archetype defines mean trait values; individual drivers vary around those means.
/// </summary>
public static class DriverPersonalityGenerator
{
    // Archetype base values: (Aggressiveness, SpeedBias, ReactionTime, SteeringSharpness,
    //                         BrakingComfort, LaneChangeBias, PatienceTimer, PreferredVehicle)
    // PreferredVehicle is a mean — see VehicleWeights below for the per-archetype spread.
    private static readonly (float aggr, float speed, float react, float steer,
        float brake, float lane, float patience, byte vehicle)[] ArchetypeMeans =
    {
        // Commuter: average everything, moderate patience
        (0.4f, 1.0f, 0.6f, 1.0f, 2.5f, 0.5f, 30f, 0),
        // SundayDriver: low speed, high reaction time, very patient
        (0.1f, 0.85f, 1.0f, 0.7f, 2.0f, 0.15f, 120f, 0),
        // LeadFoot: high speed, high aggression, sharp steering
        (0.9f, 1.25f, 0.35f, 1.6f, 3.5f, 0.85f, 10f, 0),
        // NervousNellie: sharp steering, gentle braking, large following distance
        (0.15f, 0.95f, 0.5f, 1.5f, 1.8f, 0.2f, 60f, 0),
        // Trucker: low aggression, long reaction time, very patient
        (0.2f, 0.95f, 0.8f, 0.8f, 2.2f, 0.3f, 90f, 2),
    };

    // Per-archetype vehicle-type weighted selection.
    // Each row lists (VehicleType byte, weight) for that archetype.
    // Rows are indexed by (int)DriverArchetype.
    // Commuter    — sedan 50 %, SUV 35 %, motorcycle 15 %
    // SundayDriver— sedan 70 %, SUV 30 %
    // LeadFoot    — motorcycle 40 %, sedan 30 %, SUV 30 %
    // NervousNellie — sedan 80 %, SUV 20 %
    // Trucker     — truck 60 %, bus 40 %
    private static readonly (byte type, int weight)[][] VehicleWeights =
    {
        new[] { ((byte)0, 50), ((byte)1, 35), ((byte)4, 15) },   // Commuter
        new[] { ((byte)0, 70), ((byte)1, 30) },                   // SundayDriver
        new[] { ((byte)4, 40), ((byte)0, 30), ((byte)1, 30) },   // LeadFoot
        new[] { ((byte)0, 80), ((byte)1, 20) },                   // NervousNellie
        new[] { ((byte)2, 60), ((byte)3, 40) },                   // Trucker
    };

    // Weighted archetype selection: Commuter 50%, LeadFoot 15%, Trucker 15%,
    // SundayDriver 10%, NervousNellie 10%
    private static readonly (DriverArchetype archetype, int weight)[] ArchetypeWeights =
    {
        (DriverArchetype.Commuter, 50),
        (DriverArchetype.SundayDriver, 10),
        (DriverArchetype.LeadFoot, 15),
        (DriverArchetype.NervousNellie, 10),
        (DriverArchetype.Trucker, 15),
    };

    private static readonly int TotalWeight = ArchetypeWeights.Sum(w => w.weight);

    /// <summary>
    /// Generates a random driver personality by picking a weighted archetype
    /// and adding Gaussian noise to each trait.
    /// </summary>
    public static DriverTraits GenerateRandom()
    {
        var archetype = PickRandomArchetype();
        return Generate(archetype);
    }

    /// <summary>
    /// Generates traits for a specific archetype with Gaussian noise.
    /// </summary>
    public static DriverTraits Generate(DriverArchetype archetype)
    {
        var m = ArchetypeMeans[(int)archetype];

        return new DriverTraits(
            archetype,
            aggressiveness: Clamp(Gaussian(m.aggr, 0.1f), 0f, 1f),
            speedBias: Clamp(Gaussian(m.speed, 0.05f), 0.8f, 1.3f),
            reactionTime: Clamp(Gaussian(m.react, 0.1f), 0.3f, 1.2f),
            steeringSharpness: Clamp(Gaussian(m.steer, 0.15f), 0.5f, 2.0f),
            brakingComfort: Clamp(Gaussian(m.brake, 0.3f), 1.5f, 4.0f),
            laneChangeBias: Clamp(Gaussian(m.lane, 0.1f), 0f, 1f),
            patienceTimer: Clamp(Gaussian(m.patience, 15f), 5f, 180f),
            preferredVehicle: PickVehicleType((int)archetype)
        );
    }

    /// <summary>
    /// Picks a vehicle type byte for the given archetype index using the
    /// <see cref="VehicleWeights"/> table, so each archetype produces an
    /// archetype-plausible spread of vehicle types rather than a single fixed value.
    /// </summary>
    private static byte PickVehicleType(int archetypeIndex)
    {
        var weights = VehicleWeights[archetypeIndex];
        int total = 0;
        foreach (var (_, w) in weights) total += w;
        int roll = Random.Shared.Next(total);
        int cumulative = 0;
        foreach (var (type, w) in weights)
        {
            cumulative += w;
            if (roll < cumulative) return type;
        }
        return weights[0].type;
    }

    private static DriverArchetype PickRandomArchetype()
    {
        int roll = Random.Shared.Next(TotalWeight);
        int cumulative = 0;
        foreach (var (archetype, weight) in ArchetypeWeights)
        {
            cumulative += weight;
            if (roll < cumulative) return archetype;
        }
        return DriverArchetype.Commuter;
    }

    /// <summary>
    /// Box-Muller transform: generates a normally distributed sample.
    /// </summary>
    private static float Gaussian(float mean, float stddev)
    {
        float u1 = 1f - (float)Random.Shared.NextDouble(); // (0, 1]
        float u2 = (float)Random.Shared.NextDouble();
        float z = MathF.Sqrt(-2f * MathF.Log(u1)) * MathF.Cos(2f * MathF.PI * u2);
        return mean + stddev * z;
    }

    private static float Clamp(float value, float min, float max)
        => MathF.Max(min, MathF.Min(max, value));
}
