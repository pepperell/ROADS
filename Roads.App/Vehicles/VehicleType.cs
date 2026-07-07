namespace Roads.App.Vehicles;

/// <summary>
/// Identifies the visual class of a vehicle for rendering purposes.
/// Values are intentionally byte-compatible with <see cref="DriverTraits.PreferredVehicle"/>
/// so the SoA array can be cast directly.
/// Truck = 2 aligns with the existing Trucker archetype mean.
/// </summary>
public enum VehicleType : byte
{
    Sedan      = 0,
    SUV        = 1,
    Truck      = 2,
    Bus        = 3,
    Motorcycle = 4,
}

/// <summary>
/// Provides per-type vehicle dimensions, used by BOTH the renderer (body rectangles) and
/// the simulation: car-following gaps, stop-line offsets, overlap detection, lane-change
/// fit checks, and the kinematic wheelbase all consume these via the byte-indexed helpers
/// (<see cref="GetLength"/>, <see cref="GetHalfLength"/>, <see cref="GetWheelbase"/>).
/// <see cref="SimConstants.VehicleLength"/> / <see cref="SimConstants.VehicleWidth"/> are
/// the Sedan baseline only; Sedan dimensions exactly match them so uniform-sedan traffic
/// behaves identically to the former constant-size model.
/// Acceleration also varies by type — see <see cref="VehicleTypeDynamics"/>.
/// </summary>
public static class VehicleTypeDimensions
{
    // (length, width) in metres — Sedan must equal SimConstants values.
    private static readonly (float Length, float Width)[] _dims =
    {
        (4.5f,  2.0f),   // Sedan      — matches SimConstants exactly
        (4.9f,  2.15f),  // SUV        — slightly taller/wider footprint
        (8.5f,  2.45f),  // Truck      — articulated cab, visually long
        (12.0f, 2.55f),  // Bus        — full-size city bus
        (2.2f,  0.85f),  // Motorcycle — narrow, short
    };

    /// <summary>
    /// Wheelbase as a fraction of body length (Sedan: 4.5 m × 0.556 = 2.5 m, the former
    /// global constant, so sedan handling is unchanged). Real vehicles cluster near this
    /// ratio; a single fraction keeps the kinematics simple and monotonic in length.
    /// </summary>
    private const float WheelbaseFraction = 0.556f;

    /// <summary>Half-length of the longest vehicle type (metres). Upper bound for spatial
    /// query radii that must not miss any potential body overlap regardless of the other
    /// vehicle's type.</summary>
    public static readonly float MaxHalfLength = _dims.Max(d => d.Length) * 0.5f;

    /// <summary>
    /// Returns the (length, width) for the given <paramref name="type"/>.
    /// Out-of-range values are clamped to <see cref="VehicleType.Sedan"/>.
    /// </summary>
    public static (float Length, float Width) GetDimensions(VehicleType type)
    {
        int idx = (int)type;
        if ((uint)idx >= (uint)_dims.Length) idx = 0;
        return _dims[idx];
    }

    /// <summary>Body length in metres for a raw <see cref="VehicleStore.PreferredVehicle"/>
    /// byte. Out-of-range values are clamped to Sedan.</summary>
    public static float GetLength(byte type)
    {
        int idx = type;
        if ((uint)idx >= (uint)_dims.Length) idx = 0;
        return _dims[idx].Length;
    }

    /// <summary>Half the body length in metres (bumper-to-center distance) for a raw
    /// <see cref="VehicleStore.PreferredVehicle"/> byte. The building block for
    /// center-to-center → bumper-to-bumper gap conversion.</summary>
    public static float GetHalfLength(byte type) => GetLength(type) * 0.5f;

    /// <summary>Front-to-rear axle distance in metres for a raw
    /// <see cref="VehicleStore.PreferredVehicle"/> byte, used by the bicycle-model
    /// kinematics (<see cref="VehiclePhysics"/>).</summary>
    public static float GetWheelbase(byte type) => GetLength(type) * WheelbaseFraction;
}

/// <summary>
/// Per-type driving dynamics: the full-throttle acceleration capability of each vehicle
/// class. This is the physical ceiling, not typical driver behavior — drivers usually
/// command less (see <c>SteeringController.EffectiveMaxAccel</c>, which takes the min of
/// the driver's desired launch rate and this capability). Values follow published
/// measurements: loaded heavy trucks ~0.6–1.0 m/s², transit buses ~1.0–1.5 m/s²
/// (passenger-comfort limited), typical sedans ~3.4 m/s² at full throttle, motorcycles
/// higher still (rider-limited in practice).
/// </summary>
public static class VehicleTypeDynamics
{
    // Full-throttle acceleration capability in m/s², indexed by VehicleType.
    private static readonly float[] _maxAccel =
    {
        3.4f,  // Sedan
        3.0f,  // SUV
        0.8f,  // Truck (loaded)
        1.2f,  // Bus
        4.0f,  // Motorcycle
    };

    /// <summary>
    /// Returns the full-throttle acceleration capability (m/s²) for the given
    /// <paramref name="type"/>. Out-of-range values are clamped to <see cref="VehicleType.Sedan"/>.
    /// </summary>
    public static float GetMaxAccel(VehicleType type)
    {
        int idx = (int)type;
        if ((uint)idx >= (uint)_maxAccel.Length) idx = 0;
        return _maxAccel[idx];
    }
}
