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
/// Provides per-type visual dimensions used exclusively by the renderer.
/// Collision and headway calculations continue to use
/// <see cref="SimConstants.VehicleLength"/> / <see cref="SimConstants.VehicleWidth"/>
/// for all vehicle types (rendering-only limitation, addressed in a later milestone);
/// acceleration DOES vary by type — see <see cref="VehicleTypeDynamics"/>.
/// Sedan dimensions exactly match the SimConstants defaults so existing saves
/// look unchanged when all vehicles are Sedan (byte 0).
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
    /// Returns the render (length, width) for the given <paramref name="type"/>.
    /// Out-of-range values are clamped to <see cref="VehicleType.Sedan"/>.
    /// </summary>
    public static (float Length, float Width) GetDimensions(VehicleType type)
    {
        int idx = (int)type;
        if ((uint)idx >= (uint)_dims.Length) idx = 0;
        return _dims[idx];
    }
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
