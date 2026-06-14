namespace Roads.App;

/// <summary>
/// Shared simulation constants used across rendering, physics, and traffic systems.
/// Centralizes values that must stay in sync (lane width, vehicle dimensions, etc.).
/// </summary>
public static class SimConstants
{
    /// <summary>Lane width in meters.</summary>
    public const float LaneWidth = 3.5f;

    /// <summary>Vehicle body length in meters.</summary>
    public const float VehicleLength = 4.5f;

    /// <summary>Vehicle body width in meters.</summary>
    public const float VehicleWidth = 2.0f;

    /// <summary>IDM maximum acceleration in m/s^2.</summary>
    public const float MaxAccel = 3.5f;

    /// <summary>Maximum brake deceleration in m/s^2.</summary>
    public const float MaxBrakeDecel = 9.0f;

    /// <summary>Cross-product threshold for classifying a turn vs. straight-through.</summary>
    public const float TurnThreshold = 0.3f;

    /// <summary>
    /// Minimum distance (meters) a road split / crossing intersection must sit from an
    /// edge's endpoint. A fixed distance — NOT a fraction of the edge — so the "can't split
    /// here" zone near a node does not grow with road length (which would leave long roads
    /// with large un-splittable / un-marked dead zones at the ends).
    /// </summary>
    public const float MinSplitSetback = 5f;
}
