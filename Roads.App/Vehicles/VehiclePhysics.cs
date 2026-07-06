using Roads.App;

namespace Roads.App.Vehicles;

/// <summary>
/// Bicycle-model kinematic physics for vehicles. Simulates front-wheel steering
/// by independently moving front and rear wheel positions, then deriving the
/// new heading and center position. Applies throttle/brake to update speed.
/// </summary>
public static class VehiclePhysics
{
    /// <summary>Distance in meters between front and rear axle.</summary>
    private const float Wheelbase = 2.5f;

    /// <summary>
    /// Performs a bicycle-model kinematic update for a single vehicle.
    /// Moves front and rear wheels independently based on speed and steering,
    /// then updates heading, position, and speed from throttle/brake inputs.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Index of the vehicle to update.</param>
    /// <param name="dt">Delta time in seconds.</param>
    public static void Update(VehicleStore store, int index, float dt)
    {
        if (store.State[index] != VehicleState.Driving) return;

        float heading = store.Heading[index];
        float speed = store.Speed[index];
        float steer = store.SteeringAngle[index];

        float cosH = MathF.Cos(heading);
        float sinH = MathF.Sin(heading);

        // Front and rear wheel positions
        float halfWb = Wheelbase * 0.5f;
        float fx = store.PosX[index] + halfWb * cosH;
        float fy = store.PosY[index] + halfWb * sinH;
        float rx = store.PosX[index] - halfWb * cosH;
        float ry = store.PosY[index] - halfWb * sinH;

        // Move wheels
        float frontAngle = heading + steer;
        fx += speed * dt * MathF.Cos(frontAngle);
        fy += speed * dt * MathF.Sin(frontAngle);
        rx += speed * dt * cosH;
        ry += speed * dt * sinH;

        // New heading and position
        store.Heading[index] = MathF.Atan2(fy - ry, fx - rx);
        store.PosX[index] = (fx + rx) * 0.5f;
        store.PosY[index] = (fy + ry) * 0.5f;

        if (store.DiagVehicle == index)
            SteeringController.LogDiag(store, index, "PHYSICS");

        // Apply reaction-time lag filter (exponential smoothing on throttle/brake)
        float reactionTime = store.ReactionTime[index];
        float alpha = 1f - MathF.Exp(-dt / MathF.Max(reactionTime, 0.033f));
        store.SmoothedThrottle[index] += (store.Throttle[index] - store.SmoothedThrottle[index]) * alpha;
        // Emergency brake bypass: don't lag hard stops
        if (store.Brake[index] >= 0.99f)
            store.SmoothedBrake[index] = 1.0f;
        else
            store.SmoothedBrake[index] += (store.Brake[index] - store.SmoothedBrake[index]) * alpha;

        // Apply smoothed throttle/brake to speed. Throttle is normalized by the same
        // per-driver/per-type EffectiveMaxAccel the IDM controller divided by, so the
        // round trip reproduces the IDM acceleration.
        float maxAccel = SteeringController.EffectiveMaxAccel(store, index);
        float maxBrake = SimConstants.MaxBrakeDecel;
        float accel = store.SmoothedThrottle[index] * maxAccel - store.SmoothedBrake[index] * maxBrake;
        speed += accel * dt;
        store.Speed[index] = MathF.Max(0f, speed);
    }

    /// <summary>
    /// Updates physics for all active vehicles.
    /// </summary>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="dt">Delta time in seconds.</param>
    public static void UpdateAll(VehicleStore store, float dt)
    {
        for (int i = 0; i < store.Count; i++)
        {
            Update(store, i, dt);
        }
    }
}
