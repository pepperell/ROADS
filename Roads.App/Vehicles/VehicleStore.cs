namespace Roads.App.Vehicles;

/// <summary>
/// Lifecycle state of a vehicle.
/// </summary>
public enum VehicleState : byte
{
    Driving,
}

/// <summary>
/// Struct-of-Arrays (SoA) storage for all vehicle data, organized into hot (per-tick physics),
/// warm (edge tracking), path, lane change, and cold (visual) arrays. Uses swap-and-pop
/// removal for O(1) deletes. Designed for cache-efficient iteration by simulation systems.
/// </summary>
public class VehicleStore
{
    // ── Hot data (touched every tick) ──

    /// <summary>World-space X position in meters.</summary>
    public float[] PosX = Array.Empty<float>();
    /// <summary>World-space Y position in meters.</summary>
    public float[] PosY = Array.Empty<float>();
    /// <summary>Heading angle in radians (0 = right, increases clockwise in Y-down coords).</summary>
    public float[] Heading = Array.Empty<float>();
    /// <summary>Forward speed in meters per second.</summary>
    public float[] Speed = Array.Empty<float>();
    /// <summary>Front-wheel steering angle in radians (positive = turn right).</summary>
    public float[] SteeringAngle = Array.Empty<float>();
    /// <summary>Throttle input (0–1).</summary>
    public float[] Throttle = Array.Empty<float>();
    /// <summary>Brake input (0–1).</summary>
    public float[] Brake = Array.Empty<float>();

    /// <summary>Index of vehicle to log per-frame diagnostics for, or -1 for none.</summary>
    public int DiagVehicle = -1;

    // ── Warm data (edge tracking) ──

    /// <summary>Index of the road edge the vehicle is currently traveling on.</summary>
    public int[] CurrentEdge = Array.Empty<int>();
    /// <summary>Parametric progress along the current edge (0 = FromNode, 1 = ToNode).</summary>
    public float[] EdgeProgress = Array.Empty<float>();
    /// <summary>Previous heading error in radians, used by PD steering controller.</summary>
    public float[] PrevHeadingError = Array.Empty<float>();
    /// <summary>Squared distance from vehicle position to its lane center, used for off-road detection.</summary>
    public float[] DistToRoadSq = Array.Empty<float>();

    // ── Arc tracking (intersection traversal) ──

    /// <summary>Index into IntersectionArcCache, or -1 if vehicle is on an edge.</summary>
    public int[] CurrentArc = Array.Empty<int>();
    /// <summary>Parametric progress (0–1) along the current intersection arc.</summary>
    public float[] ArcProgress = Array.Empty<float>();

    // ── Path data ──

    /// <summary>Precomputed edge-index path from spawn to destination (null if no path).</summary>
    public List<int>?[] Path = Array.Empty<List<int>?>();
    /// <summary>Current index within the <see cref="Path"/> array.</summary>
    public int[] PathIndex = Array.Empty<int>();
    /// <summary>Node index of the vehicle's final destination, or -1 if none.</summary>
    public int[] DestinationNode = Array.Empty<int>();

    // ── Lane change data ──

    /// <summary>Current lane index (0 = leftmost lane in travel direction).</summary>
    public byte[] CurrentLane = Array.Empty<byte>();
    /// <summary>Target lane index for an in-progress lane change.</summary>
    public byte[] TargetLane = Array.Empty<byte>();
    /// <summary>Interpolation progress (0–1) during a lane change.</summary>
    public float[] LaneChangeProgress = Array.Empty<float>();
    /// <summary>Seconds remaining before the next lane-change evaluation is allowed.</summary>
    public float[] LaneChangeCooldown = Array.Empty<float>();
    /// <summary>Desired lane index from the most recent lane change evaluation, or <c>byte.MaxValue</c> if none.</summary>
    public byte[] DesiredLane = Array.Empty<byte>();
    /// <summary>Urgency (0–1) for merging to the correct lane before an upcoming turn.</summary>
    public float[] MergeUrgency = Array.Empty<float>();
    /// <summary>Speed bias in m/s applied to find merge gaps (positive = speed up, negative = slow down).</summary>
    public float[] MergeSpeedBias = Array.Empty<float>();

    // ── Driver personality (cold, set at spawn) ──

    /// <summary>Aggressiveness (0–1): affects following distance, lane change gaps.</summary>
    public float[] Aggressiveness = Array.Empty<float>();
    /// <summary>Speed bias (0.8–1.3): multiplier on speed limit for desired speed.</summary>
    public float[] SpeedBias = Array.Empty<float>();
    /// <summary>Reaction time (0.3–1.2s): delay before responding to throttle/brake changes.</summary>
    public float[] ReactionTime = Array.Empty<float>();
    /// <summary>Steering sharpness (0.5–2.0): multiplier on PD steering gains.</summary>
    public float[] SteeringSharpness = Array.Empty<float>();
    /// <summary>Comfortable deceleration (1.5–4.0 m/s²): IDM 'b' parameter.</summary>
    public float[] BrakingComfort = Array.Empty<float>();
    /// <summary>Lane change eagerness (0–1): affects lane change frequency and duration.</summary>
    public float[] LaneChangeBias = Array.Empty<float>();
    /// <summary>Patience before risky maneuvers (5–180s).</summary>
    public float[] PatienceTimer = Array.Empty<float>();
    /// <summary>Preferred vehicle type index.</summary>
    public byte[] PreferredVehicle = Array.Empty<byte>();
    /// <summary>Driver archetype enum value, for UI display.</summary>
    public byte[] Archetype = Array.Empty<byte>();
    /// <summary>Smoothed throttle input after reaction-time lag filter.</summary>
    public float[] SmoothedThrottle = Array.Empty<float>();
    /// <summary>Smoothed brake input after reaction-time lag filter.</summary>
    public float[] SmoothedBrake = Array.Empty<float>();

    // ── Cold data (visual) ──

    /// <summary>Lifecycle state of the vehicle.</summary>
    public VehicleState[] State = Array.Empty<VehicleState>();
    /// <summary>Red component of the vehicle body color.</summary>
    public byte[] ColorR = Array.Empty<byte>();
    /// <summary>Green component of the vehicle body color.</summary>
    public byte[] ColorG = Array.Empty<byte>();
    /// <summary>Blue component of the vehicle body color.</summary>
    public byte[] ColorB = Array.Empty<byte>();

    /// <summary>Weighted car color palette based on real-world vehicle color popularity.</summary>
    private static readonly (byte r, byte g, byte b, int weight)[] CarColors = {
        // White variants (25%)
        (255, 255, 255, 8),  // pure white
        (240, 240, 238, 9),  // pearl white
        (228, 225, 220, 8),  // off-white
        // Black variants (22%)
        (20, 20, 22, 8),     // jet black
        (35, 35, 38, 7),     // soft black
        (28, 28, 30, 7),     // metallic black
        // Gray variants (17%)
        (130, 130, 132, 6),  // medium gray
        (100, 100, 103, 6),  // charcoal
        (160, 160, 162, 5),  // light gray
        // Silver variants (12%)
        (192, 192, 195, 6),  // classic silver
        (175, 178, 182, 6),  // dark silver
        // Blue variants (8%)
        (30, 60, 120, 3),    // dark blue
        (45, 85, 155, 3),    // medium blue
        (60, 100, 170, 2),   // steel blue
        // Red variants (7%)
        (165, 25, 25, 3),    // dark red
        (195, 30, 30, 2),    // bright red
        (140, 22, 22, 2),    // maroon
        // Brown/beige (4%)
        (110, 85, 60, 2),    // brown
        (165, 148, 125, 2),  // beige/tan
        // Green (2%)
        (40, 75, 50, 1),     // dark green
        (55, 95, 65, 1),     // forest green
        // Other (3%)
        (180, 130, 40, 1),   // gold
        (90, 55, 95, 1),     // dark purple
        (210, 120, 30, 1),   // burnt orange
    };
    /// <summary>Sum of all weights in the color palette.</summary>
    private static readonly int TotalWeight = CarColors.Sum(c => c.weight);

    /// <summary>
    /// Picks a random car color from the weighted palette.
    /// </summary>
    /// <returns>RGB tuple for the selected color.</returns>
    public static (byte r, byte g, byte b) RandomCarColor()
    {
        int roll = Random.Shared.Next(TotalWeight);
        int cumulative = 0;
        foreach (var (r, g, b, w) in CarColors)
        {
            cumulative += w;
            if (roll < cumulative) return (r, g, b);
        }
        return (CarColors[^1].r, CarColors[^1].g, CarColors[^1].b);
    }

    /// <summary>Number of active vehicles.</summary>
    public int Count { get; private set; }
    /// <summary>Current capacity of the backing arrays.</summary>
    public int Capacity { get; private set; }

    /// <summary>
    /// Adds a new vehicle at the given position and heading on the specified edge.
    /// Grows backing arrays if needed.
    /// </summary>
    /// <param name="x">World-space X position.</param>
    /// <param name="y">World-space Y position.</param>
    /// <param name="heading">Initial heading in radians.</param>
    /// <param name="edgeIndex">Index of the road edge the vehicle starts on.</param>
    /// <returns>Index of the newly created vehicle.</returns>
    public int Add(float x, float y, float heading, int edgeIndex)
    {
        if (Count >= Capacity)
            Grow(Math.Max(16, Capacity * 2));

        int i = Count++;
        PosX[i] = x;
        PosY[i] = y;
        Heading[i] = heading;
        Speed[i] = 0f;
        SteeringAngle[i] = 0f;
        Throttle[i] = 0f;
        Brake[i] = 0f;
        CurrentEdge[i] = edgeIndex;
        EdgeProgress[i] = 0f;
        PrevHeadingError[i] = 0f;
        DistToRoadSq[i] = 0f;
        CurrentArc[i] = -1;
        ArcProgress[i] = 0f;
        Path[i] = null;
        PathIndex[i] = 0;
        DestinationNode[i] = -1;
        CurrentLane[i] = 0;
        TargetLane[i] = 0;
        LaneChangeProgress[i] = 0f;
        LaneChangeCooldown[i] = 0f;
        DesiredLane[i] = byte.MaxValue;
        MergeUrgency[i] = 0f;
        MergeSpeedBias[i] = 0f;
        Aggressiveness[i] = 0.4f;
        SpeedBias[i] = 1.0f;
        ReactionTime[i] = 0.6f;
        SteeringSharpness[i] = 1.0f;
        BrakingComfort[i] = 2.5f;
        LaneChangeBias[i] = 0.5f;
        PatienceTimer[i] = 30f;
        PreferredVehicle[i] = 0;
        Archetype[i] = (byte)DriverArchetype.Commuter;
        SmoothedThrottle[i] = 0f;
        SmoothedBrake[i] = 0f;
        State[i] = VehicleState.Driving;
        var (cr, cg, cb) = RandomCarColor();
        ColorR[i] = cr;
        ColorG[i] = cg;
        ColorB[i] = cb;
        return i;
    }

    /// <summary>
    /// Removes a vehicle by swapping it with the last vehicle (swap-and-pop).
    /// </summary>
    /// <param name="index">Index of the vehicle to remove.</param>
    /// <returns>The index that was swapped in, or -1 if the removed vehicle was the last.</returns>
    public int Remove(int index)
    {
        if (index < 0 || index >= Count) return -1;

        int last = Count - 1;
        if (index < last)
        {
            // Swap last into the hole
            PosX[index] = PosX[last];
            PosY[index] = PosY[last];
            Heading[index] = Heading[last];
            Speed[index] = Speed[last];
            SteeringAngle[index] = SteeringAngle[last];
            Throttle[index] = Throttle[last];
            Brake[index] = Brake[last];
            CurrentEdge[index] = CurrentEdge[last];
            EdgeProgress[index] = EdgeProgress[last];
            PrevHeadingError[index] = PrevHeadingError[last];
            DistToRoadSq[index] = DistToRoadSq[last];
            CurrentArc[index] = CurrentArc[last];
            ArcProgress[index] = ArcProgress[last];
            Path[index] = Path[last];
            PathIndex[index] = PathIndex[last];
            DestinationNode[index] = DestinationNode[last];
            CurrentLane[index] = CurrentLane[last];
            TargetLane[index] = TargetLane[last];
            LaneChangeProgress[index] = LaneChangeProgress[last];
            LaneChangeCooldown[index] = LaneChangeCooldown[last];
            DesiredLane[index] = DesiredLane[last];
            MergeUrgency[index] = MergeUrgency[last];
            MergeSpeedBias[index] = MergeSpeedBias[last];
            Aggressiveness[index] = Aggressiveness[last];
            SpeedBias[index] = SpeedBias[last];
            ReactionTime[index] = ReactionTime[last];
            SteeringSharpness[index] = SteeringSharpness[last];
            BrakingComfort[index] = BrakingComfort[last];
            LaneChangeBias[index] = LaneChangeBias[last];
            PatienceTimer[index] = PatienceTimer[last];
            PreferredVehicle[index] = PreferredVehicle[last];
            Archetype[index] = Archetype[last];
            SmoothedThrottle[index] = SmoothedThrottle[last];
            SmoothedBrake[index] = SmoothedBrake[last];
            State[index] = State[last];
            ColorR[index] = ColorR[last];
            ColorG[index] = ColorG[last];
            ColorB[index] = ColorB[last];
        }

        Count--;
        return index < last ? index : -1;
    }

    /// <summary>Resizes all backing arrays to the new capacity.</summary>
    private void Grow(int newCapacity)
    {
        Array.Resize(ref PosX, newCapacity);
        Array.Resize(ref PosY, newCapacity);
        Array.Resize(ref Heading, newCapacity);
        Array.Resize(ref Speed, newCapacity);
        Array.Resize(ref SteeringAngle, newCapacity);
        Array.Resize(ref Throttle, newCapacity);
        Array.Resize(ref Brake, newCapacity);
        Array.Resize(ref CurrentEdge, newCapacity);
        Array.Resize(ref EdgeProgress, newCapacity);
        Array.Resize(ref PrevHeadingError, newCapacity);
        Array.Resize(ref DistToRoadSq, newCapacity);
        Array.Resize(ref CurrentArc, newCapacity);
        Array.Resize(ref ArcProgress, newCapacity);
        Array.Resize(ref Path, newCapacity);
        Array.Resize(ref PathIndex, newCapacity);
        Array.Resize(ref DestinationNode, newCapacity);
        Array.Resize(ref CurrentLane, newCapacity);
        Array.Resize(ref TargetLane, newCapacity);
        Array.Resize(ref LaneChangeProgress, newCapacity);
        Array.Resize(ref LaneChangeCooldown, newCapacity);
        Array.Resize(ref DesiredLane, newCapacity);
        Array.Resize(ref MergeUrgency, newCapacity);
        Array.Resize(ref MergeSpeedBias, newCapacity);
        Array.Resize(ref Aggressiveness, newCapacity);
        Array.Resize(ref SpeedBias, newCapacity);
        Array.Resize(ref ReactionTime, newCapacity);
        Array.Resize(ref SteeringSharpness, newCapacity);
        Array.Resize(ref BrakingComfort, newCapacity);
        Array.Resize(ref LaneChangeBias, newCapacity);
        Array.Resize(ref PatienceTimer, newCapacity);
        Array.Resize(ref PreferredVehicle, newCapacity);
        Array.Resize(ref Archetype, newCapacity);
        Array.Resize(ref SmoothedThrottle, newCapacity);
        Array.Resize(ref SmoothedBrake, newCapacity);
        Array.Resize(ref State, newCapacity);
        Array.Resize(ref ColorR, newCapacity);
        Array.Resize(ref ColorG, newCapacity);
        Array.Resize(ref ColorB, newCapacity);
        Capacity = newCapacity;
    }
}
