using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// Road classification affecting default speed limits and visual style.
/// </summary>
public enum RoadType : byte
{
    Residential = 0,
    Arterial = 1,
    Highway = 2,
    Dirt = 3,
}

/// <summary>
/// Per-edge behavior flags.
/// </summary>
[Flags]
public enum EdgeFlags : ushort
{
    None = 0,
    /// <summary>
    /// Single-lane two-way road: one physical lane shared by both directions (e.g. a
    /// driveway lead-in). Set on BOTH edges of the pair; <see cref="RoadEdge.LaneCount"/>
    /// is forced to 1. Both directions drive on the edge path itself (lateral offset 0),
    /// the road renders one lane wide, and a vehicle may only enter when no vehicle is
    /// travelling the opposite direction on the segment (see SteeringController's
    /// shared-lane entry gate). A normal one-way road needs no flag — it is simply a
    /// directed edge with no reverse (FindReverseEdge &lt; 0).
    /// </summary>
    SharedLane = 1,
}

/// <summary>
/// A directed edge in the road graph, representing one direction of travel between two nodes.
/// Two-way roads are stored as a pair of edges (forward and reverse) sharing the same nodes.
/// The curve is a cubic Bezier defined by FromNode.Position, ControlPoint1, ControlPoint2,
/// and ToNode.Position. A defunct edge has FromNode set to -1.
/// </summary>
public struct RoadEdge
{
    /// <summary>Index of the start node. Set to -1 when the edge has been deleted.</summary>
    public int FromNode;
    /// <summary>Index of the end node (travel direction is FromNode → ToNode).</summary>
    public int ToNode;
    /// <summary>Arc length in meters, precomputed for pathfinding cost.</summary>
    public float Length;
    /// <summary>Maximum speed in meters per second.</summary>
    public float SpeedLimit;
    /// <summary>Number of lanes in this travel direction (1 = single lane).</summary>
    public byte LaneCount;
    /// <summary>Classification of this road (residential, arterial, highway, dirt).</summary>
    public RoadType RoadType;
    /// <summary>Per-edge behavior flags (one-way, no parking, bus lane, etc.).</summary>
    public EdgeFlags Flags;
    /// <summary>First cubic Bezier control point (near FromNode).</summary>
    public Vector2 ControlPoint1;
    /// <summary>Second cubic Bezier control point (near ToNode).</summary>
    public Vector2 ControlPoint2;
}

/// <summary>
/// Per-road-type gameplay data: default speed limits, right-of-way class ranks, and the
/// drawn-width multiplier shared by rendering and junction-clearance geometry.
/// </summary>
public static class RoadTypeDefaults
{
    /// <summary>
    /// Multiplier applied to the lane-derived (geometric) road width to get the DRAWN
    /// asphalt width — highways render noticeably wider, dirt narrower, without affecting
    /// lane geometry. Lives here rather than in the renderer because it is not purely
    /// cosmetic: junction trims and stop lines (<see cref="StopLineCache"/>) must clear
    /// the pavement the player actually sees, so they measure against half-width × this.
    /// The renderer's <c>RoadTypeVisuals.GetWidthMultiplier</c> delegates to it.
    /// </summary>
    public static float GetDrawnWidthMultiplier(RoadType type) => type switch
    {
        RoadType.Highway     => 1.25f,
        RoadType.Arterial    => 1.10f,
        RoadType.Residential => 1.00f,
        RoadType.Dirt        => 0.85f,
        _                    => 1.00f,
    };

    /// <summary>
    /// Returns the default speed limit in m/s for a given road type.
    /// </summary>
    public static float GetDefaultSpeedLimit(RoadType type) => type switch
    {
        RoadType.Residential => 11.2f,  // ~25 mph
        RoadType.Arterial    => 20.1f,  // ~45 mph
        RoadType.Highway     => 31.3f,  // ~70 mph
        RoadType.Dirt        => 6.7f,   // ~15 mph
        _                    => 13.4f,  // ~30 mph
    };

    /// <summary>
    /// Right-of-way class rank of an approach (higher = faster/major road). Used by the
    /// signal and stop-sign auto-assignment: a junction whose approaches span more than
    /// one rank never gets a light and becomes a minor-road stop — lower-ranked
    /// approaches get the stop sign, the highest-ranked class flows free. A shared-lane
    /// (single-lane two-way) road ranks just below the plain road of the same type, so a
    /// dirt driveway yields to the dirt road it joins, without outranking anything a
    /// plain road of its type wouldn't.
    /// </summary>
    public static int GetRoadClassRank(RoadType type, bool sharedLane)
    {
        int typeRank = type switch
        {
            RoadType.Dirt        => 0,
            RoadType.Residential => 1,
            RoadType.Arterial    => 2,
            RoadType.Highway     => 3,
            _                    => 1,
        };
        return typeRank * 2 + (sharedLane ? 0 : 1);
    }
}
