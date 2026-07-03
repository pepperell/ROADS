using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// Flags indicating traffic control and intersection behavior for a node.
/// Multiple flags can be combined (e.g., ManualSignal | TrafficLight).
/// </summary>
[Flags]
public enum NodeFlags : byte
{
    None = 0,
    /// <summary>Node has a traffic light cycling through green/yellow/red phases.</summary>
    TrafficLight = 1,
    /// <summary>Node has all-way stop sign logic (first-come-first-served priority).</summary>
    StopSign = 2,
    /// <summary>Node has yield signs (vehicles slow and check for cross-traffic).</summary>
    Yield = 4,
    // 8 reserved (was Spawn, removed with spawn points — vehicles enter/leave via EntryExit nodes;
    // MapSerializer.Load masks the bit out of legacy files)
    /// <summary>Signal was explicitly placed by the user (not auto-assigned).</summary>
    ManualSignal = 16,
    /// <summary>Node is a vehicle destination (only valid on nodes with ≤ 2 outgoing edges).</summary>
    Destination = 32,
    // 64 reserved (was RegionSpawn, removed in entry/exit merge)
}

/// <summary>
/// Type of Point of Interest at a destination node.
/// </summary>
public enum POIType : byte
{
    None = 0,
    Home,
    Work,
    Shop,
    Leisure,
    School,
    Parking,
    /// <summary>Map entry/exit boundary where vehicles spawn into town and despawn leaving it
    /// (resident move-in, through-traffic, emigrants, undestined cars). Works on one-way OR two-way
    /// roads: a node with an OUTGOING edge is an ENTRY (a lane into town — vehicles spawn there at max
    /// speed and drive in) and a node with an INCOMING edge is an EXIT (a lane out of town — vehicles
    /// drive there and despawn); a two-way node is both. The map needs at least one entry-capable and
    /// one exit-capable entry/exit node to fully function — they can be the same two-way node, or the
    /// two ends of a long one-way road (spawn at the upstream end, despawn at the downstream end).
    /// Through-traffic additionally needs its spawn and despawn nodes to be different.</summary>
    EntryExit,
}

/// <summary>
/// A node in the road graph representing an intersection or endpoint.
/// Nodes own a contiguous slice of the adjacency list for outgoing edges.
/// A defunct node has Position set to NaN.
/// </summary>
public struct RoadNode
{
    /// <summary>World-space position in meters.</summary>
    public Vector2 Position;
    /// <summary>Start index into RoadGraph's adjacency array for this node's outgoing edges.</summary>
    public ushort EdgeStartIdx;
    /// <summary>Number of outgoing edges from this node.</summary>
    public byte EdgeCount;
    /// <summary>Traffic control flags for this intersection.</summary>
    public NodeFlags Flags;
    /// <summary>Point of Interest type (only meaningful when Destination flag is set).</summary>
    public POIType PointOfInterest;
}
