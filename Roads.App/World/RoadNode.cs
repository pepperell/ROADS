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
    /// <summary>Signal was explicitly placed by the user (not auto-assigned).</summary>
    ManualSignal = 16,
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
}
