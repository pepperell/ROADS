using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// A cubic Bezier curve connecting an incoming lane endpoint to an outgoing lane endpoint
/// through an intersection node. Vehicles traverse these arcs instead of jumping between edges.
/// </summary>
public struct IntersectionArc
{
    /// <summary>Index of the intersection node this arc belongs to.</summary>
    public int NodeIndex;
    /// <summary>Index of the edge arriving at the intersection.</summary>
    public int IncomingEdge;
    /// <summary>Index of the edge leaving the intersection.</summary>
    public int OutgoingEdge;
    /// <summary>Lane on the incoming edge (0 = leftmost in travel direction).</summary>
    public byte IncomingLane;
    /// <summary>Lane on the outgoing edge (0 = leftmost in travel direction).</summary>
    public byte OutgoingLane;
    /// <summary>Start point (lane center at incoming edge's stop line).</summary>
    public Vector2 P0;
    /// <summary>First control point (tangent-aligned with incoming edge direction).</summary>
    public Vector2 P1;
    /// <summary>Second control point (tangent-aligned with outgoing edge direction).</summary>
    public Vector2 P2;
    /// <summary>End point (lane center at outgoing edge's start line).</summary>
    public Vector2 P3;
    /// <summary>Precomputed arc length in meters.</summary>
    public float Length;
    /// <summary>Speed limit through the arc, reduced for sharper turns.</summary>
    public float SpeedLimit;
}
