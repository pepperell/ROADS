using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// A target location vehicles pathfind toward, snapped to a road edge.
/// The nearest node to the EdgeT position is used as the pathfinding goal.
/// </summary>
public struct DestinationPoint
{
    /// <summary>World-space position in meters, snapped to the nearest road surface.</summary>
    public Vector2 Position;
    /// <summary>Index of the road edge this point is attached to.</summary>
    public int EdgeIndex;
    /// <summary>Parametric position along the edge's Bezier curve (0 = FromNode, 1 = ToNode).</summary>
    public float EdgeT;
}
