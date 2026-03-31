using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// A location where vehicles can be spawned, snapped to a road edge.
/// Vehicles spawn facing along the edge tangent at EdgeT.
/// </summary>
public struct SpawnPoint
{
    /// <summary>World-space position in meters, snapped to the nearest road surface.</summary>
    public Vector2 Position;
    /// <summary>Index of the road edge this point is attached to.</summary>
    public int EdgeIndex;
    /// <summary>Parametric position along the edge's Bezier curve (0 = FromNode, 1 = ToNode).</summary>
    public float EdgeT;
}
