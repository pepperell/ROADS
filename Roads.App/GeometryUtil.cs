using System.Numerics;
using Roads.App.World;

namespace Roads.App;

/// <summary>
/// Shared geometry utilities for Bezier curve operations, spatial hashing, and
/// vector math used across rendering, simulation, and spatial indexing systems.
/// </summary>
public static class GeometryUtil
{
    /// <summary>
    /// Offsets a point on a road edge's Bezier curve to the right of the travel direction.
    /// Uses (-tangent.Y, tangent.X) as the right normal in Y-down screen coordinates.
    /// </summary>
    /// <param name="graph">Road graph for curve evaluation.</param>
    /// <param name="edgeIdx">Index of the edge.</param>
    /// <param name="t">Parametric position on the curve (0-1).</param>
    /// <param name="offset">Rightward offset in meters (positive = right, negative = left).</param>
    /// <returns>World-space position offset to the right of the curve.</returns>
    public static Vector2 OffsetRight(RoadGraph graph, int edgeIdx, float t, float offset)
    {
        var pos = graph.EvaluateBezier(edgeIdx, t);
        var tangent = graph.EvaluateBezierTangent(edgeIdx, t);
        float len = tangent.Length();
        if (len < 0.001f) return pos;
        float rx = -tangent.Y / len;
        float ry = tangent.X / len;
        return new Vector2(pos.X + rx * offset, pos.Y + ry * offset);
    }

    /// <summary>
    /// Estimates the arc length of a cubic Bezier curve using a 16-segment polyline approximation.
    /// </summary>
    public static float EstimateBezierLength(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3)
    {
        const int steps = 16;
        float length = 0f;
        var prev = p0;
        for (int i = 1; i <= steps; i++)
        {
            float t = (float)i / steps;
            float u = 1f - t;
            var pt = u * u * u * p0 + 3f * u * u * t * p1 + 3f * u * t * t * p2 + t * t * t * p3;
            length += Vector2.Distance(prev, pt);
            prev = pt;
        }
        return length;
    }

    /// <summary>
    /// Turn direction classification result.
    /// </summary>
    public enum TurnDirection { Straight, Left, Right }

    /// <summary>
    /// Classifies the turn direction from one edge to the next using the cross product
    /// of their tangent directions at the intersection node. Shared by YieldSignSystem
    /// and LaneChangeLogic.
    /// </summary>
    /// <param name="graph">Road graph for Bezier tangent evaluation.</param>
    /// <param name="fromEdge">Incoming edge index (tangent sampled at t=0.95).</param>
    /// <param name="toEdge">Outgoing edge index (tangent sampled at t=0.05).</param>
    /// <returns>The classified turn direction.</returns>
    public static TurnDirection ClassifyTurn(RoadGraph graph, int fromEdge, int toEdge)
    {
        var inTangent = graph.EvaluateBezierTangent(fromEdge, 0.95f);
        var outTangent = graph.EvaluateBezierTangent(toEdge, 0.05f);
        float inLen = inTangent.Length();
        float outLen = outTangent.Length();
        if (inLen < 0.001f || outLen < 0.001f) return TurnDirection.Straight;

        float cross = inTangent.X * outTangent.Y - inTangent.Y * outTangent.X;
        float crossNorm = cross / (inLen * outLen);

        if (crossNorm > SimConstants.TurnThreshold) return TurnDirection.Right;
        if (crossNorm < -SimConstants.TurnThreshold) return TurnDirection.Left;
        return TurnDirection.Straight;
    }

    /// <summary>
    /// Packs integer cell coordinates into a single hash key using bit spreading.
    /// Used by both SpatialGrid and EdgeSpatialGrid for consistent cell hashing.
    /// </summary>
    public static int PackCell(int cx, int cy)
    {
        unchecked
        {
            return cx * 73856093 ^ cy * 19349663;
        }
    }
}
