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
    /// Signed rightward lateral offset (meters) of a lane's center from the edge's Bézier
    /// path, in the travel-direction frame. This is the single source of truth for lane
    /// placement — steering, lane-change, the intersection-arc cache, and the renderer all
    /// route through it so the three road kinds stay consistent:
    /// <list type="bullet">
    /// <item><b>Single-lane two-way</b> (<see cref="EdgeFlags.SharedLane"/>): 0 — both
    /// directions drive on the path itself.</item>
    /// <item><b>One-way</b> (no reverse edge): lanes are <i>centered</i> on the path, so
    /// the asphalt sits symmetrically over the node-to-node line.</item>
    /// <item><b>Two-way</b> (paired): lanes sit to the right of the path, which is the
    /// center divider — <c>LaneWidth * (0.5 + lane)</c>, unchanged from the original model.</item>
    /// </list>
    /// </summary>
    public static float LaneLateralOffset(RoadGraph graph, int edgeIdx, int lane)
    {
        var edge = graph.Edges[edgeIdx];
        float lw = SimConstants.LaneWidth;
        if ((edge.Flags & EdgeFlags.SharedLane) != 0) return 0f;
        bool oneWay = graph.FindReverseEdge(edgeIdx) < 0;
        if (oneWay) return lw * (lane + 0.5f - edge.LaneCount * 0.5f);
        return lw * (0.5f + lane);
    }

    /// <summary>
    /// Half of the road's full asphalt width in meters — i.e. the magnitude of the boundary
    /// offset from the path center. Two-way <c>LaneCount * LaneWidth</c> (unchanged), one-way
    /// <c>LaneCount * LaneWidth / 2</c> (centered), single-lane two-way <c>LaneWidth / 2</c>.
    /// </summary>
    public static float RoadHalfWidth(RoadGraph graph, int edgeIdx)
    {
        var edge = graph.Edges[edgeIdx];
        float lw = SimConstants.LaneWidth;
        if ((edge.Flags & EdgeFlags.SharedLane) != 0) return lw * 0.5f;
        bool oneWay = graph.FindReverseEdge(edgeIdx) < 0;
        return oneWay ? edge.LaneCount * lw * 0.5f : edge.LaneCount * lw;
    }

    /// <summary>Full asphalt width in meters (= 2 × <see cref="RoadHalfWidth"/>).</summary>
    public static float RoadSurfaceWidth(RoadGraph graph, int edgeIdx)
        => 2f * RoadHalfWidth(graph, edgeIdx);

    /// <summary>
    /// Full asphalt width in meters a road with the given options would have — the same
    /// policy as <see cref="RoadHalfWidth"/> (shared-lane = one lane total, one-way =
    /// laneCount centered, two-way = laneCount per direction), but computed from option
    /// values instead of an existing edge. Used by editor previews (road-draw ghost,
    /// Update Segment hover) to show the width the operation will produce.
    /// </summary>
    public static float RoadSurfaceWidthFor(byte laneCount, bool oneWay, bool sharedLane)
    {
        float lw = SimConstants.LaneWidth;
        if (sharedLane) return lw;
        return oneWay ? laneCount * lw : 2f * laneCount * lw;
    }

    /// <summary>
    /// Visual footprint radius of a node in meters: the widest incident road's
    /// <see cref="RoadHalfWidth"/> across both incoming and outgoing edges, so a junction
    /// of wide arterials reads larger than a residential bend. Returns 0 for an isolated
    /// node (callers floor with their marker size, e.g. the renderer's node-dot radius).
    /// </summary>
    public static float NodeJunctionRadius(RoadGraph graph, int nodeIndex)
    {
        float radius = 0f;
        foreach (int e in graph.GetOutgoingEdges(nodeIndex))
            radius = MathF.Max(radius, RoadHalfWidth(graph, e));
        foreach (int e in graph.GetIncomingEdges(nodeIndex))
            radius = MathF.Max(radius, RoadHalfWidth(graph, e));
        return radius;
    }

    /// <summary>
    /// Lateral extent (min,max signed rightward offset) covered by THIS edge's own lanes,
    /// used to draw a stop line across exactly the approaching lanes. Two-way returns
    /// <c>(0, LaneCount*LaneWidth)</c> — the right half only; one-way / single-lane return
    /// the full centered span.
    /// </summary>
    public static (float min, float max) LaneSpan(RoadGraph graph, int edgeIdx)
    {
        var edge = graph.Edges[edgeIdx];
        float lw = SimConstants.LaneWidth;
        float first = LaneLateralOffset(graph, edgeIdx, 0);
        float last = LaneLateralOffset(graph, edgeIdx, Math.Max(0, edge.LaneCount - 1));
        return (MathF.Min(first, last) - lw * 0.5f, MathF.Max(first, last) + lw * 0.5f);
    }

    /// <summary>
    /// True only for a two-way road, where the edge path is the center divider and a yellow
    /// center line is painted. One-way and single-lane-two-way roads have no center divider.
    /// </summary>
    public static bool HasCenterDivider(RoadGraph graph, int edgeIdx)
    {
        var edge = graph.Edges[edgeIdx];
        if ((edge.Flags & EdgeFlags.SharedLane) != 0) return false;
        return graph.FindReverseEdge(edgeIdx) >= 0;
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
