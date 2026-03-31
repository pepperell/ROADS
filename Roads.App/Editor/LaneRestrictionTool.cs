using System.Numerics;
using SkiaSharp;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Handles lane restriction editing at intersections. When active, renders an overlay
/// showing incoming/outgoing lane dots and connection arcs, and handles clicks to
/// select incoming lanes or toggle outgoing lane connections.
/// </summary>
public class LaneRestrictionTool
{
    /// <summary>
    /// Draws the lane restriction overlay for a given node: incoming lane dots,
    /// outgoing lane dots (colored by connectivity), and Bezier arc lines
    /// from the active incoming lane to its allowed outgoing lanes.
    /// </summary>
    public void DrawOverlay(SKCanvas canvas, int nodeIdx, RoadGraph graph,
        EditorState state, StopLineCache stopLines, float cameraZoom)
    {
        const float LaneWidth = SimConstants.LaneWidth;
        float dotRadius = MathF.Max(0.8f, 1.5f / cameraZoom);
        float lineWidth = MathF.Max(0.5f, 1f / cameraZoom);

        var incoming = graph.GetIncomingEdges(nodeIdx);
        var outgoing = graph.GetOutgoingEdges(nodeIdx);

        int activeInEdge = state.LaneRestrictionEdge;
        byte activeInLane = state.LaneRestrictionLane;

        using var incomingPaint = new SKPaint { Color = new SKColor(80, 180, 255, 200), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var activePaint = new SKPaint { Color = new SKColor(255, 220, 50, 255), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var allowedPaint = new SKPaint { Color = new SKColor(80, 220, 80, 200), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var blockedPaint = new SKPaint { Color = new SKColor(120, 120, 120, 120), Style = SKPaintStyle.Fill, IsAntialias = true };
        using var arcLinePaint = new SKPaint { Color = new SKColor(0, 255, 255, 230), Style = SKPaintStyle.Stroke, StrokeWidth = lineWidth * 1.5f, IsAntialias = true };

        // Draw incoming lane dots (active one highlighted)
        foreach (int edgeIdx in incoming)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;
            float sampleT = stopLines.GetStopTAtToNode(edgeIdx);
            var tangent = graph.EvaluateBezierTangent(edgeIdx, sampleT);
            float tanLen = tangent.Length();
            if (tanLen < 0.001f) continue;
            var right = new Vector2(-tangent.Y, tangent.X) / tanLen;

            for (byte lane = 0; lane < edge.LaneCount; lane++)
            {
                float offset = LaneWidth * (0.5f + lane);
                var pos = graph.EvaluateBezier(edgeIdx, sampleT) + right * offset;
                bool isActive = edgeIdx == activeInEdge && lane == activeInLane;
                canvas.DrawCircle(pos.X, pos.Y, isActive ? dotRadius * 1.5f : dotRadius,
                    isActive ? activePaint : incomingPaint);
            }
        }

        // Draw outgoing lane dots (colored by whether active incoming lane connects to them)
        foreach (int edgeIdx in outgoing)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;
            float sampleT = stopLines.GetStopTAtFromNode(edgeIdx);
            var tangent = graph.EvaluateBezierTangent(edgeIdx, sampleT);
            float tanLen = tangent.Length();
            if (tanLen < 0.001f) continue;
            var right = new Vector2(-tangent.Y, tangent.X) / tanLen;

            for (byte lane = 0; lane < edge.LaneCount; lane++)
            {
                float offset = LaneWidth * (0.5f + lane);
                var pos = graph.EvaluateBezier(edgeIdx, sampleT) + right * offset;

                SKPaint paint;
                if (activeInEdge >= 0)
                {
                    var restrictions = graph.GetLaneRestrictions(activeInEdge, activeInLane)
                        ?? graph.GetGeometryDefaultLaneTargets(activeInEdge, activeInLane, stopLines);
                    paint = restrictions.Contains((edgeIdx, lane)) ? allowedPaint : blockedPaint;
                }
                else
                    paint = blockedPaint; // no input selected yet

                canvas.DrawCircle(pos.X, pos.Y, dotRadius, paint);
            }
        }

        // Draw arc lines from the active incoming lane to allowed outgoing lanes
        if (activeInEdge >= 0)
        {
            float inStopT = stopLines.GetStopTAtToNode(activeInEdge);
            var inTangent = graph.EvaluateBezierTangent(activeInEdge, inStopT);
            float inTanLen = inTangent.Length();
            if (inTanLen > 0.001f)
            {
                var inDir = inTangent / inTanLen;
                var inRight = new Vector2(-inTangent.Y, inTangent.X) / inTanLen;
                float inOffset = LaneWidth * (0.5f + activeInLane);
                var p0 = graph.EvaluateBezier(activeInEdge, inStopT) + inRight * inOffset;

                var restrictions = graph.GetLaneRestrictions(activeInEdge, activeInLane)
                    ?? graph.GetGeometryDefaultLaneTargets(activeInEdge, activeInLane, stopLines);

                foreach (int outEdgeIdx in outgoing)
                {
                    var outEdgeData = graph.Edges[outEdgeIdx];
                    if (outEdgeData.FromNode < 0) continue;
                    float outStartT = stopLines.GetStopTAtFromNode(outEdgeIdx);
                    var outTangent = graph.EvaluateBezierTangent(outEdgeIdx, outStartT);
                    float outTanLen = outTangent.Length();
                    if (outTanLen < 0.001f) continue;
                    var outDir = outTangent / outTanLen;
                    var outRight = new Vector2(-outTangent.Y, outTangent.X) / outTanLen;

                    for (byte outLane = 0; outLane < outEdgeData.LaneCount; outLane++)
                    {
                        if (!restrictions.Contains((outEdgeIdx, outLane))) continue;

                        float outOffset = LaneWidth * (0.5f + outLane);
                        var p3 = graph.EvaluateBezier(outEdgeIdx, outStartT) + outRight * outOffset;
                        float dist = Vector2.Distance(p0, p3);
                        float arm = MathF.Max(1.5f, MathF.Min(dist / 3f, 15f));
                        var p1 = p0 + inDir * arm;
                        var p2 = p3 - outDir * arm;

                        using var arcPath = new SKPath();
                        arcPath.MoveTo(p0.X, p0.Y);
                        for (int s = 1; s <= 12; s++)
                        {
                            float t = s / 12f;
                            float u = 1f - t;
                            float x = u * u * u * p0.X + 3 * u * u * t * p1.X + 3 * u * t * t * p2.X + t * t * t * p3.X;
                            float y = u * u * u * p0.Y + 3 * u * u * t * p1.Y + 3 * u * t * t * p2.Y + t * t * t * p3.Y;
                            arcPath.LineTo(x, y);
                        }
                        canvas.DrawPath(arcPath, arcLinePaint);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Handles a click during lane restriction mode. If no incoming lane is active,
    /// selects the nearest incoming lane. If an incoming lane is active, toggles the
    /// nearest outgoing lane connection from that input.
    /// </summary>
    public void OnClick(Vector2 worldPos, RoadGraph graph, EditorState state,
        StopLineCache stopLines)
    {
        int nodeIdx = state.SelectedNode;
        if (nodeIdx < 0) return;

        var incoming = graph.GetIncomingEdges(nodeIdx);
        var outgoing = graph.GetOutgoingEdges(nodeIdx);

        // Find the nearest lane across all edges at this node
        float bestDist = EditorState.SnapDistance;
        int bestEdge = -1;
        byte bestLane = 0;
        bool bestIsIncoming = false;

        // Check incoming edge lanes (sample at t near the ToNode end)
        foreach (int edgeIdx in incoming)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;
            float sampleT = stopLines.GetStopTAtToNode(edgeIdx);
            var tangent = graph.EvaluateBezierTangent(edgeIdx, sampleT);
            float tanLen = tangent.Length();
            if (tanLen < 0.001f) continue;
            var right = new Vector2(-tangent.Y, tangent.X) / tanLen;

            for (byte lane = 0; lane < edge.LaneCount; lane++)
            {
                float offset = SimConstants.LaneWidth * (0.5f + lane);
                var lanePos = graph.EvaluateBezier(edgeIdx, sampleT) + right * offset;
                float dist = Vector2.Distance(worldPos, lanePos);
                if (dist < bestDist) { bestDist = dist; bestEdge = edgeIdx; bestLane = lane; bestIsIncoming = true; }
            }
        }

        // Check outgoing edge lanes (sample at t near the FromNode end)
        foreach (int edgeIdx in outgoing)
        {
            var edge = graph.Edges[edgeIdx];
            if (edge.FromNode < 0) continue;
            float sampleT = stopLines.GetStopTAtFromNode(edgeIdx);
            var tangent = graph.EvaluateBezierTangent(edgeIdx, sampleT);
            float tanLen = tangent.Length();
            if (tanLen < 0.001f) continue;
            var right = new Vector2(-tangent.Y, tangent.X) / tanLen;

            for (byte lane = 0; lane < edge.LaneCount; lane++)
            {
                float offset = SimConstants.LaneWidth * (0.5f + lane);
                var lanePos = graph.EvaluateBezier(edgeIdx, sampleT) + right * offset;
                float dist = Vector2.Distance(worldPos, lanePos);
                if (dist < bestDist) { bestDist = dist; bestEdge = edgeIdx; bestLane = lane; bestIsIncoming = false; }
            }
        }

        if (bestEdge < 0) return;

        if (bestIsIncoming)
        {
            // Select this incoming lane as the active source
            state.LaneRestrictionEdge = bestEdge;
            state.LaneRestrictionLane = bestLane;
        }
        else if (state.LaneRestrictionEdge >= 0)
        {
            // Toggle this outgoing lane's connection from the active incoming lane
            graph.ToggleLaneConnection(
                state.LaneRestrictionEdge, state.LaneRestrictionLane,
                bestEdge, bestLane);
        }
    }
}
