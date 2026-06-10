using System.Numerics;
using SkiaSharp;
using Roads.App;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Rendering;

/// <summary>
/// Renders all active vehicles as colored rounded rectangles with a translucent windshield.
/// Each vehicle is drawn at its world position rotated to its heading angle.
/// </summary>
public class VehicleRenderer
{
    /// <summary>Vehicle body length in meters.</summary>
    private const float VehicleLength = SimConstants.VehicleLength;
    /// <summary>Vehicle body width in meters.</summary>
    private const float VehicleWidth = SimConstants.VehicleWidth;

    /// <summary>When true, draws red circles on vehicles stuck on arcs and magenta lines between pairs at the same node.</summary>
    public static bool ShowArcConflicts { get; set; }

    // Pre-allocated headlight cone paths (reused every frame, same shape in local vehicle space)
    private readonly SKPath _leftBeam;
    private readonly SKPath _rightBeam;

    public VehicleRenderer()
    {
        float halfL = VehicleLength * 0.5f;
        float beamLength = 15f;

        _leftBeam = new SKPath();
        _leftBeam.MoveTo(halfL, -0.4f);
        _leftBeam.LineTo(halfL + beamLength, -3.5f);
        _leftBeam.LineTo(halfL + beamLength, 0f);
        _leftBeam.Close();

        _rightBeam = new SKPath();
        _rightBeam.MoveTo(halfL, 0.4f);
        _rightBeam.LineTo(halfL + beamLength, 0f);
        _rightBeam.LineTo(halfL + beamLength, 3.5f);
        _rightBeam.Close();
    }

    /// <summary>
    /// Draws all active vehicles as colored rounded rectangles with windshield,
    /// headlight beams at night, brake lights when braking, and tail lights at night.
    /// </summary>
    public void Draw(SKCanvas canvas, VehicleStore store, float zoom, float darkness)
    {
        if (store.Count == 0) return;

        bool drawHeadlights = darkness > 0.05f;
        float ambient = 1f - darkness * 0.4f; // darken vehicle bodies at night

        using var bodyPaint = new SKPaint { Style = SKPaintStyle.Fill, IsAntialias = true };
        using var windshieldPaint = new SKPaint
        {
            Color = new SKColor(140, 200, 255, 180),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };

        // Headlight beam paint (additive blending + gradient fade with distance)
        float halfL = VehicleLength * 0.5f;
        byte beamAlpha = (byte)(darkness * 35);
        using var beamShader = drawHeadlights ? SKShader.CreateLinearGradient(
            new SKPoint(halfL, 0),
            new SKPoint(halfL + 15f, 0),
            new SKColor[] { new SKColor(255, 245, 210, beamAlpha), new SKColor(255, 245, 210, 0) },
            new float[] { 0f, 1f },
            SKShaderTileMode.Clamp) : null;
        using var beamPaint = drawHeadlights ? new SKPaint
        {
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
            BlendMode = SKBlendMode.Plus,
            Shader = beamShader,
        } : null;

        // Headlight dot paint (bright point sources)
        using var headlightDotPaint = drawHeadlights ? new SKPaint
        {
            Color = new SKColor(255, 250, 220, (byte)(120 + darkness * 135)),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        } : null;

        // Tail light paint (dim red, always on at night)
        using var tailPaint = drawHeadlights ? new SKPaint
        {
            Color = new SKColor(180, 20, 10, (byte)(darkness * 60)),
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        } : null;

        // Brake light paint (bright red, intensity varies)
        using var brakePaint = new SKPaint
        {
            Style = SKPaintStyle.Fill,
            IsAntialias = true,
        };

        float halfW = VehicleWidth * 0.5f;

        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;

            canvas.Save();
            canvas.Translate(store.PosX[i], store.PosY[i]);
            canvas.RotateRadians(store.Heading[i]);

            // 1. Headlight beams (behind body so car sits on top)
            if (drawHeadlights)
            {
                canvas.DrawPath(_leftBeam, beamPaint!);
                canvas.DrawPath(_rightBeam, beamPaint!);
            }

            // 2. Car body with per-vehicle color (darkened at night)
            bodyPaint.Color = new SKColor(
                (byte)(store.ColorR[i] * ambient),
                (byte)(store.ColorG[i] * ambient),
                (byte)(store.ColorB[i] * ambient));
            canvas.DrawRoundRect(-halfL, -halfW, VehicleLength, VehicleWidth, 0.6f, 0.6f, bodyPaint);

            // 3. Windshield (front quarter)
            canvas.DrawRect(halfL * 0.3f, -halfW + 0.3f, halfL * 0.5f, VehicleWidth - 0.6f, windshieldPaint);

            // 4. Headlight dots at front corners
            if (drawHeadlights)
            {
                canvas.DrawCircle(halfL, -halfW * 0.4f, 0.25f, headlightDotPaint!);
                canvas.DrawCircle(halfL, halfW * 0.4f, 0.25f, headlightDotPaint!);
            }

            // 5. Brake lights at rear corners (when braking)
            if (store.Brake[i] > 0.1f)
            {
                brakePaint.Color = new SKColor(255, 30, 20, 230);
                canvas.DrawRect(-halfL, -halfW * 0.7f - 0.15f, 0.3f, 0.3f, brakePaint);
                canvas.DrawRect(-halfL, halfW * 0.7f - 0.15f, 0.3f, 0.3f, brakePaint);
            }
            // 6. Tail lights (dim red at night, even when not braking)
            else if (drawHeadlights)
            {
                canvas.DrawRect(-halfL, -halfW * 0.7f - 0.15f, 0.3f, 0.3f, tailPaint!);
                canvas.DrawRect(-halfL, halfW * 0.7f - 0.15f, 0.3f, 0.3f, tailPaint!);
            }

            canvas.Restore();
        }
    }

    /// <summary>
    /// Draws a hover highlight around the vehicle under the cursor.
    /// </summary>
    public void DrawHoverOverlay(SKCanvas canvas, VehicleStore store, int index)
    {
        if (index < 0 || index >= store.Count) return;
        if (store.State[index] != VehicleState.Driving) return;

        float halfL = VehicleLength * 0.5f;
        float halfW = VehicleWidth * 0.5f;

        canvas.Save();
        canvas.Translate(store.PosX[index], store.PosY[index]);
        canvas.RotateRadians(store.Heading[index]);

        using var hoverPaint = new SKPaint
        {
            Color = new SKColor(255, 255, 100, 120),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 0.35f,
            IsAntialias = true,
        };
        canvas.DrawRoundRect(-halfL - 0.3f, -halfW - 0.3f,
            VehicleLength + 0.6f, VehicleWidth + 0.6f, 0.8f, 0.8f, hoverPaint);
        canvas.Restore();
    }

    /// <summary>
    /// Draws a selection highlight around the selected vehicle, a lookahead target dot,
    /// and a preview of the remaining path edges.
    /// </summary>
    /// <param name="canvas">SkiaSharp canvas in world-space coordinates.</param>
    /// <param name="store">Vehicle data store.</param>
    /// <param name="index">Index of the selected vehicle.</param>
    /// <param name="graph">Road graph for Bezier evaluation.</param>
    public void DrawSelectionOverlay(SKCanvas canvas, VehicleStore store, int index, RoadGraph graph, StopLineCache stopLines, IntersectionArcCache arcCache)
    {
        if (index < 0 || index >= store.Count) return;
        if (store.State[index] != VehicleState.Driving) return;

        float halfL = VehicleLength * 0.5f;
        float halfW = VehicleWidth * 0.5f;

        // Selection ring around vehicle
        canvas.Save();
        canvas.Translate(store.PosX[index], store.PosY[index]);
        canvas.RotateRadians(store.Heading[index]);

        using var selectPaint = new SKPaint
        {
            Color = new SKColor(100, 200, 255, 150),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 0.4f,
            IsAntialias = true,
        };
        canvas.DrawRoundRect(-halfL - 0.3f, -halfW - 0.3f,
            VehicleLength + 0.6f, VehicleWidth + 0.6f, 0.8f, 0.8f, selectPaint);
        canvas.Restore();

        // Lookahead target dot
        int edgeIdx = store.CurrentEdge[index];
        int currentArc = store.CurrentArc[index];

        if (currentArc >= 0)
        {
            // Vehicle is on an arc — draw lookahead on the arc
            float speed = store.Speed[index];
            float arcProgress = store.ArcProgress[index];
            var arc = arcCache.GetArc(currentArc);
            float arcLength = MathF.Max(arc.Length, 0.01f);

            float lookahead = 3f + speed * 0.3f;
            float lookaheadT = arcProgress + lookahead / arcLength;
            lookaheadT = MathF.Min(lookaheadT, 1f);

            var targetPos = arcCache.EvaluateArc(currentArc, lookaheadT);

            using var dotPaint = new SKPaint
            {
                Color = new SKColor(255, 220, 50, 200),
                Style = SKPaintStyle.Fill,
                IsAntialias = true,
            };
            canvas.DrawCircle(targetPos.X, targetPos.Y, 0.8f, dotPaint);
        }
        else if (edgeIdx >= 0 && edgeIdx < graph.Edges.Count && graph.Edges[edgeIdx].FromNode >= 0)
        {
            float edgeLength = graph.Edges[edgeIdx].Length;
            if (edgeLength < 0.01f) edgeLength = 0.01f;

            // Reuse shared lookahead computation (no collinearity check for rendering)
            var targetPos = SteeringController.ComputeEdgeLookahead(
                store, index, graph, edgeIdx, store.EdgeProgress[index], edgeLength, checkCollinearity: false);

            using var dotPaint = new SKPaint
            {
                Color = new SKColor(255, 220, 50, 200),
                Style = SKPaintStyle.Fill,
                IsAntialias = true,
            };
            canvas.DrawCircle(targetPos.X, targetPos.Y, 0.8f, dotPaint);
        }

        // Path preview — draw remaining edges and connecting arcs
        var path = store.Path[index];
        int pathIdx = store.PathIndex[index];
        if (path != null)
        {
            using var pathPaint = new SKPaint
            {
                Color = new SKColor(100, 200, 255, 80),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1.0f,
                IsAntialias = true,
            };

            const int segments = 12;
            byte currentLane = store.CurrentLane[index];

            for (int p = pathIdx + 1; p < path.Count; p++)
            {
                int eidx = path[p];
                if (eidx < 0 || eidx >= graph.Edges.Count) continue;
                if (graph.Edges[eidx].FromNode < 0) continue;

                // Draw connecting arc from previous edge to this edge
                if (p > pathIdx + 1 || (p == pathIdx + 1 && currentArc < 0))
                {
                    int prevEdge = path[p - 1];
                    if (prevEdge >= 0 && prevEdge < graph.Edges.Count && graph.Edges[prevEdge].FromNode >= 0)
                    {
                        byte outLane = (byte)Math.Min(currentLane, graph.Edges[eidx].LaneCount - 1);
                        int arcIdx = arcCache.GetArcIndex(prevEdge, eidx, currentLane, outLane);
                        if (arcIdx < 0)
                            arcIdx = arcCache.GetArcIndex(prevEdge, eidx, currentLane, currentLane);
                        // Try any lane combo as fallback
                        if (arcIdx < 0)
                        {
                            for (byte ti = 0; ti < graph.Edges[prevEdge].LaneCount && arcIdx < 0; ti++)
                                for (byte to = 0; to < graph.Edges[eidx].LaneCount && arcIdx < 0; to++)
                                    arcIdx = arcCache.GetArcIndex(prevEdge, eidx, ti, to);
                        }

                        if (arcIdx >= 0)
                        {
                            var arcPath = new SKPath();
                            var arcStart = arcCache.EvaluateArc(arcIdx, 0f);
                            arcPath.MoveTo(arcStart.X, arcStart.Y);
                            for (int s = 1; s <= segments; s++)
                            {
                                float t = s / (float)segments;
                                var pt = arcCache.EvaluateArc(arcIdx, t);
                                arcPath.LineTo(pt.X, pt.Y);
                            }
                            canvas.DrawPath(arcPath, pathPaint);
                            arcPath.Dispose();
                        }
                    }
                }

                // Draw the current arc the vehicle is traversing
                if (p == pathIdx + 1 && currentArc >= 0)
                {
                    var arcPath = new SKPath();
                    var arcStart = arcCache.EvaluateArc(currentArc, 0f);
                    arcPath.MoveTo(arcStart.X, arcStart.Y);
                    for (int s = 1; s <= segments; s++)
                    {
                        float t = s / (float)segments;
                        var pt = arcCache.EvaluateArc(currentArc, t);
                        arcPath.LineTo(pt.X, pt.Y);
                    }
                    canvas.DrawPath(arcPath, pathPaint);
                    arcPath.Dispose();
                }

                // Draw the edge itself
                var skPath = new SKPath();
                var start = graph.EvaluateBezier(eidx, 0f);
                skPath.MoveTo(start.X, start.Y);
                for (int s = 1; s <= segments; s++)
                {
                    float t = s / (float)segments;
                    var pt = graph.EvaluateBezier(eidx, t);
                    skPath.LineTo(pt.X, pt.Y);
                }
                canvas.DrawPath(skPath, pathPaint);
                skPath.Dispose();
            }
        }
    }

    /// <summary>
    /// Draws debug overlay highlighting overlapping vehicles: red circles on stuck-on-arc vehicles,
    /// orange circles on overlapping edge vehicles (full brake, nearly stopped, same edge within
    /// one vehicle length), and magenta lines connecting overlapping pairs.
    /// </summary>
    public void DrawArcConflictOverlay(SKCanvas canvas, VehicleStore store, IntersectionArcCache arcCache)
    {
        if (!ShowArcConflicts || store.Count == 0) return;

        using var arcStuckPaint = new SKPaint
        {
            Color = new SKColor(255, 40, 40, 180),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 0.5f,
            IsAntialias = true,
        };

        using var edgeOverlapPaint = new SKPaint
        {
            Color = new SKColor(255, 160, 0, 200),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 0.5f,
            IsAntialias = true,
        };

        using var linePaint = new SKPaint
        {
            Color = new SKColor(255, 0, 255, 200),
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 0.4f,
            IsAntialias = true,
        };

        // Arc-stuck vehicles (red circles)
        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            if (store.CurrentArc[i] < 0) continue;
            if (store.Brake[i] < 0.99f || store.Speed[i] >= 0.5f) continue;
            canvas.DrawCircle(store.PosX[i], store.PosY[i], 3f, arcStuckPaint);
        }

        // Edge overlap detection: find pairs of braking vehicles on the same edge
        // and same lane that are within 2x vehicle length of each other
        for (int i = 0; i < store.Count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            if (store.CurrentArc[i] >= 0) continue;
            if (store.Brake[i] < 0.8f) continue;
            if (store.Speed[i] >= 1.0f) continue;

            for (int j = i + 1; j < store.Count; j++)
            {
                if (store.State[j] != VehicleState.Driving) continue;
                if (store.CurrentArc[j] >= 0) continue;
                if (store.Brake[j] < 0.8f) continue;
                if (store.Speed[j] >= 1.0f) continue;
                if (store.CurrentEdge[j] != store.CurrentEdge[i]) continue;
                if (store.CurrentLane[j] != store.CurrentLane[i]) continue;

                float dx = store.PosX[i] - store.PosX[j];
                float dy = store.PosY[i] - store.PosY[j];
                float dist = MathF.Sqrt(dx * dx + dy * dy);
                if (dist < VehicleLength * 2f)
                {
                    // Overlapping/jammed pair on same edge + lane
                    canvas.DrawCircle(store.PosX[i], store.PosY[i], 3f, edgeOverlapPaint);
                    canvas.DrawCircle(store.PosX[j], store.PosY[j], 3f, edgeOverlapPaint);
                    canvas.DrawLine(store.PosX[i], store.PosY[i],
                        store.PosX[j], store.PosY[j], linePaint);
                }
            }
        }
    }

}
