using System.Numerics;
using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Editor tool that paints the visual water layer. Stateless like the other tools —
/// all mutable state lives in <see cref="EditorState"/> (sticky: mode/size/curved;
/// transient: stroke and chain progress). Three sub-modes (<see cref="WaterMode"/>):
/// <list type="bullet">
/// <item><b>Brush</b> — stamps a dab on click and keeps stamping along a left-drag,
/// gated to one dab per half-radius of travel so stroke density is bounded.</item>
/// <item><b>Stream</b> — a click chain like the Road tool: the first click anchors,
/// each further click commits one <see cref="WaterSegment"/> (width = 2× brush radius,
/// so width can vary mid-chain via the size presets). Curved mode reuses
/// <see cref="RoadTool.ComputeCurveControls"/> with the previous segment's end tangent;
/// the first segment of a chain is straight (no tangent reference), matching the Road
/// tool's open-space behavior.</item>
/// <item><b>Erase</b> — removes intersecting primitives on click/drag.</item>
/// </list>
/// </summary>
public class WaterTool
{
    /// <summary>Minimum stream segment chord in meters — shorter commits are ignored
    /// (the Road tool's analog is its same-node check).</summary>
    private const float MinSegmentChord = 0.5f;

    /// <summary>Handles a left-button press for the active water sub-mode.</summary>
    public void OnClick(Vector2 worldPos, WaterLayer water, EditorState state)
    {
        switch (state.WaterMode)
        {
            case WaterMode.Brush:
                water.AddCircle(worldPos, state.WaterBrushRadius);
                state.IsPaintingWater = true;
                state.WaterLastDabPos = worldPos;
                break;

            case WaterMode.Erase:
                water.EraseAt(worldPos, state.WaterBrushRadius);
                state.IsPaintingWater = true;   // drag keeps erasing
                state.WaterLastDabPos = worldPos;
                break;

            case WaterMode.Stream:
                CommitStreamPoint(worldPos, water, state);
                break;
        }
    }

    /// <summary>
    /// Handles cursor travel during an in-progress brush/erase stroke
    /// (<see cref="EditorState.IsPaintingWater"/>). Stamps or erases when the cursor
    /// has moved far enough from the last action position.
    /// </summary>
    public void OnDrag(Vector2 worldPos, WaterLayer water, EditorState state)
    {
        if (!state.IsPaintingWater) return;

        float spacing = state.WaterMode == WaterMode.Brush
            ? state.WaterBrushRadius * 0.5f   // dabs overlap heavily → smooth blob
            : state.WaterBrushRadius * 0.25f; // eraser sweeps denser so nothing is skipped
        if (state.WaterLastDabPos is { } last && Vector2.Distance(last, worldPos) < spacing)
            return;

        if (state.WaterMode == WaterMode.Brush)
            water.AddCircle(worldPos, state.WaterBrushRadius);
        else if (state.WaterMode == WaterMode.Erase)
            water.EraseAt(worldPos, state.WaterBrushRadius);
        state.WaterLastDabPos = worldPos;
    }

    /// <summary>
    /// Stream-mode click: anchor the chain on the first click, commit one segment per
    /// further click. Mirrors RoadTool's curved-mode convention with
    /// <see cref="EditorState.WaterStreamPrevDir"/> standing in for RoadPrevEdge.
    /// </summary>
    private static void CommitStreamPoint(Vector2 worldPos, WaterLayer water, EditorState state)
    {
        if (state.WaterStreamAnchor is not { } anchor)
        {
            state.WaterStreamAnchor = worldPos;
            state.WaterStreamPrevDir = null;
            return;
        }

        var chord = worldPos - anchor;
        float d = chord.Length();
        if (d < MinSegmentChord) return;

        Vector2 c1, c2;
        if (state.WaterCurved && state.WaterStreamPrevDir is { } prevDir)
        {
            (c1, c2) = RoadTool.ComputeCurveControls(anchor, worldPos, prevDir);
        }
        else
        {
            // Straight thirds (always stored, never degenerate, so the end tangent below is defined).
            c1 = anchor + chord * (1f / 3f);
            c2 = anchor + chord * (2f / 3f);
        }

        water.AddSegment(anchor, c1, c2, worldPos, 2f * state.WaterBrushRadius);

        // Cubic end tangent direction is P3 − C2 (the 3× magnitude factor cancels in Normalize).
        state.WaterStreamPrevDir = Vector2.Normalize(worldPos - c2);
        state.WaterStreamAnchor = worldPos;
    }
}
