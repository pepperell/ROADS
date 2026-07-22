using SkiaSharp;
using Roads.App.Editor;
using Roads.App.World;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Read-only details for the current node or edge selection (Select tool), under a
/// SELECTION title row (matching the statistics panel): node flags and lane-restrict mode
/// state, or edge lane count / speed limit / road type. Deliberately contains NO
/// keyboard-shortcut hints — those live in the pause menu's legend. Hidden entirely while
/// nothing valid is selected (live <see cref="Panel.VisibleWhen"/> gate; the bottom-left
/// stack skips hidden children, so the stack collapses); height follows the line count
/// via <see cref="Measure"/> with a reused line list.
/// </summary>
public class SelectionInfoPanel : Panel
{
    private const float PanelWidth = 256f;
    private const float Pad = 8f;
    private const float LineHeight = 16f;

    private readonly RoadGraph _graph;
    private readonly EditorState _editorState;
    private readonly List<string> _lines = new(6);

    private readonly SKPaint _titlePaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _labelPaint = new() { Color = new SKColor(200, 200, 200), IsAntialias = true };
    private readonly SKPaint _headerPaint = new() { Color = new SKColor(100, 200, 255), IsAntialias = true };

    public SelectionInfoPanel(RoadGraph graph, EditorState editorState)
    {
        _graph = graph;
        _editorState = editorState;
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;
        Size = new SKSize(PanelWidth, Pad * 2f);
        VisibleWhen = () => HasValidNodeSelection() || HasValidEdgeSelection();
    }

    private bool HasValidNodeSelection()
        => _editorState.SelectedNode >= 0 && _editorState.SelectedNode < _graph.Nodes.Count
           && !float.IsNaN(_graph.Nodes[_editorState.SelectedNode].Position.X);

    private bool HasValidEdgeSelection()
        => _editorState.SelectedEdge >= 0 && _editorState.SelectedEdge < _graph.Edges.Count
           && _graph.Edges[_editorState.SelectedEdge].FromNode >= 0;

    public override void Measure(float canvasWidth, float canvasHeight)
    {
        BuildLines();
        // Title row + content lines (same vertical rhythm as the statistics panel).
        Size = new SKSize(PanelWidth, Pad + 10f + _lines.Count * LineHeight + Pad);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        float textY = Bounds.Top + Pad + 10f;
        canvas.DrawText("SELECTION", Bounds.Left + Pad, textY, SKTextAlign.Left, UiTheme.Font11, _titlePaint);

        for (int i = 0; i < _lines.Count; i++)
        {
            var paint = i == 0 ? _headerPaint : _labelPaint;
            canvas.DrawText(_lines[i], Bounds.Left + Pad,
                textY + (i + 1) * LineHeight, SKTextAlign.Left, UiTheme.Font12, paint);
        }
    }

    private void BuildLines()
    {
        _lines.Clear();

        if (HasValidNodeSelection())
        {
            var node = _graph.Nodes[_editorState.SelectedNode];
            _lines.Add($"Node #{_editorState.SelectedNode}");
            _lines.Add($"Flags: {(node.Flags == NodeFlags.None ? "none" : node.Flags.ToString())}");
            if (_editorState.LaneRestrictionMode)
            {
                _lines.Add("LANE RESTRICT");
                _lines.Add(_editorState.LaneRestrictionEdge >= 0
                    ? $"Input Edge {_editorState.LaneRestrictionEdge}  Lane {_editorState.LaneRestrictionLane}"
                    : "click input lane");
            }
        }
        else if (HasValidEdgeSelection())
        {
            var edge = _graph.Edges[_editorState.SelectedEdge];
            _lines.Add($"Edge #{_editorState.SelectedEdge}");
            _lines.Add($"Lanes: {edge.LaneCount}");
            _lines.Add($"Speed: {edge.SpeedLimit * 2.23694f:F0} mph");
            _lines.Add($"Type: {edge.RoadType}");
        }
        // No else: with neither selection valid the VisibleWhen gate hides the panel.
    }
}
