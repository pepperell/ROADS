namespace Roads.App.Editor;

/// <summary>
/// Available editor tools, selectable via the toolbar.
/// </summary>
public enum EditorTool
{
    Select,
    Road,
    Delete,
    SpawnPoint,
    Destination,
    Signal,
}

/// <summary>
/// Shared mutable state for the editor: which tool is active, road-drawing progress,
/// selected edge, and control-point drag state.
/// </summary>
public class EditorState
{
    /// <summary>World-space distance threshold for snapping to nodes and selecting edges.</summary>
    public const float SnapDistance = 15f;

    /// <summary>Currently active editor tool.</summary>
    public EditorTool ActiveTool { get; set; } = EditorTool.Select;

    /// <summary>Index of the node where the current road segment starts, or <c>null</c> if not drawing.</summary>
    public int? RoadStartNode { get; set; }

    /// <summary>Whether a road segment is being drawn (i.e. <see cref="RoadStartNode"/> is set).</summary>
    public bool IsDrawingRoad => RoadStartNode.HasValue;

    /// <summary>Index of the currently selected edge, or -1 if none.</summary>
    public int SelectedEdge { get; set; } = -1;

    /// <summary>Index of the currently selected node, or -1 if none.</summary>
    public int SelectedNode { get; set; } = -1;

    /// <summary>Index of the currently selected vehicle, or -1 if none.</summary>
    public int SelectedVehicle { get; set; } = -1;

    /// <summary>Index of the node being dragged, or -1 if not dragging.</summary>
    public int DragNodeIndex { get; set; } = -1;

    /// <summary>Whether a node drag is in progress.</summary>
    public bool IsDraggingNode => DragNodeIndex >= 0;

    /// <summary>Index of the edge whose control point is being dragged, or -1 if not dragging.</summary>
    public int DragEdgeIndex { get; set; } = -1;

    /// <summary>Which Bézier control point is being dragged (1 or 2), or -1 if not dragging.</summary>
    public int DragControlPointIndex { get; set; } = -1;

    /// <summary>Whether a control point drag is in progress.</summary>
    public bool IsDraggingControlPoint => DragEdgeIndex >= 0;

    /// <summary>Preview positions for intersections detected during node drag.</summary>
    public List<System.Numerics.Vector2> DragCrossingPreviews { get; } = new();

    /// <summary>Whether lane restriction editing mode is active (node must be selected).</summary>
    public bool LaneRestrictionMode { get; set; }

    /// <summary>Incoming edge whose lane is selected in lane restriction mode, or -1.</summary>
    public int LaneRestrictionEdge { get; set; } = -1;

    /// <summary>Lane index on the incoming edge selected in lane restriction mode.</summary>
    public byte LaneRestrictionLane { get; set; }

    /// <summary>Clears all tool-specific state when switching tools.</summary>
    public void ResetToolState()
    {
        RoadStartNode = null;
        SelectedEdge = -1;
        SelectedNode = -1;
        SelectedVehicle = -1;
        DragNodeIndex = -1;
        DragEdgeIndex = -1;
        DragControlPointIndex = -1;
        LaneRestrictionMode = false;
        LaneRestrictionEdge = -1;
    }
}
