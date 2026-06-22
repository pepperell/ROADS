using Roads.App.World;

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
/// Which kind of spawn marker the Spawn tool places (selectable via its sub-menu).
/// </summary>
public enum SpawnKind
{
    /// <summary>Ordinary spawn point — origin for legacy/ambient random traffic.</summary>
    SpawnPoint,
    /// <summary>Region spawn — off-map origin residents drive in from on first appearance.</summary>
    RegionSpawn,
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

    /// <summary>POI type to assign when placing destination nodes (sticky across tool switches).</summary>
    public POIType SelectedPOIType { get; set; } = POIType.Home;

    /// <summary>Spawn marker kind to place with the Spawn tool (sticky across tool switches).</summary>
    public SpawnKind SelectedSpawnKind { get; set; } = SpawnKind.SpawnPoint;

    /// <summary>Index of the node where the current road segment starts, or <c>null</c> if not drawing.</summary>
    public int? RoadStartNode { get; set; }

    /// <summary>Whether a road segment is being drawn (i.e. <see cref="RoadStartNode"/> is set).</summary>
    public bool IsDrawingRoad => RoadStartNode.HasValue;

    /// <summary>Index of the currently selected edge, or -1 if none.</summary>
    public int SelectedEdge { get; set; } = -1;

    /// <summary>Index of the edge under the mouse cursor, or -1 if none.</summary>
    public int HoveredEdge { get; set; } = -1;

    /// <summary>Index of the node under the mouse cursor, or -1 if none.</summary>
    public int HoveredNode { get; set; } = -1;

    /// <summary>Index of the currently selected node, or -1 if none.</summary>
    public int SelectedNode { get; set; } = -1;

    /// <summary>Index of the vehicle under the mouse cursor, or -1 if none.</summary>
    public int HoveredVehicle { get; set; } = -1;

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

    /// <summary>
    /// Destination-placement ghost: world position of the new destination node (the cursor),
    /// or <c>null</c> when not in placement mode this frame (e.g. cursor is over an existing
    /// eligible node, or no nearby road exists). Set each mouse-move by the Destination hover case.
    /// </summary>
    public System.Numerics.Vector2? GhostDestPos { get; set; }

    /// <summary>
    /// Destination-placement ghost: world position of the perpendicular foot on the nearest
    /// edge (the connector's on-road end / future split point), or <c>null</c> when inactive.
    /// </summary>
    public System.Numerics.Vector2? GhostFootPos { get; set; }

    /// <summary>Nearest edge index for the placement ghost, or -1 when inactive.</summary>
    public int GhostEdge { get; set; } = -1;

    /// <summary>Parametric position of the foot on <see cref="GhostEdge"/> (already endpoint-clamped).</summary>
    public float GhostT { get; set; }

    /// <summary>
    /// Edge whose one-way cycle (the <c>O</c> key) is mid-progress, or -1. Tracks where the
    /// three-state cycle (two-way → one-way → one-way reversed → two-way) is, since the two
    /// one-way states are topologically identical. Reset lazily when a different edge is cycled.
    /// </summary>
    public int OneWayCycleEdge { get; set; } = -1;

    /// <summary>Step within the one-way cycle for <see cref="OneWayCycleEdge"/>: 0 = two-way,
    /// 1 = one-way (selected direction), 2 = one-way reversed.</summary>
    public int OneWayCycleStep { get; set; }

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
        HoveredEdge = -1;
        HoveredNode = -1;
        SelectedNode = -1;
        HoveredVehicle = -1;
        SelectedVehicle = -1;
        DragNodeIndex = -1;
        DragEdgeIndex = -1;
        DragControlPointIndex = -1;
        LaneRestrictionMode = false;
        LaneRestrictionEdge = -1;
        OneWayCycleEdge = -1;
        OneWayCycleStep = 0;
        GhostDestPos = null;
        GhostFootPos = null;
        GhostEdge = -1;
        GhostT = 0f;
    }
}
