using Roads.App.World;

namespace Roads.App.Editor;

/// <summary>
/// Available editor tools, selectable via the toolbar.
/// </summary>
public enum EditorTool
{
    Select,
    Road,
    Node,
    Delete,
    Destination,
    Signal,
    /// <summary>Toggles a traffic light between fixed-time and actuated control
    /// (badges over every light show the current type while this tool is active).</summary>
    SignalControl,
    /// <summary>Applies the road-toolbar options (type, width, one-way, shared-lane)
    /// to a clicked segment.</summary>
    UpdateSegment,
    /// <summary>Rotates a traffic light's phase grouping on click (the signals-toolbar
    /// "Rotate" tool).</summary>
    SignalRotate,
    /// <summary>Toggles the exemption of the clicked stop/yield approach (the
    /// signals-toolbar "Exempt" tool; an exempt approach does not stop at its
    /// node's sign).</summary>
    SignalExempt,
}

/// <summary>
/// Shared mutable state for the editor: which tool is active, road-drawing progress,
/// selected edge, and control-point drag state.
/// </summary>
public class EditorState
{
    /// <summary>World-space distance threshold for snapping to nodes and selecting edges.</summary>
    public const float SnapDistance = 15f;

    /// <summary>
    /// World-space distance threshold for picking a node under the cursor (hover highlight
    /// and node-targeted clicks). Tighter than <see cref="SnapDistance"/> so nearby edges
    /// stay clickable right up close to a node.
    /// </summary>
    public const float NodePickDistance = 5f;

    /// <summary>Currently active editor tool.</summary>
    public EditorTool ActiveTool { get; set; } = EditorTool.Select;

    /// <summary>POI type to assign when placing destination nodes (sticky across tool switches).</summary>
    public POIType SelectedPOIType { get; set; } = POIType.Home;

    // ── Road-toolbar options (sticky across tool switches, like SelectedPOIType) ──
    // Applied to newly drawn roads by RoadTool and to clicked segments by UpdateSegmentTool.
    // Mutual exclusion is enforced in the setters so the invariants hold for every caller:
    // one-way and shared-lane are exclusive, and shared-lane forces a 1-lane width.

    /// <summary>Road type for new roads and the Update Segment tool.</summary>
    public RoadType SelectedRoadType { get; set; } = RoadType.Arterial;

    private byte _selectedLaneCount = 1;
    /// <summary>Per-direction lane count (1–3) for new roads and the Update Segment tool.
    /// Pinned to 1 while <see cref="SelectedSharedLane"/> is set.</summary>
    public byte SelectedLaneCount
    {
        get => _selectedLaneCount;
        set => _selectedLaneCount = _selectedSharedLane ? (byte)1 : Math.Clamp(value, (byte)1, (byte)3);
    }

    private bool _selectedOneWay;
    /// <summary>One-way option for new roads and the Update Segment tool; mutually
    /// exclusive with <see cref="SelectedSharedLane"/>.</summary>
    public bool SelectedOneWay
    {
        get => _selectedOneWay;
        set
        {
            _selectedOneWay = value;
            if (value) _selectedSharedLane = false;
        }
    }

    private bool _selectedSharedLane;
    /// <summary>Single-lane two-way (shared-lane) option for new roads and the Update
    /// Segment tool; forces a 1-lane width and excludes <see cref="SelectedOneWay"/>.</summary>
    public bool SelectedSharedLane
    {
        get => _selectedSharedLane;
        set
        {
            _selectedSharedLane = value;
            if (value)
            {
                _selectedOneWay = false;
                _selectedLaneCount = 1;
            }
        }
    }

    /// <summary>Index of the EXISTING node the current road segment starts from, or <c>null</c>
    /// when the start is a pending ghost anchor (<see cref="RoadStartAnchorPos"/>) or no
    /// road is being drawn.</summary>
    public int? RoadStartNode { get; set; }

    /// <summary>Edge carrying a PENDING mid-road start anchor (drawn as a ghost node), or -1.
    /// The split is deferred until the segment commits on the second click, so canceling
    /// the road leaves the graph untouched.</summary>
    public int RoadStartEdge { get; set; } = -1;

    /// <summary>Clamped split parameter on <see cref="RoadStartEdge"/> for the pending anchor.</summary>
    public float RoadStartT { get; set; }

    /// <summary>World position of a PENDING start anchor — the on-road split point or the
    /// free-space click — used for the ghost preview and as the commit fallback. <c>null</c>
    /// when the start is an existing node or no road is being drawn.</summary>
    public System.Numerics.Vector2? RoadStartAnchorPos { get; set; }

    /// <summary>Whether a road segment is being drawn (from an existing node or a pending ghost anchor).</summary>
    public bool IsDrawingRoad => RoadStartNode.HasValue || RoadStartAnchorPos.HasValue;

    /// <summary>Ghost positions of the intersection nodes the in-progress road segment
    /// will create where its preview line crosses existing roads. Recomputed each
    /// mouse-move while drawing; empty otherwise.</summary>
    public List<System.Numerics.Vector2> RoadCrossingPreviews { get; } = new();

    /// <summary>
    /// Road-tool hover ghost: world position where a click would anchor (the snapped
    /// existing node, the clamped on-road split point, or the raw cursor in empty space).
    /// Shown at ALL times with the Road tool — before the first click it previews the
    /// chain start; while drawing it previews the segment end. <c>null</c> when inactive
    /// (over UI, mid-pan); recomputed each mouse-move.
    /// </summary>
    public System.Numerics.Vector2? RoadAnchorGhostPos { get; set; }

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
    /// Node-tool placement ghost: world position where a click would create the node (the
    /// snapped on-road split position, or the raw cursor in empty space), or <c>null</c>
    /// when inactive (cursor over an existing node, over UI, or mid-pan). Recomputed each
    /// mouse-move by the Node hover case.
    /// </summary>
    public System.Numerics.Vector2? NodeGhostPos { get; set; }

    /// <summary>Ghost radius accompanying <see cref="NodeGhostPos"/>: the half-width of
    /// the road the click would split, or 0 for a free node (the renderer floors it at
    /// the node-dot size).</summary>
    public float NodeGhostRadius { get; set; }

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
        RoadStartEdge = -1;
        RoadStartAnchorPos = null;
        RoadCrossingPreviews.Clear();
        RoadAnchorGhostPos = null;
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
        NodeGhostPos = null;
        NodeGhostRadius = 0f;
    }
}
