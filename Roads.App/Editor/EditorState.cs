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
    /// <summary>Paints the visual water layer (brush dabs, stream segments, eraser —
    /// see <see cref="WaterMode"/>). Water never touches the road graph.</summary>
    Water,
}

/// <summary>Sub-mode of the Water tool (the water-toolbar mode group).</summary>
public enum WaterMode
{
    /// <summary>Paint circular dabs on click/drag.</summary>
    Brush,
    /// <summary>Draw stream segments as a click chain (straight or curved), like the Road tool.</summary>
    Stream,
    /// <summary>Remove intersecting water primitives on click/drag.</summary>
    Erase,
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

    /// <summary>True while the menu bar's Visibility submenu (panel/overlay toggles) is
    /// open. Toggled by the Visibility button; gates the submenu's VisibleWhen. Not
    /// touched by <see cref="ResetToolState"/> — the menu is independent of tool state.
    /// The MenuBar keeps this and <see cref="WorldSettingsMenuOpen"/> mutually exclusive
    /// (the two submenus would overlap).</summary>
    public bool VisibilityMenuOpen { get; set; }

    /// <summary>True while the menu bar's World Settings submenu (per-world spawn tuning,
    /// saved in the map file) is open. Same lifecycle as <see cref="VisibilityMenuOpen"/>.</summary>
    public bool WorldSettingsMenuOpen { get; set; }

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

    /// <summary>Curved drawing mode for the Road tool (sticky). When set, each new segment
    /// leaves its start node TANGENT to the previous segment (or to the road being
    /// continued at a dead-end start node), forming a smooth arc toward the clicked end;
    /// when clear, segments are straight lines (the classic behavior). Segments with no
    /// tangent reference (first segment of a chain in open space) are straight either way.</summary>
    public bool SelectedCurved { get; set; }

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

    /// <summary>The last segment committed by the CURRENT road chain (the trailing half
    /// after crossing splits, so its ToNode is the chain's start node), or -1. Curved mode
    /// reads its end tangent so the next segment continues smoothly.</summary>
    public int RoadPrevEdge { get; set; } = -1;

    /// <summary>The legs the road tool's NEXT click will commit — the pass-through route
    /// from the chain start through every existing node within snap distance of the drawn
    /// geometry to the snapped end anchor (a single leg when nothing is passed over).
    /// Recomputed each mouse-move while drawing (<see cref="RoadTool.PlanPreviewLegs"/>);
    /// the preview band, centerline, and pass-through junction ghosts render from this
    /// list. Empty when not drawing.</summary>
    public List<RoadTool.PreviewLeg> RoadPreviewLegs { get; } = new();

    /// <summary>Ghost positions of the intersection nodes the in-progress road will
    /// create where its planned legs cross existing roads — the route planner's PENDING
    /// split points, already min-node-distance adjusted (slid along their host edges), so
    /// each ghost sits exactly where the committed node will. Recomputed each mouse-move
    /// while drawing; empty otherwise.</summary>
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

    // ── Water-toolbar options (sticky across tool switches, like the road options) ──

    /// <summary>Active sub-mode of the Water tool (sticky).</summary>
    public WaterMode WaterMode { get; set; } = WaterMode.Brush;

    /// <summary>Brush/eraser radius in meters (sticky; the water-toolbar size presets).
    /// Stream segments are drawn 2× this wide, so an S stream matches an S dab's diameter.</summary>
    public float WaterBrushRadius { get; set; } = 8f;

    /// <summary>Curved drawing mode for water streams (sticky; independent of the road
    /// tool's <see cref="SelectedCurved"/>). Same convention: each new segment leaves
    /// its start tangent to the previous segment of the chain.</summary>
    public bool WaterCurved { get; set; }

    // ── Water tool transient state (cleared by ResetToolState) ──

    /// <summary>Whether a brush/erase drag stroke is in progress.</summary>
    public bool IsPaintingWater { get; set; }

    /// <summary>Position of the last brush dab of the in-progress stroke (spacing gate), or null.</summary>
    public System.Numerics.Vector2? WaterLastDabPos { get; set; }

    /// <summary>Pending start point of the next stream segment (the stream chain anchor), or null.</summary>
    public System.Numerics.Vector2? WaterStreamAnchor { get; set; }

    /// <summary>End tangent of the chain's previous stream segment (unit), or null on the
    /// first segment — curved mode's tangent reference, mirroring RoadPrevEdge.</summary>
    public System.Numerics.Vector2? WaterStreamPrevDir { get; set; }

    /// <summary>Water-tool hover ghost: cursor world position (brush/erase circle preview,
    /// stream pending-segment end), or null when inactive (over UI, mid-pan).</summary>
    public System.Numerics.Vector2? WaterGhostPos { get; set; }

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
        RoadPrevEdge = -1;
        RoadPreviewLegs.Clear();
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
        IsPaintingWater = false;
        WaterLastDabPos = null;
        WaterStreamAnchor = null;
        WaterStreamPrevDir = null;
        WaterGhostPos = null;
    }
}
