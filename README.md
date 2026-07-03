# Roads: City-Scale Vehicle Traffic Simulation

A real-time, city-scale traffic simulation built in C# with a graphical editor. Thousands of individually-simulated vehicles with physics-based steering navigate a user-editable road network, obeying traffic rules and following daily routines.

![Platform: Windows](https://img.shields.io/badge/platform-Windows-blue)
![.NET 8](https://img.shields.io/badge/.NET-8.0-purple)
![Renderer: SkiaSharp](https://img.shields.io/badge/renderer-SkiaSharp-green)

---

## Features

- **Real-time simulation** of thousands of vehicles at 30 Hz fixed timestep, decoupled from 60 FPS rendering
- **Physics-based driving** using the bicycle model (simplified Ackermann steering) for realistic vehicle motion
- **Intelligent Driver Model (IDM)** for natural car-following behavior with per-driver parameters
- **Full road network editor** — draw roads, place intersections, set traffic signals, and configure lanes while the simulation runs
- **Traffic signals & stop signs** with phase cycling, queue-based priority, and right-of-way rules
- **Lane change logic** — vehicles merge to the correct lane ahead of turns, factoring in congestion
- **Bezier curve roads** with draggable control points for smooth, realistic road geometry
- **Multi-lane roads** with computed lane offsets and center-line tracking via PID steering
- **A\* pathfinding** on a directed road graph with path caching
- **Spatial indexing** via uniform grids for fast collision queries, nearest-road lookups, and render culling
- **Adjustable time scale** from paused (0x) to 64x speed
- **Day/night cycle** with schedule-driven commuter behavior, warm dawn/dusk tinting, and lit windows, street lights, and signal lenses at night
- **Procedural scenery** — grass terrain, buildings drawn per destination type (homes, offices, shops, schools, parking), roadside trees/bushes, and street lights, all deterministically placed with no road/building overlap
- **Visually distinct road types** — residential, arterial, highway, and dirt differ in surface, shoulders/sidewalks, lane markings, and medians, not just color
- **Realistic traffic furniture** — signal heads with colored lenses, octagonal stop signs, yield triangles, and speed-limit signs posted only where the limit actually changes

---

## Tech Stack

| Component | Technology |
|-----------|------------|
| Language | C# / .NET 8 |
| Rendering | SkiaSharp 3.x (GPU-accelerated 2D) |
| Windowing | WinForms |
| UI | Retained-mode control hierarchy (WinForms-like panels/labels/buttons) rendered via SkiaSharp |
| Architecture | Double-buffered sim/render, SoA data layout, spatial grid indexing |

---

## Architecture Overview

```
┌─────────────────────────────────────────────┐
│              Main Thread                     │
│  ┌─────────┐  ┌──────────┐  ┌────────────┐  │
│  │  Input   │→│  Render  │→│  UI/Editor │  │
│  │ Handling │  │ (SkiaSharp)│ │  Overlay   │  │
│  └─────────┘  └──────────┘  └────────────┘  │
└─────────────────────────────────────────────┘
         ↕ (double-buffered state snapshot)
┌─────────────────────────────────────────────┐
│           Simulation Thread(s)               │
│  ┌──────┐ ┌──────────┐ ┌─────────────────┐  │
│  │ Time │→│ Vehicle  │→│  Traffic Rule   │  │
│  │ Step │ │ Physics  │ │  Enforcement    │  │
│  └──────┘ └──────────┘ └─────────────────┘  │
│  ┌──────────────┐ ┌────────────────────────┐ │
│  │ Pathfinding  │ │  Spawn/Despawn/Park   │ │
│  │ (background) │ │  Manager              │ │
│  └──────────────┘ └────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### Key Design Decisions

- **Struct-of-Arrays (SoA)** layout for vehicle data — hot simulation fields packed together for cache efficiency
- **Uniform spatial grid** (50m cells) for O(1) point queries used in collision broad-phase, editor interactions, and render culling
- **Bicycle model** physics instead of per-wheel simulation — captures realistic steering at minimal cost for 10,000+ vehicles
- **IDM car-following** with per-driver personality parameters for natural traffic flow
- **Directed graph** road network with Bezier curve geometry and precomputed adjacency lists

---

## Project Structure

```
Roads/
├── Roads.slnx                        # Solution file
├── DESIGN.md                         # Full design document
├── Roads.App/                        # Main application
│   ├── Program.cs                    # Entry point
│   ├── MainForm.cs                   # WinForms host window
│   ├── SimulationLoop.cs             # Main loop orchestrator
│   ├── SimConstants.cs               # Global simulation constants
│   ├── GeometryUtil.cs               # Shared geometry helpers
│   ├── GraphChangeHandler.cs         # Thread-safe graph mutation queue
│   │
│   ├── Core/
│   │   └── Camera.cs                 # Pan/zoom/world-to-screen transform
│   │
│   ├── World/
│   │   ├── RoadGraph.cs              # Directed graph structure
│   │   ├── RoadNode.cs               # Intersection data
│   │   ├── RoadEdge.cs               # Road segment (Bezier geometry, lanes, speed limit)
│   │   ├── SpatialGrid.cs            # Uniform grid for vehicles
│   │   ├── EdgeSpatialGrid.cs        # Uniform grid for road edges
│   │   ├── Pathfinder.cs             # A* pathfinding
│   │   ├── IntersectionArc.cs        # Turn arc geometry
│   │   ├── IntersectionArcCache.cs   # Cached intersection arcs
│   │   ├── StopLineCache.cs          # Precomputed stop line positions
│   │   ├── TrafficSignalSystem.cs    # Traffic light phase logic
│   │   ├── StopSignSystem.cs         # Stop sign queue logic
│   │   └── YieldSignSystem.cs        # Yield sign logic
│   │
│   ├── Vehicles/
│   │   ├── VehicleStore.cs           # SoA vehicle data store
│   │   ├── VehiclePhysics.cs         # Bicycle model kinematics
│   │   ├── SteeringController.cs     # PID lane-center tracking
│   │   ├── LaneChangeLogic.cs        # Lane selection and merging
│   │   └── VehicleSpawner.cs         # Spawn/despawn management
│   │
│   ├── Editor/
│   │   ├── EditorState.cs            # Current tool, selection state
│   │   ├── RoadTool.cs               # Road drawing (click to place nodes)
│   │   ├── DeleteTool.cs             # Road/node deletion
│   │   ├── SignalTool.cs             # Traffic signal assignment
│   │   ├── DestinationTool.cs        # Destination point placement
│   │   ├── EdgeSnapTool.cs           # Snap-to-edge helpers
│   │   └── LaneRestrictionTool.cs    # Lane restriction editing
│   │
│   └── Rendering/
│       ├── SceneRenderer.cs          # Main render orchestrator (layer/z-order authority)
│       ├── SkiaCanvas.cs             # SkiaSharp surface management
│       ├── TerrainRenderer.cs        # Procedural grass terrain background
│       ├── RoadRenderer.cs           # Per-type road surfaces, markings, shoulders, crosswalks
│       ├── RoadTypeVisuals.cs        # Per-RoadType style table (colors, widths, shoulders)
│       ├── BuildingLayer.cs          # Deterministic building placement from destination nodes
│       ├── BuildingRenderer.cs       # Procedural building art (roofs, lots, night windows)
│       ├── PropRenderer.cs           # Street lights, trees, bushes (deterministic scatter)
│       ├── SignRenderer.cs           # Signal heads, stop/yield signs, change-only speed signs
│       ├── VehicleRenderer.cs        # Vehicle shape drawing
│       ├── CongestionHeatMap.cs      # Per-edge congestion overlay (H key)
│       ├── RenderDetail.cs           # LOD thresholds and frustum-culling helpers
│       ├── PerfTelemetry.cs          # Frame timing + pathfind stats (feeds HUD & benchmarks)
│       │
│       └── Ui/                       # Retained-mode control hierarchy (WinForms-like)
│           ├── Panel.cs              # Base control: bounds, background/border, mouse events
│           ├── Label.cs              # Panel + font/text (static or live TextSource)
│           ├── Button.cs             # Label + hover/pressed/active/disabled color states
│           ├── UiRoot.cs             # Z-order, layout, hover tracking, mouse capture
│           ├── UiTheme.cs            # Shared fonts, colors, POI palette, scratch paints
│           ├── MenuBar.cs            # File actions (New/Save/Load) + tool buttons (top-left)
│           ├── PoiSubmenu.cs         # POI-type buttons under the Dest Pt tool
│           ├── ClockPanel.cs         # Analog 12h clock (AM/PM), digital time, speed + transport buttons
│           ├── SelectionInfoPanel.cs # Selected node/edge details (always shown; "No selection" idle)
│           ├── LegendPanel.cs        # Keyboard shortcut legend
│           ├── SliderPanel.cs        # Runtime-tunable parameter sliders (container)
│           ├── Slider.cs             # One labeled slider row (drag with capture)
│           ├── MinimapPanel.cs       # Corner minimap (cached SKPicture, click/scrub)
│           ├── StatisticsPanel.cs    # Population/traffic statistics (on by default; N toggles)
│           ├── VehicleInfoPanel.cs   # Selected vehicle info (auto height)
│           ├── PerformanceHudPanel.cs# FPS / sim / draw / GC readout (on by default; P toggles)
│           ├── PerformanceBar.cs     # Stacked sim/draw/idle frame-time bar
│           └── BottomLeftStack.cs    # Overlap-free stacking of HUD/stats/vehicle/selection info
```

---

## Vehicle Physics

Each vehicle uses a **bicycle model** for steering kinematics:

```
frontWheel = position + (wheelbase/2) * direction(heading)
rearWheel  = position - (wheelbase/2) * direction(heading)

frontWheel += speed * dt * direction(heading + steeringAngle)
rearWheel  += speed * dt * direction(heading)

heading  = atan2(frontWheel.y - rearWheel.y, frontWheel.x - rearWheel.x)
position = (frontWheel + rearWheel) / 2
```

A **PID steering controller** tracks the lane center Bezier curve with a speed-scaled lookahead distance. The **Intelligent Driver Model (IDM)** governs acceleration and braking based on the gap to the vehicle ahead.

### Vehicle Types (Planned)

| Type | Length | Max Speed | Acceleration |
|------|--------|-----------|-------------|
| Sedan | 4.5m | 50 m/s | 3.5 m/s² |
| SUV | 5.2m | 45 m/s | 3.0 m/s² |
| Truck | 8.0m | 35 m/s | 2.0 m/s² |
| Bus | 12.0m | 30 m/s | 1.5 m/s² |
| Motorcycle | 2.2m | 55 m/s | 5.0 m/s² |

---

## Traffic Rules

- **Traffic signals** cycle through green/yellow/red phases per approach direction
- **Stop signs** use first-come-first-served queue priority
- **Speed limits** per road edge, respected by vehicles with personality-based bias
- **Right-of-way** at unsignalized intersections
- **Yellow light dilemma** — vehicles decide to stop or proceed based on distance and speed
- **Lane changes** are smooth lateral transitions triggered by upcoming turn requirements and congestion

---

## Editor Tools

| Tool | Action |
|------|--------|
| **Road** | Click to place nodes; creates connected road edges with Bezier curves |
| **Node** | Click to add a node — splits a nearby road at the ghost-preview position, or places a free node in empty space |
| **Delete** | Click to remove road segments or nodes |
| **Signal** | Click intersection to cycle: none → stop sign → yield → traffic light; Shift+click tunes (per-edge stop/yield exemption, light phase rotation) |
| **Destination** | Click to place vehicle destination points (incl. Entry/Exit nodes, where traffic enters and leaves the map) |
| **Lane Restriction** | Configure lane-specific rules |
| **Edge Snap** | Snap new connections to existing road edges |

Right-click (or ESC) is the universal cancel: it aborts the in-progress operation (road chain, lane-restrict mode, selection) one step per press, and with nothing left to cancel switches back to the Select tool.

Roads can be added and removed **while the simulation is running** — vehicles on affected routes automatically repath or despawn gracefully.

---

## Memory Budget

| Entity | Target | Count |
|--------|--------|-------|
| Road segment | ~64 bytes | 10,000 |
| Intersection | ~48 bytes | 5,000 |
| Vehicle | ~128 bytes | 10,000 |
| Driver | ~32 bytes | 10,000 |
| Building/POI | ~24 bytes | 5,000 |
| **Total** | **~3-4 MB** | — |

Achieved via struct-of-arrays layout and value types.

---

## Development Progress

### Phase 1: Foundation — *Complete*
> Window with pannable/zoomable canvas, basic road drawing, and a single vehicle driving along a road.

| Status | Task |
|--------|------|
| :white_check_mark: | Project setup (.NET 8, SkiaSharp, WinForms host window) |
| :white_check_mark: | Camera system (pan, zoom, world-to-screen transform) |
| :white_check_mark: | Road data structures (RoadNode, RoadEdge, RoadGraph) |
| :white_check_mark: | Road rendering (straight lines, basic lane markings) |
| :white_check_mark: | Editor: Road placement tool (click to place nodes, creates edges) |
| :white_check_mark: | Single vehicle struct, bicycle model physics |
| :white_check_mark: | Vehicle follows a single road edge (parametric Bezier tracking) |
| :white_check_mark: | Basic render loop (road + one vehicle) |
| :white_check_mark: | Toolbar skeleton (Select, Road, Delete buttons) |

### Phase 2: Road Network & Pathfinding — *Complete*
> Connected road graph with intersections, A* pathfinding, and multiple vehicles navigating the network.

| Status | Task |
|--------|------|
| :white_check_mark: | Intersection nodes (auto-created when roads cross or connect) |
| :white_check_mark: | Turn matrix per intersection (which edges connect) |
| :white_check_mark: | Bezier curve roads (control point dragging in editor) |
| :white_check_mark: | Multi-lane roads (lane offset computation, lane rendering) |
| :white_check_mark: | A* pathfinding on the road graph |
| :white_check_mark: | Path representation (edge sequence) and vehicle path-following |
| :white_check_mark: | Spawn points: place in editor, vehicles spawn and pick random destination *(since removed — traffic enters/leaves via Entry/Exit nodes)* |
| :white_check_mark: | Multiple vehicles (VehicleStore SoA, batch update loop) |
| :white_check_mark: | Spatial grid for vehicles |
| :white_check_mark: | Basic collision avoidance (brake if vehicle ahead is too close) |
| :white_check_mark: | Delete tool: spatial grid edge lookup |
| :white_check_mark: | Compact adjacency optimization |

### Phase 3: Traffic Rules & Signals — *Complete*
> Vehicles obey traffic signals, stop signs, speed limits, and intersection right-of-way rules.

| Status | Task |
|--------|------|
| :white_check_mark: | Traffic light system (phase cycling, green/yellow/red per approach) |
| :white_check_mark: | Stop sign behavior (full stop, queue-based priority) |
| :white_check_mark: | Speed limit enforcement |
| :white_check_mark: | Intersection signal rendering |
| :white_check_mark: | Editor: Signal tool |
| :white_check_mark: | IDM car-following model |
| :white_check_mark: | Yellow light dilemma handling |
| :white_check_mark: | Right-of-way at unsignalized intersections |
| :white_check_mark: | Lane change logic |

### Phase 4: Driver Personalities & Daily Routines — *In Progress*
> Each driver has unique traits, follows a daily schedule with commutes, errands, and day/night cycle.

| Status | Task |
|--------|------|
| :x: | DriverPersonality struct with trait generation |
| :x: | Named archetypes (Lead Foot, Sunday Driver, etc.) |
| :x: | Traits wired into physics (speed bias, reaction delay, steering gains) |
| :x: | Simulation clock (0-24 hour cycle, displayed on UI) |
| :white_check_mark: | Time scale controls (pause, 1x-64x, keyboard shortcuts) |
| :x: | Points of Interest (POI) data structure and editor placement |
| :x: | Daily schedule system (departure times, destination types) |
| :x: | Schedule-driven spawning (morning/evening rush) |
| :x: | Vehicle parking (arrive at POI → parked state → depart later) |
| :x: | Population manager (target vehicle count, spawn rate control) |
| :x: | Day/night visual changes (background color, headlights) |

### Phase 5: Performance & Scale — *Not Started*
> Optimize to handle 10,000+ vehicles at interactive frame rates.

| Status | Task |
|--------|------|
| :x: | Contraction Hierarchies |
| :x: | Path caching and batch pathfinding |
| :x: | Double-buffered simulation state |
| :x: | LOD rendering |
| :x: | Frustum culling via spatial grid |
| :x: | Parked vehicle optimization |
| :x: | SIMD hot loop optimization |
| :x: | Memory pooling |
| :x: | Stress testing: 10K vehicles |
| :x: | Performance HUD |

### Phase 6: Polish & Features — *Not Started*
> Complete editor, visual polish, save/load, quality-of-life features.

| Status | Task |
|--------|------|
| :x: | Save/Load (binary format) |
| :x: | JSON export |
| :x: | Auto-save |
| :x: | Undo/Redo system |
| :x: | Minimap |
| :x: | Statistics panel |
| :x: | Congestion heat-map overlay |
| :x: | Zone painting tool |
| :x: | Map templates |
| :x: | Road type visuals |
| :x: | Vehicle type variety |
| :x: | Sound effects |
| :x: | Tooltip / hover info |
| :x: | Right-click context menus |
| :x: | Settings dialog |

---

## Building & Running

### Prerequisites

- [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0) or later
- Windows (WinForms host)

### Build & Run

```bash
cd Roads
dotnet build Roads.slnx
dotnet run --project Roads.App
```

---

## Controls

| Input | Action |
|-------|--------|
| **Left Click** | Use current editor tool |
| **Right Click** | Finish road chain / cancel |
| **Middle Mouse Drag** | Pan camera |
| **Scroll Wheel** | Zoom in/out |
| **WASD** | Pan camera |
| **Space** | Pause / Resume simulation |
| **+ / -** | Increase / Decrease time scale |

---

## License

This project is not currently published under an open-source license.
