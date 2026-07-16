# Roads: City-Scale Vehicle Traffic Simulation

A real-time, city-scale traffic simulation built in C# with a graphical editor. Thousands of individually-simulated vehicles with physics-based steering navigate a user-editable road network, obeying traffic rules and following daily routines.

![Platform: Windows](https://img.shields.io/badge/platform-Windows-blue)
![.NET 8](https://img.shields.io/badge/.NET-8.0-purple)
![Renderer: SkiaSharp](https://img.shields.io/badge/renderer-SkiaSharp-green)
![License: GPL v3](https://img.shields.io/badge/license-GPLv3-blue)

---

## Features

- **Real-time simulation** at a fixed 30 Hz timestep with time scaling from pause to 64x — stress-tested at **10,000 vehicles at ~30 FPS**
- **Physics-based driving** using the bicycle model (simplified Ackermann steering) for realistic vehicle motion
- **Intelligent Driver Model (IDM)** for natural car-following behavior
- **Driver personalities** — five weighted archetypes (Commuter, Lead Foot, Sunday Driver, Nervous Nellie, Trucker) whose traits (speed bias, reaction time, following distance, steering gains) feed directly into the physics
- **Town life** — persistent residents with homes and archetype-driven daily schedules; morning and evening rush hours emerge naturally; vehicles park at their destination and depart later
- **Five vehicle types** (sedan, SUV, truck, bus, motorcycle) with per-type dimensions, wheelbase, and acceleration
- **Full road network editor** — draw straight or curved roads, place intersections, configure signals and lanes **while the simulation runs**
- **Traffic control** — traffic lights (fixed-time or vehicle-actuated), stop signs, and yield signs, with per-approach exemptions and editable phase groupings
- **Right-of-way rules** — queue priority at stop signs, road-class priority (arterial over residential) at unsignalized intersections
- **Deadlock detection & recovery** — layered mechanisms detect and dissolve gridlock; stuck-vehicle diagnostics feed a reproducible headless test harness
- **Lane change logic** — vehicles merge to the correct lane ahead of turns, factoring in congestion
- **Bezier curve roads** with draggable control points; multi-lane, one-way, and single-lane two-way (shared) roads
- **A\* pathfinding** on a directed road graph, with graceful rerouting when the network changes under moving traffic
- **Deterministic simulation RNG** — headless runs and jam replays are exactly reproducible from a seed
- **Spatial indexing** via uniform grids for fast collision queries, nearest-road lookups, and visible-only rendering
- **Day/night cycle** with schedule-driven commuter behavior, warm dawn/dusk tinting, and lit windows, street lights, and signal lenses at night
- **Procedural scenery** — grass terrain, buildings drawn per destination type (homes, offices, shops, schools, parking), roadside trees/bushes, and street lights, all deterministically placed with no road/building overlap
- **Visually distinct road types** — residential, arterial, highway, and dirt differ in surface, shoulders/sidewalks, lane markings, and medians, not just color
- **Realistic traffic furniture** — signal heads with colored lenses, octagonal stop signs, yield triangles, and speed-limit signs posted only where the limit actually changes
- **Procedural sound** — ambient traffic hum, pooled per-vehicle engine voices, and event one-shots, all synthesized in real time with NAudio (no samples)
- **Save/load** — binary map format plus human-readable JSON export, with rotating timestamped autosaves
- **In-app settings dialog** — graphics, simulation, audio, and autosave options persisted to `settings.json`
- **Overlays & tooling** — minimap, statistics panel, congestion heat-map, performance HUD, and a benchmark/stress-test harness (procedural grid city + 10K bulk spawn)

---

## Tech Stack

| Component | Technology |
|-----------|------------|
| Language | C# / .NET 8 |
| Rendering | SkiaSharp 3.x (GPU-accelerated 2D) |
| Windowing | WinForms |
| UI | Retained-mode control hierarchy (panels/labels/buttons) rendered via SkiaSharp |
| Audio | NAudio 2.x (real-time procedural synthesis) |
| Architecture | Single-threaded fixed-timestep sim (30 Hz), SoA data layout, spatial grid indexing |

---

## Architecture Overview

Everything runs on the main thread, once per rendered frame — profiling showed the tick fits comfortably in a frame even at 10K vehicles, so no sim thread or double-buffering is needed. Audio renders on NAudio's playback thread, parameter-driven once per frame.

```
┌────────────────── Main thread, per frame ───────────────────────┐
│                                                                  │
│  Input           Simulation tick                Render           │
│  ┌──────────┐    ┌──────────────────────────┐   ┌─────────────┐  │
│  │ WinForms │ →  │ fixed 30 Hz accumulator; │ → │ SkiaSharp:  │  │
│  │ mouse/   │    │ substeps scale with time │   │ terrain,    │  │
│  │ keyboard │    │ warp (1x–64x, clamped)   │   │ roads,      │  │
│  └──────────┘    │                          │   │ buildings,  │  │
│                  │ caches/signals → lane    │   │ vehicles,   │  │
│                  │ change → steering + IDM  │   │ retained UI │  │
│                  │ → physics → reroute →    │   └─────────────┘  │
│                  │ population/schedules     │                    │
│                  └──────────────────────────┘                    │
└──────────────────────────────────────────────────────────────────┘
```

The same tick logic is reused headless by two harnesses: `--autobench` (10K-vehicle performance benchmark) and `--simtest` (reproducible jam detection on a saved map).

### Key Design Decisions

- **Struct-of-Arrays (SoA)** layout for vehicle data — hot simulation fields packed together for cache efficiency
- **Uniform spatial grids** for vehicles and road edges — O(1) point queries for collision broad-phase, editor picking, and visible-only render passes
- **Bicycle model** physics instead of per-wheel simulation — captures realistic steering at minimal cost for 10,000+ vehicles
- **IDM car-following** with per-driver personality parameters for natural traffic flow
- **Directed graph** road network with Bezier curve geometry, precomputed adjacency, and cached intersection turn arcs
- **Deterministic RNG** (`SimRandom`) for every simulation-affecting draw, so headless replays reproduce jams exactly
- **Measure before optimizing** — the heavyweight planned optimizations (contraction hierarchies, double-buffered sim thread, SIMD) were all *measured out*; the real 300x win came from clamping the fixed-timestep substep spiral, replacing O(n²) arc-conflict scans with an arc-occupancy index, caching Bézier projections, and culling render passes to the visible viewport

---

## Project Structure

```
ROADS/
├── Roads.slnx                        # Solution file
├── DESIGN.md                         # Full design document + phase checklists
├── HIDDEN_DEPENDENCIES.md            # Catalogue of implicit ordering contracts
├── LICENSE                           # GPL v3
├── settings.json                     # Persisted app settings (written on change)
├── *.roads                           # Sample / test maps (binary format)
├── backups/                          # Rotating timestamped autosaves
├── scripts/                          # parse_benchmark.py, unpack_roads.py,
│                                     #   osm_to_roads.py (build a map from OpenStreetMap data)
│
└── Roads.App/                        # Main application
    ├── Program.cs                    # Entry point; --autobench / --simtest CLI modes
    ├── MainForm.cs                   # WinForms host window, input routing
    ├── SimulationLoop.cs             # Fixed 30 Hz tick orchestrator
    ├── SimConstants.cs               # Global simulation constants
    ├── GeometryUtil.cs               # Shared geometry helpers
    ├── GraphChangeHandler.cs         # Per-frame reaction to graph edits (cache rebuilds, vehicle fix-ups)
    │
    ├── Audio/
    │   ├── AudioEngine.cs            # Always-running NAudio graph, parameters driven per frame
    │   └── Synth/                    # Allocation-free DSP: engine voices, ambient hum bed,
    │                                 #   event one-shots, master gain/duck stage
    ├── Core/
    │   ├── AppSettings.cs            # All user-adjustable settings (record, dialog-paged)
    │   ├── Camera.cs                 # Pan/zoom/world-to-screen transform
    │   ├── SimRandom.cs              # Deterministic process-wide simulation RNG
    │   └── SimulationClock.cs        # In-game time of day (0–24 h)
    │
    ├── Diagnostics/
    │   ├── DeadlockReport.cs         # Stuck-vehicle diagnostics (in-app D key + headless)
    │   └── SimTestHarness.cs         # Headless reproducible sim runs (--simtest)
    │
    ├── Persistence/
    │   ├── MapSerializer.cs          # Binary .roads save/load (graph, signals, vehicles)
    │   ├── MapJsonSerializer.cs      # Human-readable JSON export (parallel walk order)
    │   ├── AutoSaveManager.cs        # Timestamped rotating backups
    │   └── SettingsStore.cs          # settings.json load/save
    │
    ├── World/
    │   ├── RoadGraph.cs              # Directed graph structure
    │   ├── RoadNode.cs               # Intersection data
    │   ├── RoadEdge.cs               # Road segment (Bezier geometry, lanes, speed limit)
    │   ├── SpatialGrid.cs            # Uniform grid for vehicles
    │   ├── EdgeSpatialGrid.cs        # Uniform grid for road edges (also drives render culling)
    │   ├── Pathfinder.cs             # A* pathfinding
    │   ├── GridNetworkGenerator.cs   # Procedural grid city for stress testing
    │   ├── IntersectionArc.cs        # Turn arc geometry
    │   ├── IntersectionArcCache.cs   # Cached intersection arcs
    │   ├── StopLineCache.cs          # Precomputed stop line positions
    │   ├── TrafficSignalSystem.cs    # Traffic light phases (fixed-time + actuated)
    │   ├── StopSignSystem.cs         # Stop sign queue logic
    │   └── YieldSignSystem.cs        # Yield sign logic
    │
    ├── Vehicles/
    │   ├── VehicleStore.cs           # SoA vehicle data store
    │   ├── VehicleType.cs            # Per-type dimensions, wheelbase, acceleration
    │   ├── VehiclePhysics.cs         # Bicycle model kinematics
    │   ├── SteeringController.cs     # PID lane-center tracking
    │   ├── LaneChangeLogic.cs        # Lane selection and merging
    │   ├── ArcOccupancyIndex.cs      # Arc → occupying-vehicles index (kills O(n²) scans)
    │   ├── VehicleSpawner.cs         # Spawn/despawn management
    │   ├── DriverPersonality.cs      # Archetypes + trait generation
    │   ├── Resident.cs               # A townsperson: home, activity state
    │   ├── ScheduleGenerator.cs      # Archetype-driven daily schedules
    │   ├── POIRegistry.cs            # POI lookup by type + occupancy tracking
    │   └── PopulationManager.cs      # Town-life orchestrator (departures, arrivals, lifecycle)
    │
    ├── Editor/
    │   ├── EditorState.cs            # Current tool, selection, sticky road options
    │   ├── RoadTool.cs               # Road drawing (straight/curved chains, ghost previews)
    │   ├── NodeTool.cs               # Add node: split a road or place a free node
    │   ├── UpdateSegmentTool.cs      # Retype an existing segment to the sticky options
    │   ├── DeleteTool.cs             # Road/node deletion
    │   ├── SignalTool.cs             # Signal type/control/rotation/exemption editing
    │   ├── DestinationTool.cs        # POI placement (incl. Entry/Exit nodes)
    │   └── LaneRestrictionTool.cs    # Per-lane turn restriction editing
    │
    └── Rendering/
        ├── SceneRenderer.cs          # Main render orchestrator (layer/z-order authority)
        ├── SkiaCanvas.cs             # SkiaSharp surface management
        ├── TerrainRenderer.cs        # Procedural grass terrain background
        ├── RoadRenderer.cs           # Per-type road surfaces, markings, shoulders, crosswalks
        ├── RoadTypeVisuals.cs        # Per-RoadType style table (colors, widths, shoulders)
        ├── BuildingLayer.cs          # Deterministic building placement from destination nodes
        ├── BuildingRenderer.cs       # Procedural building art (roofs, lots, night windows)
        ├── PropRenderer.cs           # Street lights, trees, bushes (deterministic scatter)
        ├── SignRenderer.cs           # Signal heads, stop/yield signs, change-only speed signs
        ├── VehicleRenderer.cs        # Vehicle shape drawing (per-type bodies, LOD dots)
        ├── CongestionHeatMap.cs      # Per-edge congestion overlay (H key)
        ├── RenderDetail.cs           # LOD thresholds and frustum-culling helpers
        ├── PerfTelemetry.cs          # Frame timing + pathfind stats (feeds HUD & benchmarks)
        ├── BenchmarkCapture.cs       # Appends labeled perf snapshots to benchmark.log
        │
        └── Ui/                       # Retained-mode control hierarchy
            ├── Panel.cs              # Base control: bounds, background/border, mouse events
            ├── Label.cs              # Panel + font/text (static or live TextSource)
            ├── Button.cs             # Label + hover/pressed/active/disabled color states
            ├── Checkbox.cs           # Live-bound checkbox (get/set accessors)
            ├── Slider.cs             # One labeled slider row (drag with capture)
            ├── UiRoot.cs             # Z-order, layout, hover tracking, mouse capture
            ├── UiTheme.cs            # Shared fonts, colors, POI palette, scratch paints
            ├── MenuBar.cs            # File actions (New/Save/Load), tool buttons, Settings
            ├── RoadSubmenu.cs        # Road-family tools + sticky options (type, width,
            │                         #   one-way, shared lane, straight/curved)
            ├── SignalSubmenu.cs      # Change Type / Control Type / Rotate / Exempt
            ├── PoiSubmenu.cs         # POI-type buttons under the Dest Pt tool
            ├── SettingsDialog.cs     # Modal in-canvas settings dialog (paged)
            ├── ClockPanel.cs         # Analog 12h clock, digital time, speed + transport buttons
            ├── SelectionInfoPanel.cs # Selected node/edge details ("No selection" idle)
            ├── VehicleInfoPanel.cs   # Selected vehicle info (auto height)
            ├── LegendPanel.cs        # Keyboard shortcut legend
            ├── MinimapPanel.cs       # Corner minimap (cached SKPicture, click/scrub)
            ├── StatisticsPanel.cs    # Population/traffic statistics (N toggles)
            ├── PerformanceHudPanel.cs# FPS / sim / draw / GC readout (P toggles)
            ├── PerformanceBar.cs     # Stacked sim/draw/idle frame-time bar
            └── BottomLeftStack.cs    # Overlap-free stacking of HUD/stats/vehicle/selection info
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

### Vehicle Types

| Type | Length | Width | Max Acceleration |
|------|--------|-------|------------------|
| Sedan | 4.5 m | 2.0 m | 3.4 m/s² |
| SUV | 4.9 m | 2.15 m | 3.0 m/s² |
| Truck | 8.5 m | 2.45 m | 0.8 m/s² |
| Bus | 12.0 m | 2.55 m | 1.2 m/s² |
| Motorcycle | 2.2 m | 0.85 m | 4.0 m/s² |

Dimensions feed both rendering and simulation — car-following gaps, stop-line offsets, overlap detection, and lane-change fit checks are all bumper-accurate per type, and the kinematic wheelbase scales with body length. Acceleration values are the full-throttle physical ceiling; drivers usually command less, per their personality.

---

## Traffic Rules

- **Traffic signals** cycle green/yellow/red phases per approach, as either **fixed-time or vehicle-actuated** controllers; phase groupings can be rotated per intersection in the editor
- **Stop signs** use first-come-first-served queue priority; **yield signs** give way to conflicting traffic
- **Per-approach exemptions** let individual approaches skip a stop/yield (e.g. the major road of a two-way stop)
- **Road-class right-of-way** at unsignalized intersections — minor-road traffic yields to the higher-class road
- **Speed limits** per road edge, respected by vehicles with personality-based bias
- **Yellow light dilemma** — vehicles decide to stop or proceed based on distance and speed
- **Lane changes** are smooth lateral transitions triggered by upcoming turn requirements and congestion
- **Deadlock recovery** — layered detection dissolves intersection gridlock instead of letting it spread map-wide

---

## Editor Tools

The toolbar shows **Select**, **Road**, **Signal**, and **Dest Pt**; the Road and Signal buttons open submenus with their tool families and options.

| Tool | Action |
|------|--------|
| **Select** | Click a vehicle, node, or edge to inspect it; keyboard shortcuts retype the selected segment (R = road type, +/− = lanes, [ ] = speed limit, O = one-way, J = shared single lane) |
| **Road** | Click to draw road chains — straight or curved mode; an anchor ghost always previews where the click will land (snap to node, split a road, or free) |
| **Node** *(Road submenu)* | Click to add a node — splits a nearby road at the ghost-preview position, or places a free node in empty space |
| **Delete** *(Road submenu)* | Click to remove road segments or nodes |
| **Update Seg** *(Road submenu)* | Click a segment to retype it to the current sticky road options (type, width, one-way, shared lane) |
| **Signal** *(submenu)* | **Change Type** cycles a node: light → stop → yield → none; **Control Type** toggles fixed-time ↔ actuated; **Rotate** shifts the light's phase grouping; **Exempt** toggles whether an approach must stop |
| **Dest Pt** | Click to place vehicle destination points — homes, offices, shops, schools, parking, plus Entry/Exit nodes where traffic enters and leaves the map |
| **Lane Restriction** | Configure per-lane turn restrictions (L to enter, 1–4 select lane, C applies defaults) |

The Road submenu's sticky options (road type, per-direction width, one-way, single-lane two-way, straight/curved) apply to both new roads and Update Seg clicks.

Right-click (or ESC) is the universal cancel: it aborts the in-progress operation (road chain, lane-restrict mode, selection) one step per press, and with nothing left to cancel switches back to the Select tool.

Roads can be added and removed **while the simulation is running** — vehicles on affected routes automatically repath or despawn gracefully.

---

## Memory Budget (design targets)

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

## Development Status

All planned phases are complete except a few Phase 6 stragglers. The full task-level checklists live in [DESIGN.md](DESIGN.md).

| Phase | Scope | Status |
|-------|-------|--------|
| **1 — Foundation** | Window, camera, road drawing, single vehicle on a road | :white_check_mark: Complete |
| **2 — Road Network & Pathfinding** | Intersections, Bezier/multi-lane roads, A*, many vehicles | :white_check_mark: Complete |
| **3 — Traffic Rules & Signals** | Lights, stop signs, IDM, right-of-way, lane changes | :white_check_mark: Complete |
| **4 — Driver Personalities & Daily Routines** | Archetypes, schedules, parking, sim clock, day/night | :white_check_mark: Complete |
| **4.5 — Dependency Hardening** | Implicit ordering contracts made explicit or self-enforcing | :white_check_mark: Complete |
| **5 — Performance & Scale** | 10,000+ vehicles at interactive frame rates | :white_check_mark: Complete — **10K vehicles at ~30 FPS** |
| **6 — Polish & Features** | Editor UX, save/load, scenery overhaul, UI, sound, settings | :large_orange_diamond: Nearly complete |

**Phase 5 postscript:** the heavyweight planned optimizations (contraction hierarchies, path caching, a double-buffered sim thread, SIMD) were all measured out as unnecessary. The actual bottlenecks — taking the stress scene from 0.1 to ~30 FPS — were the fixed-timestep substep spiral, O(n²) arc-conflict scans (replaced by an arc-occupancy index), per-call Bézier projection, and whole-network render passes (now culled to the visible viewport).

**Remaining Phase 6 items:** undo/redo, zone painting tool, hover tooltips, right-click context menus. (Map templates were cut.)

---

## Building & Running

### Prerequisites

- [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0) or later
- Windows (WinForms host)

### Build & Run

```bash
dotnet build Roads.slnx
dotnet run --project Roads.App
```

Sample maps (`*.roads`) live in the repo root — load them with **Ctrl+O**. Settings persist to `settings.json` and rotating autosaves land in `backups/`, both relative to the working directory.

### Headless modes

```bash
ROADS.exe --autobench[=N]       # 10K-vehicle stress benchmark, N frames (default 100);
                                #   appends metrics to benchmark.log (see scripts/parse_benchmark.py)

ROADS.exe --simtest=<map.roads> # reproducible jam detection on a saved map; exit code 0 = no jams
         [--simhours=H] [--simseed=N] [--simvehicles] [--simout=path] [--diagvehicle=N]
```

---

## Controls

| Input | Action |
|-------|--------|
| **Left Click** | Use current editor tool |
| **Right Click / Esc** | Cancel one step; falls back to the Select tool |
| **Middle Mouse Drag** | Pan camera |
| **Scroll Wheel** | Zoom in/out |
| **Space** | Pause / Resume simulation |
| **< / >** | Decrease / Increase sim speed (1x–64x) |
| **V** | Spawn a vehicle |
| **R** | Cycle road type of selected segment |
| **+ / -** | Lane count of selected segment |
| **[ / ]** | Speed limit of selected segment |
| **O** | Cycle one-way direction |
| **J** | Toggle single-lane two-way (shared) |
| **Del** | Delete selected node |
| **L, C, 1–4** | Lane-restrict mode, default restrictions, select lane |
| **Shift+Click** | Tune signal (exemptions) |
| **Ctrl+S / Ctrl+O** | Save / Load map |
| **P, M, N, H** | Toggle perf HUD, minimap, statistics, heat-map |
| **G, D, F** | Arc-conflict debug, vehicle diagnostics dump, frame diag log |
| **K, B** | Stress test (grid city + 10K vehicles), capture benchmark baseline |

---

## License

This project is licensed under the **GNU General Public License v3.0** — see [LICENSE](LICENSE).
