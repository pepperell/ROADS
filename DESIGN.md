# Roads: City-Scale Vehicle Traffic Simulation

## Overview

A real-time, city-scale traffic simulation built in C# with a graphical editor. Thousands of individually-simulated vehicles with physics-based steering navigate a user-editable road network, obeying traffic rules and following daily routines. The simulation runs a full 24-hour day/night cycle where drivers commute to work, visit shops, and return home based on individual schedules and personality traits.

---

## 1. Core Architecture

### 1.1 Application Framework

- **Platform:** Windows desktop, C# / .NET 8+
- **Rendering:** Custom 2D renderer using **SkiaSharp** (GPU-accelerated via OpenGL/Vulkan backend)
  - Why SkiaSharp: lightweight, no game-engine overhead, excellent 2D performance, cross-platform capable
  - Renders thousands of simple vehicle shapes at 60 FPS with batched draw calls
- **Windowing:** WinForms or SDL2-CS as the host window; SkiaSharp renders into a surface
- **UI Toolbar:** Lightweight immediate-mode UI panel rendered via SkiaSharp (buttons, sliders, labels) overlaid on the simulation canvas

### 1.2 Simulation Loop

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

- **Fixed timestep simulation** at 30 Hz (decoupled from render FPS)
- **Time scale:** user-adjustable from 0x (paused) to 64x speed
- At high time scales, simulation sub-steps per frame increase; rendering can skip interpolation frames
- **Double-buffered world state:** simulation writes to back buffer, renderer reads from front buffer — no locks on the render path

### 1.3 Memory Budget Targets

| Entity | Target Memory | Count |
|--------|--------------|-------|
| Road segment | ~64 bytes | 10,000 |
| Intersection node | ~48 bytes | 5,000 |
| Vehicle | ~128 bytes | 10,000 |
| Driver | ~32 bytes | 10,000 |
| Building/POI | ~24 bytes | 5,000 |
| **Total estimate** | **~3-4 MB** | — |

Achieved via struct-of-arrays (SoA) layout and value types where possible.

---

## 2. Road Network

### 2.1 Data Model — Directed Graph

The road network is a **directed graph** stored as a compact adjacency structure:

```
RoadGraph:
  nodes: Node[]          // intersections, endpoints, on/off ramps
  edges: Edge[]          // road segments between nodes
  adjacency: int[][]     // outgoing edge indices per node
```

**Node (intersection):**

```csharp
struct RoadNode          // 48 bytes
{
    Vector2 Position;    // world-space X,Y (8 bytes)
    ushort EdgeStartIdx; // index into adjacency list (2 bytes)
    byte EdgeCount;      // number of outgoing edges (1 byte)
    byte Flags;          // traffic light, stop sign, yield, roundabout (1 byte)
    float TrafficPhase;  // current signal phase timer (4 bytes)
    // padding to alignment
}
```

**Edge (road segment):**

```csharp
struct RoadEdge          // 64 bytes
{
    int FromNode;
    int ToNode;
    float Length;            // precomputed for pathfinding
    float SpeedLimit;        // m/s
    byte LaneCount;          // 1-4 lanes per direction
    byte RoadType;           // highway, arterial, residential, dirt
    ushort Flags;            // one-way, no-parking, bus-lane, etc.
    // Bezier control points for curved roads:
    Vector2 ControlPoint1;
    Vector2 ControlPoint2;
}
```

### 2.2 Road Geometry

- Each edge is a **cubic Bezier curve** (4 control points: start node, CP1, CP2, end node)
- Lanes are offset curves computed from the center Bezier at fixed lateral offsets
- Lane width: configurable per road type (default 3.5m)
- Road surface is rendered as a thick polyline with lane markings overlaid

### 2.3 Intersections

- Each intersection node has a **turn matrix**: which incoming edge can connect to which outgoing edge
- Traffic signals cycle through phases (green/yellow/red) per approach direction
- Stop signs use a first-come-first-served queue
- Right-on-red, protected lefts, etc. encoded in intersection flags
- Roundabouts: modeled as a ring of short one-way edges around a center node

### 2.4 Spatial Index

- Road edges and vehicles indexed in a **uniform grid** (cell size ~50m)
- Grid lookup is O(1) for point queries, O(neighbors) for range queries
- Used for: nearest-road queries (placing vehicles), collision broad-phase, render culling
- Grid is flat array: `int[] gridCells` + `int[] cellEntityLists` (linked-list per cell via index chaining)
- Rebuilt incrementally when roads are added/removed by the editor

---

## 3. Pathfinding

### 3.1 Hierarchical A* with Contraction Hierarchies

Full A* on every vehicle every time is too expensive at city scale. Solution:

1. **Contraction Hierarchies (CH):** precomputed shortcut edges that allow skipping intermediate nodes
   - Precomputation runs once when the road network changes (background thread)
   - Reduces average path query from O(n log n) to O(k log k) where k << n
   - Shortcut data stored as additional virtual edges in a separate overlay graph

2. **Path caching:** completed paths are stored as a compact list of edge indices
   - Vehicles follow their cached path until they reach the destination or the path is invalidated
   - Path invalidation occurs when: road is removed/added, destination changes, lane blocked by accident

3. **Batch pathfinding:** path requests are queued and processed N-per-frame on a background thread
   - Priority queue: vehicles closer to needing a new path get computed first
   - Budget: ~200 path queries per simulation tick at 30 Hz

### 3.2 Path Representation

```csharp
struct VehiclePath
{
    int[] EdgeSequence;      // compact list of edge indices to follow
    int CurrentEdgeIndex;    // progress along path
    float ProgressOnEdge;    // 0.0 to 1.0, parametric position on current Bezier edge
}
```

### 3.3 Lane Selection

- Within a multi-lane road, vehicles pick lanes based on:
  - Upcoming turn direction (move to correct lane early)
  - Current lane congestion (prefer emptier lanes)
  - Driver personality (aggressive drivers change lanes more)
- Lane changes are smooth lateral transitions over ~2 seconds

---

## 4. Vehicle Physics

### 4.1 Bicycle Model (Simplified Ackermann)

Full per-wheel physics is too expensive for 10,000 vehicles. The **bicycle model** captures realistic steering behavior at minimal cost:

```
State per vehicle:
  position: Vector2       // world X, Y
  heading: float          // radians
  speed: float            // m/s
  steeringAngle: float    // front wheel angle, radians

Update (dt):
  // Bicycle model kinematics
  frontWheel = position + (wheelbase/2) * direction(heading)
  rearWheel  = position - (wheelbase/2) * direction(heading)

  frontWheel += speed * dt * direction(heading + steeringAngle)
  rearWheel  += speed * dt * direction(heading)

  heading = atan2(frontWheel.y - rearWheel.y, frontWheel.x - rearWheel.x)
  position = (frontWheel + rearWheel) / 2
```

### 4.2 Vehicle Struct (SoA Layout)

For cache efficiency, vehicle data is stored in parallel arrays rather than an array of structs:

```csharp
// Struct-of-Arrays for vehicle simulation state
class VehicleStore
{
    // Hot data (touched every tick) — fits in cache lines together
    float[] PosX;           // world X
    float[] PosY;           // world Y
    float[] Heading;        // radians
    float[] Speed;          // m/s
    float[] SteeringAngle;  // radians
    float[] Throttle;       // 0 to 1
    float[] Brake;          // 0 to 1

    // Warm data (touched most ticks)
    int[] CurrentEdge;      // road edge index
    float[] EdgeProgress;   // 0-1 parametric
    byte[] CurrentLane;
    byte[] TargetLane;

    // Cold data (touched occasionally)
    int[] PathId;           // index into path cache
    int[] DriverId;         // index into driver personality table
    int[] DestinationNode;
    VehicleState[] State;   // enum: driving, parked, spawning, despawning

    int Count;              // active vehicle count
    int Capacity;           // allocated capacity
}
```

### 4.3 Vehicle Types

| Type | Length | Max Speed | Acceleration | Color |
|------|--------|-----------|-------------|-------|
| Sedan | 4.5m | 50 m/s | 3.5 m/s^2 | varies |
| SUV | 5.2m | 45 m/s | 3.0 m/s^2 | varies |
| Truck | 8.0m | 35 m/s | 2.0 m/s^2 | varies |
| Bus | 12.0m | 30 m/s | 1.5 m/s^2 | yellow/blue |
| Motorcycle | 2.2m | 55 m/s | 5.0 m/s^2 | varies |

### 4.4 Steering Controller

Each vehicle has an **auto-steering PID controller** that tracks the lane center:

```
target = sample lane center Bezier at (edgeProgress + lookahead)
error = signed lateral offset from vehicle to target
steeringAngle = Kp * error + Kd * d(error)/dt
```

- `Kp` and `Kd` are modulated by driver personality (nervous drivers have high Kp, relaxed drivers low Kp)
- Lookahead distance scales with speed (faster = look further ahead)

---

## 5. Traffic Rules & Behavior

### 5.1 Car-Following Model (IDM — Intelligent Driver Model)

Each vehicle adjusts speed based on the vehicle ahead:

```
acceleration = maxAccel * [1 - (v/v0)^4 - (s*(v,Δv)/gap)^2]

where:
  v     = current speed
  v0    = desired speed (speed limit * driver personality factor)
  gap   = distance to vehicle ahead
  s*    = desired gap = s0 + v*T + v*Δv/(2*sqrt(a*b))
  s0    = minimum gap (2m)
  T     = desired time headway (driver personality: 0.8s to 2.0s)
  a     = max acceleration
  b     = comfortable deceleration (driver personality: 1.5 to 4.0 m/s^2)
```

### 5.2 Traffic Signal Compliance

- Vehicles query the intersection ahead when within braking distance
- If signal is red/yellow: decelerate to stop at the stop line
- If signal is green: proceed (check for conflicting traffic at unprotected turns)
- Stop signs: come to full stop, then check for cross-traffic using a queued priority system

### 5.3 Collision Avoidance

**Broad phase:** uniform spatial grid — check only vehicles in same + adjacent cells
**Narrow phase:** simple oriented-bounding-box (OBB) overlap test (vehicle length x width rectangle)

- If potential collision detected ahead: emergency braking
- Lateral collision avoidance: slight steering correction away from encroaching vehicle
- No true "crash" physics — vehicles hard-stop before overlapping, then slowly resume

### 5.4 Right-of-Way Rules

- Right turn yields to through traffic
- Left turn yields to oncoming traffic (unless protected arrow)
- Merge/yield: entering vehicle must find gap in mainline traffic
- Emergency vehicles (future): others pull to the right

---

## 6. Driver Personalities

### 6.1 Personality Traits (per driver)

```csharp
struct DriverPersonality     // 32 bytes
{
    float Aggressiveness;    // 0-1: affects following distance, lane change frequency
    float SpeedBias;         // 0.8-1.3: multiplier on speed limit
    float ReactionTime;      // 0.3-1.2s: delay before responding to events
    float SteeringSharpness; // 0.5-2.0: Kp multiplier for steering PID
    float BrakingComfort;    // 1.5-4.0 m/s^2: comfortable deceleration
    float LaneChangeBias;    // 0-1: eagerness to change lanes
    float PatienceTimer;     // seconds before honking / attempting risky maneuver
    byte PreferredVehicle;   // vehicle type preference
}
```

### 6.2 Personality Generation

- Traits drawn from normal distributions with configurable mean/stddev
- Correlated traits: aggressive drivers tend to also have high speed bias and low patience
- Named archetypes for quick spawning:
  - **Commuter:** average everything, moderate patience
  - **Lead Foot:** high speed bias, high aggressiveness, sharp steering
  - **Sunday Driver:** low speed bias, high reaction time, very patient
  - **Nervous Nellie:** sharp steering, high braking comfort, large following distance
  - **Trucker:** low aggressiveness, long reaction time (vehicle mass), very patient

### 6.3 Behavior Effects

| Trait | Affects |
|-------|---------|
| Aggressiveness | Following distance (IDM T parameter), willingness to cut off other drivers |
| SpeedBias | Target speed = speedLimit * SpeedBias |
| ReactionTime | Delay buffer on all stimulus-response (braking, lane changes) |
| SteeringSharpness | PID Kp gain — high = snappy turns, low = wide lazy arcs |
| BrakingComfort | IDM 'b' parameter — high = gentle stops, low = hard braking |
| LaneChangeBias | Frequency of lane-change evaluation, gap acceptance threshold |

---

## 7. Daily Routine System

### 7.1 Schedule

Each driver has a **daily schedule** — a list of (time, destination_type) entries:

```csharp
struct ScheduleEntry
{
    float DepartureTime;     // hour of day (0-24), e.g. 7.5 = 7:30 AM
    byte DestinationType;    // Home, Work, Shop, Leisure, School
    int DestinationNodeId;   // specific node, or -1 for "find nearest of type"
}
```

- Typical weekday: Home → Work (7-9 AM) → Work → Home (5-7 PM) with some variation
- Some drivers add midday errands (lunch, shop visit)
- Departure times have random jitter (±30 min) to create realistic rush-hour spread
- Weekend schedules differ: more leisure trips, later wake times

### 7.2 Points of Interest (POI)

```csharp
struct PointOfInterest      // 24 bytes
{
    Vector2 Position;
    int NearestNode;         // snapped to road graph
    byte Type;               // Home, Work, Shop, Leisure, School, Parking
    byte Capacity;           // max simultaneous visitors / parked cars
    ushort CurrentOccupancy;
}
```

- POIs are placed by the user or auto-generated along road edges
- Parking lots are POIs with type=Parking; vehicles despawn visually when "parked" (stored as state only)
- Capacity tracking prevents 1000 cars all going to the same shop

### 7.3 Spawn / Despawn / Parking

- **Spawn:** vehicle appears at an Entry/Exit node (city edge entry point) — there are no dedicated spawn-point markers; immediately begins pathfinding to first destination
- **Despawn:** vehicle reaches city edge exit point and fades out; removed from simulation
- **Park:** vehicle arrives at destination POI, decelerates to stop in nearest parking area, state changes to Parked (removed from physics sim but retained in schedule system); when departure time arrives, vehicle re-enters road
- **Population control:** a maximum vehicle count can be set; spawners adjust spawn rates to never exceed the maximum

---

## 8. Rendering

### 8.1 Camera

- 2D top-down orthographic view
- **Zoom range:** from full-city overview (1 pixel = 10m) to street-level (1 pixel = 0.1m)
- Smooth zoom with scroll wheel (exponential interpolation)
- Pan with middle-mouse drag or WASD keys
- Camera state: `{ centerX, centerY, zoom }`

### 8.2 Level-of-Detail (LOD)

| Zoom Level | Roads | Vehicles | Details |
|-----------|-------|----------|---------|
| Far (city) | Colored lines by type | Dots (1-2 px) or hidden | Major labels only |
| Medium | Lane markings visible | Colored rectangles | Intersections, signals |
| Close (street) | Full detail, markings, curbs | Oriented rectangles with windshield/tail lights | Stop lines, signs, POI icons |

### 8.3 Render Pipeline (SkiaSharp)

```
Per frame:
  1. Compute visible world rect from camera
  2. Query spatial grid for visible road edges → draw road polylines (batched by type)
  3. Query spatial grid for visible vehicles → draw oriented rectangles (batched)
  4. Draw intersection decorations (signals, stop signs) for visible intersections
  5. Draw POI icons for visible POIs
  6. Draw UI overlay (toolbar, info panels, minimap)
```

- All world-space geometry transformed to screen-space via a single 3x2 affine matrix
- Road drawing uses SkiaSharp `SKPath` with thick stroke + lane-marking thin strokes
- Vehicles drawn as filled rounded-rectangles with rotation — single `drawRect` with transform per vehicle
- At far zoom, vehicles rendered as 2px colored dots (no rotation math)

### 8.4 Minimap

- Small fixed-size overview in corner showing full road network
- Current viewport shown as a rectangle
- Click minimap to jump camera

---

## 9. Editor System

### 9.1 Tools (Toolbar Buttons)

| Tool | Action |
|------|--------|
| **Select** | Click to select road/vehicle/POI, show properties panel |
| **Road** | Click to place nodes, drag to set curve control points, creates road edges |
| **Node** | Click to add a node: splits a nearby road (ghost preview shows the spot) or places a free node in empty space |
| **Delete** | Click to remove road segment, node, or POI |
| **POI Place** | Click to place Point of Interest (type selector submenu; the Entry/Exit type is where vehicles spawn/despawn) |
| **Signal** | Click intersection to cycle: none → stop sign → traffic light |
| **Zone Paint** | Paint residential/commercial/industrial zones (auto-generates appropriate POIs) |

### 9.2 Road Drawing UX

1. Click to place first node
2. Click to place second node — a straight road segment appears
3. Drag either endpoint to curve the road (adjusts Bezier control points)
4. Continue clicking to chain segments (auto-connects at intersections)
5. Right-click (or ESC) to finish the road chain — the universal cancel
6. Properties panel: set lane count, speed limit, road type, one-way

### 9.3 Live Editing

- Roads can be added/removed while simulation runs
- Adding a road: inserts into graph, triggers CH rebuild (background), vehicles on affected routes re-path
- Removing a road: vehicles currently on it are re-routed; if no path exists, vehicle despawns gracefully
- POI changes: drivers may update schedules to use new POIs

### 9.4 Undo/Redo

- Command pattern: every editor action creates an undoable command object
- Stack of commands with Ctrl+Z / Ctrl+Y

---

## 10. Simulation Time

### 10.1 Day/Night Cycle

- Simulation clock runs from 0:00 to 24:00, then wraps
- **Time display** on UI shows current time as HH:MM
- Visual changes with time of day:
  - Dawn (5-7): warm colors, increasing traffic
  - Day (7-18): full brightness, peak traffic during rush hours
  - Dusk (18-20): warm colors, evening rush
  - Night (20-5): darker background, vehicle headlights visible, minimal traffic

### 10.2 Time Scale Controls

- Pause (0x), 1x, 2x, 4x, 8x, 16x, 32x, 64x
- At high time scales (>8x), rendering may skip frames but simulation still steps correctly
- Keyboard shortcuts: Space = pause/play, +/- = adjust speed

### 10.3 Statistics Tracked Over Time

- Total vehicles active
- Average speed across network
- Congestion per road segment (color-coded overlay)
- Vehicles spawned / despawned / parked per hour
- Displayed as a collapsible stats panel and optional heat-map overlay

---

## 11. Persistence

### 11.1 Save File Format

Binary format for fast load/save:

```
Header: magic number, version, timestamp
Section 1: Road graph (nodes, edges, adjacency)
Section 2: POI list
Section 3: Simulation state (vehicle positions, paths, driver states)
Section 4: Editor state (camera, selected tool)
Section 5: Settings (time scale, maximum population, etc.)
```

- Also support JSON export for interop / debugging
- Auto-save every 5 minutes to a rotating backup

### 11.2 Map Templates

- Ship with a few pre-built maps:
  - **Grid City:** simple Manhattan-style grid (good for testing)
  - **Highway Ring:** ring highway with radial arterials
  - **Empty:** blank canvas for user to build

---

## 12. Performance Strategies Summary

| Concern | Strategy |
|---------|----------|
| Memory layout | Struct-of-Arrays (SoA) for vehicles; value types for nodes/edges |
| Spatial queries | Uniform grid (50m cells), rebuilt incrementally |
| Pathfinding | Contraction Hierarchies + path caching + batched async queries |
| Physics | Bicycle model (not per-wheel); only active (non-parked) vehicles simulated |
| Collision | Broad-phase grid + narrow-phase OBB; no continuous detection |
| Rendering | SkiaSharp batched draw calls; LOD by zoom; frustum culling via grid |
| Threading | Sim thread(s) decoupled from render thread; double-buffered state |
| Scalability | Parked vehicles removed from physics loop; distant vehicles can be simplified |

---

## 13. Development Phases

### Development Workflow

Phases are built by an Opus "director" that hands self-contained milestones to lower-cost worker agents:

- **Independent features** are grouped into conflict-free batches. The director partitions open items so no two workers modify the same existing file (newly-added files are always safe), spawns one worker per milestone on a `ms/<slug>` branch in its own git worktree, reviews each diff, then merges after the user verifies — one branch at a time, since the app is tested interactively. (Workers dispatched via the Agent tool execute *serially*, not concurrently; batching buys clean isolation and conflict-free merges, not wall-clock speedup. Genuine concurrent fan-out needs the Workflow tool.)
- **File-scope is enforced at the merge gate, not by trust.** Before merging, the director runs `git diff --name-only main..<branch>` and reverts anything outside the milestone's declared file set. Worktree isolation guarantees a stray edit can never reach another worker or `main` un-reviewed.
- **Architectural, cross-cutting work** (e.g. contraction hierarchies, double-buffered simulation, undo/redo) stays sequential and director-led — it is not handed to workers.

### Phase 1: Foundation (Core Infrastructure)

**Goal:** Window with a pannable/zoomable canvas, basic road drawing, and a single vehicle driving along a road.

- [X] Project setup (.NET 8, SkiaSharp, WinForms host window)
- [X] Camera system (pan, zoom, world-to-screen transform)
- [X] Road data structures (RoadNode, RoadEdge, RoadGraph)
- [X] Road rendering (straight lines, basic lane markings)
- [X] Editor: Road placement tool (click to place nodes, creates edges)
- [X] Single vehicle struct, bicycle model physics
- [X] Vehicle follows a single road edge (parametric Bezier tracking)
- [X] Basic render loop (road + one vehicle)
- [X] Toolbar skeleton (Select, Road, Delete buttons)

**Deliverable:** User can draw roads and watch a single car drive along them.

---

### Phase 2: Road Network & Pathfinding

**Goal:** Connected road graph with intersections, A* pathfinding, and multiple vehicles navigating the network.

- [X] Intersection nodes (auto-created when roads cross or connect)
- [X] Turn matrix per intersection (which edges connect)
- [X] Bezier curve roads (control point dragging in editor)
- [X] Multi-lane roads (lane offset computation, lane rendering)
- [X] A* pathfinding on the road graph
- [X] Path representation (edge sequence) and vehicle path-following
- [X] Spawn points: place in editor, vehicles spawn and pick random destination *(since removed — traffic enters/leaves via Entry/Exit nodes only)*
- [X] Multiple vehicles (VehicleStore SoA, batch update loop)
- [X] Spatial grid for vehicles
- [X] Basic collision avoidance (brake if vehicle ahead is too close)
- [X] Delete tool: use spatial grid for edge lookup instead of brute-force O(n*11) sampling
- [X] Compact adjacency optimization: wire up EdgeStartIdx/EdgeCount on RoadNode (replace List<List<int>>)

**Deliverable:** Multiple vehicles pathfind across a road network, following lanes and avoiding rear-end collisions.

---

### Phase 3: Traffic Rules & Signals

**Goal:** Vehicles obey traffic signals, stop signs, speed limits, and intersection right-of-way rules.

- [X] Traffic light system (phase cycling, green/yellow/red per approach)
- [X] Stop sign behavior (full stop, queue-based priority)
- [X] Speed limit enforcement (vehicles adjust speed per road edge)
- [X] Intersection signal rendering (colored circles/indicators)
- [X] Editor: Signal tool (click intersection to set signal type)
- [X] IDM car-following model (replaces basic collision avoidance)
- [X] Yellow light dilemma handling (stop or go decision)
- [X] Right-of-way at unsignalized intersections
- [X] Lane change logic (signal upcoming turn, merge to correct lane)

**Deliverable:** Vehicles obey all traffic rules; intersections have working signals and stop signs.

---

### Phase 4: Driver Personalities & Daily Routines

**Goal:** Each driver has unique traits affecting driving style, and follows a daily schedule.

- [X] DriverPersonality struct with trait generation (normal distributions)
- [X] Named archetypes (Lead Foot, Sunday Driver, etc.)
- [X] Traits wired into physics: speed bias, reaction delay, steering PID gains, following distance
- [X] Simulation clock (0-24 hour cycle, displayed on UI)
- [X] Time scale controls (pause, 1x-64x, keyboard shortcuts)
- [X] Points of Interest (POI) data structure and editor placement
- [X] Daily schedule system (departure times, destination types)
- [X] Schedule-driven spawning (morning rush, evening rush, etc.)
- [X] Vehicle parking (arrive at POI → state change to parked → depart later)
- [X] Population manager (maximum vehicle count, spawn rate control)
- [X] Day/night visual changes (background color, headlights at night)

**Deliverable:** Full day/night cycle with drivers commuting on schedules, each with unique driving behavior.

---

### Phase 4.5: Dependency Hardening

**Goal:** Make the implicit ordering contracts catalogued in [HIDDEN_DEPENDENCIES.md](HIDDEN_DEPENDENCIES.md) explicit, self-enforcing, or structurally unnecessary, so future changes can't silently violate them.

- [X] Single rebuild pipeline: extract the duplicated cache-rebuild chain (SimulationLoop paused + active branches) into one method, including signal-system rebuilds so paused-mode toggles (exemptions, phase rotations) apply immediately
- [X] Version-bump audit: every RoadGraph mutator bumps Version consistently (AddNode and SetLaneRestriction currently don't); document the invalidation-bus contract on the Version property
- [X] Eliminate mid-rebuild graph mutation: signal auto-assignment (SetNodeFlags) and ApplyDefaultLaneRestrictions bump Version inside the rebuild chain, forcing a second full cascade next tick
- [X] Automatic graph-change handling: invoke GraphChangeHandler.HandleIfNeeded once per frame instead of relying on each editor call site to remember it
- [X] Frame-protocol guards: debug assertions enforcing RebuildIfNeeded → Update → GetSignal order in TrafficSignalSystem, StopSignSystem, and YieldSignSystem
- [X] Robust load sequence: exemption/rotation setters auto-size their arrays so MapSerializer.Load no longer requires forced rebuilds before applying overrides
- [X] Centralized vehicle removal: single removal path that fixes all index holders on swap-and-pop (resident mappings, EditorState.SelectedVehicle identity)
- [X] VehicleStore field-sync guard: document the array/Add/Remove/Grow/serializer sync requirement in VehicleStore and make it hard to miss when adding fields
- [ ] ~~Sweep remaining minor contracts from HIDDEN_DEPENDENCIES.md (SplitEdge internal trio, PopulationManager dual version tracking, ScheduleModeActive handoff)~~ — **deferred**; these three remain documented in HIDDEN_DEPENDENCIES.md §7 as known, low-risk contracts.

**Status:** Complete — items 1–8 landed and verified; item 9 deferred.

**Deliverable:** Ordering mistakes fail fast (debug asserts) or can't happen at all (structure); HIDDEN_DEPENDENCIES.md updated to reflect the hardened state.

---

### Phase 5: Performance & Scale

**Goal:** Optimize to handle 10,000+ vehicles at interactive frame rates.

- [ ] ~~Contraction Hierarchies (replace A* for long-distance paths)~~ — **measured out**: at 10K, pathfinding was a few % of the tick (nested in steering), never the bottleneck.
- [ ] ~~Path caching and batch pathfinding (background thread)~~ — **measured out** (same reason); rerouting is rate-limited and cheap once steering was fixed.
- [ ] ~~Double-buffered simulation state (sim thread decoupled from render)~~ — **not needed**: after the fixes the frame fits ~1 substep; the spiral clamp gave smooth motion without a second thread.
- [X] LOD rendering (dots at far zoom, full detail up close)
- [X] Frustum culling — extended to visible-only iteration of the road + sign passes via the edge spatial grid (the big draw win at zoomed-in views).
- [X] Parked vehicle optimization (remove from physics loop)
- [X] Profile and optimize hot loops — real wins: fixed-timestep **spiral clamp**, replacing the **O(n²) arc-conflict scans** (entry + car-following) with an **arc-occupancy index**, cached-control-point **Bézier projection**, and **visible-only render culling**. (SIMD evaluated and dropped — physics was <1% of the tick.)
- [ ] ~~Memory pooling for paths and temporary allocations~~ — **not pursued**: allocations weren't the bottleneck; the per-call projection closure was removed and GC stayed flat at 10K.
- [X] Stress testing: 10K vehicles on large road network — `GridNetworkGenerator` + bulk-spawn + a headless `--autobench` harness (writes `benchmark.log`; parsed by `scripts/parse_benchmark.py`).
- [X] Performance HUD (FPS, vehicle count, sim step time) — now instantaneous (no post-unpause ramp); `benchmark.log` adds a per-subsystem sim breakdown.

**Status:** Complete — **10,000 vehicles at ~30 FPS at 5× zoom** (0.1 → 30 FPS over this phase). The heavyweight planned items (CH, double-buffering, SIMD) were measured out; the actual bottlenecks were the fixed-timestep substep spiral, O(n²) arc-conflict scans, and whole-network render passes. See project memory (`project_phase5_perf_findings`) for the measured root-cause analysis.

**Deliverable:** Simulation runs smoothly with 10,000+ vehicles at 30+ FPS. ✓ (30 FPS @ 5× zoom with 10K.)

---

### Phase 6: Polish & Features

**Goal:** Complete editor, visual polish, save/load, quality-of-life features.

- [X] Save/Load (binary format)
- [X] JSON export
- [X] Auto-save with rotating backups
- [ ] Undo/Redo system (command pattern)
- [X] Minimap
- [X] Statistics panel (vehicle count, avg speed, congestion)
- [X] Congestion heat-map overlay
- [ ] Zone painting tool (residential/commercial/industrial)
- [ ] ~~Map templates (Grid City, Highway Ring, Empty)~~ — **removed** (feature pulled 2026-06-15; not pursuing)
- [X] Road type visuals (highway thick/gray, residential thin/light)
- [X] Graphical overhaul: terrain background (procedural grass mottling, day/night tinted, replaces flat gray)
- [X] Graphical overhaul: procedural buildings replace destination dots (per-POI-type footprints, deterministic from node index, no overlaps with roads/buildings; dot fallback at far zoom)
- [X] Graphical overhaul: road-type visual identity (sidewalks/shoulders, per-type center-line policy, highway median, dirt tire tracks, crosswalks at lights)
- [X] Graphical overhaul: roadside props (street lights with night glow, trees, bushes; deterministic placement off roads/buildings)
- [X] Graphical overhaul: realistic signals & signs (signal heads with lenses, octagon stop signs; speed-limit signs only where the limit changes)
- [X] Retained-mode UI: Panel/Label/Button hierarchy under UiRoot (hover + mouse capture; panels consume background clicks; bottom-left panel stacking fixed; pan no longer stalls over buttons)
- [X] Status bar dissolved into panels (clock panel with analog dial + AM/PM, selection-info panel; stats/minimap absorb edges/residents/zoom; no free-floating UI text)
- [X] UI reorganization + spawn-point removal: spawn points removed from the simulation (Entry/Exit nodes only; legacy flag masked on load); New/Save/Load far left, transport buttons (<< Pause >>) on the clock panel; menu bar + clock panel raised into the old status-bar row; stats/perf/selection panels shown by default with matching 256-px width, title rows, and UiTheme background/border (selection panel shows "No selection" when idle; bottom-left stack moved beside the legend)
- [X] Editor UX: Bézier handles hit-test topmost (grab radius tracks the drawn handle); new Node tool (ghost preview; splits a nearby road or places a free node); right-click/ESC = universal cancel, falling back to the Select tool; Signal tuning (exemptions/phase) moved to Shift+click; Road tool first click is a deferred ghost anchor (no split/node until the segment commits — cancel leaves the graph untouched); crossing intersections ghost along the preview line
- [X] Vehicle type variety (sedan, SUV, truck, bus, motorcycle)
- [ ] Sound effects (optional: ambient traffic hum scaling with density)
- [ ] Tooltip / info on hover (vehicle speed, driver traits, destination)
- [ ] Right-click context menus
- [ ] Settings dialog (graphics quality, simulation parameters)

**Deliverable:** Polished, feature-complete simulation with save/load and full editor.

---

## 14. Key Technical Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Pathfinding bottleneck at scale | Vehicles stall waiting for paths | Contraction Hierarchies + async batch processing + path caching |
| Intersection deadlocks | Vehicles block each other permanently | Timeout-based deadlock detection → force one vehicle to yield / despawn |
| Rendering bottleneck with 10K vehicles | FPS drops | LOD system, frustum culling, batched draw calls |
| Editor changes invalidate running simulation | Crashes or inconsistent state | All graph mutations go through a thread-safe command queue; vehicles on deleted roads gracefully re-route |
| Contraction Hierarchy rebuild time | Lag spike when roads edited | Background rebuild with old CH still active until new one ready |
| Floating point drift in vehicle positions | Vehicles slowly drift off road | Periodic re-snap to lane center Bezier every N ticks |

---

## 15. File / Namespace Structure

```
Roads/
├── Roads.sln
├── Roads.App/                    # Main application project
│   ├── Program.cs                # Entry point
│   ├── MainForm.cs               # WinForms window host
│   ├── SimulationLoop.cs         # Main loop orchestrator
│   │
│   ├── Core/
│   │   ├── Camera.cs             # Pan/zoom/transform
│   │   ├── SimulationClock.cs    # Time tracking, time scale
│   │   └── Settings.cs           # Global configuration
│   │
│   ├── World/
│   │   ├── RoadGraph.cs          # Graph structure, nodes, edges
│   │   ├── RoadNode.cs           # Intersection data
│   │   ├── RoadEdge.cs           # Road segment data
│   │   ├── SpatialGrid.cs        # Uniform grid spatial index
│   │   ├── PointOfInterest.cs    # POI data
│   │   └── WorldState.cs         # Aggregate world state container
│   │
│   ├── Vehicles/
│   │   ├── VehicleStore.cs       # SoA vehicle data
│   │   ├── VehiclePhysics.cs     # Bicycle model update
│   │   ├── SteeringController.cs # PID lane tracking
│   │   ├── CarFollowing.cs       # IDM model
│   │   ├── CollisionAvoidance.cs # Broad/narrow phase
│   │   └── VehicleRenderer.cs    # Draw vehicles
│   │
│   ├── Drivers/
│   │   ├── DriverPersonality.cs  # Trait struct + generation
│   │   ├── DailySchedule.cs      # Schedule entries
│   │   └── SpawnManager.cs       # Spawn/despawn/park logic
│   │
│   ├── Pathfinding/
│   │   ├── AStar.cs              # Basic A*
│   │   ├── ContractionHierarchy.cs # CH precomputation + query
│   │   ├── PathCache.cs          # Path storage and reuse
│   │   └── PathfindingService.cs # Async batch query manager
│   │
│   ├── Traffic/
│   │   ├── TrafficSignal.cs      # Signal phase logic
│   │   ├── StopSign.cs           # Stop sign queue logic
│   │   ├── IntersectionManager.cs # Right-of-way coordination
│   │   └── LaneChangeLogic.cs    # Lane selection and merging
│   │
│   ├── Editor/
│   │   ├── EditorState.cs        # Current tool, selection
│   │   ├── RoadTool.cs           # Road drawing tool
│   │   ├── SelectTool.cs         # Selection + properties
│   │   ├── DeleteTool.cs         # Deletion tool
│   │   ├── POITool.cs            # POI placement
│   │   ├── SignalTool.cs         # Traffic signal assignment
│   │   ├── UndoRedoStack.cs      # Command pattern undo
│   │   └── EditorCommands.cs     # Concrete command classes
│   │
│   ├── Rendering/
│   │   ├── Renderer.cs           # Main render orchestrator
│   │   ├── RoadRenderer.cs       # Road/lane drawing
│   │   ├── UIRenderer.cs         # Toolbar, panels, overlays
│   │   ├── MinimapRenderer.cs    # Minimap
│   │   └── HeatmapOverlay.cs     # Congestion visualization
│   │
│   └── Persistence/
│       ├── MapSerializer.cs      # Save/Load binary
│       ├── MapJsonSerializer.cs  # JSON export
│       └── AutoSave.cs           # Timed auto-save
│
└── Roads.Tests/                  # Unit test project
    ├── PathfindingTests.cs
    ├── VehiclePhysicsTests.cs
    └── TrafficRuleTests.cs
```
