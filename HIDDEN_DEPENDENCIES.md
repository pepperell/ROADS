# Hidden Dependencies: Ordering Contracts Between Variables and Functions

This document catalogues the implicit ordering contracts in the codebase — places where updating a variable requires a follow-up function call, or where functions must run in a specific order, with nothing but doc comments and call-site discipline enforcing it. Phase 4.5 in [DESIGN.md](DESIGN.md) tracks the work to make these contracts explicit, self-enforcing, or structurally unnecessary.

Line references are accurate as of the end of Phase 4 and will drift as the code changes. Update this document as Phase 4.5 items land.

---

## 1. `RoadGraph.Version` is an invalidation bus

Nearly every cache keys off [RoadGraph.Version](Roads.App/World/RoadGraph.cs#L67): StopLineCache, IntersectionArcCache, EdgeSpatialGrid, all three signal systems, RoadRenderer's path cache, VehicleSpawner's destination cache, POIRegistry, PopulationManager, and GraphChangeHandler. The contract: **any mutation of nodes/edges/lane-restrictions must `Version++`, or every downstream system silently serves stale data** (bounds checks make failures silent, not crashes). The full contract lives on the [Version property's XML doc](Roads.App/World/RoadGraph.cs#L50-L67); this section is its prose home.

As of the Phase 4.5 audit, **every public mutator bumps Version**. The three former violators now bump:

- [AddNode](Roads.App/World/RoadGraph.cs#L100) bumps unconditionally (previously a lone node — e.g. the Road tool's first click — was invisible to every version consumer).
- [SetLaneRestriction](Roads.App/World/RoadGraph.cs#L1371) bumps unconditionally — it is now a safe general-purpose mutator instead of one that only worked inside MapSerializer.Load's window (riding on `LoadFromData`'s bump).
- [StripMarkerFlagsFromIntersections](Roads.App/World/RoadGraph.cs#L943) bumps conditionally, only when it actually strips flags, so the converging follow-up pass triggered through GraphChangeHandler strips nothing and settles (same convergence pattern as the signal auto-assignment in §2).

Two **private** surgery helpers intentionally mutate without bumping — `MarkNodeDefunct` and `SplitEdgeSingle` — and their XML docs state that the public caller (`RemoveEdge`/`RemoveNode`, `SplitEdge`) bumps once per operation. Consumers compare versions for equality only, so the multiple bumps inside one operation (e.g. `SplitEdge`'s internal `AddNode` plus its own final bump) coalesce into a single lazy rebuild.

Remaining risk is additive only: a future public mutator that forgets the bump. No runtime check can catch that from the consumer side — version-keyed caches compare for equality and cannot distinguish "unchanged" from "mutated without a bump" — so this contract remains doc-enforced (the Phase 4.5 frame-protocol guards assert staleness relative to the *reported* version, not unreported mutations).

**A second, parallel invalidation stream exists: [WaterLayer.Version](Roads.App/World/WaterLayer.cs).** The painted water layer is not graph state (the sim never reads it), so it carries its own version counter with the same contract: every mutation bumps it. Three render-side caches key on BOTH versions — MinimapPanel's recorded picture, PropRenderer's placement (props must vacate painted water), and SceneRenderer's scenery settle-gate. A new water mutator that forgets its bump leaves stale props/minimap exactly the way a graph mutator would; a new consumer of water data must add the water version to its early-out key, not just `RoadGraph.Version`.

## 2. Graph mutation during cache maintenance — confined to a normalize phase

Historically the rebuild chain mutated the graph mid-rebuild: signal auto-assignment called `graph.SetNodeFlags` inside the systems' `RebuildIfNeeded` (each call bumping Version *during* a rebuild keyed on Version, so **every version-keyed cache rebuilt a second time next tick** — the full double cascade), and `ApplyDefaultLaneRestrictions` bumped between the stop-line and arc rebuilds (re-rebuilding only stop lines next tick). Resolved in Phase 4.5: [SimulationLoop.RebuildWorldCaches](Roads.App/SimulationLoop.cs#L161) now runs two phases:

1. **Normalize** — the ONLY place graph mutation is permitted during cache maintenance. Guarded: runs only when `graph.Version` changed since the last normalize. Order contract: [TrafficSignalSystem.AutoAssign](Roads.App/World/TrafficSignalSystem.cs) → [StopSignSystem.AutoAssign](Roads.App/World/StopSignSystem.cs) (lights first — the stop policy reads the TrafficLight flag) → stop-line rebuild → [ApplyDefaultLaneRestrictions](Roads.App/World/RoadGraph.cs) (reads stop-line tangents). No step reads what a later step writes, so a single pass converges — verified at runtime by a debug-only second pass asserting no further bumps.
2. **Rebuild** — pure projections of the settled graph into caches (StopLineCache, IntersectionArcCache, EdgeSpatialGrid, the three signal systems). A debug assert verifies no step mutates the graph.

After `RebuildWorldCaches` returns, every cache is current with `graph.Version`: a single edit triggers exactly one cascade, settled within the same call. The chain is called from the paused branch, every active fixed step, and MainForm's NewMap/LoadMap; the ordering contracts live in the method's XML doc.

## 3. Per-frame protocols enforced only by doc comments

- All three signal systems have a per-frame contract: `RebuildIfNeeded` → `Update` → `GetSignal` — **now debug-asserted** (Phase 4.5). `Update` asserts version currency in all three systems; stop/yield `GetSignal` assert the full protocol via the public `CanQuery(graph)` predicate (rebuilt at the current version, not dirty, and `Update` has run since the last real rebuild); traffic `GetSignal` asserts only that a rebuild has ever run, because the renderer legitimately reads it every frame including paused. Diagnostic callers that may run while right-of-way data is stale gate on `CanQuery` — MainForm's D-key vehicle dump prints `stopSign=n/a yield=n/a (right-of-way tracking stale)` in that case. When the graph changed, static `AutoAssign` must precede `RebuildIfNeeded` for the traffic-light and stop-sign systems (rebuilds are pure flag projections; the pipeline's normalize phase enforces this — see §2).
- The pathfind-accumulator drain is **structural since the retained-UI migration**: it used to ride on `PerformanceHud.Draw`, which therefore had to be called every frame even while the HUD was hidden (a classic trap). It now lives in [PerfTelemetry.Sample()](Roads.App/Rendering/PerfTelemetry.cs), called unconditionally once per rendered frame from `MainForm.OnPaintSurface`; the HUD panel is a pure view and its visibility is no longer load-bearing. Remaining doc-only contract: exactly one `Sample()` per rendered frame, and PerfTelemetry is the single consumer of `Pathfinder.ReadPathfindStatsAndReset` (BenchmarkCapture and `--autobench` read PerfTelemetry's published properties after Sample).
- User toggles like [SetEdgeExempt](Roads.App/World/StopSignSystem.cs#L73-L78) and [RotatePhase](Roads.App/World/TrafficSignalSystem.cs#L81-L86) only set `_dirty = true` — they take effect at the *next* `RebuildIfNeeded`. The paused branch runs `RebuildWorldCaches()` every 16 ms frame, so toggles made while paused apply (and render) immediately.
- [LaneChangeLogic.ApplyMergeSpeedBias](Roads.App/Vehicles/LaneChangeLogic.cs#L543-L547) "must be called after `UpdateAll`" — it consumes `MergeUrgency`/`TargetLane` computed there.
- [SpatialGrid.Rebuild](Roads.App/World/SpatialGrid.cs#L25) must precede all vehicle queries each tick; this holds only because `VehiclePhysics.UpdateAll` (which moves vehicles) runs last.

## 4. Graph-change fix-ups — automatic, once per tick

[GraphChangeHandler.HandleIfNeeded](Roads.App/GraphChangeHandler.cs#L45) fixes stale selections, marker flags, and vehicles on deleted edges. Since Phase 4.5 it is invoked **automatically once per tick** at the top of [SimulationLoop.Tick](Roads.App/SimulationLoop.cs#L109), in both paused and active modes — the old convention where each editor call site had to remember a follow-up call is gone, and the previously uncovered mutation paths (Delete-key node removal, lane-count/speed keys, the 'C' defaults reset, flag-tool clicks) are covered for free. Before this, a forgotten call wasn't self-healing: vehicles on a Delete-key'd road would sail off-road as ghosts indefinitely (steering early-returns on defunct edges while physics keeps integrating) until some unrelated covered editor action finally ran the handler.

What remains worth knowing:

- **Fix-up latency is ≤ 1 frame (~16 ms)** instead of same-event, but always lands *before* any cache rebuild or vehicle update in the tick — vehicles never simulate against a stale graph. One render frame may paint with a stale/defunct selection; every render path is guarded (bounds, `FromNode < 0`, NaN checks).
- Internal order still matters: the edge spatial grid rebuilds *first* because the vehicle re-snap below queries it ([GraphChangeHandler.cs:50-51](Roads.App/GraphChangeHandler.cs#L50-L51)). When a strip (or the pipeline's normalize) bumps the version after the handler's grid rebuild, the grid rebuilds once more in the same tick's pipeline — output-identical (flag-only writes), bounded, and it never fires during steady drags.
- Manual calls are unnecessary but harmless (idempotent, version-keyed early-out).

## 5. Map load sequence

The historical trap here is gone (Phase 4.5): the exemption/rotation setters (`SetEdgeExempt`/`SetExemptEdges` on the stop and yield systems, `RotatePhase`/`SetPhaseRotations` on the traffic-signal system) **grow their backing arrays on demand**, so [MapSerializer.Load](Roads.App/Persistence/MapSerializer.cs) applies overrides directly with no sizing rebuild first. Previously the setters bounds-checked and silently dropped out-of-range writes, forcing a rebuild-before-overrides order — on a fresh launch, reordering would have dropped *every* loaded override. What remains is inherent deserialization order, documented in code: `LoadFromData` clears lane restrictions (so the `SetLaneRestriction` loop follows it), and the caller must run `SimulationLoop.RebuildWorldCaches()` after `Load` returns — MainForm.LoadMap does so synchronously, which consumes the setters' dirty flags and normalizes flags before the first paint. MainForm also resets editor selections by hand ([MainForm.cs:458-469](Roads.App/MainForm.cs#L458-L469)).

One lifecycle contract to know: exemption/rotation override state **deliberately survives graph edits** (the rebuilds copy-preserve it), so any path that replaces the whole map must clear it explicitly or the old map's overrides silently apply to reused node/edge indices. `Load` does this via its clear-then-set setters; `NewMap` calls the same setters with empty lists.

## 6. Swap-and-pop removal invalidates vehicle indices

[VehicleStore.Remove](Roads.App/Vehicles/VehicleStore.cs) moves the last vehicle into the removed slot, invalidating any externally-held index of either slot. The per-call-site fixup contract is gone (Phase 4.5): index fixup is **centralized on the store itself**. `Remove` raises `VehicleStore.VehicleRemoving(removedIndex, swappedFromIndex)` before any mutation, and `ClearAll` (new map / load) raises `VehiclesCleared`; every index holder subscribes once:

- **PopulationManager** redirects the swapped vehicle's resident links (`Resident.VehicleIndex`, the `_vehicleToResident` dict) and detaches the removed vehicle's resident — an externally-removed mid-trip resident goes home dormant and departs again on its next schedule entry. On `VehiclesCleared` the population resets entirely, so a map swap while residents are driving can no longer hijack the new map's vehicles.
- **MainForm** retargets `EditorState.SelectedVehicle`/`HoveredVehicle` (clear if removed, follow if swapped); the store retargets its own `DiagVehicle` the same way.
- **SpatialGrid** (the vehicle grid) unlinks the removed index and re-links the swapped one in place, so editor-time hit-tests between per-step rebuilds never see a removed or relocated index. (The grid also rebuilds every paused tick, so hit-tests work on a freshly loaded, still-paused map.)

Any removal call site — including raw `_vehicles.Remove(i)` calls in GraphChangeHandler and VehicleSpawner, and any future ones — is therefore automatically safe; none carries fixup code. Watchdogs: a per-tick debug assert in SimulationLoop that the selection is in range, a reentrancy assert in `Remove` (handlers must not remove vehicles), and PopulationManager's per-tick mapping validation.

What remains:

- Removal loops must iterate **backward** (as [GraphChangeHandler does](Roads.App/GraphChangeHandler.cs#L64)) or they skip the swapped-in vehicle — this is about the loop's own iteration, not index fixup.
- Maintenance hazard: adding any per-vehicle field requires updating **five places in sync** (array declaration, `Add`, the swap block in `Remove`, `Grow`, and MapSerializer Save/Load + load-skip — or re-init in Load if the field is derived/transient). Missing the swap block makes the field silently leak between vehicles whenever a removal happens — the nastiest failure mode here. Phase 4.5 mitigates this with a co-located **five-step checklist banner at the top of [VehicleStore](Roads.App/Vehicles/VehicleStore.cs)**, cross-referenced from the `Add`/`Remove`/`Grow` docs and the MapSerializer vehicle Save loop; there is no automated guard, so the checklist is the enforcement.

## 7. Smaller contracts

- [SplitEdge](Roads.App/World/RoadGraph.cs#L1006) must capture the reverse edge *before* `SplitEdgeSingle` (adjacency goes stale mid-operation), and `SplitEdgeSingle` intentionally skips the `RebuildAdjacency` / `RebuildTurnMatrix` / `Version++` trio — every caller doing direct edge surgery must finish with those three.
- PopulationManager tracks the graph version **twice** ([POIRegistry's internal copy](Roads.App/Vehicles/PopulationManager.cs#L71) and its own [`_poiGraphVersion`](Roads.App/Vehicles/PopulationManager.cs#L88-L92)) with different reactions to a change — easy to update one and not the other.

## 8. Generative music — playback-thread sequencing contracts

The music path ([Audio/Music/](Roads.App/Audio/Music/)) runs the [Composer](Roads.App/Audio/Music/Composer.cs) entirely on NAudio's playback thread inside [MusicProvider.Read](Roads.App/Audio/Music/MusicProvider.cs); the UI thread only writes plain float targets (the AudioEngine idiom). Three contracts are invisible at the call sites:

- **Compose trigger is the playhead, not the queue.** `ComposeBar()` fires when `_pos >= _nextBarStart`. It must NOT be gated on queue exhaustion: note-offs overhanging a bar-line keep the queue non-empty, and gating on emptiness makes `Read` spin forever at the first bar boundary (this shipped broken once — the hang was live).
- **Same-sample MIDI ordering.** The event queue sort uses `MidiEvent.TieRank` (program/CC = 0, note-off = 1, note-on = 2) because `List.Sort` is unstable: without the rank, a section-boundary program change can land after the note it was meant to voice, and a repeated note can be chopped by its predecessor's note-off.
- **The music bus bypasses the pause duck.** [MasterProvider](Roads.App/Audio/Synth/MasterProvider.cs) applies `TargetDuck` only to the SFX mixer; music joins after the duck (band plays through pause — deliberate). Any new "silence everything" feature must handle both buses.
- **The soundfont is a copied asset.** `Assets/GeneralUser-GS.sf2` reaches the output dir via a csproj `CopyToOutputDirectory` entry; if it's missing at runtime the app silently runs without music (by design), so a broken copy step looks like a music bug.
- **Determinism invariant extends here.** Nothing under `Roads.App.Audio` (Music included) may touch SimRandom or sim state; the composer's RNG is private and mood arrives as copied floats via `AudioEngine.UpdateMusicMood`.

## 9. Scenery pipeline — settle-gated, ordered, and deliberately stale during edits

The procedural scenery renderers (TerrainRenderer, BuildingLayer/BuildingRenderer, PropRenderer, SignRenderer's speed-sign cache) are wired in [SceneRenderer.Render](Roads.App/Rendering/SceneRenderer.cs) with three contracts that differ from the rest of the render pass:

- **Settle-gate.** Building/prop/speed-sign *placement* is expensive (per-destination road sampling + OBB fitting; per-candidate multi-edge clearance queries). Node/control-point drags bump `graph.Version` every mouse-move frame, so a naive `Version`-keyed rebuild would re-run the full placement pass ~60×/sec for the whole drag. Instead SceneRenderer rebuilds scenery only after the version has held still for `SceneryRebuildDelayFrames` (the first build runs immediately). **Consequence: scenery is intentionally 1 settle-window stale during a drag** — this is safe only because placements bake world positions and node indices are stable across EDITS (nodes go defunct in place; the list never shrinks). It is NOT safe across a whole-map REPLACEMENT (`LoadFromData` swaps the node list, which can shrink, so an old footprint's node index can be out of range): every map-replacing call site (NewMap / LoadMap / GenerateStressScene) must call `SceneRenderer.OnMapReplaced()` after loading, which resets the gate for an immediate rebuild; `BuildingRenderer.TryBeginLocalFrame` also bounds-checks the node index as defense-in-depth. Roads/markings/vehicles still rebuild live for drag feedback. Terrain is exempt (purely positional, no graph dependency).
- **Build order within a settled frame.** `BuildingLayer.RebuildIfNeeded` → `CollectBounds` → `PropRenderer.Rebuild(bounds)`: props reject candidates inside building AABBs, so the building layer must rebuild and publish its bounds first. PropRenderer's early-out key is `(graphVersion, buildingBounds.Count)`; the settle-gate drives both from the same settled version so the count-based key never goes stale mid-drag.
- **Speed-sign cache vs. StopLineCache.** `SignRenderer` places speed-limit signs using `StopLineCache` trims, but that cache is rebuilt by `SimulationLoop.Tick`, not the render pass. Between an edit and the next tick the render pass can see a bumped `graph.Version` with a not-yet-rebuilt StopLineCache. SceneRenderer passes `allowRebuild: scenerySettled` so the speed-sign placement cache is only burned once the graph — and therefore its tick-driven caches — have settled, preventing a sign from being anchored at a stale trim after an edge split renumbers edges.

---

## Common thread

`graph.Version` works well as a lazy invalidation signal, but the system's correctness hangs on (a) every mutator bumping it (audited and fixed in Phase 4.5 — see §1) and (b) graph mutation during cache maintenance staying confined to the pipeline's normalize phase (debug-asserted — see §2). The old class of "remember to call X after Y" contracts is gone: graph-change fix-ups run automatically per tick (§4), the load-path setters size their own storage (§5), and vehicle-removal index fixup hangs off the store's own events (§6). The phase contracts in (b) and the signal systems' per-frame protocol (§3) are enforced by debug asserts; the remaining doc-only contracts are (a)'s forgotten-bump risk, backward iteration in removal loops (§6), the VehicleStore five-place field-sync requirement (§6, mitigated by a co-located checklist), ApplyMergeSpeedBias ordering, and SpatialGrid rebuild-before-queries.
