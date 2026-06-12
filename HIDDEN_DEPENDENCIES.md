# Hidden Dependencies: Ordering Contracts Between Variables and Functions

This document catalogues the implicit ordering contracts in the codebase — places where updating a variable requires a follow-up function call, or where functions must run in a specific order, with nothing but doc comments and call-site discipline enforcing it. Phase 4.5 in [DESIGN.md](DESIGN.md) tracks the work to make these contracts explicit, self-enforcing, or structurally unnecessary.

Line references are accurate as of the end of Phase 4 and will drift as the code changes. Update this document as Phase 4.5 items land.

---

## 1. `RoadGraph.Version` is an invalidation bus

Nearly every cache keys off [RoadGraph.Version](Roads.App/World/RoadGraph.cs#L67): StopLineCache, IntersectionArcCache, EdgeSpatialGrid, all three signal systems, RoadRenderer's path cache, VehicleSpawner's spawn-point cache, POIRegistry, PopulationManager, and GraphChangeHandler. The contract: **any mutation of nodes/edges/lane-restrictions must `Version++`, or every downstream system silently serves stale data** (bounds checks make failures silent, not crashes). The full contract lives on the [Version property's XML doc](Roads.App/World/RoadGraph.cs#L50-L67); this section is its prose home.

As of the Phase 4.5 audit, **every public mutator bumps Version**. The three former violators now bump:

- [AddNode](Roads.App/World/RoadGraph.cs#L100) bumps unconditionally (previously a lone node — e.g. the Road tool's first click — was invisible to every version consumer).
- [SetLaneRestriction](Roads.App/World/RoadGraph.cs#L1371) bumps unconditionally — it is now a safe general-purpose mutator instead of one that only worked inside MapSerializer.Load's window (riding on `LoadFromData`'s bump).
- [StripMarkerFlagsFromIntersections](Roads.App/World/RoadGraph.cs#L943) bumps conditionally, only when it actually strips flags, so the converging follow-up pass triggered through GraphChangeHandler strips nothing and settles (same convergence pattern as the signal auto-assignment in §2).

Two **private** surgery helpers intentionally mutate without bumping — `MarkNodeDefunct` and `SplitEdgeSingle` — and their XML docs state that the public caller (`RemoveEdge`/`RemoveNode`, `SplitEdge`) bumps once per operation. Consumers compare versions for equality only, so the multiple bumps inside one operation (e.g. `SplitEdge`'s internal `AddNode` plus its own final bump) coalesce into a single lazy rebuild.

Remaining risk is additive only: a future public mutator that forgets the bump. No runtime check can catch that from the consumer side — version-keyed caches compare for equality and cannot distinguish "unchanged" from "mutated without a bump" — so this contract remains doc-enforced (the Phase 4.5 frame-protocol guards assert staleness relative to the *reported* version, not unreported mutations).

## 2. Graph mutation during cache maintenance — confined to a normalize phase

Historically the rebuild chain mutated the graph mid-rebuild: signal auto-assignment called `graph.SetNodeFlags` inside the systems' `RebuildIfNeeded` (each call bumping Version *during* a rebuild keyed on Version, so **every version-keyed cache rebuilt a second time next tick** — the full double cascade), and `ApplyDefaultLaneRestrictions` bumped between the stop-line and arc rebuilds (re-rebuilding only stop lines next tick). Resolved in Phase 4.5: [SimulationLoop.RebuildWorldCaches](Roads.App/SimulationLoop.cs#L161) now runs two phases:

1. **Normalize** — the ONLY place graph mutation is permitted during cache maintenance. Guarded: runs only when `graph.Version` changed since the last normalize. Order contract: [TrafficSignalSystem.AutoAssign](Roads.App/World/TrafficSignalSystem.cs) → [StopSignSystem.AutoAssign](Roads.App/World/StopSignSystem.cs) (lights first — the stop policy reads the TrafficLight flag) → stop-line rebuild → [ApplyDefaultLaneRestrictions](Roads.App/World/RoadGraph.cs) (reads stop-line tangents). No step reads what a later step writes, so a single pass converges — verified at runtime by a debug-only second pass asserting no further bumps.
2. **Rebuild** — pure projections of the settled graph into caches (StopLineCache, IntersectionArcCache, EdgeSpatialGrid, the three signal systems). A debug assert verifies no step mutates the graph.

After `RebuildWorldCaches` returns, every cache is current with `graph.Version`: a single edit triggers exactly one cascade, settled within the same call. The chain is called from the paused branch, every active fixed step, and MainForm's NewMap/LoadMap; the ordering contracts live in the method's XML doc.

## 3. Per-frame protocols enforced only by doc comments

- All three signal systems have a per-frame contract: `RebuildIfNeeded` → `Update` → `GetSignal` — **now debug-asserted** (Phase 4.5). `Update` asserts version currency in all three systems; stop/yield `GetSignal` assert the full protocol via the public `CanQuery(graph)` predicate (rebuilt at the current version, not dirty, and `Update` has run since the last real rebuild); traffic `GetSignal` asserts only that a rebuild has ever run, because the renderer legitimately reads it every frame including paused. Diagnostic callers that may run while right-of-way data is stale gate on `CanQuery` — MainForm's D-key vehicle dump prints `stopSign=n/a yield=n/a (right-of-way tracking stale)` in that case. When the graph changed, static `AutoAssign` must precede `RebuildIfNeeded` for the traffic-light and stop-sign systems (rebuilds are pure flag projections; the pipeline's normalize phase enforces this — see §2).
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

[VehicleStore.Remove](Roads.App/Vehicles/VehicleStore.cs#L241-L297) moves the last vehicle into the removed slot. Three hidden contracts follow:

- Removal loops must iterate **backward** (as [GraphChangeHandler does](Roads.App/GraphChangeHandler.cs#L61)) or they skip the swapped-in vehicle.
- Anything holding a vehicle index must do fixup using the return value. [PopulationManager.RemoveVehicle](Roads.App/Vehicles/PopulationManager.cs#L467-L489) is the canonical dance — and even the order of its dictionary removals matters (old index key removed *after* the swap).
- `EditorState.SelectedVehicle` is only cleared when out of range ([SimulationLoop.cs:149](Roads.App/SimulationLoop.cs#L149)) — after a swap the same index silently refers to a *different* car.
- Maintenance hazard: adding any per-vehicle field requires updating **four places in sync** (array declaration, `Add`, the swap block in `Remove`, `Grow`) plus MapSerializer Save/Load. Missing the swap block makes the field silently leak between vehicles whenever a removal happens — the nastiest failure mode here.

## 7. Smaller contracts

- [SplitEdge](Roads.App/World/RoadGraph.cs#L1006) must capture the reverse edge *before* `SplitEdgeSingle` (adjacency goes stale mid-operation), and `SplitEdgeSingle` intentionally skips the `RebuildAdjacency` / `RebuildTurnMatrix` / `Version++` trio — every caller doing direct edge surgery must finish with those three.
- [SimulationLoop.cs:138](Roads.App/SimulationLoop.cs#L138): `_spawner.ScheduleModeActive` is hand-copied from `_populationManager.ScheduleModeEnabled` each tick — it must land after `Population.Update` and before `AutoSpawn`.
- PopulationManager tracks the graph version **twice** ([POIRegistry's internal copy](Roads.App/Vehicles/PopulationManager.cs#L71) and its own [`_poiGraphVersion`](Roads.App/Vehicles/PopulationManager.cs#L88-L92)) with different reactions to a change — easy to update one and not the other.

---

## Common thread

`graph.Version` works well as a lazy invalidation signal, but the system's correctness hangs on (a) every mutator bumping it (audited and fixed in Phase 4.5 — see §1) and (b) graph mutation during cache maintenance staying confined to the pipeline's normalize phase (debug-asserted — see §2). The old class of "remember to call X after Y" contracts is gone: graph-change fix-ups run automatically per tick (§4) and the load-path setters size their own storage (§5). The phase contracts in (b) and the signal systems' per-frame protocol (§3) are enforced by debug asserts; the remaining doc-only contracts are (a)'s forgotten-bump risk, ApplyMergeSpeedBias ordering, and SpatialGrid rebuild-before-queries.
