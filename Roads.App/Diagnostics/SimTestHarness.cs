using System.Diagnostics;
using System.Numerics;
using System.Text;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Persistence;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Diagnostics;

/// <summary>
/// Headless, REPRODUCIBLE simulation test harness. Replaces the manual in-app 'D' deadlock
/// capture with an automated run: load a map, step the EXACT per-substep simulation logic the
/// GUI runs (via <see cref="SimulationLoop.StepDeterministic"/>) for a fixed number of in-game
/// hours with NO wall-clock / accumulator / pause dependency, track stuck vehicles every substep,
/// then write a deadlock report ending in a sentinel line.
///
/// REPRODUCIBILITY: every simulation-affecting random draw (spawns, destinations, reroutes,
/// driver-personality traits, schedules, lane assignments, region-entry decisions, signal phase
/// offsets) routes through <see cref="SimRandom"/>. The harness seeds it with a fixed value at the
/// very start of <see cref="Run"/>, BEFORE any load/spawn/step, so two identical
/// <c>--simtest=map --simhours=N</c> invocations draw the IDENTICAL sequence and therefore produce
/// the IDENTICAL jam clusters and exit code. This is the property that makes the harness a valid
/// regression check: a pass/fail cannot flip on re-run. The GUI leaves <see cref="SimRandom"/>
/// unseeded (time-seeded), so its traffic is non-repeating exactly as before — seeding is the ONLY
/// behavioural difference between harness and GUI, and it is intentional.
///
/// StepDeterministic itself is additionally timing-independent (no wall-clock / accumulator /
/// Paused dependency); the seeded RNG is what upgrades that to full run-to-run reproducibility.
///
/// Wiring is replicated EXACTLY from MainForm so swap-pop removal index fixup and edge-split
/// exemption migration behave identically. Vehicles are intentionally NOT loaded — fresh
/// schedule/region-driven traffic is what produces the jams over time, and autosaves don't store
/// vehicles anyway.
///
/// ANOMALIES / Debug.Assert: unlike the GUI (where a failed Debug.Assert fail-fasts the process),
/// the headless harness installs an <see cref="AnomalyTraceListener"/> that records the assertion
/// text and CONTINUES (a headless run must not hang/die on a modal assert dialog). Any assertions
/// are surfaced in the report's ANOMALIES section. NOTE: once an assertion fires, the run has
/// pressed past a state the GUI would never have reached, so jam results AFTER the first captured
/// assertion are not guaranteed GUI-equivalent and should be treated with suspicion.
/// </summary>
public static class SimTestHarness
{
    private const float SimDt = SimulationLoop.SimDt;        // 1/30 s
    private const int SubstepsPerSecond = 30;
    private const int StuckThresholdSubsteps = 900;          // > 30 sim-seconds stopped

    private static StringBuilder? _reportAnomalies;

    /// <summary>
    /// Runs the headless harness. Returns the process exit code: 0 if zero jam clusters were
    /// found, 1 if any. Writes the report to <paramref name="outPath"/>. <paramref name="seed"/>
    /// seeds <see cref="SimRandom"/> so the run is reproducible — two calls with the same map,
    /// hours and seed produce identical results.
    /// </summary>
    public static int Run(string mapFile, float hours, string outPath, int seed = 12345)
    {
        // Make the run REPRODUCIBLE: seed the simulation RNG before ANY load/spawn/step so every
        // spawn/destination/reroute/personality/schedule/lane/region/signal draw is identical
        // across invocations. Must happen first — POIRegistry/population draws can occur during
        // Load and RebuildWorldCaches.
        SimRandom.Seed(seed);

        // Make Debug.Assert never pop a dialog or block the headless run: clear all listeners and
        // install one that records the assertion text as a report anomaly and continues.
        _reportAnomalies = new StringBuilder();
        Trace.Listeners.Clear();
        Trace.Listeners.Add(new AnomalyTraceListener(_reportAnomalies));

        var sw = Stopwatch.StartNew();

        // Resolve map path relative to the working directory if not absolute.
        string path = Path.IsPathRooted(mapFile) ? mapFile : Path.Combine(Environment.CurrentDirectory, mapFile);

        // ── HEADLESS WIRING (mirrors MainForm.cs ctor) ──────────────────────
        var camera = new Camera();
        var graph = new RoadGraph();
        var vehicles = new VehicleStore();
        var editorState = new EditorState();
        var vehicleGrid = new SpatialGrid();
        var stopLineCache = new StopLineCache();
        var edgeSpatialGrid = new EdgeSpatialGrid();
        var trafficSignals = new TrafficSignalSystem();
        var stopSigns = new StopSignSystem();
        var yieldSigns = new YieldSignSystem();
        var intersectionArcs = new IntersectionArcCache();
        var poiRegistry = new POIRegistry();

        var spawner = new VehicleSpawner(graph, vehicles, vehicleGrid);
        var population = new PopulationManager(graph, vehicles, vehicleGrid, poiRegistry, SimulationLoop.MaxVehicles);
        var gch = new GraphChangeHandler(graph, editorState, vehicles, edgeSpatialGrid, spawner);
        var sim = new SimulationLoop(graph, vehicles, vehicleGrid, stopLineCache, intersectionArcs,
            edgeSpatialGrid, trafficSignals, stopSigns, yieldSigns, spawner, population, editorState, gch);

        // Stuck-time tracker (sim-substep counters), index-aligned to the VehicleStore.
        int[] stuckSubsteps = Array.Empty<int>();
        int totalRemoved = 0;
        int peakLive = 0;

        // Match MainForm event wiring so swap-pop removal + edge-split exemption migration behave
        // identically. We additionally hook removal to carry the stuck counter and count removals.
        vehicles.VehicleRemoving += vehicleGrid.OnEntityRemoving;
        vehicles.VehiclesCleared += vehicleGrid.Clear;
        graph.EdgeSplit += stopSigns.OnEdgeSplit;
        graph.EdgeSplit += yieldSigns.OnEdgeSplit;
        vehicles.VehicleRemoving += (removed, swappedFrom) =>
        {
            totalRemoved++;
            // Carry the swapped index's counter into the removed slot (mirror MainForm.OnVehicleRemoving).
            if (swappedFrom >= 0 && removed >= 0 && removed < stuckSubsteps.Length && swappedFrom < stuckSubsteps.Length)
                stuckSubsteps[removed] = stuckSubsteps[swappedFrom];
        };

        // ── LOAD (no vehicles) ──────────────────────────────────────────────
        if (!File.Exists(path))
        {
            File.WriteAllText(outPath,
                $"SIMTEST ERROR: map file not found: {path}\n=== SIMTEST COMPLETE ===\n");
            return 1;
        }

        MapSerializer.Load(path, graph, vehicles, camera, sim.Clock, stopSigns, yieldSigns,
            trafficSignals, population, loadVehicles: false);
        sim.RebuildWorldCaches();
        sim.Paused = false;

        double clockStart = sim.Clock.TimeOfDay;
        int dayStart = sim.Clock.DayNumber;

        int totalSubsteps = (int)MathF.Round(hours * 3600f * SubstepsPerSecond);

        // ── STEP DETERMINISTICALLY, tracking stuck vehicles every substep ───
        for (int step = 0; step < totalSubsteps; step++)
        {
            sim.StepDeterministic(1);

            if (stuckSubsteps.Length < vehicles.Count)
                Array.Resize(ref stuckSubsteps, vehicles.Count + 64);

            for (int v = 0; v < vehicles.Count; v++)
            {
                if (vehicles.State[v] == VehicleState.Driving && vehicles.Speed[v] < DeadlockReport.StuckSpeedThreshold)
                    stuckSubsteps[v]++;
                else
                    stuckSubsteps[v] = 0;
            }

            if (vehicles.Count > peakLive) peakLive = vehicles.Count;
        }

        sw.Stop();

        double clockEnd = sim.Clock.TimeOfDay;
        int dayEnd = sim.Clock.DayNumber;
        int totalSpawned = totalRemoved + vehicles.Count; // load started with 0 vehicles

        var deps = new DeadlockReport.Deps
        {
            Graph = graph,
            Vehicles = vehicles,
            VehicleGrid = vehicleGrid,
            StopLines = stopLineCache,
            Arcs = intersectionArcs,
            TrafficSignals = trafficSignals,
            StopSigns = stopSigns,
            YieldSigns = yieldSigns,
        };

        int clusterCount = WriteReport(outPath, deps, mapFile, hours, clockStart, dayStart,
            clockEnd, dayEnd, sw.Elapsed.TotalSeconds, totalSpawned, totalRemoved, peakLive,
            vehicles.Count, stuckSubsteps);

        return clusterCount > 0 ? 1 : 0;
    }

    /// <summary>
    /// Writes the full deadlock report and returns the number of jam clusters found.
    /// </summary>
    private static int WriteReport(string outPath, DeadlockReport.Deps d, string mapFile, float hours,
        double clockStart, int dayStart, double clockEnd, int dayEnd, double wallSeconds,
        int totalSpawned, int totalRemoved, int peakLive, int liveNow, int[] stuckSubsteps)
    {
        var veh = d.Vehicles;
        var sb = new StringBuilder();

        sb.AppendLine("=== SIMTEST DEADLOCK REPORT ===");
        sb.AppendLine($"Map: {mapFile}");
        sb.AppendLine($"In-game hours simulated: {hours:F3}");
        // NOTE: MapSerializer.Load restores only TimeOfDay, not DayNumber (shared with the GUI load
        // path), so the day counter below starts from a fresh clock (0), not the saved map's day.
        // The day delta is meaningful; the absolute day number is not the saved map's day.
        sb.AppendLine($"In-game clock: day {dayStart} {FormatClock(clockStart)} -> day {dayEnd} {FormatClock(clockEnd)} (day# from fresh clock, not saved map)");
        sb.AppendLine($"Wall seconds: {wallSeconds:F2}");
        sb.AppendLine($"Vehicles spawned: {totalSpawned}  removed: {totalRemoved}  live at end: {liveNow}  peak live: {peakLive}");
        sb.AppendLine();

        // ── Identify stuck vehicles (continuously stopped > 30 sim-seconds) ──
        var stuck = new List<int>();
        for (int v = 0; v < veh.Count; v++)
            if (v < stuckSubsteps.Length && stuckSubsteps[v] > StuckThresholdSubsteps
                && veh.State[v] == VehicleState.Driving)
                stuck.Add(v);

        sb.AppendLine($"STUCK VEHICLES (Driving, <{DeadlockReport.StuckSpeedThreshold} m/s for >30 sim-s): {stuck.Count}");
        sb.AppendLine();

        foreach (int v in stuck)
            DeadlockReport.DumpVehicle(sb, d, v, stuckSubsteps[v]);

        // ── Group stuck vehicles into jam clusters by contended node ────────
        var clusters = new Dictionary<int, List<int>>();
        foreach (int v in stuck)
        {
            int node = DeadlockReport.ContendedNode(d, v);
            if (node < 0) node = -1; // "off-graph" bucket
            if (!clusters.TryGetValue(node, out var list)) { list = new List<int>(); clusters[node] = list; }
            list.Add(v);
        }

        sb.AppendLine($"JAM CLUSTERS (nodes hosting >=1 stuck vehicle): {clusters.Count}");
        sb.AppendLine();

        var clusterSummaries = new List<string>();

        foreach (var kv in clusters)
        {
            int node = kv.Key;
            var members = kv.Value;
            string type = node >= 0 ? DeadlockReport.NodeTypeLabel(d, node) : "off-graph";

            sb.AppendLine($"--- JAM CLUSTER @ node {node} ({type}), {members.Count} stuck vehicle(s): {string.Join(",", members)} ---");

            // Pick the "head" vehicle: the one stuck longest.
            int head = members[0];
            foreach (int v in members)
                if (stuckSubsteps[v] > stuckSubsteps[head]) head = v;

            if (node >= 0)
            {
                // Node-type-specific control state.
                if (d.StopSigns.IsStopSign(node))
                {
                    sb.AppendLine(d.StopSigns.DescribeNodeFull(d.Graph, node));
                }
                else if (d.TrafficSignals.IsTrafficLight(node))
                {
                    sb.AppendLine($"  TrafficLight phaseRotation={d.TrafficSignals.GetPhaseRotation(node)}");
                    foreach (int e in d.Graph.GetIncomingEdges(node))
                        sb.AppendLine($"    inEdge {e} ({d.Graph.Edges[e].FromNode}->{d.Graph.Edges[e].ToNode}): signal={d.TrafficSignals.GetSignal(e)}");
                }
                else if (d.YieldSigns.IsYield(node))
                {
                    foreach (int e in d.Graph.GetIncomingEdges(node))
                        sb.AppendLine($"    inEdge {e} ({d.Graph.Edges[e].FromNode}->{d.Graph.Edges[e].ToNode}): yield={d.YieldSigns.GetSignal(e, d.Graph)}");
                }
                else
                {
                    sb.AppendLine("  (uncontrolled node)");
                }

                // Arc occupancy of every arc at the node.
                sb.AppendLine("  Arc occupancy:");
                foreach (int a in d.Arcs.GetArcsAtNode(node))
                {
                    var ad = d.Arcs.GetArc(a);
                    string occ = DeadlockReport.VehiclesOnArc(d, a);
                    int occCount = occ == "(empty)" ? 0 : occ.Split(',').Length;
                    sb.AppendLine($"    arc {a} ({ad.IncomingEdge}->{ad.OutgoingEdge}) occupants={occCount}: {occ}");
                }
            }

            // Blocking chain from each head vehicle in the cluster (flags DEADLOCK CYCLE).
            sb.AppendLine("  Blocking chains:");
            bool anyCycle = false;
            foreach (int v in members)
            {
                string chain = DeadlockReport.BuildBlockingChain(d, v);
                if (chain.Contains("DEADLOCK CYCLE")) anyCycle = true;
                sb.AppendLine($"    from {v}: {chain}");
            }
            sb.AppendLine();

            // Machine summary line.
            string mechanism = ClassifyMechanism(d, node, type, head, anyCycle);
            int he = veh.CurrentEdge[head];
            string edges = "?";
            if (he >= 0 && he < d.Graph.Edges.Count && d.Graph.Edges[he].FromNode >= 0)
            {
                int next = -1;
                var path = veh.Path[head];
                int pi = veh.PathIndex[head];
                if (path != null && pi + 1 < path.Count) next = path[pi + 1];
                edges = $"{he}->{next}";
            }
            float stuckSec = stuckSubsteps[head] / (float)SubstepsPerSecond;
            clusterSummaries.Add($"node={node} type={type} mechanism={mechanism} head={head} edges={edges} stuckSec={stuckSec:F1}");
        }

        // ── Anomalies (captured Debug.Assert failures) ──────────────────────
        if (_reportAnomalies != null && _reportAnomalies.Length > 0)
        {
            sb.AppendLine("=== ANOMALIES (captured assertions) ===");
            sb.Append(_reportAnomalies);
            sb.AppendLine();
        }

        // ── Machine-readable cluster summary block ──────────────────────────
        sb.AppendLine("===CLUSTERS-BEGIN===");
        foreach (var line in clusterSummaries)
            sb.AppendLine(line);
        sb.AppendLine("===CLUSTERS-END===");

        sb.AppendLine("=== SIMTEST COMPLETE ===");

        File.WriteAllText(outPath, sb.ToString());
        return clusters.Count;
    }

    /// <summary>Best-effort short mechanism label for the machine summary line.</summary>
    private static string ClassifyMechanism(DeadlockReport.Deps d, int node, string type, int head, bool anyCycle)
    {
        if (anyCycle) return "merge-overlap";
        if (type == "StopSign") return "stopsign-starve";
        if (type == "TrafficLight") return "light-jam";
        // No detected cycle and held by a leader rather than a signal => queue cascade.
        int leader = DeadlockReport.NearestVehicleAhead(d, head, out _, out _);
        return leader >= 0 ? "cascade" : "signal-hold";
    }

    private static string FormatClock(double timeOfDay)
    {
        int h = (int)timeOfDay;
        int m = (int)((timeOfDay - h) * 60.0);
        return $"{h:D2}:{m:D2}";
    }

    /// <summary>
    /// Trace listener that records Debug.Assert / Trace failures as report anomalies instead of
    /// popping a modal dialog (which would hang the headless run). Never aborts the process.
    /// </summary>
    private sealed class AnomalyTraceListener : TraceListener
    {
        private readonly StringBuilder _sink;
        public AnomalyTraceListener(StringBuilder sink) => _sink = sink;

        public override void Fail(string? message) => _sink.AppendLine($"ASSERT: {message}");
        public override void Fail(string? message, string? detailMessage) =>
            _sink.AppendLine($"ASSERT: {message} | {detailMessage}");

        public override void Write(string? message) { }
        public override void WriteLine(string? message) { }
    }
}
