using System.Diagnostics;
using System.Text;
using Roads.App.Core;
using Roads.App.Editor;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Diagnostics;

/// <summary>
/// Headless steering-stability regression harness (CLI <c>--steerprobe</c>, writes
/// steerprobe.log). For each case it builds a 3 km straight two-way road with the case's
/// speed limit, places ONE vehicle of a fixed type/sharpness cruising at the limit, lets
/// it settle 15 s, applies a 1 m lateral nudge, then measures 30 s: steering sign flips
/// per second (higher = ringing; ~30/s = the once-per-tick flip-flop), mean distance from
/// lane center over the final 5 s, and drift from the pre-nudge lateral position. Steps
/// the EXACT GUI substep logic via SimulationLoop.StepDeterministic (no spawner or
/// population interference: the map has no POIs). Healthy output: every case ≲0.3
/// flips/s, tail distance ≲0.4 m, drift ≈0.
///
/// The case matrix spans the steering loop's gain axes — speed, wheelbase (motorcycle vs
/// bus), and driver SteeringSharpness — because the plant gain is speed/wheelbase ×
/// sharpness (see SteeringController.SpeedGainCompensation). Run it after ANY change to
/// the PD gains, SpeedGainCompensation/MaxYawGain, lookahead, or the bicycle-model
/// kinematics: the 45+ mph flip-flop this harness reproduces was invisible in whole-map
/// deadlock soaks (oscillating cars still make progress) and easy to miss visually.
/// </summary>
public static class SteeringProbeHarness
{
    private const float SimDt = SimulationLoop.SimDt; // 1/30 s
    private const int SettleSubsteps = 15 * 30;
    private const int MeasureSubsteps = 30 * 30;
    private const int TailWindow = 5 * 30;            // "final 5 s" stats window
    private const float FlipEpsilon = 0.002f;         // rad; ignore micro-jitter around zero
    private const float Mph = 0.44704f;               // m/s per mph

    private record struct Case(string Label, byte VehicleType, float Sharpness, float SpeedMph);

    public static int Run(string outPath)
    {
        var cases = new[]
        {
            new Case("sedan      @25mph", (byte)VehicleType.Sedan, 1.0f, 25f),
            new Case("sedan      @45mph", (byte)VehicleType.Sedan, 1.0f, 45f),
            new Case("sedan      @70mph", (byte)VehicleType.Sedan, 1.0f, 70f),
            new Case("motorcycle @25mph", (byte)VehicleType.Motorcycle, 1.0f, 25f),
            new Case("motorcycle @45mph", (byte)VehicleType.Motorcycle, 1.0f, 45f),
            new Case("bus        @45mph", (byte)VehicleType.Bus, 1.0f, 45f),
            new Case("sharp s1.6 @25mph", (byte)VehicleType.Sedan, 1.6f, 25f),
            new Case("sharp s1.6 @45mph", (byte)VehicleType.Sedan, 1.6f, 45f),
        };

        // Headless: swallow Debug.Assert dialogs (SimTestHarness idiom).
        Trace.Listeners.Clear();
        var anomalies = new StringBuilder();
        Trace.Listeners.Add(new ProbeTraceListener(anomalies));

        var sb = new StringBuilder();
        sb.AppendLine("=== STEERING OSCILLATION PROBE ===");
        sb.AppendLine("protocol: settle 15s at speed limit, +1m lateral nudge, measure 30s");
        sb.AppendLine("flips/s: steering sign reversals per second (|steer| > 0.002 rad on both sides)");
        sb.AppendLine();
        sb.AppendLine("case                | settle flips/s | measure flips/s | tail dist from lane ctr | drift from pre-nudge");
        sb.AppendLine("--------------------|----------------|-----------------|-------------------------|---------------------");

        foreach (var c in cases)
        {
            var r = RunCase(c);
            sb.AppendLine($"{c.Label} | {r.settleFlipsPerSec,14:F1} | {r.measureFlipsPerSec,15:F1} | {r.tailMeanDist,21:F3} m | {r.driftFromPreNudge,17:F3} m");
        }

        if (anomalies.Length > 0)
        {
            sb.AppendLine();
            sb.AppendLine("=== ANOMALIES (captured assertions) ===");
            sb.Append(anomalies);
        }

        sb.AppendLine("=== STEERPROBE COMPLETE ===");
        File.WriteAllText(outPath, sb.ToString());
        return 0;
    }

    private static (float settleFlipsPerSec, float measureFlipsPerSec, float tailMeanDist, float driftFromPreNudge)
        RunCase(Case c)
    {
        SimRandom.Seed(4242);

        float speed = c.SpeedMph * Mph;

        // ── World: one straight 3 km two-way road along +X, no POIs/signals ──
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
        vehicles.VehicleRemoving += vehicleGrid.OnEntityRemoving;
        vehicles.VehiclesCleared += vehicleGrid.Clear;

        // A straight 3 km two-way road built as a chain of 100 m edges — the segment
        // length real maps use, and the length FindNearestT's t-space search window
        // assumes (a single km-scale edge would quantize projection far coarser than
        // the steering lookahead and invalidate the probe).
        const int segments = 30;
        const float segLen = 100f;
        var nodes = new List<RoadNode>();
        for (int i = 0; i <= segments; i++)
            nodes.Add(new RoadNode { Position = new System.Numerics.Vector2(i * segLen, 0f) });
        var edges = new List<RoadEdge>();
        var forwardPath = new List<int>();
        for (int i = 0; i < segments; i++)
        {
            forwardPath.Add(edges.Count);
            edges.Add(MakeStraightEdge(i, i + 1, nodes[i].Position, nodes[i + 1].Position, speed));
            edges.Add(MakeStraightEdge(i + 1, i, nodes[i + 1].Position, nodes[i].Position, speed));
        }
        graph.LoadFromData(nodes, edges);
        sim.RebuildWorldCaches();
        sim.Paused = false;

        // ── One vehicle mid-edge, cruising at the limit, fixed traits ──
        float startT = 0.5f;
        var pos = graph.EvaluateBezier(0, startT);
        int vi = vehicles.Add(pos.X, pos.Y, 0f, 0);
        vehicles.Aggressiveness[vi] = 0.4f;
        vehicles.SpeedBias[vi] = 1.0f;
        vehicles.ReactionTime[vi] = 0.6f;
        vehicles.SteeringSharpness[vi] = c.Sharpness;
        vehicles.BrakingComfort[vi] = 2.5f;
        vehicles.LaneChangeBias[vi] = 0f;
        vehicles.PatienceTimer[vi] = 60f;
        vehicles.PreferredVehicle[vi] = c.VehicleType;
        vehicles.Archetype[vi] = (byte)DriverArchetype.Commuter;
        vehicles.Path[vi] = forwardPath;
        vehicles.PathIndex[vi] = 0;
        vehicles.EdgeProgress[vi] = startT;
        vehicles.DestinationNode[vi] = segments;
        vehicles.Speed[vi] = speed;

        // ── Settle 15 s; count flips over its final 5 s (pre-nudge micro-oscillation) ──
        int settleFlips = 0;
        float lastSign = 0f;
        for (int step = 0; step < SettleSubsteps; step++)
        {
            sim.StepDeterministic(1);
            if (step >= SettleSubsteps - TailWindow)
                CountFlip(vehicles.SteeringAngle[vi], ref lastSign, ref settleFlips);
        }
        float preNudgeY = vehicles.PosY[vi];

        // ── 1 m lateral nudge, then measure 30 s ──
        vehicles.PosY[vi] += 1f;
        int measureFlips = 0;
        lastSign = 0f;
        double tailDistSum = 0;
        int tailSamples = 0;
        for (int step = 0; step < MeasureSubsteps; step++)
        {
            sim.StepDeterministic(1);
            CountFlip(vehicles.SteeringAngle[vi], ref lastSign, ref measureFlips);
            if (step >= MeasureSubsteps - TailWindow)
            {
                tailDistSum += MathF.Sqrt(vehicles.DistToRoadSq[vi]);
                tailSamples++;
            }
        }

        return (
            settleFlips / (TailWindow * SimDt),
            measureFlips / (MeasureSubsteps * SimDt),
            (float)(tailDistSum / Math.Max(1, tailSamples)),
            vehicles.PosY[vi] - preNudgeY);
    }

    private static RoadEdge MakeStraightEdge(int from, int to,
        System.Numerics.Vector2 a, System.Numerics.Vector2 b, float speedLimit)
    {
        return new RoadEdge
        {
            FromNode = from,
            ToNode = to,
            Length = (b - a).Length(),
            SpeedLimit = speedLimit,
            LaneCount = 1,
            RoadType = RoadType.Arterial,
            Flags = 0,
            ControlPoint1 = a + (b - a) / 3f,
            ControlPoint2 = a + (b - a) * (2f / 3f),
        };
    }

    /// <summary>Counts a flip when the steering angle crosses from beyond +eps to beyond
    /// -eps (or vice versa); dwell inside the deadband does not reset the sign.</summary>
    private static void CountFlip(float steer, ref float lastSign, ref int flips)
    {
        float sign = steer > FlipEpsilon ? 1f : steer < -FlipEpsilon ? -1f : 0f;
        if (sign != 0f)
        {
            if (lastSign != 0f && sign != lastSign) flips++;
            lastSign = sign;
        }
    }

    private sealed class ProbeTraceListener : TraceListener
    {
        private readonly StringBuilder _sink;
        public ProbeTraceListener(StringBuilder sink) => _sink = sink;
        public override void Fail(string? message) => _sink.AppendLine($"ASSERT: {message}");
        public override void Fail(string? message, string? detailMessage) =>
            _sink.AppendLine($"ASSERT: {message} | {detailMessage}");
        public override void Write(string? message) { }
        public override void WriteLine(string? message) { }
    }
}
