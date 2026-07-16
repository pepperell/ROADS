using System.Text.Json;
using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Persistence;

/// <summary>
/// Human-readable JSON snapshot of a saved map. Parallel to <see cref="MapSerializer"/>
/// (binary); shares the same parameter set and walk order so a single call site can drive both.
///
/// JSON schema (top-level object):
/// <code>
/// {
///   "version": 2,
///   "timeOfDay": 8.0,
///   "includesVehicles": true,
///   "camera": { "centerX": 0, "centerY": 0, "zoom": 5 },
///   "nodes": [
///     { "x": 0, "y": 0, "flags": 0, "flagNames": "StopSign", "poi": "Home" }
///   ],
///   "edges": [
///     {
///       "fromNode": 0, "toNode": 1,
///       "length": 100.0, "speedLimit": 11.2, "laneCount": 1,
///       "roadType": "Residential", "flags": 0,
///       "cp1": { "x": 10, "y": 0 }, "cp2": { "x": 90, "y": 0 }
///     }
///   ],
///   "laneRestrictions": [
///     {
///       "inEdge": 0, "inLane": 0,
///       "allowed": [ { "outEdge": 1, "outLane": 0 } ]
///     }
///   ],
///   "trafficControl": {
///     "stopExemptEdges": [],
///     "yieldExemptEdges": [],
///     "signalPhaseRotations": [ { "node": 3, "rotation": 1 } ]
///   },
///   "vehicles": [
///     {
///       "posX": 50, "posY": 0, "heading": 0, "speed": 12.5, "steeringAngle": 0,
///       "currentEdge": 0, "edgeProgress": 0.5, "currentLane": 0, "targetLane": 0,
///       "laneChangeProgress": 0, "currentArc": -1, "arcProgress": 0,
///       "destinationNode": 5,
///       "pathIndex": 2, "path": [0, 1, 2, 3],
///       "aggressiveness": 0.4, "speedBias": 1.0, "reactionTime": 0.6,
///       "steeringSharpness": 1.0, "brakingComfort": 2.5, "laneChangeBias": 0.5,
///       "patienceTimer": 30, "preferredVehicle": 0, "archetype": "Commuter",
///       "color": "#FF3C3C3C"
///     }
///   ],
///   "population": [
///     {
///       "id": 0, "homeNode": 2, "workNode": 7,
///       "traits": {
///         "archetype": "Commuter",
///         "aggressiveness": 0.4, "speedBias": 1.0, "reactionTime": 0.6,
///         "steeringSharpness": 1.0, "brakingComfort": 2.5, "laneChangeBias": 0.5,
///         "patienceTimer": 30, "preferredVehicle": 0
///       },
///       "color": "#FFC8C8C8",
///       "schedule": [ { "departureTime": 8.0, "destination": "Work" } ],
///       "scheduleIndex": 1,
///       "activity": "Dormant",
///       "currentPOINode": 7,
///       "vehicleIndex": -1
///     }
///   ]
/// }
/// </code>
///
/// Call order: construct the serializer-side data first, then call Save. This method is
/// purely read-only with respect to all arguments; it never mutates graph, vehicles,
/// signals, population, camera, or clock.
/// </summary>
public static class MapJsonSerializer
{
    private const int SchemaVersion = 3;

    /// <summary>
    /// Writes a human-readable JSON snapshot of the current map state to <paramref name="path"/>.
    /// The parameter set is identical to <see cref="MapSerializer.Save"/> so both serializers
    /// can be driven from a single call site. When <paramref name="includeVehicles"/> is false
    /// the <c>vehicles</c> and <c>population</c> arrays are omitted from the output.
    /// </summary>
    /// <param name="path">Destination file path; created or overwritten.</param>
    /// <param name="graph">Road graph (nodes and edges).</param>
    /// <param name="vehicles">Active vehicle store.</param>
    /// <param name="camera">Current camera state.</param>
    /// <param name="clock">Simulation clock supplying time-of-day.</param>
    /// <param name="stopSigns">Stop-sign system for exempt-edge overrides.</param>
    /// <param name="yieldSigns">Yield-sign system for exempt-edge overrides.</param>
    /// <param name="signals">Traffic-signal system for phase-rotation overrides.</param>
    /// <param name="population">Population manager owning the resident list.</param>
    /// <param name="water">Painted water layer (schema v3; always written).</param>
    /// <param name="includeVehicles">When true, serializes vehicles and the resident population.</param>
    public static void Save(string path, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        PopulationManager population, WaterLayer water, bool includeVehicles)
    {
        using var fs = File.Create(path);
        using var writer = new Utf8JsonWriter(fs, new JsonWriterOptions { Indented = true });

        writer.WriteStartObject();

        // ── Header ────────────────────────────────────────────────────────────
        writer.WriteNumber("version", SchemaVersion);
        writer.WriteNumber("timeOfDay", clock.TimeOfDay);
        writer.WriteBoolean("includesVehicles", includeVehicles);

        // ── Camera ────────────────────────────────────────────────────────────
        writer.WriteStartObject("camera");
        writer.WriteNumber("centerX", camera.CenterX);
        writer.WriteNumber("centerY", camera.CenterY);
        writer.WriteNumber("zoom", camera.Zoom);
        writer.WriteEndObject();

        // ── Section 1 — Nodes ─────────────────────────────────────────────────
        var nodes = graph.Nodes;
        writer.WriteStartArray("nodes");
        for (int i = 0; i < nodes.Count; i++)
        {
            var n = nodes[i];
            writer.WriteStartObject();
            writer.WriteNumber("x", n.Position.X);
            writer.WriteNumber("y", n.Position.Y);
            writer.WriteNumber("flags", (byte)n.Flags);
            writer.WriteString("flagNames", n.Flags == NodeFlags.None ? "None" : n.Flags.ToString());
            writer.WriteString("poi", n.PointOfInterest.ToString());
            writer.WriteEndObject();
        }
        writer.WriteEndArray();

        // ── Section 2 — Edges ─────────────────────────────────────────────────
        var edges = graph.Edges;
        writer.WriteStartArray("edges");
        for (int i = 0; i < edges.Count; i++)
        {
            var e = edges[i];
            writer.WriteStartObject();
            writer.WriteNumber("fromNode", e.FromNode);
            writer.WriteNumber("toNode", e.ToNode);
            writer.WriteNumber("length", e.Length);
            writer.WriteNumber("speedLimit", e.SpeedLimit);
            writer.WriteNumber("laneCount", e.LaneCount);
            writer.WriteString("roadType", e.RoadType.ToString());
            writer.WriteNumber("flags", (ushort)e.Flags);
            writer.WriteStartObject("cp1");
            writer.WriteNumber("x", e.ControlPoint1.X);
            writer.WriteNumber("y", e.ControlPoint1.Y);
            writer.WriteEndObject();
            writer.WriteStartObject("cp2");
            writer.WriteNumber("x", e.ControlPoint2.X);
            writer.WriteNumber("y", e.ControlPoint2.Y);
            writer.WriteEndObject();
            writer.WriteEndObject();
        }
        writer.WriteEndArray();

        // ── Section 3 — Lane Restrictions ─────────────────────────────────────
        var restrictions = graph.GetUserLaneRestrictions().ToList();
        writer.WriteStartArray("laneRestrictions");
        foreach (var (key, pairs) in restrictions)
        {
            writer.WriteStartObject();
            writer.WriteNumber("inEdge", key.inEdge);
            writer.WriteNumber("inLane", key.inLane);
            writer.WriteStartArray("allowed");
            foreach (var (outEdge, outLane) in pairs)
            {
                writer.WriteStartObject();
                writer.WriteNumber("outEdge", outEdge);
                writer.WriteNumber("outLane", outLane);
                writer.WriteEndObject();
            }
            writer.WriteEndArray();
            writer.WriteEndObject();
        }
        writer.WriteEndArray();

        // ── Section 4 — Traffic Control Overrides ─────────────────────────────
        writer.WriteStartObject("trafficControl");

        var stopExempt = stopSigns.GetExemptEdges();
        writer.WriteStartArray("stopExemptEdges");
        foreach (int e in stopExempt) writer.WriteNumberValue(e);
        writer.WriteEndArray();

        var yieldExempt = yieldSigns.GetExemptEdges();
        writer.WriteStartArray("yieldExemptEdges");
        foreach (int e in yieldExempt) writer.WriteNumberValue(e);
        writer.WriteEndArray();

        var phaseRotations = signals.GetPhaseRotations();
        writer.WriteStartArray("signalPhaseRotations");
        foreach (var (node, rot) in phaseRotations)
        {
            writer.WriteStartObject();
            writer.WriteNumber("node", node);
            writer.WriteNumber("rotation", rot);
            writer.WriteEndObject();
        }
        writer.WriteEndArray();

        writer.WriteEndObject(); // trafficControl

        // ── Section 5 — Vehicles (optional) ───────────────────────────────────
        // Only durable fields are written; transient/derived fields are excluded
        // to mirror exactly what MapSerializer persists. See the field-sync checklist
        // in VehicleStore (step 5) for the authoritative list of durable vs. transient.
        if (includeVehicles)
        {
            writer.WriteStartArray("vehicles");
            for (int i = 0; i < vehicles.Count; i++)
            {
                writer.WriteStartObject();

                // Position & physics
                writer.WriteNumber("posX", vehicles.PosX[i]);
                writer.WriteNumber("posY", vehicles.PosY[i]);
                writer.WriteNumber("heading", vehicles.Heading[i]);
                writer.WriteNumber("speed", vehicles.Speed[i]);
                writer.WriteNumber("steeringAngle", vehicles.SteeringAngle[i]);

                // Edge tracking
                writer.WriteNumber("currentEdge", vehicles.CurrentEdge[i]);
                writer.WriteNumber("edgeProgress", vehicles.EdgeProgress[i]);
                writer.WriteNumber("currentLane", vehicles.CurrentLane[i]);
                writer.WriteNumber("targetLane", vehicles.TargetLane[i]);
                writer.WriteNumber("laneChangeProgress", vehicles.LaneChangeProgress[i]);

                // Arc tracking (written for completeness; stale on reload, same as binary format)
                writer.WriteNumber("currentArc", vehicles.CurrentArc[i]);
                writer.WriteNumber("arcProgress", vehicles.ArcProgress[i]);

                // Destination
                writer.WriteNumber("destinationNode", vehicles.DestinationNode[i]);

                // Path
                writer.WriteNumber("pathIndex", vehicles.PathIndex[i]);
                var vehPath = vehicles.Path[i];
                writer.WriteStartArray("path");
                if (vehPath != null)
                    foreach (int edge in vehPath) writer.WriteNumberValue(edge);
                writer.WriteEndArray();

                // Personality
                writer.WriteNumber("aggressiveness", vehicles.Aggressiveness[i]);
                writer.WriteNumber("speedBias", vehicles.SpeedBias[i]);
                writer.WriteNumber("reactionTime", vehicles.ReactionTime[i]);
                writer.WriteNumber("steeringSharpness", vehicles.SteeringSharpness[i]);
                writer.WriteNumber("brakingComfort", vehicles.BrakingComfort[i]);
                writer.WriteNumber("laneChangeBias", vehicles.LaneChangeBias[i]);
                writer.WriteNumber("patienceTimer", vehicles.PatienceTimer[i]);
                writer.WriteNumber("preferredVehicle", vehicles.PreferredVehicle[i]);
                writer.WriteString("archetype", ((DriverArchetype)vehicles.Archetype[i]).ToString());

                // Color — encoded as "#AARRGGBB" hex for readability
                writer.WriteString("color",
                    $"#{255:X2}{vehicles.ColorR[i]:X2}{vehicles.ColorG[i]:X2}{vehicles.ColorB[i]:X2}");

                writer.WriteEndObject();
            }
            writer.WriteEndArray();

            // ── Section 6 — Population ────────────────────────────────────────
            // Written only alongside vehicles; a driving resident holds a vehicle index.
            var residents = population.Residents;
            writer.WriteStartArray("population");
            foreach (var res in residents)
            {
                writer.WriteStartObject();

                writer.WriteNumber("id", res.Id);
                writer.WriteNumber("homeNode", res.HomeNode);
                writer.WriteNumber("workNode", res.WorkNode);

                // Traits
                var t = res.Traits;
                writer.WriteStartObject("traits");
                writer.WriteString("archetype", t.Archetype.ToString());
                writer.WriteNumber("aggressiveness", t.Aggressiveness);
                writer.WriteNumber("speedBias", t.SpeedBias);
                writer.WriteNumber("reactionTime", t.ReactionTime);
                writer.WriteNumber("steeringSharpness", t.SteeringSharpness);
                writer.WriteNumber("brakingComfort", t.BrakingComfort);
                writer.WriteNumber("laneChangeBias", t.LaneChangeBias);
                writer.WriteNumber("patienceTimer", t.PatienceTimer);
                writer.WriteNumber("preferredVehicle", t.PreferredVehicle);
                writer.WriteEndObject();

                // Color
                writer.WriteString("color",
                    $"#{255:X2}{res.ColorR:X2}{res.ColorG:X2}{res.ColorB:X2}");

                // Schedule
                writer.WriteStartArray("schedule");
                foreach (var entry in res.Schedule)
                {
                    writer.WriteStartObject();
                    writer.WriteNumber("departureTime", entry.DepartureTime);
                    writer.WriteString("destination", entry.Destination.ToString());
                    writer.WriteEndObject();
                }
                writer.WriteEndArray();

                writer.WriteNumber("scheduleIndex", res.ScheduleIndex);
                writer.WriteString("activity", res.Activity.ToString());
                writer.WriteNumber("currentPOINode", res.CurrentPOINode);
                writer.WriteNumber("vehicleIndex", res.VehicleIndex);

                writer.WriteEndObject();
            }
            writer.WriteEndArray();
        }

        // ── Water (schema v3; mirrors MapSerializer Section 8, always written) ─
        writer.WriteStartObject("water");
        writer.WriteStartArray("circles");
        foreach (var c in water.Circles)
        {
            writer.WriteStartObject();
            writer.WriteNumber("x", c.Center.X);
            writer.WriteNumber("y", c.Center.Y);
            writer.WriteNumber("radius", c.Radius);
            writer.WriteEndObject();
        }
        writer.WriteEndArray();
        writer.WriteStartArray("streams");
        foreach (var s in water.Segments)
        {
            writer.WriteStartObject();
            writer.WriteNumber("p0x", s.P0.X); writer.WriteNumber("p0y", s.P0.Y);
            writer.WriteNumber("c1x", s.C1.X); writer.WriteNumber("c1y", s.C1.Y);
            writer.WriteNumber("c2x", s.C2.X); writer.WriteNumber("c2y", s.C2.Y);
            writer.WriteNumber("p3x", s.P3.X); writer.WriteNumber("p3y", s.P3.Y);
            writer.WriteNumber("width", s.Width);
            writer.WriteEndObject();
        }
        writer.WriteEndArray();
        writer.WriteEndObject();

        writer.WriteEndObject(); // root
    }
}
