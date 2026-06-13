using System.Numerics;
using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Persistence;

/// <summary>
/// Binary save/load for road maps. File format version 1.
/// Saves the road graph (nodes, edges, lane restrictions, traffic control overrides),
/// camera state, simulation time, and optionally all vehicle state.
/// </summary>
public static class MapSerializer
{
    private static readonly byte[] Magic = "ROAD"u8.ToArray();
    private const ushort FormatVersion = 1;

    /// <summary>
    /// Saves the current map state to a binary file.
    /// </summary>
    public static void Save(string path, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        bool includeVehicles)
    {
        using var fs = File.Create(path);
        using var w = new BinaryWriter(fs);

        // Header
        w.Write(Magic);
        w.Write(FormatVersion);
        byte flags = (byte)(includeVehicles ? 1 : 0);
        w.Write(flags);
        w.Write((float)clock.TimeOfDay);

        // Section 1 — Nodes
        var nodes = graph.Nodes;
        w.Write(nodes.Count);
        for (int i = 0; i < nodes.Count; i++)
        {
            var n = nodes[i];
            w.Write(n.Position.X);
            w.Write(n.Position.Y);
            w.Write((byte)n.Flags);
            w.Write((byte)n.PointOfInterest);
        }

        // Section 2 — Edges
        var edges = graph.Edges;
        w.Write(edges.Count);
        for (int i = 0; i < edges.Count; i++)
        {
            var e = edges[i];
            w.Write(e.FromNode);
            w.Write(e.ToNode);
            w.Write(e.Length);
            w.Write(e.SpeedLimit);
            w.Write(e.LaneCount);
            w.Write((byte)e.RoadType);
            w.Write((byte)e.Flags);
            w.Write(e.ControlPoint1.X);
            w.Write(e.ControlPoint1.Y);
            w.Write(e.ControlPoint2.X);
            w.Write(e.ControlPoint2.Y);
        }

        // Section 3 — Lane Restrictions (user-customized only; auto defaults rebuilt on load)
        var restrictions = graph.GetUserLaneRestrictions().ToList();
        w.Write(restrictions.Count);
        foreach (var (key, pairs) in restrictions)
        {
            w.Write(key.inEdge);
            w.Write(key.inLane);
            w.Write(pairs.Count);
            foreach (var (outEdge, outLane) in pairs)
            {
                w.Write(outEdge);
                w.Write(outLane);
            }
        }

        // Section 4 — Traffic Control Overrides
        var stopExempt = stopSigns.GetExemptEdges();
        w.Write(stopExempt.Count);
        foreach (int e in stopExempt) w.Write(e);

        var yieldExempt = yieldSigns.GetExemptEdges();
        w.Write(yieldExempt.Count);
        foreach (int e in yieldExempt) w.Write(e);

        var phaseRotations = signals.GetPhaseRotations();
        w.Write(phaseRotations.Count);
        foreach (var (node, rot) in phaseRotations)
        {
            w.Write(node);
            w.Write(rot);
        }

        // Section 5 — Camera
        w.Write(camera.CenterX);
        w.Write(camera.CenterY);
        w.Write(camera.Zoom);

        // Section 6 — Vehicles (optional)
        //
        // Persist only DURABLE per-vehicle fields. Derived/transient fields (Throttle,
        // Brake, PrevHeadingError, DistToRoadSq, LaneChangeCooldown, DesiredLane,
        // MergeUrgency, MergeSpeedBias, SmoothedThrottle, SmoothedBrake, ResidentId,
        // State) are deliberately NOT written — Load re-initializes them. This Save loop,
        // the Load loop, and the load-skip branch must stay in sync and in the same field
        // order. See the field-sync checklist at the top of VehicleStore (step 5).
        if (includeVehicles)
        {
            w.Write(vehicles.Count);
            for (int i = 0; i < vehicles.Count; i++)
            {
                // Position & physics
                w.Write(vehicles.PosX[i]);
                w.Write(vehicles.PosY[i]);
                w.Write(vehicles.Heading[i]);
                w.Write(vehicles.Speed[i]);
                w.Write(vehicles.SteeringAngle[i]);

                // Edge tracking
                w.Write(vehicles.CurrentEdge[i]);
                w.Write(vehicles.EdgeProgress[i]);
                w.Write(vehicles.CurrentLane[i]);
                w.Write(vehicles.TargetLane[i]);
                w.Write(vehicles.LaneChangeProgress[i]);

                // Arc tracking
                w.Write(vehicles.CurrentArc[i]);
                w.Write(vehicles.ArcProgress[i]);

                // Destination
                w.Write(vehicles.DestinationNode[i]);

                // Path
                var vehPath = vehicles.Path[i];
                w.Write(vehicles.PathIndex[i]);
                if (vehPath != null)
                {
                    w.Write(vehPath.Count);
                    foreach (int edge in vehPath) w.Write(edge);
                }
                else
                {
                    w.Write(0);
                }

                // Personality
                w.Write(vehicles.Aggressiveness[i]);
                w.Write(vehicles.SpeedBias[i]);
                w.Write(vehicles.ReactionTime[i]);
                w.Write(vehicles.SteeringSharpness[i]);
                w.Write(vehicles.BrakingComfort[i]);
                w.Write(vehicles.LaneChangeBias[i]);
                w.Write(vehicles.PatienceTimer[i]);
                w.Write(vehicles.PreferredVehicle[i]);
                w.Write(vehicles.Archetype[i]);

                // Color
                w.Write(vehicles.ColorR[i]);
                w.Write(vehicles.ColorG[i]);
                w.Write(vehicles.ColorB[i]);
            }
        }
    }

    /// <summary>
    /// Loads a map from a binary file and applies it to the given simulation objects.
    /// Returns true if the file contained vehicle data.
    /// </summary>
    public static bool Load(string path, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        bool loadVehicles)
    {
        using var fs = File.OpenRead(path);
        using var r = new BinaryReader(fs);

        // Header
        var magic = r.ReadBytes(4);
        if (magic[0] != Magic[0] || magic[1] != Magic[1] ||
            magic[2] != Magic[2] || magic[3] != Magic[3])
            throw new InvalidDataException("Not a valid ROADS save file.");

        ushort version = r.ReadUInt16();
        if (version > FormatVersion)
            throw new InvalidDataException($"Save file version {version} is newer than supported ({FormatVersion}).");

        byte flags = r.ReadByte();
        bool hasVehicles = (flags & 1) != 0;
        float timeOfDay = r.ReadSingle();
        clock.TimeOfDay = timeOfDay;

        // Section 1 — Nodes
        int nodeCount = r.ReadInt32();
        var nodes = new List<RoadNode>(nodeCount);
        for (int i = 0; i < nodeCount; i++)
        {
            nodes.Add(new RoadNode
            {
                Position = new Vector2(r.ReadSingle(), r.ReadSingle()),
                Flags = (NodeFlags)r.ReadByte(),
                PointOfInterest = (POIType)r.ReadByte()
            });
        }

        // Section 2 — Edges
        int edgeCount = r.ReadInt32();
        var edges = new List<RoadEdge>(edgeCount);
        for (int i = 0; i < edgeCount; i++)
        {
            edges.Add(new RoadEdge
            {
                FromNode = r.ReadInt32(),
                ToNode = r.ReadInt32(),
                Length = r.ReadSingle(),
                SpeedLimit = r.ReadSingle(),
                LaneCount = r.ReadByte(),
                RoadType = (RoadType)r.ReadByte(),
                Flags = (EdgeFlags)r.ReadByte(),
                ControlPoint1 = new Vector2(r.ReadSingle(), r.ReadSingle()),
                ControlPoint2 = new Vector2(r.ReadSingle(), r.ReadSingle())
            });
        }

        // Load graph (rebuilds adjacency and turn matrix)
        graph.LoadFromData(nodes, edges);

        // Section 3 — Lane Restrictions
        int restrictionCount = r.ReadInt32();
        for (int i = 0; i < restrictionCount; i++)
        {
            int inEdge = r.ReadInt32();
            byte inLane = r.ReadByte();
            int pairCount = r.ReadInt32();
            var pairs = new HashSet<(int, byte)>(pairCount);
            for (int j = 0; j < pairCount; j++)
                pairs.Add((r.ReadInt32(), r.ReadByte()));
            graph.SetLaneRestriction(inEdge, inLane, pairs);
        }

        // Section 4 — Traffic Control Overrides
        int stopExemptCount = r.ReadInt32();
        var stopExempt = new List<int>(stopExemptCount);
        for (int i = 0; i < stopExemptCount; i++) stopExempt.Add(r.ReadInt32());

        int yieldExemptCount = r.ReadInt32();
        var yieldExempt = new List<int>(yieldExemptCount);
        for (int i = 0; i < yieldExemptCount; i++) yieldExempt.Add(r.ReadInt32());

        int phaseCount = r.ReadInt32();
        var phaseRotations = new List<(int, byte)>(phaseCount);
        for (int i = 0; i < phaseCount; i++)
            phaseRotations.Add((r.ReadInt32(), r.ReadByte()));

        // Apply traffic-control overrides directly — the setters grow their own storage
        // on demand and mark the systems dirty, so no sizing rebuild is needed first.
        // The caller's RebuildWorldCaches after Load returns re-derives all dependent
        // state (and normalizes flags).
        stopSigns.SetExemptEdges(stopExempt);
        yieldSigns.SetExemptEdges(yieldExempt);
        signals.SetPhaseRotations(phaseRotations);

        // Section 5 — Camera
        camera.CenterX = r.ReadSingle();
        camera.CenterY = r.ReadSingle();
        camera.Zoom = r.ReadSingle();

        // Section 6 — Vehicles
        vehicles.ClearAll();

        if (hasVehicles && loadVehicles)
        {
            int vehicleCount = r.ReadInt32();
            vehicles.SetCount(vehicleCount);

            for (int i = 0; i < vehicleCount; i++)
            {
                // Position & physics
                vehicles.PosX[i] = r.ReadSingle();
                vehicles.PosY[i] = r.ReadSingle();
                vehicles.Heading[i] = r.ReadSingle();
                vehicles.Speed[i] = r.ReadSingle();
                vehicles.SteeringAngle[i] = r.ReadSingle();

                // Edge tracking
                vehicles.CurrentEdge[i] = r.ReadInt32();
                vehicles.EdgeProgress[i] = r.ReadSingle();
                vehicles.CurrentLane[i] = r.ReadByte();
                vehicles.TargetLane[i] = r.ReadByte();
                vehicles.LaneChangeProgress[i] = r.ReadSingle();

                // Arc tracking
                vehicles.CurrentArc[i] = r.ReadInt32();
                vehicles.ArcProgress[i] = r.ReadSingle();

                // Destination
                vehicles.DestinationNode[i] = r.ReadInt32();

                // Path
                vehicles.PathIndex[i] = r.ReadInt32();
                int pathLen = r.ReadInt32();
                if (pathLen > 0)
                {
                    var vehPath = new List<int>(pathLen);
                    for (int j = 0; j < pathLen; j++) vehPath.Add(r.ReadInt32());
                    vehicles.Path[i] = vehPath;
                }
                else
                {
                    vehicles.Path[i] = null;
                }

                // Personality
                vehicles.Aggressiveness[i] = r.ReadSingle();
                vehicles.SpeedBias[i] = r.ReadSingle();
                vehicles.ReactionTime[i] = r.ReadSingle();
                vehicles.SteeringSharpness[i] = r.ReadSingle();
                vehicles.BrakingComfort[i] = r.ReadSingle();
                vehicles.LaneChangeBias[i] = r.ReadSingle();
                vehicles.PatienceTimer[i] = r.ReadSingle();
                vehicles.PreferredVehicle[i] = r.ReadByte();
                vehicles.Archetype[i] = r.ReadByte();

                // Color
                vehicles.ColorR[i] = r.ReadByte();
                vehicles.ColorG[i] = r.ReadByte();
                vehicles.ColorB[i] = r.ReadByte();

                // Initialize derived fields
                vehicles.Throttle[i] = 0f;
                vehicles.Brake[i] = 0f;
                vehicles.PrevHeadingError[i] = 0f;
                vehicles.DistToRoadSq[i] = 0f;
                vehicles.LaneChangeCooldown[i] = 0f;
                vehicles.DesiredLane[i] = byte.MaxValue;
                vehicles.MergeUrgency[i] = 0f;
                vehicles.MergeSpeedBias[i] = 0f;
                vehicles.SmoothedThrottle[i] = 0f;
                vehicles.SmoothedBrake[i] = 0f;
                vehicles.ResidentId[i] = -1;
                vehicles.State[i] = VehicleState.Driving;
            }
        }
        else if (hasVehicles)
        {
            // Skip vehicle section without loading
            int vehicleCount = r.ReadInt32();
            for (int i = 0; i < vehicleCount; i++)
            {
                // Skip 5 floats (pos, heading, speed, steering)
                r.ReadBytes(5 * 4);
                // Skip edge tracking: int, float, 2 bytes, float
                r.ReadBytes(4 + 4 + 1 + 1 + 4);
                // Skip arc: int, float
                r.ReadBytes(4 + 4);
                // Skip destination: int
                r.ReadBytes(4);
                // Skip path: pathIndex (int) + pathLen (int) + pathLen * int
                r.ReadBytes(4); // pathIndex
                int pathLen = r.ReadInt32();
                if (pathLen > 0) r.ReadBytes(pathLen * 4);
                // Skip personality: 7 floats + 2 bytes
                r.ReadBytes(7 * 4 + 2);
                // Skip color: 3 bytes
                r.ReadBytes(3);
            }
        }

        return hasVehicles;
    }
}
