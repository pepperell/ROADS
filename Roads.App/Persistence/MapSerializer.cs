using System.Numerics;
using Roads.App.Core;
using Roads.App.Vehicles;
using Roads.App.World;

namespace Roads.App.Persistence;

/// <summary>
/// Binary save/load for road maps. File format version 3.
/// Saves the road graph (nodes, edges, lane restrictions, traffic control overrides),
/// camera state, simulation time, the water layer (v3 — always present, even when
/// vehicles are not saved), and optionally all vehicle state plus the resident
/// population (v2). v1 files (no population section) and v2 files (no water section)
/// still load — their vehicles fall back to legacy handling and their water is empty.
/// Load is all-or-nothing: the entire stream is parsed and index-validated into
/// temporaries before any live object is touched, so a truncated or corrupt file
/// throws while the previous world remains fully intact.
/// </summary>
public static class MapSerializer
{
    private static readonly byte[] Magic = "ROAD"u8.ToArray();
    private const ushort FormatVersion = 3;

    /// <summary>
    /// Saves the current map state to a binary file. Crash-safe: the bytes are written to
    /// a temporary sibling file, flushed to disk, and only then moved over the target — a
    /// disk-full error, crash, or power loss mid-write therefore never destroys the
    /// existing save (the worst case is a leftover <c>*.tmp</c> beside it, deleted
    /// best-effort and overwritten by the next save). No separate <c>.bak</c> is kept:
    /// AutoSaveManager's rotating backups already cover historical copies.
    /// </summary>
    public static void Save(string path, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        PopulationManager population, WaterLayer water, bool includeVehicles)
    {
        string tmpPath = path + ".tmp";
        try
        {
            using (var fs = File.Create(tmpPath))
            using (var w = new BinaryWriter(fs))
            {
                WriteMapData(w, graph, vehicles, camera, clock, stopSigns, yieldSigns,
                    signals, population, water, includeVehicles);
                w.Flush();
                fs.Flush(flushToDisk: true); // durable before the move commits it
            }
            File.Move(tmpPath, path, overwrite: true);
        }
        catch
        {
            try { File.Delete(tmpPath); } catch { /* best-effort cleanup */ }
            throw;
        }
    }

    /// <summary>Writes the complete map payload (header + sections 1–8) to
    /// <paramref name="w"/> — the format core wrapped by <see cref="Save"/>'s
    /// crash-safe temp-file-then-move commit.</summary>
    private static void WriteMapData(BinaryWriter w, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        PopulationManager population, WaterLayer water, bool includeVehicles)
    {
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
        // State, ClearingArc) are deliberately NOT written — Load re-initializes them. This Save loop,
        // ReadVehicle + the commit copy loop in Load, and the load-skip branch must stay
        // in sync and in the same field order. See the field-sync checklist at the top
        // of VehicleStore (step 5).
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

        // Section 7 — Population (v2). Saved only with vehicles, since a driving resident
        // references a vehicle index. The vehicle↔resident link is derived from each driving
        // resident's VehicleIndex on load, so it is NOT duplicated in the vehicle section.
        if (includeVehicles)
        {
            var residents = population.Residents;
            w.Write(residents.Count);
            foreach (var res in residents)
            {
                w.Write(res.Id);
                w.Write(res.HomeNode);
                w.Write(res.WorkNode);

                var t = res.Traits;
                w.Write((byte)t.Archetype);
                w.Write(t.Aggressiveness);
                w.Write(t.SpeedBias);
                w.Write(t.ReactionTime);
                w.Write(t.SteeringSharpness);
                w.Write(t.BrakingComfort);
                w.Write(t.LaneChangeBias);
                w.Write(t.PatienceTimer);
                w.Write(t.PreferredVehicle);

                w.Write(res.ColorR);
                w.Write(res.ColorG);
                w.Write(res.ColorB);

                w.Write(res.Schedule.Length);
                foreach (var entry in res.Schedule)
                {
                    w.Write(entry.DepartureTime);
                    w.Write((byte)entry.Destination);
                }

                w.Write(res.ScheduleIndex);
                w.Write((byte)res.Activity);
                w.Write(res.CurrentPOINode);
                w.Write(res.VehicleIndex);
            }
        }

        // Section 8 — Water (v3). Always written, regardless of the vehicles flag, so the
        // section's position is deterministic: directly after Camera when vehicles were
        // not saved, after Population when they were.
        w.Write(water.Circles.Count);
        foreach (var c in water.Circles)
        {
            w.Write(c.Center.X);
            w.Write(c.Center.Y);
            w.Write(c.Radius);
        }
        w.Write(water.Segments.Count);
        foreach (var s in water.Segments)
        {
            w.Write(s.P0.X); w.Write(s.P0.Y);
            w.Write(s.C1.X); w.Write(s.C1.Y);
            w.Write(s.C2.X); w.Write(s.C2.Y);
            w.Write(s.P3.X); w.Write(s.P3.Y);
            w.Write(s.Width);
        }
    }

    /// <summary>
    /// Loads a map from a binary file and applies it to the given simulation objects.
    /// All-or-nothing (see the stream overload): on any failure the live world is left
    /// untouched. Returns true if the file contained vehicle data.
    /// </summary>
    public static bool Load(string path, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        PopulationManager population, WaterLayer water, bool loadVehicles)
    {
        using var fs = File.OpenRead(path);
        return Load(fs, graph, vehicles, camera, clock, stopSigns, yieldSigns, signals,
            population, water, loadVehicles);
    }

    /// <summary>
    /// Stream-based core of the path overload above — also the direct entry for the maps
    /// embedded in the assembly (the title backdrop and New template), which never exist
    /// on disk. Consumes and disposes the stream. Returns true if the data contained
    /// vehicles.
    /// All-or-nothing: the entire stream is parsed and index-validated into temporaries
    /// BEFORE any live object is touched, and the commit phase does no reads and no
    /// unguarded indexing — a truncated or corrupt file therefore throws while the
    /// previous world (graph, vehicles, population, signals, water, clock, camera)
    /// remains fully intact, and callers may keep their current quiet-save target.
    /// </summary>
    public static bool Load(Stream stream, RoadGraph graph, VehicleStore vehicles,
        Camera camera, SimulationClock clock,
        StopSignSystem stopSigns, YieldSignSystem yieldSigns, TrafficSignalSystem signals,
        PopulationManager population, WaterLayer water, bool loadVehicles)
    {
        using var r = new BinaryReader(stream);

        // ── Parse phase ─────────────────────────────────────────────────────────
        // Reads into temporaries only; validates every index the commit phase (or the
        // running sim) will use for direct array access. List preallocation is capacity-
        // clamped so a corrupt element count fails as EndOfStreamException during the
        // reads rather than as a giant up-front allocation.

        // Header
        var magic = r.ReadBytes(4);
        if (magic.Length != 4 || magic[0] != Magic[0] || magic[1] != Magic[1] ||
            magic[2] != Magic[2] || magic[3] != Magic[3])
            throw new InvalidDataException("Not a valid ROADS save file.");

        ushort version = r.ReadUInt16();
        if (version > FormatVersion)
            throw new InvalidDataException($"Save file version {version} is newer than supported ({FormatVersion}).");

        byte flags = r.ReadByte();
        bool hasVehicles = (flags & 1) != 0;
        float timeOfDay = r.ReadSingle();

        // Section 1 — Nodes. Flags are masked to the currently defined bits so legacy files
        // carrying retired flags (bit 8 was Spawn, bit 64 was RegionSpawn) load clean.
        const byte validNodeFlags = (byte)(NodeFlags.TrafficLight | NodeFlags.StopSign
            | NodeFlags.Yield | NodeFlags.ManualSignal | NodeFlags.Destination
            | NodeFlags.ActuatedSignal);
        int nodeCount = ReadCount(r, "node");
        var nodes = new List<RoadNode>(PreallocCapacity(nodeCount));
        for (int i = 0; i < nodeCount; i++)
        {
            nodes.Add(new RoadNode
            {
                Position = new Vector2(r.ReadSingle(), r.ReadSingle()),
                Flags = (NodeFlags)(r.ReadByte() & validNodeFlags),
                PointOfInterest = (POIType)r.ReadByte()
            });
        }

        // Section 2 — Edges. Endpoints are validated here because the commit phase
        // indexes nodes by them when rebuilding adjacency and turn matrices. Defunct
        // edges are persisted with FromNode == -1 and are skipped by those rebuilds.
        int edgeCount = ReadCount(r, "edge");
        var edges = new List<RoadEdge>(PreallocCapacity(edgeCount));
        for (int i = 0; i < edgeCount; i++)
        {
            var e = new RoadEdge
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
            };
            if (e.FromNode >= nodeCount
                || (e.FromNode >= 0 && (uint)e.ToNode >= (uint)nodeCount))
                throw new InvalidDataException($"Corrupt save: edge {i} references a node out of range.");
            edges.Add(e);
        }

        // Section 3 — Lane Restrictions. Edge indices above the edge count are corrupt;
        // negatives are tolerated (the commit-phase setter guards them). Entries that
        // reference lanes at/above their edge's lane count are HEALED rather than
        // rejected: pre-fix saves could carry orphans from lane-count shrinks (see
        // RoadGraph.PruneLaneRestrictionsForLaneCount). Dropped: keys whose in-lane no
        // longer exists, pairs targeting a removed out-lane, and any set left empty by
        // that pair-filtering (an empty set means zero arcs for the lane; auto/geometry
        // defaults are the safe interpretation). Sets saved empty stay empty as saved.
        int restrictionCount = ReadCount(r, "lane restriction");
        var restrictions = new List<(int inEdge, byte inLane, HashSet<(int outEdge, byte outLane)> pairs)>(
            PreallocCapacity(restrictionCount));
        for (int i = 0; i < restrictionCount; i++)
        {
            int inEdge = r.ReadInt32();
            byte inLane = r.ReadByte();
            int pairCount = ReadCount(r, "lane restriction pair");
            var pairs = new HashSet<(int, byte)>(PreallocCapacity(pairCount));
            int droppedPairs = 0;
            for (int j = 0; j < pairCount; j++)
            {
                int outEdge = r.ReadInt32();
                byte outLane = r.ReadByte();
                if (outEdge >= edgeCount)
                    throw new InvalidDataException($"Corrupt save: lane restriction {i} references an edge out of range.");
                if (outEdge >= 0 && outLane >= edges[outEdge].LaneCount) { droppedPairs++; continue; }
                pairs.Add((outEdge, outLane));
            }
            if (inEdge >= edgeCount)
                throw new InvalidDataException($"Corrupt save: lane restriction {i} references an edge out of range.");
            if (inEdge >= 0 && inLane >= edges[inEdge].LaneCount) continue; // orphaned key
            if (droppedPairs > 0 && pairs.Count == 0) continue;             // emptied by heal
            restrictions.Add((inEdge, inLane, pairs));
        }

        // Section 4 — Traffic Control Overrides
        int stopExemptCount = ReadCount(r, "stop exemption");
        var stopExempt = new List<int>(PreallocCapacity(stopExemptCount));
        for (int i = 0; i < stopExemptCount; i++)
            stopExempt.Add(ReadIndex(r, edgeCount, "stop exemption edge"));

        int yieldExemptCount = ReadCount(r, "yield exemption");
        var yieldExempt = new List<int>(PreallocCapacity(yieldExemptCount));
        for (int i = 0; i < yieldExemptCount; i++)
            yieldExempt.Add(ReadIndex(r, edgeCount, "yield exemption edge"));

        int phaseCount = ReadCount(r, "phase rotation");
        var phaseRotations = new List<(int, byte)>(PreallocCapacity(phaseCount));
        for (int i = 0; i < phaseCount; i++)
            phaseRotations.Add((ReadIndex(r, nodeCount, "phase rotation node"), r.ReadByte()));

        // Section 5 — Camera
        float camCenterX = r.ReadSingle();
        float camCenterY = r.ReadSingle();
        float camZoom = r.ReadSingle();

        // Section 6 — Vehicles
        List<LoadedVehicle>? loadedVehicles = null;
        if (hasVehicles && loadVehicles)
        {
            int vehicleCount = ReadCount(r, "vehicle");
            loadedVehicles = new List<LoadedVehicle>(PreallocCapacity(vehicleCount));
            for (int i = 0; i < vehicleCount; i++)
                loadedVehicles.Add(ReadVehicle(r, nodeCount, edgeCount, i));
        }
        else if (hasVehicles)
        {
            // Skip vehicle section without loading
            int vehicleCount = ReadCount(r, "vehicle");
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
                int pathLen = ReadCount(r, "path length");
                if (pathLen > 0) r.ReadBytes(pathLen * 4);
                // Skip personality: 7 floats + 2 bytes
                r.ReadBytes(7 * 4 + 2);
                // Skip color: 3 bytes
                r.ReadBytes(3);
            }
        }

        // Section 7 — Population (v2+; present only when the file was saved with vehicles).
        // Always read to consume the bytes; kept for the commit phase only when vehicles
        // were loaded (a driving resident references a loaded vehicle index). When vehicles
        // are skipped, the population is discarded and the schedule system regenerates fresh.
        List<Resident>? loadedResidents = null;
        if (version >= 2 && hasVehicles)
        {
            int residentCount = ReadCount(r, "resident");
            var residents = new List<Resident>(PreallocCapacity(residentCount));
            for (int i = 0; i < residentCount; i++)
                residents.Add(ReadResident(r, nodeCount));
            if (loadVehicles)
                loadedResidents = residents;
        }

        // Section 8 — Water (v3+). Always present in v3 regardless of the vehicles flag;
        // null (older files) commits as a cleared layer so the loaded map has no water.
        List<WaterCircle>? waterCircles = null;
        List<WaterSegment>? waterSegments = null;
        if (version >= 3)
        {
            int circleCount = ReadCount(r, "water circle");
            waterCircles = new List<WaterCircle>(PreallocCapacity(circleCount));
            for (int i = 0; i < circleCount; i++)
            {
                waterCircles.Add(new WaterCircle
                {
                    Center = new Vector2(r.ReadSingle(), r.ReadSingle()),
                    Radius = r.ReadSingle(),
                });
            }
            int segmentCount = ReadCount(r, "water segment");
            waterSegments = new List<WaterSegment>(PreallocCapacity(segmentCount));
            for (int i = 0; i < segmentCount; i++)
            {
                waterSegments.Add(new WaterSegment
                {
                    P0 = new Vector2(r.ReadSingle(), r.ReadSingle()),
                    C1 = new Vector2(r.ReadSingle(), r.ReadSingle()),
                    C2 = new Vector2(r.ReadSingle(), r.ReadSingle()),
                    P3 = new Vector2(r.ReadSingle(), r.ReadSingle()),
                    Width = r.ReadSingle(),
                });
            }
        }

        // ── Commit phase ────────────────────────────────────────────────────────
        // Replaces the live world from the validated temporaries above. No stream reads
        // and no unguarded indexing happen from here on, so there is no failure window
        // that could leave a hybrid of old and new state. KEEP IT THAT WAY: a new file
        // section must be parsed and validated above, then applied here.

        clock.TimeOfDay = timeOfDay;

        // Replace graph (rebuilds adjacency and turn matrix)
        graph.LoadFromData(nodes, edges);

        // Heal Bézier handles corrupted by the pre-fix MoveNode (handle lengths not
        // rescaled on node drags) — saved maps may carry edges whose skewed
        // parametrization breaks Δt·Length distance math. Idempotent; no-op on
        // healthy maps. Edge indices are unchanged, so the sections below are safe.
        int healed = graph.NormalizeDegenerateHandles();
        if (healed > 0)
            System.Diagnostics.Debug.WriteLine($"[MapSerializer] Normalized degenerate Bezier handles on {healed} edge(s)");

        foreach (var (inEdge, inLane, pairs) in restrictions)
            graph.SetLaneRestriction(inEdge, inLane, pairs);

        // Apply traffic-control overrides directly — the setters grow their own storage
        // on demand and mark the systems dirty, so no sizing rebuild is needed first.
        // The caller's RebuildWorldCaches after Load returns re-derives all dependent
        // state (and normalizes flags).
        stopSigns.SetExemptEdges(stopExempt);
        yieldSigns.SetExemptEdges(yieldExempt);
        signals.SetPhaseRotations(phaseRotations);

        camera.CenterX = camCenterX;
        camera.CenterY = camCenterY;
        camera.Zoom = camZoom;

        vehicles.ClearAll();
        if (loadedVehicles != null)
        {
            vehicles.SetCount(loadedVehicles.Count);
            for (int i = 0; i < loadedVehicles.Count; i++)
            {
                var v = loadedVehicles[i];

                // Durable fields (mirrors the Save loop / ReadVehicle field order)
                vehicles.PosX[i] = v.PosX;
                vehicles.PosY[i] = v.PosY;
                vehicles.Heading[i] = v.Heading;
                vehicles.Speed[i] = v.Speed;
                vehicles.SteeringAngle[i] = v.SteeringAngle;
                vehicles.CurrentEdge[i] = v.CurrentEdge;
                vehicles.EdgeProgress[i] = v.EdgeProgress;
                vehicles.CurrentLane[i] = v.CurrentLane;
                vehicles.TargetLane[i] = v.TargetLane;
                vehicles.LaneChangeProgress[i] = v.LaneChangeProgress;
                vehicles.DestinationNode[i] = v.DestinationNode;
                vehicles.PathIndex[i] = v.PathIndex;
                vehicles.Path[i] = v.Path;
                vehicles.Aggressiveness[i] = v.Aggressiveness;
                vehicles.SpeedBias[i] = v.SpeedBias;
                vehicles.ReactionTime[i] = v.ReactionTime;
                vehicles.SteeringSharpness[i] = v.SteeringSharpness;
                vehicles.BrakingComfort[i] = v.BrakingComfort;
                vehicles.LaneChangeBias[i] = v.LaneChangeBias;
                vehicles.PatienceTimer[i] = v.PatienceTimer;
                vehicles.PreferredVehicle[i] = v.PreferredVehicle;
                vehicles.Archetype[i] = v.Archetype;
                vehicles.ColorR[i] = v.ColorR;
                vehicles.ColorG[i] = v.ColorG;
                vehicles.ColorB[i] = v.ColorB;

                // Initialize derived fields
                vehicles.CurrentArc[i] = -1;
                vehicles.ArcProgress[i] = 0f;
                vehicles.ClearingArc[i] = -1;
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

        if (loadedResidents != null)
            population.AdoptLoadedPopulation(loadedResidents);

        if (waterCircles != null && waterSegments != null)
            water.LoadFromData(waterCircles, waterSegments);
        else
            water.Clear();

        return hasVehicles;
    }

    /// <summary>Cap on list preallocation from file-supplied element counts. Lists grow
    /// past it normally for legitimately large maps; its only job is turning a corrupt
    /// multi-million count into an EndOfStreamException during the parse reads instead
    /// of a giant up-front allocation.</summary>
    private const int MaxPreallocCapacity = 1 << 16;

    private static int PreallocCapacity(int count) => Math.Min(count, MaxPreallocCapacity);

    /// <summary>Reads a section element count, rejecting negative values.</summary>
    private static int ReadCount(BinaryReader r, string what)
    {
        int count = r.ReadInt32();
        if (count < 0)
            throw new InvalidDataException($"Corrupt save: negative {what} count ({count}).");
        return count;
    }

    /// <summary>Reads a node/edge index that must be within <paramref name="limit"/>.
    /// Negatives are rejected too: these sections never use sentinel indices.</summary>
    private static int ReadIndex(BinaryReader r, int limit, string what)
    {
        int index = r.ReadInt32();
        if ((uint)index >= (uint)limit)
            throw new InvalidDataException($"Corrupt save: {what} {index} out of range.");
        return index;
    }

    /// <summary>One vehicle's durable fields parsed into a temporary, applied to the
    /// <see cref="VehicleStore"/> by the commit phase. Field set and order are governed
    /// by the field-sync checklist at the top of VehicleStore (step 5).</summary>
    private sealed class LoadedVehicle
    {
        public float PosX, PosY, Heading, Speed, SteeringAngle;
        public int CurrentEdge;
        public float EdgeProgress;
        public byte CurrentLane, TargetLane;
        public float LaneChangeProgress;
        public int DestinationNode;
        public int PathIndex;
        public List<int>? Path;
        public float Aggressiveness, SpeedBias, ReactionTime, SteeringSharpness,
            BrakingComfort, LaneChangeBias, PatienceTimer;
        public byte PreferredVehicle, Archetype;
        public byte ColorR, ColorG, ColorB;
    }

    /// <summary>Reads one serialized vehicle (durable fields only, in Save-loop order)
    /// and validates the indices the sim dereferences without guards: the current edge
    /// and every path edge must be real, the destination at most needs to exist
    /// (negative destination sentinels pass through).</summary>
    private static LoadedVehicle ReadVehicle(BinaryReader r, int nodeCount, int edgeCount, int i)
    {
        var v = new LoadedVehicle
        {
            // Position & physics
            PosX = r.ReadSingle(),
            PosY = r.ReadSingle(),
            Heading = r.ReadSingle(),
            Speed = r.ReadSingle(),
            SteeringAngle = r.ReadSingle(),

            // Edge tracking
            CurrentEdge = r.ReadInt32(),
            EdgeProgress = r.ReadSingle(),
            CurrentLane = r.ReadByte(),
            TargetLane = r.ReadByte(),
            LaneChangeProgress = r.ReadSingle(),
        };
        if ((uint)v.CurrentEdge >= (uint)edgeCount)
            throw new InvalidDataException($"Corrupt save: vehicle {i} is on an edge out of range.");

        // Arc tracking — read for format compatibility but DISCARD: arc indices are
        // runtime-only (the IntersectionArcCache is rebuilt on load, so a saved index
        // is stale and may be out of range). The vehicle re-acquires an arc at its
        // next intersection; the commit phase re-initializes it with the other
        // transient/derived fields.
        r.ReadInt32();   // saved CurrentArc (ignored)
        r.ReadSingle();  // saved ArcProgress (ignored)

        // Destination
        v.DestinationNode = r.ReadInt32();
        if (v.DestinationNode >= nodeCount)
            throw new InvalidDataException($"Corrupt save: vehicle {i} destination node out of range.");

        // Path
        v.PathIndex = r.ReadInt32();
        int pathLen = ReadCount(r, "path length");
        if (pathLen > 0)
        {
            var vehPath = new List<int>(PreallocCapacity(pathLen));
            for (int j = 0; j < pathLen; j++)
            {
                int edge = r.ReadInt32();
                if ((uint)edge >= (uint)edgeCount)
                    throw new InvalidDataException($"Corrupt save: vehicle {i} path references an edge out of range.");
                vehPath.Add(edge);
            }
            v.Path = vehPath;
        }
        else
        {
            v.Path = null;
        }

        // Personality
        v.Aggressiveness = r.ReadSingle();
        v.SpeedBias = r.ReadSingle();
        v.ReactionTime = r.ReadSingle();
        v.SteeringSharpness = r.ReadSingle();
        v.BrakingComfort = r.ReadSingle();
        v.LaneChangeBias = r.ReadSingle();
        v.PatienceTimer = r.ReadSingle();
        v.PreferredVehicle = r.ReadByte();
        v.Archetype = r.ReadByte();

        // Color
        v.ColorR = r.ReadByte();
        v.ColorG = r.ReadByte();
        v.ColorB = r.ReadByte();

        return v;
    }

    /// <summary>Reads one serialized <see cref="Resident"/> (v2 population section),
    /// validating its node references against the loaded node count (negative sentinels
    /// pass through; e.g. <see cref="Resident.CurrentPOINode"/> is -1 while driving).
    /// Its <see cref="Resident.VehicleIndex"/> is range-guarded by
    /// <see cref="PopulationManager.AdoptLoadedPopulation"/> instead.</summary>
    private static Resident ReadResident(BinaryReader r, int nodeCount)
    {
        var res = new Resident
        {
            Id = r.ReadInt32(),
            HomeNode = r.ReadInt32(),
            WorkNode = r.ReadInt32(),
            Traits = new DriverTraits(
                (DriverArchetype)r.ReadByte(),
                r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle(),
                r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadByte()),
        };
        res.ColorR = r.ReadByte();
        res.ColorG = r.ReadByte();
        res.ColorB = r.ReadByte();

        int schedLen = ReadCount(r, "schedule");
        var schedule = new List<ScheduleEntry>(PreallocCapacity(schedLen));
        for (int j = 0; j < schedLen; j++)
            schedule.Add(new ScheduleEntry
            {
                DepartureTime = r.ReadSingle(),
                Destination = (POIType)r.ReadByte(),
            });
        res.Schedule = schedule.ToArray();

        res.ScheduleIndex = r.ReadInt32();
        res.Activity = (ResidentActivity)r.ReadByte();
        res.CurrentPOINode = r.ReadInt32();
        res.VehicleIndex = r.ReadInt32();

        if (res.HomeNode >= nodeCount || res.WorkNode >= nodeCount || res.CurrentPOINode >= nodeCount)
            throw new InvalidDataException($"Corrupt save: resident {res.Id} references a node out of range.");
        return res;
    }
}
