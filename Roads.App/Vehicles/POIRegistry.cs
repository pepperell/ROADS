using System.Numerics;
using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Caches POI nodes by type for fast lookup and tracks per-node occupancy.
/// Rebuilt automatically when the road graph changes.
/// </summary>
public class POIRegistry
{
    private readonly Dictionary<POIType, List<int>> _nodesByType = new();
    private readonly Dictionary<int, int> _occupancy = new();
    private int _graphVersion = -1;

    /// <summary>Default capacity per POI type.</summary>
    private static int DefaultCapacity(POIType type) => type switch
    {
        POIType.Home => 2,
        POIType.Work => 20,
        POIType.Shop => 10,
        POIType.Leisure => 15,
        POIType.School => 30,
        POIType.Parking => 30,
        POIType.RegionExit => int.MaxValue, // a map boundary is never "full"
        _ => 1,
    };

    /// <summary>
    /// Rebuilds the POI cache if the graph has changed. Preserves occupancy counts
    /// for nodes that still exist; resets counts for new nodes.
    /// </summary>
    public void RebuildIfNeeded(RoadGraph graph)
    {
        if (_graphVersion == graph.Version) return;
        _graphVersion = graph.Version;

        _nodesByType.Clear();
        var survivingNodes = new HashSet<int>();

        for (int i = 0; i < graph.Nodes.Count; i++)
        {
            var node = graph.Nodes[i];
            if (float.IsNaN(node.Position.X)) continue;
            if (!node.Flags.HasFlag(NodeFlags.Destination)) continue;
            if (node.PointOfInterest == POIType.None) continue;

            if (!_nodesByType.TryGetValue(node.PointOfInterest, out var list))
            {
                list = new List<int>();
                _nodesByType[node.PointOfInterest] = list;
            }
            list.Add(i);
            survivingNodes.Add(i);

            if (!_occupancy.ContainsKey(i))
                _occupancy[i] = 0;
        }

        // Remove occupancy entries for defunct nodes
        var toRemove = new List<int>();
        foreach (var key in _occupancy.Keys)
        {
            if (!survivingNodes.Contains(key))
                toRemove.Add(key);
        }
        foreach (var key in toRemove)
            _occupancy.Remove(key);
    }

    /// <summary>Returns all node indices of the given POI type.</summary>
    public IReadOnlyList<int> GetNodesOfType(POIType type)
    {
        return _nodesByType.TryGetValue(type, out var list) ? list : Array.Empty<int>();
    }

    /// <summary>Returns the total capacity across all nodes of the given POI type.</summary>
    public int GetTotalCapacity(POIType type)
    {
        var nodes = GetNodesOfType(type);
        int total = 0;
        for (int i = 0; i < nodes.Count; i++)
            total += GetCapacity(nodes[i], type);
        return total;
    }

    /// <summary>Returns the capacity for a specific node.</summary>
    public int GetCapacity(int nodeIndex, POIType type) => DefaultCapacity(type);

    /// <summary>Returns current occupancy at a node.</summary>
    public int GetOccupancy(int nodeIndex) =>
        _occupancy.TryGetValue(nodeIndex, out int count) ? count : 0;

    /// <summary>
    /// Attempts to occupy a slot at the given node. Returns false if at capacity.
    /// </summary>
    public bool TryOccupy(int nodeIndex, POIType type)
    {
        int current = GetOccupancy(nodeIndex);
        if (current >= GetCapacity(nodeIndex, type))
            return false;
        _occupancy[nodeIndex] = current + 1;
        return true;
    }

    /// <summary>Releases an occupancy slot at the given node.</summary>
    public void Vacate(int nodeIndex)
    {
        if (_occupancy.TryGetValue(nodeIndex, out int count) && count > 0)
            _occupancy[nodeIndex] = count - 1;
    }

    /// <summary>
    /// Finds the nearest available (under capacity) node of the given type
    /// to the specified world position. Returns -1 if none available.
    /// </summary>
    public int FindNearestAvailable(RoadGraph graph, POIType type, Vector2 fromPos)
    {
        var nodes = GetNodesOfType(type);
        int best = -1;
        float bestDistSq = float.MaxValue;

        for (int i = 0; i < nodes.Count; i++)
        {
            int nodeIdx = nodes[i];
            if (GetOccupancy(nodeIdx) >= GetCapacity(nodeIdx, type))
                continue;
            var pos = graph.Nodes[nodeIdx].Position;
            float distSq = Vector2.DistanceSquared(pos, fromPos);
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                best = nodeIdx;
            }
        }
        return best;
    }

    /// <summary>
    /// Picks a uniformly-random available (under-capacity) node of the given type, or -1 if none
    /// are available. Reservoir-samples in a single pass (each available node has equal probability)
    /// so callers get a fresh random destination every trip without allocating.
    /// </summary>
    public int FindRandomAvailable(POIType type)
    {
        var nodes = GetNodesOfType(type);
        int chosen = -1;
        int seen = 0;
        for (int i = 0; i < nodes.Count; i++)
        {
            int nodeIdx = nodes[i];
            if (GetOccupancy(nodeIdx) >= GetCapacity(nodeIdx, type)) continue;
            seen++;
            if (Random.Shared.Next(seen) == 0) chosen = nodeIdx; // 1/seen chance to replace
        }
        return chosen;
    }

    /// <summary>Resets all occupancy counts to zero.</summary>
    public void ClearOccupancy()
    {
        foreach (var key in _occupancy.Keys.ToArray())
            _occupancy[key] = 0;
    }
}
