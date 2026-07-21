namespace Roads.App.Vehicles;

/// <summary>
/// Per-pass index mapping each intersection-arc index to the vehicle indices currently
/// occupying its junction: vehicles traversing the arc (<c>CurrentArc[v] == arc</c>) AND
/// vehicles whose steering has moved on but whose body has not yet physically cleared the
/// junction (<c>ClearingArc[v] == arc</c> — long vehicles on short arcs). Replaces O(n)
/// full-vehicle scans for arc-conflict checks with O(occupants-of-relevant-arcs) lookups.
///
/// Reused across ticks — no per-tick allocation. Only buckets that received entries since the
/// last <see cref="Rebuild"/>/<see cref="Clear"/> are cleared (tracked in <c>_touched</c>).
///
/// Maintenance contract:
/// <list type="bullet">
/// <item>In the steering phase, <c>CurrentArc</c> is mutated as vehicles enter/exit arcs mid-pass,
/// and later vehicles must observe those changes. The index must therefore stay live: every
/// <c>CurrentArc</c> write is paired with <see cref="Enter"/>/<see cref="Exit"/>. SteeringController
/// routes all three of its writes through a single <c>SetArc</c> helper, so the invariant is a
/// single choke-point.</item>
/// <item>In the lane-change phase, <c>CurrentArc</c> is stable (only steering writes it), so a single
/// <see cref="Rebuild"/> snapshot at the start of each public entry point is sufficient.</item>
/// <item>Outside both passes, <c>CurrentArc</c> is also written directly by
/// <c>GraphChangeHandler</c> (defunct-edge reseat) and <c>SimulationLoop.RemapVehicleArcs</c>
/// (arc-index remap after an arc-cache rebuild). Those writes bypass Enter/Exit safely
/// because every pass begins with a fresh <see cref="Rebuild"/> snapshot.</item>
/// </list>
/// Only Driving vehicles with a non-negative arc are indexed, matching the original scans' guards.
/// </summary>
public sealed class ArcOccupancyIndex
{
    private List<int>[] _buckets = Array.Empty<List<int>>();
    /// <summary>Indices of buckets that currently hold entries, so clearing touches only those.</summary>
    private readonly List<int> _touched = new();

    private void EnsureCapacity(int arcCount)
    {
        if (_buckets.Length >= arcCount) return;
        int old = _buckets.Length;
        Array.Resize(ref _buckets, arcCount);
        for (int i = old; i < arcCount; i++)
            _buckets[i] = new List<int>();
    }

    /// <summary>Empties every bucket that received entries since the last clear.</summary>
    private void ClearTouched()
    {
        for (int t = 0; t < _touched.Count; t++)
        {
            int arc = _touched[t];
            if ((uint)arc < (uint)_buckets.Length)
                _buckets[arc].Clear();
        }
        _touched.Clear();
    }

    private void AddInternal(int arc, int vehicle)
    {
        var bucket = _buckets[arc];
        if (bucket.Count == 0)
            _touched.Add(arc);
        bucket.Add(vehicle);
    }

    /// <summary>
    /// Clears prior contents and buckets every Driving vehicle whose <c>CurrentArc &gt;= 0</c>
    /// or <c>ClearingArc &gt;= 0</c> (both can be set at once only when the vehicle entered a
    /// new arc while its rear still cleared the previous junction — it then occupies both).
    /// <paramref name="arcCount"/> (from <c>IntersectionArcCache.ArcCount</c>) sizes the bucket array.
    /// </summary>
    public void Rebuild(VehicleStore store, int arcCount)
    {
        ClearTouched();
        EnsureCapacity(arcCount);
        int count = store.Count;
        for (int i = 0; i < count; i++)
        {
            if (store.State[i] != VehicleState.Driving) continue;
            int arc = store.CurrentArc[i];
            if ((uint)arc < (uint)_buckets.Length)
                AddInternal(arc, i);
            int clearing = store.ClearingArc[i];
            if ((uint)clearing < (uint)_buckets.Length && clearing != arc)
                AddInternal(clearing, i);
        }
    }

    /// <summary>Records that <paramref name="vehicle"/> entered <paramref name="arc"/>. Pair with every arc-entry write.</summary>
    public void Enter(int arc, int vehicle)
    {
        if ((uint)arc < (uint)_buckets.Length)
            AddInternal(arc, vehicle);
    }

    /// <summary>
    /// Records that <paramref name="vehicle"/> left <paramref name="arc"/>. Pair with every arc-exit write.
    /// Swap-remove; buckets hold only the handful of vehicles concurrently on one arc.
    /// </summary>
    public void Exit(int arc, int vehicle)
    {
        if ((uint)arc >= (uint)_buckets.Length) return;
        var bucket = _buckets[arc];
        for (int k = 0; k < bucket.Count; k++)
        {
            if (bucket[k] == vehicle)
            {
                bucket[k] = bucket[bucket.Count - 1];
                bucket.RemoveAt(bucket.Count - 1);
                return;
            }
        }
    }

    /// <summary>Number of vehicles currently bucketed on <paramref name="arc"/> (0 if none/out of range).</summary>
    public int OccupantCount(int arc)
        => (uint)arc < (uint)_buckets.Length ? _buckets[arc].Count : 0;

    /// <summary>The <paramref name="k"/>th occupant of <paramref name="arc"/>. Caller bounds-checks via <see cref="OccupantCount"/>.</summary>
    public int OccupantAt(int arc, int k) => _buckets[arc][k];

    /// <summary>Drops all occupants (e.g. on a full vehicle clear).</summary>
    public void Clear() => ClearTouched();
}
