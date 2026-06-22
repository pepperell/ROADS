using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Activity state of a resident in the town.
/// </summary>
public enum ResidentActivity : byte
{
    /// <summary>At a POI (home, work, etc.) — not in VehicleStore, zero sim cost.</summary>
    Dormant,
    /// <summary>Actively driving a scheduled trip — occupies a slot in VehicleStore.</summary>
    Driving,
    /// <summary>Created but not yet on the map: waiting to drive in from a region spawn the
    /// first time. Not in VehicleStore, occupies no POI. Skipped by departures; picked up by
    /// the move-in pass once a region spawn + region exit pair exists.</summary>
    OffMap,
    /// <summary>Driving the one-time region-spawn → home move-in trip — occupies a VehicleStore
    /// slot like <see cref="Driving"/>, but on arrival becomes dormant at home and resumes
    /// (rather than consuming) its schedule.</summary>
    MovingIn,
}

/// <summary>
/// A single trip in a resident's daily schedule.
/// </summary>
public struct ScheduleEntry
{
    /// <summary>Departure time in fractional hours (0–24). E.g., 7.5 = 7:30 AM.</summary>
    public float DepartureTime;
    /// <summary>Type of destination for this trip.</summary>
    public POIType Destination;
    /// <summary>
    /// When true, this trip targets the NEAREST available POI of its type (a quick local errand,
    /// e.g. a midday shop run from work); when false the destination is picked at random each time.
    /// In-memory only — not serialized (schedules regenerate daily), so it defaults to random on load.
    /// </summary>
    public bool NearestPOI;
}

/// <summary>
/// A persistent identity representing a person who lives in the simulated town.
/// Residents exist independently of VehicleStore: when driving they occupy a vehicle
/// slot, when at a POI they are dormant (invisible, zero simulation cost).
/// </summary>
public class Resident
{
    /// <summary>Unique identifier (index in PopulationManager's resident list).</summary>
    public int Id;

    /// <summary>Node index of this resident's home POI.</summary>
    public int HomeNode;
    /// <summary>Node index of this resident's workplace POI, or -1 if unemployed.</summary>
    public int WorkNode;

    /// <summary>Personality traits, copied into VehicleStore when spawning onto the road.</summary>
    public DriverTraits Traits;
    /// <summary>Car color, copied into VehicleStore when spawning.</summary>
    public byte ColorR, ColorG, ColorB;

    /// <summary>Today's schedule: ordered list of trips.</summary>
    public ScheduleEntry[] Schedule = Array.Empty<ScheduleEntry>();
    /// <summary>Index of the next schedule entry to execute.</summary>
    public int ScheduleIndex;

    /// <summary>Current activity: Dormant (at POI) or Driving (in VehicleStore).</summary>
    public ResidentActivity Activity;
    /// <summary>Index in VehicleStore when driving, or -1 when dormant.</summary>
    public int VehicleIndex = -1;
    /// <summary>Node index of the POI the resident is currently at, or -1 if driving.</summary>
    public int CurrentPOINode;

    /// <summary>
    /// A resident whose home was deleted: it always heads to a region exit and is removed from
    /// the population on arrival there (it never goes dormant). Set by the graceful-deletion drain
    /// (<see cref="PopulationManager"/>) when a home node is being removed. Owns the resident's
    /// lifecycle for the rest of its trip — driving emigrants are retargeted to an exit, dormant
    /// emigrants are spawned toward one, and arrivals remove them rather than re-parking them.
    /// </summary>
    public bool Emigrating;
}
