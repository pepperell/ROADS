using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Activity state of a resident in the town.
/// </summary>
public enum ResidentActivity : byte
{
    /// <summary>At a POI (home, work, etc.) — not in VehicleStore, zero sim cost.</summary>
    Dormant,
    /// <summary>Actively driving — occupies a slot in VehicleStore.</summary>
    Driving,
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
}
