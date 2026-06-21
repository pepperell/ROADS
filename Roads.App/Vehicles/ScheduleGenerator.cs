using Roads.App.World;

namespace Roads.App.Vehicles;

/// <summary>
/// Generates daily schedules for residents based on their driver archetype.
/// Each archetype has characteristic departure times, destinations, and
/// probability of midday errands, producing natural traffic patterns.
/// </summary>
public static class ScheduleGenerator
{
    /// <summary>
    /// Generates a weekday schedule for a resident based on their archetype.
    /// Schedules are sorted by departure time.
    /// </summary>
    public static ScheduleEntry[] GenerateWeekday(DriverArchetype archetype, int workNode)
    {
        var entries = new List<ScheduleEntry>(4);
        bool hasWork = workNode >= 0;

        switch (archetype)
        {
            case DriverArchetype.Commuter:
                if (hasWork)
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(8.0f, 0.5f), Destination = POIType.Work });
                    // 20% chance of midday shop errand — a quick local run, so NEAREST shop.
                    if (Random.Shared.NextDouble() < 0.20)
                    {
                        entries.Add(new ScheduleEntry { DepartureTime = Jitter(12.0f, 0.3f), Destination = POIType.Shop, NearestPOI = true });
                        entries.Add(new ScheduleEntry { DepartureTime = Jitter(12.5f, 0.2f), Destination = POIType.Work });
                    }
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(18.0f, 0.5f), Destination = POIType.Home });
                }
                else
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(10.0f, 1.0f), Destination = POIType.Shop });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(14.0f, 1.0f), Destination = POIType.Home });
                }
                break;

            case DriverArchetype.LeadFoot:
                if (hasWork)
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(8.5f, 0.5f), Destination = POIType.Work });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(17.5f, 0.5f), Destination = POIType.Home });
                    // 30% chance of evening leisure
                    if (Random.Shared.NextDouble() < 0.30)
                    {
                        entries[^1] = new ScheduleEntry { DepartureTime = entries[^1].DepartureTime, Destination = POIType.Leisure };
                        entries.Add(new ScheduleEntry { DepartureTime = Jitter(21.0f, 0.5f), Destination = POIType.Home });
                    }
                }
                else
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(9.0f, 1.0f), Destination = POIType.Leisure });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(15.0f, 1.0f), Destination = POIType.Home });
                }
                break;

            case DriverArchetype.Trucker:
                if (hasWork)
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(6.0f, 0.5f), Destination = POIType.Work });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(19.0f, 0.5f), Destination = POIType.Home });
                }
                else
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(7.0f, 1.0f), Destination = POIType.Shop });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(16.0f, 1.0f), Destination = POIType.Home });
                }
                break;

            case DriverArchetype.SundayDriver:
                entries.Add(new ScheduleEntry { DepartureTime = Jitter(10.0f, 0.5f), Destination = POIType.Leisure });
                // 40% chance of shop trip
                if (Random.Shared.NextDouble() < 0.40)
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(13.0f, 0.5f), Destination = POIType.Shop });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(15.0f, 0.5f), Destination = POIType.Home });
                }
                else
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(16.0f, 0.5f), Destination = POIType.Home });
                }
                break;

            case DriverArchetype.NervousNellie:
                if (hasWork)
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(7.5f, 0.25f), Destination = POIType.Work });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(16.5f, 0.25f), Destination = POIType.Home });
                }
                else
                {
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(9.0f, 0.5f), Destination = POIType.Shop });
                    entries.Add(new ScheduleEntry { DepartureTime = Jitter(11.0f, 0.5f), Destination = POIType.Home });
                }
                break;
        }

        // Sort by departure time and clamp to 0-24 range
        entries.Sort((a, b) => a.DepartureTime.CompareTo(b.DepartureTime));
        for (int i = 0; i < entries.Count; i++)
        {
            var e = entries[i];
            e.DepartureTime = Math.Clamp(e.DepartureTime, 0f, 23.99f);
            entries[i] = e;
        }

        return entries.ToArray();
    }

    /// <summary>
    /// Applies gaussian jitter to a base time in hours.
    /// </summary>
    private static float Jitter(float baseTime, float stddevHours)
    {
        float u1 = 1f - (float)Random.Shared.NextDouble();
        float u2 = (float)Random.Shared.NextDouble();
        float z = MathF.Sqrt(-2f * MathF.Log(u1)) * MathF.Cos(2f * MathF.PI * u2);
        return baseTime + stddevHours * z;
    }
}
