using System.Runtime.CompilerServices;

namespace Roads.App.Vehicles;

/// <summary>
/// Interpolated-string handler for <see cref="SteeringController.LogDiag"/> call sites.
/// The compiler lowers a <c>$"..."</c> argument into Append calls gated on the
/// constructor's <c>shouldAppend</c>, so when the gate fails (logging off, or the vehicle
/// is not the tracked DiagVehicle) NO formatting runs, NO string is allocated, and the
/// interpolation's argument expressions are not even evaluated. Call sites keep their
/// natural <c>$"..."</c> shape at zero disabled-path cost — this is what makes
/// per-vehicle-per-tick diag calls free at the 10K@30FPS target with logging off.
/// Plain-string calls bind to the string overloads instead; only interpolated arguments
/// route through here.
/// </summary>
[InterpolatedStringHandler]
internal ref struct DiagLogHandler
{
    private DefaultInterpolatedStringHandler _inner;

    /// <summary>True when the gate passed; appends and <see cref="ToStringAndClear"/> are
    /// only valid then (the receiving method must return early otherwise).</summary>
    internal readonly bool Enabled;

    public DiagLogHandler(int literalLength, int formattedCount,
        VehicleStore store, int index, out bool shouldAppend)
    {
        Enabled = shouldAppend =
            SteeringController.DebugLoggingEnabled && store.DiagVehicle == index;
        _inner = shouldAppend
            ? new DefaultInterpolatedStringHandler(literalLength, formattedCount)
            : default;
    }

    public void AppendLiteral(string value) => _inner.AppendLiteral(value);
    public void AppendFormatted<T>(T value) => _inner.AppendFormatted(value);
    public void AppendFormatted<T>(T value, string? format) => _inner.AppendFormatted(value, format);

    internal string ToStringAndClear() => _inner.ToStringAndClear();
}

/// <summary>
/// Same short-circuit pattern as <see cref="DiagLogHandler"/> but gated on
/// <see cref="SteeringController.DebugLoggingEnabled"/> alone — for
/// <c>LogSkip</c>/<c>LogArcConflict</c> call sites, whose per-event interpolations
/// otherwise still format on every blocked-gate tick with logging off.
/// </summary>
[InterpolatedStringHandler]
internal ref struct DebugLogHandler
{
    private DefaultInterpolatedStringHandler _inner;

    /// <summary>True when the gate passed; see <see cref="DiagLogHandler.Enabled"/>.</summary>
    internal readonly bool Enabled;

    public DebugLogHandler(int literalLength, int formattedCount, out bool shouldAppend)
    {
        Enabled = shouldAppend = SteeringController.DebugLoggingEnabled;
        _inner = shouldAppend
            ? new DefaultInterpolatedStringHandler(literalLength, formattedCount)
            : default;
    }

    public void AppendLiteral(string value) => _inner.AppendLiteral(value);
    public void AppendFormatted<T>(T value) => _inner.AppendFormatted(value);
    public void AppendFormatted<T>(T value, string? format) => _inner.AppendFormatted(value, format);

    internal string ToStringAndClear() => _inner.ToStringAndClear();
}
