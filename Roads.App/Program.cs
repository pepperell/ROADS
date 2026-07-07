namespace Roads.App;

/// <summary>
/// Application entry point. Initializes WinForms and launches <see cref="MainForm"/>.
/// </summary>
static class Program
{
    /// <summary>
    /// Application entry point. Pass <c>--autobench</c> (or <c>--autobench=N</c>) to run an
    /// automated 10K-vehicle benchmark: the app builds the stress scene, runs N frames
    /// (default 100), appends per-frame metrics to <c>benchmark.log</c> over the final frames,
    /// then exits. Used for headless optimization iteration.
    ///
    /// Pass <c>--simtest=&lt;map&gt;</c> to run the headless <see cref="Diagnostics.SimTestHarness"/>
    /// instead of the GUI: <c>--simhours=&lt;h&gt;</c> sim duration (default 1),
    /// <c>--simout=&lt;path&gt;</c> report path (default simtest_report.log),
    /// <c>--simseed=&lt;n&gt;</c> RNG seed, <c>--simvehicles</c> to load the map's saved
    /// vehicles (replay a captured live jam), and <c>--diagvehicle=&lt;n&gt;</c> to stream
    /// that vehicle's per-tick diagnostics to diag.log. Exit code 0 = no jams found.
    /// </summary>
    [STAThread]
    static void Main()
    {
        ApplicationConfiguration.Initialize();

        int autoBenchFrames = 0;
        string? simTestMap = null;
        float simHours = 1f;
        string simOut = "simtest_report.log";
        int simSeed = 12345;
        bool simVehicles = false;
        int diagVehicle = -1;
        foreach (var arg in Environment.GetCommandLineArgs())
        {
            if (arg == "--autobench")
                autoBenchFrames = 100;
            else if (arg.StartsWith("--autobench=", StringComparison.Ordinal)
                     && int.TryParse(arg.AsSpan("--autobench=".Length), out int n) && n > 0)
                autoBenchFrames = n;
            else if (arg.StartsWith("--simtest=", StringComparison.Ordinal))
                simTestMap = arg["--simtest=".Length..];
            else if (arg.StartsWith("--simhours=", StringComparison.Ordinal)
                     && float.TryParse(arg.AsSpan("--simhours=".Length), out float h) && h > 0f)
                simHours = h;
            else if (arg.StartsWith("--simout=", StringComparison.Ordinal))
                simOut = arg["--simout=".Length..];
            else if (arg.StartsWith("--simseed=", StringComparison.Ordinal)
                     && int.TryParse(arg.AsSpan("--simseed=".Length), out int s))
                simSeed = s;
            else if (arg == "--simvehicles")
                simVehicles = true;
            else if (arg.StartsWith("--diagvehicle=", StringComparison.Ordinal)
                     && int.TryParse(arg.AsSpan("--diagvehicle=".Length), out int dv))
                diagVehicle = dv;
        }

        if (simTestMap != null)
        {
            int code = Diagnostics.SimTestHarness.Run(simTestMap, simHours, simOut, simSeed,
                simVehicles, diagVehicle);
            Environment.Exit(code);
        }

        Application.Run(new MainForm(autoBenchFrames));
    }
}
