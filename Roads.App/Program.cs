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
    /// </summary>
    [STAThread]
    static void Main()
    {
        ApplicationConfiguration.Initialize();

        int autoBenchFrames = 0;
        foreach (var arg in Environment.GetCommandLineArgs())
        {
            if (arg == "--autobench")
                autoBenchFrames = 100;
            else if (arg.StartsWith("--autobench=", StringComparison.Ordinal)
                     && int.TryParse(arg.AsSpan("--autobench=".Length), out int n) && n > 0)
                autoBenchFrames = n;
        }

        Application.Run(new MainForm(autoBenchFrames));
    }
}
