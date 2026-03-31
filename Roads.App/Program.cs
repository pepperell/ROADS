namespace Roads.App;

/// <summary>
/// Application entry point. Initializes WinForms and launches <see cref="MainForm"/>.
/// </summary>
static class Program
{
    /// <summary>Application entry point.</summary>
    [STAThread]
    static void Main()
    {
        ApplicationConfiguration.Initialize();
        Application.Run(new MainForm());
    }
}
