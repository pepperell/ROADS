using SkiaSharp;

namespace Roads.App.Rendering.Ui;

/// <summary>
/// Top-right time-and-speed panel: a 12-hour analog dial (hour + minute hands, tick
/// marks) with AM/PM illumination, the digital game time beneath it, the current
/// simulation speed (or PAUSED), and a transport-button row (&lt;&lt; / Pause / &gt;&gt;)
/// that raises <see cref="UIAction"/>s through the owner callback — the same actions the
/// retired menu-bar buttons fired. Purely a view over <see cref="SimulationLoop"/> state —
/// hands and labels read <see cref="Core.SimulationClock.TimeOfDay"/> live each frame, so
/// they freeze while paused and sweep visibly at high time scales. Always visible (no
/// toggle); consumes clicks like every opaque panel. Drawing is allocation-free: the
/// digital string is cached and rebuilt only when the displayed minute changes, and speed
/// labels come from a fixed table indexed by the time-scale exponent.
/// </summary>
public class ClockPanel : Panel
{
    /// <summary>Panel height, public so the slider panel can anchor beneath it.</summary>
    public const float PanelHeight = 164f;
    public const float PanelWidth = 120f;

    private const float DialRadius = 44f;
    private const float Pad = 8f;
    private const float ButtonRowY = 134f;
    private const float ButtonHeight = 22f;
    private const float SpeedButtonWidth = 26f;
    private const float PauseButtonWidth = 44f;
    private const float ButtonSpacing = 4f;

    private readonly SimulationLoop _simLoop;

    // Cached digital readout, rebuilt only when the displayed minute changes.
    private string _timeText = "";
    private int _cachedMinuteOfDay = -1;

    /// <summary>Speed label per time-scale exponent (1x..64x); index clamped defensively.</summary>
    private static readonly string[] SpeedLabels = { "1x", "2x", "4x", "8x", "16x", "32x", "64x" };

    private readonly SKPaint _facePaint = new() { Color = new SKColor(35, 38, 44), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _ringPaint = new() { Color = UiTheme.Outline, Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true };
    private readonly SKPaint _tickPaint = new() { Color = UiTheme.TextDim, Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true };
    private readonly SKPaint _handPaint = new() { Color = SKColors.White, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };
    private readonly SKPaint _hubPaint = new() { Color = SKColors.White, Style = SKPaintStyle.Fill, IsAntialias = true };

    public ClockPanel(SimulationLoop simLoop, Action<UIAction> onAction)
    {
        _simLoop = simLoop;
        Anchor = UiAnchor.TopRight;
        Margin = new SKPoint(10f, 10f);
        Size = new SKSize(PanelWidth, PanelHeight);
        BackgroundColor = UiTheme.PanelBackground;
        BorderColor = UiTheme.Outline;

        // Transport row: << | Pause/Play | >> (colors verbatim from the retired menu-bar
        // action buttons; the Pause button swaps label and amber/green with live state).
        var slower = new Button
        {
            Text = "<<",
            Font = UiTheme.Font11,
            TextOffset = new SKPoint(0f, 4f),
            Size = new SKSize(SpeedButtonWidth, ButtonHeight),
            Offset = new SKPoint(Pad, ButtonRowY),
            Idle = new ButtonColors(new SKColor(50, 75, 85), new SKColor(170, 200, 210)),
            Hover = new ButtonColors(new SKColor(65, 100, 115), SKColors.White),
            Disabled = new ButtonColors(new SKColor(40, 45, 50), new SKColor(90, 95, 100)),
            IsEnabled = () => simLoop.TimeScaleExponent > 0,
        };
        slower.Click += () => onAction(UIAction.SpeedDown);
        Add(slower);

        var pause = new Button
        {
            TextSource = () => simLoop.Paused ? "Play" : "Pause",
            Font = UiTheme.Font11,
            TextOffset = new SKPoint(0f, 4f),
            Size = new SKSize(PauseButtonWidth, ButtonHeight),
            Offset = new SKPoint(Pad + SpeedButtonWidth + ButtonSpacing, ButtonRowY),
            Idle = new ButtonColors(new SKColor(120, 80, 30), SKColors.White),
            Hover = new ButtonColors(new SKColor(150, 105, 40), SKColors.White),
            Active = new ButtonColors(new SKColor(40, 120, 50), SKColors.White),
            ActiveHover = new ButtonColors(new SKColor(55, 155, 65), SKColors.White),
            IsActive = () => simLoop.Paused,
        };
        pause.Click += () => onAction(UIAction.Pause);
        Add(pause);

        var faster = new Button
        {
            Text = ">>",
            Font = UiTheme.Font11,
            TextOffset = new SKPoint(0f, 4f),
            Size = new SKSize(SpeedButtonWidth, ButtonHeight),
            Offset = new SKPoint(Pad + SpeedButtonWidth + PauseButtonWidth + 2f * ButtonSpacing, ButtonRowY),
            Idle = new ButtonColors(new SKColor(50, 75, 85), new SKColor(170, 200, 210)),
            Hover = new ButtonColors(new SKColor(65, 100, 115), SKColors.White),
            Disabled = new ButtonColors(new SKColor(40, 45, 50), new SKColor(90, 95, 100)),
            IsEnabled = () => simLoop.TimeScaleExponent < 6,
        };
        faster.Click += () => onAction(UIAction.SpeedUp);
        Add(faster);
    }

    protected override void OnDraw(SKCanvas canvas)
    {
        double timeOfDay = _simLoop.Clock.TimeOfDay;
        float cx = Bounds.MidX;
        float cy = Bounds.Top + Pad + DialRadius;

        // ── Dial face + ticks ────────────────────────────────────────────
        canvas.DrawCircle(cx, cy, DialRadius, _facePaint);
        canvas.DrawCircle(cx, cy, DialRadius, _ringPaint);
        for (int h = 0; h < 12; h++)
        {
            bool major = h % 3 == 0;
            float angle = h / 12f * MathF.Tau - MathF.PI / 2f;
            float outer = DialRadius - 2f;
            float inner = outer - (major ? 6f : 3f);
            float dx = MathF.Cos(angle), dy = MathF.Sin(angle);
            _tickPaint.StrokeWidth = major ? 1.6f : 1f;
            canvas.DrawLine(cx + dx * inner, cy + dy * inner, cx + dx * outer, cy + dy * outer, _tickPaint);
        }

        // ── Hands (clockwise from 12 o'clock; Y-down screen space) ───────
        float hourFrac = (float)(timeOfDay % 12.0) / 12f;
        float minuteFrac = (float)(timeOfDay - Math.Floor(timeOfDay));
        DrawHand(canvas, cx, cy, hourFrac, DialRadius * 0.55f, 2.5f);
        DrawHand(canvas, cx, cy, minuteFrac, DialRadius * 0.85f, 1.5f);
        canvas.DrawCircle(cx, cy, 2.2f, _hubPaint);

        // ── AM/PM illumination (inside the lower half of the dial) ───────
        bool isAm = timeOfDay < 12.0;
        var lit = new SKColor(255, 214, 150);
        var unlit = new SKColor(70, 72, 78);
        UiTheme.TextScratch.Color = isAm ? lit : unlit;
        canvas.DrawText("AM", cx - 14f, cy + DialRadius * 0.55f, SKTextAlign.Center, UiTheme.Font11, UiTheme.TextScratch);
        UiTheme.TextScratch.Color = isAm ? unlit : lit;
        canvas.DrawText("PM", cx + 14f, cy + DialRadius * 0.55f, SKTextAlign.Center, UiTheme.Font11, UiTheme.TextScratch);

        // ── Digital time (cached per displayed minute) ───────────────────
        int minuteOfDay = (int)(timeOfDay * 60.0);
        if (minuteOfDay != _cachedMinuteOfDay)
        {
            _cachedMinuteOfDay = minuteOfDay;
            _timeText = _simLoop.Clock.GetDisplayTime();
        }
        float textY = Bounds.Top + Pad + DialRadius * 2f + 16f;
        UiTheme.TextScratch.Color = UiTheme.TextPrimary;
        canvas.DrawText(_timeText, cx, textY, SKTextAlign.Center, UiTheme.Font13, UiTheme.TextScratch);

        // ── Speed / paused line ──────────────────────────────────────────
        textY += 16f;
        if (_simLoop.Paused)
        {
            UiTheme.TextScratch.Color = new SKColor(220, 160, 50);
            canvas.DrawText("PAUSED", cx, textY, SKTextAlign.Center, UiTheme.Font12, UiTheme.TextScratch);
        }
        else
        {
            int exp = Math.Clamp(_simLoop.TimeScaleExponent, 0, SpeedLabels.Length - 1);
            UiTheme.TextScratch.Color = UiTheme.Value;
            canvas.DrawText(SpeedLabels[exp], cx, textY, SKTextAlign.Center, UiTheme.Font12, UiTheme.TextScratch);
        }
    }

    /// <summary>Draws a hand at the given fraction of a full clockwise turn from 12 o'clock.</summary>
    private void DrawHand(SKCanvas canvas, float cx, float cy, float turnFraction, float length, float width)
    {
        float angle = turnFraction * MathF.Tau - MathF.PI / 2f;
        _handPaint.StrokeWidth = width;
        canvas.DrawLine(cx, cy, cx + MathF.Cos(angle) * length, cy + MathF.Sin(angle) * length, _handPaint);
    }
}
