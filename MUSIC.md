# ROADS Generative Music — Design & Reference

The background music is composed live, bar by bar, while the game runs — nothing is
pre-recorded or sequenced from files. The style target is the mid-90s city-builder
sound: SimCity 2000's smooth-jazz/new-age float merged with Transport Tycoon's
jump-blues drive, rendered through the same General MIDI palette those games were
authored for. This document describes the whole system: architecture, music theory,
every generator, and the game-state coupling.

Source lives in [Roads.App/Audio/Music/](Roads.App/Audio/Music/):
[Theory.cs](Roads.App/Audio/Music/Theory.cs) (harmony data),
[Composer.cs](Roads.App/Audio/Music/Composer.cs) (the symbolic generator),
[MusicProvider.cs](Roads.App/Audio/Music/MusicProvider.cs) (sequencer + synth bridge),
plus the mood mapping in [AudioEngine.cs](Roads.App/Audio/AudioEngine.cs).

---

## 1. Signal path & threading

```
UI thread (once per frame)                     NAudio playback thread
──────────────────────────                     ─────────────────────────────────────
AudioEngine.UpdateMusicMood                    MusicProvider.Read
  sim state → atomic float/int targets   ──▶     assemble MoodInputs snapshot
  (intensity, night, tension, hour,              Composer.GenerateBar (1 bar ahead)
   ambience, day, ResolutionSeq++)                 → MidiEvent queue (sample-stamped)
                                                 MeltySynth renders between events
                                                 → stereo float bus
                                               MasterProvider: SFX×pauseDuck + music
                                                 → master gain → tanh soft clip
```

- **The composer runs entirely on the playback thread.** The UI writes plain 32-bit
  fields (atomic by themselves); `ComposeBar` assembles them into a `MoodInputs`
  snapshot on the reading side, so no locks and no torn reads exist anywhere.
- **Events are sample-timestamped** and sorted with a tie rank (program/CC → note-off
  → note-on) so same-sample ties can never chop a repeated note or voice a note on a
  stale patch. Note-offs may overhang a bar; composition triggers on the playhead
  reaching the end of composed material, never on queue emptiness.
- **The music bus bypasses the pause duck** — the band plays while the sim is paused,
  classic city-builder behavior. Master volume and the soft clip still apply.
- **Muted = frozen.** When faded out the timeline stops advancing (zero cost); pending
  one-shot triggers are discarded so nothing fires inexplicably on unmute.
- **Determinism invariant:** nothing in the music path touches `SimRandom` or sim
  state. The composer has a private RNG seeded per launch; `--simtest` behaves
  identically with or without music.
- The synth is **MeltySynth** playing the bundled **GeneralUser GS** soundfont
  (`Roads.App/Assets/GeneralUser-GS.sf2`); rendering runs ~30–70× real time, so the
  audio callback cost is negligible. A missing soundfont simply disables music.

## 2. The setlist

The band plays discrete **tunes**, not an endless jam. A tune bundles:

| Property | Chosen from |
|---|---|
| Form / chart | Blues, minor blues, AABA, bossa, funk vamp, nocturne (see §3) |
| Key center | B♭, E♭, F, C, A♭ — never the previous tune's key |
| Tempo | live tempo setting + per-tune offset (−10…+12 BPM; nocturnes −8), clamped 66–140 |
| Lead palette | 2–3 instruments (see §5) rotated between choruses |
| Comping patch | Rhodes EP / jazz guitar / drawbar organ (form-weighted) |
| Drum kit | standard, or brushes for nocturnes (+20% of bossas) when the soundfont has bank 128 patch 40 |
| Reverb room | CC91 base 30–70 (nocturnes 55–80), fanned out per channel |
| Length budget | 4–6 minutes, rounded to whole chart cycles |

**Lifecycle:** `Playing → Ending → Break → CountIn → Playing…`

- **Ending** (2 bars): a ii–V–I tag in the tune's key (I is a 13th chord for
  blues/vamp tunes, maj9 otherwise) under a ritardando (~6% slowdown, applied via a
  dedicated factor so the live tempo slider can't cancel it).
- **Break** (1–2 bars): silence; reverb tails and the pad's overhang ring into it.
- **CountIn** (1 bar): pedal-hat clicks on each beat at the *new* tune's tempo
  (hard-set — no slew across the break). Nocturne tunes skip the count-in and fade
  in on pads instead.

Rotation is **armed** (the tune finishes its current chart cycle first) by: budget
expiry, in-game day rollover, night rising above 0.6 during a day tune, or night
falling below 0.3 during a nocturne.

**Tune selection** (`PickTune`) is the single place hour/night weighting happens:

| Form | Base weight | Hour bias |
|---|---|---|
| Blues | 1.0 | ×1.5 at 06–10 (morning shuffle) |
| Minor blues | 0.8 | ×1.3 at 15–20 |
| AABA | 1.0 | — |
| Bossa | 0.7 | ×1.8 at 10–15 (midday) |
| Funk vamp | 0.6 | ×1.6 at 15–20 (evening) |
| Nocturne | 6.0 if night > 0.5, else 0.05 | — |

The previous tune's form is weighted ×0.15 so the set never repeats back-to-back.

## 3. Forms & charts

Charts are stored in a home key and transposed to the tune's key center at a single
choke point (`TransposedBar`); pitch classes are C = 0. Each chart carries a swing
factor (multiplied with the user swing setting) and a beats-per-bar.

**12-bar jazz-blues** (home B♭, swing ×1.0) — the Transport Tycoon inheritance with
SimCity chord stacks:

```
| Bb13 | Eb13 | Bb13 | Fm9 Bb13 | Eb13 | Edim7 | Bb13 | Dm7b5 G7alt |
| Cm9  | F13  | Bb13 G7alt | Cm9 F13 |
```

**12-bar minor blues** (home Cm, swing ×1.0):

```
| Cm9 | Fm9 | Cm9 | Cm9 | Fm9 | Fm9 | Cm9 | Cm9 | Ab13 | G7alt | Cm9 | G7alt |
```

**32-bar AABA** (home E♭, swing ×1.0) — A section with a dominant-cycle bridge:

```
A: | Ebmaj9 | Cm9 | Fm9 | Bb13 | Ebmaj9 | Ab13 | Ebmaj9 Cm9 | Fm9 Bb13 |
B: | G13 | G13 | C13 | C13 | F13 | F13 | Bb13 | Bb13 |        form: A A B A
```

**16-bar bossa** (home F, swing ×0 — straight 8ths):

```
| Fmaj9 | Fmaj9 | G13 | G13 | Gm9 | C13 | Fmaj9 | C7sus |
| Gm9 | C13 | Am9 | D7alt | Gm9 | C13 | Fmaj9 | Fmaj9 |
```

**8-bar funk vamp** (home B♭ — Cm9↔F13 is its ii–V, swing ×0.45):

```
| Cm9 | F13 | Cm9 | F13 | Cm9 | F13 | Cm9 | F7alt |
```

Doubles as the interlude chart inside blues-family tunes and as the "tension" form.

**Nocturnes** (swing ×0.3 / ×0.25) — sus-plateau floats, one chord per bar:

```
#1 (Bb): | Bbmaj9 | Gm11 | Ebmaj7#11 | F7sus | Bbmaj9 | Abmaj7#11 | Gm11 | F7sus |
#2 (Eb): | Ebmaj9 | Cm11 | Abmaj7#11 | Bb7sus | Gm11 | Cm11 | Abmaj7#11 | Bb7sus |
```

Nocturne #2 also exists as a **3/4 jazz waltz** variant (35% of nocturne tunes).
Waltz charts must be single-chord bars (asserted in debug — the half-bar chord split
assumes 4/4).

**Cadence tag** (2 bars, built in the tune's key): `| iim9 V13 | I |` — shared by
tune endings and the jam-cleared resolution.

## 4. Harmony toolkit

Nine chord qualities, each mapping to a chord-scale, a chord-tone set, and two
rootless comping voicings ([Theory.cs](Roads.App/Audio/Music/Theory.cs)):

| Quality | Chord-scale | Voicing A / B (semitones above root) |
|---|---|---|
| maj9 | major | 4·7·11·14 / 11·14·16·19 |
| maj7♯11 | lydian | 4·7·11·18 / 11·16·18·21 |
| 13 | mixolydian | 4·10·14·21 / 10·14·16·21 |
| 7alt | altered | 4·10·13·20 / 10·15·16·20 |
| 7sus | mixolydian | 5·10·14·19 / 10·14·17·19 |
| m9 | dorian | 3·7·10·14 / 10·14·15·19 |
| m11 | dorian | 3·10·14·17 / 10·15·17·19 |
| m7♭5 | locrian ♮2 | 3·6·10·12 / 6·10·12·15 |
| dim7 | whole-half | 3·6·9·12 / 6·9·12·15 |

These are the Bill-Evans-style A/B rootless forms: the bass owns the root, so four
upper-structure notes read as full 9th/13th chords. Voice leading is nearest-neighbor:
each comping hit picks the shape (±1 octave) whose top note lands closest to the
previous hit's top.

A separate **blues scale** rooted at the tune's key center (`0·3·5·6·7·10`) supplies
the idiomatic licks — 35% of melody bars in blues-family tunes use it instead of the
chord-scale of the moment.

## 5. The band (MIDI channel plan)

| Ch | Role | Patches (GM #) | Generator behavior |
|---|---|---|---|
| 0 | Comping | Rhodes EP 4 · jazz guitar 26 · drawbar organ 16 | Rootless voicings on Charleston-family rhythm patterns; density scales with intensity, drops when zoomed out; staccato stabs under tension; sparse whole-bar hits in nocturnes; simplified during solos |
| 1 | Bass | acoustic 32 · finger 33 (funk) | See per-style table below |
| 2 | Lead | alto 65 · tenor 66 · muted tpt 59 · harmonica 22 · vibes 11 · clarinet 71 · soprano 64 (day pool, 2–3 per tune) — flute 73/vibes/clarinet at night, vibes/flute/muted for bossa | Phrase machine: 1–2-bar phrases, 1–3-bar rests; constrained scale walk (75% stepwise, direction-biased) with chromatic passing; phrase-final long notes land on color tones (3rd/9th/13th), 25% chromatic grace-note crush; range G3–G5 (+5 in solos) |
| 3 | Pad | warm pad 89 | Root+5th+3rd+7th (+9th when deep) whole-bar bed; presence = max(night, 0.8×zoom-ambience) |
| 4 | Boogie piano | acoustic grand 0 | Swung-8th R-3-5-6-♭7-6-5-3 ostinato; blues-family solos only, intensity > 0.72, daytime |
| 5 | Horns | brass section 61 | Guide-tone dyads (3rd+7th) held whole-bar behind solo choruses at intensity > 0.7 (not bossa/nocturne) |
| 9 | Drums | standard kit · brush kit 40 | Four styles below |

**Bass styles:**

| Style | When | Line |
|---|---|---|
| Walking | blues/minor-blues/AABA choruses | Root → chord tone → scale step toward target → chromatic approach into the next bar's root; 10% swung skip-notes; register E1–D3 |
| Two-feel | first (head) chorus of swing tunes | Half-note roots and fifths, occasional pickup — kicks into walking for the solos |
| Funk ostinato | vamp chart | Fixed syncopated 16th figure on the root (finger bass); collapses to insistent root 8ths when tension > 0.7 |
| Tumbao | bossa | Dotted root / fifth push, straight 8ths, approach note into the next bar |
| Nocturne | nocturnes | Near-whole-note roots, occasional fifth (4/4 only) |

**Drum styles:**

| Style | Pattern |
|---|---|
| Swing | Ride on beats + swung skips, pedal hat 2·4, feathered kick, ghost snare, side-stick, ride bell at high intensity, tom fill on the chart's last bar |
| Funk | 16th hats with accents + open-hat pickups, syncopated kick (1 · 1a · 3&), backbeat snare, ghosts |
| Bossa | Straight-8th shaker bed, 3-2 clave-ish rim clicks, surdo-style kick (beats 1·3 with 8th pickups), pedal hat |
| Night/waltz | Barely-there ride on 1, soft pedal hat on remaining beats — covers 4/4 nocturnes and the 3/4 waltz |

All drum velocities scale with intensity, fade toward silence as night rises past
0.35–0.75, and duck by up to 50% at far zoom.

## 6. Chorus arrangement

Within a tune, the chart repeats as **choruses** with rotating roles:

- **Chorus 0 — head.** Sparse riff-like melody (density ×0.55), two-feel bass on
  swing tunes. Lead instrument chosen from the tune's palette.
- **Middle choruses — solos.** Melody density ×1.2, range +5 semitones, dense run
  templates; comping simplifies to stay out of the way; horns pad underneath at high
  intensity; boogie piano may join on blues tunes. **Trading fours:** in solo
  choruses (chart ≥ 8 bars, intensity > 0.5) every second 4-bar group hands the bar
  to the drums — melody rests, kit answers with accents and fills.
- **Interludes.** Blues-family tunes drop into one cycle of the funk vamp when
  congestion tension exceeds 0.55 or after three consecutive main-form choruses.
- **Final chorus** (budget nearly spent): back to a head role, and 25% of the time
  the whole band takes a **half-step gear-change lift** — decided at chorus start so
  the walking bass's chromatic approach targets the lifted key from bar one.
- Every chorus start (except nocturnes) gets a crash; the lead instrument rotates
  within the palette with a no-repeat guard.

## 7. Timing & feel

- **Tempo** = live setting + tune offset, modulated ×(1 − 0.10·night)(1 + 0.05·tension),
  slewed at most ±2.5 BPM per bar. Count-ins hard-set the new tempo (the break hides
  the jump); endings apply a separate ritardando factor (−3%/bar).
- **Swing**: off-beat 8ths shift late by `swing/6` of a beat (full swing = triplet
  placement), where `swing = user setting × chart factor × (1 − 0.7·night)`.
- **Humanization**: ±4 ms timing jitter and small velocity jitter on every note;
  melody phrases get a crescendo arc toward their final note.
- Mood values slew **half-way per bar** and are only read at bar boundaries, so every
  reaction lands on musical time.

## 8. Game-state coupling

Computed once per frame in `AudioEngine.UpdateMusicMood` (UI thread → atomic targets):

| Input | Source | Mapping |
|---|---|---|
| Intensity | driving vehicles ÷ (½ × max-vehicle setting) | `0.5 + slider·(0.15 + 1.05·density − 0.5)`, clamped 0–1 — gates drums/melody/piano layers and densities |
| Night | `SimulationClock.Darkness` × slider | Mellows tempo/swing/drums, swells pads, steers tune selection to nocturnes |
| Tension | share of vehicles < 2 m/s (the StatisticsPanel threshold), minus a 15% queuing floor, × fleet factor × slider | Staccato comping, pedal bass, vamp interludes, +5% tempo |
| Hour | `Clock.TimeOfDay` | Repertoire weighting (§2) |
| Day | `Clock.DayNumber` | Rollover arms a tune rotation — each in-game day opens fresh |
| Ambience | `1 − smoothstep(0.25, 0.9, zoom)` | Far zoom: drums −50%, pads up, comping thinner — the "overview mix" |
| Resolution | tension held > 0.5 for ≥ 3 s then falling < 0.2 (45 s cooldown) | One-shot ii–V–I cadence tag at the next bar boundary, then a fresh chorus — the jam audibly resolves |

The resolution trigger crosses threads as a monotonic sequence counter
(`ResolutionSeq`, the `OneShotVoice.TriggerSeq` idiom); triggers arriving while the
music is muted are discarded.

**Settings → Music** (persisted in `settings.json`): enable, volume, tempo center,
swing feel, and the three response strengths (traffic → energy, night → mellow,
congestion → tension). At zero, a mapping pins to its neutral value. The whole page is
LIVE: every control previews through the audio engine as it changes (landing at the
next bar), Apply/OK persist, Cancel/Escape re-preview the applied record to restore.

## 9. Manual mode & the mixer

**Manual mode** (Settings → Music → Mode → Manual) replaces the game-state coupling
with a settings-pinned band, so every combination can be auditioned without building a
traffic scenario:

- Mood comes from the manual Energy/Night/Tension sliders; ambience pins to 0 (full
  band regardless of zoom), the hour/day inputs freeze, and the resolution trigger is
  suppressed (its edge timer resets, so no stale cadence fires on return to auto).
- The tune is pinned: explicit form (all six, plus the 3/4 waltz nocturne addressable
  directly), key (Bb/Eb/F/C/Ab), a single lead instrument (the full 8-voice pool),
  comping patch, and drum kit (brushes fall back to standard when the soundfont lacks
  the GS patch). The manual `TuneDef` is deterministic — no RNG: tempo offset 0 (the
  tempo slider is WYSIWYG), no count-in, effectively infinite budget. Rotation arming,
  vamp interludes, and the gear-lift are suppressed; the chorus life (head/solo roles,
  trading fours, lead crash) continues, so a pinned tune still arranges itself.
- Change semantics, applied at the next bar boundary: form or key → the chart restarts;
  lead/comping/kit → plain program changes with no restart. Toggling back to Auto arms
  a rotation — the pinned tune plays its 2-bar ending, breaks, and `PickTune` resumes.

**The mixer** (both modes, all tune phases) is two-level:

| Level | Acts via | Keyed by |
|---|---|---|
| Category strips (Comp 0, Bass 1, Lead 2, Pad 3, Piano 4, Horns 5, Drums 9) | CC7 = base level × volume, 0 when muted | MIDI channel |
| Sub-strips (8 leads, 3 comps, 2 basses, 7 drum voices: Kick/Snare/Hat/Ride/Crash/Toms/Shaker) | note emission: velocity × volume; muted notes skip the whole on/off pair | drums by percussion note; melodic subs by the channel's current program |

Mute/solo semantics: audible = `!Mute && (nothing soloed || Solo)`, with solo scoped to
its level — category solos compete with categories, sub solos with subs of the same
category (solo Hat + Kick → only those drum voices; other categories unaffected).
`Composer.EmitMixer` is the **sole CC7 owner** (the base levels moved out of
`EmitSetup`); it runs at every bar boundary and emits only changes. Silencing via CC7
keeps note-offs flowing and sub-mutes skip whole pairs, so no path can hang a note.

**Threading**: the manual and mixer fields join the provider's atomic-target contract —
the UI thread writes 32-bit fields/array elements; `ComposeBar` snapshots them at the
bar boundary (`MoodInputs` + `Composer.SetMixer`). Expect up to ~2 bars of perceived
latency (one bar is composed ahead); that quantization is the musical contract, not a
defect.

## 10. Diagnostics & verification

`ROADS.exe --musictest[=seconds] [--musicout=path]` renders the engine offline
through the exact live code path: five mood phases (morning-rush, midday-calm,
evening-jam, resolution — which fires the cadence trigger — and night with a day-number
bump), writing a WAV plus a report with per-phase RMS, the current tune per phase,
and brush-kit availability. Renders ≥ 360 s additionally assert that the setlist
actually rotated (`TunesStarted ≥ 2`). Exit 0 = every phase produced audio. Fixed
seed — reruns are byte-comparable; in the app the seed varies per launch so each
session opens with a different tune.

## 11. Invariants

The load-bearing ordering/threading contracts (compose-on-playhead, MIDI tie-rank,
CC91 and CC7 single ownership, form-persists-per-tune, ritardando isolation, trigger
discard-when-frozen, the pause-duck bypass, and the SimRandom prohibition) are
catalogued in [HIDDEN_DEPENDENCIES.md §8](HIDDEN_DEPENDENCIES.md). Read that before
modifying the sequencer or adding new mood inputs.
