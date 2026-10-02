# The bench GUI

```sh
plant-trace gui                          # opens offline; pick a port and connect
plant-trace gui --daq /dev/ttyACM0       # connect at start-up
plant-trace gui --daq tcp://127.0.0.1:7801   # a simulator started with `plant-trace simulate`
```

No board? **Simulated rig** in the connection bar starts the simulator inside
the GUI's own process and connects to it — with the plant model, or with the
plant replaced by a wire from `u_T` or `p_s`, which is the bench with the
`P_e` input looped back to an output.

One binary. The GUI is a module of `plant-trace` (`host/plant-trace/src/gui/`)
behind the default `gui` feature, and it calls the same library functions the
commands do — `check::dc`, the experiment runner, the Bode sweep — on a thread
that owns the rig link. Nothing is measured or computed in the GUI that the
command line cannot reproduce, and every file it writes is the file the
command line would have written. `cargo build --no-default-features` builds the
headless CLI without any of it.

It is built on [egui_shadcn](../../aaaaaaaaa/egui_shadcn) (`egui_sc`), taken by
path from the sibling checkout while the two evolve together.

## Tabs

| tab | what it does | the command-line equivalent |
| --- | --- | --- |
| **Monitor** | live view of `u_T`, `p_s` and `P_e`; hold either output at a level by hand, inside its window (2.25-2.75 V on `u_T`, 0-1 V on `p_s`); mean and peak-to-peak of the last 100 ms; an input divider ratio per channel, if one is ever put in front of a pin | `record`, `gen level`, `gen park` |
| **Output check** | a free-running sine generator on either output or both (offset, amplitude, frequency; started on the same tick, stopped back onto its centre line) with the live view under it; the DC sweep and the sine readback of `docs/EXPERIMENTS.md` §2, with the fitted line, its residuals and the plateau ripple drawn | `gen` + `record`, `check dc`, `check sine` |
| **Scenarios** | the experiment files as forms: steps, waveforms, output calibration; a preview of what the outputs will be commanded to do; run with the data drawn live | edit `experiments/*.toml`, `run` |
| **Bode** | edit a `bode-*.toml` plan — excitation, response channel, settle/measure cycles, the frequencies (generated log/linear, plus single points) — run the stepped-sine sweep, watch the diagram grow | `bode` |
| **Results** | open a run under `data/`, look at each recorded segment, run the analysis and see its tables | `analyze` |

The live view pauses while a job holds the link — every job starts and stops
its own stream — and resumes when the job ends. **Cancel** in the status bar
stops a job at its next block; the outputs are always stopped and parked on
the way out, as they are on the command line.

## Scenarios are files

A scenario *is* an `experiments/*.toml` file. The editor loads and saves that
format and nothing else, so a scenario built here runs with `plant-trace run`,
and a file written by hand opens here. The **TOML source** section shows the
file as it will be saved and takes edited text back into the form.

The editor checks the scenario as it is typed with the same validation the
runner applies at load time (`Experiment::validate`): a level outside an
output's safe window is flagged before anything reaches the plant. Every level
field is bounded by its output's window — 2.25-2.75 V on `u_T`, 0-1 V on
`p_s` — and a new waveform's step or amplitude is 5 % of that window's span.

**Use the last DC sweep** copies the calibration line the Output check tab just
fitted into the scenario's `[outputs.*]` tables. Anchor it with a meter before
reporting absolute gains (`docs/EXPERIMENTS.md` §2b).

The preview uses the generator the firmware runs (`proto::waveform`), so what
it draws is what the plant will be sent. Settle steps are drawn at their
shortest possible length, one steady window; on the rig they last until `P_e`
is steady.

## The Bode sweep

Frequencies above the plan's `max_freq_hz` — 15 Hz, the limit the plant is
tested to — are refused before anything is driven; the form says so. Only the
wire-loopback plan raises it.

Each frequency gets a sine around the operating point: settle for a number of
cycles (with a floor in seconds), then measure for a number of cycles (with a
floor), and fit both the excitation and the response at exactly that
frequency. The excitation is the *measured* sense channel of the driven
output, never the commanded value.

**Scan-skew correction.** The SAADC reads its channels one after the other,
each an 8× burst of about 11 µs, so the `P_e` sample of a scan is taken about
179 µs after the `u_T` one but filed in the same row. A later channel therefore
reads as a *lead*: +0.07° at 1 Hz, which the plant would never notice, and
+6.4° at 100 Hz, which a loopback would. The correction removes
`360° · f · Δt` using the spacing shown in the form. With the plant replaced by
a wire the corrected phase should be flat at 0°, and the *pure delay fit* the
sweep reports is a measurement of the real spacing.

**Where the results go.** Every sweep — finished or cancelled — writes
`data/<plan>-<UTC time>/` with `bode.csv` (the table), `bode.json` (the same
result, complete) and `stream.csv` (every raw sample). The run bar shows the
directory once the sweep ends. *Saved sweep* lists every `data/*/bode.json`,
newest first; **Open** redraws that sweep's diagram, table and delay fit
(following its own plan's response channel and frequency range), and **Use
its plan** copies that plan into the form so it can be run again.

## Development

- Tests drive the app headlessly (`tests/gui.rs`): a real `egui::Context`
  stepped frame by frame against an in-process simulator, the way egui_shadcn
  tests its own components. `App::show(ui)` is split out of `eframe::App` for
  exactly that.
- `PLANT_TRACE_CAPTURE=shot.ppm plant-trace gui` saves an image of the window
  after a few seconds and closes it (`PLANT_TRACE_TAB=n` picks the tab). It
  captures that window only.
- The dev shell carries what eframe links at run time (X11/Wayland/GL) and the
  OpenSSL egui_shadcn's build script needs to fetch its icon font.
