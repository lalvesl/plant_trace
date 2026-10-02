# Bench procedure

What to do with the board on the table, in order. Everything here has been
exercised against the simulator; the steps marked **first time** are the ones
that have never met the real hardware.

## 0. Before powering anything

- Build the two RC networks (`ngspice_filter.cir`, one per output) — the
  `u_T` one with its middle shunt going to a divider off 3.3 V instead of to
  ground, which lifts it into the plant's 2.25-2.75 V window
  (`docs/HARDWARE.md`, *The `u_T` offset ladder*) — and tie the plant's
  ground to the board's. A floating plant ground reads as a slow
  common-mode drift on all three channels at once.
- **Measure VDD on the board, powered from USB.** Nominal is 3.3 V, which is
  what the ngspice deck assumes, but the calibration in §2 is what actually
  matters — and running from a LiPo makes it a moving target.
- Wire each of the three inputs to its pin through one 10 kΩ series resistor,
  and nothing else: no divider, no clamp. The pin reads the plant's voltage
  unchanged, to 3.0 V. The resistor only limits what a miswire can push into
  the pin's own ESD structures — outside the datasheet, and chosen knowingly
  (`docs/HARDWARE.md`, *Input protection*). Nothing above the 3.3 V rail
  belongs on these wires.
- Confirm the plant's output never exceeds 3.0 V. `P_e` should live in
  2.25-2.75 V like `u_T`; 3.0 V is the SAADC's full scale referred to the
  plant, above it the reading saturates silently, and a clipped overshoot is
  indistinguishable from a well-damped one.

## 1. Bring up the rig — **first time**

```sh
nix develop
cd firmware/nrf-daq
# double-tap RESET: a drive appears, holding INFO_UF2.TXT
cargo run --release
```

That builds, converts to UF2 and copies it over; the board reboots into the
application by itself. If no drive is found it falls back to serial DFU, and
failing that prints both commands for you to run by hand.

Check `INFO_UF2.TXT` on the first flash: it names the bootloader and the
application start address. **If that address is not `0x26000`**, change both
`ORIGIN` in `firmware/nrf-daq/memory.x` and the default `--base` in
`tools/uf2.py` before flashing anything. A wrong base does not fail loudly —
the board takes the file and then never enumerates.

Then:

```sh
ls /dev/ttyACM*                       # the application, not the bootloader
lsusb | grep plant-trace              # "plant-trace rig"
plant-trace info --daq /dev/ttyACM0
plant-trace gen  --daq /dev/ttyACM0 info
```

There is no debug probe on this board, so there is no boot log to watch —
`info` answering *is* the liveness check. If the port never appears at all, the
firmware is not running: re-flash, and suspect the base address or `set-vtor`
(see `docs/HARDWARE.md`).

**Front-end check**: feed a known DC voltage into the `P0.02` resistor and
confirm the count. The scale is 3.0 V over 12 bits, so
`counts = volts × 4096 / 3.0` and 2.500 V should read ≈ 3413. `plant-trace info` prints the scale
and the headroom it is assuming. If it reads high by a percent or two, put the
correction in `gain_correction` rather than adjusting anything in the firmware.

**Output check**: put a scope on `P1.11` — 62.5 kHz, 16 µs period — and a DMM on
the `u_T` ladder's output.

```sh
plant-trace gen --daq /dev/ttyACM0 level --ch u_t --volts 2.25
plant-trace gen --daq /dev/ttyACM0 level --ch u_t --volts 2.75
```

Each level is clamped to the output's window (2.25-2.75 V on `u_t`, 0-1 V on
`p_s`), whatever is typed. If the low level reads high and the high one low,
the PWM polarity is inverted: swap `DutyCycle::normal` for
`DutyCycle::inverted` in the `waveform` task. Nothing else changes.

## 2. Check the outputs — **first time, and after any rework**

Before calibrating anything, make the rig measure itself. `u_T` and `p_s` are
wired to ADC channels precisely so it can: every level the PWM commands is read
back through the 10 kΩ sense resistor on the next pin, over the same link, in the same
session.

### 2a. DC — the whole transfer curve at once

```sh
plant-trace check --daq /dev/ttyACM0 dc            # both outputs, each across its window, 11 levels
plant-trace check --daq /dev/ttyACM0 dc --ch u_t   # one, to prove the wiring
```

Each output steps through its own window — `u_t` 2.25…2.75 V, `p_s` 0…1 V —
together; `--from`/`--to` narrow both to one range. Each level is commanded,
given 250 ms to settle, then averaged over 500 ms. What comes out:

| column | what it tells you |
| --- | --- |
| `asked` | the level commanded on that output |
| `code` | the duty the firmware chose — the fit's x axis |
| `read` | what the ADC measured through the sense resistor |
| `pp mV` | spread over the plateau: quantisation, noise and any carrier the burst averaging missed |

and then a fitted line per output, ready to paste into the `[outputs.*]` table
of the experiment files.

Read it in this order:

1. **`--ch u_t` must leave the `p_s` column flat**, and the other way round. A
   column that moves with the wrong output is crossed wiring, and every curve
   measured afterwards would be wrong in a way no analysis can detect.
2. **Worst deviation from the line** should be under a millivolt —
   roughly one ADC count (732 µV). A millivolt or more means something is
   loading the ladder in a way that is not linear. A plain resistive load only
   tilts the line, which the fit absorbs, so suspect something in the plant's
   input that conducts more at the top of the range.
3. **`pp mV` on a plateau** is the noise floor plus leftover carrier. SPICE puts
   1.32 mV_pp of 62.5 kHz at the plant and the SAADC's 8× burst should take that
   down to about a count; several millivolts here means the burst is not
   landing where the analysis assumes, or that the second filter stage is
   missing. The first stage alone leaves 80 mV_pp at the plant, and up to
   ~10 mV of it gets through the burst into this column
   (`docs/HARDWARE.md`, *One stage or two*).
4. **`offset_v`** should be within a millivolt of zero on `p_s`, and near
   2.150 V on `u_t` — the offset its divider sets. On `u_t`, check too that
   code 0 is below 2.25 V and code 255 above 2.75 V (`offset_v` and
   `offset_v + 255 × volts_per_code`); the host refuses a line that cannot
   reach the whole window, and a resistor in the divider is the fix.

### 2b. DC — the absolute reference

`check dc` measures the *shape* of the curve very well and its *absolute* scale
only as well as the internal 0.6 V reference allows, which Nordic gives to
about ±1.5 % before calibration.

**The checked-in experiment files use the `check dc` line for `p_s`**
(2026-09-27, 1 kΩ ladders): 4.237 mV/code. **`u_t` carries its offset
ladder's nominal line** — 2.150 V + 2.77 mV/code — until `check dc` is run on
the new stage; paste its two numbers over them in every file. That puts commands and
recordings on one scale — the ADC's — which is all a gain *ratio* needs, and
it is better than a meter whose own calibration is years old. What it cannot
give is absolute volts to better than ~1.5 %. If a report ever needs those,
anchor with a calibrated meter: command two levels and write down what it
says at the plant input.

```sh
plant-trace gen --daq /dev/ttyACM0 level --ch u_t --volts 2.30   # → V_lo
plant-trace gen --daq /dev/ttyACM0 level --ch u_t --volts 2.70   # → V_hi
```

`gen … level` prints the code that was actually written (`C_lo`, `C_hi`).
Then

```
volts_per_code = (V_hi − V_lo) / (C_hi − C_lo)
offset_v       = V_lo − C_lo × volts_per_code
```

and put those two numbers in the `[outputs.u_t]` block of every experiment
file. Leave `min_v`/`max_v` at the plant's window (2.25/2.75 on `u_t`, at most
0.05/0.95 on `p_s`): the host refuses anything wider.

If the DMM and `check dc` disagree on the slope by more than about a percent,
the disagreement is in the *sense* path (the reference), not in
the output. Trust the DMM for `[outputs.*]`, and carry the ratio into
`AdcScale::gain_correction` if you want the recorded `u_t` column to agree with
the meter too.

Allow ~10 ms after a level change before reading the DMM: the filter's dominant
time constant is 200 µs and its slowest visible settling is about eight of those.
The `u_T` ladder's poles are a little faster than that, not slower.

### 2c. Sine — the filter, measured

```sh
plant-trace check --daq /dev/ttyACM0 sine --freq 10     # 40 % of each window, on its middle
plant-trace check --daq /dev/ttyACM0 sine --freq 100 --ch p_s --amplitude 0.3
```

A sinusoid is applied to both outputs and fitted back off the sense channels at
a frequency that is known exactly, so everything that is *not* at that frequency
lands in the residual instead of being read as amplitude. The report gives, per
output, the measured amplitude, the gain against what was commanded, the centre
error, and that residual — and then what the design predicts for the same
frequency, from the ngspice poles plus the hold the waveform tick amounts to.
By default each output swings over 40 % of its window around the middle of
it: 2.5 ± 0.2 V on `u_t`, 0.5 ± 0.4 V on `p_s`. The prediction is for the
plain ladder; `u_t`'s offset divider lowers its middle shunt, which moves
its poles up to about 1.0/5.9 kHz — a few hundredths of a dB at 100 Hz.

- **Gain should match the prediction to a few hundredths of a dB** up to ~100 Hz.
  It is the check that the board was built the way `ngspice_filter.cir` says:
  a swapped resistor value moves the corner and shows up here long before it
  shows up in an identification run.
- **Channel skew should be zero.** Both duties go out in one DMA transfer, so
  anything above a sample period is the measurement, not the rig.
- **The residual grows with frequency over tick.** ~2 % at 10 Hz, ~20 % at
  100 Hz. That is the reconstruction staircase the 1 kHz tick leaves, not
  distortion — see *Known limitations*.
- **Absolute phase is not measurable from the host**: there is no view of the
  tick the waveform started on. Only the difference between channels means
  anything, which is why skew is reported and lag is not.

Sample at 2 kHz for anything fast — the default for this check — so the image
at `tick − freq` does not fold straight onto the fundamental and beat with it.

## 3. A first recording

```sh
plant-trace record --daq /dev/ttyACM0 --out data/first.csv --duration 30 \
    --calibrate --note "first light"
```

Watch the live line: all three channels should sit where you put them, and
`dropped` should stay at 0. A non-zero drop count means the link cannot keep
up — lower `fs` before doing anything else, because dropped blocks in the
middle of a transient cannot be recovered afterwards.

## 4. The assignment's measurements

Edit the calibration block at the top of each experiment file first — step 2 is
what fills it in.

```sh
plant-trace run experiments/static-u.toml --daq /dev/ttyACM0
plant-trace run experiments/static-p.toml --daq /dev/ttyACM0
plant-trace run experiments/step-u.toml   --daq /dev/ttyACM0
plant-trace run experiments/step-p.toml   --daq /dev/ttyACM0
plant-trace run experiments/freq-u.toml   --daq /dev/ttyACM0
plant-trace run experiments/freq-p.toml   --daq /dev/ttyACM0
plant-trace run experiments/prbs-u.toml   --daq /dev/ttyACM0
```

Rough wall-clock cost: the static curves are 12 and 8 minutes, the step sets
about 6 minutes each, the sine sweeps 7 and 6 minutes, the PRBS 6 minutes —
plus settling, which dominates the first step of every run. Start with
`step-u`: it is the shortest thing that tells you whether the wiring is right.

Then:

```sh
plant-trace analyze data/step-u-20260919T...
```

which writes `analysis/` next to the recordings: `steps.csv` with every
metric, `static-*.csv` with the gain curve, `bode-u_t.csv` with the frequency
response, and a gnuplot `.gp`/`.pdf` for each.

### 4a. Automatic Bode

> **The plant is tested up to 15 Hz.** Every Bode plan refuses a frequency
> above its `max_freq_hz` (15 Hz by default), and every experiment refuses a
> sine or chirp above 15 Hz (`experiment::PLANT_MAX_FREQ_HZ`). Only
> `bode-wire.toml` raises it, to 100 Hz, because there is no plant in that
> loop — only a wire from the `u_T` stage output to the `P_e` input.

`freq-u.toml` and `freq-p.toml` measure the frequency response as ordinary
record steps, analysed afterwards. `plant-trace bode` does the same
measurement as one sweep, on one stream, fitted as it goes:

```sh
plant-trace bode --init my-plan.toml                        # the default plan, to edit
plant-trace bode experiments/bode-u.toml --daq /dev/ttyACM0  # ΔP_e/Δu_T
plant-trace bode experiments/bode-p.toml --daq /dev/ttyACM0  # ΔP_e/Δp_s
```

It prints the estimated duration first (about 14 minutes for `bode-u`), then
one table row per frequency. Into the output directory (`--out`, default
`data/<name>-<timestamp>`) go `bode.csv` (one row per point, units in the column
names, the plan in the `#` header), `bode.json` (plan and result) and
`stream.csv`, the whole raw stream in the format of every other recording.

How a point is measured: the excited output runs a sine of a whole number of
periods, so it ends on its centre line and the next frequency starts from the
operating point; a settle span is discarded and a window of whole periods is
fitted with the same least-squares fit as the analysis, on the *measured*
input (the sense channel of the excited output) and on the response. Settle
and window lengths are given in periods and bounded in seconds
(`settle_cycles`/`settle_min_s`/`settle_max_s`,
`measure_cycles`/`measure_min_s`/`measure_max_s`), which is what keeps a sweep
down to 0.02 Hz at minutes rather than an hour. Phase is negative when the
response lags, and is unwrapped along the sweep in the order the frequencies
are listed.

**The scan skew.** The SAADC converts `u_T`, `p_s`, `P_e` one after another,
~89.5 µs apart (`docs/PROTOCOL.md`), so `P_e` is read ~179 µs after the `u_T` it
shares a row with and appears to *lead* it by `360·f·179 µs`. The table shows
the raw phase, that skew (`skew °`, positive = the response channel is
converted later), and the corrected phase `raw − skew`, which is what
`phase_deg` holds when `correct_skew = true`. Around the rotor mode it is a
tenth of a degree; at 100 Hz it is 6.4°.

**Checking the measurement with a wire.** Unplug the plant and wire the `P_e`
input to the `u_T` ladder's output. The true transfer is then exactly 1, so
everything the sweep reports is the rig:

```sh
plant-trace bode experiments/bode-wire.toml --daq /dev/ttyACM0
```

What it should show:

- **gain** 0.00 dB to a few hundredths at every frequency — sense and `P_e` read
  the same node through identical 10 kΩ resistors, so the output filter and
  the tick cancel out of the ratio;
- **raw phase** a lead growing in a straight line, +0.6° at 10 Hz, +6.4° at
  100 Hz; **corrected phase** 0° to a few hundredths;
- the closing line **pure-delay fit through the raw phase: −179 µs** — negative,
  because the later channel reads early — and an **implied scan spacing** of
  ~89.5 µs per channel. That is what this board measured on 2026-09-27
  (−178.7 and −179.0 µs in two runs, rms 0.01°); the datasheet estimate is
  96 µs.

The delay fit is only meaningful when the true system has no phase of its own
(this wire, or one sense channel against another); on the plant it is a number
without a meaning. If the implied spacing is clearly not 89.5 µs, it is the real
one: put it in `scan_spacing_s` of the other plans. A gain away from 0 dB means
the two inputs are not reading the same thing — a bad joint, or a series
resistor that is not 10 kΩ. The simulator reproduces the loopback, skew
included, without the bench:

```sh
plant-trace simulate --plant wire-u --speed 5 &
plant-trace bode experiments/bode-wire.toml --daq tcp://127.0.0.1:7801
```

## 5. Reading the results critically

- **The plateaus must be flat.** `output_sd_v` in a static curve is how flat
  they were. If it is not close to the ADC's resolution (≈0.7 mV), the dwell is
  too short and the "static" gain is a response caught in transit.
- **The incremental gain should vary.** If it does not, either the valve is
  more linear than the assignment expects or the operating range is too narrow
  to show it.
- **Check `residual_ratio` on every Bode point.** Above ~0.3 the fit did not
  explain the record: the excitation is too small for that frequency, and the
  point should be reported as unreliable or dropped, not drawn.
- **Damping from a step is a lower bound.** A ±5 % step excites the
  electromechanical mode weakly; the sine point nearest the resonance is the
  better measurement. The analysis returns no damping at all rather than a
  number it cannot support.
- **Settling time is measured to ±2 %** of the step, taken as the last
  departure from the band — not the first entry into it.
- **The identification uses the recorded `u_T`, not the command.** So the
  filter's own 1.07 ms lag is already inside the measurement and must not be
  subtracted again.

## Known limitations

| limitation | consequence |
| --- | --- |
| 8-bit duty: 2.77 mV/code on `u_T`, ~4.24 on `p_s` | a ±5 % excitation is ~9 codes on `u_T` (its window is 0.5 V wide) and ~12 on `p_s`; small sines are visibly quantised |
| Rail-referenced outputs | full scale follows VDD, so a LiPo drains the calibration away with it — run from USB |
| No debug probe | no `defmt` log and no `probe-rs`; the CLI answering is the only liveness signal |
| Output filter at 773 Hz | a half-scale edge settles to one count in 1.5 ms, so the filter is no longer what limits an excitation |
| SAADC at 3.0 V full scale | anything above 3.0 V saturates without warning, and a clipped overshoot looks well-damped; `p_s` pays 732 µV counts for sharing it |
| Channels of a row ~89.5 µs apart | a phase lead of `360·f·Δt` on later channels; `bode` corrects it, `analyze` does not (0.1° at the rotor mode) |
| Carrier rejection is −60 dB, not −81 | 1.32 mV of ripple at the plant and at the pin; the 8× burst takes it under a count, but check it on a flat plateau |
| The 1 kHz waveform tick is the bandwidth limit | above ~20 Hz the reconstruction images are barely filtered — `check sine --freq 100` shows them as ~20 % residual; below it, nothing to see |
| Sample rate capped at 2 kHz | the 8× burst oversampling costs 288 µs per scan |
| Valve rate limit and saturation | large steps leave the linear region; keep to ±5 % |
| Local linear models | a fit at `u_T` = 2.425 V does not predict behaviour at 2.60 V — that is the point of measuring two operating points |
| No link-loss watchdog | an interrupted run leaves the outputs where they were; use `gen … park` |
