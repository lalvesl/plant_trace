# Hardware notes

Everything here is what the firmware assumes. Pins live at the top of
`firmware/nrf-daq/src/main.rs`, one per line with its silkscreen label, so a
different wiring is a one-line change — but change the firmware, not the
reality, and re-check this file first.

The board is an **nRF52840 Supermini** with the nice!nano v2 pin map, running
the Adafruit nRF52 bootloader.

## Signal chain

| signal | source | scaling | destination | range at the plant |
| --- | --- | --- | --- | --- |
| `u_T` valve command | `P1.11` (`D14`), 62.5 kHz PWM | 1:3 divider + 2-pole RC, 773 Hz, with its middle node shunted to a 2.74 V divider off 3.3 V instead of to ground: 2.150 V + 0.709 V × duty | plant input, and 10 kΩ to `P0.02` / AIN0 | **2.25-2.75 V**, centred on 2.5 V |
| `p_s` steam pressure | `P1.13` (`D15`), 62.5 kHz PWM | 1:3 divider + 2-pole RC, 773 Hz | plant input, and 10 kΩ to `P0.29` / AIN5 | 0-1 V |
| `P_e` electrical power | plant output | 10 kΩ series resistor | `P0.31` / AIN7 | **2.25-2.75 V**, like `u_T` |

One board drives and measures. The two outputs are wired back into the SAADC
at the node the plant is fed from, so what the plant is fed and what it answers land on the same
scan: nothing downstream has to align two clocks or trust a commanded value.

`P0.02`, `P0.29` and `P0.31` are the only analog-capable pins this board brings
out — exactly as many as the rig needs, with none to spare. Silkscreen
numbering varies between vendors of this board; trust the GPIO names.

Tie the plant's ground to the board's. The DAQ measures voltages relative to
its own ground, so a floating plant ground shows up as a slow common-mode drift
on every channel at once.

## The PWM DAC

`ngspice_filter.cir` is the circuit; `sim_results/report.md` is its
characterisation, produced by `nix run .#rc_filter`. Every part in it is an
**1 kΩ resistor or a 100 nF capacitor**.

```
 P1.11 ──[1k]───[1k]───┬──[1k]───┬──────────────► u_T (plant input)
                       │         │
                     [1k]     [100n]
                       │         │
                    [100n]      GND
                       │         └──[10k]──► P0.02
                      GND
```

- **62.5 kHz carrier, 8-bit duty.** `PWM_CLK = 16 MHz` (prescaler ÷1) over a
  256-count `COUNTERTOP`: the fastest carrier this part produces at 8-bit
  resolution. A duty *is* an 8-bit code — which is why `DacScale` did not have
  to change shape when the outputs moved off the ESP32's DAC.
- **Code 255 is 255/256 of full scale**, not all of it. 0.4 % of range, which
  the two-point calibration folds into `volts_per_code` like everything else.
- **Averaging is exactly linear in the duty**, and so is the ladder, so
  `volts = offset_v + code × volts_per_code` holds even including the GPIO's
  on-resistance — that only moves the two endpoints. Two DMM measurements pin
  the line down completely.
- **Corner at 773 Hz**, poles at 796 Hz and 4775 Hz. A half-scale step is 90 %
  of the way in 0.5 ms and within one count in 1.5 ms; at 100 Hz the filter
  takes off 0.07 dB and 8.4°, and at 60 Hz, 0.03 dB and 5.0°.
- **Nothing loads it.** The plant input and the 10 kΩ into the sense pin are
  both high impedance, so full duty is the divider's own 1/3 of VDD — 1.100 V —
  and the pin reads exactly what the plant is fed: the series resistor carries
  no DC and drops nothing. Output impedance is 1.67 kΩ, so a plant input of
  100 kΩ would pull the level down 1.6 %, a gain error the calibration absorbs
  because the sense pin sees the same node.
- **Carrier rejection is −60.3 dB** and the ripple at the plant is 1.32 mV
  peak-to-peak (SPICE). The pin sees all of it — about 7 counts — and the
  SAADC's 8× burst averages most of it away: its eight conversions are ~12 µs
  apart against a 16 µs carrier period, landing on four evenly spaced phases.
  Expect well under one count; **measure it** on a flat plateau and compare
  against the one-count (0.7 mV) resolution of the 3.0 V full scale.
- **GPIO current is 1.1 mA** at full duty, comfortably inside high drive.
- **A duty change lands at a carrier-period boundary.** `set_all_duties` hands
  the peripheral a single DMA descriptor holding all four compare values, so
  both outputs change in the same transfer — there is no skew between `u_T` and
  `p_s` — and no pin ever changes mid-pulse: a new duty takes effect at the next
  16 µs period. The write itself busy-waits for the DMA read, microseconds, not
  a period. None of that is the limit in practice: the granularity above it is
  the 1 ms waveform tick, and the firmware skips the write entirely when the
  codes have not changed, so a held level writes nothing at all.

**To verify all of this on the bench**, `plant-trace check dc` and
`plant-trace check sine` drive the outputs and read them back through the sense
resistor in one session — the fitted line, the carrier ripple on a plateau, the
filter's gain against the poles above, and the skew between the two channels.
`docs/EXPERIMENTS.md` §2 is the procedure.

## The `u_T` offset ladder

The box takes `u_T` and gives `P_e` in a **2.25-2.75 V** window centred on
2.5 V; only `p_s` lives in 0-1 V. So the `u_T` ladder is the `p_s` ladder
with one change: the 1 kΩ from its middle node to ground is replaced by a
divider off the 3.3 V rail, and the filtered PWM rides on that instead of on
0 V. Passive, off the same rail as the PWM, and with nothing to power.

```
                      node A                     node B
 P1.11 ──[1k]──[1k]─────┬───────────[1k]─────────┬────────────► u_T (plant input)
 (PWM)                  │                        │
                        ├──[330]──[330]── 3.3 V  │
                        │                        ├──[10k]─────► P0.02 / A0
                        ├──[1k2]──[1k2]──[820]─┐ │
                        │                      │ │
                      [100n]                  GND [100n]
                        │                        │
                       GND                      GND
```

- **The line.** Node A is the weighted mean of its branches (Millman's
  theorem). The divider is a Thévenin source of 3.3 × 3.22k / 3.88k =
  **2.739 V** behind 660 Ω ∥ 3.22 kΩ = **548 Ω**; the PWM pin swings 0-3.3 V
  behind the ladder's 2 kΩ. With `k = 548 / (548 + 2000) = 0.215`:

  ```
  u_T = (1 − k)·2.739 V + k·3.3 V·duty = 2.150 V + 0.709 V·duty
  ```

  so code 0 is **2.150 V**, one code is **2.77 mV**, code 128 is 2.50 V and
  code 255 is **2.857 V**. The window is ~180 codes wide, and a ±5 % step
  (25 mV) is about nine of them. The second section carries no DC — the
  sense pin and, near enough, the plant input draw none — so node B is
  node A. `u_t_stage` in `proto/src/scale.rs` holds the resistor values and
  derives `DacScale::U_T_NOMINAL` from them: a different part is a one-line
  change there.
- **Why the divider sits at 2.74 V and not 2.5 V.** The PWM branch pulls
  node A toward its own mean, 1.65 V at 50 % duty. With the divider at
  2.74 V, 50 % lands on 2.50 V, the middle of the window. With the PWM pin
  floating — reset, flashing — node A goes to the divider's 2.74 V, inside
  the window.
- **Why the divider is stiff.** Its 548 Ω against the PWM's 2 kΩ is what
  keeps the PWM's share, and so the span, down to 0.71 V. A softer divider
  (10k/10k, 5 kΩ) lets the PWM dominate: 0.71-3.06 V, 55 codes in the
  window. The rule is `R_divider ≈ 0.27 × R_PWM`.
- **Tolerance.** With 5 % resistors, code 0 can sit anywhere in 2.07-2.22 V
  and code 255 in 2.82-2.89 V — the window is covered at every corner.
  `plant-trace check dc` measures the real line; the host refuses a table
  whose line cannot reach the whole window.
- **The ladder cannot leave 2.15-2.86 V**, whatever the duty: under the
  ADC's 3.0 V full scale and the pin's rail. Inside that, the window is
  enforced three times: the firmware clamps every level to the window of the
  scale it holds and refuses a waveform that leaves it; the host refuses an
  experiment or a Bode plan whose `[outputs.u_t]` window is wider than
  2.25-2.75 V; and a code is rounded *inwards* at the edges, so half a code
  past 2.75 V is not commanded either.
- **Parked is 2.25 V.** `Park` sends each output to the bottom of its window.
  Between reset and the first 1 ms tick the duty is 0, so the plant sees
  2.15 V for about a millisecond at power-up.
- **Ratiometric.** The divider and the PWM hang off the same 3.3 V rail, so a
  change in the rail scales the whole line instead of moving the offset
  against the span. That is why the divider is not on USB's 5 V: 1 % of
  VBUS would move `u_T` by ~21 mV, and with the board off it would feed the
  nRF through the PWM pin and the sense resistor.
- **The filter.** Node A's shunt drops from 667 Ω to 430 Ω (548 Ω ∥ 2 kΩ),
  so the poles move from 796/4775 Hz to about **1.0/5.9 kHz** and the output
  impedance from 1.67 to 1.43 kΩ. The capacitors are unchanged; the PWM's
  share of node A drops from 1/3 to 0.215, so the carrier ripple at the plant
  is about the same. `bode-wire.toml` re-measures the poles.

How it got here, for the record (code 0 … code 255):

| `u_T` stage | `u_T` | per code | codes in window |
| --- | --- | --- | --- |
| CA3130 summer, 820/330 divider, equal 10k inputs, gain 2 | 1.01 … 2.03 V | 4.0 mV | none |
| CA3130 summer, 3.3 V–10k, ladder–8k2, gain 4/3 | 2.185 … 2.92 V | 2.88 mV | ~173 |
| divider at node A, 10k/10k off 5 V | 0.71 … 3.06 V | 9.2 mV | 55 |
| **divider at node A, 660 Ω / 3.22 kΩ off 3.3 V (this one)** | **2.150 … 2.857 V** | **2.77 mV** | **~180** |

The CA3130 versions were dropped on the bench: on USB's 5 V — the bottom of
that part's rated supply — its output was erratic even with its input
grounded, and the passive network needs neither the part nor a supply.

### What the bench found (2026-09-27)

The loopback Bode (`plant-trace bode experiments/bode-wire.toml`, `P_e` wired
to the `u_T` filter output) reads the output chain directly: the excitation
amplitude it fits at each frequency *is* the filter's response, and the raw
phase between the two channels *is* the scan skew. Its first runs found three
things no DC check could:

- **The filter was a decade slow.** The fitted amplitudes followed a two-pole
  response with poles at 96.6 and 579 Hz — the design's, divided by 10.05,
  rms error 0.03 mV. The ladder had been built with 8.2 kΩ
  (grey-red-**red**) where 820 Ω (grey-red-**brown**) was meant; DC cannot see
  it, because the divider ratio is the same. The rebuild uses 1 kΩ, which is
  what this section now describes.
- **The generator's clock was the internal RC oscillator.** Fitted amplitudes
  also fell with frequency and residuals grew to 72 % at 100 Hz: the sine was
  a few tenths of a percent off the frequency the host fitted at. The RTC
  behind embassy's `Instant` and `Ticker` runs on the LFCLK, which embassy
  leaves on the internal RC. The firmware now synthesises the LFCLK from the
  32 MHz crystal; residuals dropped to 1.6-2.8 % across 1-100 Hz.
- **The scan spacing is 89.5 µs, not 96.** The raw phase is a pure delay to
  0.01° rms, −179 µs between `u_T` and `P_e`, in two runs.

After the rebuild with 1 kΩ the same sweep fitted the `u_T` ladder's poles
at **795 and 4770 Hz** against the design's 796 and 4775 Hz (rms 0.05 mV over
1-100 Hz); gain within ±0.008 dB and the skew-corrected phase within ±0.02°
up to 100 Hz; spacing 89.1 µs. `check sine --freq 100` read −0.208 dB on
`u_T` against a predicted −0.213 dB. The DC sweep put the two outputs at
4.310 and 4.237 mV/code — 5 % resistors move the divider by that much — so
anchor both with the meter before reporting absolute gains (§2b of
`docs/EXPERIMENTS.md`).

### Why the corner is 1 kHz and not 150 Hz

The first design put the corner at 151 Hz and bought −81 dB of carrier
rejection with it. That is the right trade if the filter's own lag does not
matter — and here it nearly does not, because the two sense channels record the
filtered signal and every estimator consumes *that*, not the command.

It was moved for headroom rather than correctness: a 151 Hz corner is only 1.5×
a 100 Hz excitation, which leaves nothing if the sweep is ever pushed up. A
decade of margin costs carrier rejection, and against a 62.5 kHz carrier a
passive ladder cannot have both — the measured trade, with the parts on hand:

| corner | rejection | ripple at the plant | loss at 100 Hz |
| --- | --- | --- | --- |
| 151 Hz (the old design) | −81 dB | 0.12 mV | −1.7 dB |
| 448 Hz | −61 dB | 1.13 mV | −0.2 dB |
| 943 Hz (820 Ω parts) | −57 dB | 1.96 mV | −0.05 dB |
| **773 Hz (1 kΩ parts, this one)** | **−60 dB** | **1.32 mV** | **−0.07 dB** |

### One stage or two

The second stage (the series 1 kΩ and the 100 nF at `u_T`) is easy to leave
out, because the first stage alone already gives the right DC level. It is not
optional. Both builds, simulated in ngspice with the 10 kΩ sense resistor in
place:

```
 1st order:  P1.11 ─[1k]──[1k]──┬────────┬──────────────► u_T
                               [1k]   [100n]
 2nd order:  P1.11 ─[1k]──[1k]──┬────────┬──[1k]───┬────► u_T
                               [1k]   [100n]     [100n]
```

| | 1st order (first stage only) | **2nd order (this design)** |
| --- | --- | --- |
| corner (−3 dB) | 2.4 kHz | **773 Hz** |
| rejection of the 62.5 kHz carrier | −28 dB | **−60 dB** |
| ripple at the plant, 50 % duty | 66 mV_pp | **1.32 mV_pp** |
| ripple at the plant, 25 % duty | 49 mV_pp | **0.99 mV_pp** |
| carrier left in an 8× burst reading, worst case | ±14 counts (±4.2 mV) | **0.4 count (0.12 mV)** |
| half-scale step settled to one count | 0.5 ms | **1.5 ms** |
| filter loss on a 100 Hz sine | −0.008 dB | **−0.07 dB** |
| filter loss on the 900 Hz tick image | −0.6 dB | **−3.7 dB** |
| output impedance | 667 Ω | **1.67 kΩ** |
| full scale at the plant | 1.100 V (4.30 mV/code) | **1.100 V (4.30 mV/code)** |

The first-order build fails on ripple. 66 mV_pp is 7 % of the 0-1 V range, at
62.5 kHz, and the plant sees all of it: if the box samples its input without a
filter of its own, that becomes aliased noise on the `u_T` and `p_s` it reacts
to. Our own readings suffer as well. The burst only cancels the carrier if its
conversions are exactly 12 µs apart, and between 11 and 12.5 µs up to ±14
counts are left over. That residue is not noise. The sampling timer and the
PWM both run from HFCLK, so their relative phase is fixed for a whole session:
the residue shows up as a steady pattern at 500 Hz and as scatter about the
fitted line in `check dc`. It does not average away.

The "carrier left in a burst reading" row is the worst case over start phase
and over a 11-12.5 µs burst spacing, in counts of 293 µV.

A third stage (one more 1 kΩ and 100 nF) puts the corner at 374 Hz, the
rejection at −92 dB and the ripple at 0.035 mV_pp, below one ADC count. With
nothing loading the ladder it keeps the 1.100 V full scale, so what it costs is
speed and drive: a step takes 3.2 ms to settle, a 100 Hz sine loses 0.3 dB, and
the output impedance rises to 2.67 kΩ against a plant input nobody has
measured. Two stages already leave less than a count in the reading, so a third
is not needed.

`check dc` tells the builds apart on the bench. On a plateau, `pp mV` stays
under a millivolt with both stages. With only the first stage it can reach
about 10 mV, twice the ±5 mV residue, because at 1 kHz consecutive rows start
half a carrier period apart and the sign flips. Some duties still read almost
clean, because the residue depends on the duty and the burst spacing. So
look at the whole sweep, not one level.

**The bandwidth limit is now the waveform tick, not the filter.** At a 1 kHz
update rate a 100 Hz sine carries images at 900 and 1100 Hz at about 11 % of the
fundamental, and a 773 Hz corner takes only ~3.7 dB off them. Below ~20 Hz this
does not arise. Above it, the fix is the PWM peripheral's own sequencer
(`SequencePwm`, clocked in hardware) rather than a faster `Ticker`: the RTC1
time driver's 30.5 µs granularity cannot pace a 10 kHz tick.

That granularity already shows at 1 kHz: `Duration::from_hz(1000)` rounds to 33
RTC ticks, so the tick actually runs every 1.0071 ms (993 Hz), not every 1 ms,
and on a different clock from the sampler (TIMER1, off the crystal). Harmless —
the waveforms are evaluated at their true elapsed time, and the tick being
asynchronous is what keeps its images from folding coherently onto an
excitation — but it is why the simulator ticks at 32768/33 Hz while reporting
the nominal 1000.

### VDD: read this before calibrating

Full scale is `VDD × R2/(R1+R2)`, less whatever the plant's input draws. The
Supermini regulates the nRF52840 to **3.3 V**, so full duty is 1.100 V at the
plant and the highest code (255) reaches 1.096 V: the whole 0-1 V window of
`p_s` is in range, and the experiment files guard at `max_v = 0.95` for the
plant's sake rather than the electronics'. `u_T` rides on the same rail twice
over — through the PWM and through its offset divider — so its whole line
scales with VDD too.

**Measure it anyway, and measure it the way you will run it.** Two reasons:

- On a LiPo the rail follows the cell as it drains, and the whole calibration
  line with it. Run from USB for anything you intend to report.
- The regulator's real output and the resistors' real values are both a percent
  or two off nominal. That is a percent or two of every gain in the report, and
  it takes one DMM and two codes to remove.

`docs/EXPERIMENTS.md` §2 has the arithmetic.

## nRF52840 Supermini

- SAADC: internal 0.6 V reference at gain 1/5 → **3.0 V full scale at the
  pin**, which is also the plant, since only a series resistor sits between
  them. 12 bit, 732 µV/LSB. The 2.25-2.75 V window of `u_T` and `P_e` uses
  3072…3755 of 4095 counts, with 0.25 V of room above it for an overshoot;
  gain 1/4 would stop at 2.4 V, inside the window, and 1/6 would reach 3.6 V,
  past the 3.3 V rail the pin cannot exceed anyway. `p_s` (0-1 V) uses a
  third of the range, at 2.5× the count size it had at 1.2 V full scale.
- Inputs: `P0.02`/AIN0, `P0.29`/AIN5, `P0.31`/AIN7 — the only analog pins the
  board brings out, usually silkscreened `A0`/`A1`/`A2` or `D19`/`D20`/`D21`.
- Acquisition time is 10 µs, which Nordic's table allows up to a 40 kΩ source.
  `u_T` presents 11.4 kΩ (the 10 kΩ plus its offset ladder's 1.43 kΩ), `p_s`
  11.7 kΩ (the 10 kΩ plus the filter's 1.67 kΩ);
  `P_e` presents 10 kΩ plus the plant's own output impedance, so it has room up
  to a ~28 kΩ source before `ACQUISITION_TIME` must grow.
- **Scan skew.** The channels of one scan are converted in order — `u_T`,
  `p_s`, `P_e` — and in burst mode each takes its whole 8× burst,
  about 89.5 µs as measured on this board (the datasheet estimate,
  `8 × (10 µs + ~2 µs)`, says 96 µs), before the next one starts. So the three
  values of a CSV row are ~89.5 µs apart (`SCAN_CHANNEL_SPACING_S` in `proto`),
  and a later channel reads as a phase *lead* of `360·f·Δt`: +6.4° of `P_e` against
  `u_T` at 100 Hz, +0.1° at the rotor mode. Negligible for the plant, not for a
  loopback test or a check of the output chain; `plant-trace bode` corrects it,
  and with `P_e` wired to the `u_T` filter output it measures it
  (`docs/EXPERIMENTS.md`, *Automatic Bode*). Raising `ACQUISITION_TIME` or the
  oversampling widens it in proportion.
- Outputs: `P1.11` and `P1.13` (`D14`, `D15`). On a board this small nothing is
  far from anything, but the carrier does not need distance: `C2` is 25 Ω at
  62.5 kHz, so anything that couples into the sense node is shunted straight to
  ground before the ADC sees it.
- Host link: **native USB CDC-ACM** on the nRF52840's own peripheral, which
  means one USB-C cable carries power, flashing and the whole protocol. It
  enumerates as `/dev/ttyACM0`. Full-speed bulk moves well over 100 kB/s
  against the ~8 kB/s this rig produces; the baud rate the host sets is
  decoration, because there is no UART behind it. Nor is there RTS/CTS — USB
  NAKs when an endpoint is not ready, which is flow control the firmware
  cannot get wrong.
- USB needs the 32 MHz crystal, so the firmware asks for
  `HfclkSource::ExternalXtal` at init. The low-frequency clock stays on the
  internal RC, which works whether or not your board has the 32.768 kHz crystal.
- Peripheral budget: `SAADC` + `TIMER1` + `PPI_CH0/1` for acquisition, `USBD`
  for the link, `PWM0` for both outputs. `PWM0` needs no timer or PPI of its
  own — its counter is internal — so nothing contends with anything else, and
  `TIMER2`/`UARTE0` are free for whatever comes next.
- **No debug probe.** There is no J-Link and no SWD header, only pads. So
  `probe-rs` and the `defmt` RTT log are unavailable unless you solder to them;
  `defmt-rtt` is still linked in and costs nothing, because it starts in
  non-blocking mode and only switches to blocking when a probe asks it to.
  `plant-trace info` is the liveness check in its place.

## Input protection

There is none beyond a series resistor, and that is a deliberate choice. Each
of the three inputs is wired the same way:

```
  signal ──[10k]──► P0.02 / P0.29 / P0.31
```

- **It measures exactly.** The pin draws no DC, so the resistor drops nothing:
  the pin sees the plant's voltage, nothing is loaded, and `u_T` and `p_s`
  are read on the very node the plant is fed from. No divider ratio to trust,
  no clamp leakage to bend the top of the range.
- **It protects only by limiting current.** The pin's absolute maximum is
  VDD + 0.3 V. A fault above that forward-biases the chip's own ESD structures
  into VDD, and the 10 kΩ limits the current: a 5 V fault pushes about
  (5 − 3.8) / 10 k ≈ 120 µA. Nordic specifies no injection current for these
  pins, so this is **outside the datasheet**. Two more consequences follow
  from it. With the board unpowered and the plant live, the same path feeds
  the board's rail from the signal. And the rail has to be able to absorb the
  injected current, which at 120 µA against the board's milliamps it does.

That is acceptable here and would not be elsewhere. Every signal on this bench
lives inside the rail in normal use: `p_s` in 0-1 V, `u_T` and `P_e` in
2.25-2.75 V — and the `u_T` offset ladder cannot leave 2.15-2.86 V even at the
extremes of its duty. The failure it guards
against is a miswire. And spare boards are on hand. An earlier revision put a 10 k/10 k
divider and three 1N4148 in front of each pin. The divider kept a 5 V fault
inside the rating on its own. But the diodes leaked enough through the
divider's ~6 kΩ to take ~3 counts off a 1 V reading at 25 °C, and more when
warm. If this rig is ever moved to signals that can exceed the rail, put the
divider back and leave the diodes out.

Reference and gain are per-channel settings on this part (`CH[n].CONFIG` holds
`REFSEL`, `GAIN`, `TACQ` and `BURST`; only `RESOLUTION` and `OVERSAMPLE` are
global), so the three channels *could* be scaled differently — `P_e` given a
range of its own, for instance. They are not: one 3.0 V full scale covers
both the 2.25-2.75 V channels and the 0-1 V one, and one scale is one less
table the host has to keep in step with the firmware.

## Flashing

The board runs the **Adafruit nRF52 bootloader**, which offers both UF2
mass-storage and serial DFU. `tools/flash.sh` tries them in that order and
`cargo run --release` is wired to it:

```sh
cd firmware/nrf-daq
# double-tap RESET first — a drive appears
cargo run --release
```

Two things that matter and fail silently if they are wrong:

- **The application starts at `0x26000`.** That is what the bootloader reserves
  below itself, and it is baked into both `memory.x` and the UF2 header
  (`tools/uf2.py --base`). Flash a UF2 based anywhere else and the board accepts
  it and then simply never enumerates. Verify by reading `INFO_UF2.TXT` on the
  mounted drive.
- **VTOR has to move.** The bootloader jumps to us with the vector table still
  pointing at itself, so `cortex-m-rt` is built with its `set-vtor` feature. The
  symptom of getting this wrong is a board that boots and then dies at the first
  interrupt.

Serial DFU is the scriptable alternative — `adafruit-nrfutil dfu serial` — and
`tools/flash.sh` uses it when the tool is on PATH. It is *not* in the dev shell:
nixpkgs marks it unfree, and pulling it in would force `allowUnfree` on anyone
entering the shell. Install it yourself if you want that path.

With an SWD probe soldered to the pads, swap the runner in
`firmware/nrf-daq/.cargo/config.toml` back to `probe-rs run` — faster, and it
gives you the `defmt` log back.

## Host

NixOS, non-root access to the board:

```nix
# configuration.nix
users.users.<you>.extraGroups = [ "dialout" ];      # /dev/ttyACM*
# only if you solder to the SWD pads:
services.udev.packages = [ pkgs.probe-rs-tools ];
```

`ls /dev/ttyACM*` must show the board — twice over its life, in fact: once as
the bootloader after a double-tap of RESET, and once as the application. They
are different USB devices, and `lsusb` tells them apart by product string
(`plant-trace rig` is ours).

## ESP32-WROOM-32 — kept, not wired

`firmware/esp32-sig/` still builds and still speaks `proto::gen`: DAC1 on
GPIO25, DAC2 on GPIO26, 8 bit, 0-3.3 V into a 3.3:1 divider, commands at
115200 baud over the CP2102. It is a spare, not part of the rig — the host CLI
no longer has a link for it. Bring it back by pointing a second `Framed` at it;
the message set is unchanged.

### One-time Xtensa toolchain install

The ESP32 needs the esp-rs fork of rustc, which is not in nixpkgs. `espup`
fetches it into `~/.rustup` as *generic* Linux binaries — and NixOS has no
`/lib64/ld-linux-x86-64.so.2`, so they cannot run outside an FHS sandbox.
`devShells.esp32` is that sandbox.

```sh
nix develop .#esp32                                    # enter the FHS shell
espup install --targets esp32 --export-file "$PWD/.esp-env.sh"   # ~1.9 GB, once
exit && nix develop .#esp32                            # profile sources it
rustc --version                                        # 1.97.0-nightly (1.97.0.0)
```

`nix develop --command` does **not** work with this shell: the FHS wrapper
replaces the process and drops the command. For scripts use the app form,
which forwards its arguments to the sandboxed bash:

```sh
nix run .#esp32 -- -c 'cd firmware/esp32-sig && cargo build --release'
```

The Xtensa target has no prebuilt `core`, so `.cargo/config.toml` asks for
`build-std = ["core", "alloc"]`; that is why the first build is slow.
`cargo run --release` is `espflash flash --monitor`.
