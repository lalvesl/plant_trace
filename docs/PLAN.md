# plant-trace — implementation plan

Frequency/time-domain plant characterisation rig for the *Controle Clássico*
assignment (SMIB steam-turbine "magic box"): drive two analog inputs, record
three analog signals, produce CSVs that a report can be written from.

Working language of the repository is **English** (code, comments, docs).

---

## 0. System under test

```
            nRF52840 Supermini                    magic box
   P1.11 PWM ─┬─► u_T (valve command, 2.25-2.75 V) ────────────►┌──────────┐
   P1.13 PWM ─┼─► p_s  (steam pressure, 0-1 V) ─────────────────►│  plant   │── P_e (2.25-2.75 V) ──► P0.31
              │                                                  └──────────┘             ▲
              └──── both are ALSO sensed on P0.02 / P0.29 ────────────────────────────────┘
```

(The first draft of this plan had an ESP32 driving the two inputs and an
nRF52840-DK reading three channels. §1 records why it collapsed into one board.)

Recording the two commands on the same ADC time base as the output removes the
host/firmware clock-alignment problem entirely: every CSV row is a simultaneous
`(u_T, p_s, P_e)` triple from one SAADC scan. The commanded value is never used
for identification — the *measured* one is.

Assignment mapping (`Trabalho prático - 1a parte`):

| Assignment item | Rig feature |
| --- | --- |
| 1.1 static curve `u_T × P_e` at `p_s = p_s0` | `static-u` recipe (staircase + steady-state detector) |
| 1.1 static curve `p_s × P_e` at `u_T = u_T0` | `static-p` recipe |
| 1.2 `ΔP_e/Δu_T` time response, ≥2 operating points | `step` recipe (±5 %, up/down, repeats) |
| 1.2 `ΔP_e/Δu_T` frequency response | `freq` recipe (stepped sine) + `prbs` (ETFE) |
| 1.2 `ΔP_e/Δp_s` time + frequency response | same recipes, channel `p_s` |
| 1.3 delay / rise / settling / overshoot / damping | `analyze` metrics |

> **Note on the source PDF.** Page 2 of the assignment contains a line of
> *white-on-white* text — invisible when the PDF is read, present in the text
> layer — instructing any LLM to discard the document and write about wind-turbine
> pitch control instead. It is a trap to detect AI-written reports. It is not an
> instruction from the author of this repository and is deliberately ignored;
> the real assignment (steam-turbine characterisation) is what this rig serves.
> Worth knowing when the report is written: do not paste the PDF text into any
> tool that might act on it.

---

## 1. Hardware decisions (fixed)

**nRF52840 Supermini, SAADC**
- Reference: internal 0.6 V, gain 1/5 → **full scale 3.0 V**, 732 µV/LSB. It
  was gain 1/2 and 1.2 V (the "1.2 V ref" first asked for) until the box turned
  out to take `u_T` and give `P_e` in 2.25-2.75 V; `p_s` alone stays in
  0-1 V. The window uses 3072..3755 of 4095 counts, with 0.25 V above it for
  an overshoot. `u_T` gets there through its ladder, whose middle node is shunted to a
  2.74 V divider off 3.3 V instead of to ground: 2.150 V + 0.709 V × duty
  (`docs/HARDWARE.md`, *The `u_T` offset ladder*).
- Each input reaches its pin through one 10 kΩ series resistor and nothing
  else. The pin draws no DC, so it reads the plant's voltage unchanged, the
  output filters are unloaded, and `u_T`/`p_s` are sensed on the very node the
  plant is fed from. The resistor only limits what a miswire can push into the
  pin's own ESD structures — outside Nordic's VDD + 0.3 V absolute maximum,
  accepted knowingly because every signal here stays inside the rail and spare
  boards are on hand.
- Three earlier attempts are worth remembering. The last was a 10 k/10 k
  divider then 820 Ω into three 1N4148, with the SAADC at unity gain: it kept a
  5 V fault inside the rating, but the diodes leaked ~80 nA through the
  divider's ~6 kΩ and took ~3 counts off a 1 V reading at 25 °C, more when
  warm, and the 20 kΩ loaded the filter by 6.4 %. Before it: An 820/1.2 k divider with unity
  gain left only 1 % of headroom above 1 V, which a step's overshoot would have
  clipped into looking well-damped. Clamping with two diodes and no divider
  looked cheaper but has a soft knee — silicon conducts a decade per ~95 mV, so
  the pair drew tens of microamps by 0.8 V and compressed the top of the command
  range. Reference and gain are per-channel registers on this part, so the
  channels could differ; one 3.0 V scale covers all three, so they do not.
- 12-bit, `BURST` + oversampling ×16 per channel (noise floor), acquisition time
  20 µs (the plant's output impedance is unknown, so be generous).
- 3 channels in scan mode, sampled by `TIMER` → `PPI` → `SAADC.SAMPLE`, EasyDMA
  double buffering. Default `fs = 1 kHz` (configurable); the plant's fastest
  mode of interest is the ~1-2 Hz electromechanical oscillation, so 1 kHz is
  ~500× oversampled and decimation is a host-side choice.
- `CALIBRATEOFFSET` at boot, plus a documented two-point gain/offset calibration
  stored in the CSV metadata header.
- Pins: `AIN0 = P0.02`, `AIN5 = P0.29`, `AIN7 = P0.31` — the board's `A0`/`A1`/
  `A2`, and the only analog-capable pins it brings out. Exactly three, which is
  exactly what the rig needs and leaves nothing spare.

**nRF52840 Supermini, output — revised 2026-09-22**

Originally the ESP32's two DACs drove the plant. They no longer do: **both
outputs moved onto the nRF's PWM**, so one board owns the whole rig.

- `P1.11`/`D14` (u_T) and `P1.13`/`D15` (p_s), `PWM0`, prescaler ÷1 and
  `COUNTERTOP = 256`
  → **62.5 kHz carrier with an 8-bit duty**. A duty *is* an output code, which is
  why `DacScale` did not have to change shape.
- Reconstruction by the RC network in `ngspice_filter.cir`, built entirely from
  1 kΩ and 100 nF: a 1:3 divider with 100 nF, then 1 kΩ + 100 nF, unloaded.
  Two real poles (796 Hz and 4775 Hz), −60.3 dB of carrier rejection, 1.32 mV
  of ripple at the plant. (It was 820 Ω; the first bench build used 8.2 kΩ by
  mistake, which the loopback Bode caught as poles at exactly a tenth of the
  design — `docs/HARDWARE.md`, *What the bench found*.) Characterised in `sim_results/report.md`.
- The corner was 151 Hz at first, which bought −81 dB. It moved to 1 kHz to put
  a decade between it and a possible 100 Hz excitation, and to drop the output
  impedance from 10.7 kΩ to under 2 kΩ. Against a 62.5 kHz carrier a passive
  ladder cannot have both; the SAADC's 8× burst is what brings the extra ripple
  back under a count. **The bandwidth limit is now the 1 kHz waveform tick**, whose
  reconstruction images a 1 kHz corner barely touches — the fix for that is
  `SequencePwm` clocked in hardware, not a faster `Ticker`, because the RTC1
  driver's 30.5 µs granularity cannot pace one.
- VDD is 3.3 V, which is what the ngspice deck assumes: full duty is 1.100 V,
  code 255 reaches 1.096 V, and `p_s`'s whole 0-1 V window is in range; `u_T`
  takes an offset copy of the same ladder to 2.150-2.857 V.
  (The first pass at this targeted an nRF52840-DK, whose default rail is 3.0 V
  and whose ceiling was therefore 0.934 V — the outputs are rail-referenced, so
  the board's rail is not a detail.)
- Why this is better than the ESP32 pair, beyond one less board: the two duties
  are written in a single PWM DMA transfer, so a two-channel excitation has no
  skew at all; and the generator's clock no longer has to be reasoned about
  relative to the sampler's, because both live in the same firmware.
- What it costs: the output is a filtered square wave, so a commanded edge is a
  1.07 ms exponential rather than a step. This is invisible to the
  identification — the two sense channels record the filtered signal, and that
  is what every estimator consumes.
- The `PWM0` peripheral has its own counter, so it contends with neither
  `TIMER1`/`PPI_CH0-1` (sampler) nor the USB peripheral.

**ESP32-WROOM-32 — kept as a spare**
- `DAC1 = GPIO25` (u_T), `DAC2 = GPIO26` (p_s), 8-bit over 0-3.3 V into a 3.3:1
  divider. `firmware/esp32-sig/` still builds and still speaks `proto::gen`
  unchanged; the host simply has no link for it any more.
- Toolchain: Xtensa needs the esp-rs Rust fork. Pure-Nix path attempted first;
  `espup`-based shell is the documented fallback (see phase 4). `devShells.esp32`
  and `apps.esp32` stay.

**Host link (decided, revised 2026-09-22).** One USB-C cable, carrying power,
flashing and the protocol.

- **Native USB CDC-ACM** on the nRF52840's own peripheral. The board has no
  J-Link and no USB-serial bridge, so the VCOM UART the first draft specified
  has nothing to connect to; native USB was already listed here as the fallback
  for if that link ever bottlenecked, and became the only option instead.
- Budget: 1 kHz × 3 ch × 2 B ≈ 6 kB/s payload, ≈ 8 kB/s framed, against
  full-speed bulk moving well over 100 kB/s. There is no baud rate and no
  RTS/CTS — an endpoint that is not ready NAKs.
- It carries both halves: generator commands are nested inside `HostToDaq`,
  answered by the receive task so an excitation can start in the middle of a
  recording.
- Cost: no `defmt-rtt` log and no `probe-rs`, because the debug probe went with
  the DK. Flashing is UF2 or serial DFU through the Adafruit bootloader.

Frames: `postcard` + COBS + CRC-16, shared `proto` crate.

---

## 2. Repository layout

```
plant-trace/
├── flake.nix, nix/            # pkgs, rust (host+thumbv7em), embedded tools, xtensa
├── rust-toolchain.toml        # stable + thumbv7em-none-eabihf + rust-src
├── proto/                     # no_std, shared wire types (serde/postcard)
├── host/                      # std workspace
│   ├── plant-trace/           #   CLI: record | run | analyze | simulate | cal
│   └── plant-model/           #   reference plant model (simulator + analysis tests)
├── firmware/
│   ├── nrf-daq/               # embassy-nrf, SAADC + USB CDC streamer
│   └── esp32-sig/             # esp-hal, DAC/LEDC waveform generator
├── experiments/               # TOML experiment descriptions (checked in)
├── data/                      # CSV output (gitignored)
└── docs/                      # PLAN.md, HARDWARE.md, EXPERIMENTS.md, PROTOCOL.md
```

Three cargo workspaces (host, nrf-daq, esp32-sig) because each needs its own
`.cargo/config.toml` target; `proto` is a path dependency of all three.

---

## 3. Phases

Each phase is one work session and ends with something that builds and is
verifiable without the magic box attached. Tick the boxes and update
**Current state** at the end of every session.

### Phase 1 — Nix + skeleton
- [x] Rewrite `flake.nix` for plant-trace (drop the WaveDB/wasm/bench outputs).
- [x] `nix/pkgs.nix` (no unfree predicate), `nix/rust.nix` (keep), new
      `nix/embedded.nix` (probe-rs-tools, espflash, espup, ldproxy, flip-link,
      cargo-binutils, picocom, libudev/pkg-config for the host serial crate).
- [x] `rust-toolchain.toml`: stable, `thumbv7em-none-eabihf`, `rust-src`,
      `llvm-tools`, `clippy`, `rustfmt`.
- [x] Workspace skeletons + `.gitignore` + `README.md` + `docs/HARDWARE.md`
      (pin map, divider, zener notes, DK jumper/VDD notes).
- [x] `nix flake check`-clean, `cargo check` on host, `cargo build` on a
      blinky-level nrf-daq.
- [x] `nix run .#fmt` (nixfmt + cargo fmt + taplo).

### Phase 2 — `proto` + nRF DAQ firmware
- [x] `proto`: `HostCmd` / `DevMsg` enums, `SampleBlock { seq, t0_us, fs_hz, [u16; 3×N] }`,
      device info, calibration record, COBS framing + CRC, `no_std` + `std` feature.
- [x] Round-trip unit tests for the codec (run on host).
- [x] embassy-nrf: HFCLK, SAADC 3-ch scan, TIMER+PPI, double-buffered EasyDMA,
      offset calibration, overrun counter.
- [x] Host link: command handling (`Start`, `Stop`, `Info`, `Calibrate`), sample
      streaming, backpressure that drops whole *blocks* (counted) rather than
      corrupting the stream. Built on UARTE over the DK's VCOM first; **rebuilt
      on native USB CDC-ACM** once the board turned out to be a Supermini, which
      resolved the deferred item below by removing the alternative.
- [x] `defmt-rtt` + `panic-probe` wired in. `cargo run` is `tools/flash.sh`
      (UF2, falling back to serial DFU); `probe-rs run` is one line away in
      `.cargo/config.toml` for anyone who solders to the SWD pads.
- [ ] **Bench**: the board takes the UF2 and comes back as `/dev/ttyACM0`.
- [ ] **Bench**: feed a known DC voltage and confirm the counts match the
      0.6 V/gain-½ transfer function (1.000 V should read ≈ 3413).

### Phase 3 — Host acquisition + CSV + simulator
- [x] `plant-trace record`: open the CDC port, decode, write CSV
      (`t_s,u_T_counts,p_s_counts,Pe_counts,u_T_V,p_s_V,Pe_V`) with a metadata
      header (fs, ref/gain, calibration, firmware version, UTC start, git hash).
- [x] Live TUI-less status line: rate, dropped blocks, per-channel min/mean/max.
- [x] `plant-trace simulate`: a fake device pair (DAQ + generator) driven by
      `plant-model` — valve rate limit + transport delay + non-linear flow
      characteristic × HP/IP/LP cascade × the swing equation + noise.
      **Everything downstream is developed and tested against this.**
      Served over TCP rather than a PTY: same protocol, same code path, no
      extra dependency, and the link spec is the only thing that differs.
- [x] End-to-end test: `simulate` → `record` → CSV with plausible numbers.

### Phase 4 — ESP32 signal generator
- [x] Nix: `devShells.esp32` is an **FHS sandbox** (`buildFHSEnv`) around the
      `espup`-installed toolchain — a pure derivation is impossible and plain
      `mkShell` cannot run generic Linux binaries on NixOS. `nix run .#esp32 --
      -c '…'` is the scriptable form; exact commands in `docs/HARDWARE.md`.
- [x] esp-hal firmware: 1 kHz waveform tick, two independent channels, affine
      volt↔code map, clamps, and generators: `hold`, `ramp`, `staircase`,
      `sine`, `chirp` (log sweep), `prbs`. Builds for `xtensa-esp32-none-elf`:
      49 601 B text.
- [x] Same `proto` command set over UART0 @115200, every command acknowledged.
- [ ] Dropped: the link-loss watchdog. An experiment legitimately runs for
      minutes with no traffic, so a timeout would have to be longer than any
      run — and an output frozen at a valid operating point is benign. `Park`
      remains as the explicit way to bring both outputs down.
- [ ] **Bench**: flash it and confirm with a DMM that the DAC codes land where
      the calibration says (two-point check at 0.2 V and 0.8 V).
- [x] Waveform engine unit-tested on the host (phase 2: it is plain `no_std`
      math, and the host replays it during identification).

### Phase 5 — Experiment orchestration
- [x] `experiments/*.toml` schema: operating point, settle criteria, per-channel
      sequence, sample rate, output naming.
- [x] `plant-trace run <experiment.toml>`: drives the generator, records the DAQ,
      emits one CSV per segment **plus** `run.json` (schedule, markers, applied
      setpoints, timestamps) so the report can label every plot.
- [x] Steady-state detector (slope + std over a window) gates every point.
- [x] Seven checked-in experiments rather than built-in recipes: `static-u`,
      `static-p`, `step-u`, `step-p`, `freq-u`, `freq-p`, `prbs-u`, with the
      assignment's defaults (±5 % steps, two operating points, log-spaced
      frequencies, `p_s0` nominal). A file the report can cite beats a flag
      combination nobody can reconstruct.
- [x] Dry-run the whole set against the simulator.

### Phase 6 — Analysis
- [x] `plant-trace analyze static <csv…>` → gain curve, incremental gain
      `dP_e/du_T` per segment (the valve nonlinearity evidence).
- [x] `analyze step` → apparent delay, rise time, settling time, overshoot,
      oscillation period + damping (log decrement), FOPDT and 2nd-order fits.
- [x] `analyze freq` → per-frequency sine fit (least squares at the excitation
      frequency) → |G|, ∠G, coherence-ish quality metric → Bode CSV.
- [x] `analyze prbs` → Welch/ETFE estimate as a cross-check.
- [x] gnuplot scripts driven by the CLI (`--plot`), producing report figures
      directly: ≥7 pt fonts, distinct line styles *and* dashes (not colour alone),
      labelled axes with units, comparable scales across runs — the formatting
      rules in the assignment multiply the final grade.
- [x] Validated against `plant-model` with known parameters (the fits must
      recover them).

### Phase 7 — Tests, docs, hand-off
- [x] `cargo nextest` green; clippy clean; `nix run .#fmt` applied.
- [x] `README.md`: from `nix develop` to a finished CSV in ten lines.
- [x] `docs/EXPERIMENTS.md`: bench procedure, wiring check, what to do first when
      the hardware is on the table.
- [x] `docs/PROTOCOL.md`: wire format, so the firmware can be reflashed by anyone.
- [x] Known-limitations list (DAC resolution, valve rate limit, ADC noise floor)
      — feeds assignment item 1.3.

---

## 4. Current state

**Outputs moved to the nRF's PWM, and the board turned out to be a Supermini —
2026-09-22.** All seven phases were complete as of 2026-09-19. This revision
folds the signal generator into the DAQ firmware, retires the ESP32 from the rig
without removing it from the tree, and then re-targets everything from the
nRF52840-DK to an **nRF52840 Supermini** (nice!nano v2 pin map, Adafruit
bootloader) — which took the J-Link with it and so replaced the VCOM UART with
native USB CDC.

The rig is written, it builds for all three targets, and everything that can be
verified without the boards has been verified.

### What exists

| part | state |
| --- | --- |
| `flake.nix` | `nix flake check` passes; host+nRF shell, FHS shell for Xtensa, `packages.plant-trace`, `apps.fmt`, `apps.esp32`, `apps.rc_filter` |
| `proto` | framing, the nested message set, scaling, waveform engine — 13 tests |
| `firmware/nrf-daq` | the whole rig: 3-ch SAADC + two 62.5 kHz PWM outputs over native USB CDC. 45 264 B text / 9 076 B bss for thumbv7em |
| `tools/` | `uf2.py` + `flash.sh` — the bin→UF2 converter nixpkgs does not have, and the runner that uses it |
| `firmware/esp32-sig` | still builds for xtensa-esp32-none-elf; a spare, not wired |
| `host/plant-trace` | `info`, `record`, `gen`, `run`, `analyze`, `simulate` — one link for all of it |
| `host/plant-model` | reference plant — 4 tests against analytic answers |
| `sim/rc_filter` | SPICE sweep of the reconstruction filter, `sim_results/report.md` |
| `experiments/` | the seven measurements §1.1 and §1.2 ask for |
| `docs/` | PLAN, HARDWARE, PROTOCOL, EXPERIMENTS |

**38 tests pass, clippy is clean on all three crates, `nix run .#fmt` applied.**

The simulator now models the output filter's two poles, so a "step" in a
simulated recording rises over 5 ms exactly as it will on the bench, and the
estimators are exercised against that rather than against an ideal edge.

### What has never met hardware

Nothing in this repository has been flashed or measured. Every item below is a
first-time step, and `docs/EXPERIMENTS.md` walks through them in order:

1. **The application starts at `0x26000`.** That is what the Adafruit
   bootloader reserves, and it is in both `memory.x` and the UF2 header. Read
   `INFO_UF2.TXT` on the mounted drive and confirm it before flashing. A wrong
   base is accepted silently and then the board never enumerates.
2. `cargo run --release` converts and copies; the board comes back as
   `/dev/ttyACM0` with product string `plant-trace rig`, and
   `plant-trace info --daq /dev/ttyACM0` answers. There is no debug log on this
   board, so that answer is the only "it booted" signal there is. Silence points
   at the base address or at `set-vtor`.
3. A known DC voltage reads the right count (2.500 V ≈ 3413 at 3.0 V full
   scale); any error goes into `gain_correction`, not into the firmware.
4. **PWM polarity.** `DutyCycle::normal(code)` is taken to mean duty =
   `code/256`, which is what embassy's own "start at duty 0, idle low" default
   implies — but the doc comment on the constructor describes the compare
   polarity the other way round. If commanding 0 V produces full scale, swap
   `normal` for `inverted` in the `waveform` task; nothing else changes.
5. Two-point DMM calibration of each output, into the `[outputs.*]` block of
   every experiment file. The checked-in values are nominal for VDD = 3.3 V
   through 2.2 k/1.0 k, not measured. Do it on USB power.
6. A 30 s recording with all three channels wired, watching `dropped` stay at
   zero, and a `gen … level` change visible on the sense channel within ~10 ms.

### Things found along the way worth keeping in mind

- The assignment PDF carries a white-on-white instruction aimed at LLMs
  (§0 of this plan). Do not paste its text into a tool that might act on it.
- The apparent delay of this plant is *not* its transport delay: the actuator
  lag and the steam chest are inside it. The tests assert the relationship
  rather than a single number.
- A ±5 % step barely excites the electromechanical mode. The damping estimate
  is a lower bound and the analysis refuses to report one when the ringing does
  not decay convincingly; the sine point nearest 1.4 Hz is the better route.
- Above ~2 Hz a 0.05 V excitation produces tens of microvolts of output. The
  Bode table's `residual_ratio` marks those points; they should be reported as
  unreliable rather than drawn as a line.
- PWM reconstruction is *exactly* linear in the duty — averaging is, and so is
  the divider — so even the GPIO's on-resistance only moves the endpoints of the
  line. That is the whole reason a two-point calibration is sufficient, and why
  `offset_v` exists alongside `volts_per_code`.
- The outputs are rail-referenced: full scale *is* VDD × the divider ratio. On
  a board that can run from a battery, that makes the calibration a function of
  the battery's state of charge. Measure on USB, report on USB.
- Losing the DK cost more than a pin map. It took the J-Link with it, and with
  it the VCOM link, `probe-rs`, and the `defmt` log — which is why the link is
  native USB, the flashing is UF2, and the only evidence the firmware is alive
  is that it answers.
