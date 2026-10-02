# plant-trace

Bench rig that characterises the *Single Machine Infinite Bus* steam-turbine
plant of the Controle Clássico assignment: it drives the plant's two analog
inputs, records three analog signals on a common time base, and turns the
recordings into the static curves, step metrics and Bode points a report is
written from.

```
     nRF52840 Supermini                                                   magic box
                        RC, 1 kHz                                       ┌──────────┐
  P1.11 PWM 62.5 kHz ──[shunt to 2.74 V]──► 2.150 V + 0.709 V·duty ─┬──►│ u_T      │
  P1.13 PWM 62.5 kHz ──[shunt to GND, ÷3]──► 0-1 V ─────────────┬───┼──►│ p_s      │── P_e ──► 10k → P0.31
                                                                │   │   └──────────┘
                                                                │   └──► 10k → P0.02     u_T, P_e: 2.25-2.75 V
                                                                └──────► 10k → P0.29     p_s:      0-1 V
```

One board drives and measures, over one USB-C cable. Both plant inputs are
wired back into the SAADC at the node the plant is fed from, so every CSV row is one `(u_T, p_s, P_e)` triple
from a single scan and identification never has to trust a commanded value or
align two clocks. (A scan converts the channels one after another, ~89.5 µs
apart — a known, correctable skew; see `docs/PROTOCOL.md`.)

## Eight lines from nothing to a result

```sh
nix develop                                            # host + nRF toolchain
cd firmware/nrf-daq && cargo run --release && cd -     # double-tap RESET first

cargo build --release
./target/release/plant-trace info --daq /dev/ttyACM0
./target/release/plant-trace check --daq /dev/ttyACM0 dc          # the rig measures its own outputs
./target/release/plant-trace run experiments/step-u.toml --daq /dev/ttyACM0
./target/release/plant-trace analyze data/step-u-<timestamp>
./target/release/plant-trace bode experiments/bode-u.toml --daq /dev/ttyACM0  # automatic Bode
./target/release/plant-trace gui                                           # everything above, with plots
```

No hardware? The whole pipeline runs against a simulated rig, which models the
8-bit duty quantisation, the 1 kHz tick the outputs are updated on, and both
poles of the output filter:

```sh
./target/release/plant-trace simulate --speed 60 &
./target/release/plant-trace run experiments/step-u.toml \
    --daq tcp://127.0.0.1:7801 --out-dir data/demo
./target/release/plant-trace analyze data/demo
```

## What each part does

| path | what |
| --- | --- |
| `proto/` | wire format, scaling and the waveform engine, shared by every node (`no_std`) |
| `host/plant-trace/` | CLI: `info`, `record`, `gen`, `check`, `run`, `analyze`, `bode`, `simulate`, and `gui` — a desktop GUI over the same library (live view, output checks, scenario editor, automatic Bode, results), see `docs/GUI.md` |
| `host/plant-model/` | reference plant — drives the simulator, and is the ground truth the estimators are tested against |
| `firmware/nrf-daq/` | nRF52840, the whole rig: 3-channel SAADC reading to 3.0 V at the plant via timer+PPI, two 62.5 kHz PWM outputs, all over native USB CDC |
| `firmware/esp32-sig/` | ESP32: 1 kHz waveform tick onto DAC1/DAC2. Kept as a spare, not wired into the rig |
| `sim/rc_filter/`, `ngspice_filter.cir` | SPICE characterisation of the PWM reconstruction filter — 1 kΩ and 100 nF throughout (`nix run .#rc_filter`) |
| `tools/` | `uf2.py` and `flash.sh` — nixpkgs has no nRF UF2 converter, so this is it |
| `experiments/` | the seven measurements the assignment asks for, as TOML, plus the `bode-*.toml` plans for `plant-trace bode` |
| `docs/` | plan, hardware notes, wire protocol, bench procedure |

## Where to read next

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — wiring, the PWM DAC, flashing, the
  toolchains. **Read before connecting anything** — in particular the flashing
  section: the application base address and `set-vtor` are the two things that
  fail silently.
- [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) — the bench procedure, and how to
  read the results critically.
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) — the wire format.
- [`docs/PLAN.md`](docs/PLAN.md) — the design, the phase log, and what is still
  unverified against real hardware.

## Tests

```sh
cargo test --workspace
```

They run without any hardware: the protocol round-trips, the waveform engine
and every estimator are checked against analytically known answers, and the
acquisition path, the experiment runner and the analysis are exercised
end-to-end through the simulator.
