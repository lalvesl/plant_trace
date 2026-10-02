# Wire protocol

One link: host↔rig, over the nRF52840's native USB CDC-ACM. It carries both
halves of the firmware — the sample stream and the commands that drive the
plant's two inputs. The definitions live in the `plant-trace-proto` crate and
are compiled into every node, so there is exactly one description of the wire
and it is executable.

Framing does not care that the transport is USB: bulk packets are a byte stream
like a UART's, and the COBS delimiter is what finds the boundaries either way.
What USB does change is that there is no baud rate to get wrong and no RTS/CTS
to wire — an endpoint that is not ready NAKs, and the sender waits.

`PROTOCOL_VERSION` is **2**. v1 had the generator on a link of its own, spoken
by the ESP32; v2 nests those messages inside `HostToDaq` so one board owns the
whole rig. The archived ESP32 firmware still speaks them bare — the message set
is identical, which is why it is defined once.

## Framing

```
COBS( postcard(message) ++ CRC16-LE ) ++ 0x00
```

- **postcard** — compact, schema-driven, `no_std`. Enum variants are indexed by
  declaration order, so *appending* a variant is compatible and *reordering* one
  is not; `PROTOCOL_VERSION` exists to catch the latter.
- **CRC-16/IBM-SDLC** over the postcard body. postcard would reject most
  corruption on its own, but a flipped bit inside a block of samples decodes
  into a perfectly plausible wrong number — which is exactly the kind of error
  that ends up in a report.
- **COBS** removes every `0x00` from the body, so a zero byte means "frame
  boundary" and nothing else. A receiver that starts mid-stream, or loses bytes,
  is synchronised again by the next delimiter.
- `MAX_FRAME = 1024` bytes. The largest message is a full sample block:
  `3 channels × 64 samples × 2 B = 384 B` of payload.

`frame::Decoder` implements the receive side byte by byte; `frame::encode`
the transmit side. Both are covered by round-trip, corruption and
resynchronisation tests in `proto/src/frame.rs`.

## Acquisition

| Host → rig | meaning |
| --- | --- |
| `Ping` | liveness |
| `Info` | ask for the front-end description |
| `Start { fs_hz, block_samples }` | begin streaming; either field may be 0 for the firmware default |
| `Stop` | stop and report counters |
| `Calibrate` | run the SAADC offset calibration (idle only) |
| `Gen(HostToGen)` | anything in the table below |

| Rig → host | meaning |
| --- | --- |
| `Pong` | |
| `Info(DaqInfo)` | protocol, firmware, channels, bits, full scale, oversampling, block size, max rate |
| `Started { fs_hz }` | the *effective* rate after rounding to the 1 MHz timer |
| `Block(SampleBlock)` | `seq`, `n`, `channels`, `dropped`, and `n × channels` little-endian `i16` counts |
| `Stopped { blocks, dropped }` | totals for the run |
| `Calibrated` | |
| `Error(DaqError)` | `Busy`, `NotRunning`, `BadConfig` |
| `Gen(GenToHost)` | answer to a `Gen` command |

**There are no timestamps on the wire.** Sampling is driven by a hardware timer
through PPI, so sample `i` of block `seq` was taken at
`t = (seq × n + i) / fs_hz` exactly. A stall shows up as a gap in `seq` and in
the `dropped` counter, not as drifting timestamps — and since the two filter
outputs are recorded on the same three-channel scan as the plant's output,
nothing downstream needs a common clock with the waveform tick. The tick is free
to be asynchronous to the sampler for exactly that reason.

**The three values of a row are not simultaneous.** The timer triggers one
*scan*; the SAADC then converts the channels in wire order, each as a burst of
eight `10 µs + ~2 µs` conversions averaged into one value, so each channel's
effective instant is its burst's centre and consecutive channels are
`SCAN_CHANNEL_SPACING_S` ≈ 89.5 µs apart, as measured on the bench
(`proto/src/lib.rs`): `p_s` is taken ~89.5 µs and `P_e` ~179 µs after the `u_T` of the same row. The wire format does
not say so and does not change for it; `t` above is the time of channel 0. A
channel converted later but filed on the same row reads as a **lead** of
`360·f·Δt` degrees — +6.4° of `P_e` against `u_T` at 100 Hz, +0.1° at the
1.4 Hz rotor mode. `plant-trace bode` removes it (and can measure it, on a
wire loopback); `check sine` reports it next to the skew it measures between
the two sense channels. The simulator models it (`sim::Options::scan_spacing_s`).

`block_samples` is fixed by the DMA buffers at compile time. `Start` accepts 0
or that exact value and refuses anything else, because honouring a smaller one
would mean discarding the rest of every buffer.

## Generation

| Host → rig, inside `Gen(…)` | meaning |
| --- | --- |
| `Ping`, `Info`, `Status` | |
| `SetScale { ch, scale }` | install the bench-measured volts↔code map and the safe window |
| `SetLevel { ch, volts }` | drive an output now, cancelling its waveform |
| `Program { ch, wave }` | stage a waveform without starting it |
| `Start` | start every staged waveform on the same tick |
| `Stop` | freeze the outputs |
| `Park` | drive both outputs to their configured minimum |

Levels are always in **volts at the plant input**. The host owns calibration
(it is a property of the divider on the bench, not of the firmware) and the
device owns timing (it is a property of the 1 kHz tick, not of the link).
`Program` + `Start` exist so a two-channel excitation starts on one tick with no
skew between channels — the firmware writes both duties in a single PWM DMA
transfer, so not even a carrier period separates them.

**These are answered while acquisition is running.** An excitation starts in the
middle of a recording, not between two of them, so the firmware handles them in
its receive task rather than queueing them for the main loop — which is inside
the sampler for the whole length of a record step. On the host side the mirror
of that rule is that blocks arriving while a command waits for its reply are
*queued*, never discarded: the leading edge of a step is the last thing a
recording can afford to lose.

The waveforms — `Hold`, `Ramp`, `Staircase`, `Sine`, `Chirp`, `Prbs` — are
evaluated by `proto::waveform::Generator`, the same code on the device and on
the host. That matters for the PRBS: identification has to correlate against
the exact bit sequence that was applied, and a second implementation would
eventually disagree with the first.
