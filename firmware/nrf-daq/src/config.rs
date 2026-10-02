//! Everything about this particular bench, in one place.
//!
//! Changing a rate or the front-end scaling is a change here and nowhere else;
//! the pins are `Peri` values and so live at the top of `main`, each with the
//! board's silkscreen label next to it. `docs/HARDWARE.md` describes the wiring
//! these constants assume.
//!
//! The board is an **nRF52840 Supermini** with the nice!nano v2 pin map:
//!
//! | use | GPIO | silk |
//! | --- | --- | --- |
//! | `u_T` PWM | `P1.11` | `D14` |
//! | `p_s` PWM | `P1.13` | `D15` |
//! | `u_T` sense (offset ladder output) | `P0.02` / AIN0 | `D19`, `A0` |
//! | `p_s` sense | `P0.29` / AIN5 | `D20`, `A1` |
//! | `P_e` sense | `P0.31` / AIN7 | `D21`, `A2` |
//!
//! Those three are the only analog-capable pins the board brings out, which is
//! exactly as many as the rig needs. Silkscreen numbering varies between
//! vendors — trust the GPIO column.

use embassy_nrf::gpio::OutputDrive;
use embassy_nrf::pwm::Prescaler;
use embassy_nrf::saadc::{Gain, Oversample, Reference, Resolution, Time};

/// Reported to the host and written into the CSV metadata header.
pub const FIRMWARE_VERSION: &str = concat!("nrf-daq ", env!("CARGO_PKG_VERSION"));

// ── analog front end ────────────────────────────────────────────────────────

/// Internal 0.6 V reference at gain 1/5 puts full scale **at the pin** at
/// 3.0 V. Every channel reaches its pin through one 10 kΩ series resistor and
/// nothing else, so the pin sees the plant's voltage unchanged.
///
/// 3.0 V, not 1.2 V, because `u_T` and `P_e` now live in the plant's
/// 2.25-2.75 V window, and `u_T` is sensed at the output of the offset
/// ladder that lifts the filtered PWM into it
/// (`DacScale::U_T_NOMINAL`): gain 1/4 would stop at 2.4 V, under the window,
/// and 1/6 would reach 3.6 V, past the 3.3 V rail the pin cannot exceed
/// anyway. One count is 732 µV on all three channels; `p_s` and `P_e`, which
/// still live in 0-1 V, give up 2.5× of resolution for it and gain the room
/// to read a `P_e` that leaves its range instead of clipping at 1.2 V.
///
/// The internal reference, not `VDD/4`, because `P_e` comes from the plant and
/// has nothing to do with this board's rail.
///
/// There is no divider and no external clamp on the sense side of `u_T`: the
/// offset ladder cannot leave 2.15-2.86 V whatever the duty, inside both the
/// full scale and the rail. The 10 kΩ in front of every pin only limits what
/// a miswire can push into the pin's ESD structures — outside Nordic's
/// VDD + 0.3 V absolute maximum, chosen knowingly because spare boards are on
/// hand. `docs/HARDWARE.md`, *Input protection*.
///
/// Both settings are per-channel on this part (`CH[n].CONFIG` holds `REFSEL`,
/// `GAIN`, `TACQ` and `BURST`; only `RESOLUTION` and `OVERSAMPLE` are global),
/// so the three channels *could* differ. They do not, so that one scale
/// describes all three and the host needs no per-channel table.
pub const REFERENCE: Reference = Reference::Internal;
/// See [`REFERENCE`].
pub const GAIN: Gain = Gain::Gain1_5;
/// Input voltage at full scale, in millivolts — the host needs it to turn
/// counts into volts. This is the voltage **at the pin**, which with only a
/// series resistor in front of it is also the voltage at the plant. Must
/// match [`GAIN`]: `600 mV × 5`.
pub const FULL_SCALE_MV: u16 = 3000;
/// 12 bit is the highest resolution the SAADC offers without oversampling
/// tricks; one count is 732 µV, at the pin and at the plant alike. Global,
/// unlike the reference and gain above.
pub const RESOLUTION: Resolution = Resolution::_12bit;
/// 8× hardware oversampling (in burst mode, so it costs time and not sample
/// rate) trades 288 µs of the sample period for ~1.5 bits of noise floor.
///
/// It is also what keeps the PWM carrier out of the reading. The burst's eight
/// conversions are roughly 12 µs apart against a 16 µs carrier period, so they
/// land on four evenly spaced phases and the residue largely cancels — which
/// matters more since the filter traded rejection for bandwidth. See
/// `docs/HARDWARE.md`.
pub const OVERSAMPLE: Oversample = Oversample::Over8x;
/// Oversampling factor as a number, for the host's metadata header.
pub const OVERSAMPLE_FACTOR: u16 = 8;
/// Source impedance at the pin is the 10 kΩ series resistor plus whatever
/// drives it: 11.4 kΩ behind the offset ladder's 1.43 kΩ on `u_T`, 11.7 kΩ behind the filter's
/// 1.67 kΩ on `p_s`, and 10 kΩ plus the plant's own output impedance on
/// `P_e`. Nordic's table allows 10 µs up to 40 kΩ, so
/// the plant's output may be up to ~28 kΩ before this needs to grow.
pub const ACQUISITION_TIME: Time = Time::_10US;

// ── analog outputs ──────────────────────────────────────────────────────────

/// PWM clock divider. `Div1` leaves the peripheral on the 16 MHz base clock,
/// which is what makes a 256-count period land at 62.5 kHz.
pub const PWM_PRESCALER: Prescaler = Prescaler::Div1;
/// COUNTERTOP: 256 counts of duty, so a duty *is* an 8-bit code.
///
/// 16 MHz / 256 = **62.5 kHz**, the fastest carrier this part can produce at
/// 8-bit resolution, and the frequency `ngspice_filter.cir` was designed
/// around. Code 255 is therefore 255/256 of full scale, not quite all of it —
/// 0.4 % of range that the two-point calibration accounts for.
pub const PWM_TOP: u16 = 256;
/// Carrier frequency implied by the two constants above, for the host's
/// metadata and for the log line at boot.
pub const PWM_HZ: u32 = 16_000_000 / PWM_TOP as u32;
/// High drive: each output feeds the filter's 3 kΩ to ground, i.e.
/// ~1.1 mA when high.
/// Standard drive would still work, but its larger on-resistance shows up as a
/// gain error — constant, and therefore calibrated out, but larger than it
/// needs to be.
pub const PWM_DRIVE: OutputDrive = OutputDrive::HighDrive;
/// Rate at which the waveform is re-evaluated and the duty rewritten.
///
/// 700× the fastest plant dynamics of interest (the ~1.4 Hz rotor mode), so it
/// is far more than the assignment needs.
///
/// **It is now the bandwidth limit, not the filter.** Widening the filter to a
/// ~800 Hz corner put it level with this tick, so the staircase a 1 kHz update
/// leaves on a fast sine is no longer smoothed away: a 100 Hz sine carries
/// images at 900 and 1100 Hz at ~11 % of the fundamental, and the filter now
/// takes only ~3.7 dB off them. Below ~20 Hz none of this matters. Above it,
/// the fix is the PWM peripheral's own sequencer (`SequencePwm`, clocked by
/// hardware) rather than a faster `Ticker` — the RTC1 time driver's 30.5 µs
/// granularity cannot pace a 10 kHz tick.
pub const TICK_HZ: u64 = 1000;

// ── sampling ────────────────────────────────────────────────────────────────

/// Samples per channel carried by one block. At 1 kHz this is a frame every
/// 64 ms, which keeps framing overhead under 5 % without hurting anything —
/// the timestamps come from the sample index, not from arrival time.
pub const BLOCK_SAMPLES: usize = 64;
/// Default rate if the host does not ask for one.
pub const DEFAULT_FS_HZ: u32 = 1000;
/// Slowest rate accepted.
pub const MIN_FS_HZ: u32 = 10;
/// Fastest rate accepted.
///
/// A scan of 3 channels at 8× burst costs `3 × 8 × (10 µs + 2 µs) ≈ 288 µs`,
/// so 2 kHz keeps the ADC busy 58 % of the period. Going faster means
/// dropping the oversampling first.
pub const MAX_FS_HZ: u32 = 2000;

// ── host link ───────────────────────────────────────────────────────────────

/// Native USB CDC-ACM. This board has no debug probe and no USB-serial bridge,
/// so the nRF52840's own USB peripheral is the link — one cable, and it
/// enumerates as `/dev/ttyACM0`.
///
/// Full-speed bulk endpoints move 64 bytes per packet and well over 100 kB/s in
/// practice, against the ~8 kB/s this rig produces at 1 kHz × 3 channels. The
/// baud rate a host sets on a CDC port is decoration; there is no UART behind
/// it, and no RTS/CTS either — USB NAKs when the device is not reading, which
/// is flow control the firmware cannot get wrong.
pub const USB_PACKET_BYTES: usize = 64;
/// pid.codes test VID/PID. Fine on a bench, not for anything shipped.
pub const USB_VID: u16 = 0x1209;
/// See [`USB_VID`].
pub const USB_PID: u16 = 0x0001;
/// Shown by `lsusb` and in the udev attributes, so a rule can match it.
pub const USB_MANUFACTURER: &str = "plant-trace";
/// See [`USB_MANUFACTURER`].
pub const USB_PRODUCT: &str = "plant-trace rig";
/// See [`USB_MANUFACTURER`].
pub const USB_SERIAL: &str = "plant-trace-0001";
/// Bus current drawn, milliamps. The board plus two 3 kΩ filters is well
/// under this; the descriptor just has to not lie downwards.
pub const USB_MAX_POWER_MA: u16 = 100;
