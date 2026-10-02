//! Reference model of the "magic box": a single machine connected to an
//! infinite bus through a tandem-compound steam turbine.
//!
//! Two jobs:
//!
//! 1. it backs `plant-trace simulate`, so the whole acquisition and
//!    identification pipeline can be exercised without the bench;
//! 2. it is the ground truth for the identification tests — the fits in
//!    phase 6 are checked against parameters that are known exactly here.
//!
//! The structure follows the block diagram in the assignment: a valve with a
//! rate limit, an actuator lag, a transport delay and a *non-linear* flow
//! characteristic; steam pressure entering multiplicatively; the HP/IP/LP
//! cascade of the turbine; and the electromechanical swing equation of the
//! rotor against a stiff grid.
//!
//! ```text
//!  u_T ─► delay ─► lag+rate ─► φ(·) ─►(×)─► T_CH ─► T_RH ─► T_CO ─► P_m ─► swing ─► P_e
//!                                      ▲
//!                                     p_s
//! ```
//!
//! Every level is in volts at the plant's own terminals (0-1 V), so the model
//! speaks the same units as the rig.

use std::collections::VecDeque;

/// Plant parameters, in the units the block diagram uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantParams {
    // ── valve ───────────────────────────────────────────────────────────────
    /// Transport delay between command and any movement, seconds.
    pub valve_delay_s: f32,
    /// Actuator lag, seconds.
    pub valve_tau_s: f32,
    /// Largest opening change per second (1.0 = full travel in one second).
    pub valve_rate_limit: f32,
    /// Curvature of the flow characteristic. 0 is linear; larger values make
    /// the valve "equal percentage"-like, so the incremental gain at a small
    /// opening is a fraction of the gain near full opening.
    pub valve_curvature: f32,

    // ── turbine (IEEE tandem-compound, HP/IP/LP) ────────────────────────────
    /// Steam chest time constant, seconds.
    pub t_ch_s: f32,
    /// Reheater time constant, seconds.
    pub t_rh_s: f32,
    /// Crossover time constant, seconds.
    pub t_co_s: f32,
    /// Power fraction of the high-pressure stage.
    pub f_hp: f32,
    /// Power fraction of the intermediate-pressure stage.
    pub f_ip: f32,
    /// Power fraction of the low-pressure stage.
    pub f_lp: f32,

    // ── generator and grid ──────────────────────────────────────────────────
    /// Inertia constant, seconds.
    pub inertia_h_s: f32,
    /// Damping coefficient, pu torque per pu speed.
    pub damping: f32,
    /// Maximum power transfer of the line, pu — sets the synchronising torque.
    pub p_max: f32,
    /// Grid frequency in electrical rad/s.
    pub omega_base: f32,

    // ── measurement ─────────────────────────────────────────────────────────
    /// Nominal steam pressure, volts. The disturbance enters as `p_s / p_s0`.
    pub p_s_nominal_v: f32,
    /// Volts of `P_e` per pu of electrical power.
    pub volts_per_pu: f32,
    /// Standard deviation of the additive output noise, volts.
    pub noise_v: f32,
}

impl Default for PlantParams {
    /// A plausible 60 Hz unit: a visible ~1.4 Hz rotor mode, a slow reheater,
    /// and a valve whose gain roughly triples across its travel.
    fn default() -> Self {
        Self {
            valve_delay_s: 0.15,
            valve_tau_s: 0.10,
            valve_rate_limit: 0.5,
            valve_curvature: 1.5,

            t_ch_s: 0.30,
            t_rh_s: 7.0,
            t_co_s: 0.40,
            f_hp: 0.30,
            f_ip: 0.40,
            f_lp: 0.30,

            inertia_h_s: 4.0,
            damping: 10.0,
            p_max: 1.8,
            omega_base: 377.0,

            p_s_nominal_v: 0.80,
            volts_per_pu: 1.0,
            noise_v: 3e-4,
        }
    }
}

impl PlantParams {
    /// The valve's static flow characteristic, mapping opening to flow.
    ///
    /// `(e^{k x} − 1)/(e^k − 1)`: monotone, 0→0, 1→1, with an incremental gain
    /// of `k/(e^k − 1)` at the bottom and `k e^k/(e^k − 1)` at the top. At the
    /// default `k = 1.5` that is a ratio of about 4.5 between the two ends —
    /// the non-linearity the characterisation is supposed to expose.
    pub fn flow(&self, opening: f32) -> f32 {
        let x = opening.clamp(0.0, 1.0);
        let k = self.valve_curvature;
        if k.abs() < 1e-6 {
            x
        } else {
            ((k * x).exp() - 1.0) / (k.exp() - 1.0)
        }
    }

    /// Slope of [`PlantParams::flow`] at an opening — the incremental valve
    /// gain the report is asked to compare across operating points.
    pub fn flow_gain(&self, opening: f32) -> f32 {
        let x = opening.clamp(0.0, 1.0);
        let k = self.valve_curvature;
        if k.abs() < 1e-6 {
            1.0
        } else {
            k * (k * x).exp() / (k.exp() - 1.0)
        }
    }

    /// Undamped natural frequency and damping ratio of the rotor mode at a
    /// given electrical power, in rad/s and dimensionless.
    ///
    /// Both depend on the operating point through `cos δ₀`, which is why the
    /// oscillation seen in a step test at 0.3 pu is not the one seen at
    /// 0.7 pu.
    pub fn rotor_mode(&self, p_e_pu: f32) -> (f32, f32) {
        let delta0 = (p_e_pu / self.p_max).clamp(-0.999, 0.999).asin();
        let k_s = self.p_max * delta0.cos();
        let omega_n = (k_s * self.omega_base / (2.0 * self.inertia_h_s)).sqrt();
        let zeta = self.damping * self.omega_base
            / (2.0 * 2.0 * self.inertia_h_s * omega_n)
            / self.omega_base;
        (omega_n, zeta)
    }
}

/// Continuous state of the plant.
#[derive(Debug, Clone, Copy, Default)]
struct State {
    /// Valve opening after the actuator lag, 0-1.
    opening: f32,
    /// Steam chest output, pu.
    x_ch: f32,
    /// Reheater output, pu.
    x_rh: f32,
    /// Crossover output, pu.
    x_co: f32,
    /// Rotor angle, electrical radians.
    delta: f32,
    /// Speed deviation, pu.
    d_omega: f32,
}

/// A running simulation of the plant.
pub struct Plant {
    params: PlantParams,
    state: State,
    /// Transport delay line for the valve command, one slot per time step.
    delay: VecDeque<f32>,
    /// Deterministic noise source — a simulated run has to be reproducible.
    rng: u32,
}

impl Plant {
    /// Build a plant at rest.
    pub fn new(params: PlantParams) -> Self {
        Self {
            params,
            state: State::default(),
            delay: VecDeque::new(),
            rng: 0x1234_5678,
        }
    }

    /// The parameters in use.
    pub fn params(&self) -> &PlantParams {
        &self.params
    }

    /// Advance by `dt` seconds and return the measured `P_e`, in volts.
    ///
    /// `u_t` and `p_s` are the voltages at the plant's inputs.
    pub fn step(&mut self, dt: f32, u_t: f32, p_s: f32) -> f32 {
        // Transport delay, quantised to the step — the rig samples at 1 kHz
        // and a 150 ms delay is 150 slots, so the quantisation is invisible.
        let slots = ((self.params.valve_delay_s / dt).round() as usize).max(1);
        self.delay.push_back(u_t.clamp(0.0, 1.0));
        while self.delay.len() > slots {
            self.delay.pop_front();
        }
        let commanded = if self.delay.len() == slots {
            *self.delay.front().unwrap()
        } else {
            // Still filling the delay line: the valve has not seen anything yet.
            *self.delay.front().unwrap_or(&0.0)
        };

        self.state = rk4(self.state, dt, |s| self.derivative(s, commanded, p_s));

        // The actuator cannot travel faster than its rate limit; applying it to
        // the integrated state (rather than to the derivative) also keeps the
        // opening inside its mechanical stops.
        self.state.opening = self.state.opening.clamp(0.0, 1.0);

        let p_e_pu = self.params.p_max * self.state.delta.sin();
        p_e_pu * self.params.volts_per_pu + self.noise()
    }

    /// Run the plant to steady state at a fixed operating point and return the
    /// settled output, in volts.
    ///
    /// Used by the simulator at start-up and by tests that need a plant
    /// already sitting at an operating point.
    pub fn settle(&mut self, dt: f32, u_t: f32, p_s: f32, seconds: f32) -> f32 {
        let steps = (seconds / dt) as usize;
        let mut last = 0.0;
        for _ in 0..steps {
            last = self.step(dt, u_t, p_s);
        }
        last
    }

    fn derivative(&self, s: State, commanded: f32, p_s: f32) -> State {
        let p = &self.params;

        // Valve actuator: first-order lag, rate limited.
        let raw_rate = (commanded - s.opening) / p.valve_tau_s;
        let rate = raw_rate.clamp(-p.valve_rate_limit, p.valve_rate_limit);

        // Steam flow: the valve characteristic scaled by the pressure ratio —
        // the multiplying junction of the block diagram.
        let pressure_ratio = (p_s / p.p_s_nominal_v).max(0.0);
        let flow = p.flow(s.opening) * pressure_ratio;

        // Turbine cascade.
        let d_ch = (flow - s.x_ch) / p.t_ch_s;
        let d_rh = (s.x_ch - s.x_rh) / p.t_rh_s;
        let d_co = (s.x_rh - s.x_co) / p.t_co_s;
        let p_m = p.f_hp * s.x_ch + p.f_ip * s.x_rh + p.f_lp * s.x_co;

        // Swing equation against an infinite bus.
        let p_e = p.p_max * s.delta.sin();
        let d_delta = p.omega_base * s.d_omega;
        let d_omega = (p_m - p_e - p.damping * s.d_omega) / (2.0 * p.inertia_h_s);

        State {
            opening: rate,
            x_ch: d_ch,
            x_rh: d_rh,
            x_co: d_co,
            delta: d_delta,
            d_omega,
        }
    }

    /// Zero-mean Gaussian-ish noise from a xorshift generator, so a simulated
    /// run repeats exactly.
    fn noise(&mut self) -> f32 {
        if self.params.noise_v <= 0.0 {
            return 0.0;
        }
        // Sum of four uniforms: close enough to Gaussian for a noise floor,
        // and far cheaper than a Box-Muller pair.
        let mut acc = 0.0f32;
        for _ in 0..4 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 17;
            self.rng ^= self.rng << 5;
            acc += (self.rng as f32 / u32::MAX as f32) - 0.5;
        }
        acc * self.params.noise_v * 1.73
    }
}

/// Classical fourth-order Runge-Kutta.
///
/// The rotor mode sits at ~9 rad/s and the simulation runs at 1 kHz, so
/// explicit Euler would be stable — but it also *adds* damping, and the
/// damping ratio is one of the numbers the identification is supposed to
/// recover. RK4 keeps the model honest about its own answer.
fn rk4<F: Fn(State) -> State>(s: State, dt: f32, f: F) -> State {
    let k1 = f(s);
    let k2 = f(axpy(s, dt / 2.0, k1));
    let k3 = f(axpy(s, dt / 2.0, k2));
    let k4 = f(axpy(s, dt, k3));
    let sum = State {
        opening: k1.opening + 2.0 * k2.opening + 2.0 * k3.opening + k4.opening,
        x_ch: k1.x_ch + 2.0 * k2.x_ch + 2.0 * k3.x_ch + k4.x_ch,
        x_rh: k1.x_rh + 2.0 * k2.x_rh + 2.0 * k3.x_rh + k4.x_rh,
        x_co: k1.x_co + 2.0 * k2.x_co + 2.0 * k3.x_co + k4.x_co,
        delta: k1.delta + 2.0 * k2.delta + 2.0 * k3.delta + k4.delta,
        d_omega: k1.d_omega + 2.0 * k2.d_omega + 2.0 * k3.d_omega + k4.d_omega,
    };
    axpy(s, dt / 6.0, sum)
}

fn axpy(s: State, a: f32, d: State) -> State {
    State {
        opening: s.opening + a * d.opening,
        x_ch: s.x_ch + a * d.x_ch,
        x_rh: s.x_rh + a * d.x_rh,
        x_co: s.x_co + a * d.x_co,
        delta: s.delta + a * d.delta,
        d_omega: s.d_omega + a * d.d_omega,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1e-3;

    fn quiet() -> PlantParams {
        PlantParams {
            noise_v: 0.0,
            ..Default::default()
        }
    }

    #[test]
    fn settles_where_mechanical_meets_electrical_power() {
        let params = quiet();
        let mut plant = Plant::new(params);
        let p_e = plant.settle(DT, 0.6, params.p_s_nominal_v, 120.0);

        // In steady state P_e = P_m = flow(opening) with the valve fully
        // caught up to the command and the pressure at nominal.
        let expected = params.flow(0.6) * params.volts_per_pu;
        assert!(
            (p_e - expected).abs() < 2e-3,
            "settled at {p_e}, expected {expected}"
        );
    }

    #[test]
    fn valve_gain_grows_with_opening() {
        let params = quiet();
        // e^k between the ends, so k = 1.5 is a factor of ~4.5 across the
        // full travel and ~2.5 across the middle 60 % the tests exercise.
        let (closed, open) = (params.flow_gain(0.0), params.flow_gain(1.0));
        assert!(
            open > 4.0 * closed,
            "gain {closed} -> {open} is too flat to show the non-linearity"
        );

        // And the simulated static curve agrees with the analytic slope.
        let mut plant = Plant::new(params);
        let a = plant.settle(DT, 0.60, params.p_s_nominal_v, 120.0);
        let b = plant.settle(DT, 0.65, params.p_s_nominal_v, 120.0);
        let measured = (b - a) / 0.05;
        let analytic = params.flow_gain(0.625);
        assert!(
            (measured - analytic).abs() / analytic < 0.05,
            "measured {measured} vs analytic {analytic}"
        );
    }

    #[test]
    fn pressure_drop_lowers_power() {
        let params = quiet();
        let mut plant = Plant::new(params);
        let nominal = plant.settle(DT, 0.6, params.p_s_nominal_v, 120.0);
        let reduced = plant.settle(DT, 0.6, params.p_s_nominal_v * 0.9, 120.0);
        // Pressure enters multiplicatively, so 10 % less pressure is 10 % less
        // power once the turbine has caught up.
        assert!(
            (reduced / nominal - 0.9).abs() < 0.02,
            "{nominal} -> {reduced}"
        );
    }

    #[test]
    fn a_step_rings_at_the_rotor_frequency() {
        let params = quiet();
        let mut plant = Plant::new(params);
        let base = plant.settle(DT, 0.60, params.p_s_nominal_v, 120.0);

        // Count the oscillation period in the first seconds after a step.
        let mut trace = Vec::new();
        for _ in 0..4000 {
            trace.push(plant.step(DT, 0.66, params.p_s_nominal_v));
        }
        let peaks: Vec<usize> = (1..trace.len() - 1)
            .filter(|&i| trace[i] > trace[i - 1] && trace[i] >= trace[i + 1] && trace[i] > base)
            .collect();
        assert!(peaks.len() >= 2, "no ringing found: {} peaks", peaks.len());

        let period_s = (peaks[1] - peaks[0]) as f32 * DT;
        let (omega_n, zeta) = params.rotor_mode(base);
        let expected = 2.0 * std::f32::consts::PI / (omega_n * (1.0 - zeta * zeta).sqrt());
        assert!(
            (period_s - expected).abs() / expected < 0.15,
            "period {period_s} s, expected ~{expected} s"
        );
    }
}
