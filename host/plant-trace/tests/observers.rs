//! The GUI-facing backend, end to end against an in-process simulator:
//! [`runner::run_on`] and [`bode::run_bode`] with their observers, cancel, and
//! the stepped-sine Bode on a wire loopback — the configuration the bench is
//! in with the plant unplugged, where the true transfer is exactly 1 and every
//! degree of phase is the measurement's.

use std::path::{Path, PathBuf};

use plant_trace::{
    bode::{self, BodeObserver, BodePlan, BodePoint, ExcitedOutput, ResponseChannel},
    check,
    daq::Daq,
    experiment::{Experiment, Step},
    runner::{self, RunObserver, RunStart, StepRecord},
    sim::{self, SimHandle, SimPlant},
};
use plant_trace_proto::SCAN_CHANNEL_SPACING_S;

fn start(plant: SimPlant) -> SimHandle {
    sim::spawn(sim::Options {
        // Capped to 10× at 2 kHz by the simulator itself.
        speed: 20.0,
        settle_s: 30.0,
        plant,
        ..Default::default()
    })
    .unwrap()
}

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plant-trace-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

/// A short sweep for the wire: fast enough for a test, long enough to fit.
fn wire_plan() -> BodePlan {
    BodePlan {
        name: "wire".into(),
        excite: ExcitedOutput::ValveCmd,
        response: ResponseChannel::ElectricalPower,
        // The middle of the u_T window, 80 % of it.
        center_v: 2.5,
        amplitude_v: 0.2,
        hold_v: 0.5,
        frequencies_hz: vec![2.0, 10.0, 40.0, 100.0],
        fs_hz: 2000,
        initial_settle_s: 0.2,
        settle_cycles: 2.0,
        settle_min_s: 0.1,
        settle_max_s: 1.0,
        measure_cycles: 10.0,
        measure_min_s: 0.5,
        measure_max_s: 2.0,
        // A wire, not the plant: free to go past 15 Hz.
        max_freq_hz: 100.0,
        ..BodePlan::default()
    }
}

#[derive(Default)]
struct BodeLog {
    started: Vec<(usize, f64)>,
    finished: Vec<BodePoint>,
    next_sample: u64,
    samples: u64,
    cancel_after_points: Option<usize>,
}

impl BodeObserver for BodeLog {
    fn point_started(&mut self, index: usize, freq_hz: f64) {
        self.started.push((index, freq_hz));
    }
    fn samples(&mut self, first_sample: u64, _fs_hz: u32, rows: &[[f32; 3]]) {
        assert_eq!(
            first_sample, self.next_sample,
            "a sample went missing or twice"
        );
        self.next_sample = first_sample + rows.len() as u64;
        self.samples += rows.len() as u64;
    }
    fn point_finished(&mut self, point: &BodePoint) {
        self.finished.push(point.clone());
    }
    fn cancelled(&self) -> bool {
        self.cancel_after_points
            .is_some_and(|n| self.started.len() > n)
    }
}

#[test]
fn a_wire_measures_unity_and_the_scan_skew_and_nothing_else() {
    let rig = start(SimPlant::Wire { from: 0 });
    let mut daq = Daq::open(rig.spec(), 0).unwrap();
    let out = scratch("bode-wire");
    let plan = wire_plan();
    let mut log = BodeLog::default();

    let result = bode::run_bode(&mut daq, &plan, Some(&out), &mut log).unwrap();
    assert!(!result.cancelled);
    assert_eq!(result.points.len(), 4);
    assert_eq!(log.finished, result.points);
    assert_eq!(
        log.started,
        vec![(0, 2.0), (1, 10.0), (2, 40.0), (3, 100.0)]
    );

    let skew_s = 2.0 * SCAN_CHANNEL_SPACING_S;
    for p in &result.points {
        // Sense and P_e read the same node: the ratio is 1, whatever the
        // output filter and the tick did to the signal on its way there.
        assert!(p.gain_db.abs() < 0.05, "{p:?}");
        // ...and the signal itself is what the design says the chain does.
        let expected = check::expected_response(p.freq_hz, 1000.0).gain_db;
        let input_db = 20.0 * (p.input_amplitude_v / 0.2).log10();
        assert!(
            (input_db - expected).abs() < 0.1,
            "{} Hz: input at {input_db:+.3} dB, the chain predicts {expected:+.3} dB",
            p.freq_hz
        );
        // P_e is converted two slots after u_T: an apparent lead, exactly the
        // skew, which the correction takes back out.
        let lead = 360.0 * p.freq_hz * skew_s;
        assert!((p.skew_deg - lead).abs() < 1e-9, "{p:?}");
        assert!((p.raw_phase_deg - lead).abs() < 0.2, "{p:?}");
        assert!(p.phase_deg.abs() < 0.2, "{p:?}");
    }

    // The pure-delay fit sees the skew as a negative delay, and turns it back
    // into the scan spacing.
    let d = result.delay.expect("four points make a line");
    assert!((d.delay_s + skew_s).abs() < 5e-6, "{d:?}");
    let spacing = result.implied_scan_spacing_s.unwrap();
    assert!(
        (spacing - SCAN_CHANNEL_SPACING_S).abs() < 3e-6,
        "implied spacing {} µs",
        spacing * 1e6
    );

    // Everything the observer was shown is on disk, once.
    for f in ["bode.csv", "bode.json", "stream.csv"] {
        assert!(out.join(f).exists(), "{f} missing");
    }
    // Read back to the last few bits (JSON floats are not bit-exact).
    let back = bode::BodeResult::load(&out).unwrap();
    assert_eq!(back.plan, result.plan);
    assert_eq!(back.points.len(), result.points.len());
    for (a, b) in back.points.iter().zip(&result.points) {
        assert_eq!(a.freq_hz, b.freq_hz);
        assert!((a.gain - b.gain).abs() < 1e-12 && (a.phase_deg - b.phase_deg).abs() < 1e-9);
    }
    let rows = std::fs::read_to_string(out.join("stream.csv"))
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("t_s"))
        .count() as u64;
    assert_eq!(rows, log.samples);
    let csv = std::fs::read_to_string(out.join("bode.csv")).unwrap();
    assert_eq!(
        csv.lines().filter(|l| !l.starts_with('#')).count(),
        1 + result.points.len()
    );

    // The rig was left stopped, parked and usable.
    let fs = daq.start(0).unwrap();
    assert!(fs > 0);
    daq.stop().unwrap();
    let status = daq.gen_status().unwrap();
    assert!(!status.running);
}

#[test]
fn a_cancelled_sweep_keeps_what_it_measured() {
    let rig = start(SimPlant::Wire { from: 0 });
    let mut daq = Daq::open(rig.spec(), 0).unwrap();
    let out = scratch("bode-cancel");
    let mut log = BodeLog {
        cancel_after_points: Some(1),
        ..Default::default()
    };

    let result = bode::run_bode(&mut daq, &wire_plan(), Some(&out), &mut log).unwrap();
    assert!(result.cancelled);
    assert_eq!(result.points.len(), 1, "{:?}", result.points);
    assert!(bode::BodeResult::load(&out).unwrap().cancelled);

    // Stopped, not streaming: a new stream starts.
    daq.start(0).unwrap();
    daq.stop().unwrap();
}

#[test]
fn every_bode_plan_in_the_repository_is_valid() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../experiments");
    let mut checked = 0;
    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        if !(name.starts_with("bode-") && name.ends_with(".toml")) {
            continue;
        }
        let plan = BodePlan::load(&path).unwrap_or_else(|e| panic!("{name}: {e:#}"));
        let minutes = plan.estimated_duration().as_secs_f64() / 60.0;
        assert!(minutes < 30.0, "{name} takes {minutes:.0} min");
        checked += 1;
    }
    assert!(checked >= 2, "only {checked} Bode plans found");
}

// ── runner ──────────────────────────────────────────────────────────────────

const QUICK: &str = r#"
name = "observed"
fs_hz = 1000

[[step]]
kind = "settle"
name = "op"
u_t = 2.50
p_s = 0.80
timeout_s = 20
tol_v = 0.01
window_s = 0.5

[[step]]
kind = "record"
name = "step"
duration_s = 1.5
[step.u_t]
kind = "step"
base = 2.50
step = 0.025
hold_s = 0.5

[[step]]
kind = "record"
name = "tail"
duration_s = 0.7
"#;

#[derive(Debug, PartialEq)]
enum Ev {
    Started(u32),
    StepStarted(usize, String),
    Samples(u64, usize),
    StepFinished(usize, u64, u64),
}

#[derive(Default)]
struct RunLog {
    events: Vec<Ev>,
    rows: Vec<[f32; 3]>,
    /// Cancel once this step has seen this many samples.
    cancel_in: Option<(usize, u64)>,
    step: Option<usize>,
    in_step: u64,
}

impl RunObserver for RunLog {
    fn started(&mut self, s: &RunStart) {
        self.events.push(Ev::Started(s.fs_hz));
    }
    fn step_started(&mut self, index: usize, step: &Step) {
        self.events
            .push(Ev::StepStarted(index, step.name().to_string()));
        self.step = Some(index);
        self.in_step = 0;
    }
    fn samples(&mut self, first_sample: u64, _fs_hz: u32, rows: &[[f32; 3]]) {
        self.events.push(Ev::Samples(first_sample, rows.len()));
        self.rows.extend_from_slice(rows);
        self.in_step += rows.len() as u64;
    }
    fn step_finished(&mut self, index: usize, r: &StepRecord) {
        self.events
            .push(Ev::StepFinished(index, r.first_sample, r.samples));
    }
    fn cancelled(&self) -> bool {
        self.cancel_in
            .is_some_and(|(step, n)| self.step == Some(step) && self.in_step >= n)
    }
}

#[test]
fn run_on_reports_every_step_and_every_sample_exactly_once() {
    let rig = start(SimPlant::Model);
    let mut daq = Daq::open(rig.spec(), 0).unwrap();
    let experiment: Experiment = toml::from_str(QUICK).unwrap();
    let out = scratch("run-on");
    let mut log = RunLog::default();

    let manifest = runner::run_on(&mut daq, &experiment, &out, &mut log).unwrap();
    assert!(!manifest.cancelled);
    assert_eq!(manifest.steps.len(), 3);
    assert!(out.join("run.json").exists());

    assert_eq!(log.events[0], Ev::Started(1000));
    let mut next = 0u64;
    let mut open: Option<usize> = None;
    let mut in_step = 0u64;
    let mut steps_seen = Vec::new();
    for ev in &log.events[1..] {
        match ev {
            Ev::StepStarted(i, _) => {
                assert!(open.is_none(), "step {i} started inside another");
                open = Some(*i);
                in_step = 0;
            }
            Ev::Samples(first, n) => {
                assert!(open.is_some(), "samples outside any step");
                assert_eq!(*first, next, "samples out of order or repeated");
                next = first + *n as u64;
                in_step += *n as u64;
            }
            Ev::StepFinished(i, first, samples) => {
                assert_eq!(open.take(), Some(*i));
                // What the observer saw during the step is exactly what the
                // manifest says the step covered.
                assert_eq!(*samples, in_step, "step {i}");
                assert_eq!(*first + *samples, next, "step {i}");
                steps_seen.push(*i);
            }
            Ev::Started(_) => panic!("started twice"),
        }
    }
    assert_eq!(steps_seen, vec![0, 1, 2]);
    let r = &manifest.steps;
    assert_eq!(r[1].first_sample, r[0].first_sample + r[0].samples);
    assert_eq!(next, r[2].first_sample + r[2].samples);
    assert_eq!(log.rows.len() as u64, next);

    // The rows are volts: the step is in them, and the untouched output held.
    let step = &log.rows[r[1].first_sample as usize..][..r[1].samples as usize];
    // Within a code of the u_T ladder's 2.77 mV at either end.
    assert!((step[100][0] - 2.5).abs() < 5e-3, "{:?}", step[100]);
    assert!((step[step.len() - 1][0] - 2.525).abs() < 5e-3);
    assert!((step[step.len() - 1][1] - 0.8).abs() < 5e-3);
    // The tail step programs nothing: u_T stays where the step left it.
    let tail = log.rows.last().unwrap();
    assert!((tail[0] - 2.525).abs() < 5e-3, "{tail:?}");
}

#[test]
fn a_cancelled_run_stops_cleanly_and_says_so() {
    let rig = start(SimPlant::Model);
    let mut daq = Daq::open(rig.spec(), 0).unwrap();
    let experiment: Experiment = toml::from_str(QUICK).unwrap();
    let out = scratch("run-cancel");
    // Cancel a few hundred samples into the recording step.
    let mut log = RunLog {
        cancel_in: Some((1, 300)),
        ..Default::default()
    };

    let manifest = runner::run_on(&mut daq, &experiment, &out, &mut log).unwrap();
    assert!(manifest.cancelled);
    let last = manifest.steps.last().unwrap();
    assert!(last.interrupted, "{last:?}");
    assert_eq!(manifest.steps.len(), 2);
    assert_eq!(last.name, "step");
    assert!(last.samples < 1000, "{last:?}");
    assert!(!manifest.steps[0].interrupted);
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(out.join("run.json")).unwrap()).unwrap();
    assert_eq!(json["cancelled"], true);

    // The generator was frozen and the stream stopped: the link is free.
    assert!(!daq.gen_status().unwrap().running);
    daq.start(0).unwrap();
    daq.stop().unwrap();
}

#[test]
fn the_simulator_shuts_down_when_its_handle_goes() {
    let rig = start(SimPlant::Model);
    let spec = rig.spec().to_string();
    let mut daq = Daq::open(&spec, 0).unwrap();
    daq.start(0).unwrap();
    rig.shutdown();
    // The session was ended from the simulator's side and nobody listens.
    assert!(daq.stop().is_err());
    drop(daq);
    assert!(Daq::open(&spec, 0).and_then(|mut d| d.info()).is_err());
}
