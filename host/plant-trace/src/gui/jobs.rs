//! Long jobs the rig thread runs on the UI's behalf: an experiment from the
//! scenario editor, or an automatic Bode sweep.
//!
//! Both are library functions with an observer — `runner::run_on` and
//! `bode::run_bode`, exactly what `plant-trace run` and `plant-trace bode`
//! call. This module only turns their observer callbacks into events for the
//! tabs, and the cancel button into their `cancelled()`.

use std::{
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
};

use anyhow::Result;

use crate::{
    bode::{self, BodeObserver, BodePlan, BodePoint},
    daq::Daq,
    experiment::{Experiment, Step},
    runner::{self, RunObserver},
};

use super::worker::{Evt, Out};

/// A job, as the UI describes it.
pub enum JobSpec {
    /// Run an experiment and write its results under `out_dir`.
    Experiment {
        /// The scenario.
        experiment: Experiment,
        /// Where the CSVs and the manifest go.
        out_dir: PathBuf,
    },
    /// Run a stepped-sine Bode sweep.
    Bode {
        /// The plan as edited.
        plan: BodePlan,
        /// Where the tables and the raw stream go.
        out_dir: PathBuf,
    },
}

impl JobSpec {
    /// Short label for the status line.
    pub fn label(&self) -> String {
        match self {
            JobSpec::Experiment { experiment, .. } => format!("experiment {}", experiment.name),
            JobSpec::Bode { plan, .. } => {
                format!("Bode sweep, {} frequencies", plan.frequencies_hz.len())
            }
        }
    }
}

/// Something a running job reports before it ends.
pub enum JobEvent {
    /// Step `index` began.
    StepStarted {
        /// Position in the experiment.
        index: usize,
        /// Its label.
        name: String,
    },
    /// Live samples of the job's own stream.
    Samples {
        /// Stream index of the first row.
        first: u64,
        /// Sample rate.
        fs_hz: u32,
        /// Rows of `[u_t, p_s, p_e]`, volts.
        rows: Vec<[f32; 3]>,
    },
    /// The sweep moved on to frequency `index`.
    BodeFrequency {
        /// Position in the plan.
        index: usize,
        /// The frequency.
        freq_hz: f64,
    },
    /// A frequency was measured.
    BodePoint(BodePoint),
}

/// What a finished job leaves behind.
pub struct JobOutcome {
    /// Directory holding its files.
    pub dir: PathBuf,
    /// One line for the toast.
    pub summary: String,
    /// For a Bode sweep: the pure delay that best explains the raw phase.
    pub delay_s: Option<f64>,
    /// For a Bode sweep: the per-channel scan spacing that delay implies, if
    /// the true system has no phase of its own (a wire).
    pub implied_spacing_s: Option<f64>,
}

/// Forwards a job's observer calls to the UI.
struct Relay<'a> {
    out: &'a Out,
    cancel: &'a AtomicBool,
}

impl Relay<'_> {
    fn job(&self, e: JobEvent) {
        self.out.send(Evt::Job(e));
    }
}

impl RunObserver for Relay<'_> {
    fn step_started(&mut self, index: usize, step: &Step) {
        self.job(JobEvent::StepStarted {
            index,
            name: step.name().to_string(),
        });
    }

    fn samples(&mut self, first_sample: u64, fs_hz: u32, rows: &[[f32; 3]]) {
        self.job(JobEvent::Samples {
            first: first_sample,
            fs_hz,
            rows: rows.to_vec(),
        });
    }

    fn status(&mut self, line: &str) {
        self.out.send(Evt::Status(line.to_string()));
    }

    fn log(&mut self, line: &str) {
        self.out.send(Evt::Status(line.to_string()));
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

impl BodeObserver for Relay<'_> {
    fn point_started(&mut self, index: usize, freq_hz: f64) {
        self.job(JobEvent::BodeFrequency { index, freq_hz });
    }

    fn samples(&mut self, first_sample: u64, fs_hz: u32, rows: &[[f32; 3]]) {
        RunObserver::samples(self, first_sample, fs_hz, rows);
    }

    fn status(&mut self, line: &str) {
        self.out.send(Evt::Status(line.to_string()));
    }

    fn point_finished(&mut self, point: &BodePoint) {
        self.job(JobEvent::BodePoint(point.clone()));
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

/// Run a job to completion on the rig thread.
pub fn run(daq: &mut Daq, spec: JobSpec, out: &Out, cancel: &AtomicBool) -> Result<JobOutcome> {
    let mut relay = Relay { out, cancel };
    match spec {
        JobSpec::Experiment {
            experiment,
            out_dir,
        } => {
            let manifest = runner::run_on(daq, &experiment, &out_dir, &mut relay)?;
            Ok(JobOutcome {
                summary: format!(
                    "{}: {} step(s){}",
                    manifest.experiment,
                    manifest.steps.len(),
                    if manifest.cancelled {
                        ", cancelled"
                    } else {
                        ""
                    }
                ),
                dir: out_dir,
                delay_s: None,
                implied_spacing_s: None,
            })
        }
        JobSpec::Bode { plan, out_dir } => {
            let result = bode::run_bode(daq, &plan, Some(&out_dir), &mut relay)?;
            Ok(JobOutcome {
                summary: format!(
                    "{}: {} point(s){}",
                    plan.name,
                    result.points.len(),
                    if result.cancelled { ", cancelled" } else { "" }
                ),
                dir: out_dir,
                delay_s: result.delay.map(|d| d.delay_s),
                implied_spacing_s: result.implied_scan_spacing_s,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    //! The rig thread and the job relay, end to end against the in-process
    //! simulator with the plant replaced by a wire — the bench as it is today.

    use std::time::{Duration, Instant};

    use super::*;
    use crate::{
        bode::{ExcitedOutput, ResponseChannel},
        experiment::WaveSpec,
        gui::worker::{Cmd, Worker},
        sim::{self, SimPlant},
    };

    fn rig() -> (sim::SimHandle, Worker) {
        let handle = sim::spawn(sim::Options {
            plant: SimPlant::Wire { from: 0 },
            settle_s: 0.5,
            speed: 20.0,
            ..sim::Options::default()
        })
        .unwrap();
        let worker = Worker::spawn(|| {});
        worker.send(Cmd::Connect(handle.spec().to_string()));
        (handle, worker)
    }

    /// Pump events until the job ends; returns every job event and the outcome.
    fn until_done(worker: &Worker, limit: Duration) -> (Vec<JobEvent>, Result<JobOutcome, String>) {
        let deadline = Instant::now() + limit;
        let mut events = Vec::new();
        while Instant::now() < deadline {
            for evt in worker.drain() {
                match evt {
                    Evt::Job(e) => events.push(e),
                    Evt::JobDone(r) => return (events, r),
                    Evt::Error(e) => panic!("rig error: {e}"),
                    _ => {}
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        panic!("the job did not finish in {limit:?}");
    }

    #[test]
    fn a_bode_job_on_the_wire_reads_unity_gain_and_only_the_scan_skew() {
        let (_sim, worker) = rig();
        let dir = std::env::temp_dir().join(format!("plant-trace-gui-bode-{}", std::process::id()));
        let plan = BodePlan {
            excite: ExcitedOutput::ValveCmd,
            response: ResponseChannel::ElectricalPower,
            frequencies_hz: vec![2.0, 20.0],
            initial_settle_s: 0.2,
            settle_cycles: 2.0,
            settle_min_s: 0.1,
            measure_cycles: 6.0,
            measure_min_s: 0.3,
            // A wire, not the plant: free to go past 15 Hz.
            max_freq_hz: 100.0,
            ..BodePlan::default()
        };
        plan.validate().unwrap();
        worker.send(Cmd::Job(JobSpec::Bode {
            plan,
            out_dir: dir.clone(),
        }));
        let (events, outcome) = until_done(&worker, Duration::from_secs(60));
        let outcome = outcome.expect("the sweep failed");
        let points: Vec<&BodePoint> = events
            .iter()
            .filter_map(|e| match e {
                JobEvent::BodePoint(p) => Some(p),
                _ => None,
            })
            .collect();
        assert_eq!(points.len(), 2);
        assert!(events.iter().any(|e| matches!(e, JobEvent::Samples { .. })));
        for p in points {
            assert!(p.gain_db.abs() < 0.1, "{} Hz: {} dB", p.freq_hz, p.gain_db);
            assert!(
                p.phase_deg.abs() < 1.0,
                "{} Hz: corrected {}°",
                p.freq_hz,
                p.phase_deg
            );
        }
        assert!(outcome.dir.join("bode.csv").exists());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_scenario_job_reports_every_step_and_writes_its_manifest() {
        let (_sim, worker) = rig();
        let dir = std::env::temp_dir().join(format!("plant-trace-gui-run-{}", std::process::id()));
        let mut exp = crate::gui::scenario::template_for_tests();
        exp.steps = vec![
            Step::Settle {
                name: "settle".into(),
                u_t: 2.5,
                p_s: 0.5,
                timeout_s: 5.0,
                tol_v: 0.01,
                window_s: 0.2,
            },
            Step::Record {
                name: "step".into(),
                duration_s: 1.0,
                u_t: Some(WaveSpec::Step {
                    base: 2.5,
                    step: 0.025,
                    hold_s: 0.5,
                }),
                p_s: None,
            },
        ];
        worker.send(Cmd::Job(JobSpec::Experiment {
            experiment: exp,
            out_dir: dir.clone(),
        }));
        let (events, outcome) = until_done(&worker, Duration::from_secs(60));
        outcome.expect("the run failed");
        let started: Vec<usize> = events
            .iter()
            .filter_map(|e| match e {
                JobEvent::StepStarted { index, .. } => Some(*index),
                _ => None,
            })
            .collect();
        assert_eq!(started, vec![0, 1]);
        assert!(dir.join("run.json").exists());
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Samples of channel `ch` for `wall` of wall-clock time.
    fn collect(worker: &Worker, ch: usize, wall: Duration) -> Vec<f32> {
        let deadline = Instant::now() + wall;
        let mut v = Vec::new();
        while Instant::now() < deadline {
            for evt in worker.drain() {
                match evt {
                    Evt::Samples { rows, .. } => v.extend(rows.iter().map(|r| r[ch])),
                    Evt::Error(e) => panic!("rig error: {e}"),
                    _ => {}
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        v
    }

    #[test]
    fn the_generator_runs_a_sine_until_stopped_and_returns_to_its_centre() {
        let (_sim, worker) = rig();
        worker.send(Cmd::Monitor {
            on: true,
            fs_hz: 1000,
        });
        worker.send(Cmd::Generate([
            Some(plant_trace_proto::waveform::Waveform::Sine {
                center: 2.5,
                amplitude: 0.1,
                freq_hz: 5.0,
                cycles: 0,
            }),
            None,
        ]));
        // Wait for the stream, skip the start, then look at a few whole
        // periods (20× speed).
        let deadline = Instant::now() + Duration::from_secs(10);
        while collect(&worker, 0, Duration::from_millis(20)).is_empty() {
            assert!(Instant::now() < deadline, "the live view never started");
        }
        collect(&worker, 0, Duration::from_millis(100));
        let running = collect(&worker, 0, Duration::from_millis(150));
        assert!(running.len() > 500, "{} samples", running.len());
        let lo = running.iter().cloned().fold(f32::INFINITY, f32::min);
        let hi = running.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        assert!(
            (lo - 2.4).abs() < 0.01 && (hi - 2.6).abs() < 0.01,
            "{lo}..{hi}"
        );

        worker.send(Cmd::GenStop([Some(2.5), None]));
        collect(&worker, 0, Duration::from_millis(50));
        let held = collect(&worker, 0, Duration::from_millis(100));
        let lo = held.iter().cloned().fold(f32::INFINITY, f32::min);
        let hi = held.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        assert!(
            hi - lo < 0.005 && (lo - 2.5).abs() < 0.005,
            "held at {lo}..{hi}"
        );
    }
}
