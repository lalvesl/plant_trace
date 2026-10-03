//! The acquisition path, end to end, against the simulated rig.
//!
//! This is the test that has to keep passing while the bench is unavailable:
//! it exercises the framing, the session, the CSV writer and the plant model
//! together, and checks the numbers that come out against what the model says
//! they should be.
//!
//! One connection carries both halves, as on the real rig, so a test drives the
//! outputs and records through the same [`Daq`].

use std::{
    net::TcpListener,
    path::Path,
    thread,
    time::{Duration, Instant},
};

use plant_model::PlantParams;
use plant_trace::{
    check,
    csvout::{RunMeta, RunWriter},
    daq::{Daq, Event},
    experiment::Experiment,
    sim,
};
use plant_trace_proto::{
    scale::{AdcScale, DacScale},
    waveform::Waveform,
};

/// Ask the OS for a free port, so parallel test runs cannot collide.
fn free_port() -> String {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .to_string()
}

/// Start a simulator and return the link spec for it.
fn start_sim(speed: f32) -> String {
    let addr = free_port();
    let opts = sim::Options {
        addr: addr.clone(),
        fs_hz: 1000,
        speed,
        settle_s: 150.0,
        ..Default::default()
    };
    thread::spawn(move || sim::run(opts));

    // The listener comes up a moment after the thread starts.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        if std::net::TcpStream::connect(&addr).is_ok() {
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    format!("tcp://{addr}")
}

/// Record `duration` seconds into a CSV under the temp dir, returning the
/// file's rows as `(t, u_t, p_s, p_e)` volts.
fn record(device: &mut Daq, duration_s: f64, name: &str) -> (Vec<[f32; 4]>, u64) {
    let info = device.info().unwrap();
    let fs = device.start(0).unwrap();

    let path = std::env::temp_dir().join(format!("plant-trace-{name}.csv"));
    let mut writer = RunWriter::create(
        &path,
        &RunMeta {
            source: name.to_string(),
            fs_hz: fs,
            adc: info.adc_scale(),
            oversample: info.oversample,
            firmware: info.firmware.clone(),
            note: Some(name.to_string()),
            origin_sample: 0,
        },
    )
    .unwrap();

    let wanted = (duration_s * fs as f64) as u64;
    let mut rows = 0u64;
    while rows < wanted {
        match device.next_event(Duration::from_secs(5)).unwrap() {
            Some(Event::Block {
                seq,
                n,
                channels,
                counts,
                ..
            }) => {
                writer.push_block(seq, n, channels, &counts).unwrap();
                rows += n as u64;
            }
            Some(other) => panic!("unexpected message: {other:?}"),
            None => panic!("the simulated rig went quiet"),
        }
    }
    let _ = device.stop();
    let stats = writer.finish().unwrap();

    let text = std::fs::read_to_string(&path).unwrap();
    let parsed = text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("t_s"))
        .map(|l| {
            let f: Vec<f32> = l.split(',').map(|v| v.parse().unwrap()).collect();
            [f[0], f[4], f[5], f[6]]
        })
        .collect();
    (parsed, stats.gaps)
}

#[test]
fn records_a_steady_operating_point_where_the_model_says_it_should() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    // 2.55 V at the box is 0.60 of the model's valve range (the simulator maps
    // the box's 2.25-2.75 V window onto the model's 0-1, on u_T and P_e).
    rig.set_level(0, 2.55).unwrap();
    rig.set_level(1, 0.80).unwrap();

    let (rows, gaps) = record(&mut rig, 3.0, "steady");
    assert_eq!(gaps, 0, "blocks went missing between simulator and host");
    assert!(rows.len() >= 3000, "only {} rows", rows.len());

    let params = PlantParams::default();
    let expected = 2.25 + 0.5 * params.flow(0.60) * params.volts_per_pu;
    let mean: f32 = rows.iter().map(|r| r[3]).sum::<f32>() / rows.len() as f32;
    assert!(
        (mean - expected).abs() < 5e-3,
        "settled at {mean} V, the model says {expected} V"
    );

    // The commands are recorded on the same scan as the output, which is the
    // whole point of wiring the filter outputs back into the SAADC.
    let u: f32 = rows.iter().map(|r| r[1]).sum::<f32>() / rows.len() as f32;
    // One code of the u_T ladder is 2.77 mV, so 2.55 V lands within 1.4 mV.
    assert!((u - 2.55).abs() < 5e-3, "u_T recorded as {u} V");

    // Quantisation is visible and bounded: an 8-bit duty through the filter.
    let adc = AdcScale::NOMINAL;
    assert!(
        rows.iter().all(|r| r[1] >= 2.25 && r[1] <= 2.75),
        "u_T left the safe window"
    );
    assert!(adc.volts_per_count() < 8e-4);
}

#[test]
fn a_step_moves_the_output_and_rings() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    rig.set_level(0, 2.55).unwrap();
    rig.set_level(1, 0.80).unwrap();

    // A +5 % step on the valve command, as the assignment prescribes: 5 % of
    // the 0.5 V window.
    rig.program(
        0,
        Waveform::Staircase {
            start: 2.55,
            step: 0.025,
            steps: 2,
            dwell_s: 1.0,
        },
    )
    .unwrap();
    rig.gen_start().unwrap();

    let (rows, _) = record(&mut rig, 6.0, "step");
    let before: f32 = rows[..500].iter().map(|r| r[3]).sum::<f32>() / 500.0;
    let after: f32 = rows[rows.len() - 500..].iter().map(|r| r[3]).sum::<f32>() / 500.0;
    assert!(
        after > before + 0.005,
        "the step did not move the output: {before} -> {after}"
    );

    // The rotor mode should be visible as an overshoot above the final value.
    let peak = rows.iter().map(|r| r[3]).fold(f32::MIN, f32::max);
    assert!(
        peak > after,
        "no overshoot: peak {peak}, settled {after} — the swing equation is not ringing"
    );
}

#[test]
fn the_output_filter_settles_inside_one_sample() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    rig.set_level(0, 2.30).unwrap();
    rig.set_level(1, 0.80).unwrap();
    rig.program(
        0,
        Waveform::Staircase {
            start: 2.30,
            step: 0.40,
            steps: 2,
            dwell_s: 0.5,
        },
    )
    .unwrap();
    rig.gen_start().unwrap();

    let (rows, _) = record(&mut rig, 1.5, "filter-edge");
    let fs = 1000usize;
    let lo = rows[fs / 4][1];
    let hi = rows[rows.len() - fs / 4][1];
    // 0.40 V asked for, give or take a code of the u_T stage at each end.
    assert!((hi - lo - 0.40).abs() < 9e-3, "the edge went {lo} -> {hi}");

    // The dominant pole is 164 µs and the rows are 1 ms apart, so a commanded
    // edge is over inside a single sample period. The tick that makes the
    // edge is not synchronous with the sample clock (as on the bench), so the
    // first row that moves can catch the edge anywhere along its way; the row
    // after it is at least a whole period past the edge — `1 - e^(-1000/164)`
    // = 99.8 % of the step. That is what the filter redesign bought, and the
    // reason a 100 Hz excitation is now limited by the 1 kHz waveform tick
    // rather than by the RC network.
    let moved = rows.iter().position(|r| r[1] > lo + 1e-3).unwrap();
    let next = (rows[moved + 1][1] - lo) / (hi - lo);
    assert!(
        next > 0.99,
        "one sample after the edge the output sits at {next:.3} of the step, expected ≈0.998"
    );
    // ...and it does not overshoot: two real poles cannot ring.
    let peak = rows.iter().map(|r| r[1]).fold(f32::MIN, f32::max);
    assert!(
        peak <= hi + 1e-3,
        "the filter overshot to {peak} against a final {hi}"
    );
}

#[test]
fn the_dc_sweep_recovers_the_calibration_the_outputs_were_given() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    let report = check::dc(
        &mut rig,
        &check::DcOptions {
            points: 6,
            settle: Duration::from_millis(100),
            average: Duration::from_millis(200),
            ..Default::default()
        },
    )
    .unwrap();

    assert_eq!(report.points.len(), 6);
    // Each output sweeps its own window: u_T starts at 2.25 V, code 37 on the
    // offset ladder, and p_s at 0 V.
    assert_eq!(
        report.points[0].codes,
        [
            DacScale::U_T_NOMINAL.to_code(DacScale::U_T_NOMINAL.min_v),
            0
        ]
    );
    for ch in 0..2 {
        let cal = report.calibration[ch].expect("both outputs were driven");
        // The sweep has to find the map the outputs were configured with; one
        // ADC count is 732 µV, so a straight line through six plateaus should
        // land well inside that per point.
        let want = DacScale::nominal(ch);
        let nominal = want.volts_per_code as f64;
        assert!(
            (cal.volts_per_code - nominal).abs() < 2e-5,
            "{ch}: {} V/code against a nominal {nominal}",
            cal.volts_per_code
        );
        assert!(
            (cal.offset_v - want.offset_v as f64).abs() < 2e-3,
            "{ch}: offset {}",
            cal.offset_v
        );
        assert!(
            cal.max_deviation_v < 8e-4,
            "{ch}: {} mV off the line",
            cal.max_deviation_v * 1000.0
        );
    }

    // A plateau is flat: what little spread there is is quantisation, not a
    // carrier the filter failed to remove.
    for point in &report.points {
        assert!(
            point.measured[0].span_v() < 1.5e-3,
            "plateau at {:?} V spans {} mV",
            point.commanded_v,
            point.measured[0].span_v() * 1000.0
        );
    }
}

#[test]
fn the_sine_check_finds_its_own_amplitude_and_no_skew_between_the_outputs() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    // `fs_hz` is 2 kHz against a simulator whose own default is 1 kHz, so this
    // also pins down that the rig samples at the rate it reports: at the wrong
    // spacing the fit at 10 Hz would find nothing at all.
    let report = check::sine(
        &mut rig,
        &check::SineOptions {
            // 0.2 V peak on both, each on the middle of its own window.
            amplitude_v: [0.2; 2],
            freq_hz: 10.0,
            duration: Duration::from_secs(2),
            settle: Duration::from_millis(200),
            fs_hz: 2000,
            ..Default::default()
        },
    )
    .unwrap();

    assert_eq!(report.fs_hz, 2000);
    assert_eq!(report.results.len(), 2);
    for r in &report.results {
        assert!(
            (r.fit.amplitude - 0.2).abs() < 5e-3,
            "{}: amplitude {}",
            check::name(r.channel),
            r.fit.amplitude
        );
        assert!(r.center_error_v.abs() < 2e-3, "{:?}", r);
        // Ten hertz is a hundredth of the tick, so almost everything in the
        // record is at the excitation frequency.
        assert!(r.fit.residual_ratio < 0.05, "{:?}", r);
    }

    // Both duties leave in one DMA transfer, so there is nothing to skew but
    // the reading: `p_s` is converted one scan slot after `u_T`.
    let skew = report.skew_deg.expect("two outputs were driven");
    assert!(
        (skew - report.scan_skew_deg()).abs() < 0.05,
        "{skew} degrees of skew between the outputs, the scan order explains {}",
        report.scan_skew_deg()
    );

    // And the measurement agrees with the model the design was signed off on.
    assert!(
        (report.results[0].gain_db - report.expected.gain_db).abs() < 0.1,
        "measured {:+.3} dB against a predicted {:+.3} dB",
        report.results[0].gain_db,
        report.expected.gain_db
    );
}

#[test]
fn every_experiment_file_is_loadable_and_inside_its_safe_window() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../experiments");
    let mut checked = 0;
    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("toml") {
            continue;
        }
        // Bode plans live next to the experiments but are a different file
        // format, read by `plant-trace bode`; they are checked in their own
        // test.
        if path
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("bode-"))
        {
            continue;
        }
        // `load` validates: every level and every waveform span has to sit
        // inside the window the file configures. Catching that here is what
        // stops a run from failing ten minutes in — or, worse, being clamped.
        Experiment::load(&path).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
        checked += 1;
    }
    assert!(
        checked >= 7,
        "only {checked} experiment files found in {dir:?}"
    );
}

#[test]
fn runs_an_experiment_and_writes_a_manifest() {
    let spec = start_sim(60.0);
    let out_dir = std::env::temp_dir().join(format!("plant-trace-run-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&out_dir);

    let experiment = Experiment::load(Path::new("tests/fixtures/quick-step.toml")).unwrap();
    let dir = plant_trace::runner::run(
        &experiment,
        &plant_trace::runner::RunConfig {
            daq: spec,
            daq_baud: 0,
            out_dir: Some(out_dir.clone()),
        },
    )
    .unwrap();

    let manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("run.json")).unwrap()).unwrap();
    assert_eq!(manifest["experiment"], "quick-step");
    let steps = manifest["steps"].as_array().unwrap();
    assert_eq!(steps.len(), 2);
    assert_eq!(steps[0]["settled"], true, "the plant never settled");
    // The second step's samples start where the first one's end: one stream,
    // segmented, with no gap where the transient is.
    let first_end =
        steps[0]["first_sample"].as_u64().unwrap() + steps[0]["samples"].as_u64().unwrap();
    assert_eq!(steps[1]["first_sample"].as_u64().unwrap(), first_end);

    // The recorded step is in the CSV, with its own time origin and a visible
    // change in both the command and the output.
    let csv = std::fs::read_to_string(dir.join("step-up.csv")).unwrap();
    let rows: Vec<Vec<f32>> = csv
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("t_s"))
        .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
        .collect();
    assert!(rows.len() >= 25_000, "only {} rows", rows.len());
    assert!(
        rows[0][0].abs() < 1e-6,
        "the segment clock does not start at 0"
    );

    // A 25 mV step is 9.0 codes of the u_T ladder (2.77 mV each), so what
    // arrives is nine of them, give or take one: 2.50 V and 2.525 V round to
    // codes 126 and 135 here.
    let before = rows[1000][4];
    let after = rows[rows.len() - 1][4];
    assert!(
        (after - before - 0.025).abs() <= DacScale::U_T_NOMINAL.volts_per_code + 1e-3,
        "u_T went {before} -> {after}, expected a +0.025 V step to within a code"
    );
    let p_before = rows[1000][6];
    let p_after = rows[rows.len() - 1][6];
    assert!(
        p_after > p_before + 0.005,
        "P_e did not follow the step: {p_before} -> {p_after}"
    );
}

#[test]
fn analyses_a_run_into_the_numbers_the_model_predicts() {
    let spec = start_sim(60.0);
    let out_dir = std::env::temp_dir().join(format!("plant-trace-analyze-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&out_dir);

    let experiment = Experiment::load(Path::new("tests/fixtures/quick-step.toml")).unwrap();
    let dir = plant_trace::runner::run(
        &experiment,
        &plant_trace::runner::RunConfig {
            daq: spec,
            daq_baud: 0,
            out_dir: Some(out_dir),
        },
    )
    .unwrap();

    // No plots: gnuplot is not a test dependency.
    let summary = plant_trace::analysis::report::analyze_run(&dir, false).unwrap();
    assert_eq!(summary.steps, 1, "the step segment was not analysed");

    let table = std::fs::read_to_string(dir.join("analysis/steps.csv")).unwrap();
    let row: Vec<&str> = table.lines().nth(1).unwrap().split(',').collect();
    let gain: f64 = row[2].parse().unwrap();
    let delay: f64 = row[3].parse().unwrap();

    // The step runs from 2.50 V to 2.525 V, which the simulator hands the
    // model as 0.50 to 0.55, so the gain it measures is the valve's slope at
    // the midpoint — which the model can state exactly. Both ends of the
    // window mapping scale by the same 0.5 V, so the box's V/V is the
    // model's.
    let params = PlantParams::default();
    let analytic = params.flow_gain(0.525) as f64;
    assert!(
        (gain - analytic).abs() / analytic < 0.10,
        "measured gain {gain:.3} V/V against the model's {analytic:.3} V/V"
    );
    // And the apparent delay is bounded by the transport delay from below.
    assert!(
        delay > params.valve_delay_s as f64,
        "apparent delay {delay:.3} s is below the model's transport delay"
    );
}

#[test]
fn a_stale_reply_is_dropped_instead_of_shifting_every_later_one() {
    let spec = start_sim(40.0);
    let mut rig = Daq::open(&spec, 0).unwrap();
    // A request whose reply nobody waited for — what a timed-out request, or
    // a session killed mid-exchange, leaves behind on the link.
    rig.send(plant_trace_proto::daq::HostToDaq::Gen(
        plant_trace_proto::gen::HostToGen::Info,
    ))
    .unwrap();
    // The next requests still get their own answers, in order.
    assert_eq!(rig.info().unwrap().channels, 3);
    assert_eq!(rig.gen_info().unwrap().outputs, 2);
    rig.set_level(0, 2.5).unwrap();
    assert!(rig.gen_status().unwrap().volts[0] > 2.4);
}

#[test]
fn opening_stops_a_stream_a_dead_session_left_running() {
    let spec = start_sim(40.0);
    {
        let mut first = Daq::open(&spec, 0).unwrap();
        first.start(1000).unwrap();
        // Dropped mid-stream, with blocks and no Stop.
    }
    let mut second = Daq::open(&spec, 0).unwrap();
    assert_eq!(second.info().unwrap().channels, 3);
    assert_eq!(second.start(1000).unwrap(), 1000);
    second.stop().unwrap();
}
