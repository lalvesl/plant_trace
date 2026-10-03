//! The thread that owns the rig.
//!
//! A serial port has one owner, and every exchange with the rig is a blocking
//! request/reply on it, so the link lives on its own thread and the UI talks
//! to it by message. The UI never waits on the rig: it sends a [`Cmd`], keeps
//! painting, and folds [`Evt`]s into its state as they arrive.
//!
//! Long jobs (a DC sweep, an experiment, a Bode sweep) run on this thread too.
//! They are the same library functions the command line calls, handed an
//! observer that forwards samples and progress here instead of printing them.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, RecvTimeoutError, Sender},
        Arc,
    },
    thread::JoinHandle,
    time::Duration,
};

use anyhow::{Context as _, Result};
use plant_trace_proto::{gen::N_OUTPUTS, scale::AdcScale, waveform::Waveform};

use crate::{
    check::{self, DcOptions, DcReport, SineOptions, SineReport},
    daq::{Daq, Event, GenInfo, Info},
    link::DEFAULT_BAUD,
};

use super::jobs::{self, JobOutcome, JobSpec};

/// What the UI asks the rig thread to do.
pub enum Cmd {
    /// Open a link (`/dev/ttyACM0`, `tcp://host:port`).
    Connect(String),
    /// Close the link.
    Disconnect,
    /// Stream continuously for the live view, or stop.
    Monitor {
        /// Stream or not.
        on: bool,
        /// Requested sample rate; 0 is the firmware default.
        fs_hz: u32,
    },
    /// Hold an output at a level.
    SetLevel {
        /// Output index.
        ch: u8,
        /// Volts at the plant input.
        volts: f32,
    },
    /// Both outputs to their minimum.
    Park,
    /// Run a free-running waveform on each output that has one, both started
    /// on the same tick. Whatever was running before is stopped first; an
    /// output given `None` holds where it is.
    Generate([Option<Waveform>; N_OUTPUTS]),
    /// Stop the generator and hold each output given a level at it.
    GenStop([Option<f32>; N_OUTPUTS]),
    /// DC sweep of the outputs.
    CheckDc(DcOptions),
    /// Sine readback of the outputs.
    CheckSine(SineOptions),
    /// A long job with live data: an experiment or a Bode sweep.
    Job(JobSpec),
}

/// What the rig thread reports back.
pub enum Evt {
    /// A link is open.
    Connected {
        /// The spec it was opened with.
        spec: String,
        /// Acquisition description.
        info: Info,
        /// Output description.
        outputs: GenInfo,
    },
    /// The link is closed, with the reason if it was not asked for.
    Disconnected(Option<String>),
    /// Streaming state changed.
    Streaming(Option<u32>),
    /// The free-running generator started (`true`) or stopped (`false`).
    Generating(bool),
    /// Rows of `[u_t, p_s, p_e]` in volts, starting at stream index `first`.
    Samples {
        /// Stream index of the first row.
        first: u64,
        /// Sample rate.
        fs_hz: u32,
        /// The rows.
        rows: Vec<[f32; 3]>,
    },
    /// A job began; the label says which.
    Busy(String),
    /// Progress text from the running job.
    Status(String),
    /// A DC sweep ended.
    DcDone(Result<DcReport, String>),
    /// A sine check ended.
    SineDone(Result<SineReport, String>),
    /// A long job produced something worth showing before it ends.
    Job(jobs::JobEvent),
    /// A long job ended.
    JobDone(Result<JobOutcome, String>),
    /// A command failed; the text says why.
    Error(String),
}

/// The UI's end of the rig thread.
pub struct Worker {
    tx: Sender<Cmd>,
    rx: Receiver<Evt>,
    cancel: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Worker {
    /// Start the thread. `wake` is called after every event so the UI repaints.
    pub fn spawn(wake: impl Fn() + Send + 'static) -> Self {
        let (tx, cmd_rx) = mpsc::channel();
        let (evt_tx, rx) = mpsc::channel();
        let cancel = Arc::new(AtomicBool::new(false));
        let thread = {
            let cancel = cancel.clone();
            std::thread::Builder::new()
                .name("rig".into())
                .spawn(move || {
                    Rig {
                        cmds: cmd_rx,
                        out: Out {
                            tx: evt_tx,
                            wake: Box::new(wake),
                        },
                        cancel,
                        link: None,
                        monitor: None,
                    }
                    .run()
                })
                .expect("spawning the rig thread")
        };
        Self {
            tx,
            rx,
            cancel,
            thread: Some(thread),
        }
    }

    /// Queue a command.
    pub fn send(&self, cmd: Cmd) {
        if matches!(cmd, Cmd::Job(_) | Cmd::CheckDc(_) | Cmd::CheckSine(_)) {
            self.cancel.store(false, Ordering::Relaxed);
        }
        let _ = self.tx.send(cmd);
    }

    /// Ask the running job to stop at its next opportunity.
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Everything that arrived since the last call.
    pub fn drain(&self) -> Vec<Evt> {
        self.rx.try_iter().collect()
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
        let _ = self.tx.send(Cmd::Disconnect);
        // Closing the channel ends the loop; the thread parks the outputs on
        // its way out.
        let (tx, _) = mpsc::channel();
        drop(std::mem::replace(&mut self.tx, tx));
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// The event sink, with the UI's wake-up attached.
pub(crate) struct Out {
    tx: Sender<Evt>,
    wake: Box<dyn Fn() + Send>,
}

impl Out {
    pub(crate) fn send(&self, evt: Evt) {
        let _ = self.tx.send(evt);
        (self.wake)();
    }
}

struct Link {
    daq: Daq,
    spec: String,
    adc: AdcScale,
}

struct Rig {
    cmds: Receiver<Cmd>,
    out: Out,
    cancel: Arc<AtomicBool>,
    link: Option<Link>,
    /// While the live view streams: the rate asked for, and the one granted.
    monitor: Option<(u32, u32)>,
}

impl Rig {
    fn run(mut self) {
        loop {
            // Streaming: poll the link between commands. Idle: block on the
            // command channel, with a timeout so a dead link is noticed.
            let cmd = if self.monitor.is_some() && self.link.is_some() {
                match self.cmds.try_recv() {
                    Ok(c) => Some(c),
                    Err(mpsc::TryRecvError::Empty) => {
                        self.pump();
                        None
                    }
                    Err(mpsc::TryRecvError::Disconnected) => break,
                }
            } else {
                match self.cmds.recv_timeout(Duration::from_millis(200)) {
                    Ok(c) => Some(c),
                    Err(RecvTimeoutError::Timeout) => None,
                    Err(RecvTimeoutError::Disconnected) => break,
                }
            };
            if let Some(cmd) = cmd {
                if let Err(e) = self.handle(cmd) {
                    self.out.send(Evt::Error(format!("{e:#}")));
                }
            }
        }
        self.close(None);
    }

    fn handle(&mut self, cmd: Cmd) -> Result<()> {
        match cmd {
            Cmd::Connect(spec) => {
                self.close(None);
                let mut daq =
                    Daq::open(&spec, DEFAULT_BAUD).with_context(|| format!("opening {spec}"))?;
                let info = daq.info()?;
                anyhow::ensure!(
                    info.protocol == plant_trace_proto::PROTOCOL_VERSION,
                    "the rig speaks protocol v{}, this build speaks v{}",
                    info.protocol,
                    plant_trace_proto::PROTOCOL_VERSION
                );
                let outputs = daq.gen_info()?;
                let adc = info.adc_scale();
                self.link = Some(Link {
                    daq,
                    spec: spec.clone(),
                    adc,
                });
                self.out.send(Evt::Connected {
                    spec,
                    info,
                    outputs,
                });
            }
            Cmd::Disconnect => self.close(None),
            Cmd::Monitor { on, fs_hz } => {
                let link = self.link.as_mut().context("not connected")?;
                if on {
                    if self.monitor.is_some() {
                        let _ = link.daq.stop();
                    }
                    let fs = link.daq.start(fs_hz)?;
                    self.monitor = Some((fs_hz, fs));
                    self.out.send(Evt::Streaming(Some(fs)));
                } else if self.monitor.take().is_some() {
                    let _ = link.daq.stop();
                    self.out.send(Evt::Streaming(None));
                }
            }
            Cmd::SetLevel { ch, volts } => {
                self.link
                    .as_mut()
                    .context("not connected")?
                    .daq
                    .set_level(ch, volts)?;
            }
            Cmd::Park => {
                self.link.as_mut().context("not connected")?.daq.park()?;
                self.out.send(Evt::Generating(false));
            }
            Cmd::Generate(waves) => {
                let daq = &mut self.link.as_mut().context("not connected")?.daq;
                // Program refuses nothing while a waveform runs, but Start
                // would restart the clock of an output left out; stopping
                // first makes every Generate a clean start.
                daq.gen_stop()?;
                for (ch, wave) in waves.iter().enumerate() {
                    if let Some(w) = wave {
                        daq.program(ch as u8, *w)
                            .with_context(|| format!("programming {}", check::name(ch as u8)))?;
                    }
                }
                daq.gen_start()?;
                self.out.send(Evt::Generating(true));
            }
            Cmd::GenStop(hold) => {
                let daq = &mut self.link.as_mut().context("not connected")?.daq;
                daq.gen_stop()?;
                // Stop freezes an output mid-cycle; put it back on its centre
                // line rather than wherever the tick left it.
                for (ch, level) in hold.iter().enumerate() {
                    if let Some(v) = level {
                        daq.set_level(ch as u8, *v)?;
                    }
                }
                self.out.send(Evt::Generating(false));
            }
            Cmd::CheckDc(opts) => {
                let result = self.exclusive("DC sweep", |daq, _, _| check::dc(daq, &opts));
                self.out
                    .send(Evt::DcDone(result.map_err(|e| format!("{e:#}"))));
            }
            Cmd::CheckSine(opts) => {
                let result = self.exclusive("sine check", |daq, _, _| check::sine(daq, &opts));
                self.out
                    .send(Evt::SineDone(result.map_err(|e| format!("{e:#}"))));
            }
            Cmd::Job(spec) => {
                let label = spec.label();
                let result =
                    self.exclusive(&label, |daq, out, cancel| jobs::run(daq, spec, out, cancel));
                self.out
                    .send(Evt::JobDone(result.map_err(|e| format!("{e:#}"))));
            }
        }
        Ok(())
    }

    /// Run `f` with the link to itself: the live view is paused around it,
    /// because every job starts and stops its own stream.
    fn exclusive<T>(
        &mut self,
        label: &str,
        f: impl FnOnce(&mut Daq, &Out, &AtomicBool) -> Result<T>,
    ) -> Result<T> {
        let link = self.link.as_mut().context("not connected")?;
        let resume = self.monitor.take();
        if resume.is_some() {
            let _ = link.daq.stop();
            self.out.send(Evt::Streaming(None));
        }
        self.out.send(Evt::Busy(label.to_string()));
        let result = f(&mut link.daq, &self.out, &self.cancel);
        if let Some((fs_hz, _)) = resume {
            match link.daq.start(fs_hz) {
                Ok(fs) => {
                    self.monitor = Some((fs_hz, fs));
                    self.out.send(Evt::Streaming(Some(fs)));
                }
                Err(e) => self
                    .out
                    .send(Evt::Error(format!("resuming the live view: {e:#}"))),
            }
        }
        result
    }

    /// Move whatever blocks have arrived to the UI.
    fn pump(&mut self) {
        let Some(link) = self.link.as_mut() else {
            return;
        };
        let fs = self.monitor.map_or(0, |(_, granted)| granted);
        match link.daq.next_event(Duration::from_millis(30)) {
            Ok(Some(Event::Block {
                seq,
                n,
                channels,
                counts,
                ..
            })) => {
                let rows = to_rows(&link.adc, n, channels, &counts);
                self.out.send(Evt::Samples {
                    first: seq as u64 * n as u64,
                    fs_hz: fs,
                    rows,
                });
            }
            Ok(_) => {}
            Err(e) => {
                let spec = link.spec.clone();
                self.close(Some(format!("{spec}: {e:#}")));
            }
        }
    }

    fn close(&mut self, reason: Option<String>) {
        if let Some(mut link) = self.link.take() {
            if self.monitor.take().is_some() {
                let _ = link.daq.stop();
            }
            // Leave the plant at rest, whatever the UI was doing.
            let _ = link.daq.gen_stop();
            let _ = link.daq.park();
            self.out.send(Evt::Streaming(None));
            self.out.send(Evt::Disconnected(reason));
        }
    }
}

/// Interleaved counts to rows of volts.
pub(crate) fn to_rows(adc: &AdcScale, n: u16, channels: u8, counts: &[i16]) -> Vec<[f32; 3]> {
    let ch = channels as usize;
    (0..n as usize)
        .map(|i| {
            let mut row = [0.0f32; 3];
            for (c, v) in row.iter_mut().enumerate().take(ch.min(3)) {
                *v = adc.to_volts(counts[i * ch + c]);
            }
            row
        })
        .collect()
}
