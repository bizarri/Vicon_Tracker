mod cli;
mod ekf;
mod frames;
mod mav_out;
mod source;
mod state;
#[cfg(feature = "vicon")]
mod vicon_source;

use std::sync::mpsc::{self, RecvTimeoutError, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

#[cfg(not(feature = "vicon"))]
use anyhow::bail;
use anyhow::{Context, Result};
use clap::Parser;
use nalgebra::Vector3;

use cli::{Autopilot, Cli, MessageKind, SourceKind};
use ekf::{Ekf, EkfConfig, Step};
use frames::Converter;
use mav_out::{MavSender, MsgKind, PoseMsg, host_time_us};
use source::{MockSource, PoseSource, RawSample};
use state::{TrackingState, TrackingStateMachine};

const ORIGIN_RESEND: Duration = Duration::from_secs(10);
const STATS_INTERVAL: Duration = Duration::from_secs(2);

macro_rules! trace {
    ($self:expr, $($arg:tt)*) => {
        if $self.trace {
            eprintln!($($arg)*);
        }
    };
}

/// Everything that tracks the drone over time, so the receive loop stays simple.
struct Tracker {
    ekf: Ekf,
    converter: Converter,
    gate: f64,
    max_coast: f64,
    ang_var: f64,
    /// Time of the most recent valid (non-occluded, non-rejected) measurement.
    last_valid: Option<f64>,
    needs_reinit: bool,
    reset_counter: u8,
    /// Latest input-frame attitude from Vicon (x, y, z, w).
    latest_att: Option<[f64; 4]>,
    trace: bool,
    state_machine: TrackingStateMachine,
}

impl Tracker {
    /// Transition the state machine and log the change with the true prior state.
    fn set_state(&mut self, new_state: TrackingState, reason: &str) {
        let from = self.state_machine.state();
        if self.state_machine.transition(new_state) {
            eprintln!(
                "tracker: state changed {} -> {} ({reason})",
                from.as_str(),
                new_state.as_str()
            );
        }
    }

    fn handle(&mut self, s: RawSample) {
        trace!(
            self,
            "[TRACE][RCV] t={:.4}s occl={} pos_m=[{:.3},{:.3},{:.3}] has_quat={}",
            s.t,
            s.occluded,
            s.pos_m[0],
            s.pos_m[1],
            s.pos_m[2],
            s.quat_xyzw.is_some()
        );
        if self.state_machine.state() == TrackingState::Idle {
            self.set_state(TrackingState::Searching, "first sample");
        }
        if s.occluded {
            trace!(self, "[TRACE][EKF] occluded -> skip");
            return;
        }
        let p = self.converter.pos_to_ned(s.pos_m);
        trace!(
            self,
            "[TRACE][FRM] enu->ned pos_ned=[{:.3},{:.3},{:.3}]", p[0], p[1], p[2]
        );
        if self.needs_reinit || !self.ekf.is_initialized() {
            let after_outage = self.last_valid.is_some();
            self.ekf.init(s.t, p);
            self.needs_reinit = false;
            if after_outage {
                self.reset_counter = self.reset_counter.wrapping_add(1);
                eprintln!(
                    "tracker: measurement restored after outage, EKF re-initialized (reset_counter={})",
                    self.reset_counter
                );
                trace!(
                    self,
                    "[TRACE][EKF] Reinit after outage (reset_counter={})", self.reset_counter
                );
                self.set_state(TrackingState::Tracking, "recovery");
            } else {
                trace!(self, "[TRACE][EKF] Initialized");
                self.set_state(TrackingState::Tracking, "EKF initialized");
            }
        } else {
            match self.ekf.process(s.t, p, self.gate) {
                Step::Stale => {
                    trace!(self, "[TRACE][EKF] Stale (dt<0) -> skip");
                    return;
                }
                Step::Rejected => {
                    trace!(
                        self,
                        "[TRACE][EKF] Rejected by gate (sigma>{}) -> skip", self.gate
                    );
                    return;
                }
                Step::Coasted => {
                    trace!(self, "[TRACE][EKF] Coasted");
                    self.set_state(TrackingState::Tracking, "coasting within max_coast");
                }
                Step::Initialized => {
                    trace!(self, "[TRACE][EKF] Initialized");
                    self.set_state(TrackingState::Tracking, "EKF initialized");
                }
                Step::Updated => {
                    trace!(self, "[TRACE][EKF] Updated");
                    self.set_state(TrackingState::Tracking, "measurement update");
                }
            }
        }
        self.last_valid = Some(s.t);
        if let Some(q) = s.quat_xyzw {
            self.latest_att = Some(q);
        }
    }

    /// Coasting beyond `max_coast`: mark for re-init and report a transition.
    fn update_coast_state(&mut self, now: f64) {
        let Some(lv) = self.last_valid else {
            return;
        };
        if now - lv > self.max_coast && !self.needs_reinit {
            eprintln!(
                "tracker: no valid measurement for {:.2}s, pausing MAVLink output",
                now - lv
            );
            trace!(
                self,
                "[TRACE][COAST] exceeded max_coast ({:.2}s > {:.2}s) -> pausing",
                now - lv,
                self.max_coast
            );
            self.set_state(TrackingState::Lost, "coast timeout");
            self.needs_reinit = true;
        }
    }

    fn build_pose(&mut self, now: f64) -> PoseMsg {
        self.ekf.predict_to(now);
        let st = self.ekf.state();
        let att = self.converter.attitude_ned(self.latest_att);
        let v_body = att.rot.transpose() * Vector3::new(st[3], st[4], st[5]);
        trace!(
            self,
            "[TRACE][POSE] now={:.4}s pred pos=[{:.3},{:.3},{:.3}] vel_ned=[{:.3},{:.3},{:.3}] vel_body_frd=[{:.3},{:.3},{:.3}] rpy=[{:.3},{:.3},{:.3}] reset={}",
            now,
            st[0],
            st[1],
            st[2],
            st[3],
            st[4],
            st[5],
            v_body.x,
            v_body.y,
            v_body.z,
            att.rpy[0],
            att.rpy[1],
            att.rpy[2],
            self.reset_counter
        );
        PoseMsg {
            time_us: host_time_us(),
            pos_ned: [st[0], st[1], st[2]],
            vel_body_frd: [v_body.x, v_body.y, v_body.z],
            rpy: att.rpy,
            quat_wxyz: att.quat_wxyz,
            pos_var: self.ekf.pos_var(),
            vel_var: self.ekf.vel_var(),
            ang_var: self.ang_var,
            reset_counter: self.reset_counter,
            quality: 100,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let origin = parse_origin(cli.origin.as_deref())?;
    let send_period = 1.0 / cli.send_hz.max(1.0);
    let ang_var = {
        let a = cli.ang_noise_deg.to_radians();
        a * a
    };

    let converter = Converter::new(
        cli.input_frame,
        cli.body_frame,
        cli.attitude,
        cli.yaw_offset_deg,
    );
    let msg_kind = match cli.message {
        MessageKind::Auto => match cli.autopilot {
            Autopilot::Ardupilot => MsgKind::Odometry,
            Autopilot::Px4 => MsgKind::Vision,
        },
        MessageKind::Odometry => MsgKind::Odometry,
        MessageKind::Vision => MsgKind::Vision,
    };
    let mut source = make_source(&cli)?;
    let mut sender = MavSender::connect(&cli.mavlink, msg_kind, cli.mav_sysid, cli.mav_compid)?;

    let (tx, rx) = mpsc::channel::<RawSample>();
    thread::Builder::new()
        .name("pose-source".into())
        .spawn(move || {
            while let Some(s) = source.next_sample() {
                if tx.send(s).is_err() {
                    break;
                }
            }
        })
        .context("spawning pose source thread")?;

    let t0 = Instant::now();
    let mut tracker = Tracker {
        ekf: Ekf::new(EkfConfig {
            pos_noise_m: (cli.pos_noise_mm / 1000.0).max(1e-6),
            accel_noise: cli.accel_noise.max(1e-3),
            vel_init_sigma: 1.0,
        }),
        converter,
        gate: cli.gate_sigma.max(0.0),
        max_coast: cli.max_coast_secs.max(0.0),
        ang_var,
        last_valid: None,
        needs_reinit: false,
        reset_counter: 0,
        latest_att: None,
        trace: cli.trace,
        state_machine: TrackingStateMachine::new(),
    };

    let mut next_due = send_period;
    let mut sent: u64 = 0;
    let mut last_send_err: Option<Instant> = None;
    let mut last_stats = Instant::now();
    let mut last_origin: Option<Instant> = None;
    let mut logged_first_send = false;

    loop {
        let now = t0.elapsed().as_secs_f64();
        let timeout = Duration::from_secs_f64((next_due - now).clamp(0.0005, 0.1));
        match rx.recv_timeout(timeout) {
            Ok(s) => {
                tracker.handle(s);
                loop {
                    match rx.try_recv() {
                        Ok(s) => tracker.handle(s),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => return Ok(()),
                    }
                }
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        let now = t0.elapsed().as_secs_f64();
        let now_i = Instant::now();

        // Periodic origin/home so a GPS-less EKF has a global reference.
        if let Some((lat, lon, alt)) = origin
            && last_origin
                .map(|t| t.elapsed() >= ORIGIN_RESEND)
                .unwrap_or(true)
        {
            trace!(
                &tracker,
                "[TRACE][MAV] send_origin lat={:.6} lon={:.6} alt={:.3}", lat, lon, alt
            );
            if let Err(e) = sender.send_origin(lat, lon, alt, host_time_us()) {
                eprintln!("tracker: origin send failed: {e:#}");
            }
            last_origin = Some(now_i);
        }

        if now >= next_due {
            next_due += send_period;
            while next_due <= now {
                next_due += send_period;
            }

            tracker.update_coast_state(now);
            let can_send = tracker
                .last_valid
                .is_some_and(|lv| now - lv <= tracker.max_coast)
                && tracker.ekf.is_initialized();
            let initialized = tracker.ekf.is_initialized();
            trace!(
                &tracker,
                "[TRACE][LOOP] now={:.4}s next_due={:.4}s has_valid={} initialized={} can_send={}",
                now,
                next_due,
                tracker.last_valid.is_some(),
                initialized,
                can_send
            );
            if can_send {
                let pose = tracker.build_pose(now);
                match sender.send_pose(&pose) {
                    Ok(_) => {
                        sent += 1;
                        trace!(&tracker, "[TRACE][MAV] pose sent (sent={})", sent);
                        if !logged_first_send {
                            eprintln!("tracker: streaming {:?} to flight controller", msg_kind);
                            logged_first_send = true;
                        }
                    }
                    Err(e) => {
                        let due = last_send_err
                            .map(|t| t.elapsed() >= Duration::from_secs(2))
                            .unwrap_or(true);
                        if due {
                            eprintln!("tracker: send failed: {e:#}");
                            last_send_err = Some(Instant::now());
                        }
                    }
                }
            }
        }

        if last_stats.elapsed() >= STATS_INTERVAL {
            last_stats = Instant::now();
            let age = match tracker.last_valid {
                Some(lv) => format!("{:.0}ms", (now - lv) * 1000.0),
                None if tracker.ekf.is_initialized() => "n/a".into(),
                None => "waiting".into(),
            };
            if tracker.ekf.is_initialized() {
                let st = tracker.ekf.state();
                eprintln!(
                    "tracker: pos=({:.2},{:.2},{:.2}) m vel=({:.2},{:.2},{:.2}) m/s meas_age={age} sent={sent}",
                    st[0], st[1], st[2], st[3], st[4], st[5]
                );
            } else {
                eprintln!("tracker: waiting for first measurement (sent={sent})");
            }
        }
    }
    Ok(())
}

#[cfg(feature = "vicon")]
fn make_source(cli: &Cli) -> Result<Box<dyn PoseSource>> {
    match cli.source {
        SourceKind::Mock => Ok(Box::new(MockSource::new(
            cli.mock_hz,
            cli.pos_noise_mm / 1000.0,
            cli.mock_dropout,
        ))),
        SourceKind::Vicon => {
            let subject = cli
                .subject
                .as_deref()
                .context("--subject is required with --source vicon")?;
            Ok(Box::new(vicon_source::ViconSource::connect(
                &cli.vicon_host,
                subject,
                cli.segment.as_deref(),
            )?))
        }
    }
}

#[cfg(not(feature = "vicon"))]
fn make_source(cli: &Cli) -> Result<Box<dyn PoseSource>> {
    match cli.source {
        SourceKind::Mock => Ok(Box::new(MockSource::new(
            cli.mock_hz,
            cli.pos_noise_mm / 1000.0,
            cli.mock_dropout,
        ))),
        SourceKind::Vicon => bail!(
            "this binary was built without the `vicon` feature; rebuild with \
             `cargo build --release --features vicon`"
        ),
    }
}

fn parse_origin(s: Option<&str>) -> Result<Option<(f64, f64, f64)>> {
    let Some(s) = s else {
        return Ok(None);
    };
    let parts: Vec<&str> = s.split(',').map(str::trim).collect();
    anyhow::ensure!(
        parts.len() == 3,
        "--origin must be \"latitude,longitude,altitude_m\""
    );
    let lat: f64 = parts[0].parse().context("invalid --origin latitude")?;
    let lon: f64 = parts[1].parse().context("invalid --origin longitude")?;
    let alt: f64 = parts[2].parse().context("invalid --origin altitude")?;
    anyhow::ensure!(
        (-90.0..=90.0).contains(&lat),
        "--origin latitude out of range"
    );
    anyhow::ensure!(
        (-180.0..=180.0).contains(&lon),
        "--origin longitude out of range"
    );
    Ok(Some((lat, lon, alt)))
}
