use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
#[allow(deprecated)]
use mavlink::dialects::common::{
    MavEstimatorType, MavFrame, MavMessage, ODOMETRY_DATA, SET_GPS_GLOBAL_ORIGIN_DATA,
    SET_HOME_POSITION_DATA, VISION_POSITION_ESTIMATE_DATA,
};
use mavlink::{Connection, MavConnection, MavHeader};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MsgKind {
    /// Preferred by ArduPilot: position + velocity + attitude, NED pose / body velocity
    Odometry,
    /// Position + attitude; accepted by both ArduPilot and PX4
    Vision,
}

/// Pose to publish, already in MAVLink conventions (NED metres, body FRD).
#[derive(Debug, Clone, Copy)]
pub struct PoseMsg {
    /// Host time, microseconds since system boot (or UNIX epoch — receivers infer)
    pub time_us: u64,
    pub pos_ned: [f64; 3],
    pub vel_body_frd: [f64; 3],
    /// roll, pitch, yaw (rad)
    pub rpy: [f64; 3],
    /// body -> NED, w x y z
    pub quat_wxyz: [f64; 4],
    pub pos_var: [f64; 3],
    pub vel_var: [f64; 3],
    pub ang_var: f64,
    pub reset_counter: u8,
    /// 100 = best quality, -1 = failed (ODOMETRY only)
    pub quality: i8,
}

pub struct MavSender {
    conn: Connection<MavMessage>,
    kind: MsgKind,
    sysid: u8,
    compid: u8,
    sequence: u8,
}

impl MavSender {
    pub fn connect(addr: &str, kind: MsgKind, sysid: u8, compid: u8) -> Result<Self> {
        let conn: Connection<MavMessage> =
            mavlink::connect(addr).with_context(|| format!("connecting to MAVLink {addr}"))?;
        eprintln!(
            "mavlink: sending {:?} to {addr} as sysid {sysid} compid {compid}",
            kind.name()
        );
        Ok(Self {
            conn,
            kind,
            sysid,
            compid,
            sequence: 0,
        })
    }

    pub fn send_pose(&mut self, p: &PoseMsg) -> Result<usize> {
        let msg = match self.kind {
            MsgKind::Odometry => MavMessage::ODOMETRY(self.odometry(p)),
            MsgKind::Vision => MavMessage::VISION_POSITION_ESTIMATE(self.vision(p)),
        };
        let header = MavHeader {
            system_id: self.sysid,
            component_id: self.compid,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        self.conn.send(&header, &msg).context("sending pose")
    }

    /// Send SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION (needed when flying without GPS).
    ///
    /// Marked `deprecated` in recent MAVLink definitions in favour of MAV_CMD
    /// equivalents, but these messages remain the documented mechanism for
    /// ArduPilot/PX4 EKF origin setup.
    #[allow(deprecated)]
    pub fn send_origin(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
        alt_m: f64,
        time_us: u64,
    ) -> Result<()> {
        let latitude = (lat_deg * 1e7).round() as i32;
        let longitude = (lon_deg * 1e7).round() as i32;
        let altitude = (alt_m * 1000.0).round() as i32;
        let origin = MavMessage::SET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN_DATA {
            latitude,
            longitude,
            altitude,
            target_system: 0,
            time_usec: time_us,
        });
        let home = MavMessage::SET_HOME_POSITION(SET_HOME_POSITION_DATA {
            latitude,
            longitude,
            altitude,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            q: [1.0, 0.0, 0.0, 0.0],
            approach_x: 1.0,
            approach_y: 0.0,
            approach_z: 0.0,
            target_system: 0,
            time_usec: time_us,
        });
        self.send_raw(&origin)?;
        self.send_raw(&home)?;
        Ok(())
    }

    fn send_raw(&mut self, msg: &MavMessage) -> Result<usize> {
        let header = MavHeader {
            system_id: self.sysid,
            component_id: self.compid,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        self.conn
            .send(&header, msg)
            .context("sending MAVLink message")
    }

    fn odometry(&self, p: &PoseMsg) -> ODOMETRY_DATA {
        let mut pose_cov = [0.0f32; 21];
        pose_cov[0] = p.pos_var[0] as f32;
        pose_cov[6] = p.pos_var[1] as f32;
        pose_cov[11] = p.pos_var[2] as f32;
        pose_cov[15] = p.ang_var as f32;
        pose_cov[18] = p.ang_var as f32;
        pose_cov[20] = p.ang_var as f32;
        let mut vel_cov = [0.0f32; 21];
        vel_cov[0] = p.vel_var[0] as f32;
        vel_cov[6] = p.vel_var[1] as f32;
        vel_cov[11] = p.vel_var[2] as f32;
        ODOMETRY_DATA {
            time_usec: p.time_us,
            frame_id: MavFrame::MAV_FRAME_LOCAL_FRD,
            child_frame_id: MavFrame::MAV_FRAME_BODY_FRD,
            x: p.pos_ned[0] as f32,
            y: p.pos_ned[1] as f32,
            z: p.pos_ned[2] as f32,
            q: [
                p.quat_wxyz[0] as f32,
                p.quat_wxyz[1] as f32,
                p.quat_wxyz[2] as f32,
                p.quat_wxyz[3] as f32,
            ],
            vx: p.vel_body_frd[0] as f32,
            vy: p.vel_body_frd[1] as f32,
            vz: p.vel_body_frd[2] as f32,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
            pose_covariance: pose_cov,
            velocity_covariance: vel_cov,
            reset_counter: p.reset_counter,
            estimator_type: MavEstimatorType::MAV_ESTIMATOR_TYPE_MOCAP,
            quality: p.quality,
        }
    }

    fn vision(&self, p: &PoseMsg) -> VISION_POSITION_ESTIMATE_DATA {
        let mut cov = [0.0f32; 21];
        cov[0] = p.pos_var[0] as f32;
        cov[6] = p.pos_var[1] as f32;
        cov[11] = p.pos_var[2] as f32;
        cov[15] = p.ang_var as f32;
        cov[18] = p.ang_var as f32;
        cov[20] = p.ang_var as f32;
        VISION_POSITION_ESTIMATE_DATA {
            usec: p.time_us,
            x: p.pos_ned[0] as f32,
            y: p.pos_ned[1] as f32,
            z: p.pos_ned[2] as f32,
            roll: p.rpy[0] as f32,
            pitch: p.rpy[1] as f32,
            yaw: p.rpy[2] as f32,
            covariance: cov,
            reset_counter: p.reset_counter,
        }
    }
}

impl MsgKind {
    pub fn name(&self) -> &'static str {
        match self {
            MsgKind::Odometry => "ODOMETRY",
            MsgKind::Vision => "VISION_POSITION_ESTIMATE",
        }
    }
}

/// Microseconds since system boot, cached. Falls back to time since the first
/// call if the OS boot time is unavailable. The exact time base does not
/// matter to receivers (ArduPilot/PX4 infer it from the first message); it only
/// needs to be consistent and monotonic within a run.
pub fn host_time_us() -> u64 {
    use std::sync::OnceLock;
    static BOOT_EPOCH_US: OnceLock<u64> = OnceLock::new();
    let now = epoch_us();
    let boot = *BOOT_EPOCH_US.get_or_init(|| match system_uptime_us() {
        Some(u) => now.saturating_sub(u),
        None => now,
    });
    now.saturating_sub(boot)
}

/// OS uptime in microseconds, or `None` if it cannot be determined.
#[cfg(target_os = "linux")]
fn system_uptime_us() -> Option<u64> {
    let s = std::fs::read_to_string("/proc/uptime").ok()?;
    let secs: f64 = s.split_whitespace().next()?.parse().ok()?;
    Some((secs * 1e6) as u64)
}

#[cfg(windows)]
fn system_uptime_us() -> Option<u64> {
    // kernel32 is linked by default; declare GetTickCount64 directly to avoid
    // pulling in an extra crate. It returns milliseconds since boot and never
    // fails.
    #[allow(non_snake_case)]
    unsafe extern "system" {
        fn GetTickCount64() -> u64;
    }
    // SAFETY: GetTickCount64 takes no arguments and cannot fail.
    Some(unsafe { GetTickCount64() }.saturating_mul(1000))
}

#[cfg(not(any(target_os = "linux", windows)))]
fn system_uptime_us() -> Option<u64> {
    None
}

fn epoch_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::Connectable;
    use std::net::UdpSocket;
    use std::time::Duration;

    fn sample_pose() -> PoseMsg {
        PoseMsg {
            time_us: 123_456_789,
            pos_ned: [1.5, -2.5, 3.5],
            vel_body_frd: [0.1, 0.2, -0.3],
            rpy: [0.1, 0.2, 0.3],
            quat_wxyz: [0.9, 0.1, 0.2, 0.3],
            pos_var: [1e-4, 2e-4, 3e-4],
            vel_var: [1e-3, 2e-3, 3e-3],
            ang_var: 1e-3,
            reset_counter: 7,
            quality: 100,
        }
    }

    fn loopback(kind: MsgKind) -> (MavSender, Connection<MavMessage>) {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("bind");
        let addr = sock.local_addr().expect("addr");
        let rx = mavlink::UdpConfig::from_socket(sock, mavlink::UdpMode::Udpin)
            .expect("config")
            .read_timeout(Duration::from_secs(2))
            .connect::<MavMessage>()
            .expect("rx connect");
        let tx = MavSender::connect(&format!("udpout:{addr}"), kind, 255, 190).expect("tx connect");
        (tx, rx)
    }

    #[test]
    fn odometry_roundtrip() {
        let (mut tx, rx) = loopback(MsgKind::Odometry);
        tx.send_pose(&sample_pose()).expect("send");
        let (_header, msg) = rx.recv().expect("recv");
        let MavMessage::ODOMETRY(d) = msg else {
            panic!("expected ODOMETRY");
        };
        let p = sample_pose();
        assert_eq!(d.time_usec, p.time_us);
        assert!((d.x - p.pos_ned[0] as f32).abs() < 1e-6);
        assert!((d.y - p.pos_ned[1] as f32).abs() < 1e-6);
        assert!((d.z - p.pos_ned[2] as f32).abs() < 1e-6);
        assert!((d.vx - p.vel_body_frd[0] as f32).abs() < 1e-6);
        assert_eq!(d.frame_id, MavFrame::MAV_FRAME_LOCAL_FRD);
        assert_eq!(d.child_frame_id, MavFrame::MAV_FRAME_BODY_FRD);
        assert_eq!(d.reset_counter, 7);
        assert_eq!(d.quality, 100);
        assert_eq!(d.estimator_type, MavEstimatorType::MAV_ESTIMATOR_TYPE_MOCAP);
        assert!((d.pose_covariance[0] - p.pos_var[0] as f32).abs() < 1e-10);
    }

    #[test]
    fn vision_roundtrip() {
        let (mut tx, rx) = loopback(MsgKind::Vision);
        tx.send_pose(&sample_pose()).expect("send");
        let (_header, msg) = rx.recv().expect("recv");
        let MavMessage::VISION_POSITION_ESTIMATE(d) = msg else {
            panic!("expected VISION_POSITION_ESTIMATE");
        };
        let p = sample_pose();
        assert_eq!(d.usec, p.time_us);
        assert!((d.x - p.pos_ned[0] as f32).abs() < 1e-6);
        assert!((d.yaw - p.rpy[2] as f32).abs() < 1e-6);
        assert_eq!(d.reset_counter, 7);
        assert!((d.covariance[11] - p.pos_var[2] as f32).abs() < 1e-10);
    }

    #[test]
    fn sequences_advance() {
        let (mut tx, _rx) = loopback(MsgKind::Vision);
        let p = sample_pose();
        tx.send_pose(&p).expect("send1");
        tx.send_pose(&p).expect("send2");
        assert_eq!(tx.sequence, 2);
    }
}
