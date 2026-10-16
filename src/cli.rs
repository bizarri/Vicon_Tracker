use clap::{Parser, ValueEnum};

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum SourceKind {
    /// Read poses from a Vicon DataStream server (requires `--features vicon`)
    Vicon,
    /// Synthetic circle trajectory with noise, for bench-testing without hardware
    Mock,
}

#[cfg(feature = "vicon")]
const DEFAULT_SOURCE: SourceKind = SourceKind::Vicon;
#[cfg(not(feature = "vicon"))]
const DEFAULT_SOURCE: SourceKind = SourceKind::Mock;

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum InputFrame {
    /// Vicon world is ENU: X east, Y north, Z up (typical Z-up right-handed lab setup)
    Enu,
    /// Vicon world is already NED: X north, Y east, Z down
    Ned,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum BodyFrame {
    /// Rigid body defined in Vicon as x forward, y right, z down (matches MAVLink body)
    Frd,
    /// Rigid body defined in Vicon as x forward, y left, z up (ROS/FLU convention)
    Flu,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Attitude {
    /// Forward the Vicon rigid body orientation
    Vicon,
    /// Do not forward orientation (roll = pitch = yaw = 0)
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Autopilot {
    Ardupilot,
    Px4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum MessageKind {
    /// Pick the message preferred for the selected autopilot (ODOMETRY for ArduPilot, VISION_POSITION_ESTIMATE for PX4)
    Auto,
    Odometry,
    Vision,
}

#[derive(Parser, Debug)]
#[command(
    name = "vicon-drone-tracker",
    about = "Vicon mocap -> EKF (position + velocity) -> MAVLink flight controller",
    long_about = "Streams filtered drone pose from a Vicon motion capture system to an\n\
                  ArduPilot or PX4 flight controller over MAVLink/UDP.\n\
                  Vicon output is converted to NED (metres) before sending.\n\
                  Without the `vicon` feature the mock source is available for testing."
)]
pub struct Cli {
    /// Pose source
    #[arg(long, value_enum, default_value_t = DEFAULT_SOURCE)]
    pub source: SourceKind,

    /// Vicon DataStream server address
    #[arg(long, default_value = "127.0.0.1:801")]
    pub vicon_host: String,

    /// Vicon subject (rigid body) name to track; required with --source vicon
    #[arg(long)]
    pub subject: Option<String>,

    /// Vicon segment to read (default: subject's root segment)
    #[arg(long)]
    pub segment: Option<String>,

    /// How to interpret the Vicon world axes
    #[arg(long, value_enum, default_value_t = InputFrame::Enu)]
    pub input_frame: InputFrame,

    /// Rigid body axis convention as defined in Vicon Tracker
    #[arg(long, value_enum, default_value_t = BodyFrame::Frd)]
    pub body_frame: BodyFrame,

    /// Orientation forwarding
    #[arg(long, value_enum, default_value_t = Attitude::Vicon)]
    pub attitude: Attitude,

    /// Degrees added to Vicon yaw to align the tracked object with the vehicle forward axis.
    /// Point the drone's nose along Vicon +X (or +Y for ENU north) and adjust until yaw matches.
    #[arg(long, default_value_t = 0.0)]
    pub yaw_offset_deg: f64,

    /// Target autopilot (selects default MAVLink message)
    #[arg(long, value_enum, default_value_t = Autopilot::Ardupilot)]
    pub autopilot: Autopilot,

    /// MAVLink message used to stream pose
    #[arg(long, value_enum, default_value_t = MessageKind::Auto)]
    pub message: MessageKind,

    /// MAVLink output address, e.g. udpout:127.0.0.1:14550 or udpin:0.0.0.0:14540
    #[arg(long, default_value = "udpout:127.0.0.1:14550")]
    pub mavlink: String,

    /// MAVLink source system id
    #[arg(long, default_value_t = 255)]
    pub mav_sysid: u8,

    /// MAVLink source component id (190 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
    #[arg(long, default_value_t = 190)]
    pub mav_compid: u8,

    /// Pose output rate to the flight controller
    #[arg(long, default_value_t = 50.0)]
    pub send_hz: f64,

    /// Vicon measurement noise per axis (standard deviation)
    #[arg(long, default_value_t = 3.0)]
    pub pos_noise_mm: f64,

    /// EKF process noise: white-noise acceleration standard deviation
    #[arg(long, default_value_t = 1.0)]
    pub accel_noise: f64,

    /// Reject a measurement if any innovation axis exceeds this many sigmas (0 disables gating)
    #[arg(long, default_value_t = 6.0)]
    pub gate_sigma: f64,

    /// Stop sending after this long without a valid measurement; EKF is re-initialized on recovery
    #[arg(long, default_value_t = 1.0)]
    pub max_coast_secs: f64,

    /// Assumed orientation uncertainty per axis for MAVLink covariance fields
    #[arg(long, default_value_t = 2.0)]
    pub ang_noise_deg: f64,

    /// Send SET_GPS_GLOBAL_ORIGIN/SET_HOME_POSITION at this position: "lat,lon,alt_m" (needed without GPS)
    #[arg(long)]
    pub origin: Option<String>,

    /// Mock source rate
    #[arg(long, default_value_t = 100.0)]
    pub mock_hz: f64,

    /// Simulate periodic occlusion dropouts with the mock source
    #[arg(long, default_value_t = false)]
    pub mock_dropout: bool,
}
