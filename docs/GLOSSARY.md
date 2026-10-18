# Glossary — terms and acronyms

Reference for the vocabulary used across this project, the Vicon SDK, and
MAVLink. Grouped by topic; alphabetical within a group.

## Coordinate frames and attitude

- **World / input frame** — the coordinate frame Vicon reports poses in, as
  configured by `--input-frame`.
- **Body frame** — the frame attached to the drone/rigid body, as defined in
  Vicon Tracker; selected by `--body-frame`.
- **ENU** — East-North-Up. A right-handed frame with Z up (typical Z-up lab).
  `--input-frame enu`.
- **NED** — North-East-Down. The MAVLink/aviation world frame (Z down).
  `--input-frame ned`.
- **FRD** — Forward-Right-Down. The MAVLink body frame (nose forward, right
  wing, down). `--body-frame frd`.
- **FLU** — Forward-Left-Up. The ROS convention for body axes.
  `--body-frame flu`.
- **RPY** — Roll, Pitch, Yaw, in radians, in NED/body convention.
- **Roll** — rotation about the forward (X) axis.
- **Pitch** — rotation about the right (Y) axis.
- **Yaw** — rotation about the down (Z) axis; heading.
- **Attitude** — the orientation (roll/pitch/yaw or quaternion) of the body.
- **Quaternion** — 4-number orientation representation. Vicon order is
  `(x, y, z, w)`; MAVLink uses `(w, x, y, z)`.
- **Yaw offset** — a constant yaw added to align the rigid body's forward axis
  with the vehicle nose (`--yaw-offset-deg`).
- **Level attitude** — roll = pitch = 0; sent with `--attitude none` when Vicon
  orientation is not trusted.

## Vicon / motion capture

- **Vicon** — a motion-capture (mocap) camera system that tracks reflective
  markers and reports rigid-body poses.
- **Mocap** — motion capture.
- **DataStream** — the Vicon streaming protocol/server (from Tracker / Nexus /
  Shogun) that this tool connects to.
- **DataStream SDK** — the native C client library for DataStream; required to
  build with `--features vicon`.
- **Subject** — a tracked object (rigid body) in Vicon Tracker; named and
  selected with `--subject`.
- **Rigid body** — a set of markers tracked as one object with a pose.
- **Segment** — a part of a subject's skeleton; the **root segment** is the
  object's origin. `--segment` overrides the default.
- **Occlusion** — a marker/body temporarily not visible to enough cameras, so no
  valid pose is reported for that frame.

## Filtering / estimation

- **EKF** — Extended Kalman Filter. Here a linear constant-velocity Kalman
  filter over position and velocity (`src/ekf.rs`).
- **State** — the estimated quantities: `[x, y, z, vx, vy, vz]` (NED m, m/s).
- **Prediction (predict)** — advance the state with the motion model and grow
  uncertainty; no measurement used.
- **Correction (update)** — pull the prediction toward a measurement, weighted
  by uncertainties.
- **Motion model** — the assumed dynamics. Current: **constant velocity (CV)**
  (`p += v·dt`).
- **Covariance (P)** — the state uncertainty matrix; its diagonal entries are
  the variances (`pos_var`, `vel_var`).
- **Process noise (Q)** — uncertainty added per predict step from unmodelled
  accelerations; tuned by `--accel-noise`.
- **Measurement noise (R)** — assumed Vicon position noise; tuned by
  `--pos-noise-mm`.
- **Kalman gain (K)** — the weighting that balances prediction versus
  measurement.
- **Innovation (y)** — measurement minus predicted measurement (`z - Hx`).
- **Innovation covariance (S)** — covariance of the innovation; basis for
  gating.
- **Sigma** — one standard deviation; `sqrt(variance)`.
- **Gate / gating** — rejecting a measurement whose innovation exceeds
  `--gate-sigma` sigmas on any axis.
- **Coasting** — running on prediction alone while measurements are missing.
- **Stale** — a measurement with a timestamp older than the filter state;
  ignored.
- **Re-initialise (reinit)** — reset the filter from a fresh measurement after a
  long outage.
- **Reset counter** — a MAVLink field, incremented on re-init so the autopilot
  knows the estimate restarted.
- **Dead reckoning** — propagating position from velocity/acceleration without
  external measurements.
- **Joseph form** — a numerically stable covariance-update formulation used in
  `correct`.
- **DT** — time step `dt`; clamped to `[MIN_DT, MAX_DT]` in the filter.
- **RMSE** — Root-Mean-Square Error; used in the EKF tests.
- **Bias** — a systematic sensor offset (e.g. IMU bias), estimated in more
  advanced filters.

## Filter/model variants (see `docs/ARCHITECTURE.md`)

- **CV** — Constant Velocity model (current).
- **CA** — Constant Acceleration model (adds acceleration states).
- **Singer** — CV variant where velocity decays over a time constant (drag).
- **CTRA** — Constant Turn Rate and Acceleration model (arcing flight).
- **IMM** — Interacting Multiple Model; blends several models by likelihood.
- **IMU** — Inertial Measurement Unit (accelerometer + gyro); fusing it as a
  filter input improves occlusion handling.

## MAVLink / flight controller

- **MAVLink** — the lightweight messaging protocol used to talk to the flight
  controller.
- **UDP / TCP** — network transports. Pose is sent over UDP (`udpout:` /
  `udpin:`).
- **udpout** — send to a remote address (`udpout:HOST:PORT`); the normal case.
- **udpin** — listen on a local port (`udpin:0.0.0.0:PORT`).
- **ODOMETRY** — MAVLink message with position + velocity + attitude +
  covariance; preferred by ArduPilot.
- **VISION_POSITION_ESTIMATE** — MAVLink message with position + attitude +
  covariance; used for PX4 (also accepted by ArduPilot).
- **SET_GPS_GLOBAL_ORIGIN** — sets the global origin for a GPS-less local frame.
- **SET_HOME_POSITION** — sets the home position.
- **sysid / compid** — MAVLink system and component IDs of the sender
  (`--mav-sysid`, `--mav-compid`).
- **MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY** — component id `190`, the default
  `compid`.
- **Estimator type** — `MAV_ESTIMATOR_TYPE_MOCAP` tags the odometry as mocap.
- **quality** — ODOMETRY field: `100` = best, `-1` = failed.
- **ArduPilot** — open-source autopilot stack; consumes `ODOMETRY`.
- **PX4** — open-source autopilot stack; consumes `VISION_POSITION_ESTIMATE`.
- **EKF3** — ArduPilot's onboard estimator; configured to source external nav
  (e.g. `EK3_SRC*`, `VISO*`).
- **EKF2** — PX4's onboard estimator; configured for external vision
  (`EKF2_EV_*`).
- **Origin / home** — a physical reference point (`--origin "lat,lon,alt_m"`)
  giving the GPS-less local frame a global anchor.

## Software / build

- **Rust** — the implementation language; edition 2024.
- **Cargo** — Rust's build tool and package manager.
- **Crate** — a Rust package/library. Dependencies: `nalgebra`, `clap`,
  `anyhow`, `mavlink`, `vicon-sdk`.
- **nalgebra** — linear-algebra library (matrices/vectors, `SMatrix`/`SVector`).
- **clap** — command-line argument parser (the CLI in `src/cli.rs`).
- **anyhow** — error-handling helper used throughout.
- **Feature flag** — conditional compilation. `vicon` enables the real SDK path.
- **Thread** — a unit of concurrency; the tool spawns one thread to read the
  pose source.
- **mpsc channel** — multi-producer, single-consumer queue (`std::sync::mpsc`);
  passes `RawSample` from the source thread to the main loop.
- **stdout / stderr** — output streams. All logs go to **stderr**; stdout is
  left free.
- **Trace** — `--trace`; prints every pipeline stage to stderr.
- **MSVC** — Microsoft Visual C++ toolchain (Windows builds).
- **x86_64-pc-windows-msvc** — the default 64-bit Windows Rust target.
- **DLL** — Windows dynamic library (e.g. `ViconDataStreamSDK_C.dll`).
- **Import library (.lib)** — the link-time stub for a Windows DLL.
- **VICON_SDK_DIR** — environment variable pointing at the Vicon SDK directory,
  used at build time to find the import library.
- **Hz** — Hertz; cycles per second (e.g. `--send-hz`, `--mock-hz`).
- **mm** — millimetres (Vicon native unit; converted to metres).

## Runtime concepts

- **Tracker** — the struct in `src/main.rs` that ties the pipeline together.
- **Pose** — position + orientation.
- **Sample** (`RawSample`) — one source reading, in input-frame metres.
- **Pose message** (`PoseMsg`) — the outgoing pose in MAVLink conventions.
- **Tracking state** — Idle / Searching / Tracking / Lost (`src/state.rs`).
- **Source** — where poses come from: `vicon` or `mock`.
- **Mock source** — synthetic trajectory for testing without hardware.
- **Coast window** — `--max-coast-secs`; how long output continues on
  prediction before pausing.
