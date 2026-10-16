# vicon-drone-tracker

Streams a drone's pose from a [Vicon](https://www.vicon.com/) motion capture
system to an ArduPilot or PX4 flight controller over MAVLink/UDP, so the vehicle
can fly indoors without GPS.

```
Vicon cameras ──► pose source ──► EKF (position + velocity) ──► NED converter ──► MAVLink/UDP ──► flight controller
```

The Vicon pose is noisy and occasionally drops out (marker occlusion), and it is
expressed in the lab's coordinate frame (typically ENU, millimetres). The tool:

1. reads the tracked rigid body (real Vicon SDK, or a built-in mock),
2. filters it with a small constant-velocity Kalman filter (position + velocity),
   which coasts through brief dropouts, rejects outliers with a sigma gate, and
   re-initialises after a long outage,
3. converts the pose to MAVLink conventions (NED metres, body FRD) and forwards
   orientation,
4. sends the pose at a fixed rate, plus a GPS origin/home for GPS-less operation.

## Requirements

- Rust (stable, edition 2024). `rustup` recommended.
- For `--features vicon`: the native **Vicon DataStream SDK** (C library).
  Set `VICON_SDK_DIR` to the directory containing `ViconDataStreamSDK_C` and its
  companion libraries. The library architecture must match the Rust target.
- Without the `vicon` feature the **mock** source is used, so the tool builds and
  runs on any machine with no Vicon SDK.

## Build

```sh
# Portable build (mock source only), no native SDK needed:
cargo build --release

# Full build with the real Vicon SDK:
VICON_SDK_DIR=/path/to/sdk cargo build --release --features vicon
```

Run the tests (no SDK required):

```sh
cargo test
```

## Usage

```sh
# Bench test with the mock trajectory (no hardware):
cargo run --release -- --source mock

# Stream to a flight controller:
cargo run --release -- \
  --source vicon --subject drone \
  --vicon-host 127.0.0.1:801 \
  --input-frame enu --body-frame frd --attitude vicon \
  --autopilot ardupilot \
  --mavlink udpout:127.0.0.1:14550 \
  --origin "47.39,8.51,489"
```

Run `--help` for the full option list. Key options:

| Option | Meaning |
|--------|---------|
| `--source vicon\|mock` | Pose source. Default depends on the build (see below). |
| `--vicon-host HOST:PORT` | Vicon DataStream server (default `127.0.0.1:801`). |
| `--subject NAME` | Tracked rigid body name (required with `--source vicon`). |
| `--segment NAME` | Vicon segment to read (default: subject root segment). |
| `--input-frame enu\|ned` | How to interpret the Vicon world axes. |
| `--body-frame frd\|flu` | Rigid-body axis convention as defined in Vicon Tracker. |
| `--attitude vicon\|none` | Forward Vicon orientation, or send level attitude. |
| `--yaw-offset-deg DEG` | Yaw alignment between the rigid body and the vehicle. |
| `--autopilot ardupilot\|px4` | Selects the default MAVLink message. |
| `--message auto\|odometry\|vision` | Override the MAVLink message. |
| `--mavlink ADDR` | Output address, e.g. `udpout:127.0.0.1:14550`. |
| `--send-hz HZ` | Pose output rate (default 50). |
| `--pos-noise-mm MM` | Vicon measurement noise per axis (default 3). |
| `--accel-noise A` | EKF process noise, white-noise accel std-dev (default 1). |
| `--gate-sigma N` | Reject a measurement beyond N sigmas (default 6, 0 disables). |
| `--max-coast-secs S` | Pause output after S seconds without a valid measurement. |
| `--ang-noise-deg DEG` | Orientation uncertainty reported in covariance fields. |
| `--origin "lat,lon,alt_m"` | Send GPS origin/home (needed without GPS). |

Because the default source is chosen at build time, a binary built with
`--features vicon` defaults to `--source vicon`, and one without it defaults to
`--source mock`. Pass `--source` explicitly to be sure.

## MAVLink output

The message depends on the target autopilot:

- **ArduPilot** (`--autopilot ardupilot`) → `ODOMETRY`, with
  `frame_id = LOCAL_FRD`, `child_frame_id = BODY_FRD`, NED position and body-FRD
  velocity. This is what ArduPilot's EKF accepts for external navigation.
- **PX4** (`--autopilot px4`) → `VISION_POSITION_ESTIMATE`, interpreted as a NED
  pose.

`--origin` sends `SET_GPS_GLOBAL_ORIGIN` and `SET_HOME_POSITION` (re-sent
periodically), which is required for a GPS-less EKF to have a global reference.

## Coordinate frames

| Setting | Convention |
|---------|------------|
| `--input-frame enu` | Vicon world is East-North-Up (typical Z-up lab). Converted to NED. |
| `--input-frame ned` | Vicon world is already North-East-Down. |
| `--body-frame frd` | Rigid body is Forward-Right-Down (matches MAVLink body axes). |
| `--body-frame flu` | Rigid body is Forward-Left-Up (ROS convention). |

Use `--yaw-offset-deg` to align the rigid body's forward axis with the vehicle's
nose; point the drone's nose along Vicon +X (or +Y for ENU north) and adjust
until the reported yaw matches.

## Windows deployment (Vicon Tracker PC)

Vicon Tracker and the DataStream SDK run on Windows, so the streaming tool is
built and run there. The build uses the MSVC toolchain.

### Prerequisites

```powershell
winget install Rustlang.Rustup                     # default host target x86_64-pc-windows-msvc
winget install Microsoft.VisualStudio.2022.BuildTools   # tick "Desktop development with C++"
winget install Git.Git
```

Install the Vicon DataStream SDK, e.g. to
`C:\Program Files\Vicon\DataStream SDK\Win64\` (contains `ViconDataStreamSDK_C.dll`,
its import library, and companion DLLs).

### Clone and build

```powershell
git clone https://github.com/bizarri/Vicon_Tracker.git C:\src\vicon-drone-tracker
cd C:\src\vicon-drone-tracker

# Sanity check the toolchain first (no SDK needed):
cargo run --release -- --source mock

# Full build against the SDK:
$env:VICON_SDK_DIR = "C:\Program Files\Vicon\DataStream SDK\Win64"
$env:PATH = "$env:VICON_SDK_DIR;$env:PATH"      # so the DLLs are found at runtime
cargo build --release --features vicon
```

`VICON_SDK_DIR` is needed at build time (so the linker finds the import library);
`PATH` (or copying the DLLs next to the `.exe`) is needed at run time.

### Run

```powershell
.\target\release\vicon-drone-tracker.exe `
  --source vicon --subject drone `
  --mavlink udpout:<FC_IP>:14550 `
  --origin "<lat>,<lon>,<alt_m>"
```

Allow the app through Windows Firewall if prompted (outbound UDP). The tool prints
a status line every two seconds with position, velocity, measurement age, and the
number of messages sent.

## Development layout

| File | Responsibility |
|------|----------------|
| `src/main.rs` | Main loop and the `Tracker` that ties the pipeline together. |
| `src/cli.rs` | Command-line options. |
| `src/ekf.rs` | Constant-velocity Kalman filter (position + velocity). |
| `src/frames.rs` | Coordinate-frame conversion (ENU/NED, FRD/FLU, yaw offset). |
| `src/source.rs` | Pose-source interface and mock source. |
| `src/vicon_source.rs` | Real Vicon SDK reader (feature-gated). |
| `src/mav_out.rs` | Builds and sends MAVLink messages. |
