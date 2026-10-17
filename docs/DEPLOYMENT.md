# Deployment Guide — transferring to the Vicon computer

This document is the checklist for moving the tool from a development machine
(no Vicon hardware) to the **Vicon Tracker PC** and getting it to stream a real
drone to the flight controller.

Read this together with:

- `README.md` — build options and coordinate frames.
- `docs/USER-GUIDE.md` — the physical setup (calibration, tracked object, flying).
- `docs/MODES.md` — what every runtime mode/flag does.

---

## 0. Topology — where does the tool run?

```
Vicon cameras ──► Vicon Tracker PC ──(DataStream, TCP)──► this tool ──(MAVLink/UDP)──► flight controller
```

- The tool needs the **Vicon DataStream SDK** and must reach the DataStream
  server. The simplest and recommended place to run it is the **Vicon Tracker
  PC itself** (`--vicon-host 127.0.0.1:801`).
- It does **not** have to be the Vicon PC: any machine on the same LAN that can
  reach the DataStream server works, but then the SDK must be installed there
  too. If in doubt, run it on the Vicon PC.
- The flight controller is reached over the LAN by UDP (`--mavlink
  udpout:<FC_IP>:14550`). Nothing needs to be installed on the flight
  controller.

### What is already done in the repo

- All code, tests, and docs are complete and committed.
- Without the SDK the tool builds and runs the **mock** source, so the transfer
  can be sanity-checked before the SDK is wired up.
- `cargo check --features vicon` must pass (it does on a plain machine). Only
  the final **link** step needs the native library, which is what the Vicon PC
  provides.

### What is left to do on the Vicon PC

1. Install the Rust toolchain, the MSVC C++ build tools, and Git.
2. Install the Vicon DataStream SDK.
3. Set `VICON_SDK_DIR` (link time) and put the SDK DLLs on `PATH` (run time).
4. `cargo build --release --features vicon`.
5. Open the firewall for outbound UDP and confirm the FC IP/port.
6. Run with the correct `--subject`, frames, and `--origin`.

The rest of this document expands each step.

---

## 1. Prerequisites

Install on the Vicon PC (Windows):

```powershell
winget install Rustlang.Rustup                        # default host target x86_64-pc-windows-msvc
winget install Microsoft.VisualStudio.2022.BuildTools # tick "Desktop development with C++"
winget install Git.Git
```

Then install the **Vicon DataStream SDK**. Note the install directory, e.g.
`C:\Program Files\Vicon\DataStream SDK\Win64\`. It must contain:

- `ViconDataStreamSDK_C.dll` (runtime)
- the import library (`.lib`) that matches the toolchain architecture
- the companion DLLs the SDK ships with

> The SDK's architecture must match the Rust target. On a standard x86-64
> Windows machine that is `x86_64-pc-windows-msvc`, so use the `Win64` SDK.

---

## 2. Get the code onto the Vicon PC

```powershell
git clone https://github.com/bizarri/Vicon_Tracker.git C:\src\vicon-drone-tracker
cd C:\src\vicon-drone-tracker
```

(Alternatively copy the repository folder.) The `target/` directory is ignored
and does not need to be transferred; it will be rebuilt.

---

## 3. Sanity check without the SDK (do this first)

Confirm the toolchain works before adding the SDK, using the mock source:

```powershell
cargo run --release -- --source mock
```

You should see a status line every 2 s with position/velocity and a
`tracker: streaming ... to flight controller` line. Press Ctrl+C to stop.

You can also confirm the whole pipeline on the mock, including dropouts:

```powershell
cargo run --release -- --source mock --mock-dropout --max-coast-secs 0.5 --trace
```

This prints every pipeline stage and the tracking-state changes. If this works,
the code is fine and only the SDK link remains.

---

## 4. Build against the Vicon SDK

```powershell
$env:VICON_SDK_DIR = "C:\Program Files\Vicon\DataStream SDK\Win64"
$env:PATH = "$env:VICON_SDK_DIR;$env:PATH"      # so the DLLs are found at runtime
cargo build --release --features vicon
```

- `VICON_SDK_DIR` is consumed at **build time** so the linker finds the import
  library (`ViconDataStreamSDK_C`).
- `PATH` (or copying the DLLs next to the `.exe`) is needed at **run time** so
  Windows can load `ViconDataStreamSDK_C.dll`.

Common failure and meaning:

| Error | Cause | Fix |
|-------|-------|-----|
| `unable to find library -lViconDataStreamSDK_C` | `VICON_SDK_DIR` unset/wrong, or wrong architecture | Set it to the SDK dir; use the Win64 SDK |
| `The code execution cannot proceed because ViconDataStreamSDK_C.dll was not found` | DLL not on `PATH` at runtime | Add `VICON_SDK_DIR` to `PATH`, or copy the DLLs next to the `.exe` |
| `LNK1112: module machine type 'x64' conflicts...` | 32/64-bit mismatch | Use the Win64 SDK and an x64 build |

See `README.md` for the portable (no-SDK) build command.

---

## 5. Network and firewall

- Vicon PC and flight controller must be on the **same LAN**.
- Allow the tool through Windows Firewall (outbound UDP). If prompted, accept.
- Find the flight controller's **IP and MAVLink UDP port** (ArduPilot often
  `14550`).
- Output address format:
  - `udpout:<FC_IP>:<port>` — send to the FC (normal case).
  - `udpin:0.0.0.0:<port>` — listen instead (only if your setup requires it).

Verify the path from the Vicon PC:

```powershell
ping <FC_IP>
```

---

## 6. Prepare the autopilot

The flight controller must be told to trust external vision/odometry:

- **ArduPilot**: configure the EKF source to use external nav (e.g. `EK3_SRC*`
  to external vision/odometry, and the `VISO`/`VISION` parameters as required by
  your firmware). ArduPilot consumes `ODOMETRY`.
- **PX4**: enable the external-vision/odometry path (`EKF2_EV_*` /
  `EKF2_EV_CTRL` and the vision/odometry feed). PX4 consumes
  `VISION_POSITION_ESTIMATE` (or `ODOMETRY`).

`--autopilot ardupilot|px4` selects the default message; you can override with
`--message`. Without GPS you must also supply `--origin "lat,lon,alt_m"` so the
EKF has a global reference.

---

## 7. Run against real Vicon

```powershell
.\target\release\vicon-drone-tracker.exe `
  --source vicon --subject drone `
  --vicon-host 127.0.0.1:801 `
  --input-frame enu --body-frame frd --attitude vicon `
  --autopilot ardupilot `
  --mavlink udpout:<FC_IP>:14550 `
  --origin "<lat>,<lon>,<alt_m>"
```

- `--subject` must match the Vicon Tracker object name **exactly**; on a
  mismatch the tool exits and lists the available subjects.
- See `docs/MODES.md` for every flag and `docs/USER-GUIDE.md` for how to choose
  frames and `--yaw-offset-deg`.

---

## 8. Verify the transfer

Expected startup sequence on stderr:

1. `mavlink: sending "ODOMETRY" to udpout:... as sysid 255 compid 190`
2. `vicon: connected to 127.0.0.1:801, SDK x.y.z.r, subject "drone", segment "..."`
3. `tracker: state changed Idle -> Searching (first sample)`
4. `tracker: state changed Searching -> Tracking (EKF initialized)`
5. `tracker: streaming Odometry to flight controller`
6. A status line every 2 s: position, velocity, measurement age, sent count.

On the flight controller side, confirm the external pose is being accepted and
the local position estimate moves as you move the drone. If you need to see each
stage, add `--trace`.

---

## 9. Troubleshooting quick table

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Exits with "subject ... not found" + list | `--subject` name mismatch | Use the exact Tracker object name |
| `waiting for first measurement` forever | Object not tracked / occluded / wrong segment | Check Tracker shows **Tracked**; check markers |
| `vicon: connection lost, retrying` loops | DataStream server not running or wrong host/port | Start Tracker; verify `--vicon-host` |
| Position correct but axes/yaw wrong | Wrong `--input-frame`/`--body-frame`/`--yaw-offset-deg` | See `docs/USER-GUIDE.md` Phase 7 |
| FC ignores the stream | External-vision not configured | Configure EKF params for external nav |
| No packets leaving the PC | Firewall or wrong `FC_IP:port` | Allow outbound UDP; verify IP/port |
| `...dll was not found` at startup | SDK DLLs not on `PATH` | Add `VICON_SDK_DIR` to `PATH` |

---

## 10. Other platforms

The tool also builds on Linux/macOS: install the Vicon DataStream SDK for that
platform, set `VICON_SDK_DIR` to the directory with `libViconDataStreamSDK_C`,
and build with `--features vicon`. The SDK architecture must match the Rust
target. All runtime flags and modes are identical.
