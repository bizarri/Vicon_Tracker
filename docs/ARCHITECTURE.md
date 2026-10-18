# Architecture — how the program is structured and what it prints

This document explains how a single binary turns a Vicon pose into MAVLink
messages, and how to read the output at each stage. It is the "how it works"
companion to `docs/MODES.md` (what each flag does) and `docs/USER-GUIDE.md`
(physical setup). For terminology, see `docs/GLOSSARY.md`.

## One program, several modules

It is **one program** (`vicon-drone-tracker`), compiled from several modules,
running in **one process**. There is a single extra thread for reading the pose
source; everything else happens on the main thread. The modules are the "stages"
of the pipeline:

| Stage | Module | Responsibility |
|-------|--------|----------------|
| Parse options | `src/cli.rs` | Flags and defaults (`Cli`). |
| Read from Vicon | `src/vicon_source.rs` (feature-gated) | Connect to the DataStream server, `get_frame`, produce one `RawSample` per frame (mm→m). |
| Read from mock | `src/source.rs` | `PoseSource` trait + `MockSource` synthetic trajectory. |
| Convert frames | `src/frames.rs` | ENU→NED, FLU→FRD, attitude handling, yaw offset. |
| Filter | `src/ekf.rs` | Constant-velocity Kalman filter (predict / correct / coast / gate). |
| Track state | `src/state.rs` | Idle / Searching / Tracking / Lost state machine. |
| Send to FC | `src/mav_out.rs` | Build `ODOMETRY` / `VISION_POSITION_ESTIMATE`, send over UDP. |
| Orchestrate | `src/main.rs` | `main()` + the `Tracker` that drives all of the above. |

## Runtime data flow

```
                        (one extra thread)                 (main thread — one loop)
  Vicon ──► vicon_source ──┐
                           ├── RawSample ──► mpsc channel ──► main loop ──► Tracker
  Mock  ──► source ────────┘                                                 │
                                                                             ├─ frames.rs  pos_to_ned / attitude
                                                                             ├─ ekf.rs     process / predict
                                                                             ├─ state.rs   transitions
                                                                             └─ mav_out.rs build + send pose
```

Step by step:

1. `main()` parses the flags and builds the frame `Converter` and the
   `MavSender`.
2. It picks a source and spawns **one helper thread** that loops
   `source.next_sample()` and pushes each `RawSample` into an `mpsc` channel
   (`src/main.rs`). This thread blocks on the Vicon SDK, which is why it is
   separate.
3. The **main loop** receives samples and calls `Tracker::handle`, which runs
   the frame conversion, feeds the EKF, and drives the state machine.
4. On a fixed grid (`--send-hz`) it calls `Tracker::build_pose` (EKF predicted
   forward to "now", velocity rotated into the body frame) and
   `MavSender::send_pose`.
5. Every 10 s it also re-sends the GPS origin/home when `--origin` is set.

Data passed between stages is plain values: `RawSample` (source → tracker) and
`PoseMsg` (tracker → MAVLink). No stage is a separate program.

## The EKF — what it does (and does not do)

The filter lives in `src/ekf.rs`. It is a small **Extended Kalman Filter** (here
linear, so effectively a plain Kalman filter) over the state

```
x = [ px, py, pz,  vx, vy, vz ]      # NED position (m) and velocity (m/s)
```

with **position-only measurements** from Vicon (the source provides position and
orientation, not velocity).

### It does not "understand" flight

It has **no concept of a flight path**. It does not recognise circles, waypoints,
or maneuvers, and it is not trained on data. What it does is assume a simple
**motion model** and combine it with the noisy Vicon measurements statistically:

- **Constant-velocity (CV) model** — the current model. Assumes velocity stays
  roughly constant between steps: `p += v·dt`, `v` unchanged.
- Vicon is treated as a **noisy position sensor**; the filter estimates the
  velocity the sensor cannot give.

### The two steps

1. **Predict** (`Ekf::predict` / `predict_to`) — push the state forward in time
   with the motion model and *grow* the uncertainty (covariance) because no new
   measurement was seen. `predict_to(now)` is what lets output run at
   `--send-hz` even when Vicon is slower, and what produces a pose between
   measurements.
2. **Correct** (`Ekf::correct`) — pull the prediction toward the new
   measurement, weighted by their relative uncertainties (the Kalman gain).
   This is what smooths Vicon noise *and* estimates velocity.

### Built on top of the model

- **Coast** — on a missing/occluded measurement it only predicts
  (`Step::Coasted`); because velocity is known, the predicted pose keeps moving
  instead of freezing.
- **Gate** — a measurement disagreeing with the prediction by more than
  `--gate-sigma` innovation sigmas is rejected as an outlier (`Step::Rejected`),
  and the state is left unchanged.
- **Stale** — a measurement older than the current filter time is ignored
  (`Step::Stale`).
- **Re-init** — after a long outage (`--max-coast-secs`) it is re-initialised
  from the next measurement, and `reset_counter` is bumped.

Because the model is constant-velocity, a long occlusion predicts a straight
continuation of the last velocity; a sharp change of direction is only learned
once measurements resume.

### Motion-model alternatives (not implemented)

Roughly in increasing complexity. Only 1–2 are worth it for this project.

| Model | Idea | Cost | Notes |
|-------|------|------|-------|
| **Constant Velocity (CV)** *(current)* | `v` constant, `p += v·dt` | — | Simplest useful model; good because Vicon is high-rate. |
| **Constant Acceleration (CA)** | add 3 accel states → `[p, v, a]`, `p += v·dt + ½a·dt²` | small | Better during drops for accelerating/braking motion; noisier `a`. |
| **Singer / maneuvering** | CV with velocity pulled toward zero over a time constant `τ` (drag/control) | small | Usually the best cheap upgrade for drones; a few lines in the `F` matrix. |
| **Coordinated Turn (CTRA)** | add turn rate (and climb rate) → models arcing flight | medium | Good for fixed-wing/circling; overkill for a multirotor. |
| **IMM** (Interacting Multiple Model) | run CV + CA + turn in parallel and blend by likelihood | large | Handles mode switching (hover vs accelerate) without committing; more tuning. |
| **IMU fusion** | use the FC's accelerometer/gyro as EKF control inputs | large | The real fix for occlusions: dead-reckon from true accelerations. Needs bidirectional MAVLink, time sync, bias estimation. |
| **Learned prediction** | neural / GP motion model | large | Not appropriate here: needs training data, adds latency and nondeterminism. |

Practical guidance:

- The tracker mostly **smooths and rejects outliers** — the flight controller
  runs its *own* EKF that fuses this pose with its IMU, so this filter is not
  the sole estimator.
- Cheap improvement: adopt **Singer** or **CA** (small change to the `F` matrix
  and `EkfConfig`). See `src/ekf.rs`.
- If occlusions are the real problem, **IMU fusion** helps far more than any
  motion-model change.
- **IMM** is justified only if you observe systematic lag during aggressive
  maneuvering.

## Reading the output — one step at a time

The lines below are real output from an instrumented run of the mock source:

```sh
vicon-drone-tracker --source mock --trace --origin "47.39,8.51,489"
```

```
mavlink: sending "ODOMETRY" to udpout:127.0.0.1:14550 as sysid 255 compid 190
[TRACE][RCV] t=0.0000s occl=false pos_m=[2.000,0.003,0.994] has_quat=true
tracker: state changed Idle -> Searching (first sample)
[TRACE][FRM] enu->ned pos_ned=[0.003,2.000,-0.994]
[TRACE][EKF] Initialized
tracker: state changed Searching -> Tracking (EKF initialized)
[TRACE][MAV] send_origin lat=47.390000 lon=8.510000 alt=489.000
[TRACE][RCV] t=0.0100s occl=false pos_m=[1.999,0.018,1.011] has_quat=true
[TRACE][FRM] enu->ned pos_ned=[0.018,1.999,-1.011]
[TRACE][EKF] Updated
[TRACE][RCV] t=0.0200s occl=false pos_m=[1.994,0.026,1.012] has_quat=true
[TRACE][FRM] enu->ned pos_ned=[0.026,1.994,-1.012]
[TRACE][EKF] Updated
[TRACE][LOOP] now=0.0201s next_due=0.0400s has_valid=true initialized=true can_send=true
[TRACE][POSE] now=0.0201s pred pos=[0.027,1.995,-1.014] vel_ned=[1.096,-0.263,-0.864] vel_body_frd=[1.100,0.245,0.864] rpy=[3.142,-0.000,-0.016] reset=0
[TRACE][MAV] pose sent (sent=1)
```

### 1. Startup (once)
```
mavlink: sending "ODOMETRY" to udpout:127.0.0.1:14550 as sysid 255 compid 190
```
The MAVLink socket is open. It reports the chosen message (`ODOMETRY` for
ArduPilot, `VISION_POSITION_ESTIMATE` for PX4) and the sender identity.

### 2. Raw sample from the source — `[TRACE][RCV]`
```
[TRACE][RCV] t=0.0000s occl=false pos_m=[2.000,0.003,0.994] has_quat=true
```
- `t` — seconds since process start (monotonic)
- `occl` — the body could not be resolved this frame
- `pos_m` — position in **input-world** metres (mm→m done in the source)
- `has_quat` — whether an orientation was provided

### 3. First sample seen — `state changed` (always printed)
```
tracker: state changed Idle -> Searching (first sample)
```
The state machine leaves `Idle` as soon as any sample arrives.

### 4. Frame conversion — `[TRACE][FRM]`
```
[TRACE][FRM] enu->ned pos_ned=[0.003,2.000,-0.994]
```
Input world (ENU) → NED metres. Passthrough with `--input-frame ned`.

### 5. EKF decision — `[TRACE][EKF]`
One of:
```
[TRACE][EKF] Initialized                          # first fix: pos = measurement, vel = 0
[TRACE][EKF] Updated                              # measurement accepted
[TRACE][EKF] Coasted                              # predicted only (no valid measurement)
[TRACE][EKF] Rejected by gate (sigma>6) -> skip   # innovation too large, dropped
[TRACE][EKF] Stale (dt<0) -> skip                 # out-of-order timestamp, ignored
[TRACE][EKF] occluded -> skip                     # occluded sample, coast instead
[TRACE][EKF] Reinit after outage (reset_counter=N)
```

### 6. Tracking confirmed — `state changed`
```
tracker: state changed Searching -> Tracking (EKF initialized)
```

### 7. GPS origin/home — `[TRACE][MAV]` (every 10 s while `--origin` is set)
```
[TRACE][MAV] send_origin lat=47.390000 lon=8.510000 alt=489.000
```
Sends `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION`.

### 8. Send-tick decision — `[TRACE][LOOP]` (once per `--send-hz`)
```
[TRACE][LOOP] now=0.0201s next_due=0.0400s has_valid=true initialized=true can_send=true
```
- `has_valid` — a measurement exists
- `can_send` — output is allowed (EKF initialized and within `--max-coast-secs`)

### 9. Pose built and sent — `[TRACE][POSE]` then `[TRACE][MAV]`
```
[TRACE][POSE] now=0.0201s pred pos=[...] vel_ned=[...] vel_body_frd=[...] rpy=[...] reset=0
[TRACE][MAV] pose sent (sent=1)
```
`pred` is the EKF propagated to `now`; `vel_body_frd` is NED velocity rotated
into the body frame; `rpy` is the attitude sent. `pose sent` is printed only
when the send succeeds.

### 10. Pose contents (what actually goes on the wire)
The message fields come from `PoseMsg` / `src/mav_out.rs`:

| Field | Meaning |
|-------|---------|
| `time_us` | Host time, microseconds (monotonic within a run). |
| `pos_ned` | NED position in metres. |
| `vel_body_frd` | Velocity rotated into body FRD. |
| `rpy` / `quat_wxyz` | Attitude (Vicon-forwarded, or level with `--attitude none`). |
| `pos_var` / `vel_var` | Position/velocity variance from the EKF covariance. |
| `ang_var` | Orientation uncertainty from `--ang-noise-deg`. |
| `reset_counter` | Bumped on re-init so the FC knows the estimate restarted. |
| `quality` | `100` (best) in `ODOMETRY`. |

### 11. Coast / outage handling
When no valid measurement arrives:
```
tracker: no valid measurement for 0.31s, pausing MAVLink output
[TRACE][COAST] exceeded max_coast (0.31s > 0.30s) -> pausing
tracker: state changed Tracking -> Lost (coast timeout)
...
tracker: measurement restored after outage, EKF re-initialized (reset_counter=1)
tracker: state changed Lost -> Tracking (recovery)
```
Short gaps keep streaming the predicted pose; past `--max-coast-secs` output
pauses, state goes `Lost`, and on recovery the EKF re-initializes and
`reset_counter` is bumped.

### 12. Periodic status — every 2 s (always printed, not `--trace`)
```
tracker: pos=(2.00,-0.05,-0.90) m vel=(-0.02,-1.67,0.61) m/s meas_age=1ms sent=100
```

## Verbosity

- `tracker:` and `state changed` lines are **always** printed.
- `[TRACE]` lines require `--trace` (synchronous stderr; avoid at high
  `--send-hz` during live flight).
- All output goes to **stderr**; stdout is left free for the MAVLink/UDP path.
