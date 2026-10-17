# Modes and Options — what every flag does

A reference for the runtime "modes" of `vicon-drone-tracker`. Options are
grouped by what they control. For the physical setup that must match these
choices, see `docs/USER-GUIDE.md`; for transfer/build steps see
`docs/DEPLOYMENT.md`.

Run `vicon-drone-tracker --help` for the authoritative list.

---

## 1. Build modes

The binary has two build flavours, chosen at compile time:

| Mode | Command | Effect |
|------|---------|--------|
| Portable | `cargo build --release` | No native SDK. `--source mock` only. |
| Real Vicon | `VICON_SDK_DIR=... cargo build --release --features vicon` | Adds the real Vicon source; the default `--source` becomes `vicon`. |

Because the default source is chosen at build time, pass `--source` explicitly
if you are unsure which binary you have.

---

## 2. Pose source (`--source`)

| Value | Meaning |
|-------|---------|
| `vicon` | Read a tracked rigid body from a Vicon DataStream server. Requires the `vicon` feature. |
| `mock` | Synthetic circle trajectory with Gaussian noise; no hardware needed. |

Default: `vicon` in a `--features vicon` build, otherwise `mock`.

Related options:

- `--vicon-host HOST:PORT` — DataStream server (default `127.0.0.1:801`).
- `--subject NAME` — tracked object name; **required** with `--source vicon` and
  must match Vicon Tracker exactly.
- `--segment NAME` — segment to read (default: the subject's root segment).

Source: `src/source.rs` (mock), `src/vicon_source.rs` (Vicon).

---

## 3. Coordinate-frame modes

### Input world frame (`--input-frame`)

| Value | Vicon world axes | Conversion |
|-------|------------------|------------|
| `enu` | X east, Y north, Z up (typical Z-up lab) | to NED |
| `ned` | X north, Y east, Z down | passthrough |

Default: `enu`.

### Body frame (`--body-frame`)

| Value | Rigid-body axes as defined in Vicon Tracker | Use when |
|-------|----------------------------------------------|----------|
| `frd` | x forward, y right, z down (matches MAVLink) | normal |
| `flu` | x forward, y left, z up (ROS) | roll/pitch look inverted with `frd` |

Default: `frd`.

### Attitude (`--attitude`)

| Value | Effect |
|-------|--------|
| `vicon` | Forward the Vicon orientation (rotated/offset into NED). |
| `none` | Ignore Vicon orientation and send level attitude (use when you only trust position). |

Default: `vicon`.

### Yaw alignment (`--yaw-offset-deg`)

Degrees added to the Vicon yaw so the rigid body's forward axis matches the
vehicle nose. Default `0`.

Source: `src/frames.rs`.

---

## 4. Autopilot and message (`--autopilot`, `--message`)

`--autopilot` selects the *default* message for the target stack:

| Value | Default message |
|-------|-----------------|
| `ardupilot` | `ODOMETRY` |
| `px4` | `VISION_POSITION_ESTIMATE` |

`--message` overrides the choice:

| Value | Effect |
|-------|--------|
| `auto` | Pick from `--autopilot` (default). |
| `odometry` | Always send `ODOMETRY` (NED pose, body-FRD velocity, covariance, reset counter). |
| `vision` | Always send `VISION_POSITION_ESTIMATE` (NED pose + RPY, covariance, reset counter). |

Source: `src/mav_out.rs`.

---

## 5. MAVLink transport (`--mavlink`)

Output address and direction:

| Form | Meaning |
|------|---------|
| `udpout:HOST:PORT` | Send pose to that host/port (normal case, e.g. the FC). |
| `udpin:0.0.0.0:PORT` | Listen instead of sending. |

Default: `udpout:127.0.0.1:14550`.

Identity overrides:

- `--mav-sysid N` — MAVLink source system id (default `255`).
- `--mav-compid N` — MAVLink source component id (default `190`,
  `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`).

`--origin "lat,lon,alt_m"` sends `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION`
at start and every 10 s — required for a GPS-less EKF to have a global reference.

---

## 6. Rate, filtering, and coast modes

| Option | Default | Effect |
|--------|---------|--------|
| `--send-hz HZ` | 50 | Pose output rate to the FC. The loop predicts the EKF to now before each send. |
| `--pos-noise-mm MM` | 3 | Vicon measurement noise per axis, fed to the EKF. |
| `--accel-noise A` | 1 | EKF process noise (white-noise acceleration std-dev). Larger = follows motion faster, noisier. |
| `--gate-sigma N` | 6 | Reject a measurement whose innovation exceeds N sigmas on any axis; `0` disables gating. |
| `--max-coast-secs S` | 1 | Coast through dropouts up to S seconds; beyond S output pauses and the EKF re-initializes on recovery (bumps `reset_counter`). |
| `--ang-noise-deg DEG` | 2 | Orientation uncertainty written into the MAVLink covariance fields. |

Source: `src/ekf.rs`, `src/main.rs`.

---

## 7. Mock-source modes (bench testing)

Only meaningful with `--source mock`:

| Option | Default | Effect |
|--------|---------|--------|
| `--mock-hz HZ` | 100 | Rate at which the synthetic trajectory is produced. |
| `--mock-dropout` | off | Insert a periodic occlusion window (0.5 s every 12 s) to exercise coast/recovery. |

---

## 8. Trace mode (`--trace`)

Prints every stage of the pipeline to **stderr**:

- `[TRACE][RCV]` — raw sample received (time, occluded, position, has-quat)
- `[TRACE][FRM]` — ENU→NED conversion
- `[TRACE][EKF]` — EKF decision (Initialized / Updated / Coasted / Rejected / Stale / occluded / reinit)
- `[TRACE][POSE]` — predicted pose sent (NED position, NED and body-FRD velocity, RPY, reset counter)
- `[TRACE][MAV]` — each MAVLink send and origin send
- `[TRACE][LOOP]` — send timing and whether output is currently allowed

Use it with the mock to watch the whole flow with no hardware:

```sh
cargo run --release -- --source mock --trace
```

Note: trace writes synchronously to stderr, so prefer it for debugging rather
than during a live flight at high `--send-hz`.

---

## 9. Tracking-state mode (runtime)

Independent of the flags above, the tool reports its tracking state whenever it
changes (`tracker: state changed A -> B (reason)`):

| State | Meaning |
|-------|---------|
| `Idle` | No sample received yet. |
| `Searching` | Samples arriving but no valid measurement ingested yet. |
| `Tracking` | EKF initialized and updating (or coasting within `--max-coast-secs`). |
| `Lost` | No valid measurement for longer than `--max-coast-secs`; output paused until recovery. |

Transitions on recovery re-initialize the EKF and increment `reset_counter` so
the autopilot knows the estimate restarted.

Source: `src/state.rs`, `src/main.rs`.
