# User Guide — setting up and flying with vicon-drone-tracker

This guide walks through the real-world setup of an indoor Vicon flight, pairing
each step you (the operator) take with what the program does in response. The
physical steps are what actually matter most — the software is only as good as
the calibration, the tracked object, and the coordinate-frame choices.

> New to the tool? Read `README.md` first for build and option reference.
> For transferring the tool to the Vicon PC, see `docs/DEPLOYMENT.md`.
> For a reference of every runtime mode/flag, see `docs/MODES.md`.

---

## Phase 0 — Prerequisites (hardware + software)

**You**
- A Vicon system (cameras + Tracker software) and retro-reflective markers
  attached to the drone.
- The drone's autopilot powered and reachable on the network.
- On the Vicon PC: Rust + VS C++ Build Tools, the Vicon DataStream SDK, and the
  cloned repo (see the README's Windows deployment section).

**Program:** not running yet.

---

## Phase 1 — Calibrate the Vicon volume

**You**
- Power on the cameras and run the **calibration** (wand wave / active).
- **Set the origin and world axes.** This determines whether the world is
  "Z-up ENU" or something else, and where `(0,0,0)` is.

**Program**
- Not involved yet — but remember this choice. A normal Z-up right-handed lab is
  `--input-frame enu` (the default). Position is converted with
  `(N,E,D) = (y, x, −z)` (`src/frames.rs`).

---

## Phase 2 — Define the drone as a tracked object (subject)

**You**
- In Vicon Tracker, create/review the **object (rigid body)** for the drone,
  define its markers, and give it a clear **name** (e.g. `drone`).
- Decide the object's **local axes** while defining it (which way is "forward").
- Confirm the object shows as **Tracked**.

**Program**
- On startup with `--source vicon`, the tool connects to the DataStream server,
  enables segment data, and resolves the subject's **root segment by name**
  (`src/vicon_source.rs`).
- `--subject` must match the Tracker object name **exactly**, or the tool exits
  and lists the available subjects.
- It logs the SDK version and the resolved segment.

---

## Phase 3 — Put the drone in the volume and check visibility

**You**
- Place/hover the drone so **all its markers are visible**. Occluded markers mean
  gaps in the data.
- Watch Tracker: the object should stay smoothly tracked as you move it by hand.

**Program**
- The source thread polls `get_frame()` in a loop.
- If the subject is **occluded**, the sample is ignored — the filter coasts
  instead of ingesting garbage.
- If the **connection drops**, it warns (once per second) and reconnects
  automatically.

---

## Phase 4 — Get the network path right

**You**
- Put the Vicon PC and the flight controller / GCS on the **same LAN**.
- Find the **FC's IP and MAVLink UDP port** (ArduPilot often `14550`).
- Allow the tool through Windows Firewall (outbound UDP).

**Program**
- Output address is `--mavlink udpout:<FC_IP>:<port>`. `udpout` sends to that
  address; use `udpin` only if you need to listen instead.

---

## Phase 5 — Launch the tool

**You**
```powershell
.\target\release\vicon-drone-tracker.exe `
  --source vicon --subject drone `
  --input-frame enu --body-frame frd --attitude vicon `
  --autopilot ardupilot `
  --mavlink udpout:<FC_IP>:14550 `
  --origin "<lat>,<lon>,<alt_m>"
```

**Program**
- Parses flags, builds the frame converter, opens the MAVLink socket, prints what
  it will send.
- Connects to Vicon (see Phase 2 log line).
- **Waits for the first valid measurement**, printing `waiting for first
  measurement` every 2 s until it arrives.

---

## Phase 6 — First measurement / initialization

**You**
- Move the drone slightly so it's tracked, or leave it visible at power-up.

**Program**
- The first non-occluded sample **initializes the EKF**: position = the
  measurement, velocity = 0, fresh covariance.
- From then on it filters: predict forward, then correct with each new position.
- Outlier glitches are rejected by the `--gate-sigma` gate.

---

## Phase 7 — Alignment checks (the fiddly part)

**You**
- **Position:** is the reported position where the drone actually is, in the
  right axes? If it's north but the tool says east, `--input-frame` is wrong.
- **Attitude:** point the nose along your reference (Vicon +X, or +Y for
  ENU-north). Does the reported yaw match? Adjust `--yaw-offset-deg` for a
  constant offset.
- **Body frame:** if roll/pitch look inverted, the object was defined FLU, not
  FRD — switch `--body-frame flu`.

**Program**
- Applies your choices: ENU→NED position, FLU→FRD body swap, then attitude as
  `R_ned_frd = C_enu_ned · R_vicon · C_body`, rotated by `--yaw-offset-deg` about
  NED-down, then converted to roll/pitch/yaw and a quaternion (`src/frames.rs`).
- With `--attitude none`, Vicon orientation is ignored and level attitude is sent
  (useful when you only trust position).

---

## Phase 8 — Tell the autopilot where "here" is (no GPS)

**You**
- Pick a physical reference point and note its `lat,lon,alt`.
- Configure the autopilot to use external vision/odometry (e.g. ArduPilot
  `EK3_SRC`/`VISO` parameters, or PX4 vision parameters).

**Program**
- Sends `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION` at start and every 10 s.
- Streams `ODOMETRY` (ArduPilot) or `VISION_POSITION_ESTIMATE` (PX4) at
  `--send-hz`.

---

## Phase 9 — Live operation

**You**
- Watch the autopilot's position estimate converge; arm and fly when confident.
- Mind marker occlusion — fly where the cameras still see the drone.

**Program**
- Sends pose on a fixed grid, predicting between measurements so output is smooth.
- Rotates NED velocity into **body FRD** for the message.
- **Short dropout:** keeps outputting the predicted (coasted) pose.
- **Long dropout (> `--max-coast-secs`):** pauses output and logs it; on recovery
  it **re-initializes and bumps `reset_counter`** so the autopilot knows the
  estimate was reset.
- Prints a status line every 2 s: position, velocity, measurement age, sent count.
- Logs tracking-state changes (`Idle → Searching → Tracking`, and `Tracking →
  Lost` on a long dropout, back to `Tracking` on recovery). Add `--trace` to see
  every pipeline stage (raw sample, frame conversion, EKF decision, pose sent).

---

## Phase 10 — Shutdown

**You**
- Stop the tool (Ctrl+C).

**Program**
- Stops sending. The autopilot falls back to other position sources (or loses
  position) — don't kill the tool mid-flight without a plan.

---

## Practical gotchas

- **Vicon orientation is forwarded, not filtered** (`--attitude vicon`). Noise in
  the rigid-body rotation goes straight to the autopilot's attitude; fix it by
  improving the object definition and calibration, not in software.
- **The tool is only as good as the object frame.** Most setup problems are
  frame/name mismatches, not filter tuning. Get Phases 2, 4, and 7 right and the
  rest generally just works.
