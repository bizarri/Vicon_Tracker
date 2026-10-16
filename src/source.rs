use std::time::{Duration, Instant};

/// One mocap sample, already converted from millimetres to metres and stamped
/// with a monotonic timestamp (seconds since process start).
#[derive(Debug, Clone, Copy)]
pub struct RawSample {
    pub t: f64,
    /// Position in metres, in the configured input world frame.
    pub pos_m: [f64; 3],
    /// Body -> input-world quaternion in Vicon order (x, y, z, w).
    pub quat_xyzw: Option<[f64; 4]>,
    /// Vicon could not resolve the subject/segment in this frame.
    pub occluded: bool,
}

impl RawSample {
    /// Occluded/absent sample at time `t`: the EKF will coast through it.
    #[cfg(feature = "vicon")]
    pub fn missing(t: f64) -> Self {
        Self {
            t,
            pos_m: [0.0; 3],
            quat_xyzw: None,
            occluded: true,
        }
    }
}

/// Blocking sample producer. `next_sample` returning `None` means the source
/// is finished (used by finite test sources).
pub trait PoseSource: Send {
    fn next_sample(&mut self) -> Option<RawSample>;
}

/// Synthetic Z-up circle trajectory with Gaussian noise, for bench testing.
pub struct MockSource {
    hz: f64,
    step: u64,
    t0: Instant,
    next_due: Instant,
    noise_m: f64,
    dropout: bool,
    rng: u64,
}

impl MockSource {
    pub fn new(hz: f64, noise_m: f64, dropout: bool) -> Self {
        let now = Instant::now();
        Self {
            hz: hz.max(1.0),
            step: 0,
            t0: now,
            next_due: now,
            noise_m,
            dropout,
            rng: 0x1234_5678_9ABC_DEF0,
        }
    }

    fn uniform(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let x = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
        ((x >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn gauss(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

impl PoseSource for MockSource {
    fn next_sample(&mut self) -> Option<RawSample> {
        if self.step > 0 {
            std::thread::sleep(self.next_due.saturating_duration_since(Instant::now()));
        }
        let t = self.step as f64 / self.hz;
        self.step += 1;
        self.next_due = self.t0 + Duration::from_secs_f64(self.step as f64 / self.hz);

        let r = 2.0;
        let w = 0.8;
        let pos = [
            r * (w * t).cos() + self.noise_m * self.gauss(),
            r * (w * t).sin() + self.noise_m * self.gauss(),
            1.0 + 0.4 * (1.7 * t).sin() + self.noise_m * self.gauss(),
        ];
        let yaw = w * t + std::f64::consts::FRAC_PI_2;
        let quat = [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()];
        // periodic dropout window: 0.5 s every 12 s
        let occluded = self.dropout && (t % 12.0) >= 8.0 && (t % 12.0) < 8.5;
        Some(RawSample {
            t,
            pos_m: pos,
            quat_xyzw: Some(quat),
            occluded,
        })
    }
}
