use nalgebra::{SMatrix, SVector, Vector3};

pub type State = SVector<f64, 6>;
pub type Cov = SMatrix<f64, 6, 6>;

const MIN_DT: f64 = 1e-4;
const MAX_DT: f64 = 0.5;
const STALE_DT: f64 = -1e-3;

#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    /// Position measurement standard deviation per axis (m)
    pub pos_noise_m: f64,
    /// White-noise acceleration standard deviation (m/s^2)
    pub accel_noise: f64,
    /// Initial velocity standard deviation per axis (m/s)
    pub vel_init_sigma: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            pos_noise_m: 0.003,
            accel_noise: 1.0,
            vel_init_sigma: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    Initialized,
    Updated,
    /// Predicted only: measurement was rejected by the innovation gate
    Rejected,
    /// Predicted only: no measurement in this step
    Coasted,
    /// Measurement timestamp older than the filter state; ignored
    Stale,
}

/// Constant-velocity EKF over [x, y, z, vx, vy, vz] (NED, metres / m/s)
/// with position-only measurements.
pub struct Ekf {
    cfg: EkfConfig,
    x: State,
    p: Cov,
    last_t: f64,
    initialized: bool,
}

impl Ekf {
    pub fn new(cfg: EkfConfig) -> Self {
        Self {
            cfg,
            x: State::zeros(),
            p: Cov::zeros(),
            last_t: 0.0,
            initialized: false,
        }
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn state(&self) -> &State {
        &self.x
    }

    pub fn pos(&self) -> Vector3<f64> {
        self.x.fixed_rows::<3>(0).into_owned()
    }

    #[cfg(test)]
    pub fn vel(&self) -> Vector3<f64> {
        self.x.fixed_rows::<3>(3).into_owned()
    }

    pub fn pos_var(&self) -> [f64; 3] {
        [self.p[(0, 0)], self.p[(1, 1)], self.p[(2, 2)]]
    }

    pub fn vel_var(&self) -> [f64; 3] {
        [self.p[(3, 3)], self.p[(4, 4)], self.p[(5, 5)]]
    }

    /// (Re-)initialize from a position measurement at time `t`.
    pub fn init(&mut self, t: f64, z: Vector3<f64>) {
        self.x = State::zeros();
        self.x.fixed_rows_mut::<3>(0).copy_from(&z);
        self.p = Cov::zeros();
        let r = self.cfg.pos_noise_m * self.cfg.pos_noise_m;
        let v = self.cfg.vel_init_sigma * self.cfg.vel_init_sigma;
        for i in 0..3 {
            self.p[(i, i)] = r;
            self.p[(i + 3, i + 3)] = v;
        }
        self.last_t = t;
        self.initialized = true;
    }

    /// Advance the state to time `t` without applying a measurement.
    pub fn predict_to(&mut self, t: f64) {
        if !self.initialized {
            return;
        }
        let dt = t - self.last_t;
        if dt <= 0.0 {
            return;
        }
        self.predict(dt);
        self.last_t = t;
    }

    /// Predict to `t` and apply the position measurement (innovation-gated).
    ///
    /// `gate_sigma` is a per-axis innovation gate in units of the innovation
    /// standard deviation; values <= 0 disable gating.
    pub fn process(&mut self, t: f64, z: Vector3<f64>, gate_sigma: f64) -> Step {
        if !self.initialized {
            self.init(t, z);
            return Step::Initialized;
        }
        let dt = t - self.last_t;
        if dt < STALE_DT {
            return Step::Stale;
        }
        self.predict(dt.clamp(MIN_DT, MAX_DT));
        self.last_t = t;

        let h = self.measurement(&z);
        match h {
            None => Step::Coasted,
            Some((y, s)) => {
                if gate_sigma > 0.0 && self.gated(&y, &s, gate_sigma) {
                    Step::Rejected
                } else {
                    self.correct(&y, &s);
                    Step::Updated
                }
            }
        }
    }

    fn predict(&mut self, dt: f64) {
        let mut f = Cov::identity();
        for i in 0..3 {
            f[(i, i + 3)] = dt;
        }

        let q = self.cfg.accel_noise * self.cfg.accel_noise;
        let mut qc = Cov::zeros();
        for i in 0..3 {
            qc[(i, i)] = q * dt * dt * dt / 3.0;
            qc[(i, i + 3)] = q * dt * dt / 2.0;
            qc[(i + 3, i)] = q * dt * dt / 2.0;
            qc[(i + 3, i + 3)] = q * dt;
        }

        self.x = f * self.x;
        self.p = f * self.p * f.transpose() + qc;
    }

    /// Innovation `y = z - x_pos` and innovation covariance `S = P_pos + R`.
    fn measurement(
        &self,
        z: &Vector3<f64>,
    ) -> Option<(Vector3<f64>, nalgebra::SMatrix<f64, 3, 3>)> {
        let r = self.cfg.pos_noise_m * self.cfg.pos_noise_m;
        let s = self.p.fixed_view::<3, 3>(0, 0) + nalgebra::SMatrix::<f64, 3, 3>::identity() * r;
        let y = z - self.pos();
        Some((y, s))
    }

    fn gated(&self, y: &Vector3<f64>, s: &nalgebra::SMatrix<f64, 3, 3>, gate_sigma: f64) -> bool {
        for i in 0..3 {
            let sigma = s[(i, i)].max(0.0).sqrt();
            if sigma > 0.0 && y[i].abs() > gate_sigma * sigma {
                return true;
            }
        }
        false
    }

    /// Joseph-form update. `s` must be the innovation covariance from `measurement`.
    fn correct(&mut self, y: &Vector3<f64>, s: &nalgebra::SMatrix<f64, 3, 3>) {
        let Some(s_inv) = s.try_inverse() else {
            return;
        };
        // K = P * H^T * S^-1 with H = [I 0]; P * H^T is P's first three columns.
        let mut ph_cols = Cov::zeros();
        for r in 0..6 {
            for c in 0..3 {
                ph_cols[(r, c)] = self.p[(r, c)];
            }
        } // 6x3 stored in 6x6 layout, columns 0..3

        let mut k = nalgebra::SMatrix::<f64, 6, 3>::zeros();
        for r in 0..6 {
            for c in 0..3 {
                let mut acc = 0.0;
                for m in 0..3 {
                    acc += ph_cols[(r, m)] * s_inv[(m, c)];
                }
                k[(r, c)] = acc;
            }
        }

        for r in 0..6 {
            for c in 0..3 {
                self.x[r] += k[(r, c)] * y[c];
            }
        }

        // P = (I - K H) P (I - K H)^T + K R K^T,  H picks the first three columns
        let r_scalar = self.cfg.pos_noise_m * self.cfg.pos_noise_m;
        let mut ikh = Cov::identity();
        for r in 0..6 {
            for c in 0..3 {
                ikh[(r, c)] -= k[(r, c)];
            }
        }
        self.p = ikh * self.p * ikh.transpose() + (k * k.transpose()) * r_scalar;

        // keep exact symmetry against float drift
        self.p = (self.p + self.p.transpose()) * 0.5;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_trajectory(noise: f64, seed: u64) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
        // ground truth: circle in x/y with a sinusoidal altitude
        let dt = 0.01;
        let r = 2.0;
        let w = 0.8;
        let mut ekf = Ekf::new(EkfConfig::default());
        let mut rng = seed | 1;
        let mut gauss = || {
            rng ^= rng >> 12;
            rng ^= rng << 25;
            rng ^= rng >> 27;
            let u = ((rng.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64) / (1u64 << 53) as f64;
            let u2 = (((rng ^ 0x9E3779B97F4A7C15).wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64)
                / (1u64 << 53) as f64;
            (-2.0 * u.max(1e-12).ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        };

        let mut est = Vec::new();
        let mut truth = Vec::new();
        for k in 0..600 {
            let t = k as f64 * dt;
            let gt = Vector3::new(
                r * (w * t).cos(),
                r * (w * t).sin(),
                1.0 + 0.4 * (1.7 * t).sin(),
            );
            let meas = gt + Vector3::new(gauss(), gauss(), gauss()) * noise;
            ekf.process(t, meas, 6.0);
            if k > 100 {
                est.push(ekf.pos());
                truth.push(gt);
            }
        }
        (est, truth)
    }

    #[test]
    fn converges_on_noisy_circle() {
        let (est, truth) = run_trajectory(0.003, 42);
        let mut err = 0.0;
        for (e, t) in est.iter().zip(truth.iter()) {
            err += (e - t).norm_squared();
        }
        let rmse = (err / est.len() as f64).sqrt();
        assert!(rmse < 0.01, "position rmse too large: {rmse:.4} m");
    }

    #[test]
    fn velocity_estimates_converge() {
        let dt = 0.01;
        let r = 2.0;
        let w = 0.8;
        let mut ekf = Ekf::new(EkfConfig::default());
        let mut rng = 7u64;
        let mut gauss = || {
            rng ^= rng >> 12;
            rng ^= rng << 25;
            rng ^= rng >> 27;
            let u = ((rng.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64) / (1u64 << 53) as f64;
            let u2 = (((rng ^ 0x9E3779B97F4A7C15).wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64)
                / (1u64 << 53) as f64;
            (-2.0 * u.max(1e-12).ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        };
        let mut sum_verr2 = 0.0f64;
        let mut n_after_settle = 0.0f64;
        for k in 0..600 {
            let t = k as f64 * dt;
            let gt = Vector3::new(r * (w * t).cos(), r * (w * t).sin(), 1.0);
            let meas = gt + Vector3::new(gauss(), gauss(), gauss()) * 0.003;
            ekf.process(t, meas, 6.0);
            if k > 300 {
                let gt_vel = Vector3::new(-r * w * (w * t).sin(), r * w * (w * t).cos(), 0.0);
                sum_verr2 += (ekf.vel() - gt_vel).norm_squared();
                n_after_settle += 1.0;
            }
        }
        let rmse = (sum_verr2 / n_after_settle).sqrt();
        assert!(
            rmse < 0.15,
            "velocity rmse too large: {rmse:.3} m/s (speed is {:.2})",
            r * w
        );
    }

    #[test]
    fn gate_rejects_outlier() {
        let mut ekf = Ekf::new(EkfConfig::default());
        let t = 0.0;
        ekf.process(t, Vector3::new(0.0, 0.0, 1.0), 6.0);
        ekf.process(t + 0.01, Vector3::new(0.0, 0.0, 1.0), 6.0);
        let before = ekf.state().into_owned();
        // 1 m jump in x is far beyond the gate right after initialization
        let step = ekf.process(t + 0.02, Vector3::new(1.0, 0.0, 1.0), 6.0);
        assert_eq!(step, Step::Rejected);
        let after = ekf.state();
        assert!(
            (after[0] - before[0]).abs() < 1e-9,
            "state must not move on rejection"
        );
    }

    #[test]
    fn stale_measurement_ignored() {
        let mut ekf = Ekf::new(EkfConfig::default());
        ekf.process(1.0, Vector3::new(0.0, 0.0, 1.0), 6.0);
        let step = ekf.process(0.5, Vector3::new(9.0, 9.0, 9.0), 6.0);
        assert_eq!(step, Step::Stale);
        assert!((ekf.pos()[0]).abs() < 1e-6);
    }

    #[test]
    fn coast_grows_uncertainty() {
        let mut ekf = Ekf::new(EkfConfig::default());
        ekf.process(0.0, Vector3::new(0.0, 0.0, 1.0), 6.0);
        let var_before = ekf.pos_var()[0];
        ekf.predict_to(1.0);
        let var_after = ekf.pos_var()[0];
        assert!(
            var_after > var_before * 10.0,
            "coasting must grow position uncertainty: {var_before} -> {var_after}"
        );
    }
}
