use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

use crate::cli::{Attitude, BodyFrame, InputFrame};

/// Converts raw Vicon world-frame poses (metres, input frame) into the NED /
/// body-FRD convention expected by MAVLink vision/odometry messages.
pub struct Converter {
    world_to_ned: Matrix3<f64>,
    body_to_input: Matrix3<f64>,
    yaw_offset: f64,
    attitude: Attitude,
}

#[derive(Debug, Clone, Copy)]
pub struct NedAttitude {
    /// roll, pitch, yaw (rad), aerospace ZYX in NED
    pub rpy: [f64; 3],
    /// body -> NED quaternion, w, x, y, z
    pub quat_wxyz: [f64; 4],
    /// body -> NED rotation matrix
    pub rot: Matrix3<f64>,
}

impl Converter {
    pub fn new(
        input: InputFrame,
        body: BodyFrame,
        attitude: Attitude,
        yaw_offset_deg: f64,
    ) -> Self {
        // ENU -> NED: north = enu.y, east = enu.x, down = -enu.z.
        // The matrix is its own inverse.
        let world_to_ned = match input {
            InputFrame::Enu => Matrix3::new(
                0.0, 1.0, 0.0, //
                1.0, 0.0, 0.0, //
                0.0, 0.0, -1.0,
            ),
            InputFrame::Ned => Matrix3::identity(),
        };
        // If the tracked rigid body is FLU, express it in FRD coordinates.
        let body_to_input = match body {
            BodyFrame::Frd => Matrix3::identity(),
            BodyFrame::Flu => Matrix3::new(
                1.0, 0.0, 0.0, //
                0.0, -1.0, 0.0, //
                0.0, 0.0, -1.0,
            ),
        };
        Self {
            world_to_ned,
            body_to_input,
            yaw_offset: yaw_offset_deg.to_radians(),
            attitude,
        }
    }

    /// Position, metres, input world frame -> NED.
    pub fn pos_to_ned(&self, p: [f64; 3]) -> Vector3<f64> {
        self.world_to_ned * Vector3::new(p[0], p[1], p[2])
    }

    /// Vicon body->world quaternion (x, y, z, w) -> attitude in NED / body FRD.
    pub fn attitude_ned(&self, quat_xyzw: Option<[f64; 4]>) -> NedAttitude {
        if self.attitude == Attitude::None {
            return NedAttitude {
                rpy: [0.0; 3],
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
                rot: Matrix3::identity(),
            };
        }
        let Some([x, y, z, w]) = quat_xyzw else {
            return NedAttitude {
                rpy: [0.0; 3],
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
                rot: Matrix3::identity(),
            };
        };
        let q = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
        let r_input = q.to_rotation_matrix().into_inner();
        // input-body FRD -> NED
        let r_frd_ned = self.world_to_ned * r_input * self.body_to_input;
        // Rotate about NED down (z) axis: Rz(offset) * R.
        let rz = Matrix3::new(
            self.yaw_offset.cos(),
            -self.yaw_offset.sin(),
            0.0, //
            self.yaw_offset.sin(),
            self.yaw_offset.cos(),
            0.0, //
            0.0,
            0.0,
            1.0,
        );
        let r = rz * r_frd_ned;
        let (roll, pitch, yaw) = euler_zyx(&r);
        let quat =
            UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(r));
        let q = quat.quaternion();
        NedAttitude {
            rpy: [roll, pitch, yaw],
            quat_wxyz: [q.w, q.i, q.j, q.k],
            rot: r,
        }
    }
}

/// ZYX (yaw-pitch-roll) extraction: R = Rz(yaw) Ry(pitch) Rx(roll).
fn euler_zyx(r: &Matrix3<f64>) -> (f64, f64, f64) {
    let yaw = r[(1, 0)].atan2(r[(0, 0)]);
    let pitch = (-r[(2, 0)]).clamp(-1.0, 1.0).asin();
    let roll = r[(2, 1)].atan2(r[(2, 2)]);
    (roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn quat_xyzw(m: Matrix3<f64>) -> [f64; 4] {
        let uq =
            UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(m));
        let q = uq.quaternion();
        [q.i, q.j, q.k, q.w]
    }

    #[test]
    fn enu_to_ned_position() {
        let c = Converter::new(InputFrame::Enu, BodyFrame::Frd, Attitude::None, 0.0);
        let p = c.pos_to_ned([1.0, 2.0, 3.0]);
        assert!(approx(p.x, 2.0, 1e-12));
        assert!(approx(p.y, 1.0, 1e-12));
        assert!(approx(p.z, -3.0, 1e-12));
    }

    #[test]
    fn ned_passthrough_position() {
        let c = Converter::new(InputFrame::Ned, BodyFrame::Frd, Attitude::None, 0.0);
        let p = c.pos_to_ned([1.0, 2.0, 3.0]);
        assert!(approx(p.x, 1.0, 1e-12));
        assert!(approx(p.y, 2.0, 1e-12));
        assert!(approx(p.z, 3.0, 1e-12));
    }

    #[test]
    fn roundtrip_ned_attitude_through_enu() {
        // A known NED attitude, expressed the way Vicon would report it for a
        // FRD body in an ENU world: R_enu = C * R_ned.
        let ned = UnitQuaternion::from_euler_angles(
            10f64.to_radians(),
            -5f64.to_radians(),
            40f64.to_radians(),
        );
        let c = Matrix3::new(
            0.0, 1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, -1.0,
        );
        let r_enu = c * ned.to_rotation_matrix().into_inner();
        let conv = Converter::new(InputFrame::Enu, BodyFrame::Frd, Attitude::Vicon, 0.0);
        let att = conv.attitude_ned(Some(quat_xyzw(r_enu)));
        let want = [10f64.to_radians(), -5f64.to_radians(), 40f64.to_radians()];
        assert!(approx(att.rpy[0], want[0], 1e-9), "roll {}", att.rpy[0]);
        assert!(approx(att.rpy[1], want[1], 1e-9), "pitch {}", att.rpy[1]);
        assert!(approx(att.rpy[2], want[2], 1e-9), "yaw {}", att.rpy[2]);
    }

    #[test]
    fn enu_yaw_90_frd_body_is_level_north() {
        // FRD body facing north, level: in ENU its axes are x=north, y=east,
        // z=down, which is exactly the ENU<->NED change-of-basis matrix C.
        let r_enu = Matrix3::new(
            0.0, 1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, -1.0,
        );
        let c = Converter::new(InputFrame::Enu, BodyFrame::Frd, Attitude::Vicon, 0.0);
        let att = c.attitude_ned(Some(quat_xyzw(r_enu)));
        assert!(approx(att.rpy[2], 0.0, 1e-9), "yaw = {}", att.rpy[2]);
        assert!(approx(att.rpy[0], 0.0, 1e-9), "roll = {}", att.rpy[0]);
        assert!(approx(att.rpy[1], 0.0, 1e-9), "pitch = {}", att.rpy[1]);
    }

    #[test]
    fn yaw_offset_adds() {
        // north-facing FRD body (R_enu = C), plus a 30 deg yaw offset
        let r_enu = Matrix3::new(
            0.0, 1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, -1.0,
        );
        let c = Converter::new(InputFrame::Enu, BodyFrame::Frd, Attitude::Vicon, 30.0);
        let att = c.attitude_ned(Some(quat_xyzw(r_enu)));
        assert!(approx(att.rpy[2], 30f64.to_radians(), 1e-9));
    }

    #[test]
    fn flu_body_facing_north_is_level() {
        // FLU body (ROS convention) facing north, level: x=north, y=west, z=up in ENU.
        let r_enu_flu = Matrix3::new(
            0.0, -1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0,
        );
        let c = Converter::new(InputFrame::Enu, BodyFrame::Flu, Attitude::Vicon, 0.0);
        let att = c.attitude_ned(Some(quat_xyzw(r_enu_flu)));
        assert!(approx(att.rpy[2], 0.0, 1e-9), "yaw = {}", att.rpy[2]);
        assert!(approx(att.rpy[0], 0.0, 1e-9), "roll = {}", att.rpy[0]);
        assert!(approx(att.rpy[1], 0.0, 1e-9), "pitch = {}", att.rpy[1]);
    }
}
