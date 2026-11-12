//! How a stilt walker's body rides its legs: the dip between steps, the lean
//! into a walk, the rock over the planted foot, and the lag of a heavy top
//! behind a change of speed.
//!
//! ```text
//!   speed ───────────► stride share ─┬─► dip (pelvis down, feet both planted)
//!   foot heights ──► which is lifted ┤
//!   velocity change ─► acceleration ─┴─► targets ──spring──► pitch, roll
//! ```
//!
//! The pitch and roll chase their targets on an underdamped spring, so the
//! body overshoots: a walker stopping short nods forward and settles, one
//! rocking from foot to foot swings a little past each.

use nalgebra::{UnitQuaternion, Vector3};

use crate::animation::rig::right_vector;

/// How a walker's body is carried, per creature.
#[derive(Clone, Copy, Debug)]
pub struct Carriage {
    /// How far the pelvis sinks between steps at a full stride, as a share
    /// of standing height: lowest with both feet down, level at mid-swing.
    pub dip: f32,
    /// How far it leans into its walk at a full stride, radians.
    pub lean: f32,
    /// The speed at which the stride is full, m/s: the dip and lean grow
    /// with speed up to it.
    pub stride_speed: f32,
    /// How far it rocks over its planted foot while the other is at the top
    /// of its swing, radians.
    pub sway: f32,
    /// How far it rocks from side to side standing still, radians.
    pub idle_sway: f32,
    /// How far its top tips behind a change of velocity, radians per m/s²:
    /// back as it sets off, forward as it stops, outward on a turn.
    pub heft: f32,
    /// Natural frequency of its rock on its legs, rad/s.
    pub frequency: f32,
    /// Damping ratio of that rock: under 1 and it swings past.
    pub damping: f32,
}

impl Default for Carriage {
    fn default() -> Self {
        Self {
            dip: 0.08,
            lean: 0.07,
            stride_speed: 2.5,
            sway: 0.085,
            idle_sway: 0.012,
            heft: 0.03,
            frequency: 11.0,
            damping: 0.35,
        }
    }
}

/// Largest acceleration the heft answers to, m/s². A creature knocked by a
/// blast is thrown, not carried, and its body should not fold flat.
const MAX_ACCELERATION: f32 = 8.0;

/// How fast the acceleration estimate follows the body, 1/s. Physics
/// velocity jitters frame to frame; the eye reads only the trend.
const ACCELERATION_RATE: f32 = 10.0;

/// How fast the dip follows the stride, 1/s.
const DIP_RATE: f32 = 20.0;

/// Seconds a slow idle rock takes, as a rate in rad/s.
const IDLE_RATE: f32 = 0.7;

/// What a carriage needs to know each frame.
pub struct Gait {
    /// Body velocity relative to what holds it up, m/s.
    pub velocity: Vector3<f32>,
    /// Horizontal facing.
    pub facing: Vector3<f32>,
    /// Which foot is lifted and how high: positive the left, negative the
    /// right, 1 at the top of a swing.
    pub lifted: f32,
    /// Seconds alive, for the idle rock.
    pub clock: f32,
}

/// A carriage moving.
#[derive(Clone, Debug, Default)]
pub struct CarriageMotion {
    previous_velocity: Option<Vector3<f32>>,
    acceleration: Vector3<f32>,
    /// Pelvis offset below where the body would put it, m.
    dip: f32,
    /// Forward tip, radians, and how fast it is changing.
    pitch: f32,
    pitch_rate: f32,
    /// Lean to the walker's right, radians, and how fast it is changing.
    roll: f32,
    roll_rate: f32,
}

impl CarriageMotion {
    /// Advance one frame of `dt` seconds for a walker carried as
    /// `carriage`, standing `standing_height` above its feet.
    pub fn step(&mut self, carriage: &Carriage, standing_height: f32, dt: f32, gait: &Gait) {
        if dt <= 0.0 {
            return;
        }
        let measured = self
            .previous_velocity
            .map_or(Vector3::zeros(), |previous| (gait.velocity - previous) / dt);
        self.previous_velocity = Some(gait.velocity);
        let measured = Vector3::new(measured.x, 0.0, measured.z);
        self.acceleration += (measured - self.acceleration) * approach(ACCELERATION_RATE, dt);
        let acceleration = self.acceleration.cap_magnitude(MAX_ACCELERATION);

        let speed = Vector3::new(gait.velocity.x, 0.0, gait.velocity.z).norm();
        let stride = (speed / carriage.stride_speed.max(1e-3)).clamp(0.0, 1.0);
        let lifted = gait.lifted.clamp(-1.0, 1.0);

        let dip = carriage.dip * standing_height * stride * (1.0 - lifted.abs());
        self.dip += (dip - self.dip) * approach(DIP_RATE, dt);

        let right = right_vector(gait.facing);
        let pitch_target = carriage.lean * stride - carriage.heft * acceleration.dot(&gait.facing);
        let roll_target = carriage.sway * lifted
            + carriage.idle_sway * (gait.clock * IDLE_RATE).sin()
            - carriage.heft * acceleration.dot(&right);

        let (omega, zeta) = (carriage.frequency, carriage.damping);
        spring(
            &mut self.pitch,
            &mut self.pitch_rate,
            pitch_target,
            omega,
            zeta,
            dt,
        );
        spring(
            &mut self.roll,
            &mut self.roll_rate,
            roll_target,
            omega,
            zeta,
            dt,
        );
    }

    /// How far below the body's own pelvis the rig's pelvis is put, m.
    pub fn dip(&self) -> f32 {
        self.dip
    }

    /// The body's tilt over its hips: the forward tip, then the lean to its
    /// right, for a walker facing `facing`.
    pub fn tilt(&self, facing: Vector3<f32>) -> UnitQuaternion<f32> {
        let forward_axis = Vector3::y().cross(&facing);
        let tip = UnitQuaternion::from_scaled_axis(forward_axis * self.pitch);
        let lean = UnitQuaternion::from_scaled_axis(facing * self.roll);
        lean * tip
    }

    /// Forward tip, radians.
    pub fn pitch(&self) -> f32 {
        self.pitch
    }

    /// Lean to the walker's right, radians.
    pub fn roll(&self) -> f32 {
        self.roll
    }
}

/// One step of a damped spring chasing `target`, integrated semi-implicitly
/// so it stays stable at any frame rate the game runs at.
fn spring(value: &mut f32, rate: &mut f32, target: f32, omega: f32, zeta: f32, dt: f32) {
    // Substeps keep a stiff spring stable through a long frame.
    let steps = ((dt * omega / 0.25).ceil() as usize).max(1);
    let h = dt / steps as f32;
    for _ in 0..steps {
        let pull = omega * omega * (target - *value) - 2.0 * zeta * omega * *rate;
        *rate += pull * h;
        *value += *rate * h;
    }
}

/// The fraction of the way to a target an exponential chase covers in
/// `dt` at `rate` per second.
#[inline]
fn approach(rate: f32, dt: f32) -> f32 {
    1.0 - (-rate * dt).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn gait(velocity: Vector3<f32>, lifted: f32) -> Gait {
        Gait {
            velocity,
            facing: Vector3::z(),
            lifted,
            clock: 0.0,
        }
    }

    fn run(motion: &mut CarriageMotion, seconds: f32, gait: &Gait) {
        for _ in 0..(seconds / DT) as usize {
            motion.step(&Carriage::default(), 1.0, DT, gait);
        }
    }

    #[test]
    fn standing_still_it_neither_dips_nor_leans() {
        let mut motion = CarriageMotion::default();
        run(&mut motion, 2.0, &gait(Vector3::zeros(), 0.0));
        assert!(motion.dip().abs() < 1e-4);
        assert!(motion.pitch().abs() < 1e-3);
    }

    #[test]
    fn walking_it_leans_forward_and_sinks_with_both_feet_down() {
        let mut motion = CarriageMotion::default();
        let walking = Vector3::z() * 3.0;
        run(&mut motion, 2.0, &gait(walking, 0.0));
        let carriage = Carriage::default();
        assert!(
            (motion.pitch() - carriage.lean).abs() < 0.005,
            "{}",
            motion.pitch()
        );
        assert!(
            (motion.dip() - carriage.dip).abs() < 0.002,
            "{}",
            motion.dip()
        );

        run(&mut motion, 1.0, &gait(walking, 1.0));
        assert!(
            motion.dip() < 0.002,
            "mid-swing it rides level: {}",
            motion.dip()
        );
    }

    #[test]
    fn it_tips_back_setting_off_and_forward_stopping_short() {
        let mut motion = CarriageMotion::default();
        run(&mut motion, 0.5, &gait(Vector3::zeros(), 0.0));
        let mut least = 0.0f32;
        for frame in 0..30 {
            let speed = (frame as f32 * 0.1).min(3.0);
            motion.step(
                &Carriage::default(),
                1.0,
                DT,
                &gait(Vector3::z() * speed, 0.0),
            );
            least = least.min(motion.pitch());
        }
        assert!(least < -0.02, "it never tipped back: {least}");

        run(&mut motion, 2.0, &gait(Vector3::z() * 3.0, 0.0));
        let walking = motion.pitch();
        let mut most = walking;
        for frame in 0..30 {
            let speed = (3.0 - frame as f32 * 0.1).max(0.0);
            motion.step(
                &Carriage::default(),
                1.0,
                DT,
                &gait(Vector3::z() * speed, 0.0),
            );
            most = most.max(motion.pitch());
        }
        assert!(most > walking + 0.02, "it never nodded forward: {most}");
    }

    #[test]
    fn it_rocks_past_the_planted_foot_and_back() {
        let mut motion = CarriageMotion::default();
        run(&mut motion, 1.0, &gait(Vector3::zeros(), 0.0));
        let mut most = 0.0f32;
        for _ in 0..30 {
            motion.step(&Carriage::default(), 1.0, DT, &gait(Vector3::zeros(), 1.0));
            most = most.max(motion.roll());
        }
        let sway = Carriage::default().sway;
        assert!(most > sway, "underdamped, it swings past: {most}");
        assert!(most < 1.6 * sway, "but not wildly: {most}");
    }

    #[test]
    fn a_blast_does_not_fold_it_flat() {
        let mut motion = CarriageMotion::default();
        run(&mut motion, 0.5, &gait(Vector3::zeros(), 0.0));
        motion.step(
            &Carriage::default(),
            1.0,
            DT,
            &gait(Vector3::x() * 30.0, 0.0),
        );
        run(&mut motion, 0.5, &gait(Vector3::x() * 30.0, 0.0));
        assert!(motion.roll().abs() < 0.6 && motion.pitch().abs() < 0.6);
    }
}
