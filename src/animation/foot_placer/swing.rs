//! Swing-leg trajectory.
//!
//! A swinging foot carries its own motion: position and velocity along
//! its path, and a lift over that path. Each substep
//! it steers from where it is onto where it lands, over the time left,
//! along the cubic that carries its velocity and comes to rest there: on
//! a target that holds still, the smoothstep a swing has always followed. Whatever moves the landing or its time
//! mid-swing (a plan changing its mind, a flight taking the foot up, the
//! probe finding the ground) bends the path from where the foot is; it
//! never moves the foot.

use nalgebra::{Point3, Vector3};

use super::planned::StepId;

/// Least fraction of a swing left that a retime spreads its time over,
/// so a swing retimed at its very end keeps a finite duration.
const MIN_LEFT: f32 = 1.0e-3;

/// One foot's swing: where it left, where and when it lands, and the
/// motion carrying it there.
#[derive(Clone, Copy, Debug)]
pub struct Swing {
    /// Where the foot left the ground.
    pub from: Point3<f32>,
    /// Where the foot lands.
    pub to: Point3<f32>,
    /// Toe direction at takeoff.
    pub from_forward: Vector3<f32>,
    /// Toe direction at landing.
    pub to_forward: Vector3<f32>,
    /// Seconds into the swing. With `duration`, it gives the swing's
    /// progress (`t / duration`) and the time left (`duration - t`); a
    /// retime keeps the progress and changes the time left.
    pub t: f32,
    /// Seconds the swing lasts.
    pub duration: f32,
    /// How high the foot is lifted over its path at mid-swing.
    pub peak_lift: f32,
    /// The foot on its path, without the lift.
    path: Point3<f32>,
    /// The path's velocity. Its acceleration is not carried: a path bent
    /// hard onto a target moving every substep, as a held swing's is, would
    /// hand the next path an acceleration that throws the foot.
    velocity: Vector3<f32>,
    /// The lift drawn, chasing `peak_lift`.
    lift: f32,
    /// The extra lift that clears the ground under the foot, chasing
    /// what the ground asks for.
    clearance: f32,
    /// What the ground under the foot asks of the clearance.
    clearance_wanted: f32,
    /// Whether the swing found nowhere to land when it was due, and goes on
    /// after a target gathering under the hip.
    pub held: bool,
    /// The planned step `to` steers onto, if it steers onto one.
    pub footfall: Option<StepId>,
}

impl Swing {
    /// A swing leaving `from` at `velocity`, landing on `to` in `duration`.
    pub fn new(
        from: Point3<f32>,
        to: Point3<f32>,
        from_forward: Vector3<f32>,
        to_forward: Vector3<f32>,
        duration: f32,
        peak_lift: f32,
        velocity: Vector3<f32>,
    ) -> Self {
        Self {
            from,
            to,
            from_forward,
            to_forward,
            t: 0.0,
            duration,
            peak_lift,
            path: from,
            velocity,
            lift: peak_lift,
            clearance: 0.0,
            clearance_wanted: 0.0,
            held: false,
            footfall: None,
        }
    }

    /// How far through the swing the foot is, in `[0, 1]`.
    pub fn progress(&self) -> f32 {
        (self.t / self.duration.max(1e-4)).clamp(0.0, 1.0)
    }

    /// Seconds until the foot lands.
    pub fn remaining(&self) -> f32 {
        (self.duration - self.t).max(0.0)
    }

    /// Land `remaining` seconds from now, keeping the swing's progress.
    /// Never sooner than `soonest` unless the landing was already due
    /// sooner: a foot asked to land at once would cover the rest of its
    /// path in a frame.
    pub fn retime(&mut self, remaining: f32, soonest: f32) {
        let remaining = remaining.max(soonest.min(self.remaining()));
        if (remaining - self.remaining()).abs() <= 1.0e-6 {
            return;
        }
        let left = (1.0 - self.progress()).max(MIN_LEFT);
        let progress = 1.0 - left;
        self.duration = remaining / left;
        self.t = progress * self.duration;
    }

    /// Ask for `wanted` metres of extra lift where the foot is now, to
    /// clear the ground under it.
    pub fn clear(&mut self, wanted: f32) {
        self.clearance_wanted = wanted.max(0.0);
    }

    /// Where the foot is drawn: on its path, lifted.
    pub fn position(&self) -> Point3<f32> {
        let u = self.progress();
        let arc = self.lift * 4.0 * u * (1.0 - u);
        let over = self.clearance * (std::f32::consts::PI * u).sin();
        self.path + Vector3::y() * arc.max(over)
    }

    /// Advance by `dt`: the path steers onto `to` over the time left, and
    /// the lift and clearance close `chase` of their gap.
    pub fn advance(&mut self, dt: f32, chase: f32) {
        let remaining = self.remaining();
        if remaining <= dt {
            self.path = self.to;
            self.velocity = Vector3::zeros();
        } else {
            let (path, velocity) = cubic_step(self.path, self.velocity, self.to, remaining, dt);
            self.path = path;
            self.velocity = velocity;
        }
        self.t += dt;
        self.lift += (self.peak_lift - self.lift) * chase;
        self.clearance += (self.clearance_wanted - self.clearance) * chase;
    }

    /// Move the whole swing with the surface under it.
    pub fn carry(&mut self, by: Vector3<f32>) {
        self.from += by;
        self.to += by;
        self.path += by;
    }

    /// The foot on its path, without the lift.
    pub fn path(&self) -> Point3<f32> {
        self.path
    }
}

/// One step of `dt` along the cubic from `position`, moving at `velocity`,
/// that comes to rest on `target` in `time`. From rest onto a still target
/// it is smoothstep; planned afresh from the current motion every step, it
/// stays smooth through a target or time that changes along the way.
fn cubic_step(
    position: Point3<f32>,
    velocity: Vector3<f32>,
    target: Point3<f32>,
    time: f32,
    dt: f32,
) -> (Point3<f32>, Vector3<f32>) {
    let (d, v, t) = (target - position, velocity, time);
    let c2 = (d * 3.0 - v * (2.0 * t)) / (t * t);
    let c3 = (d * -2.0 + v * t) / (t * t * t);
    let position = position + v * dt + c2 * (dt * dt) + c3 * (dt * dt * dt);
    let velocity = v + c2 * (2.0 * dt) + c3 * (3.0 * dt * dt);
    (position, velocity)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn swing(duration: f32) -> Swing {
        Swing::new(
            Point3::origin(),
            Point3::new(0.0, 0.0, 1.0),
            Vector3::z(),
            Vector3::z(),
            duration,
            0.1,
            Vector3::zeros(),
        )
    }

    #[test]
    fn a_swing_lands_on_its_target_on_time() {
        let mut swing = swing(0.3);
        let mut frames = 0;
        while swing.remaining() > 0.0 {
            swing.advance(DT, 1.0);
            frames += 1;
        }
        assert_eq!(frames, 18);
        assert!((swing.position() - swing.to).norm() < 1e-5);
    }

    #[test]
    fn a_target_moved_every_frame_moves_the_foot_no_faster_than_its_swing() {
        let (mut steady, mut churned) = (swing(0.3), swing(0.3));
        let (mut fastest_steady, mut fastest_churned) = (0.0f32, 0.0f32);
        for frame in 0..17 {
            if churned.progress() < 0.9 {
                let side = if frame % 2 == 0 { 0.3 } else { -0.3 };
                churned.to = Point3::new(side, 0.0, 1.0);
            }
            for (swing, fastest) in [
                (&mut steady, &mut fastest_steady),
                (&mut churned, &mut fastest_churned),
            ] {
                let before = swing.position();
                swing.advance(DT, 1.0);
                *fastest = fastest.max((swing.position() - before).norm());
            }
        }
        assert!(
            fastest_churned < 2.0 * fastest_steady,
            "a churned target moved the foot {fastest_churned:.3} m in a frame, \
             a steady one {fastest_steady:.3} m"
        );
    }

    #[test]
    fn a_retime_keeps_the_foot_where_it_is() {
        let mut swing = swing(0.3);
        for _ in 0..6 {
            swing.advance(DT, 1.0);
        }
        let before = swing.position();
        swing.retime(0.6, 0.09);
        assert!((swing.position() - before).norm() < 1e-6);
        assert!((swing.remaining() - 0.6).abs() < 1e-5);
    }

    #[test]
    fn a_retime_lands_no_sooner_than_the_soonest() {
        let mut swing = swing(0.3);
        swing.advance(DT, 1.0);
        swing.retime(0.0, 0.09);
        assert!((swing.remaining() - 0.09).abs() < 1e-5);
        swing.retime(0.5, 0.09);
        swing.retime(0.05, 0.09);
        assert!((swing.remaining() - 0.09).abs() < 1e-5);
    }
}
