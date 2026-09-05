//! Ground that does not hold still.
//!
//! A platform on its way between two points, a lift, a crate falling out from
//! under the character: the surface is the same shape throughout, it is simply
//! somewhere else each frame. That is exactly a rigid offset, so a moving
//! surface is any `Ground` plus the displacement it has accumulated.
//!
//! ```text
//!   SupportMotion ─velocity(t)─▶ what the character is carried by
//!                 └─offset(t)──▶ Carried { ground, offset } ─▶ Ground
//! ```
//!
//! Keeping it an offset rather than a time-varying height field is what lets
//! the whole existing catalogue move without changing: a slope on a lift is a
//! `Slope` in a `Carried`, and the probes, the body and the rendered checker
//! all follow from the one displacement. The checker moving is the point — a
//! foot stuck to the world instead of to the platform reads instantly against
//! it.

use nalgebra::Vector3;

use super::ground::Ground;

/// How the ground moves during a scenario.
#[derive(Clone, Copy, Debug)]
pub enum SupportMotion {
    /// Static terrain. The whole original catalogue.
    Still,
    /// Constant velocity: a platform mid-run, a conveyor, a lift.
    Steady(Vector3<f32>),
    /// Still until `at` seconds, then departing at `velocity` — a crate that
    /// gives way underfoot, a plank kicked out, a fragment blown aside.
    ///
    /// It leaves rather than merely falling, because a surface in free fall
    /// accelerates exactly as the character does: the two would stay in
    /// contact all the way down and nothing about the feet would be tested.
    /// What is worth watching is support vanishing — the feet have to let go
    /// of a floor that is no longer under them.
    GivesWay { at: f32, velocity: Vector3<f32> },
}

impl SupportMotion {
    /// Velocity of the surface at `time`.
    pub fn velocity(&self, time: f32) -> Vector3<f32> {
        match self {
            SupportMotion::Still => Vector3::zeros(),
            SupportMotion::Steady(velocity) => *velocity,
            SupportMotion::GivesWay { at, velocity } => {
                if time >= *at {
                    *velocity
                } else {
                    Vector3::zeros()
                }
            }
        }
    }

    /// Where the surface has got to by `time`, relative to where it started.
    pub fn offset(&self, time: f32) -> Vector3<f32> {
        match self {
            SupportMotion::Still => Vector3::zeros(),
            SupportMotion::Steady(velocity) => velocity * time,
            SupportMotion::GivesWay { at, velocity } => velocity * (time - at).max(0.0),
        }
    }

    /// True for a surface that never goes anywhere, which is worth knowing
    /// because everything about it can then be answered once.
    pub fn is_still(&self) -> bool {
        matches!(self, SupportMotion::Still)
    }
}

/// A surface displaced bodily from where it was authored.
///
/// Every derived answer — normals, probe hits, the rendered mesh — comes from
/// the shifted height, so nothing can disagree about where the ground is this
/// frame.
pub struct Carried<'a> {
    ground: &'a dyn Ground,
    offset: Vector3<f32>,
}

impl<'a> Carried<'a> {
    pub fn new(ground: &'a dyn Ground, offset: Vector3<f32>) -> Self {
        Self { ground, offset }
    }
}

impl Ground for Carried<'_> {
    fn name(&self) -> &str {
        self.ground.name()
    }

    fn height(&self, x: f32, z: f32) -> Option<f32> {
        self.ground
            .height(x - self.offset.x, z - self.offset.z)
            .map(|y| y + self.offset.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::anim_viewer::ground::{Flat, Ledge};

    #[test]
    fn a_steady_surface_is_where_its_velocity_put_it() {
        let motion = SupportMotion::Steady(Vector3::new(2.0, 0.0, 0.0));
        assert_eq!(motion.velocity(3.0), Vector3::new(2.0, 0.0, 0.0));
        assert_eq!(motion.offset(3.0), Vector3::new(6.0, 0.0, 0.0));
    }

    #[test]
    fn a_floor_that_gives_way_holds_still_until_it_does() {
        let motion = SupportMotion::GivesWay {
            at: 1.0,
            velocity: Vector3::new(0.0, -6.0, 0.0),
        };
        assert_eq!(motion.offset(0.5), Vector3::zeros());
        assert_eq!(motion.velocity(0.5), Vector3::zeros());
        assert_eq!(motion.velocity(2.0), Vector3::new(0.0, -6.0, 0.0));
        assert_eq!(motion.offset(2.0), Vector3::new(0.0, -6.0, 0.0));
    }

    #[test]
    fn carrying_a_ledge_carries_its_edge() {
        let ledge = Ledge::new("lip", 4.0);
        let carried = Carried::new(&ledge, Vector3::new(3.0, 0.0, 0.0));
        assert!(ledge.height(5.0, 0.0).is_none());
        assert_eq!(
            carried.height(5.0, 0.0),
            Some(0.0),
            "the edge moved with the platform, so 5 m is now on it"
        );
        assert!(carried.height(7.5, 0.0).is_none());
    }

    #[test]
    fn a_lift_raises_the_floor_it_carries() {
        let flat = Flat::at(0.0);
        let carried = Carried::new(&flat, Vector3::new(0.0, 1.5, 0.0));
        assert_eq!(carried.height(0.0, 0.0), Some(1.5));
        assert!(carried.normal(0.0, 0.0).y > 0.99);
    }
}
