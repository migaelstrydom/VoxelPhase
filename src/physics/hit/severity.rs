//! How bad a hit was, worked out from what it was: none of it reads the
//! solver. The pair is taken to stop at a constant rate over its give, the
//! simplest contact law that orders hits rightly; a stiffer law (Hertz, a
//! flat punch) would change these functions and nothing that calls them.

use std::f32::consts::PI;

use super::event::Hit;
use super::patch::Patch;
use crate::physics::handle::RigidBodyHandle;

/// The least give a hit is judged over, m: a pair that claims none would
/// stop in no time, at no finite force.
const MIN_GIVE: f32 = 1.0e-4;

impl Hit {
    /// The change in velocity, m/s, the hit gave a side of the pair weighing
    /// `mass` kg: `(1 + e) · approach · m_eff / m`. A body landing square on
    /// the ground with no bounce loses exactly its approach.
    pub fn delta_v(&self, mass: f32) -> f32 {
        if !mass.is_finite() || mass <= 0.0 {
            return 0.0;
        }
        (1.0 + self.restitution) * self.approach * self.effective_mass.min(mass) / mass
    }

    /// The impulse that stopped the pair, N·s, the same on either side:
    /// `(1 + e) · approach · m_eff`.
    pub fn impulse(&self) -> f32 {
        (1.0 + self.restitution) * self.approach * self.effective_mass
    }

    /// The impulse the hit drove through `body`'s side, N·s: the pair's
    /// ([`impulse`](Self::impulse)), unless that side was backed, when it
    /// stopped the other side as a wall would. What breaks a block struck
    /// where it lies: it barely moves, and passes the whole blow on to what
    /// it lies on. `None` if the hit was not on `body`.
    pub fn impulse_through(&self, body: RigidBodyHandle) -> Option<f32> {
        let (own, other) = if self.body_b == body {
            (1, 0)
        } else if self.body_a == Some(body) {
            (0, 1)
        } else {
            return None;
        };
        let stopped = match self.side_masses[other] {
            wall if self.backed[own] && wall.is_finite() => wall.max(self.effective_mass),
            _ => self.effective_mass,
        };
        Some((1.0 + self.restitution) * self.approach * stopped)
    }

    /// The greatest force between the pair, N: what stops the effective
    /// mass at a constant rate over the give.
    pub fn peak_force(&self) -> f32 {
        self.effective_mass * self.approach * self.approach / (2.0 * self.give())
    }

    /// How long the pair took to stop, s.
    pub fn duration(&self) -> f32 {
        if self.approach > 0.0 {
            2.0 * self.give() / self.approach
        } else {
            f32::INFINITY
        }
    }

    /// The area the pair pressed on each other over, m²: a face's own; a line
    /// or a point pressed into the other side to the depth of the give, its
    /// contact as wide as `sqrt(bluntness · give)` (Hertz).
    pub fn contact_area(&self) -> f32 {
        let squeeze = self.bluntness * self.give();
        match self.patch {
            Patch::Point => PI * squeeze,
            Patch::Line { length } => length * 2.0 * squeeze.sqrt(),
            Patch::Face { area } => area,
        }
    }

    /// The greatest pressure between the pair, Pa: the peak force over the
    /// contact area. What decides whether a brittle side cracks where it was
    /// struck.
    pub fn peak_pressure(&self) -> f32 {
        self.peak_force() / self.contact_area()
    }

    fn give(&self) -> f32 {
        self.give.max(MIN_GIVE)
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;

    fn hit(patch: Patch, approach: f32, effective_mass: f32, give: f32) -> Hit {
        Hit {
            body_a: None,
            collider_a: None,
            body_b: RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0)),
            collider_b: None,
            point: Point3::origin(),
            normal: Vector3::y(),
            approach,
            restitution: 0.0,
            give,
            bluntness: 0.01,
            effective_mass,
            side_masses: [f32::INFINITY, effective_mass],
            backed: [false; 2],
            patch,
            continues: false,
        }
    }

    /// A hammer on a block: the pair's impulse moves the block if nothing
    /// is behind it; if the ground is, the block stops the hammer as a wall
    /// would and passes the whole blow on.
    #[test]
    fn a_backed_side_takes_the_whole_stop_of_what_struck_it() {
        let block = RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0));
        let hammer = RigidBodyHandle(generational_arena::Index::from_raw_parts(1, 0));
        let free = Hit {
            body_a: Some(hammer),
            side_masses: [400.0, 60.0],
            ..hit(Patch::Point, 5.0, 400.0 * 60.0 / 460.0, 0.01)
        };
        let pair = free.impulse();
        assert_eq!(free.impulse_through(block), Some(pair));
        let lying = Hit {
            backed: [false, true],
            ..free
        };
        assert!((lying.impulse_through(block).unwrap() - 2000.0).abs() < 1e-2);
        assert_eq!(lying.impulse_through(hammer), Some(pair));
        let elsewhere = RigidBodyHandle(generational_arena::Index::from_raw_parts(2, 0));
        assert_eq!(lying.impulse_through(elsewhere), None);
    }

    #[test]
    fn a_square_landing_without_bounce_loses_its_approach() {
        let landing = hit(Patch::Face { area: 0.25 }, 7.0, 56.0, 0.5);
        assert!((landing.delta_v(56.0) - 7.0).abs() < 1e-5);
        let bouncing = Hit {
            restitution: 0.5,
            ..landing
        };
        assert!((bouncing.delta_v(56.0) - 10.5).abs() < 1e-5);
    }

    /// Two equal bodies meeting head on share the blow: each changes speed
    /// by the approach times half.
    #[test]
    fn a_side_feels_the_effective_mass_over_its_own() {
        let head_on = hit(Patch::Point, 4.0, 0.5, 0.002);
        assert!((head_on.delta_v(1.0) - 2.0).abs() < 1e-5);
        assert_eq!(head_on.delta_v(f32::INFINITY), 0.0);
    }

    /// The worked example in docs/IMPACT_METER.md: a 125 kg crate at 5 m/s
    /// onto a pane, gives 1 mm and 3 mm. The same force either way; the
    /// pressure on a 1 cm corner hundreds of times the face's.
    #[test]
    fn a_crate_on_a_pane_presses_far_harder_with_a_corner() {
        let face = hit(Patch::Face { area: 0.25 }, 5.0, 125.0, 0.004);
        let corner = Hit {
            patch: Patch::Point,
            ..face
        };
        assert!((face.peak_force() - 390_625.0).abs() < 1.0);
        assert_eq!(face.peak_force(), corner.peak_force());
        assert!((face.peak_pressure() - 1.5625e6).abs() < 10.0);
        let ratio = corner.peak_pressure() / face.peak_pressure();
        assert!(
            (ratio - 0.25 / (PI * 0.01 * 0.004)).abs() < 1.0,
            "ratio {ratio}"
        );
        assert!((face.duration() - 0.0016).abs() < 1e-7);
    }

    #[test]
    fn an_edge_presses_a_strip_as_wide_as_its_squeeze() {
        let edge = hit(Patch::Line { length: 0.5 }, 5.0, 125.0, 0.004);
        let width = 2.0 * (0.01f32 * 0.004).sqrt();
        assert!((edge.contact_area() - 0.5 * width).abs() < 1e-7);
    }

    #[test]
    fn a_pair_that_claims_no_give_is_still_judged_finitely() {
        let rigid = hit(Patch::Point, 5.0, 1.0, 0.0);
        assert!(rigid.peak_force().is_finite());
        assert!(rigid.peak_pressure().is_finite());
    }
}
