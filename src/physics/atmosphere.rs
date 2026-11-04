//! The air every body moves through: the world's ambient medium, as gravity
//! is its ambient force.
//!
//! ```text
//!   PhysicsConfig::atmosphere ── density, wind, felt_within
//!        │
//!        ▼  every substep, before any SubstepForceProvider
//!   each dynamic body the air can slow ── ½ρ·ΣCd·A (its bulk's DragArea)
//!        │                                 relaxed towards the wind
//!        ▼
//!   MediumDrag, which providers' media (water, leaves) then combine with
//! ```
//!
//! Air drags a body by its bulk, the shape fluids see
//! ([`super::RigidBodyDesc::bulk`]), not its colliders. It matters only to
//! bodies light for their size: a cardboard box falls at about 7 m/s, and a
//! clump of leaves floats down. A body too heavy for the air to slow within
//! [`Atmosphere::felt_within`] is passed by, which keeps a grenade's flight
//! the parabola its aim predicts and costs nothing for rocks and characters.
//!
//! The air drags a body's whole bulk, under water too: at a thousandth of
//! water's density it adds a tenth of a percent to the water's drag there,
//! and is left in rather than split by depth. It is not a rule for stacking
//! media.

use nalgebra::Vector3;

use super::body::RigidBody;
use super::force_provider::MediumDrag;

/// The air bodies move through.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Atmosphere {
    /// Density, kg/m³.
    pub density: f32,
    /// Velocity of the air, m/s: what it carries a light body towards.
    pub wind: Vector3<f32>,
    /// Distance, m, within which the air must slow a body's speed by a factor
    /// of e (`m / (½ρ·ΣCd·A)`, the same at any speed) for it to drag the body
    /// at all. Past it the drag is a few percent over a throw, and the body is
    /// treated as moving through vacuum.
    pub felt_within: f32,
}

impl Atmosphere {
    /// Air at sea level, still.
    pub const AIR: Self = Self {
        density: 1.2,
        wind: Vector3::new(0.0, 0.0, 0.0),
        felt_within: 60.0,
    };

    /// No air: nothing is dragged.
    pub const VACUUM: Self = Self {
        density: 0.0,
        wind: Vector3::new(0.0, 0.0, 0.0),
        felt_within: 0.0,
    };

    /// Whether the air drags `body` at all: a dynamic body light enough for
    /// its size that the air slows it within [`Self::felt_within`].
    pub fn drags(&self, body: &RigidBody) -> bool {
        let area = body.drag_area().linear;
        body.is_dynamic() && area > 0.0 && body.mass() < self.felt_within * self.half_rho() * area
    }

    /// The air's drag on `body` moving as it is now. Call only for a body the
    /// air [`drags`](Self::drags).
    pub fn drag(&self, body: &RigidBody) -> MediumDrag {
        let area = body.drag_area();
        let speed = (body.linear_velocity() - self.wind).magnitude();
        let spin = body.angular_velocity().magnitude();
        MediumDrag {
            linear: self.half_rho() * area.linear * speed,
            angular: self.half_rho() * area.angular * spin,
            velocity: self.wind,
            spin: Vector3::zeros(),
        }
    }

    fn half_rho(&self) -> f32 {
        0.5 * self.density
    }
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self::AIR
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{
        BulkShape, ColliderDesc, ColliderShape, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
        SequentialStepper, Stepper, Volume,
    };

    /// A dropped body's speed after `seconds` of falling in `world`.
    fn fall(world: &mut PhysicsWorld, desc: ColliderDesc, seconds: f32) -> f32 {
        fall_as(world, RigidBodyDesc::dynamic(), desc, seconds)
    }

    /// [`fall`], for a body described by `body`.
    fn fall_as(
        world: &mut PhysicsWorld,
        body: RigidBodyDesc,
        desc: ColliderDesc,
        seconds: f32,
    ) -> f32 {
        let body = world.create_body(body.position(Point3::new(0.0, 500.0, 0.0)));
        world.attach_collider(body, desc);
        world.wake_body(body);
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        for _ in 0..(seconds * 60.0) as usize {
            stepper.step(
                world,
                1.0 / 60.0,
                &EmptyGeometry,
                &[],
                &[],
                &mut DebugLines::default(),
            );
        }
        world.body(body).unwrap().linear_velocity().magnitude()
    }

    /// A cardboard box (40 cm, half a kilo) falls at the speed the drag law
    /// gives: `√(2mg / ρ·Cd·A)`, about 7 m/s.
    #[test]
    fn a_cardboard_box_falls_at_its_terminal_speed() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let half: f32 = 0.2;
        let mass = 0.5;
        let density = mass / (2.0 * half).powi(3);
        let box_ = ColliderDesc::box_shape(Vector3::repeat(half)).density(density);
        let speed = fall(&mut world, box_, 4.0);
        let area = 1.05 * 2.0 * 3.0 * half * half;
        let terminal = (2.0 * mass * 9.81 / (1.2 * area)).sqrt();
        assert!(
            (speed - terminal).abs() < 0.05 * terminal,
            "fell at {speed} m/s, terminal {terminal}"
        );
    }

    /// A stone the air would take more than `felt_within` to slow falls as
    /// it would through vacuum.
    #[test]
    fn a_stone_falls_through_air_as_through_vacuum() {
        let stone = || ColliderDesc::sphere(0.1).density(2500.0);
        let mut air = PhysicsWorld::new(PhysicsConfig::default());
        let mut vacuum = PhysicsWorld::new(PhysicsConfig {
            atmosphere: Atmosphere::VACUUM,
            ..PhysicsConfig::default()
        });
        assert_eq!(
            fall(&mut air, stone(), 2.0),
            fall(&mut vacuum, stone(), 2.0)
        );
    }

    /// A ball whose bulk declares a porous body's drag coefficient falls at
    /// that coefficient's terminal speed, not a smooth ball's.
    #[test]
    fn a_declared_drag_coefficient_sets_the_terminal_speed() {
        let radius: f32 = 0.2;
        let mass = 0.3;
        let density = mass / (4.0 / 3.0 * std::f32::consts::PI * radius.powi(3));
        let ball = || ColliderDesc::sphere(radius).density(density);
        let porous = RigidBodyDesc::dynamic().bulk(
            BulkShape::displacing(vec![Volume::centred(ColliderShape::Sphere { radius })])
                .dragging_as(1.0),
        );
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let speed = fall_as(&mut world, porous, ball(), 4.0);
        let area = std::f32::consts::PI * radius * radius;
        let terminal = (2.0 * mass * 9.81 / (1.2 * area)).sqrt();
        let smooth = fall(&mut world, ball(), 4.0);
        assert!(
            (speed - terminal).abs() < 0.05 * terminal,
            "fell at {speed} m/s, terminal {terminal}, a smooth ball {smooth}"
        );
    }
}
