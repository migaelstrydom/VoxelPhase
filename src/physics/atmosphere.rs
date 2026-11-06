//! The air every body moves through: the world's ambient medium, as gravity
//! is its ambient force.
//!
//! ```text
//!   PhysicsConfig::atmosphere ── density
//!        │
//!        ▼  every substep, before any SubstepForceProvider
//!   each awake dynamic body ── ½ρ·Cd·A·|v| over its bulk (DragArea::exposed),
//!        │                     towards rest
//!        │
//!        ▼
//!   MediumDrag, which providers' media (water, leaves) then combine with
//! ```
//!
//! Air drags a body by its bulk, the shape fluids see
//! ([`super::ColliderDesc::bulk`]), not its colliders. Every dynamic body
//! feels it, but it matters only to bodies light for their size: a cardboard
//! box falls flat at about 7 m/s, and a clump of leaves floats down, while the air
//! takes a grenade a couple of kilometres to slow by a factor of e, a percent
//! over a throw.
//!
//! The air drags a body's whole bulk, under water too: at a thousandth of
//! water's density it adds a tenth of a percent to the water's drag there,
//! and is left in rather than split by depth. It is not a rule for stacking
//! media.

use super::body::RigidBody;
use super::bulk::EnvelopePart;
use super::force_provider::MediumDrag;

/// The air bodies move through. It is still: it drags bodies towards rest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Atmosphere {
    /// Density, kg/m³.
    pub density: f32,
}

impl Atmosphere {
    /// Air at sea level.
    pub const AIR: Self = Self { density: 1.2 };

    /// No air: nothing is dragged.
    pub const VACUUM: Self = Self { density: 0.0 };

    /// The air's drag on `body` moving as it is now, over its envelope, by
    /// the drag the world has measured for it.
    pub(crate) fn drag<'a>(
        &self,
        body: &RigidBody,
        envelope: impl IntoIterator<Item = EnvelopePart<'a>>,
    ) -> MediumDrag {
        if self.density <= 0.0 {
            return MediumDrag::NONE;
        }
        let flow = body.rotation().inverse() * body.linear_velocity();
        body.drag_area()
            .exposed(envelope.into_iter().map(|part| (part, 1.0)), flow)
            .drag(
                self.density,
                flow.norm(),
                body.angular_velocity().magnitude(),
            )
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
    use crate::physics::fluid_drag::ANGULAR_COEFFICIENT;
    use crate::physics::{
        BulkShape, ColliderDesc, ColliderShape, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
        SequentialStepper, Stepper, Volume,
    };

    /// A dropped body's speed after `seconds` of falling in `world`.
    fn fall(world: &mut PhysicsWorld, desc: ColliderDesc, seconds: f32) -> f32 {
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 500.0, 0.0)));
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

    /// A cardboard box (40 cm, half a kilo) dropped flat falls at the speed
    /// the drag law gives over the face it shows the air: `√(2mg / ρ·Cd·A)`,
    /// about 7 m/s.
    #[test]
    fn a_cardboard_box_falls_at_its_terminal_speed() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let half: f32 = 0.2;
        let mass = 0.5;
        let density = mass / (2.0 * half).powi(3);
        let box_ = ColliderDesc::box_shape(Vector3::repeat(half)).density(density);
        let speed = fall(&mut world, box_, 4.0);
        let area = 1.05 * (2.0 * half).powi(2);
        let terminal = (2.0 * mass * 9.81 / (1.2 * area)).sqrt();
        assert!(
            (speed - terminal).abs() < 0.05 * terminal,
            "fell at {speed} m/s, terminal {terminal}"
        );
    }

    /// A stone barely notices the air: two seconds' fall is slowed by about
    /// a percent (`k·g²·t³/3` for `k = ½ρ·Cd·A/m`).
    #[test]
    fn a_stone_barely_notices_the_air() {
        let stone = || ColliderDesc::sphere(0.1).density(2500.0);
        let mut air = PhysicsWorld::new(PhysicsConfig::default());
        let mut vacuum = PhysicsWorld::new(PhysicsConfig {
            atmosphere: Atmosphere::VACUUM,
            ..PhysicsConfig::default()
        });
        let in_air = fall(&mut air, stone(), 2.0);
        let in_vacuum = fall(&mut vacuum, stone(), 2.0);
        assert!(
            in_air < 0.995 * in_vacuum && in_air > 0.98 * in_vacuum,
            "fell at {in_air} m/s in air, {in_vacuum} in vacuum"
        );
    }

    /// A plank spinning flat about its stiffest axis slows as the spin drag
    /// law gives for that axis's own inertia: `ω = ω₀ / (1 + c·ω₀·t / I)`,
    /// for `c = ½ρ·Cω·A·R³`.
    #[test]
    fn a_spinning_plank_slows_at_its_own_inertia() {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity: Vector3::zeros(),
            ..PhysicsConfig::default()
        });
        let half = Vector3::new(1.0, 0.2, 0.3);
        let density = 500.0;
        let body = world.create_body(RigidBodyDesc::dynamic().angular_damping(0.0));
        world.attach_collider(body, ColliderDesc::box_shape(half).density(density));
        let spin = 3.0;
        world.set_body_velocity(body, Vector3::zeros(), Vector3::z() * spin);
        let seconds = 5.0;
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        for _ in 0..(seconds * 60.0) as usize {
            stepper.step(
                &mut world,
                1.0 / 60.0,
                &EmptyGeometry,
                &[],
                &[],
                &mut DebugLines::default(),
            );
        }
        let mass = density * 8.0 * half.x * half.y * half.z;
        let inertia = mass * 4.0 * (half.x * half.x + half.y * half.y) / 12.0;
        let mean_area = 2.0 * (half.x * half.y + half.y * half.z + half.x * half.z);
        let c = 0.5 * 1.2 * ANGULAR_COEFFICIENT * mean_area * half.norm().powi(3);
        let expected = spin / (1.0 + c * spin * seconds / inertia);
        let got = world.body(body).unwrap().angular_velocity().z;
        assert!(
            (got - expected).abs() < 0.01 * expected,
            "spun at {got} rad/s, the law gives {expected}"
        );
    }

    /// A ball whose bulk declares a porous clump's drag coefficient falls at
    /// that coefficient's terminal speed, not a smooth ball's.
    #[test]
    fn a_declared_drag_coefficient_sets_the_terminal_speed() {
        let radius: f32 = 0.2;
        let mass = 0.3;
        let density = mass / (4.0 / 3.0 * std::f32::consts::PI * radius.powi(3));
        let ball = || ColliderDesc::sphere(radius).density(density);
        let porous = ball().bulk(
            BulkShape::displacing(vec![Volume::centred(ColliderShape::Sphere { radius })])
                .dragging_as(1.0),
        );
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let speed = fall(&mut world, porous, 4.0);
        let area = std::f32::consts::PI * radius * radius;
        let terminal = (2.0 * mass * 9.81 / (1.2 * area)).sqrt();
        let smooth = fall(&mut world, ball(), 4.0);
        assert!(
            (speed - terminal).abs() < 0.05 * terminal,
            "fell at {speed} m/s, terminal {terminal}, a smooth ball {smooth}"
        );
    }
}
