//! Per-substep external force callback.
//!
//! External systems (buoyancy, wind, etc.) implement `SubstepForceProvider`
//! so the physics engine can recompute position-dependent forces each substep
//! instead of using stale frame-start values.

use generational_arena::Arena;
use nalgebra::Vector3;

use super::body::RigidBody;
use super::collider::Collider;
use super::handle::RigidBodyHandle;

/// Context passed to force providers each substep, giving read access to
/// body and collider state needed to compute forces.
pub struct ForceContext<'a> {
    pub bodies: &'a Arena<RigidBody>,
    pub colliders: &'a Arena<Collider>,
    pub gravity: Vector3<f32>,
    pub gravity_magnitude: f32,
}

/// Drag from the medium a body moves through: it relaxes the body's velocity
/// and spin towards the medium's own, exactly however stiff the drag is
/// against the body's mass and inertia, so water carries a light body at its
/// own speed and spins it no faster than it moves past it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MediumDrag {
    /// Linear drag per unit velocity relative to `velocity`, N·s/m.
    pub linear: f32,
    /// Angular drag per unit angular velocity relative to `spin`,
    /// N·m·s/rad.
    pub angular: f32,
    /// Velocity of the medium, m/s: a current or a falling stream.
    pub velocity: Vector3<f32>,
    /// Spin the medium drives the body towards, rad/s: zero in still
    /// water; where a stream strikes a body off its centre, the spin at
    /// which the struck side moves with the water.
    pub spin: Vector3<f32>,
}

impl MediumDrag {
    /// No medium.
    pub const NONE: Self = Self {
        linear: 0.0,
        angular: 0.0,
        velocity: nalgebra::Vector3::new(0.0, 0.0, 0.0),
        spin: nalgebra::Vector3::new(0.0, 0.0, 0.0),
    };

    /// Whether this medium drags nothing.
    pub fn is_none(&self) -> bool {
        self.linear == 0.0 && self.angular == 0.0
    }

    /// The one medium that drags a body as `self` and `other` do together.
    ///
    /// Two linear drags pulling towards different velocities sum to one drag
    /// as stiff as both, pulling towards their stiffness-weighted mean
    /// velocity: `k1(v1 - v) + k2(v2 - v) = (k1 + k2)(v̄ - v)`. Exact, so a
    /// body wading out of water into a bush is dragged by both at once.
    pub fn combined(self, other: Self) -> Self {
        // Exactly the other medium when one is none, so a body only one
        // medium drags (the air, or one provider, never both) is dragged bit
        // for bit as that medium says.
        if self.is_none() {
            return other;
        }
        if other.is_none() {
            return self;
        }
        let linear = self.linear + other.linear;
        let angular = self.angular + other.angular;
        let velocity = if linear > 0.0 {
            (self.velocity * self.linear + other.velocity * other.linear) / linear
        } else {
            Vector3::zeros()
        };
        let spin = if angular > 0.0 {
            (self.spin * self.angular + other.spin * other.angular) / angular
        } else {
            Vector3::zeros()
        };
        Self {
            linear,
            angular,
            velocity,
            spin,
        }
    }
}

/// Mutable target for a single body's force output.
#[derive(Debug, Clone, Copy)]
pub struct ForceOutput {
    pub force: Vector3<f32>,
    pub torque: Vector3<f32>,
    pub drag: MediumDrag,
}

impl ForceOutput {
    pub fn zero() -> Self {
        Self {
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
            drag: MediumDrag::NONE,
        }
    }
}

/// External force provider called each substep before force integration.
///
/// Several providers may affect one body (water and foliage both drag a body
/// wading out of one into the other); their outputs accumulate.
///
/// Implementations recompute position-dependent forces (buoyancy, wind fields,
/// etc.) from the body's current position, ensuring forces stay accurate as
/// bodies move within a frame.
pub trait SubstepForceProvider {
    /// Return the set of body handles that this provider affects.
    /// Called once per substep; only these bodies will have `compute_force` called.
    fn affected_bodies(&self) -> &[RigidBodyHandle];

    /// Compute force, torque, and drag for a single body.
    /// Called for each handle returned by `affected_bodies()`.
    fn compute_force(&self, handle: RigidBodyHandle, ctx: &ForceContext) -> ForceOutput;
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{
        Atmosphere, ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, SequentialStepper,
        Stepper,
    };

    /// A provider asking the same of one body every substep.
    struct Steady {
        body: [RigidBodyHandle; 1],
        output: ForceOutput,
    }

    impl SubstepForceProvider for Steady {
        fn affected_bodies(&self) -> &[RigidBodyHandle] {
            &self.body
        }

        fn compute_force(&self, _: RigidBodyHandle, _: &ForceContext) -> ForceOutput {
            self.output
        }
    }

    /// A ball's velocity after a second of `providers` and `atmosphere`,
    /// with no gravity.
    fn velocity_after(
        atmosphere: Atmosphere,
        density: f32,
        outputs: &[ForceOutput],
    ) -> Vector3<f32> {
        let mut config = PhysicsConfig {
            gravity: Vector3::zeros(),
            atmosphere,
            ..PhysicsConfig::default()
        };
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let ball =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 10.0, 0.0)));
        world.attach_collider(ball, ColliderDesc::sphere(0.2).density(density));
        world.wake_body(ball);
        let providers: Vec<Steady> = outputs
            .iter()
            .map(|&output| Steady {
                body: [ball],
                output,
            })
            .collect();
        let providers: Vec<&dyn SubstepForceProvider> = providers
            .iter()
            .map(|p| p as &dyn SubstepForceProvider)
            .collect();
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        for _ in 0..60 {
            stepper.step(
                &mut world,
                1.0 / 60.0,
                &EmptyGeometry,
                &[],
                &providers,
                &mut DebugLines::default(),
            );
        }
        world.body(ball).unwrap().linear_velocity()
    }

    fn push(force: Vector3<f32>) -> ForceOutput {
        ForceOutput {
            force,
            ..ForceOutput::zero()
        }
    }

    /// Two providers pushing one body push it with their sum: neither
    /// overwrites the other.
    #[test]
    fn two_providers_push_a_body_together() {
        let mass = 1000.0 * 4.0 / 3.0 * std::f32::consts::PI * 0.008;
        let one = velocity_after(Atmosphere::VACUUM, 1000.0, &[push(Vector3::x() * 10.0)]);
        let both = velocity_after(
            Atmosphere::VACUUM,
            1000.0,
            &[push(Vector3::x() * 10.0), push(Vector3::z() * 5.0)],
        );
        assert!((one.x - 10.0 / mass).abs() < 0.01 * one.x, "{one:?}");
        assert!(
            (both.x - one.x).abs() < 1e-4 && (both.z - 0.5 * one.x).abs() < 0.01 * one.x,
            "{both:?}"
        );
    }

    /// A current carrying a light ball through the air carries it a little
    /// slower than through vacuum: the air's drag and the current's combine.
    #[test]
    fn the_air_and_a_provider_drag_a_body_together() {
        let current = ForceOutput {
            drag: drag(5.0, Vector3::x() * 5.0),
            ..ForceOutput::zero()
        };
        let vacuum = velocity_after(Atmosphere::VACUUM, 20.0, &[current]);
        let air = velocity_after(Atmosphere::AIR, 20.0, &[current]);
        assert!(
            (vacuum.x - 5.0).abs() < 0.05,
            "in vacuum it reached {:.3}",
            vacuum.x
        );
        assert!(
            air.x < vacuum.x - 0.05 && air.x > 0.0,
            "in air it reached {:.3}",
            air.x
        );
    }

    fn drag(linear: f32, velocity: Vector3<f32>) -> MediumDrag {
        MediumDrag {
            linear,
            angular: 0.0,
            velocity,
            spin: Vector3::zeros(),
        }
    }

    /// Two media pulling a body pull it as one: as stiff as both, towards
    /// their stiffness-weighted velocity, so the force each would exert on a
    /// body at any velocity sums to what the combined medium exerts.
    #[test]
    fn two_media_drag_as_their_sum() {
        let a = drag(3.0, Vector3::new(2.0, 0.0, 0.0));
        let b = drag(1.0, Vector3::new(0.0, 0.0, -4.0));
        let both = a.combined(b);
        let v = Vector3::new(0.5, 1.0, 0.25);
        let separate = (a.velocity - v) * a.linear + (b.velocity - v) * b.linear;
        let combined = (both.velocity - v) * both.linear;
        assert!((separate - combined).magnitude() < 1e-5);
    }

    /// No medium combined with a medium is that medium.
    #[test]
    fn no_medium_changes_nothing() {
        let a = drag(2.0, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(MediumDrag::NONE.combined(a), a);
        assert_eq!(a.combined(MediumDrag::NONE), a);
    }
}
