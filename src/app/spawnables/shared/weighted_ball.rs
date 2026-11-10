//! A ball that floats and comes to rest one way up: a giant fruit with its
//! stalk in the air, an orb with its finial on top.
//!
//! ```text
//!   sphere collider ── bulk: a solid ball less a hollow set above its middle
//!        │                   └▶ the mass sits low, so the top turns up
//!        └─ re-centred on that mass; the model follows the collider
//! ```
//!
//! Its collider is the plain sphere, so whatever its model adds beyond the
//! sphere (a stalk, a finial) touches nothing; being weighted is what keeps
//! those clear of the ground.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::finish::ColliderSurface;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::model::Model;
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, Volume,
};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;

/// The hollow's radius, and how far above the ball's centre it is set, as
/// shares of the ball's radius. It has to stay inside the ball.
const HOLLOW_RADIUS: f32 = 0.7;
const HOLLOW_LIFT: f32 = 0.25;

/// What settles its rocking once it is upright, per second.
const ANGULAR_DAMPING: f32 = 0.6;
const LINEAR_DAMPING: f32 = 0.1;

/// A weighted ball's body.
#[derive(Clone, Copy, Debug)]
pub struct WeightedBall {
    /// Radius of the sphere it collides as, m.
    pub radius: f32,
    /// Its grip and bounce, and the density of what is not hollow.
    pub surface: PhysicalSurface,
}

impl WeightedBall {
    /// Its bulk: solid at the surface's density but for the hollow.
    fn bulk(&self) -> BulkShape {
        let ball = |radius: f32| ColliderShape::Sphere { radius };
        BulkShape::hollow(
            Volume::centred(ball(self.radius)),
            Volume::at(
                ball(self.radius * HOLLOW_RADIUS),
                Vector3::y() * self.radius * HOLLOW_LIFT,
            ),
            self.surface.density,
        )
    }

    /// Make its body in `physics` with the ball's centre at `centre`.
    /// Returns the body and where its origin, the centre of its mass, lies
    /// from the ball's centre, in the body's frame.
    fn create_body(
        &self,
        physics: &mut PhysicsWorld,
        centre: Point3<f32>,
    ) -> (RigidBodyHandle, Vector3<f32>) {
        let body = physics.create_body(
            RigidBodyDesc::dynamic()
                .position(centre)
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING),
        );
        physics.attach_collider(
            body,
            ColliderDesc::sphere(self.radius)
                .with_physical_surface(self.surface)
                .bulk(self.bulk()),
        );
        let moved = physics.recenter_on_colliders(body);
        (body, moved)
    }

    /// Spawn it with the ball's centre at `centre`, drawn as `model`, which
    /// is built round the ball's centre with +Y the way up it rests.
    pub fn spawn(&self, world: &mut World, centre: Point3<f32>, mut model: Model) -> Entity {
        let (body, moved) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            self.create_body(&mut physics.world, centre)
        };
        for vertex in model
            .parts
            .iter_mut()
            .flat_map(|part| part.primitives.iter_mut())
            .flat_map(|primitive| primitive.vertices.iter_mut())
        {
            vertex.pos -= moved;
        }

        world
            .create_entity()
            .with(Position(centre.coords + moved))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(model)))
            .with(Renderable)
            .build()
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use nalgebra::UnitQuaternion;

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::PhysicsConfig;

    const BALL: WeightedBall = WeightedBall {
        radius: 0.5,
        surface: PhysicalSurface {
            restitution: 0.25,
            friction: 0.6,
            density: 600.0,
        },
    };

    #[test]
    fn its_mass_sits_below_its_centre() {
        let mut world = PhysicsWorld::default();
        let (_, moved) = BALL.create_body(&mut world, Point3::origin());
        assert!(
            moved.y < -0.1 * BALL.radius,
            "the mass is centred {moved:?} from the ball's centre"
        );
        assert!(moved.x.abs() < 1e-5 && moved.z.abs() < 1e-5);
    }

    #[test]
    fn it_is_light_enough_to_float_mostly_out_of_the_water() {
        let mass = BALL.bulk().mass().expect("the ball declares its mass");
        let volume = ColliderShape::Sphere {
            radius: BALL.radius,
        }
        .compute_mass(1.0);
        let density = mass / volume;
        assert!((250.0..500.0).contains(&density), "{density} kg/m³ overall");
    }

    #[test]
    fn laid_on_its_side_it_comes_to_rest_upright() {
        const DT: f32 = 1.0 / 240.0;
        // Sleep would freeze it wherever it first slowed, at the top of a
        // rock.
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let (body, _) = BALL.create_body(&mut world, Point3::new(0.0, BALL.radius, 0.0));
        let on_side = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        world
            .body_mut(body)
            .expect("the ball")
            .set_rotation(on_side);
        world.wake_body(body);

        let floor = FlatQuadGeometry::new(20.0);
        let mut debug = DebugLines::default();
        for _ in 0..(10.0 / DT) as usize {
            world.update_contacts(DT, 1, &floor, &[], &mut debug);
            world.substep(DT, &floor, &[]);
        }

        let up = world.body(body).expect("the ball").rotation() * Vector3::y();
        assert!(up.y > 0.97, "its top points along {up:?}");
    }
}
