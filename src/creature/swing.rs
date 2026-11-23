use nalgebra::{Point3, Vector2, Vector3};

use crate::physics::{
    CastObstacles, PhysicsImpulse, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry,
};

/// What a blow sweeps, and how hard it lands: the striking part (a beak, a
/// head, jaws) carried as a sphere from where it is drawn back to where it
/// is thrown out, in the vertical plane the striker faces.
///
/// ```text
///        from ●───────────▶● to        side view, the striker facing right
///             │  radius ◯
///   centre ✚
/// ```
///
/// The first body the sphere touches is struck where it touched, along the
/// sweep. A blow is aimed where the striker faces, so one at something
/// behind it misses, and anything in the way (a wall, a crate, another
/// creature) takes it instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Swing {
    /// Where the striking part starts, m: `x` ahead of the striker's centre
    /// of mass, `y` above it.
    pub from: Vector2<f32>,
    /// Where it ends, thrown out, the same way.
    pub to: Vector2<f32>,
    /// The striking part's radius, m: what it touches with.
    pub radius: f32,
    /// The impulse it drives into what it touches, N·s.
    pub impulse: f32,
}

impl Swing {
    /// The sweep's start and end, in world space, for a striker centred at
    /// `centre` and facing `facing` (level, unit length).
    pub fn path(&self, centre: Point3<f32>, facing: Vector3<f32>) -> (Point3<f32>, Point3<f32>) {
        let at = |offset: Vector2<f32>| centre + facing * offset.x + Vector3::y() * offset.y;
        (at(self.from), at(self.to))
    }

    /// The strike it lands for `striker`, centred at `centre` and facing
    /// `facing`: on the first body its sweep touches, where it touched,
    /// along the sweep. `None` if the sweep touches nothing, or only
    /// `scenery` or a static body first; it passes through the striker's
    /// own body and touches what that body meets.
    pub fn land(
        &self,
        world: &PhysicsWorld,
        scenery: &dyn StaticGeometry,
        striker: RigidBodyHandle,
        centre: Point3<f32>,
        facing: Vector3<f32>,
    ) -> Option<PhysicsImpulse> {
        let (from, to) = self.path(centre, facing);
        let sweep = to - from;
        let length = sweep.norm();
        let direction = sweep.try_normalize(1e-4)?;
        let touch = world.cast_sphere(
            scenery,
            &SphereCast {
                origin: from,
                direction,
                length,
                radius: self.radius,
                exclude: &[striker],
                solid_bodies: true,
                obstacles: CastObstacles::Everything,
                filter: world.filter_of(striker),
            },
        )?;
        let body = touch
            .body
            .filter(|&body| world.body(body).is_some_and(|body| body.is_dynamic()))?;
        Some(PhysicsImpulse::Strike {
            body,
            impulse: direction * self.impulse,
            point: touch.point,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    /// A peck: from just above the striker's centre to 0.8 m ahead of it.
    const PECK: Swing = Swing {
        from: Vector2::new(0.0, 0.3),
        to: Vector2::new(0.8, 0.3),
        radius: 0.2,
        impulse: 300.0,
    };

    fn ball(world: &mut PhysicsWorld, at: Point3<f32>, radius: f32) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(at));
        world.attach_collider(body, ColliderDesc::sphere(radius).density(1000.0));
        body
    }

    /// A world with a striker at the origin and a ball where `at` says.
    fn yard(at: Point3<f32>) -> (PhysicsWorld, RigidBodyHandle, RigidBodyHandle) {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let striker = ball(&mut world, Point3::origin(), 0.3);
        let target = ball(&mut world, at, 0.3);
        (world, striker, target)
    }

    fn land(
        world: &PhysicsWorld,
        striker: RigidBodyHandle,
        facing: Vector3<f32>,
    ) -> Option<PhysicsImpulse> {
        PECK.land(world, &EmptyGeometry, striker, Point3::origin(), facing)
    }

    #[test]
    fn the_sweep_runs_ahead_of_the_striker_where_it_faces() {
        let (from, to) = PECK.path(Point3::new(1.0, 2.0, 3.0), Vector3::z());
        assert_eq!(from, Point3::new(1.0, 2.3, 3.0));
        assert_eq!(to, Point3::new(1.0, 2.3, 3.8));
    }

    /// A body in front of the striker is struck where the sweep touched
    /// it, along the sweep.
    #[test]
    fn a_blow_strikes_what_is_in_front_where_it_touches() {
        let (world, striker, target) = yard(Point3::new(0.9, 0.0, 0.0));
        let Some(PhysicsImpulse::Strike {
            body,
            impulse,
            point,
        }) = land(&world, striker, Vector3::x())
        else {
            panic!("the blow missed");
        };
        assert_eq!(body, target);
        assert!(
            (impulse - Vector3::x() * 300.0).norm() < 1e-3,
            "{impulse:?}"
        );
        assert!(point.x < 0.9, "struck on its near side: {point:?}");
    }

    /// A body behind the striker, or beside it, or past the end of the
    /// sweep, is not struck.
    #[test]
    fn a_blow_misses_what_is_not_in_front() {
        for at in [
            Point3::new(-0.9, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.9),
            Point3::new(1.8, 0.0, 0.0),
        ] {
            let (world, striker, _) = yard(at);
            assert!(land(&world, striker, Vector3::x()).is_none(), "{at:?}");
        }
    }

    /// The first body in the way takes the blow.
    #[test]
    fn a_body_in_the_way_takes_the_blow() {
        let (mut world, striker, _) = yard(Point3::new(0.9, 0.0, 0.0));
        let between = ball(&mut world, Point3::new(0.35, 0.3, 0.0), 0.1);
        let strike = land(&world, striker, Vector3::x());
        assert!(
            matches!(strike, Some(PhysicsImpulse::Strike { body, .. }) if body == between),
            "{strike:?}"
        );
    }
}
