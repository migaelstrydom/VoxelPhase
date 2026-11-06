//! Splitting a body in two: what the new body is ([`BodyTraits`]) and what
//! the split did ([`Split`]). The split itself is
//! [`PhysicsWorld::split_off`](super::PhysicsWorld::split_off).

use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

use super::body::RigidBodyDesc;
use super::collision_filter::CollisionFilter;
use super::handle::RigidBodyHandle;

/// What a body split off another is, beyond where it is and how it moves.
#[derive(Debug, Clone, Copy)]
pub struct BodyTraits {
    /// Linear velocity damping, as [`RigidBodyDesc::linear_damping`].
    pub linear_damping: f32,
    /// Angular velocity damping, as [`RigidBodyDesc::angular_damping`].
    pub angular_damping: f32,
    /// How much of gravity the body feels.
    pub gravity_scale: f32,
    /// Which bodies it meets and which queries see it; its colliders carry
    /// it too.
    pub filter: CollisionFilter,
    /// Whether it passes through static geometry.
    pub ignores_static: bool,
}

impl BodyTraits {
    /// A dynamic body with these traits, placed and moving as given.
    pub(crate) fn body_desc(
        &self,
        position: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        linear_velocity: Vector3<f32>,
        angular_velocity: Vector3<f32>,
    ) -> RigidBodyDesc {
        RigidBodyDesc::dynamic()
            .position(position)
            .rotation(rotation)
            .linear_velocity(linear_velocity)
            .angular_velocity(angular_velocity)
            .linear_damping(self.linear_damping)
            .angular_damping(self.angular_damping)
            .gravity_scale(self.gravity_scale)
            .filter(self.filter)
            .ignores_static(self.ignores_static)
    }
}

/// What a split did: the new body, and how each body's frame moved, for
/// whoever keeps points in those frames (bond anchors).
#[derive(Debug, Clone, Copy)]
pub struct Split {
    /// The body the colliders moved to.
    pub body: RigidBodyHandle,
    /// How far the kept body's origin moved, in its own frame.
    pub kept_moved: Vector3<f32>,
    /// The new body's frame in the old body's frame as it was.
    pub new_frame: Isometry3<f32>,
}

#[cfg(test)]
mod tests {
    use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

    use super::{BodyTraits, Split};
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{
        BulkShape, ColliderDesc, ColliderHandle, ColliderShape, CollisionFilter, ConstraintHandle,
        ConstraintKind, Layers, PhysicsConfig, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
        SequentialStepper, Stepper, Volume,
    };

    const TOLERANCE: f32 = 1e-4;

    /// A world without gravity, so nothing moves but what is set moving.
    fn still_world() -> PhysicsWorld {
        PhysicsWorld::new(PhysicsConfig {
            gravity: Vector3::zeros(),
            ..PhysicsConfig::default()
        })
    }

    /// One 60 Hz frame of the game's stepping.
    fn step(world: &mut PhysicsWorld) {
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        stepper.step(
            world,
            1.0 / 60.0,
            &EmptyGeometry,
            &[],
            &[],
            &mut DebugLines::default(),
        );
    }

    /// Traits unlike the default body's, so a test sees them arrive.
    fn traits() -> BodyTraits {
        BodyTraits {
            linear_damping: 0.2,
            angular_damping: 0.3,
            gravity_scale: 0.5,
            filter: CollisionFilter {
                membership: Layers::layer(3),
                meets: Layers::ALL,
            },
            ignores_static: true,
        }
    }

    /// A turned, moving and spinning body of three unlike colliders, centred
    /// on its mass, and its colliders: `[stays, moves, moves]`.
    struct Compound {
        body: RigidBodyHandle,
        colliders: [ColliderHandle; 3],
    }

    impl Compound {
        fn new(world: &mut PhysicsWorld) -> Self {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(1.0, 2.0, 3.0))
                    .rotation(UnitQuaternion::from_euler_angles(0.4, 0.7, -0.2))
                    .linear_velocity(Vector3::new(1.0, 0.0, 0.5))
                    .angular_velocity(Vector3::new(0.3, 1.2, -0.4)),
            );
            let stays = world
                .attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::new(0.5, 0.3, 0.4))
                        .offset_translation(Vector3::new(-1.0, 0.0, 0.0))
                        .density(500.0),
                )
                .unwrap();
            let moves = world
                .attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::new(0.4, 0.2, 0.6))
                        .offset_translation(Vector3::new(1.5, 0.5, 0.0))
                        .offset_rotation(UnitQuaternion::from_euler_angles(0.0, 0.5, 0.3))
                        .density(800.0),
                )
                .unwrap();
            let also_moves = world
                .attach_collider(
                    body,
                    ColliderDesc::sphere(0.3)
                        .offset_translation(Vector3::new(0.5, 1.0, -0.5))
                        .density(300.0),
                )
                .unwrap();
            world.recenter_on_colliders(body);
            Self {
                body,
                colliders: [stays, moves, also_moves],
            }
        }

        fn moving(&self) -> [ColliderHandle; 2] {
            [self.colliders[1], self.colliders[2]]
        }
    }

    /// The body a collider is on.
    fn owner(world: &PhysicsWorld, collider: ColliderHandle) -> RigidBodyHandle {
        world
            .bodies()
            .iter()
            .find(|(_, body)| body.colliders().contains(&collider))
            .map(|(index, _)| RigidBodyHandle(index))
            .expect("every collider is on a body")
    }

    fn world_transform(world: &PhysicsWorld, collider: ColliderHandle) -> Isometry3<f32> {
        let body = world.body(owner(world, collider)).unwrap();
        world
            .collider(collider)
            .unwrap()
            .world_transform(body.position(), body.rotation())
    }

    fn pose(world: &PhysicsWorld, body: RigidBodyHandle) -> Isometry3<f32> {
        let body = world.body(body).unwrap();
        Isometry3::from_parts(body.position().coords.into(), body.rotation())
    }

    fn assert_isometry_eq(a: &Isometry3<f32>, b: &Isometry3<f32>, what: &str) {
        let moved = (a.translation.vector - b.translation.vector).norm();
        let turned = a.rotation.angle_to(&b.rotation);
        assert!(
            moved < TOLERANCE && turned < TOLERANCE,
            "{what}: moved {moved} m and turned {turned} rad"
        );
    }

    /// Each end of a joint: its body (`None` for the world) and anchor.
    fn ends(
        world: &PhysicsWorld,
        handle: ConstraintHandle,
    ) -> Vec<(Option<RigidBodyHandle>, Vector3<f32>)> {
        match world.constraint(handle).unwrap().kind {
            ConstraintKind::Fixed {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            }
            | ConstraintKind::BallJoint {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            }
            | ConstraintKind::Hinge {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            } => vec![(body_a, local_anchor_a), (Some(body_b), local_anchor_b)],
            _ => unreachable!("only joints are made here"),
        }
    }

    /// Where each end of a joint holds, in the world.
    fn anchors(world: &PhysicsWorld, handle: ConstraintHandle) -> Vec<Point3<f32>> {
        ends(world, handle)
            .into_iter()
            .map(|(body, anchor)| match body {
                Some(body) => pose(world, body) * Point3::from(anchor),
                None => Point3::from(anchor),
            })
            .collect()
    }

    fn bodies_of(world: &PhysicsWorld, handle: ConstraintHandle) -> Vec<RigidBodyHandle> {
        world
            .constraint(handle)
            .unwrap()
            .kind
            .referenced_bodies()
            .into_vec()
    }

    /// Everything a refused split must leave as it was.
    fn fingerprint(world: &PhysicsWorld, constraints: &[ConstraintHandle]) -> String {
        let bodies: Vec<_> = world
            .bodies()
            .iter()
            .map(|(index, body)| {
                let colliders: Vec<_> = body
                    .colliders()
                    .iter()
                    .map(|c| (*c, *world.collider(*c).unwrap().offset()))
                    .collect();
                (
                    index,
                    body.position(),
                    body.linear_velocity(),
                    body.mass(),
                    colliders,
                )
            })
            .collect();
        let constraints: Vec<_> = constraints
            .iter()
            .map(|handle| ends_or_kind(world, *handle))
            .collect();
        format!("{bodies:?} {constraints:?}")
    }

    fn ends_or_kind(world: &PhysicsWorld, handle: ConstraintHandle) -> String {
        match world.constraint(handle).unwrap().kind {
            ConstraintKind::KeepUpright { body, .. } => format!("upright {body:?}"),
            _ => format!("{:?}", ends(world, handle)),
        }
    }

    #[test]
    fn the_moved_colliders_keep_their_handles_on_the_new_body() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let original_filter = *world.collider(compound.colliders[0]).unwrap().filter();

        let split = world
            .split_off(compound.body, &compound.moving(), &[], traits())
            .unwrap();

        assert_eq!(
            world.body(compound.body).unwrap().colliders(),
            &[compound.colliders[0]]
        );
        let new_body = world.body(split.body).unwrap();
        assert_eq!(new_body.colliders().len(), 2);
        for moved in compound.moving() {
            assert!(new_body.colliders().contains(&moved));
            assert_eq!(*world.collider(moved).unwrap().filter(), traits().filter);
        }
        assert_eq!(
            *world.collider(compound.colliders[0]).unwrap().filter(),
            original_filter
        );
        assert!(matches!(
            world.collider(compound.colliders[1]).unwrap().shape(),
            ColliderShape::Box { .. }
        ));
        assert_eq!(*new_body.filter(), traits().filter);
        assert!(new_body.ignores_static());
        assert_eq!(new_body.gravity_scale(), 0.5);
    }

    #[test]
    fn every_collider_stays_where_it_was() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let before: Vec<_> = compound
            .colliders
            .iter()
            .map(|c| world_transform(&world, *c))
            .collect();

        world
            .split_off(compound.body, &compound.moving(), &[], traits())
            .unwrap();

        for (collider, was) in compound.colliders.iter().zip(&before) {
            assert_isometry_eq(&world_transform(&world, *collider), was, "a collider");
        }
    }

    #[test]
    fn every_point_keeps_its_velocity() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let probes = [
            Vector3::zeros(),
            Vector3::new(0.3, -0.2, 0.1),
            Vector3::new(-0.2, 0.4, 0.5),
        ];
        let velocities = |world: &PhysicsWorld| -> Vec<Vector3<f32>> {
            compound
                .colliders
                .iter()
                .flat_map(|c| {
                    let transform = world_transform(world, *c);
                    let body = world.body(owner(world, *c)).unwrap();
                    probes
                        .iter()
                        .map(move |p| body.velocity_at(transform * Point3::from(*p)))
                })
                .collect()
        };
        let before = velocities(&world);

        world
            .split_off(compound.body, &compound.moving(), &[], traits())
            .unwrap();

        for (now, was) in velocities(&world).iter().zip(&before) {
            assert!((now - was).norm() < TOLERANCE, "{was:?} became {now:?}");
        }
    }

    /// Each body after the split weighs, turns and centres as a body built
    /// afresh from its colliders where they stand.
    #[test]
    fn both_bodies_recentre_on_what_they_hold() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let old_pose = pose(&world, compound.body);
        let probe = Vector3::new(0.7, -0.3, 1.1);
        let probe_in_world = old_pose * Point3::from(probe);

        let split = world
            .split_off(compound.body, &compound.moving(), &[], traits())
            .unwrap();

        for body in [compound.body, split.body] {
            let mut fresh = still_world();
            let rebuilt = fresh.create_body(RigidBodyDesc::dynamic());
            for collider in world.body(body).unwrap().colliders() {
                let c = world.collider(*collider).unwrap();
                let ColliderShape::Box { half_extents } = c.shape() else {
                    let ColliderShape::Sphere { radius } = c.shape() else {
                        unreachable!()
                    };
                    let placed = world_transform(&world, *collider);
                    fresh.attach_collider(
                        rebuilt,
                        ColliderDesc::sphere(*radius)
                            .offset_translation(placed.translation.vector)
                            .offset_rotation(placed.rotation)
                            .density(300.0),
                    );
                    continue;
                };
                let placed = world_transform(&world, *collider);
                let density = c.mass() / (8.0 * half_extents.x * half_extents.y * half_extents.z);
                fresh.attach_collider(
                    rebuilt,
                    ColliderDesc::box_shape(*half_extents)
                        .offset_translation(placed.translation.vector)
                        .offset_rotation(placed.rotation)
                        .density(density),
                );
            }
            fresh.recenter_on_colliders(rebuilt);
            let expected = fresh.body(rebuilt).unwrap();
            let actual = world.body(body).unwrap();
            assert!((actual.position() - expected.position()).norm() < TOLERANCE);
            assert!((actual.mass() - expected.mass()).abs() < TOLERANCE * expected.mass());
            let inertia_error = (actual.world_inv_inertia() - expected.world_inv_inertia()).norm();
            assert!(
                inertia_error < 1e-3 * expected.world_inv_inertia().norm(),
                "inertia off by {inertia_error}"
            );
        }

        let Split {
            kept_moved,
            new_frame,
            ..
        } = split;
        let kept = pose(&world, compound.body) * Point3::from(probe - kept_moved);
        let moved = pose(&world, split.body) * (new_frame.inverse() * Point3::from(probe));
        assert!(
            (kept - probe_in_world).norm() < TOLERANCE,
            "kept frame: {kept:?}"
        );
        assert!(
            (moved - probe_in_world).norm() < TOLERANCE,
            "new frame: {moved:?}"
        );
        assert!(kept_moved.norm() > 0.1, "the kept body's centre must move");
    }

    #[test]
    fn listed_constraints_follow_and_the_rest_stay() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let neighbour =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(5.0, 2.0, 3.0)));
        world.attach_collider(neighbour, ColliderDesc::sphere(0.5));
        let rotation = world.body(compound.body).unwrap().rotation();
        let weld_point = world_transform(&world, compound.colliders[1]) * Point3::origin();
        let weld_local = pose(&world, compound.body).inverse() * weld_point;
        let weld = world.create_constraint(ConstraintKind::world_fixed(
            compound.body,
            weld_point,
            weld_local.coords,
            &rotation,
            0.0,
            f32::INFINITY,
        ));
        let ball = world.create_constraint(ConstraintKind::BallJoint {
            body_a: Some(compound.body),
            body_b: neighbour,
            local_anchor_a: Vector3::new(1.0, 0.5, 0.2),
            anchor_velocity: Vector3::zeros(),
            local_anchor_b: Vector3::new(-0.5, 0.0, 0.0),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        let hinge = world.create_constraint(ConstraintKind::world_hinge(
            compound.body,
            Point3::new(0.0, 2.0, 3.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::y_axis(),
            &rotation,
            0.0,
            f32::INFINITY,
        ));
        let before: Vec<_> = [weld, ball, hinge]
            .iter()
            .map(|c| anchors(&world, *c))
            .collect();

        let split = world
            .split_off(compound.body, &compound.moving(), &[weld, ball], traits())
            .unwrap();

        assert_eq!(bodies_of(&world, weld), vec![split.body]);
        assert_eq!(bodies_of(&world, ball), vec![split.body, neighbour]);
        assert_eq!(bodies_of(&world, hinge), vec![compound.body]);
        let ConstraintKind::Fixed { reference, .. } = world.constraint(weld).unwrap().kind else {
            unreachable!()
        };
        assert!(
            reference.angle_to(&rotation) < TOLERANCE,
            "the weld's reference turned"
        );
        for (constraint, was) in [weld, ball, hinge].iter().zip(&before) {
            for (now, was) in anchors(&world, *constraint).iter().zip(was) {
                assert!(
                    (now - was).norm() < TOLERANCE,
                    "an anchor moved from {was:?} to {now:?}"
                );
            }
        }
    }

    #[test]
    fn a_split_it_cannot_make_changes_nothing() {
        let mut world = still_world();
        let compound = Compound::new(&mut world);
        let other =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(9.0, 0.0, 0.0)));
        let stranger = world
            .attach_collider(other, ColliderDesc::sphere(0.5))
            .unwrap();
        let held_elsewhere = world.create_constraint(ConstraintKind::world_ball_joint(
            other,
            Point3::new(9.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            0.0,
            f32::INFINITY,
        ));
        let upright = world.create_constraint(ConstraintKind::KeepUpright {
            body: compound.body,
            target_up: Vector3::y_axis(),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        let weld = world.create_constraint(ConstraintKind::world_fixed(
            compound.body,
            Point3::new(1.0, 2.0, 3.0),
            Vector3::zeros(),
            &UnitQuaternion::identity(),
            0.0,
            f32::INFINITY,
        ));
        let [stays, moves, also_moves] = compound.colliders;
        let refused: [(&str, Vec<ColliderHandle>, Vec<ConstraintHandle>); 7] = [
            ("no colliders", vec![], vec![]),
            ("every collider", vec![stays, moves, also_moves], vec![]),
            ("a collider of another body", vec![moves, stranger], vec![]),
            ("a collider named twice", vec![moves, moves], vec![]),
            (
                "a constraint not on the body",
                vec![moves],
                vec![held_elsewhere],
            ),
            ("a kind that cannot be rebound", vec![moves], vec![upright]),
            ("a constraint named twice", vec![moves], vec![weld, weld]),
        ];
        let watched = [held_elsewhere, upright, weld];
        let before = fingerprint(&world, &watched);

        for (case, colliders, constraints) in refused {
            assert!(
                world
                    .split_off(compound.body, &colliders, &constraints, traits())
                    .is_none(),
                "{case} was accepted"
            );
            assert_eq!(
                fingerprint(&world, &watched),
                before,
                "{case} changed the world"
            );
        }
        assert!(world.split_off(other, &[stranger], &[], traits()).is_none());
    }

    /// Contacts the moved collider made on the old body are not carried to
    /// the new one; those of the collider that stayed are kept.
    #[test]
    fn the_moved_colliders_cached_contacts_are_dropped() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let floor = world.create_body(RigidBodyDesc::static_body());
        world.attach_collider(
            floor,
            ColliderDesc::box_shape(Vector3::new(10.0, 0.5, 10.0)),
        );
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 0.79, 0.0)));
        let stays = world
            .attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(0.3, 0.3, 0.3))
                    .offset_translation(Vector3::new(-1.0, 0.0, 0.0)),
            )
            .unwrap();
        let moves = world
            .attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(0.3, 0.3, 0.3))
                    .offset_translation(Vector3::new(1.0, 0.0, 0.0)),
            )
            .unwrap();
        world.wake_body(body);
        for _ in 0..5 {
            step(&mut world);
        }
        assert!(world.has_cached_manifold(stays) && world.has_cached_manifold(moves));

        world.split_off(body, &[moves], &[], traits()).unwrap();

        assert!(!world.has_cached_manifold(moves));
        assert!(world.has_cached_manifold(stays));
    }

    #[test]
    fn both_bodies_wake() {
        let mut world = still_world();
        let body = world.create_body(RigidBodyDesc::dynamic());
        world.attach_collider(body, ColliderDesc::sphere(0.3));
        let moves = world
            .attach_collider(
                body,
                ColliderDesc::sphere(0.3).offset_translation(Vector3::new(1.0, 0.0, 0.0)),
            )
            .unwrap();
        assert!(
            world.is_sleeping(body),
            "a body placed at rest starts asleep"
        );

        let split = world.split_off(body, &[moves], &[], traits()).unwrap();

        assert!(!world.is_sleeping(body));
        assert!(!world.is_sleeping(split.body));
    }

    #[test]
    fn a_colliders_bulk_goes_with_it() {
        let mut world = still_world();
        let body = world.create_body(RigidBodyDesc::dynamic());
        world.attach_collider(body, ColliderDesc::sphere(0.3));
        let bulk = BulkShape::solid(Volume::centred(ColliderShape::Sphere { radius: 0.6 }), 50.0);
        let declared_mass = bulk.mass().unwrap();
        let moves = world
            .attach_collider(
                body,
                ColliderDesc::sphere(0.3)
                    .offset_translation(Vector3::new(1.0, 0.0, 0.0))
                    .bulk(bulk),
            )
            .unwrap();

        let split = world.split_off(body, &[moves], &[], traits()).unwrap();

        assert!(world.collider(moves).unwrap().bulk().is_some());
        let mass = world.body(split.body).unwrap().mass();
        assert!((mass - declared_mass).abs() < TOLERANCE * declared_mass);
        let envelope: Vec<_> = world.envelope(split.body).collect();
        assert_eq!(envelope.len(), 1);
        assert!(matches!(
            envelope[0].shape,
            ColliderShape::Sphere { radius } if *radius == 0.6
        ));
    }

    /// A plank welded to the world at one end: the welded half split off with
    /// its weld stays up, and the other half, free now, falls.
    #[test]
    fn a_welded_half_split_off_with_its_weld_stays_up() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 5.0, 0.0)));
        let falls = world
            .attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(0.5, 0.1, 0.3))
                    .offset_translation(Vector3::new(-0.5, 0.0, 0.0)),
            )
            .unwrap();
        let held = world
            .attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(0.5, 0.1, 0.3))
                    .offset_translation(Vector3::new(0.5, 0.0, 0.0)),
            )
            .unwrap();
        let weld = world.create_constraint(ConstraintKind::world_fixed(
            body,
            Point3::new(0.5, 5.0, 0.0),
            Vector3::new(0.5, 0.0, 0.0),
            &UnitQuaternion::identity(),
            0.0,
            f32::INFINITY,
        ));
        for _ in 0..10 {
            step(&mut world);
        }
        let held_at = world_transform(&world, held).translation.vector;
        let falls_from = world_transform(&world, falls).translation.vector;

        world.split_off(body, &[held], &[weld], traits()).unwrap();
        for _ in 0..60 {
            step(&mut world);
        }

        let held_moved = (world_transform(&world, held).translation.vector - held_at).norm();
        let fell = falls_from.y - world_transform(&world, falls).translation.y;
        assert!(held_moved < 0.01, "the welded half moved {held_moved} m");
        assert!(fell > 1.0, "the free half fell only {fell} m");
    }
}
