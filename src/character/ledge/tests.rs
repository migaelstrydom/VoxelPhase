//! The edge test on boxes: walls through the catch band, narrow lips, a roof
//! over the top, a free slab, tilted tops, hands at two heights, and an edge
//! carried by its body.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::physics::bench_harness::geometry::{ConvexSolid, EmptyGeometry, SolidsGeometry};
use crate::physics::{
    ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry,
};
use crate::traversal::WorldCaster;

use super::{edge_at_hands, Edge, LedgeConfig, Reach};

/// The body's centre: its feet at 0.5 m.
const CENTRE: Point3<f32> = Point3::new(0.0, 1.0, 0.0);
/// Where the faces in front of the body stand, along +x: 5 cm off its capsule.
const FRONT: f32 = 0.3;

fn reach() -> Reach {
    Reach {
        centre: CENTRE,
        heading: Vector3::x(),
        radius: 0.25,
        half_height: 0.5,
    }
}

fn world() -> PhysicsWorld {
    PhysicsWorld::new(PhysicsConfig::default())
}

/// A static box from `min` to `max`, turned by `rotation` about its centre.
fn solid(world: &mut PhysicsWorld, min: Point3<f32>, max: Point3<f32>) -> RigidBodyHandle {
    turned_solid(world, min, max, UnitQuaternion::identity())
}

fn turned_solid(
    world: &mut PhysicsWorld,
    min: Point3<f32>,
    max: Point3<f32>,
    rotation: UnitQuaternion<f32>,
) -> RigidBodyHandle {
    let body = world.create_body(
        RigidBodyDesc::static_body()
            .position(nalgebra::center(&min, &max))
            .rotation(rotation),
    );
    world.attach_collider(body, ColliderDesc::box_shape((max - min) / 2.0));
    body
}

/// A wall in front of the body, its top `lip` metres above the body's centre.
fn wall(world: &mut PhysicsWorld, lip: f32) -> RigidBodyHandle {
    solid(
        world,
        Point3::new(FRONT, 0.0, -1.0),
        Point3::new(FRONT + 1.0, CENTRE.y + lip, 1.0),
    )
}

fn edge_in(world: &PhysicsWorld, scenery: &dyn StaticGeometry) -> Option<Edge> {
    let caster = WorldCaster {
        world,
        scenery,
        exclude: &[],
        carriers: &[],
    };
    edge_at_hands(&caster, &reach(), &LedgeConfig::default())
}

fn edge(world: &PhysicsWorld) -> Option<Edge> {
    edge_in(world, &EmptyGeometry)
}

fn assert_near(actual: f32, expected: f32, what: &str) {
    assert!(
        (actual - expected).abs() < 0.02,
        "{what}: {actual} against {expected}"
    );
}

#[test]
fn a_wall_top_in_the_catch_band_is_held_and_climbed() {
    for lip in [0.25, 0.4, 0.6] {
        let mut world = world();
        wall(&mut world, lip);
        let edge = edge(&world).unwrap_or_else(|| panic!("no edge at {lip} m"));
        let held = edge.in_world(&world).unwrap();
        for hand in held.hands {
            assert_near(hand.y, CENTRE.y + lip, "hand height");
            assert_near(hand.x, FRONT + 0.04, "hand behind the lip");
        }
        assert_near(held.hands[0].z.abs(), 0.15, "hand spacing");
        assert_near(held.outward.x, -1.0, "outward");
        let stand = held.stand.expect("climbable");
        assert_near(stand.x, FRONT + 0.35, "stand point along");
        assert_near(stand.y, CENTRE.y + lip + 0.5, "standing centre");
    }
}

#[test]
fn a_top_outside_the_catch_band_is_no_edge() {
    for lip in [0.1, 0.75] {
        let mut world = world();
        wall(&mut world, lip);
        assert!(edge(&world).is_none(), "an edge at {lip} m");
    }
}

#[test]
fn a_wall_beyond_the_hands_is_no_edge() {
    let mut world = world();
    solid(
        &mut world,
        Point3::new(0.6, 0.0, -1.0),
        Point3::new(1.6, CENTRE.y + 0.4, 1.0),
    );
    assert!(edge(&world).is_none());
}

#[test]
fn a_wall_behind_the_body_is_no_edge() {
    let mut world = world();
    solid(
        &mut world,
        Point3::new(-1.3, 0.0, -1.0),
        Point3::new(-FRONT, CENTRE.y + 0.4, 1.0),
    );
    assert!(edge(&world).is_none());
}

/// A lip `depth` deep in front of a wall rising well above it: a finger hold
/// at most, and no room to stand.
fn narrow_lip(depth: f32) -> Option<Edge> {
    let mut world = world();
    let lip = CENTRE.y + 0.4;
    solid(
        &mut world,
        Point3::new(FRONT, 0.0, -1.0),
        Point3::new(FRONT + depth, lip, 1.0),
    );
    solid(
        &mut world,
        Point3::new(FRONT + depth, 0.0, -1.0),
        Point3::new(FRONT + 1.0, lip + 1.5, 1.0),
    );
    edge(&world)
}

#[test]
fn a_finger_lip_is_held_but_not_climbed() {
    let edge = narrow_lip(0.06).expect("a 6 cm lip holds the fingers");
    assert!(!edge.climbable());
}

#[test]
fn a_lip_too_shallow_for_the_fingers_is_no_edge() {
    assert!(narrow_lip(0.02).is_none());
}

#[test]
fn a_top_under_a_low_roof_is_held_but_not_climbed() {
    let mut world = world();
    let lip = 0.4;
    wall(&mut world, lip);
    let top = CENTRE.y + lip;
    solid(
        &mut world,
        Point3::new(FRONT + 0.15, top + 0.5, -1.0),
        Point3::new(FRONT + 1.0, top + 0.7, 1.0),
    );
    let edge = edge(&world).expect("the lip is held under the roof");
    assert!(!edge.climbable());
}

#[test]
fn a_slab_with_nothing_under_its_lip_is_held() {
    let mut world = world();
    let top = CENTRE.y + 0.4;
    solid(
        &mut world,
        Point3::new(FRONT, top - 0.15, -1.0),
        Point3::new(FRONT + 1.0, top, 1.0),
    );
    let edge = edge(&world).expect("a slab's free edge is held");
    assert!(edge.climbable());
}

/// A block whose top rises away from the body at `degrees`, its near top
/// corner 0.4 m above the body's centre.
fn tilted_top(degrees: f32) -> Option<Edge> {
    let mut world = world();
    let angle = degrees.to_radians();
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
    let corner = rotation * Vector3::new(-1.0, 1.0, 0.0);
    let centre = Point3::new(FRONT, CENTRE.y + 0.4, 0.0) - corner;
    turned_solid(
        &mut world,
        centre - Vector3::new(1.0, 1.0, 1.0),
        centre + Vector3::new(1.0, 1.0, 1.0),
        rotation,
    );
    edge(&world)
}

#[test]
fn a_top_tilted_past_the_grip_is_no_edge() {
    assert!(tilted_top(30.0).is_some(), "a 30° top is held");
    assert!(tilted_top(50.0).is_none(), "a 50° top is held");
}

/// Two terrain walls side by side, one hand on each, their tops `step`
/// apart.
fn stepped_lip(step: f32) -> Option<Edge> {
    let lip = CENTRE.y + 0.4;
    let scenery = SolidsGeometry::new(
        10.0,
        &[
            ConvexSolid::block(
                Point3::new(FRONT, 0.0, -1.0),
                Point3::new(FRONT + 1.0, lip, 0.0),
            ),
            ConvexSolid::block(
                Point3::new(FRONT, 0.0, 0.0),
                Point3::new(FRONT + 1.0, lip + step, 1.0),
            ),
        ],
    );
    edge_in(&world(), &scenery)
}

#[test]
fn hands_at_two_heights_hold_only_a_small_step() {
    assert!(stepped_lip(0.02).is_some(), "a 2 cm step is held");
    assert!(stepped_lip(0.06).is_none(), "a 6 cm step is held");
    assert!(stepped_lip(0.0).is_some(), "a level lip is held");
}

#[test]
fn a_terrain_wall_is_held_like_a_body() {
    let lip = CENTRE.y + 0.4;
    let scenery = SolidsGeometry::new(
        10.0,
        &[ConvexSolid::block(
            Point3::new(FRONT, 0.0, -1.0),
            Point3::new(FRONT + 1.0, lip, 1.0),
        )],
    );
    let edge = edge_in(&world(), &scenery).expect("the terrain's lip is held");
    assert_eq!(edge.body, None);
    assert!(edge.climbable());
    assert_near(edge.hands[0].y, lip, "hand height");
}

#[test]
fn an_edge_goes_where_its_body_takes_it() {
    let mut world = world();
    let crate_body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
        FRONT + 0.5,
        CENTRE.y - 0.1,
        0.0,
    )));
    world.attach_collider(
        crate_body,
        ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 1.0)),
    );
    let edge = edge(&world).expect("the crate's lip is held");
    assert_eq!(edge.body, Some(crate_body));

    let moved = Vector3::new(0.0, -0.3, 2.0);
    let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.5);
    let held = edge.in_world(&world).unwrap();
    let body = world.body_mut(crate_body).unwrap();
    let pivot = body.position();
    body.set_position(pivot + moved);
    body.set_rotation(turn);
    let carried = edge.in_world(&world).unwrap();
    for (before, after) in held.hands.iter().zip(carried.hands) {
        let expected = pivot + moved + turn * (before - pivot);
        assert!(
            (after - expected).norm() < 1.0e-4,
            "{after} against {expected}"
        );
    }
    assert!((carried.outward.into_inner() - turn * held.outward.into_inner()).norm() < 1.0e-4);
}
