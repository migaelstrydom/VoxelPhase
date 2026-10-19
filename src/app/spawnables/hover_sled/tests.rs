//! The sled under real physics: it rests at its ride height and sleeps,
//! tips and recovers, dips under a load and carries it, shrugs off a blast,
//! and glides over a plank deck's gaps.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::def::{build, HoverSledDef};
use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::{ConvexSolid, FlatQuadGeometry, SolidsGeometry};
use crate::physics::{
    ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry,
};

/// A game frame, and the substeps it takes.
const FRAME: f32 = 1.0 / 60.0;
const SUBSTEPS: u32 = 4;

fn world() -> PhysicsWorld {
    PhysicsWorld::new(PhysicsConfig::default())
}

fn frame(world: &mut PhysicsWorld, ground: &dyn StaticGeometry) {
    let dt = FRAME / SUBSTEPS as f32;
    world.update_contacts(dt, SUBSTEPS, ground, &[], &mut DebugLines::default());
    for _ in 0..SUBSTEPS {
        world.substep(dt, ground, &[]);
    }
}

fn run(world: &mut PhysicsWorld, ground: &dyn StaticGeometry, seconds: f32) {
    for _ in 0..(seconds / FRAME).round() as usize {
        frame(world, ground);
    }
}

/// A sled over flat ground at its ride height, woken as the spawnable does.
fn sled_on_flat(world: &mut PhysicsWorld) -> RigidBodyHandle {
    let centre = Point3::new(0.0, HoverSledDef::ride_height(), 0.0);
    let (hull, _) = build(world, centre, UnitQuaternion::identity());
    world.wake_body(hull);
    hull
}

/// How far the hull is tipped from level, in degrees.
fn tilt(world: &PhysicsWorld, hull: RigidBodyHandle) -> f32 {
    let up = world.body(hull).unwrap().rotation() * Vector3::y();
    up.y.clamp(-1.0, 1.0).acos().to_degrees()
}

fn height(world: &PhysicsWorld, hull: RigidBodyHandle) -> f32 {
    world.body(hull).unwrap().position().y
}

#[test]
fn it_rests_at_its_ride_height_and_sleeps() {
    let ground = FlatQuadGeometry::new(20.0);
    let mut world = world();
    let hull = sled_on_flat(&mut world);
    run(&mut world, &ground, 3.0);
    let settled = height(&world, hull);
    println!(
        "ride height {settled:.4} (authored {:.4})",
        HoverSledDef::ride_height()
    );
    assert!((settled - HoverSledDef::ride_height()).abs() < 0.005);
    assert!(tilt(&world, hull) < 0.2);
    assert!(
        world.is_sleeping(hull),
        "a sled at rest on flat ground sleeps"
    );
}

#[test]
fn pushed_down_at_a_corner_it_tips_and_recovers() {
    let ground = FlatQuadGeometry::new(20.0);
    let mut world = world();
    let hull = sled_on_flat(&mut world);
    run(&mut world, &ground, 1.0);

    // A shove down on a front corner, as a body landing there would give.
    let corner = Vector3::new(0.6, 0.0, 0.33);
    let shove = Vector3::new(0.0, -60.0, 0.0);
    world.apply_impulse(hull, shove);
    world.apply_angular_impulse(hull, corner.cross(&shove));
    let mut most = 0.0f32;
    for _ in 0..30 {
        frame(&mut world, &ground);
        most = most.max(tilt(&world, hull));
    }
    run(&mut world, &ground, 3.0);
    let after = tilt(&world, hull);
    println!("tipped {most:.1}°, back to {after:.2}°");
    assert!(most > 3.0, "it tips: {most}°");
    assert!(after < 0.3, "and recovers: {after}°");
}

#[test]
fn a_body_landing_on_it_dips_it_and_is_carried() {
    let ground = FlatQuadGeometry::new(20.0);
    let mut world = world();
    let hull = sled_on_flat(&mut world);
    run(&mut world, &ground, 1.0);
    let unloaded = height(&world, hull);

    // About the player's weight, dropped half a metre onto the deck.
    let deck = unloaded + 0.115;
    let load =
        world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, deck + 0.7, 0.0)));
    let _ = world.attach_collider(
        load,
        ColliderDesc::box_shape(Vector3::new(0.2, 0.2, 0.2))
            .density(56.0 / 0.064)
            .friction(0.8),
    );
    world.wake_body(load);
    let mut lowest = unloaded;
    for _ in 0..90 {
        frame(&mut world, &ground);
        lowest = lowest.min(height(&world, hull));
    }
    run(&mut world, &ground, 3.0);
    let loaded = height(&world, hull);
    let on_deck = world.body(load).unwrap().position().y - height(&world, hull);
    println!(
        "dipped {:.3} m, settled {:.3} m lower",
        unloaded - lowest,
        unloaded - loaded
    );
    // The landing dips it well past where the load leaves it.
    assert!(unloaded - loaded > 0.005, "the load presses it down");
    assert!(
        unloaded - lowest > 2.0 * (unloaded - loaded),
        "the landing dips it"
    );
    assert!(
        (on_deck - (0.115 + 0.2)).abs() < 0.02,
        "it carries the load: {on_deck}"
    );
}

#[test]
fn a_blast_throws_it_and_it_lands_and_settles() {
    let ground = FlatQuadGeometry::new(40.0);
    let mut world = world();
    let hull = sled_on_flat(&mut world);
    run(&mut world, &ground, 1.0);

    // A grenade going off beside and under it: up, away, and spun.
    let mass = world.body(hull).unwrap().mass();
    world.apply_impulse(hull, Vector3::new(3.0, 4.0, 0.0) * mass);
    world.apply_angular_impulse(hull, Vector3::new(0.0, 0.0, -8.0));
    let mut highest = 0.0f32;
    for _ in 0..(6.0 / FRAME) as usize {
        frame(&mut world, &ground);
        highest = highest.max(height(&world, hull));
        let body = world.body(hull).unwrap();
        assert!(body.position().coords.iter().all(|v| v.is_finite()));
    }
    let after = tilt(&world, hull);
    println!("thrown to {highest:.2} m, settled at {after:.1}°");
    assert!(highest > HoverSledDef::ride_height() + 0.5, "it is thrown");
    assert!(
        after < 1.0 || (after - 180.0).abs() < 1.0,
        "and comes to rest: {after}°"
    );
}

#[test]
fn it_glides_over_a_plank_decks_gaps() {
    // A deck 0.56 m up, as the Traversal Garden's bridge lane, of planks 0.76
    // m deep with gaps of 0.2 to 0.6 m between them.
    let deck = 0.56;
    let mut solids = vec![ConvexSolid::block(
        Point3::new(-12.0, 0.0, -2.0),
        Point3::new(0.0, deck, 2.0),
    )];
    let mut x = 0.4;
    for gap in [0.6, 0.3, 0.5, 0.2, 0.45, 0.6, 0.35] {
        solids.push(ConvexSolid::block(
            Point3::new(x, 0.0, -1.2),
            Point3::new(x + 0.76, deck, 1.2),
        ));
        x += 0.76 + gap;
    }
    solids.push(ConvexSolid::block(
        Point3::new(x, 0.0, -2.0),
        Point3::new(x + 12.0, deck, 2.0),
    ));
    let ground = SolidsGeometry::new(40.0, &solids);

    let mut world = world();
    let centre = Point3::new(-4.0, deck + HoverSledDef::ride_height(), 0.0);
    let (hull, _) = build(&mut world, centre, UnitQuaternion::identity());
    world.wake_body(hull);
    run(&mut world, &ground, 1.0);
    let cruise = 4.5;
    let mut lowest = f32::MAX;
    let mut most_tilt = 0.0f32;
    let mut frames = 0;
    while world.body(hull).unwrap().position().x < x + 2.0 {
        frames += 1;
        assert!(frames < 1200, "it got across");
        // A rider's lean, holding cruise.
        let velocity = world.body(hull).unwrap().linear_velocity();
        let mass = world.body(hull).unwrap().mass();
        let push = (cruise - velocity.x).clamp(-5.0 * FRAME, 5.0 * FRAME);
        world.apply_impulse(hull, Vector3::x() * push * mass);
        frame(&mut world, &ground);
        lowest = lowest.min(height(&world, hull) - deck);
        most_tilt = most_tilt.max(tilt(&world, hull));
        assert!(height(&world, hull) > deck, "it never falls into a gap");
        if world.body(hull).unwrap().position().x > -1.0 {
            assert!(
                world.body(hull).unwrap().linear_velocity().x > 0.5,
                "it never stops"
            );
        }
    }
    println!("over the gaps: lowest {lowest:.3} m above the deck, most tilt {most_tilt:.1}°");
    assert!(most_tilt < 25.0);
}
