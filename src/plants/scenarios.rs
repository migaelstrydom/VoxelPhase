//! Plant scenarios: whole trees stepped for seconds on flat ground. A tree
//! standing still, one let go by the ground, and a grenade beside a forked
//! one.

use nalgebra::{Isometry3, Point3, Vector3};

use super::bodies::{Joinery, PlantBodies};
use super::lollipop::LollipopSpec;
use super::plant::Plant;
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    ColliderDesc, ConstraintHandle, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    SequentialStepper, Stepper,
};

const FRAME_DT: f32 = 1.0 / 60.0;
const PHYSICS_DT: f32 = 1.0 / 240.0;

struct Grove {
    world: PhysicsWorld,
    plant: Plant,
    bodies: PlantBodies,
    ground: FlatQuadGeometry,
    stepper: SequentialStepper,
}

impl Grove {
    fn of(spec: LollipopSpec, seed: u32) -> Self {
        Self::standing_at(spec, seed, 0.0)
    }

    /// A tree grown with its frame `height` above the ground.
    fn standing_at(spec: LollipopSpec, seed: u32, height: f32) -> Self {
        let mut world = PhysicsWorld::default();
        let plant = spec.grow(seed);
        let root = Isometry3::translation(0.0, height, 0.0);
        let bodies = Joinery::default().build(&mut world, &plant, root);
        for &body in &bodies.limbs {
            world.wake_body(body);
        }
        Self {
            world,
            plant,
            bodies,
            ground: FlatQuadGeometry::new(200.0),
            stepper: SequentialStepper::new(PHYSICS_DT, 12),
        }
    }

    fn run(&mut self, seconds: f32) {
        let mut lines = DebugLines::default();
        for _ in 0..(seconds / FRAME_DT).round() as usize {
            self.stepper.step(
                &mut self.world,
                FRAME_DT,
                &self.ground,
                &[],
                &[],
                &mut lines,
            );
            lines.clear();
        }
    }

    fn at(&self, body: RigidBodyHandle) -> Point3<f32> {
        self.world.body(body).expect("a tree's body").position()
    }

    fn holds(&self, weld: ConstraintHandle) -> bool {
        self.world.constraint(weld).is_some_and(|c| c.active)
    }

    /// Where the `m`th crown's frame is in the world.
    fn crown(&self, m: usize) -> Point3<f32> {
        let limb = self.bodies.limbs[self.plant.masses[m].limb];
        let body = self.world.body(limb).expect("a limb");
        let pose = Isometry3::from_parts(body.position().coords.into(), body.rotation());
        Point3::from((pose * self.bodies.mass_offsets[m]).translation.vector)
    }
}

/// A forked tree left alone stands where it grew: its crowns do not sag on
/// their welds and no branch breaks under its own crown.
#[test]
fn a_tree_stands_where_it_grew() {
    let mut grove = Grove::of(
        LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        },
        4,
    );
    let crowns = grove.plant.masses.len();
    let start: Vec<Point3<f32>> = (0..crowns).map(|m| grove.crown(m)).collect();
    grove.run(10.0);
    for (m, start) in start.iter().enumerate() {
        let moved = (grove.crown(m) - start).magnitude();
        assert!(moved < 0.02, "a crown sagged {moved:.3} m");
    }
    for weld in grove.bodies.branch_welds.iter().flatten() {
        assert!(grove.holds(*weld), "a branch broke under its own crown");
    }
}

/// Let go by the ground, a tree falls over and comes to rest on its crown's
/// core, not sunk into the ground to its trunk.
#[test]
fn a_tree_let_go_falls_and_rests_on_its_crown() {
    // The ground it grew in is gone: its foot stands just clear of what is
    // left.
    let mut grove = Grove::standing_at(LollipopSpec::default(), 2, 0.35);
    let trunk = grove.bodies.limbs[0];
    grove.world.remove_constraint(grove.bodies.rooting);
    grove.world.set_ignores_static(trunk, false);
    // A nudge, as the blast that took its ground would give it.
    grove
        .world
        .apply_impulse(trunk, Vector3::new(60.0, 0.0, 0.0));
    grove.run(12.0);
    let crown = grove.crown(0);
    let lowest_core = grove.plant.masses[0].shape.bounds().1.y * 0.6;
    let length = grove.world.body(trunk).unwrap().rotation() * Vector3::y();
    println!(
        "fallen: crown at {crown:?}, trunk {:.0}° from upright",
        length.y.abs().acos().to_degrees()
    );
    assert!(length.y.abs() < 0.5, "it is still standing");
    assert!(crown.y > lowest_core, "its crown sank to {:.2} m", crown.y);
    assert!(grove.world.is_sleeping(trunk), "it never came to rest");
}

/// A grenade beside a forked tree's side crown breaks that branch off; the
/// trunk, welded into the ground, stays.
#[test]
fn a_blast_breaks_a_branch_off_but_not_the_trunk() {
    let mut grove = Grove::of(
        LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        },
        4,
    );
    let side = grove.crown(1);
    let trunk = grove.bodies.limbs[0];
    let trunk_start = grove.at(trunk);
    let outward = Vector3::new(side.x, 0.0, side.z).normalize();
    let blast = Explosion::new(side + outward * 1.2).physics_impulse();
    for body in grove.bodies.limbs.clone() {
        if let Some(kick) = grove.world.impulse_on(body, &blast) {
            grove.world.apply_impulse(body, kick);
        }
    }
    grove.run(3.0);
    let snapped: Vec<bool> = grove
        .bodies
        .branch_welds
        .iter()
        .flatten()
        .map(|&weld| !grove.holds(weld))
        .collect();
    println!("branches snapped: {snapped:?}");
    assert!(snapped[0], "the branch beside the blast held");
    assert!(grove.holds(grove.bodies.rooting));
    let moved = (grove.at(trunk) - trunk_start).magnitude();
    assert!(moved < 0.05, "the trunk moved {moved:.2} m");
}

/// A grenade a few metres off shakes a forked tree but breaks nothing.
#[test]
fn a_distant_blast_breaks_no_branch() {
    let mut grove = Grove::of(
        LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        },
        4,
    );
    let side = grove.crown(1);
    let outward = Vector3::new(side.x, 0.0, side.z).normalize();
    let blast = Explosion::new(side + outward * 4.0).physics_impulse();
    for body in grove.bodies.limbs.clone() {
        if let Some(kick) = grove.world.impulse_on(body, &blast) {
            grove.world.apply_impulse(body, kick);
        }
    }
    grove.run(3.0);
    for weld in grove.bodies.branch_welds.iter().flatten() {
        assert!(grove.holds(*weld), "a branch broke 4 m from the blast");
    }
}

/// A grenade thrown at a tree's trunk strikes it and comes back, as off any
/// body: trees are solid to grenades.
#[test]
fn a_grenade_strikes_a_trunk() {
    let mut grove = Grove::of(LollipopSpec::default(), 2);
    let trunk = grove.plant.limbs.limbs[0];
    let at = trunk.base + (trunk.tip - trunk.base) * 0.3;
    let grenade = grove.world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(at.x - 2.0, at.y, at.z))
            .linear_velocity(Vector3::x() * 12.0)
            .gravity_scale(0.0),
    );
    grove
        .world
        .attach_collider(grenade, ColliderDesc::sphere(0.2).density(2000.0));
    grove.world.wake_body(grenade);
    grove.run(0.5);
    let x = grove.at(grenade).x;
    println!(
        "grenade at x {x:.2} after 0.5 s, the trunk at x {:.2}",
        at.x
    );
    assert!(x < at.x, "it went through the trunk to x {x:.2}");
}
