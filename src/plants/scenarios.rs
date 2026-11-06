//! Plant scenarios: whole trees stepped for seconds on flat ground. A tree
//! standing still, one let go by the ground, blasts near and far from a
//! forked one, and a branch broken loose.

use nalgebra::{Isometry3, Point3, Vector3};

use super::bodies::{Joinery, PlantPieceBody};
use super::cypress::CypressSpec;
use super::lollipop::LollipopSpec;
use super::plant::Plant;
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    ColliderDesc, ConstraintHandle, PhysicsImpulse, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    SequentialStepper, Stepper,
};

const FRAME_DT: f32 = 1.0 / 60.0;
const PHYSICS_DT: f32 = 1.0 / 240.0;

struct Grove {
    world: PhysicsWorld,
    plant: Plant,
    joinery: Joinery,
    tree: PlantPieceBody,
    rooting: ConstraintHandle,
    ground: FlatQuadGeometry,
    stepper: SequentialStepper,
}

impl Grove {
    fn of(plant: Plant) -> Self {
        Self::standing_at(plant, 0.0)
    }

    /// A tree with its frame `height` above the ground, welded there.
    fn standing_at(plant: Plant, height: f32) -> Self {
        let mut world = PhysicsWorld::default();
        let joinery = Joinery::default();
        let all: Vec<usize> = (0..plant.limbs.limbs.len()).collect();
        let root = Isometry3::translation(0.0, height, 0.0);
        let tree = joinery.build(&mut world, &plant, &all, root);
        let rooting = joinery.root(&mut world, &plant, &tree);
        world.wake_body(tree.body);
        Self {
            world,
            plant,
            joinery,
            tree,
            rooting,
            ground: FlatQuadGeometry::new(200.0),
            stepper: SequentialStepper::new(PHYSICS_DT, 12),
        }
    }

    fn forked() -> Self {
        Self::of(
            LollipopSpec {
                crowns: 3,
                ..LollipopSpec::default()
            }
            .grow(4),
        )
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

    /// Where the `m`th crown's frame is in the world, on `piece`.
    fn crown_on(&self, piece: &PlantPieceBody, m: usize) -> Point3<f32> {
        let pose = piece.plant_pose(&self.world).expect("a piece");
        Point3::from((pose * self.plant.masses[m].pose).translation.vector)
    }

    /// The blast `blast` would give the branch holding crown `m`, alone.
    fn kick(&self, m: usize, blast: &PhysicsImpulse) -> f32 {
        let branch = self.plant.masses[m].limb;
        self.joinery
            .blast_on_branch(&self.world, &self.plant, &self.tree, branch, blast)
            .map_or(0.0, |kick| kick.magnitude())
    }

    /// A blast `distance` out from crown `m`, away from the trunk.
    fn blast_beside(&self, m: usize, distance: f32) -> PhysicsImpulse {
        let crown = self.crown_on(&self.tree, m);
        let outward = Vector3::new(crown.x, 0.0, crown.z).normalize();
        Explosion::new(crown + outward * distance).physics_impulse()
    }
}

/// A forked tree left alone stands where it grew, its weld holding.
#[test]
fn a_tree_stands_where_it_grew() {
    let mut grove = Grove::forked();
    let start: Vec<Point3<f32>> = (0..grove.plant.masses.len())
        .map(|m| grove.crown_on(&grove.tree, m))
        .collect();
    grove.run(10.0);
    for (m, start) in start.iter().enumerate() {
        let moved = (grove.crown_on(&grove.tree, m) - start).magnitude();
        assert!(moved < 0.02, "a crown sagged {moved:.3} m");
    }
}

/// Let go by the ground, a tree falls over and comes to rest on its crown
/// and roots, not sunk into the ground to its trunk.
#[test]
fn a_tree_let_go_falls_and_rests_on_its_crown() {
    for plant in [
        LollipopSpec::default().grow(2),
        CypressSpec::default().grow(3),
    ] {
        // The ground it grew in is gone: its roots hang just clear of what
        // is left.
        let mut grove = Grove::standing_at(plant, 0.4);
        let body = grove.tree.body;
        grove.world.remove_constraint(grove.rooting);
        grove.world.set_ignores_static(body, false);
        // A nudge, as the blast that took its ground would give it.
        grove
            .world
            .apply_impulse(body, Vector3::new(60.0, 0.0, 0.0));
        grove.run(12.0);
        let crown = grove.crown_on(&grove.tree, 0);
        // Lying on its leaves, the crown's middle is at least about half its
        // widest lobe's radius up; sunk to its trunk, it is a few
        // centimetres.
        let lowest_core = 0.5
            * grove.plant.masses[0]
                .shape
                .lobes
                .iter()
                .map(|lobe| lobe.least_radius())
                .fold(0.0, f32::max);
        let length = grove.tree.plant_pose(&grove.world).unwrap().rotation * Vector3::y();
        println!(
            "fallen: crown at {crown:?}, trunk {:.0}° from upright",
            length.y.abs().acos().to_degrees()
        );
        assert!(length.y.abs() < 0.5, "it is still standing");
        assert!(crown.y > lowest_core, "its crown sank to {:.2} m", crown.y);
        assert!(grove.world.is_sleeping(body), "it never came to rest");
    }
}

/// What a blast gives a side branch's crown falls off with distance across
/// the branch's strength: within 2 m it breaks, from 3 m it holds.
#[test]
fn a_branch_breaks_within_two_metres_of_a_blast() {
    let grove = Grove::forked();
    let strength = grove.joinery.branch_strength;
    for distance in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0] {
        let kick = grove.kick(1, &grove.blast_beside(1, distance));
        println!("blast {distance} m off: {kick:.1} N·s against {strength}");
        if distance <= 2.0 {
            assert!(kick > strength, "{distance} m off it held");
        }
        if distance >= 3.0 {
            assert!(kick < strength, "{distance} m off it broke");
        }
    }
}

/// A branch broken off, as by a blast too small to throw it, drops from its
/// fork: the tree it overlapped there pushes it out without flinging it,
/// and it lands beside the tree, which still stands.
#[test]
fn a_broken_branch_drops_from_its_fork() {
    let mut grove = Grove::forked();
    let branch = grove.plant.masses[1].limb;
    let broken = grove
        .joinery
        .break_off(&mut grove.world, &grove.plant, &mut grove.tree, branch)
        .expect("a branch to break");
    let trunk_at = grove.at(grove.tree.body);
    // Sideways and upwards, before it lands: falling adds to neither.
    let mut fastest: f32 = 0.0;
    for _ in 0..15 {
        grove.run(FRAME_DT);
        let v = grove.world.body(broken.body).unwrap().linear_velocity();
        fastest = fastest.max(Vector3::new(v.x, v.y.max(0.0), v.z).magnitude());
    }
    grove.run(8.0);
    let landed = grove.at(broken.body);
    println!("fastest {fastest:.2} m/s across in its first quarter second; landed at {landed:?}");
    assert!(fastest < 1.0, "flung at {fastest:.2} m/s");
    assert!(
        landed.y > 0.0 && landed.y < 1.0,
        "came to rest at {landed:?}"
    );
    assert!(
        grove.world.is_sleeping(broken.body),
        "it never came to rest"
    );
    let moved = (grove.at(grove.tree.body) - trunk_at).magnitude();
    assert!(moved < 0.05, "the tree moved {moved:.2} m");
}

/// A grenade thrown at a tree's trunk strikes it and comes back, as off any
/// body: trees are solid to grenades.
#[test]
fn a_grenade_strikes_a_trunk() {
    let mut grove = Grove::of(LollipopSpec::default().grow(2));
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
