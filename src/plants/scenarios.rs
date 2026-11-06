//! Plant scenarios: whole trees spawned as compounds and stepped for
//! seconds on flat ground, through the break pipeline. A tree standing
//! still, one let go by the ground, blasts near and far from a forked one,
//! a branch broken loose, and landings soft and hard.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use specs::{Entity, Join, World, WorldExt};

use super::components::{GrownPlant, PlantPiece};
use super::cypress::CypressSpec;
use super::drawing::PlantLook;
use super::lollipop::LollipopSpec;
use super::plant::Plant;
use crate::app::breakage;
use crate::app::spawnables::spawn_plant;
use crate::components::{RigidBodyComponent, TerrainAnchored};
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::foliage::Foliage;
use crate::fracture::{Bond, BondLoads, CompoundFracture, ContactLoadTracker};
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    ColliderDesc, PhysicsImpulse, PhysicsImpulseQueue, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, SequentialStepper, Stepper,
};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::time::Time;

const FRAME_DT: f32 = 1.0 / 60.0;
const PHYSICS_DT: f32 = 1.0 / 240.0;
/// The seed every tree's skins grow from.
const SEED: u32 = 4;
const LOOK: PlantLook = PlantLook {
    leaves: MaterialId(0),
    bark: MaterialId(1),
};

struct Grove {
    world: World,
    tree: Entity,
    ground: FlatQuadGeometry,
    stepper: SequentialStepper,
}

impl Grove {
    fn of(plant: Plant) -> Self {
        Self::standing_at(plant, Isometry3::identity())
    }

    /// A tree with its frame at `root`, welded there.
    fn standing_at(plant: Plant, root: Isometry3<f32>) -> Self {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsResource::default());
        let foot = root * Point3::new(0.0, -0.3, 0.0);
        let tree = spawn_plant(&mut world, plant, root, SEED, LOOK, foot);
        let grove = Self {
            world,
            tree,
            ground: FlatQuadGeometry::new(200.0),
            stepper: SequentialStepper::new(PHYSICS_DT, 12),
        };
        let body = grove.body(tree);
        grove.physics_mut().world.wake_body(body);
        grove
    }

    fn forked() -> Self {
        Self::of(forked_plant())
    }

    /// The ground the tree grew in gone: its weld removed, it meets the
    /// ground again.
    fn let_go(&mut self) {
        let welds = self
            .world
            .write_storage::<TerrainAnchored>()
            .remove(self.tree)
            .expect("a tree is anchored")
            .constraints;
        let body = self.body(self.tree);
        let mut physics = self.physics_mut();
        for weld in welds {
            physics.world.remove_constraint(weld);
        }
        physics.world.set_ignores_static(body, false);
    }

    /// One frame: this frame's blasts and the physics step, then the break
    /// pipeline.
    fn step(&mut self) {
        let mut lines = DebugLines::default();
        {
            let mut queue = self.world.write_resource::<PhysicsImpulseQueue>();
            let blasts: Vec<PhysicsImpulse> = queue.drain().collect();
            let mut physics = self.world.write_resource::<PhysicsResource>();
            self.stepper.step(
                &mut physics.world,
                FRAME_DT,
                &self.ground,
                &blasts,
                &[],
                &mut lines,
            );
        }
        breakage::run(&mut self.world);
    }

    fn run(&mut self, seconds: f32) {
        for _ in 0..(seconds / FRAME_DT).round() as usize {
            self.step();
        }
    }

    fn physics_mut(&self) -> specs::shred::FetchMut<'_, PhysicsResource> {
        self.world.write_resource::<PhysicsResource>()
    }

    fn body(&self, entity: Entity) -> RigidBodyHandle {
        self.world
            .read_storage::<RigidBodyComponent>()
            .get(entity)
            .expect("a piece has a body")
            .0
    }

    fn at(&self, body: RigidBodyHandle) -> Point3<f32> {
        self.world
            .read_resource::<PhysicsResource>()
            .world
            .body(body)
            .expect("a live body")
            .position()
    }

    fn plant(&self) -> Arc<GrownPlant> {
        self.world
            .read_storage::<PlantPiece>()
            .get(self.tree)
            .expect("a tree")
            .plant
            .clone()
    }

    /// Every piece of the tree: its entity and its body.
    fn pieces(&self) -> Vec<(Entity, RigidBodyHandle)> {
        let pieces = self.world.read_storage::<PlantPiece>();
        let bodies = self.world.read_storage::<RigidBodyComponent>();
        (&self.world.entities(), &pieces, &bodies)
            .join()
            .map(|(e, _, b)| (e, b.0))
            .collect()
    }

    /// Where the plant's frame is in the world on `body`, read off limb
    /// `l`'s collider.
    fn plant_pose(&self, body: RigidBodyHandle, l: usize) -> Isometry3<f32> {
        let plant = self.plant();
        let physics = self.world.read_resource::<PhysicsResource>();
        let rigid = physics.world.body(body).expect("a live body");
        let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
        pose * plant
            .frame_on(&physics.world, l)
            .expect("the limb is on it")
    }

    /// Where the `m`th crown's frame is in the world, on `body`.
    fn crown_on(&self, body: RigidBodyHandle, m: usize) -> Point3<f32> {
        let limb = self.plant().plant.masses[m].limb;
        let pose = self.plant_pose(body, limb) * self.plant().plant.masses[m].pose;
        Point3::from(pose.translation.vector)
    }

    /// The tree's bond holding the branch that holds crown `m`.
    fn bond_of_crown(&self, m: usize) -> Bond {
        let plant = self.plant();
        let part = plant.colliders.parts[plant.plant.masses[m].limb];
        let fractures = self.world.read_storage::<CompoundFracture>();
        *fractures
            .get(self.tree)
            .expect("a tree is a compound")
            .graph
            .bonds()
            .iter()
            .find(|bond| bond.b == part)
            .expect("a side crown's branch is bonded")
    }

    /// The loading per area `blast` gives the fork of the branch holding
    /// crown `m`, as fracture judges it.
    fn loading_at_fork(&self, m: usize, blast: &PhysicsImpulse) -> f32 {
        let bond = self.bond_of_crown(m);
        let physics = self.world.read_resource::<PhysicsResource>();
        let rigid = physics.world.body(self.body(self.tree)).expect("a tree");
        let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
        BondLoads::new([], std::slice::from_ref(blast), pose, None).blast_load(&bond)
    }

    /// A blast `distance` out from crown `m`, away from the trunk.
    fn blast_beside(&self, m: usize, distance: f32) -> PhysicsImpulse {
        let crown = self.crown_on(self.body(self.tree), m);
        let outward = Vector3::new(crown.x, 0.0, crown.z).normalize();
        Explosion::new(crown + outward * distance).physics_impulse()
    }

    /// Steps `seconds` with the tree's branches unbreakable, reporting the
    /// largest contact load any fork felt, N·s, as fracture reads it.
    fn peak_fork_load(&mut self, seconds: f32) -> f32 {
        let body = self.body(self.tree);
        let unbreakable = {
            let mut fractures = self.world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(self.tree).expect("a tree");
            fracture.default_toughness.replace(f32::INFINITY)
        };
        let mut tracker = ContactLoadTracker::default();
        let mut peak: f32 = 0.0;
        for _ in 0..(seconds / FRAME_DT).round() as usize {
            self.step();
            let physics = self.world.read_resource::<PhysicsResource>();
            let fractures = self.world.read_storage::<CompoundFracture>();
            let graph = &fractures.get(self.tree).expect("a tree").graph;
            let rigid = physics.world.body(body).expect("a tree");
            let colliders = rigid.colliders().to_vec();
            let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
            let spikes = tracker.advance(&physics.world, body, &colliders, FRAME_DT);
            let loads = BondLoads::new(colliders.into_iter().zip(spikes), &[], pose, None);
            for bond in graph.bonds() {
                peak = peak.max(loads.contact_load(graph, bond));
            }
        }
        let mut fractures = self.world.write_storage::<CompoundFracture>();
        fractures
            .get_mut(self.tree)
            .expect("a tree")
            .default_toughness = unbreakable;
        peak
    }

    /// The weakest contact load that breaks one of the tree's branches.
    fn branch_toughness(&self) -> f32 {
        self.plant().joinery.branch_toughness
    }
}

fn forked_plant() -> Plant {
    LollipopSpec {
        crowns: 3,
        ..LollipopSpec::default()
    }
    .grow(SEED)
}

/// A tree's frame lying on its side `height` up, its crowns towards +x.
fn lying(height: f32) -> Isometry3<f32> {
    Isometry3::from_parts(
        Vector3::new(-1.5, height, 0.0).into(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -std::f32::consts::FRAC_PI_2),
    )
}

/// A forked tree left alone stands where it grew, its weld holding and its
/// branches on it.
#[test]
fn a_tree_stands_where_it_grew() {
    let mut grove = Grove::forked();
    let body = grove.body(grove.tree);
    let crowns = grove.plant().plant.masses.len();
    let start: Vec<Point3<f32>> = (0..crowns).map(|m| grove.crown_on(body, m)).collect();
    grove.run(10.0);
    assert_eq!(
        grove.pieces().len(),
        1,
        "a branch fell off a tree standing still"
    );
    for (m, start) in start.iter().enumerate() {
        let moved = (grove.crown_on(body, m) - start).magnitude();
        assert!(moved < 0.02, "a crown sagged {moved:.3} m");
    }
}

/// Let go by the ground, a tree falls over and comes to rest on its crown
/// and roots, not sunk into the ground to its trunk; a forked one keeps
/// its branches through the fall.
#[test]
fn a_tree_let_go_falls_and_rests_on_its_crown() {
    for plant in [
        LollipopSpec::default().grow(2),
        CypressSpec::default().grow(3),
        forked_plant(),
    ] {
        // The ground it grew in is gone: its roots hang just clear of what
        // is left.
        let mut grove = Grove::standing_at(plant, Isometry3::translation(0.0, 0.4, 0.0));
        let body = grove.body(grove.tree);
        grove.let_go();
        // A nudge, as the blast that took its ground would give it.
        grove
            .physics_mut()
            .world
            .apply_impulse(body, Vector3::new(60.0, 0.0, 0.0));
        let peak = grove.peak_fork_load(12.0);
        let crown = grove.crown_on(body, 0);
        // Lying on its leaves, the crown's middle is at least about half its
        // widest lobe's radius up; sunk to its trunk, it is a few
        // centimetres.
        let lowest_core = 0.5
            * grove.plant().plant.masses[0]
                .shape
                .lobes
                .iter()
                .map(|lobe| lobe.least_radius())
                .fold(0.0, f32::max);
        let length = grove.plant_pose(body, 0).rotation * Vector3::y();
        println!(
            "fallen: crown at {crown:?}, trunk {:.0}° from upright; largest load at a fork \
             {peak:.0} N·s against {:.0}",
            length.y.abs().acos().to_degrees(),
            grove.branch_toughness()
        );
        assert!(length.y.abs() < 0.5, "it is still standing");
        assert!(crown.y > lowest_core, "its crown sank to {:.2} m", crown.y);
        assert!(
            grove.physics_mut().world.is_sleeping(body),
            "it never came to rest"
        );
        assert!(
            peak < grove.branch_toughness(),
            "the fall would break a branch: {peak:.0} N·s at a fork"
        );
    }
}

/// A forked tree felled through the break pipeline lands with every branch
/// on it.
#[test]
fn a_felled_tree_keeps_its_branches() {
    let mut grove = Grove::standing_at(forked_plant(), Isometry3::translation(0.0, 0.4, 0.0));
    let body = grove.body(grove.tree);
    grove.let_go();
    grove
        .physics_mut()
        .world
        .apply_impulse(body, Vector3::new(60.0, 0.0, 0.0));
    grove.run(8.0);
    assert_eq!(grove.pieces().len(), 1, "a branch broke off in the fall");
}

/// A tree dropped on its side from 6 m or more lands hard enough on its
/// crowns to break a branch at its fork, and each piece keeps its leaves.
#[test]
fn a_hard_landing_breaks_a_branch() {
    for height in [6.0, 10.0, 20.0] {
        let mut measured = Grove::standing_at(forked_plant(), lying(height));
        measured.let_go();
        let peak = measured.peak_fork_load(4.0);
        println!(
            "dropped lying from {height} m: largest load at a fork {peak:.0} N·s against {:.0}",
            measured.branch_toughness()
        );
        assert!(
            peak > measured.branch_toughness(),
            "from {height} m no fork felt enough to break: {peak:.0} N·s"
        );

        let mut grove = Grove::standing_at(forked_plant(), lying(height));
        grove.let_go();
        grove.run(4.0);
        let pieces = grove.pieces();
        assert!(
            pieces.len() > 1,
            "from {height} m it landed with every branch on it"
        );
        let foliage = grove.world.read_storage::<Foliage>();
        for (entity, _) in &pieces {
            assert!(foliage.contains(*entity), "a piece without its leaves");
        }
    }
}

/// What a blast gives a side branch's fork falls off with distance from
/// its crown across the branch's strength: within 2 m it breaks, from 3 m
/// it holds.
#[test]
fn a_branch_breaks_within_two_metres_of_a_blast() {
    let grove = Grove::forked();
    let strength = grove.plant().joinery.branch_strength;
    for distance in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0] {
        let loading = grove.loading_at_fork(1, &grove.blast_beside(1, distance));
        println!(
            "blast {distance} m off its crown: {loading:.0} N·s/m² at the fork against {strength}"
        );
        if distance <= 2.0 {
            assert!(loading > strength, "{distance} m off it held");
        }
        if distance >= 3.0 {
            assert!(loading < strength, "{distance} m off it broke");
        }
    }
}

/// A branch broken off, as by a blast too small to throw it, drops from its
/// fork: the tree it overlapped there pushes it out without flinging it,
/// and it lands beside the tree, which still stands.
#[test]
fn a_broken_branch_drops_from_its_fork() {
    let mut grove = Grove::forked();
    let trunk = grove.body(grove.tree);
    let plant = grove.plant();
    let branch = plant.plant.masses[1].limb;
    {
        let mut fractures = grove.world.write_storage::<CompoundFracture>();
        let fracture = fractures.get_mut(grove.tree).expect("a tree");
        fracture.sever(plant.colliders.parts[branch], Vector3::zeros());
    }
    grove.step();
    let (_, broken) = *grove
        .pieces()
        .iter()
        .find(|(_, body)| *body != trunk)
        .expect("the branch came off");
    let trunk_at = grove.at(trunk);
    // Sideways and upwards, before it lands: falling adds to neither.
    let mut fastest: f32 = 0.0;
    for _ in 0..15 {
        grove.step();
        let v = grove
            .physics_mut()
            .world
            .body(broken)
            .expect("the branch")
            .linear_velocity();
        fastest = fastest.max(Vector3::new(v.x, v.y.max(0.0), v.z).magnitude());
    }
    grove.run(8.0);
    let landed = grove.at(broken);
    println!("fastest {fastest:.2} m/s across in its first quarter second; landed at {landed:?}");
    assert!(fastest < 1.0, "flung at {fastest:.2} m/s");
    assert!(
        landed.y > 0.0 && landed.y < 1.0,
        "came to rest at {landed:?}"
    );
    assert!(
        grove.physics_mut().world.is_sleeping(broken),
        "it never came to rest"
    );
    let moved = (grove.at(trunk) - trunk_at).magnitude();
    assert!(moved < 0.05, "the tree moved {moved:.2} m");
}

/// A grenade thrown at a tree's trunk strikes it and comes back, as off any
/// body: trees are solid to grenades.
#[test]
fn a_grenade_strikes_a_trunk() {
    let mut grove = Grove::of(LollipopSpec::default().grow(2));
    let trunk = grove.plant().plant.limbs.limbs[0];
    let at = trunk.base + (trunk.tip - trunk.base) * 0.3;
    let grenade = {
        let mut physics = grove.physics_mut();
        let world: &mut PhysicsWorld = &mut physics.world;
        let grenade = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(at.x - 2.0, at.y, at.z))
                .linear_velocity(Vector3::x() * 12.0)
                .gravity_scale(0.0),
        );
        world.attach_collider(grenade, ColliderDesc::sphere(0.2).density(2000.0));
        world.wake_body(grenade);
        grenade
    };
    grove.run(0.5);
    let x = grove.at(grenade).x;
    println!(
        "grenade at x {x:.2} after 0.5 s, the trunk at x {:.2}",
        at.x
    );
    assert!(x < at.x, "it went through the trunk to x {x:.2}");
}
