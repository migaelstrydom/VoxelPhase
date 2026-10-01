//! Structural stability regression tests.
//!
//! Each test builds a multi-body structure on flat terrain and verifies that
//! it settles to rest within tight tolerance. These are regression guards
//! against solver changes that break stacking stability.

use nalgebra::Vector3;

use crate::debug::DebugLines;
use crate::physics::bench_harness::framework::{
    run_scenario, BenchRunConfig, PhysicsBenchScenario,
};
use crate::physics::bench_harness::scenarios::{
    BoxGridScenario, HoneycombWallScenario, JengaTowerScenario, TempleScenario,
    VoussoirArchScenario,
};
use crate::physics::{
    ConstraintKind, PhysicsWorld, RigidBodyHandle, SequentialStepper, StaticGeometry, Stepper,
};

use super::write_exports;

// ── Honeycomb wall ───────────────────────────────────────────────────

#[test]
fn honeycomb_wall_settles() {
    let scenario = HoneycombWallScenario::new();
    let cfg = BenchRunConfig {
        duration: 8.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "honeycomb_wall");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 0.0 && final_y < 3.0,
        "tracked cell should stay at rest height, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "honeycomb wall should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "honeycomb wall should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}

// ── Voussoir arch ────────────────────────────────────────────────────

#[test]
fn voussoir_arch_holds_together() {
    let scenario = VoussoirArchScenario::new();
    let cfg = BenchRunConfig {
        duration: 15.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "voussoir_arch");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // The keystone should stay near the top of the arch.
    // inner_r=10, outer_r=15, center_y=0.5 → keystone centroid ≈ y=12.5+0.5=13
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 10.0,
        "keystone should stay near arch apex, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(3.0);
    assert!(
        tail_linear < 0.30,
        "arch keystone should settle (linear < 0.30 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.30,
        "arch keystone should settle (angular < 0.30 rad/s), got {tail_angular:.4}"
    );
}

// ── Jenga tower ──────────────────────────────────────────────────────

#[test]
fn jenga_tower_settles() {
    let scenario = JengaTowerScenario::new(7);
    let cfg = BenchRunConfig {
        duration: 8.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "jenga_tower_7");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Top-center block of a 7-layer tower: y ≈ block_half_height + 6 * block_height
    // half_height = 0.15, block_height = 0.3 → y ≈ 0.15 + 6*0.3 = 1.95
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 1.5 && final_y < 2.5,
        "top block should stay near tower top, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "jenga tower should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "jenga tower should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}

/// A sleeping tower nudged at the top wakes whole on that frame, and stands:
/// no block below the nudged one sinks.
///
/// A sleeping body generates no contacts, so a woken one has none until a
/// pass generates them. When that waited for the next frame, the tower woke a
/// layer per frame, each layer falling a frame's worth onto the one below
/// before being pushed back up.
#[test]
fn a_sleeping_tower_nudged_at_the_top_wakes_whole_and_stands() {
    const FRAME_DT: f32 = 1.0 / 60.0;
    let scenario = JengaTowerScenario::new(12);
    let mut config = scenario.build_world().config().clone();
    config.sleep.enabled = true;
    let mut world = PhysicsWorld::new(config);
    scenario.setup(&mut world);
    let blocks: Vec<RigidBodyHandle> = world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect();
    for &block in &blocks {
        world.wake_body(block);
    }
    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    let mut debug = DebugLines::default();
    let mut step = |world: &mut PhysicsWorld| {
        stepper.step(world, FRAME_DT, scenario.geometry(), &[], &[], &mut debug)
    };
    for _ in 0..240 {
        step(&mut world);
    }
    assert!(
        blocks.iter().all(|&b| world.is_sleeping(b)),
        "the tower never fell asleep"
    );

    let height = |world: &PhysicsWorld, b: RigidBodyHandle| world.body(b).unwrap().position().y;
    let top = *blocks
        .iter()
        .max_by(|&&a, &&b| height(&world, a).total_cmp(&height(&world, b)))
        .unwrap();
    let rest: Vec<(RigidBodyHandle, f32)> = blocks
        .iter()
        .filter(|&&b| b != top)
        .map(|&b| (b, height(&world, b)))
        .collect();
    world.set_body_velocity(top, Vector3::new(0.05, 0.0, 0.0), Vector3::zeros());

    step(&mut world);
    let still_asleep = blocks.iter().filter(|&&b| world.is_sleeping(b)).count();
    let mut sunk = 0.0f32;
    for _ in 0..30 {
        step(&mut world);
        for &(block, rest_y) in &rest {
            sunk = sunk.max(rest_y - height(&world, block));
        }
    }
    eprintln!("nudged tower: {still_asleep} still asleep after a frame, sank up to {sunk:.5} m");

    assert_eq!(still_asleep, 0, "the tower did not wake whole");
    assert!(sunk < 0.001, "a block sank {sunk:.4} m when the tower woke");
}

// ── Temple ───────────────────────────────────────────────────────────

#[test]
fn temple_stands_stable() {
    let scenario = TempleScenario::new();
    let cfg = BenchRunConfig {
        duration: 10.0,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "temple");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Tracked body is a front column. Column base is at stylobate_top ≈ 1.0,
    // column height = 8.0, centroid ≈ y=5.0.
    let final_y = run.samples.last().unwrap().y;
    assert!(
        final_y > 3.0 && final_y < 7.0,
        "tracked column should stay upright, got y={final_y:.4}"
    );

    let (tail_linear, tail_angular) = run.tail_max_speeds(2.0);
    assert!(
        tail_linear < 0.02,
        "temple should settle (linear < 0.02 m/s), got {tail_linear:.4}"
    );
    assert!(
        tail_angular < 0.05,
        "temple should settle (angular < 0.05 rad/s), got {tail_angular:.4}"
    );
}

// ── Every block holds still ──────────────────────────────────────────

/// Once a structure has settled, none of its blocks creeps. The tests above
/// follow one body each, with tolerances loose enough to pass a structure in
/// slow collapse: with warm starts at 60 %, the arch sagged 9 cm in three
/// seconds and the jenga tower crept 1.4 cm while each passed. Sleep hides
/// that in the game until something wakes the structure.
///
/// Not the honeycomb wall, whose top cell rolls off as it settles.
#[test]
fn every_block_of_a_settled_structure_holds_still() {
    const LIMIT: f32 = 1.0e-3;
    let structures: [(&str, &dyn PhysicsBenchScenario); 4] = [
        ("voussoir arch", &VoussoirArchScenario::new()),
        ("jenga tower", &JengaTowerScenario::new(12)),
        ("box grid", &BoxGridScenario::new(5)),
        ("temple", &TempleScenario::new()),
    ];
    let failures: Vec<String> = structures
        .into_iter()
        .filter_map(|(name, scenario)| {
            let creep = creep_once_settled(scenario, 3.0, 3.0);
            (creep > LIMIT).then(|| format!("{name}: a block crept {creep:.4} m"))
        })
        .collect();
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

/// How far the block that moved most moved over `watch` seconds, after
/// `settle` seconds to come to rest. Every block starts awake, as a blast
/// or a footstep would leave it, and the scenario's world keeps it awake.
fn creep_once_settled(scenario: &dyn PhysicsBenchScenario, settle: f32, watch: f32) -> f32 {
    const FRAME_DT: f32 = 1.0 / 60.0;
    let mut world = scenario.build_world();
    scenario.setup(&mut world);
    let blocks: Vec<RigidBodyHandle> = world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect();
    for &block in &blocks {
        world.wake_body(block);
    }

    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    let mut debug = DebugLines::default();
    let mut run_for = |world: &mut _, seconds: f32| {
        for _ in 0..(seconds / FRAME_DT).round() as u32 {
            stepper.step(world, FRAME_DT, scenario.geometry(), &[], &[], &mut debug);
        }
    };
    run_for(&mut world, settle);
    let settled: Vec<_> = blocks
        .iter()
        .map(|&b| world.body(b).unwrap().position())
        .collect();
    run_for(&mut world, watch);
    blocks
        .iter()
        .zip(&settled)
        .map(|(&b, at)| (world.body(b).unwrap().position() - at).norm())
        .fold(0.0, f32::max)
}

// ── A knocked arch gains nothing ─────────────────────────────────────

/// An arch knocked about resettles without regaining energy it lost. It did:
/// shock propagation took part of every contact's push off the lower block,
/// which the ground takes up under a stack, but along an arch's leaning
/// joints the part withheld pushed sideways with nothing to push back, and
/// the warm start carried it on. Its crown rose up to 7 cm on its own.
///
/// How much it regained depended on the order contacts were solved in, so
/// one knock proves little: this knocks it sixteen ways.
#[test]
fn a_knocked_arch_regains_no_energy() {
    const LIMIT: f32 = 0.1;
    let scenario = VoussoirArchScenario::new();
    let worst = (0..8)
        .flat_map(|pattern| [false, true].map(|ordered| (pattern, ordered)))
        .map(|(pattern, ordered)| energy_regained_after_knock(&scenario, pattern, ordered))
        .fold(0.0, f32::max);
    assert!(
        worst < LIMIT,
        "a knocked arch regained {worst:.3} J/kg, as if it rose {:.1} cm on its own",
        worst / 9.81 * 100.0
    );
}

/// A body held upright is not ground. Shock propagation counted any body with
/// a constraint of its own as resting on the world, so a block held upright in
/// the middle of a tower was depth zero, and the tower above it was ordered
/// and scaled from there — upside down against the blocks beside it. The
/// player's capsule is held so, and jumped into a jenga tower it threw the top
/// half twenty metres into the air.
#[test]
fn a_tower_with_a_block_held_upright_regains_no_energy() {
    const LIMIT: f32 = 0.1;
    let scenario = HeldBlockTower(JengaTowerScenario::new(18));
    let worst = (0..4)
        .flat_map(|pattern| [false, true].map(|ordered| (pattern, ordered)))
        .map(|(pattern, ordered)| energy_regained_after_knock(&scenario, pattern, ordered))
        .fold(0.0, f32::max);
    assert!(
        worst < LIMIT,
        "a knocked tower regained {worst:.3} J/kg, as if it rose {:.1} cm on its own",
        worst / 9.81 * 100.0
    );
}

/// A jenga tower whose middle block, halfway up, is held upright the way a
/// character's capsule is.
struct HeldBlockTower(JengaTowerScenario);

impl PhysicsBenchScenario for HeldBlockTower {
    fn name(&self) -> &'static str {
        "held_block_tower"
    }

    fn restitution(&self) -> f32 {
        self.0.restitution()
    }

    fn build_world(&self) -> PhysicsWorld {
        self.0.build_world()
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let tracked = self.0.setup(world);
        let held = world
            .bodies()
            .iter()
            .filter(|(_, body)| body.is_dynamic())
            .map(|(index, _)| RigidBodyHandle(index))
            .nth((self.0.layers / 2 * 3 + 1) as usize)
            .unwrap();
        world.create_constraint(ConstraintKind::KeepAttitude {
            body: held,
            pitch: 0.0,
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        tracked
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        self.0.geometry()
    }
}

/// Settle a structure, knock every block up and sideways at 1 m/s — in
/// directions set by `pattern` — and return the most its energy per kilogram
/// rose above the least it had had since.
fn energy_regained_after_knock(
    scenario: &dyn PhysicsBenchScenario,
    pattern: usize,
    ordered_contacts: bool,
) -> f32 {
    const FRAME_DT: f32 = 1.0 / 60.0;
    let mut config = scenario.build_world().config().clone();
    config.deterministic_contact_ordering = ordered_contacts;
    let mut world = PhysicsWorld::new(config);
    scenario.setup(&mut world);
    let blocks: Vec<RigidBodyHandle> = world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect();
    for &block in &blocks {
        world.wake_body(block);
    }
    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    let mut debug = DebugLines::default();
    for _ in 0..180 {
        stepper.step(
            &mut world,
            FRAME_DT,
            scenario.geometry(),
            &[],
            &[],
            &mut debug,
        );
    }

    let golden_angle = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
    for (i, &block) in blocks.iter().enumerate() {
        let angle = golden_angle * (i + pattern * 7) as f32 + pattern as f32;
        let body = world.body_mut(block).unwrap();
        let velocity = body.linear_velocity() + Vector3::new(angle.cos(), 1.0, angle.sin());
        body.set_linear_velocity(velocity);
    }

    let mut lowest = energy_per_kg(&world, &blocks);
    let mut regained = 0.0f32;
    for _ in 0..480 {
        stepper.step(
            &mut world,
            FRAME_DT,
            scenario.geometry(),
            &[],
            &[],
            &mut debug,
        );
        let energy = energy_per_kg(&world, &blocks);
        lowest = lowest.min(energy);
        regained = regained.max(energy - lowest);
    }
    regained
}

/// Kinetic energy, spin included, and height, per kilogram of the bodies.
fn energy_per_kg(world: &PhysicsWorld, bodies: &[RigidBodyHandle]) -> f32 {
    let gravity = world.config().gravity;
    let (energy, mass) = bodies
        .iter()
        .filter_map(|&h| world.body(h))
        .map(|body| {
            let spin = body.angular_velocity();
            let rotational = body
                .world_inv_inertia()
                .try_inverse()
                .map_or(0.0, |inertia| 0.5 * spin.dot(&(inertia * spin)));
            let kinetic = 0.5 * body.mass() * body.linear_velocity().norm_squared();
            let height = -body.mass() * gravity.dot(&body.position().coords);
            (kinetic + rotational + height, body.mass())
        })
        .fold((0.0, 0.0), |(e, m), (be, bm)| (e + be, m + bm));
    energy / mass
}
