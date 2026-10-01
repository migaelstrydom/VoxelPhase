//! A heavy body on light ones: how far the stack sags and creeps, and how
//! long it takes to settle, under the solver settings that bear on it.
//!
//! Shock propagation's mass scaling exists for these stacks, so they are the
//! measure of what it buys: `mass_ratio_sweep` prints each stack's figures
//! with the scaling on and off, at the game's iteration count and below it.

use crate::debug::DebugLines;
use crate::physics::bench_harness::framework::PhysicsBenchScenario;
use crate::physics::bench_harness::scenarios::{MassRatioArrangement, MassRatioStackScenario};
use crate::physics::{
    PgsNgsConfig, PgsNgsSolver, PhysicsWorld, RigidBodyHandle, SequentialStepper,
    ShockPropagationConditioner, ShockPropagationConfig, Stepper, SweepClampCcd,
};

const FRAME_DT: f32 = 1.0 / 60.0;
/// Seconds a stack is given to settle before it is watched.
const SETTLE_SECONDS: f32 = 3.0;
/// Seconds it is watched for creep and motion.
const WATCH_SECONDS: f32 = 2.0;

/// The solver settings a stack is measured under.
#[derive(Clone, Copy, Debug)]
struct SolverSettings {
    shock_alpha: f32,
    iterations: u32,
}

impl SolverSettings {
    fn game() -> Self {
        Self {
            shock_alpha: ShockPropagationConfig::default().shock_alpha,
            iterations: PgsNgsConfig::default().solver_iterations,
        }
    }
}

/// How a stack stood.
#[derive(Clone, Copy, Debug)]
struct StackFigures {
    /// How far the heavy body ended below where it was set, in metres.
    sag: f32,
    /// How far the heavy body moved while watched, in metres.
    creep: f32,
    /// The fastest any body moved while watched, in m/s.
    motion: f32,
    /// How far the heavy body ended from where it was set, sideways, in metres.
    shift: f32,
}

fn measure(scenario: &MassRatioStackScenario, settings: SolverSettings) -> StackFigures {
    let config = scenario.build_world().config().clone();
    let mut world = PhysicsWorld::with_components(
        config,
        Box::new(PgsNgsSolver::new(PgsNgsConfig {
            solver_iterations: settings.iterations,
            ..PgsNgsConfig::default()
        })),
        Box::new(ShockPropagationConditioner::new(ShockPropagationConfig {
            shock_alpha: settings.shock_alpha,
            ..ShockPropagationConfig::default()
        })),
        Box::new(SweepClampCcd::default()),
    );
    let heavy = scenario.setup(&mut world);
    let bodies: Vec<RigidBodyHandle> = world
        .bodies()
        .iter()
        .filter(|(_, body)| body.is_dynamic())
        .map(|(index, _)| RigidBodyHandle(index))
        .collect();
    for &body in &bodies {
        world.wake_body(body);
    }
    let set_at = world.body(heavy).unwrap().position();

    let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
    let mut debug = DebugLines::default();
    let mut step = |world: &mut PhysicsWorld| {
        stepper.step(world, FRAME_DT, scenario.geometry(), &[], &[], &mut debug)
    };
    for _ in 0..(SETTLE_SECONDS / FRAME_DT) as usize {
        step(&mut world);
    }
    let watched_from = world.body(heavy).unwrap().position();
    let mut creep = 0.0f32;
    let mut motion = 0.0f32;
    for _ in 0..(WATCH_SECONDS / FRAME_DT) as usize {
        step(&mut world);
        creep = creep.max((world.body(heavy).unwrap().position() - watched_from).norm());
        for &body in &bodies {
            motion = motion.max(world.body(body).unwrap().linear_velocity().norm());
        }
    }
    let end = world.body(heavy).unwrap().position();
    StackFigures {
        sag: set_at.y - end.y,
        creep,
        motion,
        shift: (end - set_at).xz().norm(),
    }
}

/// Every stack this file measures: both arrangements, short and tall, at
/// mass ratios from even to a hundred to one.
fn stacks() -> Vec<MassRatioStackScenario> {
    let mut stacks = Vec::new();
    for arrangement in [
        MassRatioArrangement::HeavyOnColumn,
        MassRatioArrangement::SlabOnColumns,
    ] {
        for layers in [3, 8] {
            for ratio in [1.0, 10.0, 100.0] {
                stacks.push(MassRatioStackScenario::new(layers, ratio, arrangement));
            }
        }
    }
    stacks
}

/// A cube a hundred times heavier than each of the eight under it stands,
/// at the game's settings, sagging no more than the slop its contacts allow
/// and not creeping. Without shock scaling it topples.
#[test]
fn a_heavy_cube_on_a_tall_light_column_stands() {
    let stack = MassRatioStackScenario::new(8, 100.0, MassRatioArrangement::HeavyOnColumn);
    let figures = measure(&stack, SolverSettings::game());
    eprintln!("heavy cube on 8 light: {figures:?}");

    assert!(figures.sag < 0.015, "the stack sagged {:.4} m", figures.sag);
    assert!(
        figures.creep < 0.002,
        "the heavy cube crept {:.4} m in {WATCH_SECONDS} s",
        figures.creep
    );
}

/// Print every stack's figures with shock scaling at the game's α and off,
/// at the game's iteration count and fewer.
///
/// ```text
/// cargo test --release --features bench_harness --lib mass_ratio_sweep -- --ignored --nocapture
/// ```
#[test]
#[ignore]
fn mass_ratio_sweep() {
    let game = SolverSettings::game();
    eprintln!(
        "{:<18} {:>6} {:>6} {:>5} {:>4} | {:>9} {:>9} {:>9} {:>9}",
        "stack", "layers", "ratio", "alpha", "iter", "sag mm", "creep mm", "motion", "shift mm"
    );
    for stack in stacks() {
        for iterations in [game.iterations, 1] {
            for shock_alpha in [game.shock_alpha, 0.6, 1.0] {
                let figures = measure(
                    &stack,
                    SolverSettings {
                        shock_alpha,
                        iterations,
                    },
                );
                eprintln!(
                    "{:<18} {:>6} {:>6} {:>5.1} {:>4} | {:>9.3} {:>9.3} {:>9.4} {:>9.3}",
                    stack.name(),
                    stack.layers,
                    stack.ratio,
                    shock_alpha,
                    iterations,
                    figures.sag * 1000.0,
                    figures.creep * 1000.0,
                    figures.motion,
                    figures.shift * 1000.0,
                );
            }
        }
    }
}
