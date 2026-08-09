//! Does the substep cap change what physics *does*, or only how fast it keeps up?
//!
//! `FixedTimestep` clamps its accumulator to `fixed_dt * max_substeps` and
//! discards the excess, so a frame longer than that budget advances the world
//! by less than real time. With the shipped 1/240 × 8 the budget is 0.0333s —
//! exactly 30 FPS — which a 30Hz display sits right on top of.
//!
//! Raising the cap is not free, though, and the reason is the stepping shape:
//! contacts are generated **once per frame** and then reused across every
//! substep. A bigger cap lets a single contact set carry more simulated time,
//! which is the classic way a fast body ends up resolved against geometry it
//! has already left.
//!
//! These tests pin both halves of that trade: the cap must relieve the
//! time-loss, and it must not degrade contact handling at the frame rates the
//! game actually runs at.

use super::super::framework::{run_scenario, BenchRunConfig, PhysicsBenchScenario};
use super::super::scenarios::{
    GrazingSphereWallCcdScenario, GrenadeSpeedWallCcdScenario, HighSpeedSphereCcdScenario,
};
use crate::physics::stepping::FixedTimestep;
use crate::physics::{PhysicsWorld, RigidBodyHandle, StaticGeometry};

/// The shipped configuration.
const FIXED_DT: f32 = 1.0 / 240.0;
const SHIPPED_CAP: u32 = 8;
const PROPOSED_CAP: u32 = 12;

/// The frame time actually measured in-game on a 30Hz display.
const MEASURED_FRAME_DT: f32 = 0.03403;

/// Runs an existing scenario at a chosen frame rate.
///
/// Frame rate belongs to the scenario rather than the run config, so exercising
/// one scenario across several is otherwise a copy-paste job. Everything except
/// `frame_dt` delegates to the inner scenario.
struct AtFrameRate<'a, S: PhysicsBenchScenario> {
    inner: &'a S,
    frame_dt: f32,
}

impl<'a, S: PhysicsBenchScenario> PhysicsBenchScenario for AtFrameRate<'a, S> {
    fn name(&self) -> &'static str {
        self.inner.name()
    }
    fn restitution(&self) -> f32 {
        self.inner.restitution()
    }
    fn build_world(&self) -> PhysicsWorld {
        self.inner.build_world()
    }
    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        self.inner.setup(world)
    }
    fn geometry(&self) -> &dyn StaticGeometry {
        self.inner.geometry()
    }
    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        self.frame_dt
    }
}

/// Simulated seconds advanced per real second at a given frame time and cap.
///
/// Measured against `FixedTimestep` directly: the bench runner drives its loop
/// off `sim_time` and keeps going until the target duration is reached, so it
/// compensates for dropped time by construction and cannot observe this.
fn time_kept(frame_dt: f32, max_substeps: u32) -> f32 {
    let mut timestep = FixedTimestep::new(FIXED_DT, max_substeps);
    let frames = 600;
    let mut simulated = 0.0;
    for _ in 0..frames {
        simulated += timestep.accumulate(frame_dt) as f32 * FIXED_DT;
    }
    simulated / (frames as f32 * frame_dt)
}

/// The reported symptom: at 30Hz the shipped cap runs the world slow.
///
/// Measured: 0.9795 at the shipped cap, 0.9999 at the proposed one. At 20 FPS
/// the gap widens to 0.667 vs 0.972.
#[test]
fn the_shipped_cap_loses_time_at_the_measured_frame_rate() {
    let kept = time_kept(MEASURED_FRAME_DT, SHIPPED_CAP);

    assert!(
        kept < 0.99,
        "expected measurable time loss at the shipped cap, kept={kept:.4}"
    );
}

#[test]
fn the_proposed_cap_keeps_real_time_at_the_measured_frame_rate() {
    let kept = time_kept(MEASURED_FRAME_DT, PROPOSED_CAP);

    assert!(
        kept > 0.999,
        "expected the proposed cap to keep up, kept={kept:.4}"
    );
}

/// The cap only binds below its own frame rate, so raising it must be inert
/// where the game already keeps up. If this fails, the change is not free.
#[test]
fn raising_the_cap_changes_nothing_at_sixty_frames_per_second() {
    for frame_dt in [1.0 / 60.0, 1.0 / 120.0] {
        let shipped = time_kept(frame_dt, SHIPPED_CAP);
        let proposed = time_kept(frame_dt, PROPOSED_CAP);

        assert_eq!(
            shipped, proposed,
            "cap should be inert at frame_dt={frame_dt}"
        );
    }
}

/// The cost side of the trade. A bigger cap lets one contact set carry more
/// simulated time, so the fast-body scenarios have to survive it — at the
/// frame rate the cap is being raised *for*, and at a genuine hitch.
#[test]
fn the_proposed_cap_does_not_let_a_fast_sphere_tunnel() {
    let scenario = GrazingSphereWallCcdScenario::new();

    for frame_dt in [MEASURED_FRAME_DT, 1.0 / 30.0, 1.0 / 20.0] {
        for cap in [SHIPPED_CAP, PROPOSED_CAP] {
            let run = run_scenario(
                &AtFrameRate {
                    inner: &scenario,
                    frame_dt,
                },
                BenchRunConfig {
                    duration: 0.5,
                    max_substeps_per_frame: cap as usize,
                    ..BenchRunConfig::default()
                },
            );

            // The sphere comes to rest against the wall at x = radius. The
            // centre crossing x = 0 at all means it entered the wall, which is
            // a far tighter bar than the "fully through" check in `ccd.rs`.
            let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
            assert!(
                min_x > 0.0,
                "entered the wall: frame_dt={frame_dt} cap={cap} min_x={min_x}"
            );
        }
    }
}

#[test]
fn the_proposed_cap_does_not_let_a_grenade_speed_sphere_tunnel() {
    let scenario = GrenadeSpeedWallCcdScenario::new();

    for frame_dt in [MEASURED_FRAME_DT, 1.0 / 20.0] {
        for cap in [SHIPPED_CAP, PROPOSED_CAP] {
            let run = run_scenario(
                &AtFrameRate {
                    inner: &scenario,
                    frame_dt,
                },
                BenchRunConfig {
                    duration: 0.5,
                    max_substeps_per_frame: cap as usize,
                    ..BenchRunConfig::default()
                },
            );

            // The sphere comes to rest against the wall at x = radius. The
            // centre crossing x = 0 at all means it entered the wall, which is
            // a far tighter bar than the "fully through" check in `ccd.rs`.
            let min_x = run.samples.iter().map(|s| s.x).fold(f32::MAX, f32::min);
            assert!(
                min_x > 0.0,
                "entered the wall: frame_dt={frame_dt} cap={cap} min_x={min_x}"
            );
        }
    }
}

/// Resting contact must not degrade either — a deeper substep budget per
/// contact set is exactly where a body would start sinking.
#[test]
fn the_proposed_cap_does_not_sink_a_sphere_through_the_floor() {
    let scenario = HighSpeedSphereCcdScenario::new();

    for cap in [SHIPPED_CAP, PROPOSED_CAP] {
        let run = run_scenario(
            &AtFrameRate {
                inner: &scenario,
                frame_dt: MEASURED_FRAME_DT,
            },
            BenchRunConfig {
                duration: 6.0,
                max_substeps_per_frame: cap as usize,
                ..BenchRunConfig::default()
            },
        );

        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(
            min_y > -0.1,
            "sank through the floor: cap={cap} min_y={min_y}"
        );

        let last = run.samples.last().unwrap();
        assert!(
            last.y > 0.2,
            "should rest above ground: cap={cap} y={}",
            last.y
        );
    }
}
