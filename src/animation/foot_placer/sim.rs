//! Shared scenario simulator for trace and invariant tests. Drives
//! the placer with synthetic body motion over an analytic terrain
//! height field, recording one snapshot per display frame.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use nalgebra::{Point3, Vector2, Vector3};

use super::config::FootPlacerConfig;
use super::placer::{angle_diff, facing_from_yaw, FootPlacer, FootSide, PlacerCtx, PlacerFoot};

pub const HIP_WIDTH: f32 = 0.12;
pub const LEG_LENGTH: f32 = 0.5;
pub const STANDING_HEIGHT: f32 = 0.425;

/// The rig a scenario is walked on.
///
/// The placer is rig-independent by design — the player, the heart
/// critter and the peeper all walk on the same one — so the harness has
/// to be too. Anything measured on one set of proportions and not the
/// others is measuring the proportions.
#[derive(Clone, Copy, Debug)]
pub struct RigDims {
    pub hip_width: f32,
    pub leg_length: f32,
    pub standing_height: f32,
}

impl RigDims {
    /// The player's proportions, which every pre-existing scenario was
    /// written against.
    pub fn player() -> Self {
        Self {
            hip_width: HIP_WIDTH,
            leg_length: LEG_LENGTH,
            standing_height: STANDING_HEIGHT,
        }
    }

    /// A peeper: twice the leg, half the hip separation. Kept in step
    /// with `PeeperRigConfig::default` rather than guessed.
    pub fn peeper() -> Self {
        let config = crate::animation::peeper::PeeperRigConfig::default();
        let dims = config.leg_dims();
        Self {
            hip_width: dims.hip_width,
            leg_length: dims.leg_length,
            standing_height: dims.standing_height,
        }
    }
}

impl Default for RigDims {
    fn default() -> Self {
        Self::player()
    }
}

/// Per-gait knobs the animator sources from the active `GaitPreset`.
#[derive(Clone, Copy)]
pub struct GaitParams {
    pub stride_gain: f32,
    pub step_height: f32,
}

impl GaitParams {
    pub fn walk() -> Self {
        Self {
            stride_gain: 0.4,
            step_height: 0.15,
        }
    }

    /// A peeper's: a longer stride and a higher-lifted foot than the
    /// player's walk. Sourced from `PeeperRigConfig::default` rather than
    /// written out again.
    pub fn peeper() -> Self {
        let config = crate::animation::peeper::PeeperRigConfig::default();
        Self {
            stride_gain: config.stride_gain,
            step_height: config.step_height,
        }
    }

    pub fn crouch() -> Self {
        Self {
            stride_gain: 0.7,
            step_height: 0.06,
        }
    }
}

/// Per-frame body inputs supplied by a scenario. `intent` is the
/// player-requested direction (may be non-zero while velocity is
/// still zero — that's the anticipation case).
pub struct Input {
    pub velocity: Vector3<f32>,
    pub yaw: f32,
    pub intent: Vector3<f32>,
}

impl Input {
    /// Moving with intent matching velocity (the common case).
    pub fn moving(velocity: Vector3<f32>, yaw: f32) -> Self {
        Self {
            velocity,
            yaw,
            intent: velocity,
        }
    }

    pub fn still() -> Self {
        Self {
            velocity: Vector3::zeros(),
            yaw: 0.0,
            intent: Vector3::zeros(),
        }
    }
}

/// Snapshot of placer + body state at a frame boundary.
pub struct Frame {
    pub time: f32,
    pub pelvis: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub yaw: f32,
    pub gait_phase: f32,
    /// The cadence the placer planned this frame, which is what a
    /// measured gait has to be judged against.
    pub timing: Option<super::timing::GaitTiming>,
    pub left: PlacerFoot,
    pub right: PlacerFoot,
}

/// Run a scenario at a fixed display rate with walk-gait parameters.
pub fn simulate(
    frames: usize,
    fps: f32,
    height: impl Fn(f32, f32) -> f32,
    input: impl Fn(usize) -> Input,
) -> Vec<Frame> {
    simulate_gait(frames, fps, GaitParams::walk(), height, input)
}

/// Run a scenario at a fixed display rate with explicit gait parameters.
pub fn simulate_gait(
    frames: usize,
    fps: f32,
    gait: GaitParams,
    height: impl Fn(f32, f32) -> f32,
    input: impl Fn(usize) -> Input,
) -> Vec<Frame> {
    simulate_var_dt(frames, |_| 1.0 / fps, gait, height, input)
}

/// Run a scenario with a per-frame display dt (jittered frame times).
pub fn simulate_var_dt(
    frames: usize,
    dt_of: impl Fn(usize) -> f32,
    gait: GaitParams,
    height: impl Fn(f32, f32) -> f32,
    input: impl Fn(usize) -> Input,
) -> Vec<Frame> {
    simulate_with_suspend(frames, dt_of, gait, height, input, |_| false)
}

/// Run a scenario with per-frame dt and a per-frame placer suspension
/// flag — models airborne blips (terrain-seam contact loss, small hops)
/// where the animator suspends the placer for a frame or two.
/// Integrates the pelvis from per-frame velocity, derives per-foot
/// ground contacts / normals from `height`, ticks the placer, snapshots
/// each frame.
pub fn simulate_with_suspend(
    frames: usize,
    dt_of: impl Fn(usize) -> f32,
    gait: GaitParams,
    height: impl Fn(f32, f32) -> f32,
    input: impl Fn(usize) -> Input,
    suspend: impl Fn(usize) -> bool,
) -> Vec<Frame> {
    simulate_over(
        frames,
        dt_of,
        gait,
        |x, z| Some(height(x, z)),
        input,
        suspend,
    )
}

/// Run a scenario over terrain that need not be everywhere: `terrain`
/// returns `None` where a probe would find nothing — a pit, a ledge, the
/// far side of a gap. The pelvis is scripted, so it walks out over the
/// void; the feet may not follow it there.
pub fn simulate_over(
    frames: usize,
    dt_of: impl Fn(usize) -> f32,
    gait: GaitParams,
    terrain: impl Fn(f32, f32) -> Option<f32>,
    input: impl Fn(usize) -> Input,
    suspend: impl Fn(usize) -> bool,
) -> Vec<Frame> {
    simulate_rig(
        RigDims::player(),
        FootPlacerConfig::default(),
        frames,
        dt_of,
        gait,
        terrain,
        input,
        suspend,
    )
}

/// The full form: a scenario over arbitrary terrain, on an arbitrary rig,
/// under an arbitrary placer config. Everything above is this with the
/// player's proportions filled in.
#[allow(clippy::too_many_arguments)]
pub fn simulate_rig(
    dims: RigDims,
    cfg: FootPlacerConfig,
    frames: usize,
    dt_of: impl Fn(usize) -> f32,
    gait: GaitParams,
    terrain: impl Fn(f32, f32) -> Option<f32>,
    input: impl Fn(usize) -> Input,
    suspend: impl Fn(usize) -> bool,
) -> Vec<Frame> {
    let RigDims {
        hip_width,
        leg_length,
        standing_height,
    } = dims;

    let mut pelvis_xz = Vector2::new(0.0, 0.0);
    let init_yaw = input(0).yaw;
    let init_facing = facing_from_yaw(init_yaw);
    let foot_y0 = terrain(0.0, 0.0).expect("the scenario must start on ground");
    // The pelvis walks on regardless of what is under it — a capsule
    // resting on a ledge is still held up when its centre passes the
    // edge — so ground height falls back to the last surface it knew.
    let mut last_ground_y = foot_y0;
    let mut placer = FootPlacer::new(
        Point3::new(0.0, foot_y0 + standing_height, 0.0),
        init_facing,
        hip_width,
        foot_y0,
    );
    let mut last_yaw = init_yaw;
    let mut prev_pelvis_y = foot_y0 + standing_height;
    let mut time = 0.0;
    let mut out = Vec::with_capacity(frames);

    for f in 0..frames {
        let dt = dt_of(f);
        let Input {
            velocity,
            yaw,
            intent,
        } = input(f);
        let yaw_rate = angle_diff(yaw, last_yaw) / dt;
        last_yaw = yaw;
        time += dt;

        pelvis_xz.x += velocity.x * dt;
        pelvis_xz.y += velocity.z * dt;
        last_ground_y = terrain(pelvis_xz.x, pelvis_xz.y).unwrap_or(last_ground_y);
        let pelvis = Point3::new(pelvis_xz.x, last_ground_y + standing_height, pelvis_xz.y);
        // The pelvis follows the terrain, so the placer must see the
        // implied vertical velocity — the game's physics velocity has
        // it, and the overstretch opening gate reads it on slopes.
        let velocity = Vector3::new(velocity.x, (pelvis.y - prev_pelvis_y) / dt, velocity.z);
        prev_pelvis_y = pelvis.y;

        // Probes aim at each foot's anchor (landing target while
        // stepping), matching the game's probe configuration.
        let probe = |p: Point3<f32>| match terrain(p.x, p.z) {
            Some(y) => (
                Some(Point3::new(p.x, y, p.z)),
                normal_at(&terrain, p.x, p.z),
            ),
            None => (None, Vector3::y()),
        };
        let (left_ground, left_ground_normal) = probe(placer.left.probe_anchor());
        let (right_ground, right_ground_normal) = probe(placer.right.probe_anchor());

        placer.set_suspended(suspend(f));

        let ctx = PlacerCtx {
            dt,
            pelvis,
            velocity,
            // The offline sim walks on static ground.
            support_velocity: Vector3::zeros(),
            intent_direction: intent,
            yaw,
            yaw_rate,
            hip_width,
            leg_length,
            standing_height,
            foot_y_fallback: pelvis.y - standing_height,
            step_height: gait.step_height,
            stride_gain: gait.stride_gain,
            left_ground_normal,
            right_ground_normal,
            left_ground,
            right_ground,
            config: &cfg,
        };
        placer.tick(&ctx);

        out.push(Frame {
            time,
            pelvis,
            velocity,
            yaw,
            gait_phase: placer.gait_phase(),
            timing: placer.timing(),
            left: placer.left.clone(),
            right: placer.right.clone(),
        });
    }
    out
}

/// Terrain normal from central differences of the height field. Samples
/// that fall in a void hold the centre height, so the normal at a ledge
/// lip is the lip's own rather than a cliff face.
fn normal_at(terrain: &impl Fn(f32, f32) -> Option<f32>, x: f32, z: f32) -> Vector3<f32> {
    let e = 0.05;
    let Some(centre) = terrain(x, z) else {
        return Vector3::y();
    };
    let at = |x: f32, z: f32| terrain(x, z).unwrap_or(centre);
    let dx = (at(x + e, z) - at(x - e, z)) / (2.0 * e);
    let dz = (at(x, z + e) - at(x, z - e)) / (2.0 * e);
    Vector3::new(-dx, 1.0, -dz).normalize()
}

/// Takeoff events (side + frame index) extracted from a recording.
pub fn takeoffs(frames: &[Frame]) -> Vec<(FootSide, usize)> {
    let mut events = Vec::new();
    for (i, pair) in frames.windows(2).enumerate() {
        if pair[0].left.is_planted() && pair[1].left.is_stepping() {
            events.push((FootSide::Left, i + 1));
        }
        if pair[0].right.is_planted() && pair[1].right.is_stepping() {
            events.push((FootSide::Right, i + 1));
        }
    }
    events
}
