//! Runs a scenario and records what the animator did.
//!
//! ```text
//!   Script ─beat─▶ CharacterIntent ─┬─▶ Body ─pelvis, velocity, grounded─┐
//!                                   │                                    │
//!                                   │        Ground ─raycast─▶ probes ───┤
//!                                   │                                    ▼
//!                                   └──────────────────▶ CharacterAnimator::update
//!                                                                  │
//!                                                                  ▼
//!                                                              FrameSample
//! ```
//!
//! The animator is the real one, unmodified, driven through exactly the two
//! calls `AnimationProbeConfigSystem` and `CharacterAnimationSystem` make and
//! in the same order. Nothing here reimplements gait, foot placement or IK; if
//! the tool shows a limp, the limp is in the engine.

use nalgebra::{Point3, Vector3};

use crate::animation::humanoid::pose_state::{Gait, PoseState};
use crate::animation::{BodyReading, CharacterAnimator, CharacterRigConfig};
use crate::character::grab::GrabConfig;
use crate::character::{CharacterIntent, Grounding, Immersion, LocomotionConfig};
use crate::sensing::ContactCandidate;

use super::body::Body;
use super::ground::Ground;
use super::script::Script;
use super::support::{Carried, SupportMotion};
use super::take::{CapturedMesh, FootSample, FrameSample, Take, TimingSample};

/// Display rate the scenarios run at, matching the rate the game is locked to
/// by the display it runs on. The placer substeps and is frame-rate
/// independent, but the transitions around it are not — a landing or a takeoff
/// costs two to three times as much reach at 30 Hz as at 60 — so a harness
/// running faster than the game grades work that will not ship.
///
/// Fixed rather than jittered: a gait measured against a moving frame time is
/// measuring two things at once. Use `--fps` to vary it deliberately.
pub const FRAME_RATE: f32 = 30.0;

/// How often to capture a drawable character mesh.
#[derive(Clone, Copy, Debug)]
pub enum MeshCapture {
    /// Numbers only. Needs no GPU and no mesh generation, so a whole catalogue
    /// runs in well under a second.
    None,
    /// Keep the mesh from every `n`th frame, for the filmstrip.
    Every(usize),
}

/// Everything a run needs that is not the animator itself.
pub struct Run<'a> {
    pub name: String,
    pub ground: &'a dyn Ground,
    pub script: &'a Script,
    pub rig: CharacterRigConfig,
    pub locomotion: LocomotionConfig,
    /// Where on the ground the character starts, in x and z.
    pub start: (f32, f32),
    /// Height of the body's origin above the ground while supported. Defaults
    /// to the game's own — the physics capsule's resting centre — via
    /// `Body::ride_height`; overridable so the tool can answer "what would this
    /// gait look like if the body rode where the rig expects it to?".
    pub ride_height: f32,
    /// Display rate to run at. The placer substeps internally, so this is not
    /// meant to change what a gait does — which is exactly why it is worth
    /// being able to turn: a heavy scene is a slow scene, and a gait that only
    /// holds together at 60 Hz will come apart in the one place the game has
    /// the most to draw.
    pub frame_rate: f32,
    /// How the ground itself moves. `Still` for the static catalogue.
    pub support: SupportMotion,
    pub capture: MeshCapture,
}

/// Run one scenario to the end of its script.
pub fn run(spec: &Run<'_>) -> Take {
    let dt = 1.0 / spec.frame_rate.max(1.0);
    let grab_config = GrabConfig::default();
    let mut body = Body::standing_at(
        spec.ground,
        spec.start.0,
        spec.start.1,
        spec.ride_height,
        spec.locomotion.clone(),
    );
    // A character on a moving platform arrived there already travelling with
    // it. Starting at rest instead would open every such scenario with the
    // floor being yanked out from under a standing body, which is a different
    // thing to look at and not the one these runs are for.
    body.velocity = spec.support.velocity(0.0);
    // The animator is told how high the body's origin rides so it can put the
    // pelvis where the rig belongs — the same seam the player spawner wires to
    // the capsule's half height.
    let mut animator =
        CharacterAnimator::new(spec.rig.clone(), body.position, spec.ride_height, body.yaw);

    let mut frames = Vec::new();
    let mut time = 0.0;
    let mut index = 0;
    // Held so a beat that turns can rotate its direction continuously.
    let mut turn = 0.0f32;
    let mut previous_crouch = false;

    while let Some((beat, first_frame)) = spec.script.at(time, dt) {
        turn += beat.turn_rate * dt;
        let direction = rotate_y(beat.direction, turn);

        let intent = CharacterIntent {
            direction,
            jump: beat.jump && first_frame,
            jump_held: beat.jump,
            jump_released: false,
            crouch: beat.crouch,
            crouch_just_pressed: beat.crouch && !previous_crouch,
            sprint: beat.sprint,
            ..Default::default()
        };
        previous_crouch = beat.crouch;

        // Where the ground has got to this frame. On a still surface this is
        // the scenario's own ground, unshifted.
        let support_velocity = spec.support.velocity(time);
        let support_offset = spec.support.offset(time);
        let ground = Carried::new(spec.ground, support_offset);

        body.step(dt, &intent, &ground, support_velocity);

        let pelvis = animator.pelvis_for(body.position, Vector3::y());
        let grounding = if body.grounded {
            Grounding::on(ground.normal(body.position.x, body.position.z))
                .carried_by(support_velocity)
        } else {
            Grounding::airborne()
        };

        // Probe configuration and sensing, exactly as the two ECS systems
        // sequence them: the animator aims the probes, the world answers.
        let probes = animator.configure_probes(pelvis, body.yaw);
        let contacts: Vec<ContactCandidate> = probes
            .iter()
            .filter_map(|probe| {
                ground
                    .raycast(probe.origin, probe.direction, probe.length)
                    .map(|hit| ContactCandidate {
                        tag: probe.tag,
                        point: hit.point,
                        normal: hit.normal,
                        distance: hit.t * probe.length,
                    })
            })
            .collect();

        animator.update(
            dt,
            &BodyReading {
                pelvis,
                yaw: body.yaw,
                velocity: body.velocity,
                up: Vector3::y(),
                grounding: &grounding,
                immersion: &Immersion::dry(),
            },
            &body.state,
            &intent,
            &grab_config,
            &contacts,
        );

        let mesh = match spec.capture {
            MeshCapture::None => None,
            MeshCapture::Every(n) if n > 0 && index % n == 0 => {
                let (vertices, indices) = animator.mesh();
                Some(CapturedMesh {
                    vertices: vertices.to_vec(),
                    indices: indices.to_vec(),
                })
            }
            MeshCapture::Every(_) => None,
        };

        frames.push(sample(
            index,
            time,
            dt,
            beat.label,
            &animator,
            &body,
            support_velocity,
            support_offset,
            &contacts,
            mesh,
        ));

        time += dt;
        index += 1;
    }

    Take {
        scenario: spec.name.clone(),
        ground: spec.ground.name().to_string(),
        leg_length: spec.rig.leg_length(),
        standing_height: spec.rig.standing_height(),
        ride_height: spec.ride_height,
        frames,
    }
}

/// Snapshot one frame. Kept separate from the loop so the loop reads as the
/// system sequence it mirrors.
#[allow(clippy::too_many_arguments)]
fn sample(
    index: usize,
    time: f32,
    dt: f32,
    beat: &'static str,
    animator: &CharacterAnimator,
    body: &Body,
    support_velocity: Vector3<f32>,
    support_offset: Vector3<f32>,
    contacts: &[ContactCandidate],
    mesh: Option<CapturedMesh>,
) -> FrameSample {
    use crate::animation::probe_tags;

    let placer = animator.foot_placer();
    let skeleton = &animator.skeleton;

    let state = &animator.state;
    let mut left = FootSample::capture(
        &placer.left,
        skeleton.left_foot,
        state.left.position,
        skeleton.left_hip,
    );
    let mut right = FootSample::capture(
        &placer.right,
        skeleton.right_foot,
        state.right.position,
        skeleton.right_hip,
    );
    for contact in contacts {
        match contact.tag {
            probe_tags::FOOT_LEFT => left.probe = Some(contact.point),
            probe_tags::FOOT_RIGHT => right.probe = Some(contact.point),
            _ => {}
        }
    }

    FrameSample {
        index,
        time,
        dt,
        beat,
        pose: pose_label(&animator.pose_state),
        pelvis: animator.pelvis_for(body.position, Vector3::y()),
        velocity: body.velocity,
        yaw: body.yaw,
        grounded: body.grounded,
        support_velocity,
        support_offset,
        gait_phase: placer.gait_phase(),
        stride_phase: animator.state.stride_phase,
        stride_activity: animator.state.stride_activity,
        timing: placer.timing().map(TimingSample::from),
        left,
        right,
        mesh,
    }
}

/// A short stable name for a pose variant, for the report's state column.
fn pose_label(pose: &PoseState) -> &'static str {
    match pose {
        PoseState::Grounded { gait } => match gait {
            Gait::Idle => "idle",
            Gait::Walk => "walk",
            Gait::Sprint => "sprint",
            Gait::Crouch { walking: true } => "crouch-walk",
            Gait::Crouch { walking: false } => "crouch",
        },
        PoseState::Launching { .. } => "launch",
        PoseState::Airborne { .. } => "air",
        PoseState::Landing { .. } => "land",
        PoseState::Swimming { .. } => "swim",
    }
}

/// Rotate a horizontal direction about the Y axis.
fn rotate_y(direction: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let (sin, cos) = angle.sin_cos();
    Vector3::new(
        direction.x * cos + direction.z * sin,
        direction.y,
        -direction.x * sin + direction.z * cos,
    )
}

/// Where the character ends up, for framing a shot.
pub fn travel_bounds(take: &Take) -> (Point3<f32>, Point3<f32>) {
    let mut min = Point3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max = Point3::new(f32::MIN, f32::MIN, f32::MIN);
    for frame in &take.frames {
        for axis in 0..3 {
            min[axis] = min[axis].min(frame.pelvis[axis]);
            max[axis] = max[axis].max(frame.pelvis[axis]);
        }
    }
    (min, max)
}
