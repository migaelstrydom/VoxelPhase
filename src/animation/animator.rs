//! Character animation driver.
//!
//! This is the single source of truth for character animation.
//! All animation logic flows through this animator.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::{CharacterRigConfig, GaitPreset};
use super::foot_placer::{FootPlacer, PlacerCtx, PlacerFoot, PlacerRecorder};
use super::humanoid::pose_state::{AirKind, Gait, PoseState, SampleCtx, Takeoff, TickCtx};
use super::humanoid::skeleton::{generate_character_mesh, Skeleton, FOOT_HEIGHT};
use super::humanoid::stride_sync;
use super::humanoid::upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
use super::pose::{Crossfade, Linear, PoseFragment};
use super::state::{AnimationState, FootState};
use crate::character::grab::GrabConfig;
use crate::character::{AirSteering, ArmState, CharacterIntent, CharacterState, LocomotionState};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

/// Duration of the crossfade when either FSM changes variant kind.
const TRANSITION_BLEND_DURATION: f32 = 0.1;

/// Probe tags used by the character animator.
pub mod probe_tags {
    pub const FOOT_LEFT: u32 = 0;
    pub const FOOT_RIGHT: u32 = 1;
}

/// The character animation driver.
///
/// Owns all character-specific state and logic:
/// - Configures probes based on animation state
/// - Processes probe results
/// - Updates gait and foot positions
/// - Computes skeleton pose
#[derive(Component)]
#[storage(VecStorage)]
pub struct CharacterAnimator {
    pub config: CharacterRigConfig,
    pub state: AnimationState,
    pub skeleton: Skeleton,

    /// Current lower-body / core pose FSM variant.
    pub pose_state: PoseState,
    /// Current upper-body FSM variant.
    pub upper_state: UpperState,
    /// Active blend for the lower-body FSM. `Some` only during the
    /// short window after a variant-kind change; `None` between transitions.
    pose_crossfade: Option<Crossfade<Linear>>,
    /// Active blend for the upper-body FSM.
    upper_crossfade: Option<Crossfade<Linear>>,

    /// Procedural foot placer (Stage 1: ticked for debug/tuning only,
    /// not yet driving skeleton foot positions).
    pub foot_placer: FootPlacer,
    /// Previous-frame yaw, used to derive yaw rate for `FootPlacer`.
    last_yaw: f32,
    /// Optional input/output recorder for the foot placer (enabled via
    /// the `PLACER_REC` env var). Feeds the offline replay harness.
    recorder: Option<PlacerRecorder>,

    // Mesh caching
    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl CharacterAnimator {
    /// Create a new character animator at the given position.
    pub fn new(config: CharacterRigConfig, pelvis_position: Point3<f32>) -> Self {
        let leg_length = config.leg_length();
        let facing = Vector3::new(0.0, 0.0, 1.0);

        let state = AnimationState::new(pelvis_position, leg_length);
        let skeleton = Skeleton::new(&config, pelvis_position, facing);
        let foot_centre_y = pelvis_position.y - config.standing_height() - FOOT_HEIGHT;
        let foot_placer = FootPlacer::new(pelvis_position, facing, config.hip_width, foot_centre_y);
        let recorder =
            PlacerRecorder::from_env(pelvis_position, 0.0, config.hip_width, foot_centre_y);

        Self {
            config,
            state,
            skeleton,
            pose_state: PoseState::Grounded { gait: Gait::Idle },
            upper_state: UpperState::Swinging,
            pose_crossfade: None,
            upper_crossfade: None,
            foot_placer,
            last_yaw: 0.0,
            recorder,
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
        }
    }

    /// Configure probes for the next frame based on current animation state.
    pub fn configure_probes(&self, pelvis_position: Point3<f32>, yaw: f32) -> Vec<Probe> {
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
        let right = facing.cross(&Vector3::y());
        let left_hip_offset = -right * self.config.hip_width;
        let right_hip_offset = right * self.config.hip_width;

        let probe_length = self.config.probe_length();

        let mut probes = Vec::with_capacity(2);

        // Each foot's probe aims at the placer's probe anchor — the
        // committed landing target while stepping (so the swing reads
        // the terrain height where it will plant, not where the foot
        // currently hangs), the planted anchor otherwise (so planted
        // feet track the surface directly beneath them). While the
        // placer is suspended (airborne), aim at the visible foot from
        // the airborne sampler instead.
        let left_aim = if self.foot_placer.is_suspended() {
            self.state.left.position
        } else {
            self.foot_placer.left.probe_anchor()
        };
        probes.push(self.configure_foot_probe(
            probe_tags::FOOT_LEFT,
            pelvis_position + left_hip_offset,
            left_aim,
            probe_length,
        ));

        let right_aim = if self.foot_placer.is_suspended() {
            self.state.right.position
        } else {
            self.foot_placer.right.probe_anchor()
        };
        probes.push(self.configure_foot_probe(
            probe_tags::FOOT_RIGHT,
            pelvis_position + right_hip_offset,
            right_aim,
            probe_length,
        ));

        probes
    }

    /// Configure a single foot probe: a ray from the hip through the
    /// aim point. Falls back to straight down when the aim point is
    /// degenerate (at the hip itself).
    fn configure_foot_probe(
        &self,
        tag: u32,
        hip_position: Point3<f32>,
        aim: Point3<f32>,
        length: f32,
    ) -> Probe {
        let direction = (aim - hip_position)
            .try_normalize(1e-4)
            .unwrap_or_else(|| Vector3::new(0.0, -1.0, 0.0));

        Probe {
            tag,
            origin: hip_position,
            direction,
            length,
        }
    }

    /// Process probe results and update animation.
    pub fn update(
        &mut self,
        dt: f32,
        pelvis_position: Point3<f32>,
        yaw: f32,
        velocity: Vector3<f32>,
        is_grounded: bool,
        character_state: &CharacterState,
        target: &CharacterIntent,
        grab_config: &GrabConfig,
        contacts: &[ContactCandidate],
    ) {
        let speed = Vector3::new(velocity.x, 0.0, velocity.z).magnitude();
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());

        self.state.facing = facing;
        self.state.pelvis_position = pelvis_position;
        self.process_contacts(contacts);
        // Support is decided by the contacts the physics engine solved, not by
        // where the gait happened to aim a probe — see
        // `docs/TRACTION_DRIVE_DESIGN.md` §6.6. The probes below remain the
        // rangefinder: how high the ground is under a landing target, and
        // which way it faces.
        self.state.is_grounded = is_grounded;

        // Snapshot the foot-centre y for a potential Landing splice this
        // frame. `foot.position.y` now represents the foot centre (sole
        // is one radius below), so probe contact is the correct value
        // directly. Fallback estimates the terrain surface as
        // `pelvis - standing_height - FOOT_HEIGHT` — i.e. one full foot
        // below the pelvis-relative ankle height used by the rest rig.
        let landing_ground_y = self
            .state
            .left
            .ground_contact
            .or(self.state.right.ground_contact)
            .map(|c| c.y)
            .unwrap_or(pelvis_position.y - self.config.standing_height() - FOOT_HEIGHT);

        // Map CharacterState → next PoseState variant.
        let new_pose = next_pose_state(
            self.pose_state,
            character_state,
            target,
            speed,
            self.config.idle_threshold,
            facing,
            landing_ground_y,
        );

        // Tick the foot placer before sampling so the pose layer reads a
        // current foot position. Airborne states suspend the placer; feet
        // come from the airborne/landing samplers in those cases.
        self.tick_foot_placer(dt, pelvis_position, yaw, velocity, target, &new_pose);
        mirror_placer_into_state(&mut self.state, &self.foot_placer);

        // Stride phase is derived directly from the placer's stepping
        // state — arm swing / shoulder twist / head bob lock to real
        // foot events rather than a speed-driven wheel. At idle (both
        // feet planted) the phase holds, so arms coast instead of
        // snapping to rest.
        self.state.stride_phase =
            stride_sync::phase_from_placer(&self.foot_placer, self.state.stride_phase, dt);

        // Stride activity snaps to 1 while anything is stepping and
        // decays exponentially to 0 once both feet are planted. Upper
        // body amplitudes (arm swing, shoulder twist, head bob) are
        // scaled by it so the rest→swing→rest transition is continuous
        // — matters most on slope slides where gait can flicker between
        // Idle and Walk even though the body is clearly moving.
        let stepping = !self.foot_placer.left.is_planted() || !self.foot_placer.right.is_planted();
        if stepping {
            self.state.stride_activity = 1.0;
        } else {
            let decay_tc = 0.25;
            self.state.stride_activity *= (-dt / decay_tc).exp();
        }

        // Pick the upper variant alongside the new pose variant so Idle-vs-
        // airborne gating sees the just-computed pose.
        let new_upper = map_upper_state(&character_state.arm, new_pose);

        let pose_sample_ctx = SampleCtx {
            rig: &self.config,
            anim: &self.state,
            velocity,
        };

        // Snapshot outgoing fragments BEFORE the variant swap, so the
        // crossfade `from` reflects what the old state was producing.
        begin_crossfade_if_changed(
            &mut self.pose_crossfade,
            self.pose_state.transition_key(),
            new_pose.transition_key(),
            pelvis_position,
            || self.pose_state.sample(&pose_sample_ctx),
        );
        begin_crossfade_if_changed(
            &mut self.upper_crossfade,
            self.upper_state.transition_key(),
            new_upper.transition_key(),
            pelvis_position,
            || {
                let ctx = build_upper_ctx(&self.pose_state, &self.config, &self.state, grab_config);
                self.upper_state.sample(&ctx)
            },
        );

        // Swap in new variants, then tick.
        self.pose_state = new_pose;
        self.upper_state = new_upper;

        let pose_tick_ctx = TickCtx {
            dt,
            velocity,
            horizontal_speed: speed,
            rig: &self.config,
        };
        self.pose_state = self.pose_state.tick(&pose_tick_ctx);
        self.upper_state = self.upper_state.tick(&UpperTickCtx { dt });

        // Sample the (now current) states.
        let to_pose = self.pose_state.sample(&pose_sample_ctx);
        let upper_sample_ctx =
            build_upper_ctx(&self.pose_state, &self.config, &self.state, grab_config);
        let to_upper = self.upper_state.sample(&upper_sample_ctx);

        // Apply any running crossfades. `is_active` is checked AFTER ticking
        // so the final frame of a blend lands on weight=1.0 cleanly.
        let pose_fragment = blend_through(&mut self.pose_crossfade, to_pose, pelvis_position, dt);
        let upper_fragment =
            blend_through(&mut self.upper_crossfade, to_upper, pelvis_position, dt);

        let fragment = pose_fragment.compose(&upper_fragment);
        self.apply_fragment_to_state(&fragment);
        self.skeleton
            .apply_fragment(&fragment, &self.state, &self.config);
    }

    /// Advance the procedural foot placer. Runs before pose sampling;
    /// its output is mirrored into `AnimationState` and read by the
    /// grounded pose samplers. Suspends while airborne so feet don't
    /// step against a body that isn't on the ground.
    fn tick_foot_placer(
        &mut self,
        dt: f32,
        pelvis_position: Point3<f32>,
        yaw: f32,
        velocity: Vector3<f32>,
        target: &CharacterIntent,
        next_pose: &PoseState,
    ) {
        let yaw_rate = if dt > 0.0 {
            shortest_angle_diff(yaw, self.last_yaw) / dt
        } else {
            0.0
        };
        self.last_yaw = yaw;

        let airborne = matches!(
            next_pose,
            PoseState::Launching { .. } | PoseState::Airborne { .. }
        );
        self.foot_placer.set_suspended(airborne);

        let preset = gait_preset_for(next_pose, &self.config);
        let step_height = preset
            .map(|p| p.step_height)
            .unwrap_or(self.config.step_height);
        // `stride_gain` scales the capture-point target: 1.0 plants at
        // the stopping foothold, values below 1 let the body pass over
        // the foot. Airborne and landing states fall back to the walk
        // preset — any nonzero value is fine since the placer is
        // suspended during Launching/Airborne.
        let stride_gain = preset
            .map(|p| p.stride_gain)
            .unwrap_or(self.config.gait_presets.walk.stride_gain);
        // Fallback foot-centre y used by the placer when no probe hit is
        // available. Matches the rest-rig terrain level: one full foot
        // below the pelvis-relative ankle height, so the sole sits at
        // pelvis-standing_height-(3/2)·FOOT_HEIGHT — i.e. submerged by a
        // radius as intended.
        let foot_centre_y = pelvis_position.y - self.config.standing_height() - FOOT_HEIGHT;
        let left_ground_normal = self.state.left.ground_normal.unwrap_or_else(Vector3::y);
        let right_ground_normal = self.state.right.ground_normal.unwrap_or_else(Vector3::y);
        let left_ground = self.state.left.ground_contact;
        let right_ground = self.state.right.ground_contact;

        let ctx = PlacerCtx {
            dt,
            pelvis: pelvis_position,
            velocity,
            intent_direction: target.direction,
            yaw,
            yaw_rate,
            hip_width: self.config.hip_width,
            leg_length: self.config.leg_length(),
            standing_height: self.config.standing_height(),
            foot_y_fallback: foot_centre_y,
            step_height,
            stride_gain,
            left_ground_normal,
            right_ground_normal,
            left_ground,
            right_ground,
            config: &self.config.foot_placer,
        };
        self.foot_placer.tick(&ctx);

        if let Some(recorder) = &mut self.recorder {
            recorder.record(airborne, pose_tag(next_pose), &ctx, &self.foot_placer);
        }
    }

    /// Mirror fragment channels into `AnimationState` so next-frame probes
    /// and any external readers see a coherent snapshot.
    fn apply_fragment_to_state(&mut self, fragment: &PoseFragment) {
        if let Some(feet) = &fragment.feet {
            self.state.left.position = feet.left;
            self.state.right.position = feet.right;
        }
        if let Some(hands) = &fragment.hands {
            self.state.left_hand.position = hands.left;
            self.state.right_hand.position = hands.right;
        }
        if let Some(twist) = fragment.shoulder_twist {
            self.state.shoulder_twist = twist;
        }
        if let Some(tilt) = fragment.head_tilt {
            self.state.head_tilt = tilt;
        }
        if let Some(bob) = fragment.head_bob {
            self.state.head_bob = bob;
        }
    }

    /// Process contact candidates from probes.
    fn process_contacts(&mut self, contacts: &[ContactCandidate]) {
        // Clear previous contacts
        self.state.left.ground_contact = None;
        self.state.left.ground_normal = None;
        self.state.right.ground_contact = None;
        self.state.right.ground_normal = None;

        // Find best contact for each foot (closest)
        let mut best_left: Option<&ContactCandidate> = None;
        let mut best_right: Option<&ContactCandidate> = None;

        for contact in contacts {
            match contact.tag {
                probe_tags::FOOT_LEFT => {
                    if best_left.map_or(true, |b| contact.distance < b.distance) {
                        best_left = Some(contact);
                    }
                }
                probe_tags::FOOT_RIGHT => {
                    if best_right.map_or(true, |b| contact.distance < b.distance) {
                        best_right = Some(contact);
                    }
                }
                _ => {}
            }
        }

        // Store contacts in state. `normal` mirrors `ground_normal` so
        // downstream readers see a coherent snapshot.
        if let Some(contact) = best_left {
            self.state.left.ground_contact = Some(contact.point);
            self.state.left.ground_normal = Some(contact.normal);
            self.state.left.normal = contact.normal;
        }
        if let Some(contact) = best_right {
            self.state.right.ground_contact = Some(contact.point);
            self.state.right.ground_normal = Some(contact.normal);
            self.state.right.normal = contact.normal;
        }
    }

    /// Whether something is holding the character up, as the Support Set
    /// resolved it this frame.
    pub fn is_grounded(&self) -> bool {
        self.state.is_grounded
    }

    /// Get mesh vertices and indices for rendering.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        self.regenerate_mesh();
        (&self.cached_vertices, &self.cached_indices)
    }

    /// Regenerate the mesh from current skeleton state.
    fn regenerate_mesh(&mut self) {
        let (vertices, indices) = generate_character_mesh(&self.skeleton, &self.config);
        self.cached_vertices = vertices;
        self.cached_indices = indices;
    }
}

/// Open a fresh crossfade into a new FSM variant when the transition key
/// has changed. The `sample_from` closure produces the outgoing fragment
/// — it is only evaluated when a crossfade is actually needed.
fn begin_crossfade_if_changed<K: PartialEq, F: FnOnce() -> PoseFragment>(
    slot: &mut Option<Crossfade<Linear>>,
    old_key: K,
    new_key: K,
    pelvis: Point3<f32>,
    sample_from: F,
) {
    if old_key != new_key {
        *slot = Some(Crossfade {
            from: sample_from(),
            from_pelvis: pelvis,
            to_duration: TRANSITION_BLEND_DURATION,
            elapsed: 0.0,
            policy: Linear,
        });
    }
}

/// Build the `UpperSampleCtx` derived from the current `PoseState` — the
/// cycle, preset, pelvis offset, torso pitch and airborne context the
/// upper body should track this frame.
fn build_upper_ctx<'a>(
    pose: &PoseState,
    rig: &'a CharacterRigConfig,
    anim: &'a AnimationState,
    grab: &'a GrabConfig,
) -> UpperSampleCtx<'a> {
    let preset = gait_preset_for(pose, rig);
    let (pelvis_offset, torso_pitch) = upper_torso_inputs(preset);
    UpperSampleCtx {
        rig,
        anim,
        grab,
        cycle: pose.cycle(anim),
        preset,
        pelvis_offset,
        torso_pitch,
        airborne: airborne_ctx_for(pose),
    }
}

/// Shortest signed angle difference `b - a`, wrapped into `(-PI, PI]`.
#[inline]
fn shortest_angle_diff(b: f32, a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut d = (b - a) % two_pi;
    if d > std::f32::consts::PI {
        d -= two_pi;
    } else if d < -std::f32::consts::PI {
        d += two_pi;
    }
    d
}

/// Advance a crossfade (if any) and return the fragment the driver should
/// use this frame. Clears the slot once the blend has fully resolved.
fn blend_through(
    slot: &mut Option<Crossfade<Linear>>,
    to: PoseFragment,
    current_pelvis: Point3<f32>,
    dt: f32,
) -> PoseFragment {
    match slot.as_mut() {
        Some(cf) => {
            let blended = cf.sample(&to, current_pelvis);
            cf.tick(dt);
            if !cf.is_active() {
                *slot = None;
            }
            blended
        }
        None => to,
    }
}

/// Look up the active `GaitPreset` for the current `PoseState`. Returns
/// `None` when the lower body is not Grounded — the upper body falls back
/// to rig-level defaults in that case.
/// Short comma-free tag identifying the pose FSM variant, for the placer
/// input recorder.
fn pose_tag(pose: &PoseState) -> &'static str {
    match pose {
        PoseState::Grounded { gait } => match gait {
            Gait::Idle => "idle",
            Gait::Walk => "walk",
            Gait::Sprint => "sprint",
            Gait::Crouch { walking: false } => "crouch",
            Gait::Crouch { walking: true } => "crouch_walk",
        },
        PoseState::Launching { .. } => "launching",
        PoseState::Airborne { .. } => "airborne",
        PoseState::Landing { .. } => "landing",
    }
}

fn gait_preset_for(pose: &PoseState, rig: &CharacterRigConfig) -> Option<GaitPreset> {
    match pose {
        PoseState::Grounded { gait } => Some(rig.gait_presets.for_gait(*gait)),
        _ => None,
    }
}

/// Pelvis offset and torso pitch that `UpperState` should apply this frame.
/// Mirrors the values `PoseState::Grounded::sample` emits into the pose
/// fragment so the upper body tracks a crouched/pitched torso.
fn upper_torso_inputs(preset: Option<GaitPreset>) -> (Vector3<f32>, f32) {
    let pelvis_offset = preset
        .map(|p| Vector3::new(0.0, -p.pelvis_crouch_offset, 0.0))
        .unwrap_or_else(Vector3::zeros);
    let torso_pitch = preset.map(|p| p.torso_pitch).unwrap_or(0.0);
    (pelvis_offset, torso_pitch)
}

/// Airborne context (kind + latched takeoff) to expose to `UpperState`,
/// when the lower body is mid-air. `Launching` and `Airborne` both count
/// — both reflect a committed takeoff that Braced should read.
fn airborne_ctx_for(pose: &PoseState) -> Option<(AirKind, Takeoff)> {
    match pose {
        PoseState::Launching { kind, takeoff, .. } | PoseState::Airborne { kind, takeoff } => {
            Some((*kind, *takeoff))
        }
        _ => None,
    }
}

/// Copy the placer's current per-foot position into `AnimationState` so
/// the pose layer samples coherent foot channels this frame. Planted
/// position mirrors too, primarily for debug overlays that still read it.
fn mirror_placer_into_state(state: &mut AnimationState, placer: &FootPlacer) {
    copy_placer_foot(&mut state.left, &placer.left);
    copy_placer_foot(&mut state.right, &placer.right);
}

fn copy_placer_foot(foot: &mut FootState, placer: &PlacerFoot) {
    foot.position = placer.position;
    foot.planted_position = placer.planted_position;
    foot.up = placer.up;
    foot.forward = placer.forward;
}

/// Map `ArmState` (+ current `PoseState`) to an `UpperState` variant.
/// `Idle + grounded` → `Swinging`; anything else with `Idle` (airborne,
/// launching, landing) → `Braced`. `Reaching`/`Holding` mirror their
/// `ArmState` shape.
fn map_upper_state(arm: &ArmState, pose: PoseState) -> UpperState {
    match arm {
        ArmState::Idle => match pose {
            PoseState::Grounded { .. } => UpperState::Swinging,
            _ => UpperState::Braced,
        },
        ArmState::Reaching { elapsed, target } => UpperState::Reaching {
            elapsed: *elapsed,
            target: *target,
        },
        ArmState::Holding {
            target_body,
            constraint,
            current_hold_height,
        } => UpperState::Holding {
            target_body: *target_body,
            constraint: *constraint,
            current_hold_height: *current_hold_height,
        },
    }
}

/// Decide the next `PoseState`, honouring in-flight Launching/Landing
/// timers so anticipation/follow-through play through to completion even
/// if the underlying player state moves on. `landing_ground_y` is the
/// impact y baked into any fresh `Landing` splice.
fn next_pose_state(
    current: PoseState,
    character: &CharacterState,
    target: &CharacterIntent,
    speed: f32,
    idle_threshold: f32,
    facing: Vector3<f32>,
    landing_ground_y: f32,
) -> PoseState {
    // Launching: hold for the full anticipation window unless physics
    // reports an early touchdown (rare — e.g. hit ceiling, dropped back).
    if let PoseState::Launching { kind, takeoff, t } = current {
        if matches!(character.locomotion, LocomotionState::Grounded) {
            return PoseState::Landing {
                kind,
                t: 0.0,
                ground_y: landing_ground_y,
            };
        }
        if !current.timer_expired() {
            return PoseState::Launching { kind, takeoff, t };
        }
        // Timer expired: fall through to the player-driven mapping (which
        // will typically observe Airborne and produce Airborne { kind }).
    }

    // Landing: hold for the full follow-through window. No early exits —
    // even if the player input changes, we keep squashing.
    if let PoseState::Landing { kind, t, ground_y } = current {
        if !current.timer_expired() {
            return PoseState::Landing { kind, t, ground_y };
        }
    }

    let moving = speed > idle_threshold || target.direction.magnitude_squared() > 0.001;

    let mapped = match character.locomotion {
        // CoyoteTime exists to bridge one-frame ground-contact losses
        // (seams, lips). Treat it as grounded: flicking to Airborne here
        // would suspend the foot placer and splice a spurious Landing on
        // every blip. The fall pose starts only when the grace period
        // genuinely expires into Airborne.
        LocomotionState::Grounded | LocomotionState::CoyoteTime(_) => {
            let gait = if target.crouch {
                Gait::Crouch { walking: moving }
            } else if !moving {
                Gait::Idle
            } else if target.sprint {
                Gait::Sprint
            } else {
                Gait::Walk
            };
            PoseState::Grounded { gait }
        }
        LocomotionState::Launching { steering, .. } => {
            let kind = match steering {
                AirSteering::Locked { .. } => AirKind::LongJump,
                AirSteering::Responsive => AirKind::Jump,
            };
            let takeoff = Takeoff {
                facing,
                air_speed: character.air_speed,
            };
            PoseState::Launching {
                kind,
                takeoff,
                t: 0.0,
            }
        }
        LocomotionState::Airborne { steering, .. } => {
            let kind = match (current, steering) {
                (PoseState::Launching { kind, .. }, _) => kind,
                (PoseState::Airborne { kind, .. }, _) => kind,
                (_, AirSteering::Locked { .. }) => AirKind::LongJump,
                (_, AirSteering::Responsive) => AirKind::Fall,
            };
            let takeoff = match current {
                PoseState::Launching { takeoff, .. } | PoseState::Airborne { takeoff, .. } => {
                    takeoff
                }
                _ => Takeoff {
                    facing,
                    air_speed: character.air_speed,
                },
            };
            PoseState::Airborne { kind, takeoff }
        }
    };

    // Splice Landing on the Airborne-ish → Grounded edge. Uses the prior
    // airborne `kind` so a LongJump ends in a heavy squash.
    if let PoseState::Grounded { .. } = mapped {
        let prev_air_kind = match current {
            PoseState::Airborne { kind, .. } | PoseState::Launching { kind, .. } => Some(kind),
            _ => None,
        };
        if let Some(kind) = prev_air_kind {
            return PoseState::Landing {
                kind,
                t: 0.0,
                ground_y: landing_ground_y,
            };
        }
    }

    mapped
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One-frame ground-contact losses (seams) put locomotion in
    /// CoyoteTime; the pose must stay Grounded — flicking to Airborne
    /// suspends the placer and splices a spurious Landing on return.
    #[test]
    fn coyote_time_keeps_grounded_pose() {
        let mut character = CharacterState::default();
        character.locomotion = LocomotionState::CoyoteTime(0.1);
        let mut target = CharacterIntent::default();
        target.direction = Vector3::new(0.0, 0.0, 1.0);

        let next = next_pose_state(
            PoseState::Grounded { gait: Gait::Walk },
            &character,
            &target,
            2.0,
            0.1,
            Vector3::new(0.0, 0.0, 1.0),
            0.0,
        );

        assert!(matches!(next, PoseState::Grounded { gait: Gait::Walk }));
    }
}
