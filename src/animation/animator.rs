//! Character animation driver.
//!
//! This is the single source of truth for character animation.
//! All animation logic flows through this animator.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::{CharacterRigConfig, GaitPreset};
use super::humanoid::pose_state::{AirKind, Gait, PoseState, SampleCtx, Takeoff, TickCtx};
use super::humanoid::skeleton::{generate_character_mesh, Skeleton, FOOT_HEIGHT};
use super::humanoid::stride_wheel;
use super::humanoid::upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
use super::pose::{Crossfade, Linear, PoseFragment};
use super::state::{AnimationState, FootState};
use crate::player::grab::GrabConfig;
use crate::player::{AirSteering, ArmState, LocomotionState, PlayerState, PlayerTargetState};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

/// Minimum movement (metres) required before an idle foot replants.
/// Keeps planting stable against probe jitter.
const IDLE_PLANT_SNAP: f32 = 0.03;

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
    /// Previous-frame `PlayerState.locomotion`, kept so the driver can
    /// detect variant edges (airborne→grounded, grounded→launching) for
    /// Landing/Launching splicing.
    last_locomotion: LocomotionState,

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

        Self {
            config,
            state,
            skeleton,
            pose_state: PoseState::Grounded { gait: Gait::Idle },
            upper_state: UpperState::Swinging,
            pose_crossfade: None,
            upper_crossfade: None,
            last_locomotion: LocomotionState::Grounded,
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

        // Left foot probe
        let left_probe = self.configure_foot_probe(
            probe_tags::FOOT_LEFT,
            pelvis_position + left_hip_offset,
            &self.state.left,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::LEFT_PHASE),
            probe_length,
        );
        probes.push(left_probe);

        // Right foot probe
        let right_probe = self.configure_foot_probe(
            probe_tags::FOOT_RIGHT,
            pelvis_position + right_hip_offset,
            &self.state.right,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::RIGHT_PHASE),
            probe_length,
        );
        probes.push(right_probe);

        probes
    }

    /// Configure a single foot probe based on foot state.
    fn configure_foot_probe(
        &self,
        tag: u32,
        hip_position: Point3<f32>,
        foot: &super::state::FootState,
        is_swinging: bool,
        length: f32,
    ) -> Probe {
        let (origin, direction) = if is_swinging {
            // Probe ahead toward target with downward bias
            let to_target = foot.position - hip_position;
            (hip_position, to_target.normalize())
        } else {
            // Probe straight down from hip position
            (hip_position, Vector3::new(0.0, -1.0, 0.0))
        };

        Probe {
            tag,
            origin,
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
        player_state: &PlayerState,
        target: &PlayerTargetState,
        grab_config: &GrabConfig,
        contacts: &[ContactCandidate],
    ) {
        let speed = Vector3::new(velocity.x, 0.0, velocity.z).magnitude();
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());

        self.state.facing = facing;
        self.state.pelvis_position = pelvis_position;
        // Feet always point along the body facing. Per-foot swing yaw is
        // future polish.
        self.state.left.forward = facing;
        self.state.right.forward = facing;

        self.process_contacts(contacts);
        let has_ground_contact =
            self.state.left.ground_contact.is_some() || self.state.right.ground_contact.is_some();
        self.state.is_grounded = has_ground_contact;

        // Snapshot the ankle y for a potential Landing splice this frame.
        // Probe contact is a surface point; lift by `FOOT_HEIGHT` so the
        // stored value is ankle-y (matches the convention used by every
        // other sample path). Fallback is already ankle-y.
        let landing_ground_y = self
            .state
            .left
            .ground_contact
            .or(self.state.right.ground_contact)
            .map(|c| c.y + FOOT_HEIGHT)
            .unwrap_or(pelvis_position.y - self.config.standing_height());

        // Map PlayerState → next PoseState variant.
        let new_pose = next_pose_state(
            self.pose_state,
            player_state,
            target,
            speed,
            self.config.idle_threshold,
            facing,
            landing_ground_y,
        );

        // On transition INTO an idle-like gait from a non-idle pose,
        // anchor the planted targets directly beneath the current hips.
        // Launching tracks the pelvis each frame; Landing carries its own
        // ground_y — neither needs `planted_position`.
        if is_idle_gait(&new_pose) && !is_idle_gait(&self.pose_state) {
            anchor_feet_under_hips(&mut self.state, &self.config, pelvis_position, facing);
        }

        // Re-plant feet when idle. Hysteresis prevents per-frame jitter
        // driven by probe noise. Ankle y is pelvis-relative (matches the
        // anchor formula) so the rendered foot bottom sits on the same
        // terrain line as the walking stride.
        if is_idle_gait(&new_pose) {
            let ankle_y = pelvis_position.y - self.config.standing_height();
            replant_foot(&mut self.state.left, IDLE_PLANT_SNAP, ankle_y);
            replant_foot(&mut self.state.right, IDLE_PLANT_SNAP, ankle_y);
        }

        // Advance stride wheel only when a gait cycle is actually playing.
        let running_stride =
            matches!(new_pose, PoseState::Grounded { .. }) && !is_idle_gait(&new_pose);
        if running_stride {
            stride_wheel::advance_wheel(
                &mut self.state.wheel_angle,
                speed,
                dt,
                self.config.body_radius,
            );
        }

        // Pick the upper variant alongside the new pose variant so Idle-vs-
        // airborne gating sees the just-computed pose.
        let new_upper = map_upper_state(&player_state.arm, new_pose);

        let pose_sample_ctx = SampleCtx {
            rig: &self.config,
            anim: &self.state,
            velocity,
        };

        // Snapshot outgoing fragments BEFORE the variant swap, so the
        // crossfade `from` reflects what the old state was producing.
        if self.pose_state.transition_key() != new_pose.transition_key() {
            let from = self.pose_state.sample(&pose_sample_ctx);
            self.pose_crossfade = Some(Crossfade {
                from,
                from_pelvis: pelvis_position,
                to_duration: TRANSITION_BLEND_DURATION,
                elapsed: 0.0,
                policy: Linear,
            });
        }
        let old_cycle = self.pose_state.cycle(&self.state);
        let old_preset = gait_preset_for(&self.pose_state, &self.config);
        let (old_pelvis_offset, old_torso_pitch) = upper_torso_inputs(old_preset);
        let old_airborne = airborne_ctx_for(&self.pose_state);
        if self.upper_state.transition_key() != new_upper.transition_key() {
            let old_upper_ctx = UpperSampleCtx {
                rig: &self.config,
                anim: &self.state,
                grab: grab_config,
                cycle: old_cycle,
                preset: old_preset,
                pelvis_offset: old_pelvis_offset,
                torso_pitch: old_torso_pitch,
                airborne: old_airborne,
            };
            let from = self.upper_state.sample(&old_upper_ctx);
            self.upper_crossfade = Some(Crossfade {
                from,
                from_pelvis: pelvis_position,
                to_duration: TRANSITION_BLEND_DURATION,
                elapsed: 0.0,
                policy: Linear,
            });
        }

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
        let cycle = self.pose_state.cycle(&self.state);
        let preset = gait_preset_for(&self.pose_state, &self.config);
        let (pelvis_offset, torso_pitch) = upper_torso_inputs(preset);
        let airborne = airborne_ctx_for(&self.pose_state);
        let upper_sample_ctx = UpperSampleCtx {
            rig: &self.config,
            anim: &self.state,
            grab: grab_config,
            cycle,
            preset,
            pelvis_offset,
            torso_pitch,
            airborne,
        };
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

        self.last_locomotion = player_state.locomotion;
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

        if let Some(normal) = self.state.left.ground_normal {
            self.state.left.normal = normal;
        }
        if let Some(normal) = self.state.right.ground_normal {
            self.state.right.normal = normal;
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

        // Store contacts in state
        if let Some(contact) = best_left {
            self.state.left.ground_contact = Some(contact.point);
            self.state.left.ground_normal = Some(contact.normal);
        }
        if let Some(contact) = best_right {
            self.state.right.ground_contact = Some(contact.point);
            self.state.right.ground_normal = Some(contact.normal);
        }
    }

    /// Whether the character has any ground contact.
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

/// Whether a pose variant is an idle-like gait — no stride cycle, feet
/// planted. Used to trigger foot-replant on transitions INTO such a gait.
fn is_idle_gait(pose: &PoseState) -> bool {
    matches!(
        pose,
        PoseState::Grounded {
            gait: Gait::Idle | Gait::Crouch { walking: false },
        }
    )
}

/// Anchor both feet's `planted_position` directly beneath the current
/// hips. x/z come from the hip; y uses the same pelvis-relative formula
/// as the walking stride (`pelvis.y - standing_height`) so idle and
/// running share the ankle convention by construction. Probe
/// `ground_contact.y` is intentionally NOT used for vertical positioning
/// — the probe point isn't always the visible terrain surface.
fn anchor_feet_under_hips(
    state: &mut AnimationState,
    rig: &CharacterRigConfig,
    pelvis_position: Point3<f32>,
    facing: Vector3<f32>,
) {
    let right = facing.cross(&Vector3::y());
    let ankle_y = pelvis_position.y - rig.standing_height();

    let left_hip = pelvis_position - right * rig.hip_width;
    let right_hip = pelvis_position + right * rig.hip_width;

    let left = Point3::new(left_hip.x, ankle_y, left_hip.z);
    let right_foot = Point3::new(right_hip.x, ankle_y, right_hip.z);

    state.left.planted_position = left;
    state.left.position = left;
    state.right.planted_position = right_foot;
    state.right.position = right_foot;
}

/// Replant an idle foot at its latest ground contact if the contact has
/// shifted beyond `snap_threshold`. Smaller shifts are ignored to avoid
/// per-frame jitter from probe noise. Vertical y is supplied by the
/// caller (pelvis-relative ankle y) — we do NOT trust the probe's y,
/// since the probe hit point isn't always the visible terrain surface.
fn replant_foot(foot: &mut FootState, snap_threshold: f32, ankle_y: f32) {
    let (desired_xz, _) = foot
        .ground_contact
        .map(|c| ((c.x, c.z), true))
        .unwrap_or(((foot.position.x, foot.position.z), false));
    let desired = Point3::new(desired_xz.0, ankle_y, desired_xz.1);
    if (desired - foot.planted_position).magnitude() > snap_threshold {
        foot.planted_position = desired;
    }
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
    player: &PlayerState,
    target: &PlayerTargetState,
    speed: f32,
    idle_threshold: f32,
    facing: Vector3<f32>,
    landing_ground_y: f32,
) -> PoseState {
    // Launching: hold for the full anticipation window unless physics
    // reports an early touchdown (rare — e.g. hit ceiling, dropped back).
    if let PoseState::Launching { kind, takeoff, t } = current {
        if matches!(player.locomotion, LocomotionState::Grounded) {
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

    let mapped = match player.locomotion {
        LocomotionState::Grounded => {
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
                air_speed: player.air_speed,
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
                    air_speed: player.air_speed,
                },
            };
            PoseState::Airborne { kind, takeoff }
        }
        LocomotionState::CoyoteTime(_) => {
            let takeoff = Takeoff {
                facing,
                air_speed: player.air_speed,
            };
            PoseState::Airborne {
                kind: AirKind::Fall,
                takeoff,
            }
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
