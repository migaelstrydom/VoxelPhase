//! Grab, drop, and throw helpers for the player arm/action system.
//!
//! Called from `PlayerControlSystem` — not a standalone ECS system.

use nalgebra::{Point3, Vector3};

use crate::aim::Launch;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyHandle};

use super::components::ArmState;

/// Configuration for grab mechanics. ECS resource (single player).
#[derive(Debug, Clone)]
pub struct GrabConfig {
    /// Maximum distance for the grab probe (world units).
    pub grab_range: f32,
    /// Distance in front of the player to hold the object.
    pub hold_distance: f32,
    /// Height offset above pelvis for the hold point.
    pub hold_height: f32,
    /// Constraint compliance (0 = rigid, >0 = springy).
    pub compliance: f32,
    /// Maximum constraint force (prevents absurd accelerations).
    pub max_force: f32,
    /// Softness of the angular (orientation-lock) constraint.
    pub angular_compliance: f32,
    /// Maximum angular impulse per axis per substep. Controls how much
    /// torque the grab can exert — light objects lock orientation, heavy
    /// objects droop under gravity.
    pub angular_max_impulse: f32,
    /// Duration of the reaching animation (seconds).
    pub reach_duration: f32,
    /// Speed at which the hold point rises from grab height to hold height
    /// (units/s). The anchor only rises as fast as the object can follow,
    /// so heavy objects lift slowly or not at all.
    pub lift_speed: f32,
    /// Forward impulse magnitude for throwing.
    pub throw_impulse: f32,
    /// Show debug overlays for probe ray, hold point, etc.
    pub debug_draw: bool,
}

impl Default for GrabConfig {
    fn default() -> Self {
        Self {
            grab_range: 2.0,
            hold_distance: 0.5,
            hold_height: 0.3,
            compliance: 0.0,
            max_force: 1000.0,
            angular_compliance: 0.0,
            angular_max_impulse: 500.0,
            lift_speed: 0.5,
            reach_duration: 0.15,
            throw_impulse: 1000.0,
            debug_draw: false,
        }
    }
}

/// Compute the player-body-local hold point offset from the center of mass.
///
/// The hold point is at `hold_distance` forward along the body's local Z axis
/// and `hold_height` upward along local Y. Since the player's turning is
/// physics-driven, the constraint target moves smoothly through the solver.
pub fn hold_point_local_anchor(config: &GrabConfig) -> Vector3<f32> {
    Vector3::new(0.0, config.hold_height, config.hold_distance)
}

/// Compute the desired hold point in world space (for animation / debug viz).
pub fn desired_hold_point(
    player_pos: Point3<f32>,
    facing: Vector3<f32>,
    config: &GrabConfig,
) -> Point3<f32> {
    player_pos + facing * config.hold_distance + Vector3::y() * config.hold_height
}

/// Attempt to initiate a grab: probe for a body in range.
///
/// Returns the new `ArmState::Reaching` with the probe result.
pub fn begin_reach(
    physics: &PhysicsWorld,
    player_pos: Point3<f32>,
    facing: Vector3<f32>,
    player_body: RigidBodyHandle,
    config: &GrabConfig,
) -> ArmState {
    let probe_hit = physics.probe_bodies(player_pos, facing, config.grab_range, &[player_body]);
    ArmState::Reaching {
        elapsed: 0.0,
        target: probe_hit.map(|h| (h.body, h.hit.point)),
    }
}

/// Finalize the reach: create a two-body FollowPoint constraint connecting
/// the player body to the held body and transition to Holding.
///
/// `hit_point` is the world-space point where the probe hit the body's surface.
/// It is converted to a body-local anchor so the constraint grabs at that point
/// rather than at the center of mass.
///
/// Returns `Some(ArmState::Holding)` if successful, `None` if the body is gone.
pub fn finalize_grab(
    physics: &mut PhysicsWorld,
    player_body: RigidBodyHandle,
    target_body: RigidBodyHandle,
    hit_point: Point3<f32>,
    config: &GrabConfig,
) -> Option<ArmState> {
    let player = physics.body(player_body)?;
    let player_pos = player.position();
    let player_rot = player.rotation();

    let body = physics.body(target_body)?;
    let body_rot = body.rotation();
    let world_offset = hit_point - body.position();
    let local_anchor_b = body_rot.inverse() * world_offset;

    // Start the hold point at the grab height (where the hit actually is)
    // rather than the final hold height, to avoid a sudden snap upward.
    // The lift system will raise it gradually.
    let initial_height = hit_point.y - player_pos.y;
    let local_anchor_a = Vector3::new(0.0, initial_height, config.hold_distance);

    // Snapshot the relative orientation at grab time: R_a⁻¹ * R_b.
    // The angular constraint drives the current relative orientation
    // back toward this snapshot.
    let relative_orientation = player_rot.inverse() * body_rot;

    let constraint = physics.create_constraint(ConstraintKind::FollowPoint {
        body_a: player_body,
        local_anchor_a,
        body_b: target_body,
        local_anchor_b,
        compliance: config.compliance,
        max_impulse: config.max_force,
        relative_orientation,
        angular_compliance: config.angular_compliance,
        angular_max_impulse: config.angular_max_impulse,
    });

    Some(ArmState::Holding {
        target_body,
        constraint,
        current_hold_height: initial_height,
    })
}

/// Gradually raise `local_anchor_a.y` toward `hold_height`, but never ahead
/// of where the held body's grab point actually is.
///
/// The anchor rises at `lift_speed` per second, clamped so it doesn't get
/// above the grab point's current height relative to the player. Heavy objects
/// that can't keep up naturally limit how fast the anchor rises.
/// Returns the new hold height (for animation).
pub fn update_lift(
    physics: &mut PhysicsWorld,
    player_body: RigidBodyHandle,
    target_body: RigidBodyHandle,
    constraint_handle: ConstraintHandle,
    dt: f32,
    config: &GrabConfig,
) -> f32 {
    // Read body positions and the constraint's local_anchor_b before taking
    // a mutable borrow on the constraint.
    let player_y = physics.body(player_body).map(|b| b.position().y);
    let body_b_data = physics
        .body(target_body)
        .map(|b| (b.position(), b.rotation()));
    let local_anchor_b_copy = physics
        .constraint(constraint_handle)
        .and_then(|c| match &c.kind {
            ConstraintKind::FollowPoint { local_anchor_b, .. } => Some(*local_anchor_b),
            _ => None,
        });

    let (Some(player_y), Some((body_b_pos, body_b_rot)), Some(local_anchor_b)) =
        (player_y, body_b_data, local_anchor_b_copy)
    else {
        return config.hold_height;
    };

    // Current grab point height relative to the player
    let r_b = body_b_rot * local_anchor_b;
    let grab_point_y = body_b_pos.y + r_b.y;
    let grab_height_rel = grab_point_y - player_y;

    let Some(constraint) = physics.constraint_mut(constraint_handle) else {
        return config.hold_height;
    };
    let ConstraintKind::FollowPoint {
        ref mut local_anchor_a,
        ..
    } = constraint.kind
    else {
        return config.hold_height;
    };

    // Nudge anchor upward, but don't exceed the grab point's current height
    // or the final hold height.
    let target_y = config.hold_height;
    let new_y = (local_anchor_a.y + config.lift_speed * dt).min(target_y);
    local_anchor_a.y = new_y.min(grab_height_rel + 0.05);
    local_anchor_a.y
}

/// Release a held object (drop cleanup). Removes the constraint.
pub fn release(physics: &mut PhysicsWorld, constraint_handle: ConstraintHandle) {
    physics.remove_constraint(constraint_handle);
}

/// Throw a held object: remove constraint and apply forward impulse.
pub fn throw(
    physics: &mut PhysicsWorld,
    target_body: RigidBodyHandle,
    constraint_handle: ConstraintHandle,
    facing: Vector3<f32>,
    throw_impulse: f32,
) {
    physics.remove_constraint(constraint_handle);
    physics.apply_impulse(target_body, facing * throw_impulse);
}

/// The arc a held body would fly if it were thrown this instant.
///
/// Stated from the same impulse [`throw`] applies, so the aiming cursor and
/// the throw agree by construction. A fixed impulse means a light crate leaves
/// fast and a heavy one barely leaves at all — the mass is doing the talking,
/// and the cursor says so.
///
/// Returns `None` for a body that has gone (destroyed terrain took it) or one
/// that cannot be moved at all.
pub fn throw_launch(
    physics: &PhysicsWorld,
    target_body: RigidBodyHandle,
    facing: Vector3<f32>,
    throw_impulse: f32,
) -> Option<Launch> {
    let body = physics.body(target_body)?;
    let inv_mass = body.inv_mass();
    if inv_mass <= 0.0 {
        return None;
    }

    Some(Launch {
        origin: body.position(),
        velocity: body.linear_velocity() + facing * throw_impulse * inv_mass,
        gravity: physics.config().gravity * body.gravity_scale(),
    })
}

/// Check if a held body still exists in the physics world.
/// Returns false if the body was destroyed (e.g. by terrain destruction).
pub fn is_body_alive(physics: &PhysicsWorld, body: RigidBodyHandle) -> bool {
    physics.body(body).is_some()
}
