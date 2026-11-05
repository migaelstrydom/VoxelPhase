//! Grab, drop, and throw helpers for the player arm/action system.
//!
//! Called from `PlayerControlSystem` — not a standalone ECS system.

use nalgebra::{Point3, Vector3};

use crate::aim::Launch;
use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyHandle};

use crate::character::components::ArmState;

use super::probe::{GrabProbe, ReachFrame};

/// Configuration for grab mechanics. ECS resource (single player).
#[derive(Debug, Clone)]
pub struct GrabConfig {
    /// Decides what a grab reaches for.
    pub probe: GrabProbe,
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
    /// How hard a throw is.
    pub throw: ThrowStrength,
    /// Show debug overlays for probe ray, hold point, etc.
    pub debug_draw: bool,
}

impl Default for GrabConfig {
    fn default() -> Self {
        Self {
            probe: GrabProbe::default(),
            hold_distance: 0.5,
            hold_height: 0.3,
            compliance: 0.0,
            max_force: 1000.0,
            angular_compliance: 0.0,
            angular_max_impulse: 500.0,
            lift_speed: 0.5,
            reach_duration: 0.15,
            throw: ThrowStrength::default(),
            debug_draw: false,
        }
    }
}

/// How hard a character throws what it holds.
///
/// An arm is not a fixed impulse: it has mass of its own and moves only so
/// fast. A throw puts its impulse into the arm and the object together, so a
/// heavy object leaves at about `impulse / mass`, and a light one at no more
/// than `impulse / arm_mass`, the hand's own speed.
#[derive(Debug, Clone, Copy)]
pub struct ThrowStrength {
    /// Impulse the arm puts into itself and what it throws, N·s.
    pub impulse: f32,
    /// Effective mass of the throwing arm, kg.
    pub arm_mass: f32,
}

impl Default for ThrowStrength {
    /// A 14 kg clump of leaves leaves at about 19 m/s, a 30 kg crate at 14,
    /// a 100 kg barrel at 7, and nothing faster than 25.
    fn default() -> Self {
        Self {
            impulse: 1000.0,
            arm_mass: 40.0,
        }
    }
}

impl ThrowStrength {
    /// Speed, m/s, at which a body of `mass` kg leaves the hand.
    pub fn speed(&self, mass: f32) -> f32 {
        self.impulse / (mass + self.arm_mass)
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

/// Attempt to initiate a grab: probe the reach volume for a body.
///
/// `ground_depth` is the distance from `player_pos` down to the soles.
/// Returns the new `ArmState::Reaching` with the probe result.
pub fn begin_reach(
    physics: &PhysicsWorld,
    player_pos: Point3<f32>,
    facing: Vector3<f32>,
    ground_depth: f32,
    player_body: RigidBodyHandle,
    config: &GrabConfig,
) -> ArmState {
    let target = ReachFrame::new(player_pos, facing, ground_depth)
        .and_then(|frame| config.probe.find_target(physics, &frame, &[player_body]));
    ArmState::Reaching {
        elapsed: 0.0,
        target,
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
    strength: &ThrowStrength,
) {
    physics.remove_constraint(constraint_handle);
    let Some(mass) = physics.body(target_body).map(|b| b.mass()) else {
        return;
    };
    physics.apply_impulse(target_body, facing * strength.speed(mass) * mass);
}

/// The arc a held body would fly if it were thrown this instant.
///
/// Stated from the same speed [`throw`] gives, so the aiming cursor and the
/// throw agree by construction. A light crate leaves faster than a heavy one,
/// but never faster than the hand, and the cursor says so.
///
/// Returns `None` for a body that has gone (destroyed terrain took it) or one
/// that cannot be moved at all.
pub fn throw_launch(
    physics: &PhysicsWorld,
    target_body: RigidBodyHandle,
    facing: Vector3<f32>,
    strength: &ThrowStrength,
) -> Option<Launch> {
    let body = physics.body(target_body)?;
    let inv_mass = body.inv_mass();
    if inv_mass <= 0.0 {
        return None;
    }

    Some(Launch {
        origin: body.position(),
        velocity: body.linear_velocity() + facing * strength.speed(body.mass()),
        gravity: physics.config().gravity * body.gravity_scale(),
    })
}

/// Check if a held body still exists in the physics world.
/// Returns false if the body was destroyed (e.g. by terrain destruction).
pub fn is_body_alive(physics: &PhysicsWorld, body: RigidBodyHandle) -> bool {
    physics.body(body).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{ColliderDesc, RigidBodyDesc};

    /// A light object leaves no faster than the hand moves; a heavy one
    /// leaves at about the impulse over its own mass.
    #[test]
    fn a_throw_is_limited_by_the_hand_not_the_object() {
        let strength = ThrowStrength::default();
        let hand = strength.impulse / strength.arm_mass;
        assert!(strength.speed(0.1) < hand);
        assert!(strength.speed(0.1) > 0.99 * hand);
        let heavy = strength.speed(1000.0);
        let alone = strength.impulse / 1000.0;
        assert!((heavy - alone).abs() < 0.05 * alone, "{heavy} vs {alone}");
        assert!(strength.speed(14.0) < strength.speed(2.0));
    }

    /// The aiming arc launches a held body at the throw's speed for its mass.
    #[test]
    fn the_aimed_launch_leaves_at_the_throws_speed() {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        world.attach_collider(body, ColliderDesc::sphere(0.3).density(100.0));
        let mass = world.body(body).expect("just made").mass();
        let strength = ThrowStrength::default();
        let launch = throw_launch(&world, body, Vector3::x(), &strength).expect("movable");
        assert!((launch.velocity.x - strength.speed(mass)).abs() < 1e-4);
    }
}
