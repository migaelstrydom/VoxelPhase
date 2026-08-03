use specs::{Component, DenseVecStorage};

/// Movement tuning for one character.
///
/// This is a per-entity component rather than a global resource: a scuttling
/// creature and a lumbering one want different speeds and body sizes, and both
/// go through the same `CharacterControlSystem`. Named constructors below are
/// the presets; `Default` is the player.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct LocomotionConfig {
    /// Movement speed when on the ground (neutral gait).
    pub walk_speed: f32,
    /// Multiplier applied to `walk_speed` when sprint is held.
    pub sprint_speed_mul: f32,
    /// Multiplier applied to `walk_speed` when crouch is held (wins over sprint).
    pub crouch_speed_mul: f32,
    /// How quickly horizontal velocity steers toward input direction while airborne (units/s)
    pub air_steer_speed: f32,
    /// Ground acceleration used to ramp horizontal velocity toward the gait
    /// target speed. Higher = snappier; lower = more momentum.
    pub ground_accel: f32,
    /// Upward velocity applied when jumping
    pub jump_speed: f32,
    /// Multiplier applied to upward velocity when jump is released early while
    /// still rising. Lower = shorter tap-jumps. Range (0, 1].
    pub jump_cutoff_factor: f32,
    /// Window in which a jump press is remembered so it fires on landing
    /// (forgives "pressed jump just before hitting the ground"). Seconds.
    pub jump_buffer_window: f32,
    /// Window after pressing crouch during which jump triggers a long jump.
    /// Matches the Mario 64 "crouch-then-jump" cadence (~150ms).
    pub crouch_buffer_window: f32,
    /// Horizontal speed multiplier applied to `walk_speed` at long-jump launch.
    pub long_jump_speed_mul: f32,
    /// Vertical speed multiplier applied to `jump_speed` at long-jump launch
    /// (lower = flatter arc, longer range).
    pub long_jump_vertical_mul: f32,
    /// After landing from a long jump, suppress crouch for this many seconds.
    /// Prevents "held Ctrl" from instantly grinding the player to a halt.
    pub long_jump_crouch_lockout: f32,
    /// Duration of the air-steering lock during a long jump (seconds). When
    /// the timer expires mid-flight, the player regains air control.
    pub long_jump_air_lock_duration: f32,
    /// Grace period after leaving ground where the character still behaves as grounded (seconds).
    /// Prevents ramp launches and enables coyote-time jumping.
    pub ground_grace_period: f32,
    /// Proportional gain for the yaw angular velocity drive.
    /// Higher = snappier turns when unloaded. When holding a heavy object,
    /// the constraint reaction torque limits the actual turn rate regardless.
    pub turn_aggression: f32,

    /// Radius of the player's capsule collider.
    ///
    /// Lives here rather than at the spawn site because it is not only a
    /// physics number: it is the width of the thing that has to fit on a ledge,
    /// and `level_check` derives a route's minimum width from it.
    pub collider_radius: f32,
    /// Half the height of the capsule's cylindrical section, excluding caps.
    pub collider_half_height: f32,
}

impl LocomotionConfig {
    /// The player's movement envelope. `level_check` derives reachability from
    /// this, so changing it changes which routes a level is asserted to have.
    pub fn player() -> Self {
        Self {
            walk_speed: 5.0,
            sprint_speed_mul: 1.6,
            crouch_speed_mul: 0.4,
            air_steer_speed: 8.0,
            ground_accel: 40.0,
            jump_speed: 7.0,
            jump_cutoff_factor: 0.45,
            jump_buffer_window: 0.12,
            crouch_buffer_window: 0.5,
            long_jump_speed_mul: 2.5,
            long_jump_vertical_mul: 0.6,
            long_jump_crouch_lockout: 1.0,
            long_jump_air_lock_duration: 2.0,
            ground_grace_period: 0.08,
            turn_aggression: 10.0,
            collider_radius: 0.25,
            collider_half_height: 0.5,
        }
    }

    /// A ground creature that walks but never sprints, crouches, or long-jumps.
    /// Brains do not emit those intents, so the multipliers are inert; the
    /// numbers that matter are `walk_speed`, `ground_accel` and the body size.
    pub fn creature(walk_speed: f32, radius: f32, half_height: f32) -> Self {
        Self {
            walk_speed,
            ground_accel: walk_speed * 6.0,
            jump_speed: 5.0,
            turn_aggression: 6.0,
            collider_radius: radius,
            collider_half_height: half_height,
            ..Self::player()
        }
    }
}

impl Default for LocomotionConfig {
    fn default() -> Self {
        Self::player()
    }
}
