//! Where the pelvis is, frame by frame.
//!
//! The animator is a pure function of body motion: give it a pelvis, a yaw, a
//! velocity and a grounding flag and it produces a pose. This is the smallest
//! thing that can supply those honestly without standing up the physics engine,
//! a terrain world and an ECS dispatcher.
//!
//! ```text
//!   Beat ──▶ CharacterIntent ──▶ LocomotionState::tick ──▶ MovementRule
//!                                        │                     │
//!                                        ▼                     ▼
//!                                  jump impulse          planar target
//!                                        └──────┬──────────────┘
//!                                               ▼
//!                                    Body: integrate, ride the ground
//! ```
//!
//! Everything above the dashed line is the game's own code — the same state
//! machine, the same movement rule, the same speed resolution. What is a
//! stand-in is the last box: the real body is a capsule solved by the physics
//! engine against terrain contacts, and this one rides the height field
//! directly.
//!
//! That substitution is deliberate and it is the tool's main limitation, so it
//! is worth being precise about what it costs. Contact chatter, penetration
//! recovery and the traction ramp are all absent, so the velocity the animator
//! sees here is cleaner than the game's. A gait artefact reproduced here is
//! therefore an animation artefact, not a physics one — which is the whole
//! point of isolating it. The converse does not hold: an artefact that only
//! appears in game may still be real and live on the physics side.

use nalgebra::{Point3, Vector3};

use crate::character::{
    CharacterIntent, CharacterState, LocomotionConfig, LocomotionInput, LocomotionState,
};
use crate::systems::resolve_ground_speed;

use super::ground::Ground;

/// Gravity, matching `PhysicsWorld`'s default.
const GRAVITY: f32 = -9.81;

/// How fast a supported body may be steered toward its target velocity, in
/// m/s².
///
/// The one number with no counterpart in the game: on real ground the ramp from
/// standstill to walk speed is the traction budget at the supporting contacts,
/// which needs an engine to compute. This is a stand-in chosen to reach walk
/// speed in about a quarter of a second, which is what the traction rows
/// deliver on ordinary ground.
const GROUND_STEER_ACCEL: f32 = 20.0;

/// How far below its standing height the body may sit and still count as
/// supported.
const GROUND_TOLERANCE: f32 = 0.02;

/// Fastest the surface may carry the body upward, in m/s.
///
/// A height field can step by any amount between two samples; a capsule cannot.
/// It meets the riser, and its bottom cap rolls it over the edge across roughly
/// a radius of travel, which bounds the climb rate. Without a cap here a stair
/// nosing teleports the pelvis up a whole rise in one frame and hands the
/// animator a vertical velocity of tens of metres a second — an artefact of the
/// stand-in, and one that would be reported as an animation defect.
const MAX_CLIMB_RATE: f32 = 3.0;

/// The scripted body the animator is hung on.
pub struct Body {
    /// Body position — the physics capsule's centre in the game, which is what
    /// the animator is handed and what it hangs the rig below.
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub yaw: f32,
    pub grounded: bool,

    pub state: CharacterState,
    pub config: LocomotionConfig,

    /// Height of the body's origin above the ground when standing.
    ///
    /// Not the rig's `standing_height`: this is the capsule's resting centre,
    /// which is what the game hands the animator. The animator is what turns
    /// one into the other — see `CharacterAnimator::pelvis_for` — and taking
    /// the rig's number here would hide whether it does.
    ride_height: f32,
}

impl Body {
    /// The height a supported capsule's centre rests at, which is what the
    /// game hands the animator as the pelvis.
    ///
    /// `ColliderShape::Capsule::half_height` includes the caps, so a resting
    /// capsule's centre sits exactly one half-height above the surface.
    pub fn ride_height(config: &LocomotionConfig) -> f32 {
        config.collider_half_height
    }

    /// Place a body standing on `ground` at `(x, z)`, facing +x.
    pub fn standing_at(
        ground: &dyn Ground,
        x: f32,
        z: f32,
        ride_height: f32,
        config: LocomotionConfig,
    ) -> Self {
        let surface = ground
            .height(x, z)
            .expect("a scenario must start on solid ground");
        Self {
            position: Point3::new(x, surface + ride_height, z),
            velocity: Vector3::zeros(),
            yaw: std::f32::consts::FRAC_PI_2,
            grounded: true,
            state: CharacterState::default(),
            config,
            ride_height,
        }
    }

    /// Advance one frame under `intent`.
    ///
    /// Order mirrors `CharacterControlSystem`: buffers first, then the
    /// locomotion transition, then the movement rule, then integration.
    pub fn step(
        &mut self,
        dt: f32,
        intent: &CharacterIntent,
        ground: &dyn Ground,
        support_velocity: Vector3<f32>,
    ) {
        self.tick_buffers(dt, intent);

        let move_dir = intent
            .direction
            .try_normalize(1e-4)
            .unwrap_or_else(Vector3::zeros);
        let horizontal_speed = self.horizontal_speed();

        let outcome = self.state.locomotion.tick(&LocomotionInput {
            dt,
            is_grounded: self.grounded,
            jump_pressed: self.state.jump_buffer.active(),
            horizontal_speed,
            move_dir,
            long_jump_armed: self.state.crouch_buffer.active(),
            config: &self.config,
        });
        self.state.locomotion = outcome.next_state;
        if let Some(vy) = outcome.set_vy {
            self.velocity.y = vy;
        }
        if let Some(air) = outcome.set_air_speed {
            self.state.air_speed = air;
        }
        if outcome.consumed_jump {
            self.state.jump_buffer.clear();
        }

        let ground_speed = resolve_ground_speed(intent, &self.config, &self.state.crouch_lockout);
        let mut rule = self.state.locomotion.movement_rule(
            move_dir,
            ground_speed,
            self.state.air_speed,
            self.config.air_steer_speed,
        );
        // The gait's speed is relative to the floor: standing still on a
        // platform means matching it, not holding a world position. The game
        // says so at the contact rather than here — a traction row drives the
        // relative velocity across it, so `CharacterControlSystem` states the
        // same thing by leaving the target alone. This body has no contacts to
        // say it at, so it adds the deck itself and arrives at the velocity the
        // engine would have produced, which is all the animator is given.
        if self.grounded {
            rule.target += Vector3::new(support_velocity.x, 0.0, support_velocity.z);
        }

        // Supported bodies get the ground ramp; unsupported ones get whatever
        // authority the rule grants, and a committed arc gets none at all.
        let accel = if self.grounded {
            Some(GROUND_STEER_ACCEL)
        } else {
            rule.steer_accel
        };
        if let Some(accel) = accel {
            self.steer_planar(rule.target, accel, dt);
        }
        if rule.clamp_up {
            self.velocity.y = self.velocity.y.min(0.0);
        }

        self.steer_yaw(move_dir, dt);
        self.integrate(dt, ground);
    }

    pub fn horizontal_speed(&self) -> f32 {
        Vector3::new(self.velocity.x, 0.0, self.velocity.z).magnitude()
    }

    /// Whether the locomotion state is one of the airborne ones. The animator
    /// reads grounding separately; this is for the report's pose column.
    pub fn is_airborne(&self) -> bool {
        matches!(
            self.state.locomotion,
            LocomotionState::Airborne { .. } | LocomotionState::Launching { .. }
        )
    }

    fn tick_buffers(&mut self, dt: f32, intent: &CharacterIntent) {
        self.state.jump_buffer.tick(dt);
        self.state.crouch_buffer.tick(dt);
        self.state.crouch_lockout.tick(dt);
        if intent.jump {
            self.state.jump_buffer.arm(self.config.jump_buffer_window);
        }
        if intent.crouch_just_pressed {
            self.state
                .crouch_buffer
                .arm(self.config.crouch_buffer_window);
        }
    }

    /// Move the planar velocity toward `target` at no more than `accel`.
    fn steer_planar(&mut self, target: Vector3<f32>, accel: f32, dt: f32) {
        let current = Vector3::new(self.velocity.x, 0.0, self.velocity.z);
        let delta = Vector3::new(target.x, 0.0, target.z) - current;
        let budget = accel * dt;
        let step = if delta.magnitude() <= budget {
            delta
        } else {
            delta.normalize() * budget
        };
        self.velocity.x += step.x;
        self.velocity.z += step.z;
    }

    /// Turn toward the direction of travel, as the yaw drive does.
    fn steer_yaw(&mut self, move_dir: Vector3<f32>, dt: f32) {
        if move_dir.magnitude() <= 1e-3 {
            return;
        }
        let target = -move_dir.z.atan2(move_dir.x) + std::f32::consts::FRAC_PI_2;
        let error = wrap_angle(target - self.yaw);
        // The game commands an angular velocity proportional to the error and
        // lets the body integrate it; with no body in the way, that is the same
        // exponential approach expressed directly.
        self.yaw = wrap_angle(self.yaw + error * (self.config.turn_aggression * dt).min(1.0));
    }

    /// Integrate position, then resolve support against the ground.
    fn integrate(&mut self, dt: f32, ground: &dyn Ground) {
        if !self.grounded {
            self.velocity.y += GRAVITY * dt;
        }

        let previous_y = self.position.y;
        self.position.x += self.velocity.x * dt;
        self.position.z += self.velocity.z * dt;
        self.position.y += self.velocity.y * dt;

        let Some(surface) = ground.height(self.position.x, self.position.z) else {
            // Walked out over nothing: the body keeps its arc and support ends.
            self.grounded = false;
            return;
        };
        let stand_y = surface + self.ride_height;

        // Support is decided by height alone, never by the sign of the vertical
        // velocity. A body climbing a slope has a genuinely positive rise rate
        // every frame — it is being carried up — and testing for a descent
        // would send it airborne the moment the hill started. A jump separates
        // itself the honest way: its impulse lifts the body clear of
        // `stand_y` within one frame.
        if self.position.y <= stand_y + GROUND_TOLERANCE {
            // Rise at the capped rate; fall to the surface immediately, since
            // nothing holds a body up on the way down.
            let climb_limit = previous_y + MAX_CLIMB_RATE * dt;
            self.position.y = stand_y.min(climb_limit.max(stand_y.min(previous_y)));
            self.grounded = true;
            // The vertical velocity the animator sees on a slope is the one the
            // surface imposes, not zero — the overstretch gate reads it.
            self.velocity.y = (self.position.y - previous_y) / dt.max(1e-6);
        } else {
            self.grounded = false;
        }
    }
}

#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let pi = std::f32::consts::PI;
    let mut a = angle % (2.0 * pi);
    if a > pi {
        a -= 2.0 * pi;
    } else if a < -pi {
        a += 2.0 * pi;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::anim_viewer::ground::{Flat, Slope};

    fn walking_intent() -> CharacterIntent {
        CharacterIntent {
            direction: Vector3::x(),
            ..Default::default()
        }
    }

    #[test]
    fn a_walk_reaches_walk_speed_and_stays_on_the_floor() {
        let ground = Flat::at(0.0);
        let config = LocomotionConfig::player();
        let mut body = Body::standing_at(
            &ground,
            0.0,
            0.0,
            Body::ride_height(&config),
            config.clone(),
        );

        let intent = walking_intent();
        for _ in 0..120 {
            body.step(1.0 / 60.0, &intent, &ground, Vector3::zeros());
        }

        assert!((body.horizontal_speed() - config.walk_speed).abs() < 0.05);
        assert!((body.position.y - config.collider_half_height).abs() < 1e-3);
        assert!(body.grounded);
    }

    #[test]
    fn a_slope_shows_up_as_vertical_velocity() {
        let ground = Slope::degrees("up20", 20.0);
        let config = LocomotionConfig::player();
        let mut body = Body::standing_at(&ground, 0.0, 0.0, Body::ride_height(&config), config);

        let intent = walking_intent();
        for _ in 0..120 {
            body.step(1.0 / 60.0, &intent, &ground, Vector3::zeros());
        }

        assert!(body.grounded, "walking uphill must stay supported");
        assert!(
            body.velocity.y > 1.0,
            "climbing at speed implies a real rise rate, got {}",
            body.velocity.y
        );
    }

    /// Standing on a moving platform with no input: the body is carried, and
    /// after a moment it is travelling with the deck rather than sliding off
    /// the back of it. Mirrors `CharacterControlSystem`, which adds the same
    /// term to the same rule.
    #[test]
    fn an_idle_body_is_carried_by_the_surface_under_it() {
        let ground = Flat::at(0.0);
        let config = LocomotionConfig::player();
        let mut body = Body::standing_at(&ground, 0.0, 0.0, Body::ride_height(&config), config);
        let deck = Vector3::new(3.0, 0.0, 0.0);

        let idle = CharacterIntent::default();
        for _ in 0..60 {
            body.step(1.0 / 60.0, &idle, &ground, deck);
        }

        assert!(
            (body.velocity.x - deck.x).abs() < 0.05,
            "the body should be riding at {} m/s, got {}",
            deck.x,
            body.velocity.x
        );
    }

    #[test]
    fn a_jump_leaves_the_ground_and_comes_back() {
        let ground = Flat::at(0.0);
        let config = LocomotionConfig::player();
        let mut body = Body::standing_at(&ground, 0.0, 0.0, Body::ride_height(&config), config);

        let mut intent = walking_intent();
        intent.jump = true;
        intent.jump_held = true;
        body.step(1.0 / 60.0, &intent, &ground, Vector3::zeros());
        intent.jump = false;

        let mut left_ground = false;
        for _ in 0..200 {
            body.step(1.0 / 60.0, &intent, &ground, Vector3::zeros());
            left_ground |= !body.grounded;
            if left_ground && body.grounded {
                break;
            }
        }
        assert!(left_ground, "a jump must leave the ground");
        assert!(body.grounded, "and must land again");
    }
}
