use super::effects::EntryEffects;
use super::input::Requests;
use crate::character::components::CharacterIntent;
use crate::character::config::LocomotionConfig;
use crate::character::forgiveness::GroundForgiveness;
use crate::character::grounding::Grounding;

/// Countdown timer for input-grace windows (jump buffer, crouch buffer,
/// lockouts). Armed with a duration, it decays each frame to zero.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Timer(f32);

impl Timer {
    fn arm(&mut self, duration: f32) {
        self.0 = duration;
    }
    fn tick(&mut self, dt: f32) {
        self.0 = (self.0 - dt).max(0.0);
    }
    fn active(&self) -> bool {
        self.0 > 0.0
    }
    fn clear(&mut self) {
        self.0 = 0.0;
    }
}

/// Turns the raw intent and contacts into the facts and requests the state
/// machine reads: the jump and crouch buffers, ground forgiveness and the
/// crouch lockout. Only entry effects reset them ([`apply`](Self::apply)).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Conditioning {
    /// Jump-press grace: armed when jump is pressed, spent by a jump.
    jump_buffer: Timer,
    /// Crouch-press grace: while active, a jump is a long jump.
    crouch_buffer: Timer,
    /// After a long jump's landing, a held crouch is ignored.
    crouch_lockout: Timer,
    /// Holds grounding true briefly after contact is lost, so the chatter of
    /// a real contact set does not reach the state machine.
    forgiveness: GroundForgiveness,
}

impl Conditioning {
    /// Advance the buffers by `dt` and read this frame's `intent` into the
    /// requests. The plan's requests pass through as the intent carries them.
    pub fn requests(
        &mut self,
        intent: &CharacterIntent,
        dt: f32,
        config: &LocomotionConfig,
    ) -> Requests {
        self.jump_buffer.tick(dt);
        self.crouch_buffer.tick(dt);
        self.crouch_lockout.tick(dt);
        if intent.jump {
            self.jump_buffer.arm(config.jump_buffer_window);
        }
        if intent.crouch_just_pressed {
            self.crouch_buffer.arm(config.crouch_buffer_window);
        }
        Requests {
            move_dir: intent.direction,
            jump: self.jump_buffer.active(),
            jump_held: intent.jump_held,
            long_jump_armed: self.crouch_buffer.active(),
            takeoff: intent.takeoff,
            landing: intent.landing,
            edge: intent.edge,
            drop: intent.crouch_just_pressed,
        }
    }

    /// The contacts' `grounding`, forgiven for `dt`: real support re-arms
    /// the window, its absence spends it.
    pub fn support(
        &mut self,
        grounding: &Grounding,
        dt: f32,
        config: &LocomotionConfig,
    ) -> Grounding {
        self.forgiveness
            .observe(grounding, dt, config.ground_forgiveness_window)
    }

    /// Apply the conditioning's share of a state's entry effects.
    pub fn apply(&mut self, effects: &EntryEffects, config: &LocomotionConfig) {
        if effects.consume_jump {
            self.jump_buffer.clear();
        }
        if effects.cancel_forgiveness {
            self.forgiveness.cancel();
        }
        if effects.arm_crouch_lockout {
            self.crouch_lockout.arm(config.long_jump_crouch_lockout);
        }
    }

    /// Seconds since the contacts last held the body up; zero in contact.
    pub fn since_contact(&self) -> f32 {
        self.forgiveness.since_contact()
    }

    /// Whether a held crouch is being ignored, after a long jump's landing.
    pub fn crouch_locked(&self) -> bool {
        self.crouch_lockout.active()
    }

    /// The support the body was last actually on, however long ago.
    pub fn departed(&self) -> &Grounding {
        self.forgiveness.departed()
    }
}
