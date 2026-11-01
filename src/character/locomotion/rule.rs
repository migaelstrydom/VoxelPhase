use nalgebra::Vector3;

/// Air steering policy: how the character's horizontal velocity responds to
/// input while airborne.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AirSteering {
    /// Input-responsive: velocity steers toward the move direction at the
    /// air steering rate.
    Responsive,
    /// Committed planar velocity (a long jump). `remaining` seconds until
    /// the lock lapses and the character regains air control.
    Locked {
        velocity: Vector3<f32>,
        remaining: f32,
    },
}

impl AirSteering {
    /// Advance a committed lock by `dt`, lapsing to `Responsive` on expiry.
    ///
    /// Every state that can carry a lock must call this. A state that holds a
    /// `Locked` without decaying it pins the character to the committed
    /// velocity for as long as that state lasts.
    pub fn decayed(self, dt: f32) -> Self {
        match self {
            AirSteering::Locked {
                velocity,
                remaining,
            } => {
                let r = remaining - dt;
                if r <= 0.0 {
                    AirSteering::Responsive
                } else {
                    AirSteering::Locked {
                        velocity,
                        remaining: r,
                    }
                }
            }
            AirSteering::Responsive => AirSteering::Responsive,
        }
    }
}

/// The speeds and rates a movement rule chooses between, resolved from the
/// intent and the config before the rule is asked for.
#[derive(Debug, Clone, Copy)]
pub struct RuleSpeeds {
    /// Gait-resolved speed on the ground.
    pub ground: f32,
    /// Latched takeoff cap for air steering.
    pub air: f32,
    /// Air steering rate, in m/s².
    pub air_accel: f32,
    /// Swimming speed, relative to the water.
    pub swim: f32,
    /// Swimming acceleration, in m/s².
    pub swim_accel: f32,
}

/// Horizontal movement rule for the current locomotion state. Every state
/// collapses to "ask for a planar target velocity, say whether the body may
/// be steered toward it while nothing holds it up, and optionally cancel
/// upward vy."
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MovementRule {
    /// Planar target velocity (y unused; gravity owns vertical).
    pub target: Vector3<f32>,
    /// Rate the body may be steered toward `target` at while unsupported, in
    /// m/s². Spent against the actuator's allowance and no further.
    ///
    /// Nothing reads it while a character is supported: the traction rows ramp
    /// the body toward the target at the contact's own `μ·N`, so how quickly a
    /// walk starts is a property of the surface rather than a number the
    /// controller counts out. `None` is a state that wants no air authority at
    /// all: a committed long jump, whose arc is ballistic and stays that way
    /// because nothing corrects it.
    pub steer_accel: Option<f32>,
    /// Cancel positive vy, so a character walking off a crest does not rise
    /// off it on the climb it carried over the top.
    pub clamp_up: bool,
}

impl MovementRule {
    /// Steered toward `move_dir` at `speed`. The rate is the airborne one
    /// whatever the state, because it is only ever spent while nothing is
    /// holding the character up.
    pub fn steered(move_dir: Vector3<f32>, speed: f32, accel: f32) -> Self {
        Self {
            target: Vector3::new(move_dir.x * speed, 0.0, move_dir.z * speed),
            steer_accel: Some(accel),
            clamp_up: false,
        }
    }

    /// Held to a committed planar velocity, with no authority to steer it
    /// anywhere else: that is what keeps the arc committed.
    pub fn locked(velocity: Vector3<f32>) -> Self {
        Self {
            target: velocity,
            steer_accel: None,
            clamp_up: false,
        }
    }

    /// In the air under `steering`: steered at the air speed, or locked.
    pub fn airborne(steering: AirSteering, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> Self {
        match steering {
            AirSteering::Locked { velocity, .. } => Self::locked(velocity),
            AirSteering::Responsive => Self::steered(move_dir, speeds.air, speeds.air_accel),
        }
    }
}
