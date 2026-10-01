//! The player, as the physics engine sees it, driven by a script.
//!
//! The body and its actuator are the game's own — `create_player_body` and
//! `player_actuator` — and the command reaches the engine by the game's own
//! path, `apply_drive`. What stands in for the game is only the decision of
//! what to ask for: a planar target at the gait's speed, steered at the air
//! rate while nothing holds the body up, and a jump on a frame it is
//! standing on something. That is the movement rule `CharacterControlSystem`
//! writes, without the state machine around it.

use std::fmt;

use nalgebra::{Point3, Vector3};
use rand::rngs::StdRng;
use rand::Rng;

use crate::app::spawners::{create_player_body, player_actuator};
use crate::character::LocomotionConfig;
use crate::drive::{apply_drive, Actuator, DriveIntent};
use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// What the walker is told to do: run at a structure from some side, perhaps
/// jump on the way, and keep pushing for a while once it gets there.
///
/// Distances are from the structure's edge, so one script fits a structure
/// of any size.
#[derive(Clone, Debug)]
pub struct WalkerScript {
    /// Compass bearing it runs in, in radians.
    pub bearing: f32,
    /// How far from the structure's edge it starts, in metres.
    pub run_up: f32,
    /// Where across the structure it aims, as a fraction of its reach either
    /// side of the centre.
    pub aim_across: f32,
    /// Sprinting rather than walking.
    pub sprint: bool,
    /// Jump this far out from the structure's edge, in metres (negative is
    /// inside it), if at all.
    pub jump_from_edge: Option<f32>,
    /// How long it keeps running at the aim, in seconds, before it stops.
    pub run_seconds: f32,
}

impl WalkerScript {
    pub fn generate(rng: &mut StdRng) -> Self {
        Self {
            bearing: rng.gen_range(0.0..std::f32::consts::TAU),
            run_up: rng.gen_range(2.0..7.0),
            aim_across: rng.gen_range(-0.5..0.5),
            sprint: rng.gen_bool(0.7),
            jump_from_edge: rng.gen_bool(0.7).then(|| rng.gen_range(-0.5..2.5)),
            run_seconds: rng.gen_range(1.0..4.0),
        }
    }
}

impl fmt::Display for WalkerScript {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let gait = if self.sprint { "sprints" } else { "walks" };
        write!(
            f,
            "walker {gait} from {:.1} m out for {:.1} s",
            self.run_up, self.run_seconds
        )?;
        if let Some(edge) = self.jump_from_edge {
            write!(f, ", jumps {edge:.1} m from the edge")?;
        }
        Ok(())
    }
}

/// The player's body, following a script.
pub struct Walker {
    script: WalkerScript,
    /// Where it runs to, on the floor.
    aim: Point3<f32>,
    /// Distance from the aim at which it jumps, if it does.
    jump_at: Option<f32>,
    locomotion: LocomotionConfig,
    actuator: Actuator,
    intent: DriveIntent,
    body: RigidBodyHandle,
    jumped: bool,
}

impl Walker {
    /// Stand the player's body where the script starts it, against a
    /// structure whose bodies lie within `reach` of `centre` on the floor.
    pub fn spawn(
        world: &mut PhysicsWorld,
        script: WalkerScript,
        centre: Point3<f32>,
        reach: f32,
    ) -> Self {
        let locomotion = LocomotionConfig::player();
        let toward = Vector3::new(script.bearing.cos(), 0.0, script.bearing.sin());
        let across = Vector3::new(-toward.z, 0.0, toward.x);
        let start = centre - toward * (reach + script.run_up);
        let aim = centre + across * script.aim_across * reach;
        let jump_at = script.jump_from_edge.map(|edge| reach + edge);
        let standing = Point3::new(start.x, locomotion.collider_half_height, start.z);
        let (body, _) = create_player_body(world, standing, &locomotion);
        let actuator = player_actuator(&locomotion);
        Self {
            script,
            aim: Point3::new(aim.x, 0.0, aim.z),
            jump_at,
            locomotion,
            actuator,
            intent: DriveIntent::default(),
            body,
            jumped: false,
        }
    }

    pub fn body(&self) -> RigidBodyHandle {
        self.body
    }

    /// Ask for this frame's motion, `elapsed` seconds into the run.
    pub fn drive(&mut self, world: &mut PhysicsWorld, elapsed: f32) {
        let Some(position) = world.body(self.body).map(|b| b.position()) else {
            return;
        };
        let to_aim = Vector3::new(self.aim.x - position.x, 0.0, self.aim.z - position.z);
        let running = elapsed < self.script.run_seconds;
        let direction = if running {
            to_aim.try_normalize(1e-3).unwrap_or_else(Vector3::zeros)
        } else {
            Vector3::zeros()
        };
        let speed = self.locomotion.walk_speed
            * if self.script.sprint {
                self.locomotion.sprint_speed_mul
            } else {
                1.0
            };
        self.intent.linear_target = direction * speed;
        self.intent.steer_accel = Some(self.locomotion.air_steer_speed);

        let jump_now = !self.jumped
            && running
            && self.jump_at.is_some_and(|at| to_aim.norm() <= at)
            && world.support_sets().is_supported(self.body);
        if jump_now {
            self.intent.jump(self.locomotion.jump_speed);
            self.jumped = true;
        }
        apply_drive(world, self.body, &mut self.intent, &self.actuator);
    }
}
