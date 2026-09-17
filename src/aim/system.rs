//! The frame's prediction: pick a source, fly its launch, keep the landing.

use nalgebra::Point3;
use specs::{Join, Read, ReadExpect, ReadStorage, System, Write};

use crate::camera::FollowTarget;
use crate::character::grab::GrabConfig;
use crate::character::{facing_from_rotation, ArmState, CharacterIntent, CharacterState};
use crate::components::{Position, RigidBodyComponent, Rotation};
use crate::player::Player;
use crate::projectile::GrenadeConfig;
use crate::sensing::{ProbeSet, ProbeTarget};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

use super::gate::AimGate;
use super::probe::BodiesExcept;
use super::source::{AimContext, AimSource, GrenadeAim, HeldObjectAim};
use super::state::{AimSolution, AimState};
use super::trajectory::TrajectoryPredictor;

/// Fills [`AimState`] while the player is aiming.
///
/// Three decisions, each one owned elsewhere: [`AimGate`] says whether to aim,
/// an [`AimSource`] says what arc a throw would fly, and
/// [`TrajectoryPredictor`] says what that arc hits. This system only wires the
/// three together with the frame's camera, arm and world.
#[derive(Default)]
pub struct AimPredictionSystem {
    predictor: TrajectoryPredictor,
    gate: AimGate,
}

impl<'a> System<'a> for AimPredictionSystem {
    type SystemData = (
        Write<'a, AimState>,
        ReadExpect<'a, PhysicsResource>,
        ReadExpect<'a, GrenadeConfig>,
        ReadExpect<'a, GrabConfig>,
        Option<Read<'a, TerrainWorld>>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, FollowTarget>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut aim,
            physics,
            grenade_config,
            grab_config,
            terrain,
            players,
            positions,
            rotations,
            states,
            intents,
            rigid_bodies,
            follow_targets,
        ) = data;

        aim.solution = None;

        let player = (
            &players,
            &positions,
            &rotations,
            &states,
            &intents,
            &rigid_bodies,
        )
            .join()
            .next();
        let Some((_, position, rotation, state, intent, body)) = player else {
            self.gate.reset();
            return;
        };

        if !self
            .gate
            .update(intent.grab_held, intent.throw, intent.throw_grenade)
        {
            return;
        }

        let Some((look_angle, camera_pitch)) = follow_targets
            .join()
            .next()
            .map(|follow| (follow.orbit_angle + std::f32::consts::PI, follow.pitch))
        else {
            return;
        };

        let ctx = AimContext {
            physics: &physics.world,
            thrower: body.0,
            position: Point3::new(position.0.x, position.0.y, position.0.z),
            facing: facing_from_rotation(rotation.0),
            look_angle,
            camera_pitch,
            arm: &state.arm,
        };

        let grenade = GrenadeAim::new(&grenade_config);
        let held = HeldObjectAim::new(&grab_config);
        let source: &dyn AimSource = match state.arm {
            ArmState::Holding { .. } => &held,
            _ => &grenade,
        };

        let Some(launch) = source.launch(&ctx) else {
            return;
        };

        let ignored = source.ignored_bodies(&ctx);
        let bodies = BodiesExcept::new(&physics.world, &ignored);
        let world = ProbeSet::new([
            terrain.as_ref().map(|t| &**t as &dyn ProbeTarget),
            Some(&bodies as &dyn ProbeTarget),
        ]);

        aim.solution = self
            .predictor
            .predict(&launch, &world)
            .map(|impact| AimSolution {
                kind: source.kind(),
                impact,
            });
    }
}
