//! Carries every fish rig on a frame and draws it.

use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

use super::rig::FishRig;
use crate::components::{Orientation, PosedModel, Position, ProceduralMesh, Velocity};
use crate::player::Player;
use crate::time::Time;

/// Carries each [`FishRig`] on by its body's speed, watching the player,
/// and writes its posed parts and its rippled fins.
///
/// It only draws: nothing it does reaches the body or what it collides as.
pub struct FishRigSystem;

impl<'a> System<'a> for FishRigSystem {
    type SystemData = (
        WriteStorage<'a, FishRig>,
        WriteStorage<'a, PosedModel>,
        WriteStorage<'a, ProceduralMesh>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, Player>,
        ReadExpect<'a, Time>,
    );

    fn run(
        &mut self,
        (mut rigs, mut posed, mut meshes, positions, orientations, velocities, players, time): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        let player = (&players, &positions).join().next().map(|(_, at)| at.0);
        for (rig, posed, mesh, position, orientation, velocity) in (
            &mut rigs,
            &mut posed,
            &mut meshes,
            &positions,
            &orientations,
            &velocities,
        )
            .join()
        {
            let watched = player.map(|at| orientation.0.inverse() * (at - position.0));
            rig.advance(dt, velocity.0.norm(), watched);
            rig.pose(&mut posed.poses);
            rig.shape(&mut mesh.primitives);
        }
    }
}
