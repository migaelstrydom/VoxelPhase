use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::character::{facing_from_rotation, ArmState, CharacterState, LocomotionConfig};
use crate::components::{RigidBodyComponent, Rotation};
use crate::debug::DebugOverlays;
use crate::physics::RigidBodyHandle;
use crate::platform::MovingPlatform;
use crate::rendering::Colour;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::traversal::{carriers, TraversalPlanState, WorldCaster};

use super::edge::WorldEdge;
use super::probe::{edge_at_hands, Reach};
use super::sensor::LedgeSensor;

/// Finds the edge at each sensing body's hands, from the world as last
/// stepped, and draws it with the traversal plan's overlay (F6).
pub struct LedgeSensorSystem;

impl<'a> System<'a> for LedgeSensorSystem {
    type SystemData = (
        WriteStorage<'a, LedgeSensor>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, LocomotionConfig>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, MovingPlatform>,
        Read<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, TraversalPlanState>,
        Write<'a, DebugOverlays>,
    );

    fn run(
        &mut self,
        (
            mut sensors,
            bodies,
            rotations,
            configs,
            states,
            platforms,
            physics,
            terrain,
            plan_state,
            mut overlays,
        ): Self::SystemData,
    ) {
        let Some(terrain) = terrain else {
            return;
        };
        let carriers = carriers(&platforms, &bodies);
        for (sensor, body, rotation, config, state) in
            (&mut sensors, &bodies, &rotations, &configs, &states).join()
        {
            let Some(rigid) = physics.world.body(body.0) else {
                sensor.edge = None;
                continue;
            };
            let exclude = excluded(body.0, &state.arm);
            let caster = WorldCaster {
                world: &physics.world,
                scenery: &*terrain,
                exclude: &exclude,
                carriers: &carriers,
            };
            let reach = Reach {
                centre: rigid.position(),
                heading: facing_from_rotation(rotation.0),
                radius: config.collider_radius,
                half_height: config.collider_half_height,
            };
            sensor.edge = edge_at_hands(&caster, &reach, &sensor.config);
            if plan_state.overlay {
                if let Some(edge) = sensor.edge.and_then(|edge| edge.in_world(&physics.world)) {
                    draw(&mut overlays, &edge);
                }
            }
        }
    }
}

/// The bodies the hands feel through: their own, and whatever they hold.
fn excluded(own: RigidBodyHandle, arm: &ArmState) -> Vec<RigidBodyHandle> {
    match arm {
        ArmState::Holding { target_body, .. } => vec![own, *target_body],
        ArmState::Idle | ArmState::Reaching { .. } => vec![own],
    }
}

/// The hands as yellow points on the lip, the way out as a line, and where
/// a pull-up stands the body as a green point.
fn draw(overlays: &mut DebugOverlays, edge: &WorldEdge) {
    for hand in edge.hands {
        overlays.add_sphere(hand, 0.03, Colour::YELLOW);
    }
    let grip = edge.grip();
    overlays.add_line(grip, grip + edge.outward.into_inner() * 0.3, Colour::YELLOW);
    if let Some(stand) = edge.stand {
        overlays.add_sphere(stand, 0.05, Colour::GREEN);
    }
}
