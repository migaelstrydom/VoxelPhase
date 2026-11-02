use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::character::locomotion::{Ledge, LedgePhase};
use crate::character::{
    facing_from_rotation, ArmState, CharacterIntent, CharacterState, LocomotionConfig,
    LocomotionState,
};
use crate::components::{RigidBodyComponent, Rotation};
use crate::debug::DebugOverlays;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::platform::MovingPlatform;
use crate::rendering::Colour;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;
use crate::traversal::{carriers, TraversalPlanState, WorldCaster};

use super::config::LedgeConfig;
use super::edge::{SensedEdge, WorldEdge};
use super::probe::{edge_at_hands, Reach};
use super::sensor::{EdgeAhead, LedgeSensor};

/// Finds the edge at each sensing body's hands, from the world as last
/// stepped, and hands it to the locomotion state through the intent: while
/// the hands are free, an edge they may catch; while they hold one, that
/// edge found again, with whether it carries on either way for a shimmy.
/// Draws it with the traversal plan's overlay (F6).
pub struct LedgeSensorSystem;

impl<'a> System<'a> for LedgeSensorSystem {
    type SystemData = (
        WriteStorage<'a, LedgeSensor>,
        WriteStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, LocomotionConfig>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, MovingPlatform>,
        Read<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, Time>,
        Read<'a, TraversalPlanState>,
        Write<'a, DebugOverlays>,
    );

    fn run(
        &mut self,
        (
            mut sensors,
            mut intents,
            bodies,
            rotations,
            configs,
            states,
            platforms,
            physics,
            terrain,
            time,
            plan_state,
            mut overlays,
        ): Self::SystemData,
    ) {
        let Some(terrain) = terrain else {
            return;
        };
        let carriers = carriers(&platforms, &bodies);
        let dt = time.delta_seconds();
        for (sensor, intent, body, rotation, config, state) in (
            &mut sensors,
            &mut intents,
            &bodies,
            &rotations,
            &configs,
            &states,
        )
            .join()
        {
            let ledge = &config.ledge;
            let holding = matches!(state.locomotion, LocomotionState::Ledge(_));
            let let_go = sensor.was_holding
                && !holding
                && !matches!(state.locomotion, LocomotionState::OnFoot(_));
            sensor.regrab_lock = if let_go {
                ledge.regrab_lock
            } else {
                (sensor.regrab_lock - dt).max(0.0)
            };
            sensor.was_holding = holding;

            let world = &physics.world;
            let Some(rigid) = world.body(body.0) else {
                intent.edge = None;
                continue;
            };
            let exclude = excluded(body.0, &state.arm);
            let mut seen = carriers.clone();
            if let LocomotionState::Ledge(Ledge {
                edge: held_edge, ..
            }) = state.locomotion
            {
                seen.extend(held_edge.body);
            }
            let caster = WorldCaster {
                world,
                scenery: &*terrain,
                exclude: &exclude,
                carriers: &seen,
            };
            let reach = |heading: Vector3<f32>| Reach {
                centre: rigid.position(),
                heading,
                radius: config.collider_radius,
                half_height: config.collider_half_height,
                mass: rigid.mass(),
            };
            intent.edge = match state.locomotion {
                LocomotionState::Ledge(held) => {
                    let grip_holds = state.grip.is_some_and(|grip| grip.holds(world));
                    let started = state.grip.is_none();
                    (grip_holds || started)
                        .then(|| found_again(&caster, &held, &reach, ledge))
                        .flatten()
                }
                _ if state.locomotion.hands_free() && sensor.regrab_lock <= 0.0 => {
                    let heading = facing_from_rotation(rotation.0);
                    edge_at_hands(&caster, &reach(heading), ledge)
                        .and_then(|edge| sensed(world, edge, [false; 2]))
                        .filter(|sensed| {
                            catchable(world, body.0, sensed, heading, &state.locomotion, ledge)
                        })
                }
                _ => None,
            };
            let gravity = world.config().gravity;
            let velocity = rigid.linear_velocity();
            sensor.wall_below = match (state.locomotion, &intent.edge) {
                (LocomotionState::Ledge(_), Some(held)) => face_below(
                    &caster,
                    rigid.position(),
                    held,
                    config.collider_radius,
                    ledge,
                ),
                _ => false,
            };
            let flying = matches!(
                state.locomotion,
                LocomotionState::Jump(_) | LocomotionState::Fall(_)
            );
            // An edge already at the hands that the state has not taken (below
            // the chest, on the way up) is one they reach for now.
            sensor.ahead = (flying && sensor.regrab_lock <= 0.0)
                .then(|| match &intent.edge {
                    Some(at_hands) => Some(EdgeAhead {
                        hands: at_hands.at.hands,
                        in_seconds: 0.0,
                    }),
                    None => {
                        let heading = facing_from_rotation(rotation.0);
                        edge_ahead(&caster, &reach(heading), velocity, gravity, ledge)
                    }
                })
                .flatten();
            if plan_state.overlay {
                if let Some(sensed) = &intent.edge {
                    draw(&mut overlays, &sensed.at);
                }
                if let Some(ahead) = &sensor.ahead {
                    for hand in ahead.hands {
                        overlays.add_sphere(hand, 0.03, Colour::new(1.0, 0.6, 0.0, 1.0));
                    }
                }
            }
        }
    }
}

/// The held edge found again on the same body, facing it from where the
/// body is now, and whether it carries on either way. A pull-up is
/// committed: its edge is the one held, wherever its body has taken it.
fn found_again(
    caster: &WorldCaster,
    held: &Ledge,
    reach: &dyn Fn(Vector3<f32>) -> Reach,
    config: &LedgeConfig,
) -> Option<SensedEdge> {
    let world = caster.world;
    let at = held.edge.in_world(world)?;
    if let LedgePhase::PullUp { .. } = held.phase {
        return sensed(world, held.edge, [false; 2]);
    }
    let heading = -at.outward.into_inner();
    let here = reach(heading);
    let same_body = |edge: &super::Edge| edge.body == held.edge.body;
    let edge = edge_at_hands(caster, &here, config).filter(same_body)?;
    let along = at.along() * config.shimmy_probe;
    let room = [-along, along].map(|step| {
        let moved = Reach {
            centre: here.centre + step,
            ..here
        };
        edge_at_hands(caster, &moved, config).is_some_and(|edge| same_body(&edge))
    });
    sensed(world, edge, room)
}

/// Whether a face stands below the held edge where the hanging feet are: a
/// cast from the shins towards the wall.
fn face_below(
    caster: &WorldCaster,
    centre: Point3<f32>,
    held: &SensedEdge,
    radius: f32,
    config: &LedgeConfig,
) -> bool {
    let shins = centre - Vector3::y() * config.brace_drop;
    let towards = -held.at.outward.into_inner();
    let reach = radius + config.face_gap + config.reach_ahead;
    caster
        .cast(shins, towards, reach, FOOT)
        .is_some_and(|hit| hit.contact.normal.dot(&towards) < -0.5)
}

/// Radius of the sphere the feet feel for a face with, in metres.
const FOOT: f32 = 0.05;

/// The edge the hands meet along the body's arc in the next
/// `reach_ahead_time`, looked for every `reach_ahead_step` from where the
/// body will be: a ballistic arc, the air steering left out. Only an edge
/// the body would be falling or near the top of its rise to.
fn edge_ahead(
    caster: &WorldCaster,
    now: &Reach,
    velocity: Vector3<f32>,
    gravity: Vector3<f32>,
    config: &LedgeConfig,
) -> Option<EdgeAhead> {
    let steps = (config.reach_ahead_time / config.reach_ahead_step).round() as usize;
    (1..=steps).find_map(|step| {
        let t = step as f32 * config.reach_ahead_step;
        let rising = velocity.y + gravity.y * t;
        if rising > config.catch_rise {
            return None;
        }
        let there = Reach {
            centre: now.centre + velocity * t + gravity * (0.5 * t * t),
            ..*now
        };
        let edge = edge_at_hands(caster, &there, config)?;
        let at = edge.in_world(caster.world)?;
        Some(EdgeAhead {
            hands: at.hands,
            in_seconds: t,
        })
    })
}

fn sensed(world: &PhysicsWorld, edge: super::Edge, room: [bool; 2]) -> Option<SensedEdge> {
    Some(SensedEdge {
        edge,
        at: edge.in_world(world)?,
        room,
    })
}

/// Whether the free hands catch `sensed`: the body faces it, is not rising
/// past it nor moving too fast against it. On foot the body is not rising,
/// whatever the contacts make of its vertical speed.
fn catchable(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    sensed: &SensedEdge,
    heading: Vector3<f32>,
    state: &LocomotionState,
    config: &LedgeConfig,
) -> bool {
    let facing = heading.dot(&-sensed.at.outward.into_inner())
        >= config.catch_facing_degrees.to_radians().cos();
    let grip = sensed.at.grip();
    let edge_velocity = sensed
        .edge
        .body
        .and_then(|handle| world.body(handle))
        .map_or(Vector3::zeros(), |edge| edge.velocity_at(grip));
    let relative = world
        .body(body)
        .map_or(Vector3::zeros(), |body| body.linear_velocity())
        - edge_velocity;
    let on_foot = matches!(state, LocomotionState::OnFoot(_));
    facing && (on_foot || relative.y <= config.catch_rise) && relative.norm() <= config.catch_speed
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
