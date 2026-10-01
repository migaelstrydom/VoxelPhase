use nalgebra::{Point3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::{CharacterAnimator, CharacterRigConfig};
use crate::character::{
    AttitudeControl, CharacterIntent, CharacterState, Grounding, Immersion, LocomotionConfig,
};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::damage::{Health, Ragdoll};
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, ConstraintHandle, ConstraintKind, FrictionModel,
    PhysicsWorld, RigidBodyDesc, RigidBodyHandle, Volume,
};
use crate::player::Player;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// The figure inside the capsule, as the world weighs it and the water
/// floats it.
///
/// The capsule is a *bounding* volume — 1m tall and half a metre across —
/// for contacts, and a spindly humanoid fills a third to a half of it.
/// Declared as the capsule, the player either weighs as solid meat (131kg
/// at 800, enough to craze a block of ice by walking into it) or, lightened
/// to a believable weight, floats high on the water like a cork. The body
/// shape is the same height and as thick as the figure, so the capsule can
/// stay the contact envelope.
///
/// Everything about how the player *moves* is authored as a velocity or an
/// acceleration and multiplied by the mass where it is applied — see
/// `TractionPlanner::plan` and `Allowance` — so the mass decides what the
/// player weighs against the world and not how they handle: 56kg.
const FIGURE_RADIUS: f32 = 0.17;
/// A real swimmer, at about 985, floats with the head barely out, which on
/// this figure — a head sat on top of a 1m body — reads as drowning. This
/// floats it shoulder deep upright, at 0.68m, and awash lying down, which is
/// where a swimmer is.
const FIGURE_DENSITY: f32 = 700.0;

/// The figure inside a capsule `half_height` tall: what weighs the player and
/// what the water floats.
fn figure(half_height: f32) -> BulkShape {
    BulkShape::solid(
        Volume::centred(ColliderShape::Capsule {
            half_height,
            radius: FIGURE_RADIUS,
        }),
        FIGURE_DENSITY,
    )
}

/// The player's body as the physics world sees it: the capsule, the figure
/// inside it, and the constraint that holds it upright. Returns the body and
/// that constraint.
///
/// Everything the engine knows about the player is here and in
/// [`player_actuator`], so a harness that drives the player without the rest
/// of the game — `physics_fuzz`'s walker — drives the same body.
pub fn create_player_body(
    physics: &mut PhysicsWorld,
    position: Point3<f32>,
    locomotion: &LocomotionConfig,
) -> (RigidBodyHandle, ConstraintHandle) {
    let body_desc = RigidBodyDesc::dynamic()
        .bulk(figure(locomotion.collider_half_height))
        .position(position)
        .gravity_scale(1.0)
        .linear_damping(0.0)
        .angular_damping(0.95);
    let body = physics.create_body(body_desc);
    // A real material coefficient: it keeps the player planted on slopes
    // and lets moving surfaces (play-wheels, platforms) drag them
    // tangentially, and it means the same thing to whatever the player
    // leans on. Which contacts the player is allowed to draw it at is the
    // actuator's business — see `non_support_grip` in `player_actuator`.
    let collider_desc =
        ColliderDesc::capsule(locomotion.collider_half_height, locomotion.collider_radius)
            .restitution(0.0)
            .friction_model(FrictionModel::Isotropic(0.8));
    physics.attach_collider(body, collider_desc);
    physics
        .body_mut(body)
        .unwrap()
        .scale_local_inertia(Vector3::new(1.0, 50.0, 1.0));
    // Upright on land, laid over to swim: `CharacterControlSystem` sets
    // the pitch through `AttitudeControl`. Held so `DeathSystem` can
    // release it. Without that the corpse stays rigidly upright, which
    // reads as a bug rather than a death.
    let upright = physics.create_constraint(ConstraintKind::KeepAttitude {
        body,
        pitch: 0.0,
        compliance: 0.0,
        max_impulse: f32::INFINITY,
    });
    (body, upright)
}

/// How the player turns intent into momentum.
pub fn player_actuator(locomotion: &LocomotionConfig) -> Actuator {
    // Swimming is steered out of the same allowance as the air: nothing holds
    // a swimmer up but the water, and the stroke is the one authority it has.
    // The budget is the larger of the two; each asks only for its own rate.
    let swim_accel = locomotion.swim.accel(true);
    // The yaw allowance's ceiling, in rad/s². A capsule's supports are a point
    // and a torsional row bounded by `μ·N·r` therefore has nothing to bear on
    // (§6.2), so this is the whole of the player's angular authority. It is the
    // acceleration the old reactionless yaw drive was bounded by, so a turn
    // costs what it always did.
    const TURN_AUTHORITY: f32 = 500.0;

    // Grip nothing that is not holding them up, so jumps along vertical
    // surfaces are not grabbed; and push five times harder through the
    // contacts that do. The gain is the game's decision that the player is
    // a cartoon: an honest 0.8 against gravity caps acceleration at
    // 7.85 m/s², which was measured in the old engine and is far too slow
    // to play. See `Actuator::drive_gain`.
    // The airborne half of the same character. Nothing is holding the
    // player up mid-jump, so every scrap of air steering, jump shaping and
    // turning is momentum the world did not have — granted here, by name,
    // with a ceiling on each. See `Actuator::allowance`.
    Actuator::character()
        .with_non_support_grip(0.0)
        .with_drive_gain(5.0)
        .with_allowance(Allowance::character(
            locomotion.air_steer_speed.max(swim_accel),
            TURN_AUTHORITY,
            locomotion.jump_speed,
        ))
}

/// Spawns the player entity with all required components.
pub fn spawn_player(world: &mut World, initial_pos: Point3<f32>) -> Entity {
    let locomotion = LocomotionConfig::player();
    /// Facing the player spawns with, matching the `Rotation` below.
    const INITIAL_YAW: f32 = 0.0;

    let rig_config = CharacterRigConfig::default();
    // The capsule's centre rides half its height above the floor; the rig's
    // pelvis belongs lower than that, on bent knees. The animator owns the
    // difference.
    let animator = CharacterAnimator::new(
        rig_config,
        initial_pos,
        locomotion.collider_half_height,
        INITIAL_YAW,
    );

    let (body_handle, upright_handle) = {
        let mut physics = world.write_resource::<PhysicsResource>();
        create_player_body(&mut physics.world, initial_pos, &locomotion)
    };
    let actuator = player_actuator(&locomotion);

    world
        .create_entity()
        .with(Player)
        .with(CharacterIntent::default())
        .with(CharacterState::default())
        .with(locomotion)
        .with(Grounding::default())
        .with(Immersion::default())
        .with(AttitudeControl {
            constraint: upright_handle,
        })
        // The player's corpse is never despawned — death should be a state to
        // recover from, not an entity disappearing out from under the camera.
        .with(Health::persistent(100.0))
        .with(Ragdoll::new(vec![upright_handle]))
        .with(animator)
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Rotation(0.0))
        .with(Orientation::default())
        .with(Renderable)
        .with(SensorSet::default())
        .with(ContactCandidates::default())
        .with(RigidBodyComponent(body_handle))
        .with(actuator)
        .with(DriveIntent::default())
        .with(BodyMotion::default())
        .build()
}

#[cfg(test)]
mod tests {
    use nalgebra::UnitQuaternion;

    use super::*;
    use crate::water::buoyancy::{compute_buoyancy, StillWater, FLUID_DENSITY};

    const GRAVITY: f32 = 9.81;

    /// The depth of still water that lifts the figure, standing on the
    /// floor, off its feet.
    fn float_depth(config: &LocomotionConfig) -> f32 {
        let half_height = config.collider_half_height;
        let shape = ColliderShape::Capsule {
            half_height,
            radius: FIGURE_RADIUS,
        };
        let weight = figure(half_height).mass().unwrap() * GRAVITY;
        let lift = |depth: f32| {
            compute_buoyancy(
                Point3::new(0.0, half_height, 0.0),
                UnitQuaternion::identity(),
                &shape,
                FLUID_DENSITY,
                Vector3::new(0.0, -GRAVITY, 0.0),
                GRAVITY,
                None,
                &StillWater {
                    surface: depth,
                    floor: 0.0,
                },
            )
            .map_or(0.0, |f| f.buoyancy_force.y)
        };
        let (mut shallow, mut deep) = (0.0, 2.0 * half_height);
        for _ in 0..40 {
            let mid = 0.5 * (shallow + deep);
            if lift(mid) < weight {
                shallow = mid;
            } else {
                deep = mid;
            }
        }
        shallow
    }

    #[test]
    fn the_figure_weighs_what_a_player_should() {
        let config = LocomotionConfig::player();
        let mass = figure(config.collider_half_height).mass().unwrap();
        assert!((50.0..65.0).contains(&mass), "{mass} kg");
    }

    #[test]
    fn the_player_swims_just_before_the_water_lifts_it_off_its_feet() {
        let config = LocomotionConfig::player();
        let floats_at = float_depth(&config);
        let swim = config.swim;
        assert!(
            swim.stand_depth < swim.swim_depth && swim.swim_depth < floats_at,
            "stand {} swim {} float {floats_at}",
            swim.stand_depth,
            swim.swim_depth
        );
        assert!(
            floats_at - swim.swim_depth < 0.12,
            "swims only at {} but floats at {floats_at}: wades a stride on tiptoe",
            swim.swim_depth
        );
    }
}
