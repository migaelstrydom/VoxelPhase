//! Spawning a stilt walker: the body, drive and wits every creature that
//! walks on the stilt rig shares.
//!
//! ```text
//!   StiltWalker { rig, dress, speed, sight, peck, health, brain }
//!        │ spawn
//!        ▼
//!   capsule body held upright ── StiltAnimator ──poses──▶ PosedModel (the dress)
//!   CharacterIntent ◀── Brain ◀── Perception        MeleeAttack, Harm
//! ```
//!
//! A def says what its creature is (its proportions, its dress, how fast
//! and how hardy it is) and this builds it, so that a new walker is a new
//! sculpture and a handful of numbers.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::anatomy::{self, LIMB_GRAIN};
use crate::animation::stilts::{Dress, Joint, StiltAnimator, StiltRigConfig, Worn};
use crate::animation::FootSide;
use crate::character::locomotion::DownConfig;
use crate::character::{
    AttitudeControl, CharacterIntent, CharacterState, Grounding, LocomotionConfig,
};
use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, ProceduralMesh, Renderable, RigidBodyComponent,
    Rotation, Velocity,
};
use crate::creature::{Brain, MeleeAttack, Perception, Rush, RushAim, Shell};
use crate::damage::Ragdoll;
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::harm::{Death, Harm, HarmConfig};
use crate::model::MeshPrimitive;
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, ConstraintKind, FrictionModel, RigidBodyDesc, Volume,
};
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Half-width of the thing a walker strikes at, m: the player's, with room
/// to spare, so that it holds station clear of them.
const TARGET_HALF_WIDTH: f32 = 0.45;

/// How hard a walker's legs drive it into a lunge, m/s²: about what its
/// traction drive gives it on the lawn, where a bell carrier gains its
/// 4 m/s in under a fifth of a second ([`Rush::grip`]).
const LUNGE_GRIP: f32 = 22.0;

/// How far past its standoff a walker still throws its blow, as a share of
/// it: as far as its brain lets it drift while it circles.
const REACH_PAST_STANDOFF: f32 = 0.2;

/// Seconds a walker's body lies where it fell before it is cleared away.
const CORPSE_SECONDS: f32 = 6.0;

/// A pair of bare human legs for a walker of proportions `rig`, each
/// thigh `girth` thick at the hip and each foot `foot` long: thighs, shins
/// and feet, each worn on its own bone.
pub fn bare_legs(rig: &StiltRigConfig, girth: f32, foot: f32) -> Vec<(Worn, Vec<MeshPrimitive>)> {
    let thigh = anatomy::flesh(&anatomy::thigh(rig.upper_leg_length, girth), LIMB_GRAIN);
    let shin = anatomy::flesh(&anatomy::shin(rig.lower_leg_length, girth), LIMB_GRAIN);
    let mut legs = Vec::new();
    for side in [FootSide::Left, FootSide::Right] {
        legs.push((Worn::on(Joint::Thigh(side)), vec![thigh.clone()]));
        legs.push((Worn::on(Joint::Shin(side)), vec![shin.clone()]));
        legs.push((Worn::on(Joint::Foot(side)), vec![anatomy::foot(foot, side)]));
    }
    legs
}

/// A walker's strike: its head thrown at what it attacks, and its body
/// behind it ([`Rush`]).
#[derive(Clone, Copy, Debug)]
pub struct Peck {
    /// Hit points it takes off.
    pub damage: f32,
    /// The speed it lunges at, m/s: what its legs drive its body toward,
    /// along the way it faces, as it throws the blow. Their grip on the
    /// ground decides how soon it gets there.
    pub lunge: f32,
    /// Seconds of rearing back before it lands: the warning.
    pub windup: f32,
    /// Seconds before it can strike again.
    pub recovery: f32,
}

/// A creature on the stilt rig, ready to spawn.
pub struct StiltWalker {
    /// Where it stands, (x, z): its height comes from the ground there.
    pub at: (f32, f32),
    /// Its proportions.
    pub rig: StiltRigConfig,
    /// What it is seen as, in the level's materials.
    pub dress: Arc<Dress>,
    /// Top speed on the flat, m/s.
    pub speed: f32,
    /// How far it sees, m.
    pub sight_range: f32,
    /// Density of the capsule it collides as, kg/m³.
    pub density: f32,
    /// How hard it can turn, rad/s².
    pub turn_authority: f32,
    /// Its strike. A walker without one only crowds its target.
    pub peck: Option<Peck>,
    /// How the blows it takes are judged; `None` for one nothing harms.
    pub harm: Option<HarmConfig>,
    /// Its shell, if it wears one, which the blow that kills it breaks off
    /// in shards: the parts of its dress worn brittle.
    pub shell: Option<Shell>,
    /// Its wits, given the distance it holds from what it attacks.
    pub brain: fn(f32) -> Brain,
}

/// What water and wind see of a stilt walker: two stilts, a haunch, a neck and a
/// head, and not the air between them that the capsule round it also holds.
///
/// It weighs what the capsule says, which is far more than this displaces,
/// so it wades on its stilts until the water reaches its eye, and past that
/// walks the bottom. The legs are straight and the neck upright: a rest pose
/// is all a rigid body can carry.
fn stilts_and_head(rig: &StiltRigConfig, clearance: f32) -> BulkShape {
    // Heights above the foot centres, which sit `clearance` below the
    // capsule's centre.
    let hip = rig.standing_height();
    let shoulder = hip + rig.haunch_height;
    let crown = shoulder + rig.neck_length;
    let at = |height: f32, across: f32| Vector3::new(across, height - clearance, 0.0);
    let capsule = |length: f32, radius: f32| ColliderShape::Capsule {
        half_height: 0.5 * length + radius,
        radius,
    };
    let stilt = capsule(hip, rig.leg_radius);
    BulkShape::displacing(vec![
        Volume::at(stilt.clone(), at(0.5 * hip, -0.5 * rig.hip_width)),
        Volume::at(stilt, at(0.5 * hip, 0.5 * rig.hip_width)),
        Volume::at(
            capsule(rig.haunch_height, rig.haunch_radius),
            at(hip + 0.5 * rig.haunch_height, 0.0),
        ),
        Volume::at(
            capsule(rig.neck_length, rig.neck_radius),
            at(shoulder + 0.5 * rig.neck_length, 0.0),
        ),
        Volume::at(
            ColliderShape::Sphere {
                radius: rig.head_radius,
            },
            at(crown, 0.0),
        ),
    ])
}

impl StiltWalker {
    /// Stand it on the ground at its place. Nothing, if there is no ground
    /// there: the level moved and its spot went with it, and dropping it
    /// silently beats spawning something that falls forever.
    pub fn spawn(self, world: &mut World) -> Vec<Entity> {
        let rig = self.rig;
        // The capsule covers the whole creature, so its centre — and with
        // it the character's ground clearance — sits at half its height.
        let clearance = rig.body_height() * 0.5;
        let radius = rig.head_radius;
        let mut locomotion = LocomotionConfig::creature(self.speed, radius, clearance);
        if let Some(peck) = self.peck {
            locomotion.sprint_speed_mul = peck.lunge / self.speed;
        }
        let (air_steer_speed, jump_speed) = (locomotion.air_steer_speed, locomotion.jump_speed);

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.at.0, self.at.1)
        };
        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let initial_pos = Point3::new(self.at.0, surface_y + clearance, self.at.1);
        let animator =
            StiltAnimator::new(rig, initial_pos, clearance, 0.0).wearing(self.dress.clone());
        // Posed as it stands, so that a level looked at without being run
        // shows it.
        let posed = PosedModel {
            model: self.dress.model.clone(),
            poses: animator.poses_from(&initial_pos.coords),
            frame: PoseFrame::Position,
        };
        let drape = self.dress.drape.is_some().then(|| {
            let mut mesh = ProceduralMesh::default();
            animator.drape_into(
                &initial_pos.coords,
                &UnitQuaternion::identity(),
                &mut mesh.primitives,
            );
            mesh
        });

        // It holds station a run-up away from touching its target: what
        // its legs need to bring its body up to its lunge speed. Any nearer
        // and its blow is a push; it backs off to there before it throws
        // one, and throws it from anywhere about there.
        let contact = radius + TARGET_HALF_WIDTH;
        let rush = |speed: f32, carry: f32| Rush {
            speed,
            grip: LUNGE_GRIP,
            carry,
            radius,
            half_height: clearance,
            aim: RushAim::Facing,
        };
        let run_up = self.peck.map_or(0.0, |peck| rush(peck.lunge, 0.0).run_up());
        let standoff = contact + run_up;
        let reach = standoff * (1.0 + REACH_PAST_STANDOFF);
        // It keeps the room from its kin that the two of them need to turn
        // aside from each other at a walk.
        let spacing = 2.0 * radius + self.speed * self.speed / LUNGE_GRIP;

        let (body_handle, upright) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(initial_pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.0)
                    .angular_damping(0.95),
            );
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::capsule(clearance, radius)
                    .density(self.density)
                    .restitution(0.0)
                    .friction_model(FrictionModel::Isotropic(0.8))
                    .bulk(stilts_and_head(&rig, clearance)),
            );
            // A capsule has nothing to resist a torque about its long axis,
            // so without this it topples the first time it clips a rock. Let
            // go when it is knocked down (`AttitudeControl`), as the
            // player's is, and for good when it dies.
            let upright = physics
                .world
                .create_constraint(ConstraintKind::KeepAttitude {
                    body: body_handle,
                    pitch: 0.0,
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            // It gets up as the player does, however heavy and tall it is.
            let mass = physics
                .world
                .body(body_handle)
                .map_or(0.0, |body| body.mass());
            let gravity = physics.world.config().gravity.norm();
            locomotion.down = DownConfig::tipped_at(mass * gravity * clearance);
            (body_handle, upright)
        };

        let mut entity = world
            .create_entity()
            // Intent is the seam: `BrainSystem` writes it, the shared
            // character control chain reads it. Neither knows about the
            // other, and neither knows what this one has for a head.
            .with(CharacterIntent::default())
            .with(CharacterState::default())
            .with(locomotion)
            .with(Grounding::default())
            .with(AttitudeControl {
                constraint: upright,
            })
            .with(animator)
            .with(posed)
            .with(Position(initial_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Rotation(0.0))
            .with(Orientation::default())
            .with(Renderable)
            .with(SensorSet::default())
            .with(ContactCandidates::default())
            .with(RigidBodyComponent(body_handle))
            .with(
                Actuator::character()
                    .with_non_support_grip(0.0)
                    .with_drive_gain(5.0)
                    .with_allowance(Allowance::character(
                        air_steer_speed,
                        self.turn_authority,
                        jump_speed,
                    )),
            )
            .with(DriveIntent::default())
            .with(BodyMotion::default())
            .with(Perception::ground_creature(self.sight_range))
            .with((self.brain)(standoff).keeping_apart(spacing));
        if let Some(peck) = self.peck {
            entity = entity.with(
                MeleeAttack::new(peck.damage, reach, rush(peck.lunge, reach))
                    .with_timing(peck.windup, peck.recovery),
            );
        }
        if let Some(config) = self.harm {
            entity = entity
                .with(Harm {
                    config,
                    death: Death::Dies {
                        lingers: CORPSE_SECONDS,
                    },
                    ..Harm::default()
                })
                .with(Ragdoll::new(vec![upright]));
        }
        if let Some(drape) = drape {
            entity = entity.with(drape);
        }
        if let Some(shell) = self.shell {
            entity = entity.with(Shell {
                parts: self.dress.brittle_parts(),
                ..shell
            });
        }
        vec![entity.build()]
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    /// The lightest walker's density, kg/m³: the peeper's.
    const PEEPER_DENSITY: f32 = 300.0;
    use crate::physics::{PhysicsWorld, RigidBodyHandle};
    use crate::water::buoyancy::{lift, StillWater};

    /// A peeper's body standing with its feet on a floor at zero.
    fn walker(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let rig = StiltRigConfig::default();
        let clearance = rig.body_height() * 0.5;
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, clearance, 0.0)));
        world.attach_collider(
            body,
            ColliderDesc::capsule(clearance, rig.head_radius)
                .density(PEEPER_DENSITY)
                .bulk(stilts_and_head(&rig, clearance)),
        );
        body
    }

    #[test]
    fn a_walker_as_light_as_a_peeper_wades_on_its_stilts_and_is_never_floated_off_them() {
        let mut world = PhysicsWorld::default();
        let body = walker(&mut world);
        let weight = world.body(body).unwrap().mass() * 9.81;
        let rig = StiltRigConfig::default();

        // Knee deep, the capsule round it would already have floated it.
        let knee_deep = StillWater {
            surface: 0.5 * rig.standing_height(),
            floor: 0.0,
        };
        assert!(lift(&world, body, &knee_deep) < 0.05 * weight);
        // Over its eye, it still keeps its feet.
        let drowned = StillWater {
            surface: 2.0 * rig.body_height(),
            floor: 0.0,
        };
        let fully = lift(&world, body, &drowned);
        assert!(fully < weight, "lift {fully} N against weight {weight} N");
    }
}
