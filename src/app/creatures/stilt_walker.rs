//! Spawning a stilt walker: the body, drive and wits every creature that
//! walks on the stilt rig shares.
//!
//! ```text
//!   StiltWalker { rig, costume, speed, sight, peck, health, brain }
//!        │ spawn
//!        ▼
//!   capsule body kept upright ── StiltAnimator (legs, and the eye or costume)
//!   CharacterIntent ◀── Brain ◀── Perception        MeleeAttack, Health
//! ```
//!
//! A def says what its creature is (its proportions, what it wears, how fast
//! and how hardy it is) and this builds it, so that a new walker is a new
//! costume and a handful of numbers.

use std::sync::Arc;

use nalgebra::{Point3, UnitVector3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::stilts::{Costume, Garment, StiltAnimator, StiltRigConfig};
use crate::app::sculpt::{painted_meshes, ContactShade, Mottle, SculptedPart};
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::creature::{Brain, MeleeAttack, Perception};
use crate::damage::{Health, Ragdoll};
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, ConstraintKind, FrictionModel, RigidBodyDesc, Volume,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Half-width of the thing a walker pecks at. The attack has to cover the
/// target's own girth or a walker standing against the player misses.
const TARGET_HALF_WIDTH: f32 = 0.45;

/// Where a walker holds station while attacking, between the distance at
/// which the two bodies touch and the distance its peck reaches.
///
/// Halfway, and both bounds matter. Any closer and the solver is what ends
/// up deciding the standoff — the creature walks into the player, gets
/// shoved out, and walks back in. Any further and the peck cannot land
/// from where the brain parks it.
const STANDOFF_BETWEEN_CONTACT_AND_REACH: f32 = 0.5;

/// Seconds a walker's body lies where it fell before it is cleared away.
const CORPSE_SECONDS: f32 = 6.0;

/// The material a costume's parts name. A rig's mesh is drawn in its
/// vertices' colours alone, so this is never looked up.
const PAINT: MaterialId = MaterialId(0);

/// A costume's colour, as a sculpted part's look.
pub fn paint(r: f32, g: f32, b: f32) -> (Colour, MaterialId) {
    (Colour::new(r, g, b, 1.0), PAINT)
}

/// `parts` as one garment, mottled and shaded where they meet.
pub fn garment(parts: &[SculptedPart], mottle: &Mottle, shade: &ContactShade) -> Garment {
    let mut garment = Garment::default();
    for mesh in painted_meshes(parts, mottle, shade) {
        let first = garment.vertices.len() as u32;
        garment.vertices.extend(mesh.vertices);
        garment
            .indices
            .extend(mesh.indices.iter().map(|i| i + first));
    }
    garment
}

/// The shade where one part of a costume meets another.
pub const COSTUME_SHADE: ContactShade = ContactShade {
    reach: 0.04,
    depth: 0.6,
    floor: 0.3,
};

/// A walker's strike with its head.
#[derive(Clone, Copy, Debug)]
pub struct Peck {
    /// Hit points it takes off.
    pub damage: f32,
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
    /// What it wears in place of the peeper's eye, if anything.
    pub costume: Option<Arc<Costume>>,
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
    /// Hit points it can take; `None` for one that cannot be killed.
    pub health: Option<f32>,
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
    let eye = shoulder + rig.neck_length;
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
                radius: rig.eye_radius,
            },
            at(eye, 0.0),
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
        let radius = rig.eye_radius;
        let locomotion = LocomotionConfig::creature(self.speed, radius, clearance);
        let (air_steer_speed, jump_speed) = (locomotion.air_steer_speed, locomotion.jump_speed);

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.at.0, self.at.1)
        };
        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let initial_pos = Point3::new(self.at.0, surface_y + clearance, self.at.1);
        let mut animator = StiltAnimator::new(rig, initial_pos, clearance, 0.0);
        if let Some(costume) = self.costume {
            animator = animator.wearing(costume);
        }

        // Reach is the rig's own geometry plus the target's girth, so
        // lengthening the neck lengthens the attack by itself. The standoff
        // the brain holds is derived from the same two numbers, which is
        // what keeps the creature out of the player's collider while still
        // close enough to connect.
        let contact = radius + TARGET_HALF_WIDTH;
        let reach = rig.strike_extent() + TARGET_HALF_WIDTH;
        let standoff = contact + (reach - contact) * STANDOFF_BETWEEN_CONTACT_AND_REACH;

        let (body_handle, keep_upright) = {
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
            // so without this it topples the first time it clips a rock.
            // Released on death, which is what lets the legs fold.
            let keep_upright = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            (body_handle, keep_upright)
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
            .with(animator)
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
            .with((self.brain)(standoff));
        if let Some(peck) = self.peck {
            entity = entity
                .with(MeleeAttack::new(peck.damage, reach).with_timing(peck.windup, peck.recovery));
        }
        if let Some(health) = self.health {
            entity = entity
                .with(Health::new(health, CORPSE_SECONDS))
                .with(Ragdoll::new(vec![keep_upright]));
        }
        vec![entity.build()]
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    /// A costume that panics while it is sculpted, or comes out empty or
    /// with a part flung off to nowhere, would only show when its creature
    /// was first met in a level.
    #[test]
    fn every_walkers_costume_is_whole_and_sits_on_its_head() {
        use crate::app::creatures::{BellCarrierDef, EggWalkerDef, FishWalkerDef, StalkerDef};
        let costumes = [
            ("stalker", StalkerDef::costume(), StalkerDef::rig()),
            ("egg", EggWalkerDef::costume(), EggWalkerDef::rig()),
            ("fish", FishWalkerDef::costume(), FishWalkerDef::rig()),
            ("bell", BellCarrierDef::costume(), BellCarrierDef::rig()),
        ];
        for (name, costume, rig) in costumes {
            assert!(!costume.head.indices.is_empty(), "the {name} has no head");
            for garment in [&costume.head, &costume.body] {
                let count = garment.vertices.len() as u32;
                assert!(garment.indices.iter().all(|index| *index < count));
                for vertex in &garment.vertices {
                    assert!(vertex.pos.iter().all(|x| x.is_finite()));
                    assert!(
                        vertex.pos.norm() < rig.body_height(),
                        "part of the {name}'s costume is {} m from where it is worn",
                        vertex.pos.norm()
                    );
                }
            }
        }
    }

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
            ColliderDesc::capsule(clearance, rig.eye_radius)
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
