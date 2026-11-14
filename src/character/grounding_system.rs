use nalgebra::Vector3;
use specs::{Join, LendJoin, Read, ReadStorage, System, Write, WriteStorage};

use super::config::LocomotionConfig;
use super::grounding::Grounding;
use crate::components::RigidBodyComponent;
use crate::platform::MovingPlatform;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::traversal::{carriers, WorldCaster};

/// Derives [`Grounding`] from physics contacts, for every body that has one.
///
/// The Support Set is the single source of support in the engine, so this is
/// the only writer of `Grounding`: a rigged humanoid's answer comes from the
/// same contacts a roller's does. Foot probes remain a rangefinder for the
/// animator — where the ground is, and which way it faces at a landing target
/// nothing is touching yet — and no longer decide whether the character is on
/// it. See `docs/TRACTION_DRIVE_DESIGN.md` §6.6.
///
/// Contacts cannot say whether the support is under the body: a brink that
/// terrain rounds into a bevel holds a body as a slope would. So for a
/// grounded character whose config carries a `DropProbe`, this also looks
/// under its centre (`Grounding::over_drop`).
///
/// Contact grounding is truthful and chattery where reach-based grounding was
/// lenient and smooth; the forgiveness that costs is restored deliberately, by
/// `GroundForgiveness` on the FSM side.
pub struct ContactGroundingSystem;

impl<'a> System<'a> for ContactGroundingSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, LocomotionConfig>,
        ReadStorage<'a, MovingPlatform>,
        Option<Read<'a, TerrainWorld>>,
        WriteStorage<'a, Grounding>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physics_res, rigid_bodies, configs, platforms, terrain, mut groundings) = data;

        let grounded = physics_res.world.grounded_bodies();
        let world = &physics_res.world;
        let up = world
            .config()
            .gravity_direction()
            .map(|down| -down.into_inner());
        let carriers = carriers(&platforms, &rigid_bodies);

        for (rb, config, grounding) in (&rigid_bodies, (&configs).maybe(), &mut groundings).join() {
            let Some(support) = grounded.get(&rb.0) else {
                *grounding = Grounding::airborne();
                continue;
            };
            let over_drop = match (config, up, &terrain) {
                (Some(config), Some(up), Some(terrain)) => {
                    let caster = WorldCaster {
                        world,
                        scenery: &**terrain,
                        exclude: &[rb.0],
                        filter: world.filter_of(rb.0),
                        carriers: &carriers,
                    };
                    way_off_drop(&caster, rb, config, support.normal, up)
                }
                _ => None,
            };
            *grounding = Grounding::on(support.normal)
                .carried_by(support.surface_velocity)
                .over_drop(over_drop);
        }
    }
}

/// The way off a drop the body `rb` stands over, held up along
/// `support_normal`, if its config probes for drops.
fn way_off_drop(
    caster: &WorldCaster<'_>,
    rb: &RigidBodyComponent,
    config: &LocomotionConfig,
    support_normal: Vector3<f32>,
    up: Vector3<f32>,
) -> Option<Vector3<f32>> {
    let probe = config.drop_probe?;
    let centre = caster.world.body(rb.0)?.position();
    probe.way_off(
        caster,
        centre,
        support_normal,
        up,
        config.collider_half_height,
    )
}
