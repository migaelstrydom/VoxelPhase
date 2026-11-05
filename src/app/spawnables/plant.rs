//! A grown plant spawned into the world: its bodies built (`plants::Joinery`)
//! and an entity per limb, drawing the limb and the crown it holds.
//!
//! Shared by every kind of plant's spawnable; each finds its ground, grows
//! its `Plant` and hands it here.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::seed_from_ground;
use super::spawnable::MaterialCtx;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::foliage::{Foliage, FoliageShape, Rooting, Skin, SkinSpec};
use crate::model::{Model, ModelPart};
use crate::plants::{BarkSpec, HeldCrown, Joinery, Plant};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Leaves are matte, with a faint sheen off the tops of the sprigs.
const LEAF_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};
/// Bark is smooth and dull, the painted trees' trunks catching only a soft
/// sheen.
const BARK_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.75,
    metallic: 0.0,
};
/// How far below the ground at its foot the trunk checks for terrain: inside
/// the solid, not on its surface.
const ANCHOR_PROBE: f32 = 0.1;
/// How many materials a plant registers: its leaves, then its bark.
pub const PLANT_MATERIALS: usize = 2;

/// A plant's materials, leaves then bark, for one grown from `seed`: its bark
/// is a tile of its own, painted from that seed, with its relief in alpha.
pub fn plant_materials(ctx: &mut MaterialCtx, seed: u32) -> EngineResult<Vec<MaterialId>> {
    // The greens ride on the skin's vertices.
    let leaves = Material::coloured(Colour::WHITE)
        .with_finish(LEAF_FINISH)
        .as_foliage();
    let spec = BarkSpec::default();
    let size = spec.texture_size;
    let tile = ctx
        .textures
        .create_from_rgba(size, size, &spec.texture(seed), true)?;
    let bark = Material::textured(tile)
        .with_finish(BARK_FINISH)
        .with_relief(spec.relief_depth);
    Ok(vec![
        ctx.materials.register(leaves),
        ctx.materials.register(bark),
    ])
}

/// The plant `grow` makes from the seed of where it stands, spawned on the
/// ground at `pos`, drawn in `materials` ([`plant_materials`]). Nothing if
/// there is no ground there.
pub fn spawn_on_ground(
    world: &mut World,
    pos: (f32, f32),
    materials: &[MaterialId],
    grow: impl FnOnce(u32) -> Plant,
) -> Vec<Entity> {
    let ground = {
        let terrain = world.read_resource::<TerrainWorld>();
        terrain.mesh_surface_height_at(pos.0, pos.1)
    };
    let Some(ground) = ground else {
        return Vec::new();
    };
    let seed = seed_from_ground(pos, 1);
    let look = PlantLook {
        leaves: materials[0],
        bark: materials[1],
    };
    let root = Isometry3::translation(pos.0, ground, pos.1);
    let anchor = Point3::new(pos.0, ground - ANCHOR_PROBE, pos.1);
    spawn_plant(world, grow(seed), root, seed, look, anchor)
}

/// The materials a plant is drawn in.
#[derive(Debug, Clone, Copy)]
pub struct PlantLook {
    /// The leaves: greens ride on the skin's vertices.
    pub leaves: MaterialId,
    /// The limbs.
    pub bark: MaterialId,
}

/// `plant` built and spawned with its frame at `root`, its skins grown from
/// `seed`. The trunk is welded into the ground, let go when the terrain at
/// `anchor` is blown away.
pub fn spawn_plant(
    world: &mut World,
    plant: Plant,
    root: Isometry3<f32>,
    seed: u32,
    look: PlantLook,
    anchor: Point3<f32>,
) -> Vec<Entity> {
    let PlantLook { leaves, bark } = look;
    let bodies = {
        let mut physics = world.write_resource::<PhysicsResource>();
        Joinery::default().build(&mut physics.world, &plant, root)
    };
    let holders: Vec<usize> = plant.masses.iter().map(|mass| mass.limb).collect();
    let mut skins: Vec<Option<(Skin, Arc<FoliageShape>)>> = plant
        .masses
        .into_iter()
        .enumerate()
        .map(|(index, mass)| {
            let skin_seed = seed.wrapping_add(index as u32 * 7919);
            let skin = SkinSpec::default().grow(&mass.shape, skin_seed);
            Some((skin, Arc::new(mass.shape)))
        })
        .collect();
    let mut entities = Vec::new();
    for (index, limb) in plant.limbs.limbs.iter().enumerate() {
        let body = bodies.limbs[index];
        let frame = bodies.limb_frames[index];
        let pose = root * frame.inverse();
        let (vertices, indices) = BarkSpec::default().mesh(limb, seed.wrapping_add(index as u32));
        let tube =
            Skin { vertices, indices }.moved(&(frame * Translation3::from(limb.midpoint().coords)));
        let wood = tube.primitive(bark);
        let mut primitives = vec![wood.clone()];
        // A lollipop's limb holds at most one crown: an entity carries
        // one clump of foliage.
        let mut crown = None;
        for m in (0..holders.len()).filter(|&m| holders[m] == index) {
            let Some((skin, shape)) = skins[m].take() else {
                continue;
            };
            let offset = bodies.mass_offsets[m];
            let held = HeldCrown {
                colliders: bodies.mass_colliders[m].clone(),
                skin: skin.clone().model(leaves),
                wood: wood.clone(),
            };
            primitives.push(skin.moved(&offset).primitive(leaves));
            debug_assert!(crown.is_none(), "a limb holds two crowns");
            crown = Some((Foliage::new(shape, Rooting::Loose { body, offset }), held));
        }
        let model = Model::flat(vec![ModelPart::new(primitives)]);
        let mut entity = world
            .create_entity()
            .with(Position(pose.translation.vector))
            .with(Orientation(pose.rotation))
            .with(Velocity(Vector3::zeros()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(Arc::new(model)))
            .with(Renderable);
        if let Some((clump, held)) = crown {
            entity = entity.with(clump).with(held);
        }
        if index == 0 {
            // Let go when the ground under its foot is blown away.
            entity = entity.with(TerrainAnchored {
                constraints: vec![bodies.rooting],
                anchor_points: vec![anchor],
                released_model: None,
            });
        }
        entities.push(entity.build());
    }
    entities
}

#[cfg(test)]
mod tests {
    use specs::{Join, RunNow};

    use super::*;
    use crate::debug::DebugLines;
    use crate::explosion::Explosion;
    use crate::particles::ParticleEmitter;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{ConstraintKind, SequentialStepper, Stepper};
    use crate::plants::{CrownBreakSystem, HeldCrown, LollipopSpec};

    fn world() -> World {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Orientation>();
        world.register::<Velocity>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<TerrainAnchored>();
        world.register::<Foliage>();
        world.register::<HeldCrown>();
        world.register::<Explosion>();
        world.register::<ParticleEmitter>();
        world.insert(PhysicsResource::default());
        world
    }

    /// A blast beside a tree's crown breaks it off: the crown is a loose
    /// clump on a core of its own, and the trunk, bare, still stands where
    /// it grew, its weld into the ground on the same point of it.
    #[test]
    fn a_blast_breaks_a_crown_off_and_leaves_the_trunk_standing() {
        let mut world = world();
        let plant = LollipopSpec::default().grow(2);
        let crown_at = plant.masses[0].pose.translation.vector;
        let look = PlantLook {
            leaves: MaterialId(0),
            bark: MaterialId(1),
        };
        let foot = Point3::new(0.0, -0.3, 0.0);
        let entities = spawn_plant(&mut world, plant, Isometry3::identity(), 2, look, foot);
        let trunk = world
            .read_storage::<RigidBodyComponent>()
            .get(entities[0])
            .unwrap()
            .0;
        let weld = world
            .read_storage::<TerrainAnchored>()
            .get(entities[0])
            .unwrap()
            .constraints[0];
        world
            .create_entity()
            .with(Explosion::new(Point3::from(
                crown_at + Vector3::new(1.0, 0.0, 0.0),
            )))
            .build();

        CrownBreakSystem.run_now(&world);
        world.maintain();

        assert!(
            world.read_storage::<HeldCrown>().is_empty(),
            "the crown is still held"
        );
        let loose: Vec<_> = (&world.read_storage::<Foliage>())
            .join()
            .filter(|clump| clump.body().is_some_and(|b| b != trunk))
            .map(|clump| clump.body().unwrap())
            .collect();
        assert_eq!(loose.len(), 1, "no loose clump came of it");
        {
            let physics = world.read_resource::<PhysicsResource>();
            assert_eq!(physics.world.body(trunk).unwrap().colliders().len(), 1);
            let crown = physics.world.body(loose[0]).unwrap().position();
            assert!(
                (crown.coords - crown_at).norm() < 0.05,
                "the crown came loose at {crown:?}"
            );
        }

        let mut physics = world.write_resource::<PhysicsResource>();
        let trunk_at = physics.world.body(trunk).unwrap().position();
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        let ground = FlatQuadGeometry::new(50.0);
        let mut lines = DebugLines::default();
        physics.world.wake_body(trunk);
        for _ in 0..120 {
            stepper.step(
                &mut physics.world,
                1.0 / 60.0,
                &ground,
                &[],
                &[],
                &mut lines,
            );
            lines.clear();
        }
        let body = physics.world.body(trunk).unwrap();
        let ConstraintKind::Fixed { local_anchor_b, .. } =
            physics.world.constraint(weld).unwrap().kind
        else {
            unreachable!("the trunk's weld")
        };
        let held = body.position() + body.rotation() * local_anchor_b;
        assert!(
            (held - foot).norm() < 0.01,
            "the weld is at {held:?}, not the foot"
        );
        let sagged = (body.position() - trunk_at).norm();
        assert!(sagged < 0.01, "the bare trunk moved {sagged:.3} m");
    }
}
