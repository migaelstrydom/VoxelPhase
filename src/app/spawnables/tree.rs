//! Tree spawnable: a lollipop tree, a slender trunk under one or more round
//! crowns (`plants::LollipopSpec`).
//!
//! Each limb is a body and each crown a clump of foliage on a core of its
//! own, welded together (`plants::Joinery`); the trunk is welded into the
//! ground until the ground under it is blown away, and a branch breaks off
//! under a blast. The tree is grown
//! from a seed taken from where it stands, so it is the same tree every
//! load.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::seed_from_ground;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::foliage::{Foliage, FoliageShape, Rooting, Skin, SkinSpec};
use crate::model::{Model, ModelPart};
use crate::plants::{Joinery, LollipopSpec};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Leaves are matte, with a faint sheen off the tops of the sprigs.
const LEAF_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};
/// A smooth, pale trunk, as the painted topiary's are.
const BARK: Colour = Colour::new(0.58, 0.40, 0.25, 1.0);
/// How far below the ground at its foot the trunk checks for terrain: inside
/// the solid, not on its surface.
const ANCHOR_PROBE: f32 = 0.1;
const BARK_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.7,
    metallic: 0.0,
};

#[derive(Deserialize)]
pub struct TreeDef {
    /// Ground point (x, z) the trunk grows from. Y is the terrain's there.
    pub pos: (f32, f32),
    /// Height from the ground to the top of its leaves, in metres.
    #[serde(default = "TreeDef::default_height")]
    pub height: f32,
    /// Radius of the main crown, in metres.
    #[serde(default = "TreeDef::default_crown_radius")]
    pub crown_radius: f32,
    /// How many crowns: one on the trunk, or more on branches from a fork.
    #[serde(default = "TreeDef::default_crowns")]
    pub crowns: usize,
}

impl TreeDef {
    pub fn default_height() -> f32 {
        LollipopSpec::default().height
    }

    pub fn default_crown_radius() -> f32 {
        LollipopSpec::default().crown_radius
    }

    pub fn default_crowns() -> usize {
        LollipopSpec::default().crowns
    }

    fn spec(&self) -> LollipopSpec {
        LollipopSpec {
            height: self.height,
            crown_radius: self.crown_radius,
            crowns: self.crowns,
            ..LollipopSpec::default()
        }
    }
}

impl Spawnable for TreeDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // The greens ride on the skin's vertices, the bark's colour on the
        // trunk's.
        let leaves = Material::coloured(Colour::WHITE)
            .with_finish(LEAF_FINISH)
            .as_foliage();
        let bark = Material::coloured(Colour::WHITE).with_finish(BARK_FINISH);
        Ok(vec![
            ctx.materials.register(leaves),
            ctx.materials.register(bark),
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ground = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        let Some(ground) = ground else {
            return Vec::new();
        };
        let (leaves, bark) = (materials[0], materials[1]);
        let seed = seed_from_ground(self.pos, 1);
        let plant = self.spec().grow(seed);
        let root = Isometry3::translation(self.pos.0, ground, self.pos.1);
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
            let (vertices, indices) = limb.mesh(BARK);
            let tube = Skin { vertices, indices }
                .moved(&(frame * Translation3::from(limb.midpoint().coords)));
            let mut primitives = vec![tube.primitive(bark)];
            // A lollipop's limb holds at most one crown: an entity carries
            // one clump of foliage.
            let mut crown = None;
            for m in (0..holders.len()).filter(|&m| holders[m] == index) {
                let Some((skin, shape)) = skins[m].take() else {
                    continue;
                };
                let offset = bodies.mass_offsets[m];
                primitives.push(skin.moved(&offset).primitive(leaves));
                debug_assert!(crown.is_none(), "a limb holds two crowns");
                crown = Some(Foliage::new(shape, Rooting::Loose { body, offset }));
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
            if let Some(crown) = crown {
                entity = entity.with(crown);
            }
            if index == 0 {
                // Let go when the ground under its foot is blown away.
                entity = entity.with(TerrainAnchored {
                    constraints: vec![bodies.rooting],
                    anchor_points: vec![Point3::new(self.pos.0, ground - ANCHOR_PROBE, self.pos.1)],
                    released_model: None,
                });
            }
            entities.push(entity.build());
        }
        entities
    }
}
