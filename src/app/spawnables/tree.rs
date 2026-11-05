//! Tree spawnable: a lollipop tree, a slender trunk under one or more round
//! crowns (`plants::LollipopSpec`).
//!
//! The trunk and branches are one static body, so the trunk is solid. Each
//! crown is a clump of foliage held on its limb: drawn and dragging as a
//! shrub's leaves do, but a blast does not tear it loose. The tree is grown
//! from a seed taken from where it stands, so it is the same tree every
//! load.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::seed_from_ground;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{ModelInstance, Orientation, Position, Renderable};
use crate::core::error::EngineResult;
use crate::foliage::{Foliage, Rooting, SkinSpec};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::RigidBodyDesc;
use crate::plants::LollipopSpec;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::rendering::substance::{self, ColliderSubstance};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Leaves are matte, with a faint sheen off the tops of the sprigs.
const LEAF_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};
/// A smooth, pale trunk, as the painted topiary's are.
const BARK: Colour = Colour::new(0.58, 0.40, 0.25, 1.0);
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
        let root = Isometry3::from_parts(
            Translation3::new(self.pos.0, ground, self.pos.1),
            Default::default(),
        );

        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics.world.create_body(
                RigidBodyDesc::static_body().position(Point3::from(root.translation.vector)),
            );
            for collider in plant.limbs.colliders() {
                physics
                    .world
                    .attach_collider(body, collider.of(&substance::OAK));
            }
        }
        let (vertices, indices) = plant.limbs.mesh(BARK);
        let wood = Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
            vertices,
            indices,
            material: bark,
        }])]);
        let mut entities = vec![world
            .create_entity()
            .with(Position(root.translation.vector))
            .with(ModelInstance::new(Arc::new(wood)))
            .with(Renderable)
            .build()];

        for (index, mass) in plant.masses.into_iter().enumerate() {
            let pose = root * mass.pose;
            let skin_seed = seed.wrapping_add(index as u32 * 7919);
            let skin = SkinSpec::default().grow(&mass.shape, skin_seed);
            let clump =
                Foliage::grown(Arc::new(mass.shape), skin, leaves, Rooting::OnLimb { pose });
            entities.push(
                world
                    .create_entity()
                    .with(Position(pose.translation.vector))
                    .with(Orientation(pose.rotation))
                    .with(clump.model())
                    .with(clump)
                    .with(Renderable)
                    .build(),
            );
        }
        entities
    }
}
