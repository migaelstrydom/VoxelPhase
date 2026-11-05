//! Tree spawnable: a lollipop tree, a slender trunk under one or more round
//! crowns (`plants::LollipopSpec`), spawned as every plant is
//! (`spawnables::plant`). It is grown from a seed taken from where it
//! stands, so it is the same tree every load.

use serde::Deserialize;
use specs::{Entity, World};

use super::plant::{plant_materials, spawn_on_ground, PLANT_MATERIALS};
use super::shared::textures::seed_from_ground;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::plants::LollipopSpec;
use crate::rendering::material::MaterialId;

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
        PLANT_MATERIALS
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        plant_materials(ctx, seed_from_ground(self.pos, 1))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        spawn_on_ground(world, self.pos, materials, |seed| self.spec().grow(seed))
    }
}
