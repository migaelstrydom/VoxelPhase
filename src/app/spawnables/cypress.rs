//! Cypress spawnable: one tall, narrow crown on a short trunk
//! (`plants::CypressSpec`), spawned as every plant is
//! (`spawnables::plant`). It is grown from a seed taken from where it
//! stands, so it is the same tree every load.

use serde::Deserialize;
use specs::{Entity, World};

use super::plant::{plant_materials, spawn_on_ground, PLANT_MATERIALS};
use super::shared::textures::seed_from_ground;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::plants::CypressSpec;
use crate::rendering::material::MaterialId;

#[derive(Deserialize)]
pub struct CypressDef {
    /// Ground point (x, z) the trunk grows from. Y is the terrain's there.
    pub pos: (f32, f32),
    /// Height from the ground to the top of its leaves, in metres.
    #[serde(default = "CypressDef::default_height")]
    pub height: f32,
    /// Radius of the crown at its widest, in metres.
    #[serde(default = "CypressDef::default_radius")]
    pub radius: f32,
}

impl CypressDef {
    pub fn default_height() -> f32 {
        CypressSpec::default().height
    }

    pub fn default_radius() -> f32 {
        CypressSpec::default().radius
    }

    fn spec(&self) -> CypressSpec {
        CypressSpec {
            height: self.height,
            radius: self.radius,
            ..CypressSpec::default()
        }
    }
}

impl Spawnable for CypressDef {
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
