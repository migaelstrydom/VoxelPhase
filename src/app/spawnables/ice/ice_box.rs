//! A single loose block of ice, of any rectangular-prism proportions.
//!
//! Authored by half-extents rather than by one size, so the same object covers
//! an ice cube, a frozen paving slab and a pillar of ice. The block's
//! behaviour — light, see-through, and almost frictionless — comes entirely
//! from [`super::block`]; this file is the level-facing shape of it.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::super::shared::orientation::Yaw;
use super::super::shared::textures::seed_from_position;
use super::super::{MaterialCtx, Spawnable};
use super::block::{ice_material, texture_spread, IceBlock};
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;

#[derive(Deserialize)]
pub struct IceBoxDef {
    pub pos: (f32, f32, f32),

    /// Half-extents of the block, before the edges are cut back.
    #[serde(default = "IceBoxDef::default_half_extents")]
    pub half_extents: (f32, f32, f32),

    /// Rotation about `+Y`, in degrees.
    #[serde(default)]
    pub yaw: f32,
}

impl IceBoxDef {
    /// A cube the size of a large ice cube out of a freezer tray.
    pub fn default_half_extents() -> (f32, f32, f32) {
        (0.4, 0.4, 0.4)
    }

    /// The authored half-extents as a vector. The one place the tuple is
    /// unpacked, because the texture spread and the mesh must be sized from
    /// the same numbers.
    fn block_half_extents(&self) -> Vector3<f32> {
        Vector3::new(
            self.half_extents.0,
            self.half_extents.1,
            self.half_extents.2,
        )
    }
}

impl Spawnable for IceBoxDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ice_material(
            ctx,
            seed_from_position(self.pos, 0),
            texture_spread(self.block_half_extents()),
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let centre = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let half_extents = self.block_half_extents();

        vec![IceBlock::new(
            centre,
            half_extents,
            materials[0],
            texture_spread(half_extents),
        )
        .rotated(Yaw::degrees(self.yaw).rotation())
        .spawn(world)]
    }
}
