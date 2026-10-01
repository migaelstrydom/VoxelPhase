//! Stack spawnable — vertical stack of mixed items, auto-computing Y positions.

use serde::Deserialize;
use specs::Entity;

use super::beach_ball::BeachBallDef;
use super::box_object::{
    create_box_material_for_style, CrateDef, HeavyCrateDef, PlankDef, CRATE_SURFACE,
    HEAVY_CRATE_SURFACE, PLANK_HALF_THICKNESS, PLANK_SURFACE,
};
use super::capsule::CapsuleDef;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::level::BoxStyle;
use crate::rendering::material::MaterialId;

#[derive(Deserialize)]
pub struct StackDef {
    pub base: (f32, f32, f32),
    pub items: Vec<StackItemDef>,
    /// Rotation about `+Y`, in degrees. The stack is vertical, so this reaches
    /// only the items that have an axis of their own — the planks.
    #[serde(default)]
    pub yaw: f32,
}

/// Items that can appear inside a Stack.
#[derive(Deserialize)]
pub enum StackItemDef {
    Crate { size: f32 },
    HeavyCrate { size: f32 },
    Plank { length: f32, width: f32 },
    BeachBall,
    Capsule { half_height: f32, radius: f32 },
}

impl StackItemDef {
    /// Half-height below the center point for placement.
    fn half_height_below(&self) -> f32 {
        match self {
            StackItemDef::Crate { size } => *size,
            StackItemDef::HeavyCrate { size } => *size,
            StackItemDef::Plank { .. } => PLANK_HALF_THICKNESS,
            StackItemDef::BeachBall => 0.5,
            StackItemDef::Capsule { half_height, .. } => *half_height,
        }
    }

    /// Half-height above the center point for placement.
    fn half_height_above(&self) -> f32 {
        self.half_height_below()
    }

    fn material_count(&self) -> usize {
        match self {
            StackItemDef::Crate { .. }
            | StackItemDef::HeavyCrate { .. }
            | StackItemDef::Plank { .. }
            | StackItemDef::Capsule { .. } => 1,
            StackItemDef::BeachBall => 1,
        }
    }
}

impl Spawnable for StackDef {
    fn material_count(&self) -> usize {
        self.items.iter().map(|i| i.material_count()).sum()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let mut mats = Vec::new();
        for (index, item) in self.items.iter().enumerate() {
            let seed = seed_from_position(self.base, index as u32);
            match item {
                StackItemDef::Crate { .. } => {
                    mats.push(create_box_material_for_style(
                        BoxStyle::WoodenCrate,
                        CRATE_SURFACE,
                        seed,
                        ctx.textures,
                        ctx.materials,
                    )?);
                }
                StackItemDef::HeavyCrate { .. } => {
                    mats.push(create_box_material_for_style(
                        BoxStyle::Metal,
                        HEAVY_CRATE_SURFACE,
                        seed,
                        ctx.textures,
                        ctx.materials,
                    )?);
                }
                StackItemDef::Plank { .. } => {
                    mats.push(create_box_material_for_style(
                        BoxStyle::WoodenCrate,
                        PLANK_SURFACE,
                        seed,
                        ctx.textures,
                        ctx.materials,
                    )?);
                }
                StackItemDef::BeachBall => {
                    let bb = BeachBallDef {
                        pos: (0.0, 0.0, 0.0),
                    };
                    mats.extend(bb.create_materials(ctx)?);
                }
                StackItemDef::Capsule { .. } => {
                    let cap = CapsuleDef {
                        pos: (0.0, 0.0, 0.0),
                        half_height: 0.5,
                        radius: 0.25,
                        density: 50.0,
                        restitution: 0.2,
                        friction: 0.6,
                    };
                    mats.extend(cap.create_materials(ctx)?);
                }
            }
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut specs::World, materials: &[MaterialId]) -> Vec<Entity> {
        let mut entities = Vec::new();
        let mut y = self.base.1;
        let mut mat_offset = 0;

        for item in &self.items {
            let count = item.material_count();
            let slice = &materials[mat_offset..mat_offset + count];
            y += item.half_height_below();

            let pos = (self.base.0, y, self.base.2);

            let new_entities = match item {
                StackItemDef::Crate { size } => {
                    let def = CrateDef { pos, size: *size };
                    def.spawn(world, slice)
                }
                StackItemDef::HeavyCrate { size } => {
                    let def = HeavyCrateDef { pos, size: *size };
                    def.spawn(world, slice)
                }
                StackItemDef::Plank { length, width } => {
                    let def = PlankDef {
                        pos,
                        length: *length,
                        width: *width,
                        yaw: self.yaw,
                    };
                    def.spawn(world, slice)
                }
                StackItemDef::BeachBall => {
                    let def = BeachBallDef { pos };
                    def.spawn(world, slice)
                }
                StackItemDef::Capsule {
                    half_height,
                    radius,
                } => {
                    let def = CapsuleDef {
                        pos,
                        half_height: *half_height,
                        radius: *radius,
                        density: 50.0,
                        restitution: 0.2,
                        friction: 0.6,
                    };
                    def.spawn(world, slice)
                }
            };

            entities.extend(new_entities);
            y += item.half_height_above();
            mat_offset += count;
        }

        entities
    }
}
