//! Fish-walker — a fish on a man's bare legs.
//!
//! The amphibian. It walks the shore and walks on into the water after
//! whoever it is following, so a lake is no refuge from it. It gapes and
//! bites. `visual_bench fish_walker` shows it.

use std::sync::Arc;

use serde::Deserialize;
use specs::{Entity, World};

use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{blockout, Peck, StiltWalker};
use crate::animation::stilts::{Dress, StiltRigConfig};
use crate::animation::FootPlacerConfig;
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::rendering::material::MaterialId;

/// The name its look is kept under.
const KIND: &str = "fish_walker";

/// Denser than water, so it keeps its feet on the bed of a lake.
const DENSITY: f32 = 1300.0;
/// Rad/s².
const TURN_AUTHORITY: f32 = 380.0;
/// Seconds of rearing back before it bites, and before it can again.
const BITE_WINDUP: f32 = 0.45;
const BITE_RECOVERY: f32 = 0.9;

/// A fish-walker.
#[derive(Deserialize)]
pub struct FishWalkerDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

impl FishWalkerDef {
    /// Its proportions: a man's legs to the hip, and a fish from there up.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.42,
            lower_leg_length: 0.4,
            hip_width: 0.1,
            leg_radius: 0.045,
            haunch_height: 0.05,
            haunch_radius: 0.08,
            neck_length: 0.2,
            neck_radius: 0.05,
            head_radius: 0.24,
            rest_pitch: 0.0,
            rear_pitch: -0.45,
            strike_pitch: 0.7,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.16,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: 1.0,
            foot_placer: FootPlacerConfig::default(),
        }
    }

    /// Its dress, against the palette's slots: a blockout, until it is
    /// sculpted.
    pub fn sculpt() -> Dress {
        blockout(&Self::rig(), (0.3, 0.4, 0.4))
    }

    /// Its dress against the palette's slots, sculpted once: for tools
    /// that draw it in the palette's own materials.
    pub fn dress() -> Arc<Dress> {
        likeness::dress_sculpted(KIND, Self::sculpt)
    }

    fn walker(&self, materials: &[MaterialId]) -> StiltWalker {
        StiltWalker {
            at: self.pos,
            rig: Self::rig(),
            dress: likeness::dress(KIND, materials, Self::sculpt),
            speed: 3.2,
            sight_range: 22.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 10.0,
                windup: BITE_WINDUP,
                recovery: BITE_RECOVERY,
            }),
            health: Some(40.0),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for FishWalkerDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        self.walker(materials).spawn(world)
    }
}
