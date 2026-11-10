//! Stalker — a tall bird-headed figure in a funnel hat, on stilts.
//!
//! The ground chaser. It is slow and it looms: it sees over what a roller
//! cannot, comes on at a walk, and its long beak reaches well past its body.
//! It can be killed, but it takes a good deal of killing; the quicker way is
//! to take the ground out from under its legs. `visual_bench stalker` shows
//! it.

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
const KIND: &str = "stalker";

/// Heavier than a peeper: a blast staggers it without throwing it.
const DENSITY: f32 = 600.0;
/// Rad/s². It turns slowly, so it can be circled.
const TURN_AUTHORITY: f32 = 300.0;
/// Seconds of rearing back before the beak comes down, and before it can
/// again.
const PECK_WINDUP: f32 = 0.7;
const PECK_RECOVERY: f32 = 1.1;

/// A stalker.
#[derive(Deserialize)]
pub struct StalkerDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

impl StalkerDef {
    /// Its proportions: nearly three metres of leg, robe and neck.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.78,
            lower_leg_length: 0.74,
            hip_width: 0.1,
            leg_radius: 0.03,
            haunch_height: 0.42,
            haunch_radius: 0.06,
            neck_length: 0.5,
            neck_radius: 0.04,
            head_radius: 0.24,
            rest_pitch: 0.3,
            rear_pitch: -0.5,
            strike_pitch: 1.5,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.3,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: -1.0,
            foot_placer: FootPlacerConfig::default(),
        }
    }

    /// Its dress, against the palette's slots: a blockout, until it is
    /// sculpted.
    pub fn sculpt() -> Dress {
        blockout(&Self::rig(), (0.5, 0.46, 0.38))
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
            speed: 2.6,
            sight_range: 32.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 22.0,
                windup: PECK_WINDUP,
                recovery: PECK_RECOVERY,
            }),
            health: Some(110.0),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for StalkerDef {
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
