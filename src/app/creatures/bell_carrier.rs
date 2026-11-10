//! Bell-carrier — a small thing under a great bronze bell, with only its
//! shins and its eyes showing.
//!
//! The armoured one. Nothing hurts it: a blast rings the bell and shoves it,
//! and it comes on. It is heavy and slow, so it is dealt with by where it is
//! made to walk: off a ledge, into a pit, into deep water. `visual_bench
//! bell_carrier` shows it.

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
const KIND: &str = "bell_carrier";

/// Bronze: it sinks like one, and a blast barely moves it.
const DENSITY: f32 = 2600.0;
/// Rad/s².
const TURN_AUTHORITY: f32 = 260.0;
/// Seconds of rocking back before it rams, and before it can again.
const RAM_WINDUP: f32 = 0.8;
const RAM_RECOVERY: f32 = 1.2;

/// The bell's half-width at its mouth, m.
const BELL_RADIUS: f32 = 0.42;

/// A bell-carrier.
#[derive(Deserialize)]
pub struct BellCarrierDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

impl BellCarrierDef {
    /// Its proportions: short legs, and the bell over everything above them.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.2,
            lower_leg_length: 0.2,
            hip_width: 0.12,
            leg_radius: 0.04,
            haunch_height: 0.05,
            haunch_radius: 0.08,
            neck_length: 0.4,
            neck_radius: 0.05,
            head_radius: BELL_RADIUS,
            rest_pitch: 0.0,
            rear_pitch: -0.25,
            strike_pitch: 0.35,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.08,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: 1.0,
            foot_placer: FootPlacerConfig::default(),
        }
    }

    /// Its dress, against the palette's slots: a blockout, until it is
    /// sculpted.
    pub fn sculpt() -> Dress {
        blockout(&Self::rig(), (0.4, 0.27, 0.1))
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
            speed: 2.0,
            sight_range: 16.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 18.0,
                windup: RAM_WINDUP,
                recovery: RAM_RECOVERY,
            }),
            // Nothing breaks the bell.
            health: None,
            brain: |standoff| Brain::hunter(standoff).relentless(),
        }
    }
}

impl Spawnable for BellCarrierDef {
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
