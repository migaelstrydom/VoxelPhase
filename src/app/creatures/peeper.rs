//! Peeper — a one-eyed stalker that walks on stilts.
//!
//! The counterweight to the roller. A roller is heavy, blind-ish, and
//! unkillable; you deal with one by using the terrain. A peeper is the
//! opposite on every axis: it is fragile, it sees a long way, and it can
//! be killed outright — but it is *tall*, which means it spots you across
//! ground a roller would never see you over, and its neck reaches further
//! than its body suggests.
//!
//! Its one trick is the eye. A peeper with nothing to look at stands
//! dozing with its lid shut, which makes it a landmark rather than a
//! threat: the player can creep past one, and the snap of the eye opening
//! is the whole alert beat. That is why it carries no
//! [`AlertTelegraph`](crate::creature::AlertTelegraph) — the hop-and-
//! shiver exists for creatures with no face to pull, and this one is
//! mostly face.
//!
//! Everything below the eye is shared: it walks on the same
//! [`LeggedLocomotion`](crate::animation::LeggedLocomotion) as the player
//! and the heart critter, through the same character control chain, driven
//! by the same [`Brain`] as the roller.

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
const KIND: &str = "peeper";

/// Spindly. A peeper is legs and air, and a blast should send it
/// cartwheeling — being thrown about is most of what makes it fun to
/// fight.
const DENSITY: f32 = 300.0;

/// Yaw authority, in rad/s². Below the heart critter's: a peeper pivots
/// deliberately, so circling it is a real option.
const TURN_AUTHORITY: f32 = 420.0;

/// Seconds of wind-up before a peck lands, and of recovery after. The
/// wind-up is long on purpose: it is the warning, and the head rearing
/// back over the haunch is unmistakable.
const PECK_WINDUP: f32 = 0.5;
const PECK_RECOVERY: f32 = 0.85;

/// A stalking creature that pecks.
#[derive(Deserialize)]
pub struct PeeperDef {
    /// Position (x, z). Y comes from the terrain surface, since a creature
    /// authored at a hand-picked height would either hang in the air or
    /// start embedded once the terrain around it is reshaped.
    pub pos: (f32, f32),

    /// Flat-ground top speed in m/s. The player walks at 5.0 and sprints
    /// at 8.0, so a peeper is outrunnable — it is a thing you leave
    /// behind, not a thing you escape.
    #[serde(default = "PeeperDef::default_speed")]
    pub speed: f32,
    /// How far it can see. Long: being tall is the creature's advantage.
    #[serde(default = "PeeperDef::default_sight_range")]
    pub sight_range: f32,
    /// Hit points a peck takes off.
    #[serde(default = "PeeperDef::default_peck_damage")]
    pub peck_damage: f32,
    /// Hit points it can take before it dies.
    #[serde(default = "PeeperDef::default_health")]
    pub health: f32,
}

impl PeeperDef {
    pub fn default_speed() -> f32 {
        3.4
    }
    pub fn default_sight_range() -> f32 {
        28.0
    }
    pub fn default_peck_damage() -> f32 {
        14.0
    }
    pub fn default_health() -> f32 {
        55.0
    }

    /// Its proportions: almost all leg, with an eye the size of a melon.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.52,
            lower_leg_length: 0.5,
            hip_width: 0.07,
            leg_radius: 0.026,
            haunch_height: 0.16,
            haunch_radius: 0.085,
            neck_length: 0.42,
            neck_radius: 0.035,
            head_radius: 0.22,
            rest_pitch: 0.2,
            rear_pitch: -0.55,
            strike_pitch: 1.35,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.22,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: -1.0,
            foot_placer: FootPlacerConfig::default(),
        }
    }

    /// Its dress, against the palette's slots: a blockout, until it is
    /// sculpted.
    pub fn sculpt() -> Dress {
        blockout(&Self::rig(), (0.8, 0.76, 0.66))
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
            speed: self.speed,
            sight_range: self.sight_range,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: self.peck_damage,
                windup: PECK_WINDUP,
                recovery: PECK_RECOVERY,
            }),
            health: Some(self.health),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for PeeperDef {
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
