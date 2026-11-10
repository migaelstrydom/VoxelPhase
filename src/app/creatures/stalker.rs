//! Stalker — a tall bird-headed figure in a funnel hat, on stilts.
//!
//! The ground chaser. It is slow and it looms: it sees over what a roller
//! cannot, comes on at a walk, and its long beak reaches well past its body.
//! It can be killed, but it takes a good deal of killing; the quicker way is
//! to take the ground out from under its legs. `visual_bench bestiary` shows
//! it.

use std::sync::Arc;

use nalgebra::Vector3;
use serde::Deserialize;
use specs::{Entity, World};

use super::stilt_walker::{garment, paint, Peck, StiltWalker, COSTUME_SHADE};
use crate::animation::rig::FootShape;
use crate::animation::stilts::{Costume, StiltRigConfig};
use crate::app::sculpt::{Ellipsoid, Mottle, SculptedPart, Warp};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

/// Heavier than a peeper: a blast staggers it without throwing it.
const DENSITY: f32 = 600.0;
/// Rad/s². It turns slowly, so it can be circled.
const TURN_AUTHORITY: f32 = 300.0;
/// Seconds of rearing back before the beak comes down, and before it can
/// again.
const PECK_WINDUP: f32 = 0.7;
const PECK_RECOVERY: f32 = 1.1;

const MOTTLE: Mottle = Mottle {
    frequency: 9.0,
    value: 0.22,
    warmth: 0.12,
    seed: 31,
};

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
            knee_radius: 0.05,
            haunch_height: 0.42,
            haunch_radius: 0.06,
            neck_length: 0.5,
            neck_radius: 0.04,
            eye_radius: 0.24,
            rest_pitch: 0.3,
            rear_pitch: -0.5,
            strike_pitch: 1.5,
            step_height: 0.3,
            foot: FootShape {
                radius: 0.04,
                half_length: 0.15,
                ankle_forward_offset: 0.05,
            },
            hide_colour: Colour::new(0.62, 0.44, 0.2, 1.0),
            foot_colour: Colour::new(0.5, 0.32, 0.12, 1.0),
            ..StiltRigConfig::default()
        }
    }

    /// What it wears: a heron's head under a tin funnel, and a dark robe
    /// hunched over its hips.
    pub fn costume() -> Costume {
        let skull = paint(0.74, 0.7, 0.6);
        let beak = paint(0.7, 0.5, 0.16);
        let tin = paint(0.42, 0.44, 0.46);
        let robe = paint(0.12, 0.2, 0.26);
        let lining = paint(0.5, 0.12, 0.12);
        let bead = paint(0.03, 0.03, 0.03);
        let ring = paint(0.85, 0.7, 0.2);

        let mut head = vec![
            SculptedPart::new(Ellipsoid::new((0.15, 0.155, 0.2), (0.0, 0.0, 0.0)), skull),
            // The beak: long, straight, and hooked a little at its tip.
            SculptedPart::new(
                Ellipsoid::new((0.045, 0.05, 0.38), (0.0, -0.04, 0.48)).pitched(0.12),
                beak,
            )
            .warped(Warp::Taper {
                along: Vector3::z(),
                tip: 0.12,
            }),
            // The funnel, mouth down over the crown, and its spout.
            SculptedPart::new(Ellipsoid::new((0.22, 0.2, 0.22), (0.0, 0.22, -0.02)), tin).warped(
                Warp::Taper {
                    along: Vector3::y(),
                    tip: 0.16,
                },
            ),
            SculptedPart::new(
                Ellipsoid::new((0.035, 0.15, 0.035), (0.0, 0.46, -0.02)),
                tin,
            ),
            SculptedPart::new(Ellipsoid::new((0.25, 0.022, 0.25), (0.0, 0.08, -0.02)), tin),
        ];
        for side in [-1.0, 1.0] {
            // A round staring eye each side, ringed in yellow.
            head.push(SculptedPart::new(
                Ellipsoid::new((0.014, 0.05, 0.05), (0.138 * side, 0.01, 0.07)),
                ring,
            ));
            head.push(SculptedPart::new(
                Ellipsoid::new((0.014, 0.026, 0.026), (0.148 * side, 0.01, 0.07)),
                bead,
            ));
        }

        let body = vec![
            // The robe, hunched forward over the hips, open at the hem.
            SculptedPart::new(Ellipsoid::new((0.24, 0.36, 0.3), (0.0, 0.2, 0.03)), robe).warped(
                Warp::Taper {
                    along: Vector3::y(),
                    tip: 0.55,
                },
            ),
            SculptedPart::new(
                Ellipsoid::new((0.2, 0.05, 0.25), (0.0, -0.12, 0.03)),
                lining,
            ),
            // A hump behind the shoulders, and the tail of the robe.
            SculptedPart::new(Ellipsoid::new((0.19, 0.18, 0.2), (0.0, 0.38, -0.14)), robe),
            SculptedPart::new(
                Ellipsoid::new((0.1, 0.04, 0.26), (0.0, 0.0, -0.28)).pitched(-0.5),
                robe,
            )
            .warped(Warp::Taper {
                along: -Vector3::z(),
                tip: 0.2,
            }),
        ];
        Costume {
            head: garment(&head, &MOTTLE, &COSTUME_SHADE),
            body: garment(&body, &MOTTLE, &COSTUME_SHADE),
            shows_trunk: true,
        }
    }

    fn walker(&self) -> StiltWalker {
        StiltWalker {
            at: self.pos,
            rig: Self::rig(),
            costume: Some(Arc::new(Self::costume())),
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
        0
    }

    fn create_materials(&self, _ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // Drawn in its vertices' colours, like every rig.
        Ok(Vec::new())
    }

    fn spawn(&self, world: &mut World, _materials: &[MaterialId]) -> Vec<Entity> {
        self.walker().spawn(world)
    }
}
