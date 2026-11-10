//! Fish-walker — a fish on a man's bare legs.
//!
//! The amphibian. It walks the shore and walks on into the water after
//! whoever it is following, so a lake is no refuge from it. It gapes and
//! bites. `visual_bench bestiary` shows it.

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

/// Denser than water, so it keeps its feet on the bed of a lake.
const DENSITY: f32 = 1300.0;
/// Rad/s².
const TURN_AUTHORITY: f32 = 380.0;
/// Seconds of rearing back before it bites, and before it can again.
const BITE_WINDUP: f32 = 0.45;
const BITE_RECOVERY: f32 = 0.9;

const MOTTLE: Mottle = Mottle {
    frequency: 14.0,
    value: 0.2,
    warmth: 0.18,
    seed: 41,
};

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
            knee_radius: 0.055,
            haunch_height: 0.05,
            haunch_radius: 0.08,
            neck_length: 0.2,
            neck_radius: 0.05,
            eye_radius: 0.24,
            rest_pitch: 0.0,
            rear_pitch: -0.45,
            strike_pitch: 0.7,
            step_height: 0.16,
            knee_bend: 1.0,
            foot: FootShape {
                radius: 0.045,
                half_length: 0.12,
                ankle_forward_offset: 0.05,
            },
            hide_colour: Colour::new(0.82, 0.64, 0.55, 1.0),
            foot_colour: Colour::new(0.76, 0.56, 0.48, 1.0),
            ..StiltRigConfig::default()
        }
    }

    /// What it wears: the fish, level over its hips, mouth agape.
    pub fn costume() -> Costume {
        let scales = paint(0.24, 0.36, 0.36);
        let belly = paint(0.82, 0.6, 0.52);
        let fin = paint(0.7, 0.3, 0.22);
        let maw = paint(0.25, 0.04, 0.05);
        let white = paint(0.9, 0.88, 0.75);
        let bead = paint(0.03, 0.03, 0.03);

        let mut head = vec![
            // The body, narrowing to the tail, with a pale belly under it.
            SculptedPart::new(
                Ellipsoid::new((0.15, 0.21, 0.46), (0.0, 0.0, -0.06)),
                scales,
            )
            .warped(Warp::Taper {
                along: -Vector3::z(),
                tip: 0.25,
            }),
            SculptedPart::new(
                Ellipsoid::new((0.13, 0.12, 0.36), (0.0, -0.1, -0.02)),
                belly,
            )
            .warped(Warp::Taper {
                along: -Vector3::z(),
                tip: 0.3,
            }),
            // The mouth, open, and its lips.
            SculptedPart::new(
                Ellipsoid::new((0.085, 0.075, 0.03), (0.0, -0.02, 0.385)),
                maw,
            ),
            SculptedPart::new(Ellipsoid::new((0.1, 0.03, 0.04), (0.0, 0.055, 0.37)), belly),
            SculptedPart::new(
                Ellipsoid::new((0.095, 0.03, 0.04), (0.0, -0.1, 0.36)),
                belly,
            ),
            // Tail and dorsal fin: thin, upright, red.
            SculptedPart::new(Ellipsoid::new((0.012, 0.2, 0.13), (0.0, 0.0, -0.58)), fin).warped(
                Warp::Taper {
                    along: Vector3::z(),
                    tip: 0.25,
                },
            ),
            SculptedPart::new(Ellipsoid::new((0.01, 0.07, 0.2), (0.0, 0.2, -0.1)), fin),
        ];
        for side in [-1.0, 1.0] {
            // A round flat eye and a small fin each side.
            head.push(SculptedPart::new(
                Ellipsoid::new((0.02, 0.055, 0.055), (0.125 * side, 0.06, 0.2)),
                white,
            ));
            head.push(SculptedPart::new(
                Ellipsoid::new((0.02, 0.028, 0.028), (0.137 * side, 0.06, 0.205)),
                bead,
            ));
            head.push(SculptedPart::new(
                Ellipsoid::new((0.012, 0.06, 0.12), (0.15 * side, -0.06, 0.02))
                    .yawed(0.6 * side)
                    .pitched(0.4),
                fin,
            ));
        }
        Costume {
            head: garment(&head, &MOTTLE, &COSTUME_SHADE),
            ..Costume::default()
        }
    }

    fn walker(&self) -> StiltWalker {
        StiltWalker {
            at: self.pos,
            rig: Self::rig(),
            costume: Some(Arc::new(Self::costume())),
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
