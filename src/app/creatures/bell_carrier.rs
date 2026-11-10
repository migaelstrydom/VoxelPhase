//! Bell-carrier — a small thing under a great bronze bell, with only its
//! shins and its eyes showing.
//!
//! The armoured one. Nothing hurts it: a blast rings the bell and shoves it,
//! and it comes on. It is heavy and slow, so it is dealt with by where it is
//! made to walk: off a ledge, into a pit, into deep water. `visual_bench
//! bestiary` shows it.

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

/// Bronze: it sinks like one, and a blast barely moves it.
const DENSITY: f32 = 2600.0;
/// Rad/s².
const TURN_AUTHORITY: f32 = 260.0;
/// Seconds of rocking back before it rams, and before it can again.
const RAM_WINDUP: f32 = 0.8;
const RAM_RECOVERY: f32 = 1.2;

/// The bell's half-width at its mouth and its height, m.
const BELL_RADIUS: f32 = 0.42;
const BELL_HEIGHT: f32 = 0.95;

const MOTTLE: Mottle = Mottle {
    frequency: 6.0,
    value: 0.35,
    warmth: -0.45,
    seed: 43,
};

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
            knee_radius: 0.05,
            haunch_height: 0.05,
            haunch_radius: 0.08,
            neck_length: 0.4,
            neck_radius: 0.05,
            eye_radius: BELL_RADIUS,
            rest_pitch: 0.0,
            rear_pitch: -0.25,
            strike_pitch: 0.35,
            step_height: 0.08,
            knee_bend: 1.0,
            foot: FootShape {
                radius: 0.05,
                half_length: 0.11,
                ankle_forward_offset: 0.04,
            },
            hide_colour: Colour::new(0.3, 0.24, 0.2, 1.0),
            foot_colour: Colour::new(0.22, 0.17, 0.14, 1.0),
            ..StiltRigConfig::default()
        }
    }

    /// What it wears: the bell, its lip a hand above the ground, a slot cut
    /// in its front and two eyes in the dark behind the slot.
    pub fn costume() -> Costume {
        let bronze = paint(0.4, 0.27, 0.1);
        let dark = paint(0.03, 0.03, 0.03);
        let glow = paint(0.95, 0.8, 0.3);

        // Sculpted about the head: the bell hangs from its crown down past
        // the hips, and rocks with the neck.
        let crown = 0.3;
        let bell = SculptedPart::new(
            Ellipsoid::new(
                (BELL_RADIUS, 0.5 * BELL_HEIGHT, BELL_RADIUS),
                (0.0, crown - 0.5 * BELL_HEIGHT, 0.0),
            ),
            bronze,
        )
        .warped(Warp::Taper {
            along: Vector3::y(),
            tip: 0.3,
        });
        let mut head = vec![
            bell.clone(),
            // The flared lip, and the loop it was hung by.
            SculptedPart::new(
                Ellipsoid::new(
                    (BELL_RADIUS * 0.97, 0.07, BELL_RADIUS * 0.97),
                    (0.0, crown - 0.9 * BELL_HEIGHT, 0.0),
                ),
                bronze,
            ),
            SculptedPart::new(
                Ellipsoid::new((0.03, 0.09, 0.09), (0.0, crown + 0.06, 0.0)),
                bronze,
            ),
        ];
        // The slot and the eyes behind it.
        let (slot, out) = bell.surface_at(0.0, 0.1);
        head.push(SculptedPart::new(
            Ellipsoid::new((0.2, 0.06, 0.03), (slot.x, slot.y, slot.z + 0.004)),
            dark,
        ));
        for side in [-1.0, 1.0] {
            head.push(SculptedPart::new(
                Ellipsoid::ball(0.03, slot + out * 0.012 + Vector3::x() * 0.08 * side),
                glow,
            ));
            head.push(SculptedPart::new(
                Ellipsoid::ball(0.014, slot + out * 0.036 + Vector3::x() * 0.08 * side),
                dark,
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
