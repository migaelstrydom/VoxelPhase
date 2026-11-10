//! Egg-walker — a cracked eggshell on two bare legs, with something looking
//! out of the crack.
//!
//! The swarm. One is no trouble: it is quick, but a single blow breaks it and
//! its butt only bruises. They come several at a time. `visual_bench
//! bestiary` shows it.

use std::f32::consts::{FRAC_PI_2, PI};
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

/// Light as a shell: a blast scatters a clutch of them.
const DENSITY: f32 = 220.0;
/// Rad/s². Nimble.
const TURN_AUTHORITY: f32 = 520.0;
/// Seconds of rocking back before it butts, and before it can again.
const BUTT_WINDUP: f32 = 0.3;
const BUTT_RECOVERY: f32 = 0.6;

const MOTTLE: Mottle = Mottle {
    frequency: 12.0,
    value: 0.1,
    warmth: 0.1,
    seed: 37,
};

/// An egg-walker.
#[derive(Deserialize)]
pub struct EggWalkerDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

impl EggWalkerDef {
    /// Its proportions: an egg the size of a barrel on short bare legs.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.27,
            lower_leg_length: 0.26,
            hip_width: 0.1,
            leg_radius: 0.03,
            knee_radius: 0.04,
            haunch_height: 0.03,
            haunch_radius: 0.05,
            neck_length: 0.27,
            neck_radius: 0.03,
            eye_radius: 0.27,
            rest_pitch: 0.06,
            rear_pitch: -0.35,
            strike_pitch: 0.6,
            step_height: 0.12,
            // Knees that bend the way a man's do.
            knee_bend: 1.0,
            foot: FootShape {
                radius: 0.04,
                half_length: 0.09,
                ankle_forward_offset: 0.04,
            },
            hide_colour: Colour::new(0.8, 0.62, 0.52, 1.0),
            foot_colour: Colour::new(0.74, 0.54, 0.46, 1.0),
            ..StiltRigConfig::default()
        }
    }

    /// What it wears: the shell, broken open at the front, and an eye in the
    /// dark inside it.
    pub fn costume() -> Costume {
        let shell = paint(0.9, 0.86, 0.74);
        let dark = paint(0.05, 0.03, 0.03);
        let white = paint(0.92, 0.9, 0.8);
        let iris = paint(0.5, 0.1, 0.08);

        let egg = SculptedPart::new(Ellipsoid::new((0.27, 0.35, 0.27), (0.0, 0.0, 0.0)), shell)
            .warped(Warp::Taper {
                along: Vector3::y(),
                tip: 0.72,
            });
        let mut head = vec![egg.clone()];
        // The break: a dark hole high on the front, its edge a ring of
        // jagged teeth of shell.
        let (hole, facing) = egg.surface_at(0.0, 0.45);
        head.push(SculptedPart::new(
            Ellipsoid {
                radii: Vector3::new(0.11, 0.09, 0.02),
                centre: hole + facing * 0.004,
                turn: nalgebra::UnitQuaternion::rotation_between(&Vector3::z(), &facing)
                    .unwrap_or_else(nalgebra::UnitQuaternion::identity),
            },
            dark,
        ));
        for tooth in 0..9 {
            let angle = tooth as f32 / 9.0 * 2.0 * PI;
            let (at, out) = egg.surface_at(0.44 * angle.cos(), 0.45 + 0.36 * angle.sin());
            head.push(
                SculptedPart::new(
                    Ellipsoid {
                        radii: Vector3::new(0.035, 0.05, 0.012),
                        centre: at + out * 0.006,
                        turn: nalgebra::UnitQuaternion::rotation_between(&Vector3::z(), &out)
                            .unwrap_or_else(nalgebra::UnitQuaternion::identity)
                            * nalgebra::UnitQuaternion::from_axis_angle(
                                &Vector3::z_axis(),
                                angle + FRAC_PI_2,
                            ),
                    },
                    shell,
                )
                .warped(Warp::Taper {
                    along: Vector3::y(),
                    tip: 0.1,
                }),
            );
        }
        // The eye, looking out.
        head.push(SculptedPart::new(
            Ellipsoid::ball(0.06, hole + facing * 0.0),
            white,
        ));
        head.push(SculptedPart::new(
            Ellipsoid::ball(0.028, hole + facing * 0.03),
            iris,
        ));
        head.push(SculptedPart::new(
            Ellipsoid::ball(0.015, hole + facing * 0.048),
            dark,
        ));
        // A hairline crack running down from the break.
        for step in 0..5 {
            let (at, out) =
                egg.surface_at(0.07 * (step as f32 * 2.1).sin(), 0.08 - 0.13 * step as f32);
            head.push(SculptedPart::new(
                Ellipsoid::new((0.005, 0.045, 0.005), (0.0, 0.0, 0.0)),
                dark,
            ));
            let crack = head.last_mut().expect("just pushed");
            crack.shape.centre = at + out * 0.002;
            crack.shape = crack.shape.rolled(0.7 * (step as f32 * 2.1).cos());
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
            speed: 4.4,
            sight_range: 18.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 6.0,
                windup: BUTT_WINDUP,
                recovery: BUTT_RECOVERY,
            }),
            health: Some(12.0),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for EggWalkerDef {
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
