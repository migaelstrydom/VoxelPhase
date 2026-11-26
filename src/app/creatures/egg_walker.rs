//! Egg-walker — a broken eggshell on a man's bare legs. Something inside
//! holds the break open with its fingers and looks out.
//!
//! The swarm. One is no trouble: it is quick, but a single blow breaks it
//! and its butt only bruises. They come several at a time. `visual_bench
//! egg_walker` shows it.
//!
//! ```text
//!   shell    a hollow egg, broken open at the front and cracked from the
//!            break; two holes in its underside for the legs      (Head)
//!   within   wet dark flesh, an eye that follows you and its lids,
//!            eight fingers hooked over the broken edge           (Head, Eye, Hinge)
//!   legs     bare thighs, shins and feet                         (the bones)
//! ```

use std::f32::consts::FRAC_PI_2;

use nalgebra::{UnitQuaternion, Vector3};
use std::sync::Arc;

use serde::Deserialize;
use specs::{Entity, World};

use super::anatomy::{self, EyeLook, DIGIT_GRAIN};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{bare_legs, Peck, StiltWalker};
use crate::animation::stilts::{Carriage, Dress, Drive, Joint, StiltRigConfig, Worn};
use crate::animation::{FootPlacerConfig, FootSide};
use crate::app::sculpt::{Clay, Grime, Lump, Mottle, Relief, Spots, VertexPaint};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::harm::HarmConfig;
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;

/// The name its look is kept under.
const KIND: &str = "egg_walker";

/// Light as a shell: a blast scatters a clutch of them.
const DENSITY: f32 = 220.0;
/// Rad/s². Nimble.
const TURN_AUTHORITY: f32 = 520.0;
/// Seconds of rocking back before it butts, and before it can again.
const BUTT_WINDUP: f32 = 0.3;
const BUTT_RECOVERY: f32 = 0.6;
/// The impulse its butt drives into what it strikes, N·s: about 5.5 m/s on
/// the player, which only just knocks them down.
const BUTT_BLOW: f32 = 310.0;

/// The shell (linear), and how thick it is, m.
const SHELL: (f32, f32, f32) = (0.8, 0.74, 0.6);
const SHELL_THICKNESS: f32 = 0.016;
/// Side of the grid the shell is skinned from, m.
const SHELL_GRAIN: f32 = 0.009;
/// Where the break is on the shell: how far up from its equator, radians.
const BREAK_ELEVATION: f32 = 0.28;
/// Radius of the eye that looks out of it, m, and how far behind the
/// shell's wall its middle sits.
const EYE_RADIUS: f32 = 0.046;
const EYE_SET_BACK: f32 = 0.085;
/// A hand of it, wrist to fingertip, m, and how far each joint of its
/// fingers is curled to hook over the shell's edge, radians.
const HAND: f32 = 0.22;
const GRIP: f32 = 1.4;
/// How far a gripping hand leans out from the wall toward its fingers,
/// radians.
const LEAN: f32 = 0.5;
/// What is inside (linear).
const INNARDS: (f32, f32, f32) = (0.13, 0.03, 0.03);
/// The lids of its eye: skin that has not seen light (linear).
const LID: (f32, f32, f32) = (0.3, 0.17, 0.15);
/// What runs from the break (linear).
const ICHOR: (f32, f32, f32) = (0.2, 0.035, 0.02);

/// An egg-walker.
#[derive(Deserialize)]
pub struct EggWalkerDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

/// A mesh turned by `turn` and then moved by `shift`.
fn placed(
    mut mesh: MeshPrimitive,
    turn: UnitQuaternion<f32>,
    shift: Vector3<f32>,
) -> MeshPrimitive {
    for vertex in &mut mesh.vertices {
        vertex.pos = turn * vertex.pos + shift;
        vertex.normal = turn * vertex.normal;
    }
    mesh
}

impl EggWalkerDef {
    /// Its proportions: an egg the size of a barrel on a man's legs, cut
    /// off short.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.3,
            lower_leg_length: 0.3,
            ankle_height: anatomy::ANKLE_RISE,
            hip_width: 0.085,
            leg_radius: 0.05,
            haunch_height: 0.02,
            haunch_radius: 0.08,
            neck_length: 0.3,
            neck_radius: 0.1,
            head_radius: 0.27,
            rest_pitch: 0.05,
            rear_pitch: -0.3,
            strike_pitch: 0.55,
            lid_rate: 14.0,
            standing_height_ratio: 0.9,
            step_height: 0.12,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            // Knees that bend the way a man's do.
            knee_bend: 1.0,
            foot_placer: FootPlacerConfig::default(),
            carriage: Carriage::default(),
        }
    }

    /// The egg before it broke: solid, about the head's middle.
    fn egg() -> Clay {
        Clay::new(0.2)
            .add(Lump::mass((0.27, 0.3, 0.27), (0.0, -0.05, 0.0)), SHELL)
            .add(Lump::mass((0.2, 0.24, 0.2), (0.0, 0.13, 0.0)), SHELL)
    }

    /// The point on the whole egg's surface `azimuth` radians round from
    /// straight ahead and `elevation` radians up, and the way out there.
    fn on_shell(azimuth: f32, elevation: f32) -> (Vector3<f32>, Vector3<f32>) {
        let out = Vector3::new(
            elevation.cos() * azimuth.sin(),
            elevation.sin(),
            elevation.cos() * azimuth.cos(),
        );
        let egg = Self::egg();
        // Walk out from the middle to where the egg ends.
        let (mut inside, mut outside) = (0.0, 0.6);
        for _ in 0..24 {
            let middle = 0.5 * (inside + outside);
            if egg.distance(out * middle) < 0.0 {
                inside = middle;
            } else {
                outside = middle;
            }
        }
        (out * inside, out)
    }

    /// The shell's form: hollow, broken open, cracked, and holed for the
    /// legs.
    fn shell_form(rig: &StiltRigConfig) -> Clay {
        let at = |azimuth: f32, elevation: f32| {
            let (point, _) = Self::on_shell(azimuth, elevation);
            (point.x, point.y, point.z)
        };
        let mut shell = Self::egg().hollow(SHELL_THICKNESS);
        // The break: bites out of the wall that overlap, so the edge
        // between two is a tooth of shell.
        for (azimuth, up, bite) in [
            (0.0, 0.0, 0.105),
            (0.34, 0.2, 0.07),
            (-0.36, -0.08, 0.062),
            (-0.2, 0.3, 0.055),
            (0.24, -0.2, 0.05),
            (0.02, 0.4, 0.04),
        ] {
            shell = shell.cut(Lump::ball(bite, at(azimuth, BREAK_ELEVATION + up)), 0.003);
        }
        // Cracks running from the break: each a chain of slits through the
        // wall, narrowing as it goes.
        let cracks: [&[(f32, f32)]; 4] = [
            &[(0.3, -0.02), (0.42, -0.3), (0.3, -0.52), (0.44, -0.8)],
            &[(-0.46, 0.2), (-0.8, 0.12), (-1.0, 0.3), (-1.3, 0.22)],
            &[(0.5, 0.42), (0.86, 0.62), (0.8, 0.9)],
            &[(-0.12, -0.14), (-0.2, -0.46), (-0.08, -0.7)],
        ];
        for crack in cracks {
            for (step, pair) in crack.windows(2).enumerate() {
                let wide = 0.0075 - 0.0015 * step as f32;
                shell = shell.cut(
                    Lump::limb(
                        at(pair[0].0, pair[0].1 + BREAK_ELEVATION * 0.3),
                        at(pair[1].0, pair[1].1 + BREAK_ELEVATION * 0.3),
                        (wide, 0.8 * wide),
                    ),
                    0.0,
                );
            }
        }
        // Where the legs come through.
        let hips = -(rig.neck_length + rig.haunch_height) - 0.02;
        for side in [-1.0, 1.0] {
            shell = shell.cut(Lump::ball(0.088, (side * rig.hip_width, hips, 0.0)), 0.012);
        }
        shell
    }

    /// How far up from the equator the break's lower edge is, radians, at
    /// its highest within `reach` radians either side of `azimuth`: the
    /// edge a hand that wide can hook over without a finger through the
    /// wall.
    fn rim(shell: &Clay, azimuth: f32, reach: f32) -> f32 {
        let solid = |azimuth: f32, elevation: f32| {
            let (surface, out) = Self::on_shell(azimuth, elevation);
            shell.distance(surface - out * 0.5 * SHELL_THICKNESS) < 0.0
        };
        // Down from the break's middle to the first of the wall, wherever
        // the break is open at that height.
        let lower_edge = |azimuth: f32| {
            (!solid(azimuth, BREAK_ELEVATION)).then(|| {
                (1..80)
                    .map(|step| BREAK_ELEVATION - 0.01 * step as f32)
                    .find(|&elevation| solid(azimuth, elevation))
            })?
        };
        (-2..=2)
            .filter_map(|across| lower_edge(azimuth + 0.5 * reach * across as f32))
            .fold(f32::MIN, f32::max)
    }

    /// The shell, skinned and stained.
    fn shell(form: &Clay) -> MeshPrimitive {
        let mut mesh = form.skin(SHELL_GRAIN, Paint::Shell.id());
        Mottle {
            frequency: 4.0,
            value: 0.12,
            warmth: 0.1,
            seed: 37,
        }
        .paint(&mut mesh);
        Spots {
            frequency: 34.0,
            presence: 0.3,
            radius: (0.12, 0.34),
            tint: Vector3::new(0.62, 0.48, 0.34),
            salt: 38,
        }
        .paint(&mut mesh);
        // It has been dragged about: stained toward its underside.
        Grime {
            below: -0.24,
            fade: 0.34,
            tint: Vector3::new(0.5, 0.42, 0.32),
        }
        .paint(&mut mesh);
        // Inside it is dark, and not with shadow.
        for vertex in &mut mesh.vertices {
            let inward = -vertex.normal.dot(&vertex.pos.normalize());
            if inward > 0.2 {
                vertex.color.x *= 0.2;
                vertex.color.y *= 0.09;
                vertex.color.z *= 0.07;
            }
        }
        mesh
    }

    /// Its dress, against the palette's slots.
    pub fn sculpt() -> Dress {
        let rig = Self::rig();
        let shell = Self::shell_form(&rig);
        let (hole, out) = Self::on_shell(0.0, BREAK_ELEVATION);
        let socket = hole - out * EYE_SET_BACK;

        // What lives in the shell: a wet mass in the bottom of it, and the
        // eye held up on a stalk in the dark.
        let innards = Clay::new(0.1)
            .add(Lump::mass((0.21, 0.15, 0.21), (0.0, -0.16, 0.0)), INNARDS)
            // A stalk up out of it to the back of the eye.
            .add(
                Lump::limb(
                    (0.0, -0.1, -0.02),
                    (socket.x, socket.y - 0.01, socket.z - 0.04),
                    (0.06, 0.03),
                ),
                INNARDS,
            )
            .relief(Relief::Lumps {
                frequency: 9.0,
                depth: 0.012,
                seed: 39,
            })
            .skin(0.012, Paint::Wet.id());

        // Eight fingers hooked over the lower edge of the break, from
        // inside: each palm flat against the inside of the wall under the
        // edge, its fingers up over the edge and down the outside.
        let grip = |side: FootSide, azimuth: f32| {
            let size = HAND;
            let elevation = Self::rim(&shell, azimuth, 0.15 * size / 0.27);
            let (rim, out) = Self::on_shell(azimuth, elevation);
            let mut hand = anatomy::flesh(&anatomy::hand(size, GRIP, side), DIGIT_GRAIN);
            // Fingers that have scrabbled at the inside of a shell.
            for vertex in &mut hand.vertices {
                let along = (vertex.pos.z / size - 0.55).clamp(0.0, 1.0);
                let dirt = 1.0 - 0.45 * along;
                vertex.color.x *= dirt;
                vertex.color.y *= dirt * 0.96;
                vertex.color.z *= dirt * 0.9;
            }
            // Up the wall here, and the hand laid along that with its
            // back to the middle of the egg, then leant so its wrist is
            // deep inside: the wall curves, and a flat palm against it
            // would come out through it lower down.
            let wall = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), azimuth)
                * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -(FRAC_PI_2 + elevation));
            let leant = wall * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), LEAN);
            // Its knuckles inside the wall and just proud of the edge, as
            // far in as its fingers reach back to the outside.
            let knuckles = leant * Vector3::new(0.0, 0.0, 0.44 * size);
            let inside = rim - out * (SHELL_THICKNESS + 0.11 * size) + wall * Vector3::z() * 0.006;
            placed(hand, leant, inside - knuckles)
        };

        // What has run from the break and down the shell, and dried there
        // still shining.
        let mut leak = Clay::new(0.006);
        for (azimuth, length) in [(-0.34f32, 0.55f32), (0.05, 0.3), (0.3, 0.42)] {
            let top = BREAK_ELEVATION - 0.36;
            let mut from = Self::on_shell(azimuth, top).0;
            let runs = 6;
            for run in 1..=runs {
                let down = top - length * run as f32 / runs as f32;
                let wander = 0.03 * (run as f32 * 1.7 + azimuth * 9.0).sin();
                let (to, out) = Self::on_shell(azimuth + wander, down);
                let to = to + out * 0.002;
                let thick = 0.006 - 0.0005 * run as f32;
                leak = leak.add(
                    Lump::limb(
                        (from.x, from.y, from.z),
                        (to.x, to.y, to.z),
                        (thick, 0.9 * thick),
                    ),
                    ICHOR,
                );
                from = to;
            }
            // A bead hanging at the end of it.
            leak = leak.add(Lump::ball(0.008, (from.x, from.y - 0.003, from.z)), ICHOR);
        }
        let leak = leak.skin(0.0028, Paint::Wet.id());

        let eye = Joint::Eye {
            socket,
            swivel: 0.55,
        };
        let lid = |shut: f32| Joint::Hinge {
            pivot: socket,
            axis: Vector3::x(),
            drive: Drive::Lid { shut },
        };
        let mut pieces = vec![
            (Worn::on(Joint::Head), vec![Self::shell(&shell)]),
            (Worn::on(Joint::Head).breathing(0.035), vec![innards]),
            (
                Worn::on(Joint::Head),
                vec![
                    grip(FootSide::Left, 0.17),
                    grip(FootSide::Right, -0.2),
                    leak,
                ],
            ),
            (
                Worn::on(eye),
                vec![anatomy::eyeball(
                    EYE_RADIUS,
                    EyeLook {
                        iris: (0.42, 0.3, 0.1),
                        iris_angle: 0.42,
                        pupil_share: 0.34,
                        bloodshot: 0.85,
                    },
                )],
            ),
            // A heavy upper lid that never quite clears the iris, and a
            // lower one swollen under it.
            (
                Worn::on(lid(FRAC_PI_2 - 0.5)),
                vec![anatomy::eyelid(EYE_RADIUS, 1.17, false, LID)],
            ),
            (
                Worn::on(lid(-0.45)),
                vec![anatomy::eyelid(EYE_RADIUS, 0.92, true, LID)],
            ),
        ];
        pieces.extend(bare_legs(&rig, 0.066, 0.21));
        Dress::of(pieces)
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
            speed: 4.4,
            sight_range: 18.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 6.0,
                blow: BUTT_BLOW,
                windup: BUTT_WINDUP,
                recovery: BUTT_RECOVERY,
            }),
            harm: Some(HarmConfig::default()),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for EggWalkerDef {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn its_dress_is_tens_of_thousands_of_triangles_and_every_part_is_worn() {
        let dress = EggWalkerDef::sculpt();
        assert_eq!(dress.worn.len(), dress.model.parts.len());
        let triangles = dress.triangles();
        println!("an egg-walker is {triangles} triangles");
        assert!(
            (30_000..400_000).contains(&triangles),
            "{triangles} triangles"
        );
    }
}
