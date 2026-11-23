//! Fish-walker — a carp the length of a man's arm, carried level on a
//! man's bare legs, drowning in the air.
//!
//! The amphibian. It walks the shore and walks on into the water after
//! whoever it is following, so a lake is no refuge from it. It gapes and
//! bites. `visual_bench fish_walker` shows it.
//!
//! ```text
//!   trunk   scaled, dark-backed, the belly blown and pale and let down
//!           over the hips; holed beneath for the legs; its fins on it,
//!           limp and torn; raw gills in the hollow behind the head     (Head, breathing)
//!   head    bare bone-plated skull, gill covers stood open, a man's
//!           upper teeth in a red gum under a thin lip, a hook rusted
//!           into the corner of the mouth                               (Head)
//!   jaw     the lower jaw, its teeth and tongue, and the barbels
//!           under its chin, working for air                            (Hinge, Gasp)
//!   eyes    one gold and filming over, one already blind               (SideEye)
//!   legs    bare thighs, shins and feet, slimed from above             (the bones)
//! ```

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::anatomy;
use super::fish_anatomy::{
    crown, flay, glass_eye, moved, point, sided, surface_along, Crevices, Fin, FinPaint, GlassLook,
    Hide, Ray, Runoff, RAW,
};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{bare_legs, Peck, StiltWalker};
use crate::animation::stilts::{Carriage, Dress, Drive, Joint, StiltRigConfig, Worn};
use crate::animation::FootPlacerConfig;
use crate::app::sculpt::{scatter, Blush, Clay, Ellipsoid, Lump, Mottle, Relief, VertexPaint};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::model::MeshPrimitive;
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
/// The impulse its bite drives into what it strikes, N·s: about 5.5 m/s on
/// the player, which only just knocks them down.
const BITE_BLOW: f32 = 310.0;

/// Side of the grid each part is skinned from, m: finest where the eye
/// goes.
const TRUNK_GRAIN: f32 = 0.007;
const HEAD_GRAIN: f32 = 0.0052;
const JAW_GRAIN: f32 = 0.0042;
const GILL_GRAIN: f32 = 0.005;
const BARBEL_GRAIN: f32 = 0.002;
const TOOTH_GRAIN: f32 = 0.0016;

/// What its hide is finished in: slime. Its fins are duller: a web that
/// shone like the hide would flash like a blade.
const HIDE: Paint = Paint::Wet;
const FIN: Paint = Paint::Horn;

/// Its hide before it is shaded (linear): the brass of a carp's flank.
const FLANK: (f32, f32, f32) = (0.36, 0.27, 0.11);
/// The bare plates of its head.
const PLATE: (f32, f32, f32) = (0.3, 0.24, 0.12);
/// Lips and barbels: the hide gone soft and pale.
const LIP: (f32, f32, f32) = (0.36, 0.26, 0.17);
/// The gum its teeth are set in.
const GUM: (f32, f32, f32) = (0.34, 0.07, 0.07);
const TONGUE: (f32, f32, f32) = (0.19, 0.045, 0.05);
/// A tooth, and what stains it at the gum.
const TOOTH: (f32, f32, f32) = (0.74, 0.68, 0.5);
const TARTAR: (f32, f32, f32) = (0.42, 0.3, 0.14);
/// Rust.
const RUST: (f32, f32, f32) = (0.3, 0.13, 0.06);

/// Where the lower jaw turns, in the head's frame.
const JAW_PIVOT: (f32, f32, f32) = (0.0, -0.075, 0.262);
/// How far it drops, radians, and how many times a second it gasps.
const GASP: f32 = 0.42;
const GASP_RATE: f32 = 1.25;

/// The arch its teeth and lips follow: how far forward of the head's
/// middle the arch's own middle is, and how far round from its front the
/// lips and the teeth run, radians.
const ARCH_BACK: f32 = 0.385;
const LIP_SPAN: f32 = 1.35;
const TOOTH_SPAN: f32 = 1.15;
/// The arches of its upper and lower teeth (across, forward), and of the
/// lips outside them.
const UPPER_TEETH: (f32, f32) = (0.04, 0.05);
const LOWER_TEETH: (f32, f32) = (0.037, 0.046);
const UPPER_LIP: (f32, f32) = (0.05, 0.062);
const LOWER_LIP: (f32, f32) = (0.044, 0.053);
/// Heights of the upper gum line and the lower, in the head's frame.
const UPPER_GUM: f32 = -0.042;
const LOWER_GUM: f32 = -0.062;

/// Its eyes: their radius, how high and how far forward on the head they
/// sit, how much of their radius is sunk in it, and the way the right one
/// looks out (the left mirrors it).
const EYE_RADIUS: f32 = 0.028;
const EYE_AT: (f32, f32) = (0.045, 0.3);
const EYE_SUNK: f32 = 0.58;
const EYE_OUT: (f32, f32, f32) = (1.0, 0.06, 0.22);
/// How far an eye turns to follow what it watches, radians. Hardly at
/// all: enough to know it does.
const EYE_SWIVEL: f32 = 0.32;

/// Where the hook is set: one corner of the mouth, in the head's frame.
const HOOK_AT: (f32, f32, f32) = (-0.047, -0.036, 0.405);

/// How thick a thigh is at the hip, and how long a foot, m.
const THIGH_GIRTH: f32 = 0.07;
const FOOT_LENGTH: f32 = 0.23;

/// A fish-walker.
#[derive(Deserialize)]
pub struct FishWalkerDef {
    /// Position (x, z). It stands on the ground there.
    pub pos: (f32, f32),
}

/// The point `round` radians round an arch from its front, for an arch
/// `reach` (across, forward) about `ARCH_BACK` at `height`.
fn on_arch(round: f32, reach: (f32, f32), height: f32) -> Vector3<f32> {
    Vector3::new(
        reach.0 * round.sin(),
        height,
        ARCH_BACK + reach.1 * round.cos(),
    )
}

/// A roll of flesh `thick` through, along an arch as far as `span` round
/// from its front, pressed onto `clay`: a lip, or a gum.
fn roll(
    mut clay: Clay,
    reach: (f32, f32),
    height: f32,
    span: f32,
    thick: f32,
    colour: (f32, f32, f32),
) -> Clay {
    let steps = 10;
    let at = |step: usize| {
        let round = -span + 2.0 * span * step as f32 / steps as f32;
        // Thinner toward the corners of the mouth, and never quite even.
        let swell = 1.0 - 0.3 * (round / span).powi(2) + 0.12 * scatter(step as u32, 5);
        (on_arch(round, reach, height), thick * swell)
    };
    for step in 0..steps {
        let ((from, thick_from), (to, thick_to)) = (at(step), at(step + 1));
        clay = clay.add(
            Lump::Limb {
                from,
                to,
                radii: (thick_from, thick_to),
            },
            colour,
        );
    }
    clay
}

/// A row of a man's teeth along an arch from the gum at `height`, standing
/// `tall` along `up` (+1 or −1): uneven, stained at the gum, one gone
/// dark, with gaps where `missing` says.
fn teeth(reach: (f32, f32), height: f32, up: f32, tall: f32, missing: &[usize]) -> MeshPrimitive {
    let count = 9;
    let round = |tooth: usize| -TOOTH_SPAN + 2.0 * TOOTH_SPAN * tooth as f32 / (count - 1) as f32;
    let mut clay = Clay::new(0.0012);
    for tooth in (0..count).filter(|tooth| !missing.contains(tooth)) {
        let root = on_arch(round(tooth), reach, height);
        let next = on_arch(
            round(tooth) + 2.0 * TOOTH_SPAN / (count - 1) as f32,
            reach,
            height,
        );
        let out = Vector3::new(
            round(tooth).sin() * reach.1,
            0.0,
            round(tooth).cos() * reach.0,
        );
        // Each its own size and lean.
        let key = tooth as u32 + if up > 0.0 { 40 } else { 0 };
        let lean = Vector3::new(0.12 * scatter(key, 1), up, 0.1 * scatter(key, 2));
        let size = (
            0.94 * (next - root).norm() * (1.0 + 0.1 * scatter(key, 3)),
            tall * (1.0 + 0.16 * scatter(key, 4)),
            0.008,
        );
        let colour = if tooth == 6 { TARTAR } else { TOOTH };
        clay = clay.add(crown(root, lean, out, size), colour);
    }
    let mut mesh = clay.skin(TOOTH_GRAIN, Paint::Horn.id());
    let tartar = point(TARTAR).component_div(&point(TOOTH));
    for vertex in &mut mesh.vertices {
        let from_gum = ((vertex.pos.y - height) * up / tall).clamp(0.0, 1.0);
        let stain = (1.0 - from_gum / 0.5).max(0.0);
        let wash = Vector3::repeat(1.0).lerp(&tartar, 0.85 * stain);
        vertex.color.x *= wash.x;
        vertex.color.y *= wash.y;
        vertex.color.z *= wash.z;
    }
    Mottle {
        frequency: 160.0,
        value: 0.12,
        warmth: 0.08,
        seed: 61,
    }
    .paint(&mut mesh);
    mesh
}

/// A carp's hide, between a belly at `span.0` and a back at `span.1`.
fn hide(span: (f32, f32)) -> Hide {
    Hide {
        span,
        back: (0.2, 0.26, 0.24),
        belly: (1.55, 1.8, 3.2),
        sick: 1.0,
        seed: 51,
    }
}

impl FishWalkerDef {
    /// Its proportions: a man's legs to the hip, and a fish from there up.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.42,
            lower_leg_length: 0.4,
            ankle_height: anatomy::ANKLE_RISE,
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
            carriage: Carriage::default(),
        }
    }

    /// How far below the head's middle the hips are.
    fn hips(rig: &StiltRigConfig) -> f32 {
        -(rig.neck_length + rig.haunch_height)
    }

    /// The fish behind its head, whole: before it is scaled, or opened
    /// for the legs. About the head's middle, as every part of it is.
    fn trunk_form() -> Clay {
        Clay::new(0.07)
            .add(Lump::mass((0.105, 0.19, 0.3), (0.0, -0.02, -0.03)), FLANK)
            // The belly: blown, and let down over the hips like a bag.
            .add(
                Lump::mass((0.195, 0.118, 0.175), (0.0, -0.195, -0.01)),
                FLANK,
            )
            // The ridge of the back, and the neck the head is set on.
            .add(Lump::mass((0.055, 0.08, 0.22), (0.0, 0.11, -0.04)), FLANK)
            .add(Lump::mass((0.07, 0.16, 0.12), (0.0, -0.01, 0.17)), FLANK)
            // The wrist of the tail.
            .add(Lump::mass((0.036, 0.075, 0.11), (0.0, 0.015, -0.34)), FLANK)
    }

    /// The hole a leg comes through, on `side`: what is cut out of the
    /// belly for it. Longer than it is wide, for the thigh to swing in.
    fn leg_hole(rig: &StiltRigConfig, side: f32) -> [Lump; 2] {
        let hips = Self::hips(rig);
        let x = side * rig.hip_width;
        [-0.022, 0.022].map(|z| Lump::limb((x, hips + 0.04, z), (x, hips - 0.3, z), (0.08, 0.08)))
    }

    /// The hollow behind a gill cover, on `side`, that the gills lie in.
    fn gill_hollow(side: f32) -> Lump {
        Lump::limb(
            sided(side, (0.1, 0.05, 0.1)),
            sided(side, (0.102, -0.085, 0.095)),
            (0.026, 0.028),
        )
    }

    /// The trunk: scaled, opened for the legs, and painted.
    fn trunk(rig: &StiltRigConfig) -> MeshPrimitive {
        let hips = Self::hips(rig);
        let mut clay = Self::trunk_form();
        let mut raw = Clay::new(0.0);
        for side in [-1.0, 1.0] {
            for lump in Self::leg_hole(rig, side) {
                clay = clay.cut(lump, 0.014);
                raw = raw.add(lump, RAW);
            }
            clay = clay.cut(Self::gill_hollow(side), 0.008);
            raw = raw.add(Self::gill_hollow(side), RAW);
        }
        // An old gash along one flank, healed badly.
        let scar = [
            (-0.112, 0.04, -0.02),
            (-0.112, 0.01, -0.09),
            (-0.104, 0.0, -0.15),
        ];
        for pair in scar.windows(2) {
            clay = clay.cut(Lump::limb(pair[0], pair[1], (0.009, 0.007)), 0.004);
        }
        let clay = clay.relief(Relief::Shingles {
            origin: Vector3::new(0.0, -0.06, 0.0),
            toward: -Vector3::z(),
            around: 22,
            row: 0.025,
            depth: 0.005,
        });

        let mut mesh = clay.skin(TRUNK_GRAIN, HIDE.id());
        hide((-0.31, 0.2)).paint(&mut mesh);
        Crevices {
            depth: 0.6,
            lift: 0.25,
        }
        .paint(&mut mesh);
        // Blood under the skin where the legs come through, and along
        // the old gash.
        for side in [-1.0, 1.0] {
            Blush {
                at: Vector3::new(side * rig.hip_width, hips - 0.06, 0.0),
                radius: 0.15,
                tint: Vector3::new(1.0, 0.5, 0.48),
            }
            .paint(&mut mesh);
        }
        for at in scar {
            Blush {
                at: point(at),
                radius: 0.03,
                tint: Vector3::new(1.5, 0.7, 0.7),
            }
            .paint(&mut mesh);
        }
        // Inside the holes and the hollows it is raw, and dark further in.
        flay(&mut mesh, |at, _| {
            (raw.distance(at) < 0.004).then(|| {
                if at.z > 0.05 {
                    0.25
                } else {
                    (at.y - (hips - 0.06)) / 0.07
                }
            })
        });
        mesh
    }

    /// The gills in the hollow behind one gill cover: raw combs.
    fn gills(side: f32) -> MeshPrimitive {
        let mut clay = Clay::new(0.004);
        for arch in 0..3 {
            // Each further out than the last, as the cover over them is.
            let x = 0.079 + 0.008 * arch as f32;
            let z = 0.118 - 0.019 * arch as f32;
            let bar = [
                (x, 0.055, z + 0.008),
                (x + 0.006, -0.02, z - 0.004),
                (x, -0.09, z + 0.008),
            ];
            for pair in bar.windows(2) {
                clay = clay.add(
                    Lump::limb(sided(side, pair[0]), sided(side, pair[1]), (0.011, 0.011)),
                    RAW,
                );
            }
        }
        let mut mesh = clay
            .relief(Relief::Folds {
                along: Vector3::z(),
                frequency: 70.0,
                depth: 0.0035,
                seed: 56,
            })
            .skin(GILL_GRAIN, Paint::Wet.id());
        Mottle {
            frequency: 40.0,
            value: 0.3,
            warmth: 0.1,
            seed: 57,
        }
        .paint(&mut mesh);
        Crevices {
            depth: 0.7,
            lift: 0.5,
        }
        .paint(&mut mesh);
        mesh
    }

    /// The bare mass of the head: what its features are set on.
    fn skull() -> Clay {
        Clay::new(0.045)
            .add(
                Lump::mass((0.088, 0.165, 0.165), (0.0, -0.015, 0.215)),
                PLATE,
            )
            .add(
                Lump::limb((0.0, -0.005, 0.3), (0.0, -0.02, 0.4), (0.075, 0.042)),
                PLATE,
            )
    }

    /// The point on the skull's side `height` up and `forward` along it.
    fn on_skull(side: f32, height: f32, forward: f32) -> Vector3<f32> {
        surface_along(
            &Self::skull(),
            Vector3::new(0.0, height, forward),
            Vector3::x() * side,
        )
    }

    /// Where the middle of the eye on `side` is, in the head's frame.
    fn eye_socket(side: f32) -> Vector3<f32> {
        Self::on_skull(side, EYE_AT.0, EYE_AT.1) - Vector3::x() * (side * EYE_SUNK * EYE_RADIUS)
    }

    /// The head, whole: before the mouth and the sockets are cut.
    fn head_form() -> Clay {
        let mut clay = Self::skull().smoothing(0.016);
        for side in [-1.0, 1.0] {
            // A gill cover, stood open at the back.
            clay = clay.add(
                Lump::Ball(
                    Ellipsoid::new((0.012, 0.125, 0.085), sided(side, (0.09, -0.03, 0.15)))
                        .yawed(-0.4 * side),
                ),
                PLATE,
            );
            // The ridge of bone before it, from behind the eye down to
            // the hinge of the jaw.
            let (high, low) = (
                Self::on_skull(side, 0.06, 0.235),
                Self::on_skull(side, -0.1, 0.25),
            );
            clay = clay.add(
                Lump::Limb {
                    from: high,
                    to: low,
                    radii: (0.009, 0.012),
                },
                PLATE,
            );
            // The ring of bone the eye is set in: heavy over it, like a
            // brow.
            let beads = 12;
            for bead in 0..beads {
                let round = bead as f32 / beads as f32 * TAU;
                let reach = EYE_RADIUS + 0.003;
                let at = Self::on_skull(
                    side,
                    EYE_AT.0 + reach * round.sin(),
                    EYE_AT.1 + reach * round.cos(),
                );
                let heavy = 0.0032 + 0.0022 * round.sin().max(0.0) + 0.001 * scatter(bead, 9);
                clay = clay.add(Lump::ball(heavy, (at.x, at.y, at.z)), PLATE);
            }
        }
        // A thin upper lip, drawn back off the gum.
        let clay = roll(
            clay.smoothing(0.008),
            UPPER_LIP,
            UPPER_GUM + 0.008,
            LIP_SPAN,
            0.008,
            LIP,
        );
        roll(
            clay,
            UPPER_TEETH,
            UPPER_GUM + 0.003,
            TOOTH_SPAN + 0.1,
            0.0075,
            GUM,
        )
    }

    /// What is cut out of the head for its mouth: the room the lower jaw
    /// shuts into, and the vault of the palate over it.
    fn mouth() -> [Lump; 2] {
        [
            Lump::mass((0.09, 0.075, 0.115), (0.0, UPPER_GUM - 0.073, 0.38)),
            Lump::mass((0.03, 0.022, 0.07), (0.0, UPPER_GUM, 0.358)),
        ]
    }

    /// The head: its skull, lip and gum, sockets cut for the eyes.
    fn head() -> MeshPrimitive {
        let mut clay = Self::head_form();
        let mut mouth = Clay::new(0.0);
        for lump in Self::mouth() {
            clay = clay.cut(lump, 0.006);
            mouth = mouth.add(lump, RAW);
        }
        for side in [-1.0, 1.0] {
            let socket = Self::eye_socket(side);
            clay = clay
                .cut(
                    Lump::ball(EYE_RADIUS + 0.0012, (socket.x, socket.y, socket.z)),
                    0.003,
                )
                // A nostril.
                .cut(Lump::ball(0.006, (side * 0.03, 0.012, 0.416)), 0.003);
        }
        // Bone under thin skin: pitted, and never quite smooth.
        let clay = clay
            .relief(Relief::Lumps {
                frequency: 16.0,
                depth: 0.004,
                seed: 54,
            })
            .relief(Relief::Lumps {
                frequency: 70.0,
                depth: 0.0014,
                seed: 74,
            });

        let mut mesh = clay.skin(HEAD_GRAIN, HIDE.id());
        hide((-0.18, 0.15)).paint(&mut mesh);
        Crevices {
            depth: 0.5,
            lift: 0.15,
        }
        .paint(&mut mesh);
        for side in [-1.0, 1.0] {
            // Sore round the eye, and along the edge of the gill cover.
            Blush {
                at: Self::on_skull(side, EYE_AT.0, EYE_AT.1),
                radius: 0.042,
                tint: Vector3::new(1.0, 0.6, 0.56),
            }
            .paint(&mut mesh);
            Blush {
                at: Vector3::new(side * 0.125, -0.03, 0.075),
                radius: 0.05,
                tint: Vector3::new(1.0, 0.45, 0.42),
            }
            .paint(&mut mesh);
        }
        // Where the hook went in.
        Blush {
            at: point(HOOK_AT),
            radius: 0.028,
            tint: Vector3::new(1.2, 0.45, 0.45),
        }
        .paint(&mut mesh);
        // Each socket is dark round its eye.
        for side in [-1.0, 1.0] {
            let socket = Self::eye_socket(side);
            flay(&mut mesh, |at, _| {
                ((at - socket).norm() < EYE_RADIUS + 0.004).then_some(0.7)
            });
        }
        // The roof of the mouth is raw, and dark toward the throat.
        flay(&mut mesh, |at, normal| {
            (mouth.distance(at) < 0.003 && normal.y < 0.3 && at.y > UPPER_GUM - 0.05)
                .then(|| (0.43 - at.z) / 0.12)
        });
        mesh
    }

    /// Barbels: a pair under the chin and a shorter pair behind them,
    /// hanging. In the head's frame, with the jaw shut.
    fn barbels() -> MeshPrimitive {
        let mut clay = Clay::new(0.002);
        let strands: [&[(f32, f32, f32)]; 2] = [
            &[
                (0.016, -0.082, 0.41),
                (0.02, -0.104, 0.412),
                (0.022, -0.128, 0.406),
                (0.02, -0.146, 0.394),
                (0.016, -0.156, 0.38),
            ],
            &[
                (0.03, -0.09, 0.372),
                (0.036, -0.108, 0.37),
                (0.038, -0.124, 0.362),
            ],
        ];
        for side in [-1.0, 1.0] {
            for (strand, joints) in strands.into_iter().enumerate() {
                // No two hang quite alike.
                let sway = 0.005 * scatter(strand as u32 + if side > 0.0 { 2 } else { 0 }, 12);
                for (step, pair) in joints.windows(2).enumerate() {
                    let hung = |p: (f32, f32, f32), far: usize| {
                        sided(side, (p.0 + sway * far as f32, p.1, p.2))
                    };
                    let thick = 0.0052 - 0.0008 * step as f32;
                    clay = clay.add(
                        Lump::limb(
                            hung(pair[0], step),
                            hung(pair[1], step + 1),
                            (thick, thick - 0.0008),
                        ),
                        LIP,
                    );
                }
            }
        }
        let mut mesh = clay.skin(BARBEL_GRAIN, HIDE.id());
        Mottle {
            frequency: 50.0,
            value: 0.25,
            warmth: 0.2,
            seed: 75,
        }
        .paint(&mut mesh);
        mesh
    }

    /// The lower jaw, its tongue and its teeth, about the jaw's pivot.
    fn jaw() -> Vec<MeshPrimitive> {
        let mut clay = Clay::new(0.026)
            // The knuckle it turns on, which is the same whichever way it
            // is turned.
            .add(Lump::mass((0.058, 0.04, 0.05), (0.0, -0.097, 0.262)), PLATE)
            .add(Lump::mass((0.04, 0.02, 0.08), (0.0, -0.092, 0.35)), PLATE)
            // The wattle under it.
            .add(Lump::mass((0.03, 0.022, 0.05), (0.0, -0.116, 0.31)), LIP);
        for side in [-1.0, 1.0] {
            clay = clay
                .add(
                    Lump::limb(
                        sided(side, (0.052, -0.087, 0.27)),
                        sided(side, (0.038, -0.073, 0.39)),
                        (0.017, 0.014),
                    ),
                    PLATE,
                )
                .add(
                    Lump::limb(
                        sided(side, (0.038, -0.073, 0.39)),
                        (0.0, -0.069, 0.434),
                        (0.014, 0.013),
                    ),
                    PLATE,
                );
        }
        let clay = roll(
            clay.smoothing(0.008),
            LOWER_LIP,
            LOWER_GUM - 0.001,
            LIP_SPAN,
            0.008,
            LIP,
        );
        let trough = Lump::mass((0.03, 0.02, 0.06), (0.0, LOWER_GUM + 0.007, 0.36));
        let clay = roll(
            clay,
            LOWER_TEETH,
            LOWER_GUM - 0.002,
            TOOTH_SPAN + 0.1,
            0.007,
            GUM,
        )
        .cut(trough, 0.006)
        .relief(Relief::Lumps {
            frequency: 50.0,
            depth: 0.0016,
            seed: 58,
        });
        let mut flesh = clay.skin(JAW_GRAIN, HIDE.id());
        hide((-0.18, 0.15)).paint(&mut flesh);
        let floor = Clay::new(0.0).add(trough, RAW);
        flay(&mut flesh, |at, _| {
            (floor.distance(at) < 0.003).then(|| (0.43 - at.z) / 0.12)
        });

        // The tongue: too thick for the mouth, and lolling to the side
        // where a tooth is gone.
        let tongue = Clay::new(0.012)
            .add(
                Lump::limb((0.0, -0.074, 0.29), (0.004, -0.069, 0.35), (0.015, 0.013)),
                TONGUE,
            )
            .add(
                Lump::limb((0.004, -0.069, 0.35), (0.016, -0.06, 0.394), (0.013, 0.01)),
                TONGUE,
            )
            .add(
                Lump::mass((0.011, 0.006, 0.012), (0.022, -0.055, 0.407)),
                TONGUE,
            )
            .relief(Relief::Lumps {
                frequency: 70.0,
                depth: 0.0016,
                seed: 59,
            });
        let mut tongue = tongue.skin(JAW_GRAIN, Paint::Wet.id());
        Mottle {
            frequency: 30.0,
            value: 0.2,
            warmth: 0.12,
            seed: 60,
        }
        .paint(&mut tongue);

        let lower = teeth(LOWER_TEETH, LOWER_GUM, 1.0, 0.017, &[6]);
        let to_pivot = -point(JAW_PIVOT);
        [flesh, tongue, lower, Self::barbels()]
            .into_iter()
            .map(|mesh| moved(mesh, UnitQuaternion::identity(), to_pivot))
            .collect()
    }

    /// The hook it once took, rusted into the corner of its mouth, and a
    /// length of the line it broke.
    fn hook() -> Vec<MeshPrimitive> {
        let at = point(HOOK_AT);
        // The bend: through the lip, down, and round and up to the barb.
        let bend: [(f32, f32, f32); 7] = [
            (-0.004, 0.014, -0.004),
            (0.004, 0.0, 0.002),
            (0.008, -0.024, 0.004),
            (0.006, -0.048, 0.006),
            (-0.009, -0.061, 0.008),
            (-0.024, -0.05, 0.01),
            (-0.026, -0.032, 0.01),
        ];
        let mut iron = Clay::new(0.001);
        for pair in bend.windows(2) {
            iron = iron.add(
                Lump::Limb {
                    from: at + point(pair[0]),
                    to: at + point(pair[1]),
                    radii: (0.003, 0.003),
                },
                RUST,
            );
        }
        // The barb.
        let tip = at + point(bend[6]);
        iron = iron.add(
            Lump::Limb {
                from: tip,
                to: tip + Vector3::new(0.008, -0.01, 0.0),
                radii: (0.0025, 0.001),
            },
            RUST,
        );
        let mut iron = iron
            .relief(Relief::Lumps {
                frequency: 260.0,
                depth: 0.0007,
                seed: 62,
            })
            .skin(0.0012, Paint::Husk.id());
        Mottle {
            frequency: 140.0,
            value: 0.4,
            warmth: 0.3,
            seed: 63,
        }
        .paint(&mut iron);

        // The line: gut, kinked, hanging from the eye of the hook.
        let line: [(f32, f32, f32); 6] = [
            (-0.004, 0.014, -0.004),
            (-0.012, 0.024, 0.002),
            (-0.02, 0.012, 0.01),
            (-0.022, -0.03, 0.016),
            (-0.03, -0.085, 0.012),
            (-0.026, -0.15, 0.02),
        ];
        let mut gut = Clay::new(0.0006);
        for pair in line.windows(2) {
            gut = gut.add(
                Lump::Limb {
                    from: at + point(pair[0]),
                    to: at + point(pair[1]),
                    radii: (0.0013, 0.0013),
                },
                (0.62, 0.58, 0.44),
            );
        }
        vec![iron, gut.skin(0.001, Paint::Horn.id())]
    }

    /// Its fins, in the head's frame: limp, and the worse for wear.
    fn fins() -> Vec<MeshPrimitive> {
        let trunk = Self::trunk_form();
        let paint = FinPaint {
            ray: (0.3, 0.25, 0.16),
            web: (0.2, 0.12, 0.08),
            edge: (0.44, 0.3, 0.2),
        };
        let top = |z: f32| surface_along(&trunk, Vector3::new(0.0, 0.0, z), Vector3::y());
        let under = |x: f32, z: f32| surface_along(&trunk, Vector3::new(x, 0.0, z), -Vector3::y());
        let share = |ray: usize, of: usize| ray as f32 / (of - 1) as f32;

        // The fin of the back: spines first, then soft rays, the whole of
        // it fallen over to one side.
        let dorsal = (0..12)
            .map(|ray| {
                let along = share(ray, 12);
                let root = top(0.11 - 0.34 * along) - Vector3::y() * 0.012;
                let tall = if ray < 3 {
                    0.085 + 0.022 * ray as f32
                } else {
                    0.13 - 0.085 * along
                };
                let flop = 0.25 + 0.5 * along + 0.12 * scatter(ray as u32, 11);
                Ray::new(
                    root,
                    root + Vector3::new(flop * tall, 0.8 * tall, -0.55 * tall),
                )
                .bowed(Vector3::new(-0.2 * flop * tall, 0.04 * tall, 0.0))
            })
            .collect();
        let dorsal = Fin::on(dorsal, paint, 64)
            .edged(0.2, 0.16)
            .bellied(0.004)
            .torn(&[(3.5, 0.9, 0.8), (6.0, 0.7, 0.45), (8.4, 1.1, 0.6)])
            .holed_at(&[((1.5, 0.55), (0.22, 0.12))]);

        // The tail: forked, the lower lobe bitten short.
        let tail = (0..15)
            .map(|ray| {
                let across = share(ray, 15) * 2.0 - 1.0;
                let root = Vector3::new(0.0, 0.015 + 0.05 * across, -0.415);
                let spread = 0.95 * across;
                let long = 0.1 + 0.11 * across.abs().powf(1.4);
                let sway = 0.03 * (across * 2.2 + 0.4).sin();
                Ray::new(
                    root,
                    root + Vector3::new(sway, spread.sin(), -spread.cos()) * long,
                )
                .bowed(Vector3::new(0.5 * sway, 0.0, 0.0))
            })
            .collect();
        let tail = Fin::on(tail, paint, 65)
            .edged(0.12, 0.2)
            .torn(&[
                (1.6, 3.6, 0.5),
                (5.5, 0.7, 0.7),
                (9.5, 0.6, 0.85),
                (12.0, 0.5, 0.4),
            ])
            .holed_at(&[((11.4, 0.5), (0.3, 0.1))]);

        // The fin under the tail.
        let anal = (0..7)
            .map(|ray| {
                let along = share(ray, 7);
                let root = under(0.0, -0.21 - 0.1 * along) + Vector3::y() * 0.01;
                let long = 0.1 - 0.05 * along;
                Ray::new(
                    root,
                    root + Vector3::new(-0.2 * long, -0.75 * long, -0.6 * long),
                )
            })
            .collect();
        let anal = Fin::on(anal, paint, 66).torn(&[(2.5, 0.8, 0.6)]);

        // A fin behind each gill cover, hanging; the left one in ribbons.
        let breast = |side: f32| {
            (0..8)
                .map(|ray| {
                    let across = share(ray, 8);
                    let root =
                        Vector3::new(side * 0.1, -0.105 - 0.03 * across, 0.045 - 0.012 * across);
                    let swing = -0.5 + 1.5 * across;
                    let long = 0.15 - 0.05 * (across - 0.4).abs();
                    let way =
                        Vector3::new(side * 0.42, -swing.cos() * 0.85, -swing.sin() * 0.85 - 0.1);
                    Ray::new(root, root + way * long).bowed(Vector3::new(side * 0.012, 0.0, 0.0))
                })
                .collect()
        };
        let right = Fin::on(breast(1.0), paint, 67).torn(&[(5.5, 0.6, 0.35)]);
        let left = Fin::on(breast(-1.0), paint, 68).torn(&[
            (1.5, 0.8, 0.85),
            (3.5, 0.7, 0.9),
            (5.5, 0.9, 0.75),
        ]);

        // A small fin each side of the belly, before the legs.
        let belly = |side: f32| {
            (0..7)
                .map(|ray| {
                    let across = share(ray, 7);
                    let root = under(side * 0.06, 0.12 - 0.045 * across) + Vector3::y() * 0.01;
                    let long = 0.1 - 0.035 * across;
                    Ray::new(
                        root,
                        root + Vector3::new(side * 0.3, -0.7, -0.45 - 0.5 * across) * long,
                    )
                })
                .collect()
        };

        let mut fins = vec![
            dorsal.mesh(7, 18, FIN.id()),
            tail.mesh(6, 22, FIN.id()),
            anal.mesh(6, 14, FIN.id()),
            right.mesh(6, 16, FIN.id()),
            left.mesh(6, 16, FIN.id()),
            Fin::on(belly(1.0), paint, 76).mesh(6, 12, FIN.id()),
            Fin::on(belly(-1.0), paint, 77)
                .torn(&[(2.5, 1.2, 0.7)])
                .mesh(6, 12, FIN.id()),
        ];
        for fin in &mut fins {
            Mottle {
                frequency: 22.0,
                value: 0.3,
                warmth: 0.25,
                seed: 68,
            }
            .paint(fin);
        }
        fins
    }

    /// One eye, about its own middle and looking along +Z.
    fn eye(blind: bool) -> MeshPrimitive {
        glass_eye(
            EYE_RADIUS,
            GlassLook {
                iris: (0.22, 0.15, 0.04),
                iris_angle: 1.55,
                pupil_share: 0.4,
                bloodshot: 0.9,
                film: if blind { 0.9 } else { 0.16 },
            },
        )
    }

    /// Bare legs, slimed: what runs off a fish runs down them.
    fn legs(rig: &StiltRigConfig) -> Vec<(Worn, Vec<MeshPrimitive>)> {
        let mut legs = bare_legs(rig, THIGH_GIRTH, FOOT_LENGTH);
        for (worn, meshes) in &mut legs {
            // A thigh is wet from its top; a shin less, from its knee.
            let (above, trail) = match worn.joint {
                Joint::Thigh(_) => (-0.06, 0.3),
                Joint::Shin(_) => (0.0, 0.16),
                _ => continue,
            };
            for mesh in meshes {
                Runoff {
                    above,
                    trail,
                    tint: Vector3::new(0.58, 0.64, 0.52),
                    seed: 70,
                }
                .paint(mesh);
            }
        }
        legs
    }

    /// Its dress, against the palette's slots.
    pub fn sculpt() -> Dress {
        let rig = Self::rig();
        let mut body = vec![Self::trunk(&rig), Self::gills(1.0), Self::gills(-1.0)];
        body.extend(Self::fins());

        let mut head = vec![
            Self::head(),
            teeth(UPPER_TEETH, UPPER_GUM, -1.0, 0.02, &[2]),
        ];
        head.extend(Self::hook());

        let eye = |side: f32| Joint::SideEye {
            socket: Self::eye_socket(side),
            out: point(sided(side, EYE_OUT)),
            swivel: EYE_SWIVEL,
        };
        let jaw = Joint::Hinge {
            pivot: point(JAW_PIVOT),
            axis: Vector3::x(),
            drive: Drive::Gasp {
                open: GASP,
                rate: GASP_RATE,
            },
        };
        let mut pieces = vec![
            // Its flanks heave.
            (Worn::on(Joint::Head).breathing(0.03), body),
            (Worn::on(Joint::Head), head),
            (Worn::on(jaw), Self::jaw()),
            (Worn::on(eye(1.0)), vec![Self::eye(false)]),
            (Worn::on(eye(-1.0)), vec![Self::eye(true)]),
        ];
        pieces.extend(Self::legs(&rig));
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
            speed: 3.2,
            sight_range: 22.0,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: 10.0,
                blow: BITE_BLOW,
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

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn its_dress_is_tens_of_thousands_of_triangles_and_every_part_is_worn() {
        let started = Instant::now();
        let dress = FishWalkerDef::sculpt();
        println!("a fish-walker is sculpted in {:?}", started.elapsed());
        assert_eq!(dress.worn.len(), dress.model.parts.len());
        let triangles = dress.triangles();
        println!("a fish-walker is {triangles} triangles");
        assert!(
            (30_000..250_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn its_eyes_are_set_in_the_sides_of_its_head_and_look_out_of_them() {
        for side in [-1.0, 1.0] {
            let socket = FishWalkerDef::eye_socket(side);
            assert!(socket.x * side > 0.03, "an eye at {socket:?}");
            // Sunk in the skull, and standing out of it.
            let skull = FishWalkerDef::skull();
            assert!(skull.distance(socket) < 0.0);
            assert!(skull.distance(socket + Vector3::x() * (side * EYE_RADIUS)) > 0.0);
        }
    }
}
