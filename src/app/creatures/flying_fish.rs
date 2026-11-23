//! Flying fish — a fish that looks a day dead, held up on a pair of skin
//! wings it was never meant to have.
//!
//! The air threat. It holds a height nothing on foot can reach, comes over
//! whoever it has seen, and stoops to bite when it is over them; standing
//! in the open is what it punishes. It is frail: one good blow, or a blast
//! anywhere near it, ends it. `visual_bench flying_fish` shows it.
//!
//! ```text
//!   body    a scaled trunk, hunched at the shoulder and heavy under; a
//!           fish's own head, plated, the gill covers stood open on raw
//!           gills that have bled down its flanks, a sunken filmed eye
//!           flat in each side, the jaw hung slack on small teeth and a
//!           dark mouth; a crest of spines, a tail and belly fins in
//!           ribbons                                              (part 0)
//!   wings   a fin's rays grown into fingers, knuckled, a web of veined
//!           skin between them, holed and torn along its edge     (parts 1, 2)
//! ```
//!
//! The wings are parts of their own, modelled where they are at rest and
//! turned on their roots by [`FlyerLocomotionSystem`](crate::creature::FlyerLocomotionSystem).

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::fish_anatomy::{
    flay, glass_eye, moved, point, sided, surface_along, Crevices, Fin, FinPaint, GlassLook, Hide,
    Ray, RAW,
};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::sculpted::{all_round, ground_under, Exhibit};
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::character::CharacterIntent;
use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Flyer, MeleeAttack, Swing, Wing, WingBeat};
use crate::damage::Health;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// The name its look is kept under.
const KIND: &str = "flying_fish";

/// Radius of the ball it collides as, m.
const RADIUS: f32 = 0.25;
/// Kilograms to the cubic metre.
const DENSITY: f32 = 250.0;
/// Level speed, m/s, and seconds to reach it.
const SPEED: f32 = 6.5;
const ACCEL_TIME: f32 = 0.8;
/// Height it holds over the ground, and the distance from its target
/// within which it comes down, m.
const CRUISE_HEIGHT: f32 = 6.0;
const STOOP_RANGE: f32 = 5.0;
/// How far it sees, m.
const SIGHT: f32 = 30.0;
/// Its bite: hit points, reach, and the seconds before and after.
const BITE: f32 = 8.0;
const REACH: f32 = 1.1;
const BITE_WINDUP: f32 = 0.25;
const BITE_RECOVERY: f32 = 1.6;
/// The impulse its bite drives into what it strikes, N·s: about 5.5 m/s on
/// the player, which only just knocks them down.
const BITE_BLOW: f32 = 310.0;
/// How far its bite carries its body ahead of its centre, m: it bites what
/// its own sphere touches on the way.
const BITE_LENGTH: f32 = 0.4;
/// Hit points it can take.
const HEALTH: f32 = 14.0;

/// The parts of its model: the body, and the wing on each side.
const BODY: usize = 0;
const RIGHT_WING: usize = 1;
const LEFT_WING: usize = 2;

/// Where a wing turns on the body, on the right (the left mirrors it), in
/// the entity's frame, and the rays it is stretched on.
const WING_ROOT: (f32, f32, f32) = (0.068, 0.072, 0.0);
const WING_RAYS: usize = 6;
/// How far a wing swings above and below its rest, how far it feathers,
/// radians, and its beats a second gliding and working.
const WING_RAISE: f32 = 0.8;
const WING_LOWER: f32 = 0.6;
const WING_FEATHER: f32 = 0.22;
const GLIDE_BEATS: f32 = 1.5;
const HARD_BEATS: f32 = 4.2;

/// Side of the grid each part is skinned from, m.
const TRUNK_GRAIN: f32 = 0.0055;
const HEAD_GRAIN: f32 = 0.0042;
const BONE_GRAIN: f32 = 0.005;
const TOOTH_GRAIN: f32 = 0.0016;

/// Its hide before it is shaded (linear): pewter.
const FLANK: (f32, f32, f32) = (0.34, 0.33, 0.3);
/// Its trunk's hide: a slate back and a pale belly.
const TRUNK_HIDE: Hide = Hide {
    span: (-0.17, 0.14),
    back: (0.26, 0.33, 0.42),
    belly: (1.7, 1.6, 1.5),
    sick: 0.7,
    seed: 31,
};
/// The plates of its head: dark along the crown, tin below.
const HEAD_HIDE: Hide = Hide {
    span: (-0.1, 0.09),
    back: (0.16, 0.18, 0.22),
    belly: (1.5, 1.5, 1.4),
    sick: 0.8,
    seed: 61,
};
/// The bare plates of its head.
const SKULL: (f32, f32, f32) = (0.27, 0.25, 0.2);
/// Soft parts: a throat, the meat of a shoulder.
const SOFT: (f32, f32, f32) = (0.42, 0.27, 0.22);
/// A tooth, and the bone of a finger.
const TOOTH: (f32, f32, f32) = (0.7, 0.66, 0.5);
const BONE: (f32, f32, f32) = (0.46, 0.4, 0.3);
/// Blood that has run and dried, and slime.
const GORE: (f32, f32, f32) = (0.1, 0.012, 0.01);
const SLIME: (f32, f32, f32) = (0.5, 0.5, 0.4);

/// Its eyes: their radius, how high and how far forward on the side of
/// the head they sit, and how much of their radius is sunk in it.
const EYE_RADIUS: f32 = 0.026;
const EYE_AT: (f32, f32) = (0.03, 0.262);
const EYE_SUNK: f32 = 0.62;

/// Where the lower jaw hangs from, in the entity's frame, and how far it
/// has dropped, radians.
const JAW_HINGE: (f32, f32, f32) = (0.0, -0.055, 0.25);
const GAPE: f32 = 0.36;
/// Teeth along each side of each jaw.
const TEETH: usize = 15;

/// A flying fish.
#[derive(Deserialize)]
pub struct FlyingFishDef {
    /// Position (x, z). It starts perched on the ground there, and takes
    /// off as soon as the game runs.
    pub pos: (f32, f32),
}

/// The edge of the upper jaw on `side`, `t` of the way from the corner of
/// the mouth to the tip of the snout: let down toward the corner, as a
/// fish's gape is.
fn upper_edge(side: f32, t: f32) -> Vector3<f32> {
    Vector3::new(
        side * 0.046 * (1.0 - t * t),
        -0.058 + 0.036 * t,
        0.262 + 0.162 * t,
    )
}

/// The edge of the lower jaw as it would be with the mouth shut.
fn lower_edge(side: f32, t: f32) -> Vector3<f32> {
    Vector3::new(
        side * 0.041 * (1.0 - t * t),
        -0.064 + 0.034 * t,
        0.262 + 0.152 * t,
    )
}

/// Where a point of the lower jaw is with the jaw hung open, given where
/// it would be with the mouth shut.
fn hung(shut: Vector3<f32>) -> Vector3<f32> {
    let hinge = point(JAW_HINGE);
    hinge + UnitQuaternion::from_axis_angle(&Vector3::x_axis(), GAPE) * (shut - hinge)
}

/// A strap of bone pressed onto `clay` along `edge` (a line given from 0
/// to 1), `thick.0` through at its start and `thick.1` at its end.
fn strap(clay: Clay, edge: impl Fn(f32) -> Vector3<f32>, thick: (f32, f32)) -> Clay {
    let joints = 4;
    (0..joints).fold(clay, |clay, joint| {
        let (from, to) = (
            joint as f32 / joints as f32,
            (joint + 1) as f32 / joints as f32,
        );
        let girth = |t: f32| thick.0 + (thick.1 - thick.0) * t;
        clay.add(
            Lump::Limb {
                from: edge(from),
                to: edge(to),
                radii: (girth(from), girth(to)),
            },
            SKULL,
        )
    })
}

impl FlyingFishDef {
    /// The fish behind its head, whole: before it is scaled. Nose along
    /// +Z, about the middle of the ball it collides as.
    fn trunk_form() -> Clay {
        let mut clay = Clay::new(0.05)
            .add(Lump::mass((0.082, 0.11, 0.31), (0.0, -0.005, -0.04)), FLANK)
            // Hunched at the shoulder, where the wings pull.
            .add(Lump::mass((0.085, 0.075, 0.12), (0.0, 0.065, 0.06)), FLANK)
            // A belly too heavy to be carried, hanging.
            .add(Lump::mass((0.08, 0.08, 0.15), (0.0, -0.095, 0.0)), FLANK)
            .add(
                Lump::limb((0.0, 0.0, -0.25), (0.0, 0.015, -0.43), (0.04, 0.018)),
                FLANK,
            );
        // The meat a wing is rooted in.
        for side in [-1.0, 1.0] {
            clay = clay.add(
                Lump::mass((0.028, 0.03, 0.12), sided(side, (0.064, 0.07, 0.0))),
                SOFT,
            );
        }
        clay
    }

    /// The hollow behind a gill cover, on `side`, that the gills lie in.
    fn gill_hollow(side: f32) -> Lump {
        Lump::limb(
            sided(side, (0.074, 0.04, 0.118)),
            sided(side, (0.078, -0.06, 0.112)),
            (0.02, 0.022),
        )
    }

    /// The trunk: scaled and painted.
    fn trunk() -> MeshPrimitive {
        let hollows = Clay::new(0.0)
            .add(Self::gill_hollow(1.0), RAW)
            .add(Self::gill_hollow(-1.0), RAW);
        let clay = Self::trunk_form()
            .cut(Self::gill_hollow(1.0), 0.006)
            .cut(Self::gill_hollow(-1.0), 0.006)
            .relief(Relief::Shingles {
                origin: Vector3::new(0.0, -0.01, 0.0),
                toward: -Vector3::z(),
                around: 22,
                row: 0.017,
                depth: 0.0036,
            });
        let mut mesh = clay.skin(TRUNK_GRAIN, Paint::Wet.id());
        TRUNK_HIDE.paint(&mut mesh);
        Crevices {
            depth: 0.6,
            lift: 0.25,
        }
        .paint(&mut mesh);
        // Chafed raw where the wings work.
        for side in [-1.0, 1.0] {
            for along in [-0.1, 0.0, 0.1] {
                Blush {
                    at: point(sided(side, WING_ROOT)) + Vector3::z() * along,
                    radius: 0.07,
                    tint: Vector3::new(1.15, 0.5, 0.46),
                }
                .paint(&mut mesh);
            }
        }
        flay(&mut mesh, |at, _| {
            (hollows.distance(at) < 0.003).then_some(0.3)
        });
        mesh
    }

    /// The bare mass of the head: a wedge, deep at the gills and run out
    /// to a snout.
    fn skull() -> Clay {
        Clay::new(0.03)
            .add(Lump::mass((0.066, 0.092, 0.115), (0.0, 0.0, 0.2)), SKULL)
            .add(
                Lump::mass((0.038, 0.044, 0.105), (0.0, -0.006, 0.335)),
                SKULL,
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

    /// Where the middle of the eye on `side` is.
    fn eye_socket(side: f32) -> Vector3<f32> {
        Self::on_skull(side, EYE_AT.0, EYE_AT.1) - Vector3::x() * (side * EYE_SUNK * EYE_RADIUS)
    }

    /// The head, whole: before the mouth and the sockets are cut.
    fn head_form() -> Clay {
        let mut clay = Self::skull().smoothing(0.011);
        for side in [-1.0, 1.0] {
            // The bone the mouth is edged with above.
            clay = strap(clay, |t| upper_edge(side, t), (0.0085, 0.006));
            // A gill cover: a hard plate, stood open at the back, ribbed
            // from its hinge.
            let cover = Ellipsoid::new((0.009, 0.088, 0.062), sided(side, (0.064, -0.008, 0.165)))
                .yawed(-0.5 * side);
            clay = clay.add(Lump::Ball(cover), SKULL);
            let hinge = cover.surface_at(side * (FRAC_PI_2 - 0.8), 0.15).0;
            for rib in [0.7, 0.25, -0.2, -0.65] {
                clay = clay.add(
                    Lump::Limb {
                        from: hinge,
                        to: cover.surface_at(side * (FRAC_PI_2 + 0.95), rib).0,
                        radii: (0.0045, 0.003),
                    },
                    SKULL,
                );
            }
            // The ridge of bone before it, from behind the eye down to
            // the corner of the mouth.
            clay = clay.add(
                Lump::Limb {
                    from: Self::on_skull(side, 0.055, 0.215),
                    to: Self::on_skull(side, -0.06, 0.228),
                    radii: (0.007, 0.009),
                },
                SKULL,
            );
            // The ring of bone the eye has sunk back into.
            let beads = 12;
            for bead in 0..beads {
                let round = bead as f32 / beads as f32 * TAU;
                let reach = EYE_RADIUS + 0.005;
                let at = Self::on_skull(
                    side,
                    EYE_AT.0 + reach * round.sin(),
                    EYE_AT.1 + reach * round.cos(),
                );
                clay = clay.add(
                    Lump::ball(0.0045 + 0.001 * scatter(bead, 21), (at.x, at.y, at.z)),
                    SKULL,
                );
            }
        }
        clay
    }

    /// What is cut out of the head for its mouth: the vault under the
    /// upper jaw.
    fn vault() -> Lump {
        Lump::limb((0.0, -0.085, 0.275), (0.0, -0.052, 0.425), (0.045, 0.03))
    }

    /// What is cut out of the lower jaw: the trough of the mouth's floor.
    fn trough() -> Lump {
        Lump::Limb {
            from: hung(Vector3::new(0.0, -0.05, 0.285)),
            to: hung(Vector3::new(0.0, -0.03, 0.395)),
            radii: (0.02, 0.007),
        }
    }

    /// The skin of the head, on `clay`: dried on the bone, pitted, and
    /// painted as its plates are.
    fn head_skin(clay: Clay) -> MeshPrimitive {
        let clay = clay
            .relief(Relief::Lumps {
                frequency: 26.0,
                depth: 0.0045,
                seed: 35,
            })
            .relief(Relief::Lumps {
                frequency: 80.0,
                depth: 0.0012,
                seed: 36,
            });
        // Wet, the crown would only mirror the sky.
        let mut mesh = clay.skin(HEAD_GRAIN, Paint::Skin.id());
        HEAD_HIDE.paint(&mut mesh);
        Crevices {
            depth: 0.55,
            lift: 0.2,
        }
        .paint(&mut mesh);
        mesh
    }

    /// The lower jaw, hung open: its two bones, the floor between them and
    /// the knuckle it hangs from.
    fn jaw() -> MeshPrimitive {
        let mut clay = Clay::new(0.02)
            .add(
                Lump::Limb {
                    from: hung(Vector3::new(0.0, -0.08, 0.265)),
                    to: hung(Vector3::new(0.0, -0.048, 0.4)),
                    radii: (0.03, 0.012),
                },
                SKULL,
            )
            .add(Lump::ball(0.03, (0.0, -0.068, 0.25)), SKULL)
            .smoothing(0.011);
        for side in [-1.0, 1.0] {
            clay = strap(clay, |t| hung(lower_edge(side, t)), (0.012, 0.008));
        }
        let mut mesh = Self::head_skin(clay.cut(Self::trough(), 0.005));
        // Dark wherever it is inside the mouth: its floor, and what of it
        // lies up under the upper jaw.
        let (trough, vault) = (
            Clay::new(0.0).add(Self::trough(), RAW),
            Clay::new(0.0).add(Self::vault(), RAW),
        );
        flay(&mut mesh, |at, normal| {
            let floor = trough.distance(at) < 0.003;
            let within = vault.distance(at) < -0.002 && normal.y > 0.2;
            (floor || within).then(|| 0.45 + (0.43 - at.z) / 0.2)
        });
        mesh
    }

    /// The head: plated, dark under the upper jaw, the eyes' sockets cut.
    fn head() -> MeshPrimitive {
        let mut clay = Self::head_form().cut(Self::vault(), 0.005);
        let vault = Clay::new(0.0).add(Self::vault(), RAW);
        for side in [-1.0, 1.0] {
            let socket = Self::eye_socket(side);
            clay = clay
                // Wider than the eye: it has shrunk in its socket.
                .cut(
                    Lump::ball(EYE_RADIUS + 0.0035, (socket.x, socket.y, socket.z)),
                    0.003,
                )
                // A nostril.
                .cut(Lump::ball(0.0045, sided(side, (0.02, 0.022, 0.392))), 0.002);
        }
        let mut mesh = Self::head_skin(clay);
        for side in [-1.0, 1.0] {
            // Blood at the edge of the gill cover and the corner of the
            // mouth, and the gums.
            for (at, radius) in [
                (Vector3::new(side * 0.09, -0.02, 0.112), 0.05),
                (upper_edge(side, 0.0), 0.03),
            ] {
                Blush {
                    at,
                    radius,
                    tint: Vector3::new(0.9, 0.36, 0.33),
                }
                .paint(&mut mesh);
            }
            for step in 0..8 {
                let t = 0.1 + 0.12 * step as f32;
                Blush {
                    at: upper_edge(side, t),
                    radius: 0.011,
                    tint: Vector3::new(0.95, 0.5, 0.46),
                }
                .paint(&mut mesh);
            }
            // The socket round the shrunken eye is dark.
            let socket = Self::eye_socket(side);
            flay(&mut mesh, |at, _| {
                let from_eye = (at - socket).norm() - EYE_RADIUS;
                (from_eye < 0.006).then_some(0.75)
            });
        }
        // The mouth is dark all the way in.
        flay(&mut mesh, |at, _| {
            (vault.distance(at) < 0.003).then(|| 0.45 + (0.43 - at.z) / 0.2)
        });
        mesh
    }

    /// The gills in the hollow behind one gill cover: a raw comb.
    fn gills(side: f32) -> MeshPrimitive {
        let mut mesh = Clay::new(0.004)
            .add(
                Lump::mass((0.012, 0.056, 0.018), sided(side, (0.066, -0.01, 0.118))),
                RAW,
            )
            .relief(Relief::Folds {
                along: Vector3::z(),
                frequency: 130.0,
                depth: 0.0024,
                seed: 37,
            })
            .skin(0.003, Paint::Wet.id());
        Crevices {
            depth: 0.7,
            lift: 0.5,
        }
        .paint(&mut mesh);
        mesh
    }

    /// Its teeth: small, close-set and many, a few at the front grown
    /// long.
    fn teeth() -> MeshPrimitive {
        let mut clay = Clay::new(0.0008);
        for tooth in 0..TEETH {
            let t = 0.12 + 0.86 * tooth as f32 / (TEETH - 1) as f32;
            for side in [-1.0, 1.0] {
                let key = tooth as u32 * 2 + u32::from(side > 0.0);
                let long = |salt: u32| {
                    let front = if t > 0.86 { 0.006 } else { 0.0 };
                    (0.0065 + front) * (1.0 + 0.3 * scatter(key, salt))
                };
                // Set a little inside the bone, and leaning in.
                let root = upper_edge(side, t) + Vector3::new(-side * 0.004, -0.004, 0.0);
                let way = Vector3::new(-side * 0.12 + 0.15 * scatter(key, 22), -1.0, 0.1);
                clay = clay.add(
                    Lump::Limb {
                        from: root,
                        to: root + way.normalize() * long(24),
                        radii: (0.0025, 0.001),
                    },
                    TOOTH,
                );
                let root = lower_edge(side, t) + Vector3::new(-side * 0.004, 0.004, 0.0);
                let way = Vector3::new(-side * 0.12 + 0.15 * scatter(key, 25), 1.0, 0.1);
                clay = clay.add(
                    Lump::Limb {
                        from: hung(root),
                        to: hung(root + way.normalize() * long(27)),
                        radii: (0.0025, 0.001),
                    },
                    TOOTH,
                );
            }
        }
        let mut mesh = clay.skin(TOOTH_GRAIN, Paint::Horn.id());
        Mottle {
            frequency: 90.0,
            value: 0.25,
            warmth: 0.2,
            seed: 28,
        }
        .paint(&mut mesh);
        mesh
    }

    /// Its eyes: flat in the sides of its head, lidless, filmed over, and
    /// dried dull: a wet eye is a mirror for the sky, and shows no pupil.
    fn eyes() -> Vec<MeshPrimitive> {
        [-1.0, 1.0]
            .into_iter()
            .map(|side| {
                let mut eye = glass_eye(
                    EYE_RADIUS,
                    GlassLook {
                        iris: (0.16, 0.13, 0.06),
                        iris_angle: 1.5,
                        pupil_share: 0.42,
                        bloodshot: 0.9,
                        film: 0.14,
                    },
                );
                eye.material = Paint::Horn.id();
                let stare = Vector3::new(side, -0.05, 0.12);
                moved(
                    eye,
                    UnitQuaternion::face_towards(&stare, &Vector3::y()),
                    Self::eye_socket(side),
                )
            })
            .collect()
    }

    /// What has run out of it: blood from under each gill cover, down the
    /// flank, and dried there.
    fn leaks() -> MeshPrimitive {
        let trunk = Self::trunk_form();
        let mut clay = Clay::new(0.004);
        for (side, runs) in [(1.0, 7), (-1.0, 4)] {
            let on_flank = |run: usize| {
                let wander = 0.006 * (run as f32 * 1.9 + side).sin();
                let within = Vector3::new(
                    0.0,
                    -0.045 - 0.02 * run as f32,
                    0.106 - 0.005 * run as f32 + wander,
                );
                surface_along(&trunk, within, Vector3::x() * side) + Vector3::x() * (side * 0.0015)
            };
            for run in 0..runs {
                let thick = 0.0048 - 0.0004 * run as f32;
                clay = clay.add(
                    Lump::Limb {
                        from: on_flank(run),
                        to: on_flank(run + 1),
                        radii: (thick, thick - 0.0004),
                    },
                    GORE,
                );
            }
            // A bead where it stopped.
            let end = on_flank(runs);
            clay = clay.add(Lump::ball(0.0052, (end.x, end.y - 0.002, end.z)), GORE);
        }
        clay.skin(0.0024, Paint::Wet.id())
    }

    /// A thread of slime let down from the tip of the slack jaw, blown
    /// back.
    fn drool() -> MeshPrimitive {
        let tip = hung(lower_edge(1.0, 1.0)) - Vector3::y() * 0.008;
        let thread = [
            (0.0, 0.0, 0.0),
            (0.003, -0.022, -0.005),
            (0.0, -0.046, -0.014),
            (0.004, -0.066, -0.026),
        ];
        let mut clay = Clay::new(0.001);
        for pair in thread.windows(2) {
            clay = clay.add(
                Lump::Limb {
                    from: tip + point(pair[0]),
                    to: tip + point(pair[1]),
                    radii: (0.0013, 0.0011),
                },
                SLIME,
            );
        }
        let bead = tip + point(thread[3]);
        clay.add(Lump::ball(0.0028, (bead.x, bead.y, bead.z)), SLIME)
            .skin(0.0009, Paint::Wet.id())
    }

    /// Its fins: a crest of spines down the back, a tail and a pair of
    /// belly fins in ribbons, and what is left of the fin under the tail.
    fn fins() -> Vec<MeshPrimitive> {
        let trunk = Self::trunk_form();
        let paint = FinPaint {
            ray: (0.34, 0.3, 0.24),
            web: (0.22, 0.12, 0.1),
            edge: (0.5, 0.3, 0.22),
        };
        let top = |z: f32| surface_along(&trunk, Vector3::new(0.0, 0.0, z), Vector3::y());
        let under = |x: f32, z: f32| surface_along(&trunk, Vector3::new(x, 0.0, z), -Vector3::y());
        let share = |ray: usize, of: usize| ray as f32 / (of - 1) as f32;

        // The crest: more spine than web.
        let crest = (0..11)
            .map(|ray| {
                let along = share(ray, 11);
                let root = top(0.13 - 0.36 * along) - Vector3::y() * 0.01;
                let tall = 0.07 + 0.07 * (along * 3.0).min(1.0) - 0.07 * along
                    + 0.015 * scatter(ray as u32, 41);
                let lean = 0.02 * scatter(ray as u32, 42);
                Ray::new(root, root + Vector3::new(lean, 0.85 * tall, -0.6 * tall))
                    .bowed(Vector3::new(0.0, 0.0, -0.12 * tall))
            })
            .collect();
        let crest = Fin::on(crest, paint, 43).edged(0.45, 0.25).torn(&[
            (1.5, 0.9, 0.7),
            (3.5, 0.8, 0.85),
            (5.5, 1.0, 0.6),
            (7.5, 0.8, 0.9),
        ]);

        // The tail: the lower lobe drawn out, and trailing in ribbons.
        let tail = (0..13)
            .map(|ray| {
                let across = share(ray, 13) * 2.0 - 1.0;
                let root = Vector3::new(0.0, 0.015 + 0.02 * across, -0.425);
                let spread = 0.8 * across - 0.12;
                let long = if across < 0.0 {
                    0.12 + 0.2 * across.abs().powf(1.2)
                } else {
                    0.1 + 0.11 * across.powf(1.4)
                };
                let sway = 0.035 * (across * 2.6 + 0.3).sin();
                Ray::new(
                    root,
                    root + Vector3::new(sway, spread.sin(), -spread.cos()) * long,
                )
                .bowed(Vector3::new(0.4 * sway, -0.02 * long, 0.0))
            })
            .collect();
        let tail = Fin::on(tail, paint, 44).edged(0.14, 0.22).torn(&[
            (0.8, 0.8, 0.5),
            (2.5, 0.7, 0.8),
            (4.4, 0.6, 0.65),
            (8.5, 0.7, 0.75),
            (10.6, 0.6, 0.45),
        ]);

        // A long fin each side of the belly, trailing.
        let trailing = |side: f32| {
            (0..6)
                .map(|ray| {
                    let across = share(ray, 6);
                    let root = under(side * 0.03, 0.03 - 0.05 * across) + Vector3::y() * 0.008;
                    let long = 0.3 - 0.13 * across;
                    let way =
                        Vector3::new(side * (0.12 + 0.1 * across), -0.42 + 0.2 * across, -0.9);
                    Ray::new(root, root + way.normalize() * long).bowed(Vector3::new(
                        side * 0.02,
                        -0.035,
                        0.0,
                    ))
                })
                .collect()
        };
        let right = Fin::on(trailing(1.0), paint, 45)
            .edged(0.2, 0.2)
            .torn(&[(1.5, 0.8, 0.75), (3.5, 0.9, 0.85)]);
        let left = Fin::on(trailing(-1.0), paint, 46).edged(0.2, 0.2).torn(&[
            (0.6, 0.7, 0.6),
            (2.5, 0.8, 0.9),
            (4.4, 0.7, 0.7),
        ]);

        // The fin under the tail.
        let anal = (0..6)
            .map(|ray| {
                let along = share(ray, 6);
                let root = under(0.0, -0.2 - 0.1 * along) + Vector3::y() * 0.008;
                let long = 0.075 - 0.035 * along;
                Ray::new(root, root + Vector3::new(0.0, -0.75 * long, -0.65 * long))
            })
            .collect();
        let anal = Fin::on(anal, paint, 47).torn(&[(2.5, 0.9, 0.7)]);

        let mut fins = vec![
            crest.mesh(6, 16, Paint::Horn.id()),
            tail.mesh(6, 26, Paint::Horn.id()),
            right.mesh(7, 30, Paint::Horn.id()),
            left.mesh(7, 30, Paint::Horn.id()),
            anal.mesh(6, 12, Paint::Horn.id()),
        ];
        for fin in &mut fins {
            Mottle {
                frequency: 24.0,
                value: 0.3,
                warmth: 0.25,
                seed: 48,
            }
            .paint(fin);
        }
        fins
    }

    /// The rays of the wing on `side`, where it is at rest: fanned from
    /// the shoulder, the first of them the arm at its leading edge and the
    /// last lying back along the flank.
    fn wing_rays(side: f32) -> Vec<Ray> {
        (0..WING_RAYS)
            .map(|ray| {
                let across = ray as f32 / (WING_RAYS - 1) as f32;
                let root = Vector3::new(
                    side * (WING_ROOT.0 + 0.006 - 0.02 * across * across),
                    WING_ROOT.1,
                    0.13 - 0.26 * across,
                );
                // Swept round from just ahead of square to nearly along
                // the body, and shorter as they go.
                let sweep = -0.14 + 1.5 * across.powf(1.15);
                let long = 0.66 - 0.36 * across.powf(1.6) + 0.03 * (across * 9.0).sin();
                let droop = -0.1 - 0.12 * across;
                let way = Vector3::new(side * sweep.cos(), droop, -sweep.sin());
                // Each bowed up over the air under it, and back.
                Ray::new(root, root + way * long).bowed(Vector3::new(
                    0.0,
                    0.05 * long,
                    -0.06 * long * (1.0 - across),
                ))
            })
            .collect()
    }

    /// The wing on `side`: its web, and the bones of its fingers.
    fn wing(side: f32) -> Vec<MeshPrimitive> {
        let rays = Self::wing_rays(side);
        let paint = FinPaint {
            ray: BONE,
            web: (0.3, 0.12, 0.09),
            edge: (0.56, 0.36, 0.24),
        };
        // The right wing's web is torn where the left's is whole: neither
        // has had the better of it.
        let (tears, holes): (&[(f32, f32, f32)], &[((f32, f32), (f32, f32))]) = if side > 0.0 {
            (
                &[(2.5, 0.5, 0.6), (3.6, 0.4, 0.3), (4.4, 0.7, 0.75)],
                &[((1.5, 0.62), (0.2, 0.07)), ((3.4, 0.4), (0.16, 0.05))],
            )
        } else {
            (
                &[(1.5, 0.5, 0.4), (3.5, 0.8, 0.8)],
                &[
                    ((0.5, 0.7), (0.22, 0.06)),
                    ((2.5, 0.5), (0.24, 0.09)),
                    ((4.4, 0.3), (0.15, 0.06)),
                ],
            )
        };
        // A wing's front is under it on the right and over it on the
        // left: either way the web bellies up, full of air.
        let mut web = Fin::on(rays.clone(), paint, if side > 0.0 { 81 } else { 82 })
            .thick(0.006, 0.0012)
            .edged(0.16, 0.08)
            .bellied(-side * 0.022)
            .torn(tears)
            .holed_at(holes)
            .mesh(14, 48, Paint::Skin.id());
        Mottle {
            frequency: 9.0,
            value: 0.3,
            warmth: 0.3,
            seed: 83,
        }
        .paint(&mut web);
        Veins {
            frequency: 7.0,
            width: 0.05,
            tint: Vector3::new(0.5, 0.2, 0.18),
            seed: 84,
        }
        .paint(&mut web);
        Veins {
            frequency: 19.0,
            width: 0.035,
            tint: Vector3::new(0.66, 0.36, 0.32),
            seed: 85,
        }
        .paint(&mut web);
        Spots {
            frequency: 26.0,
            presence: 0.3,
            radius: (0.15, 0.42),
            tint: Vector3::new(0.55, 0.45, 0.4),
            salt: 86,
        }
        .paint(&mut web);

        // Bone: the arm along the leading edge, whole, and a knuckle or
        // two on every finger behind it.
        let mut bones = Clay::new(0.006);
        let arm = &rays[0];
        let joints = 6;
        for joint in 0..joints {
            let (from, to) = (
                joint as f32 / joints as f32,
                (joint + 1) as f32 / joints as f32,
            );
            let thick = |v: f32| 0.0125 - 0.008 * v;
            bones = bones.add(
                Lump::Limb {
                    from: arm.at(from),
                    to: arm.at(to),
                    radii: (thick(from), thick(to)),
                },
                BONE,
            );
        }
        for (finger, ray) in rays.iter().enumerate() {
            let knuckles: &[f32] = if finger == 0 {
                &[0.0, 0.34, 0.68]
            } else {
                &[0.3, 0.62]
            };
            for (which, &along) in knuckles.iter().enumerate() {
                let key = (finger * 4 + which) as u32;
                let along = along + 0.05 * scatter(key, 87);
                let swell = if finger == 0 { 0.017 } else { 0.0085 } * (1.0 - 0.35 * along);
                bones = bones.add(
                    Lump::Limb {
                        from: ray.at(along - 0.012),
                        to: ray.at(along + 0.012),
                        radii: (swell, 0.9 * swell),
                    },
                    BONE,
                );
            }
        }
        // A claw at the bend of the arm, where a thumb would be.
        let wrist = arm.at(0.34);
        bones = bones.add(
            Lump::Limb {
                from: wrist,
                to: wrist + Vector3::new(side * 0.01, 0.012, 0.05),
                radii: (0.007, 0.0025),
            },
            TOOTH,
        );
        let mut bones = bones
            .relief(Relief::Lumps {
                frequency: 60.0,
                depth: 0.0012,
                seed: 88,
            })
            .skin(BONE_GRAIN, Paint::Skin.id());
        Mottle {
            frequency: 30.0,
            value: 0.25,
            warmth: 0.2,
            seed: 89,
        }
        .paint(&mut bones);
        Crevices {
            depth: 0.4,
            lift: 0.2,
        }
        .paint(&mut bones);
        vec![web, bones]
    }

    /// How it looks, against the palette's slots: its body, then a wing
    /// each side, each wing where it is at rest. Nose along +Z.
    pub fn sculpt() -> Model {
        let mut body = vec![
            Self::trunk(),
            Self::head(),
            Self::jaw(),
            Self::teeth(),
            Self::gills(1.0),
            Self::gills(-1.0),
            Self::leaks(),
            Self::drool(),
        ];
        body.extend(Self::eyes());
        body.extend(Self::fins());
        let mut parts = vec![ModelPart::new(Vec::new()); 3];
        parts[BODY] = ModelPart::new(body);
        parts[RIGHT_WING] = ModelPart::new(Self::wing(1.0));
        parts[LEFT_WING] = ModelPart::new(Self::wing(-1.0));
        Model::flat(parts)
    }

    /// How its wings beat: each on a hinge along the body at its root.
    fn wing_beat() -> WingBeat {
        let wing = |part: usize, side: f32| Wing {
            part,
            root: point(sided(side, WING_ROOT)),
            hinge: Vector3::z(),
            span: Vector3::x() * side,
        };
        WingBeat {
            wings: [wing(RIGHT_WING, 1.0), wing(LEFT_WING, -1.0)],
            raise: WING_RAISE,
            lower: WING_LOWER,
            feather: WING_FEATHER,
            glide_rate: GLIDE_BEATS,
            hard_rate: HARD_BEATS,
        }
    }

    /// The states it is shown in by tools: its wings at the top of a beat,
    /// at the bottom, and half way down between them.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        let beat = Self::wing_beat();
        [("wings_up", 0.0), ("wings_down", 0.5), ("mid_beat", 0.25)]
            .into_iter()
            .map(|(name, phase)| Exhibit {
                name,
                poses: beat.poses(phase, model.parts.len()),
                model: model.clone(),
                shaped: Vec::new(),
                lift: 1.3,
                reach: 0.82,
            })
            .collect()
    }
}

impl Spawnable for FlyingFishDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let Some(ground) = ground_under(world, self.pos) else {
            return Vec::new();
        };
        // Perched: it takes off when it is first flown.
        let at = Point3::new(self.pos.0, ground + RADIUS, self.pos.1);
        let (body, mass) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics
                .world
                .create_body(RigidBodyDesc::dynamic().position(at).angular_damping(0.9));
            physics.world.attach_collider(
                body,
                ColliderDesc::sphere(RADIUS)
                    .density(DENSITY)
                    .restitution(0.0),
            );
            let mass = physics.world.body(body).map_or(1.0, |body| body.mass());
            (body, mass)
        };
        let model: Arc<Model> = likeness::model(KIND, materials, Self::sculpt);
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(PosedModel::at_rest(model, PoseFrame::Entity))
            .with(Renderable)
            .with(CharacterIntent::default())
            .with(all_round(SIGHT))
            .with(Brain::hunter(0.5 * REACH))
            .with(Flyer {
                mass,
                yaw_inertia: 0.4 * mass * RADIUS * RADIUS,
                speed: SPEED,
                accel_time: ACCEL_TIME,
                cruise_height: CRUISE_HEIGHT,
                stoop_range: STOOP_RANGE,
                stoop_height: 0.4,
                wings: Self::wing_beat(),
                beat_phase: 0.0,
            })
            .with(
                MeleeAttack::new(BITE, REACH)
                    .with_timing(BITE_WINDUP, BITE_RECOVERY)
                    .with_swing(Swing {
                        from: Vector2::zeros(),
                        to: Vector2::new(BITE_LENGTH, 0.0),
                        radius: RADIUS,
                        impulse: BITE_BLOW,
                    }),
            )
            .with(Health::new(HEALTH, 5.0))
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    fn triangles(part: &ModelPart) -> usize {
        let meshes = part.primitives.iter();
        meshes.map(|mesh| mesh.indices.len() / 3).sum()
    }

    #[test]
    fn it_is_tens_of_thousands_of_triangles_with_a_wing_of_its_own_each_side() {
        let started = Instant::now();
        let model = FlyingFishDef::sculpt();
        println!("a flying fish is sculpted in {:?}", started.elapsed());
        assert_eq!(model.parts.len(), 3);
        let total: usize = model.parts.iter().map(triangles).sum();
        println!("a flying fish is {total} triangles");
        assert!((30_000..200_000).contains(&total), "{total} triangles");

        // Each wing lies on its own side of the body and reaches well out
        // from it; the body keeps to the middle.
        for (part, side) in [(RIGHT_WING, 1.0), (LEFT_WING, -1.0)] {
            let reach = model.parts[part]
                .primitives
                .iter()
                .flat_map(|mesh| &mesh.vertices)
                .map(|vertex| vertex.pos.x * side)
                .fold((f32::MAX, f32::MIN), |(least, most), x| {
                    (least.min(x), most.max(x))
                });
            assert!(reach.0 > 0.0 && reach.1 > 0.6, "a wing spans {reach:?}");
        }
        let widest = model.parts[BODY]
            .primitives
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .map(|vertex| vertex.pos.x.abs())
            .fold(0.0, f32::max);
        assert!(widest < 0.2, "the body is {widest} m to a side");
    }

    #[test]
    fn its_wings_are_hinged_where_they_are_rooted_and_shown_up_and_down() {
        let beat = FlyingFishDef::wing_beat();
        for (wing, side) in beat.wings.iter().zip([1.0, -1.0]) {
            // Every ray leaves the body hard by the hinge, so a beat does
            // not tear the wing off its root.
            for ray in FlyingFishDef::wing_rays(side) {
                let from_hinge = ray.root - wing.root;
                let off = from_hinge - wing.hinge * from_hinge.dot(&wing.hinge);
                assert!(off.norm() < 0.03, "a ray rooted {} m off", off.norm());
            }
        }
        let exhibits = FlyingFishDef::exhibits();
        let tip = |name: &str| {
            let exhibit = exhibits.iter().find(|e| e.name == name).unwrap();
            let pose = &exhibit.poses[RIGHT_WING];
            let at_rest = FlyingFishDef::wing_rays(1.0)[0].tip;
            (pose.rotation * at_rest + pose.translation).y
        };
        assert!(tip("wings_up") > tip("mid_beat") + 0.2);
        assert!(tip("wings_down") < tip("mid_beat") - 0.2);
        // Nothing moves but the wings.
        for exhibit in &exhibits {
            assert!(exhibit.poses[BODY].rotation.angle() < 1e-6);
        }
    }
}
