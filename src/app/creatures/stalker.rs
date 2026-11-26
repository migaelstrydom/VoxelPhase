//! Stalker — a tall bird-headed figure in a funnel hat, on stilts.
//!
//! The ground chaser. It is slow and it looms: it sees over what a roller
//! cannot, comes on at a walk, and its long beak reaches well past its body.
//! It can be killed, but it takes a good deal of killing; the quicker way is
//! to take the ground out from under its legs. `visual_bench stalker` shows
//! it.
//!
//! ```text
//!   hat      a tin funnel, dented and rusting, jammed on askew      (Head)
//!   head     a bare scabbed skull, a stork's beak stained at the tip,
//!            and a man's two pale eyes set close above it           (Head, Eye)
//!   lids     four, heavy; shut when it dozes                        (Hinge)
//!   jaw      the lower half of the beak and the tongue in it: it
//!            gapes as the head rears back and snaps shut on the blow (Hinge)
//!   neck     plucked, in rolls, with a crop hanging from it         (Neck)
//!   robe     heavy cloth in folds from a hunched back to a torn hem,
//!            a cowl over the shoulders, and out of one sleeve a
//!            man's hand telling beads                               (Hips)
//!   drape    the back of the robe, below the hips: pushed out over
//!            each hock that would come through it                   (Drape)
//!   legs     a wader's, dark and heavy                              (the bones)
//! ```

use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::anatomy::{self, EyeLook, DIGIT_GRAIN};
use super::bird_anatomy::{
    bristles, cord, placed, plucked, shade_relief, surface_inward, surface_toward, BirdLeg, Bristle,
};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{Peck, StiltWalker};
use crate::animation::stilts::{Carriage, Drape, Dress, Drive, Joint, StiltRigConfig, Worn};
use crate::animation::{FootPlacerConfig, FootSide};
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::harm::HarmConfig;
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::utils::noise::fbm_perlin_3d;

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
/// The impulse its peck drives into what it strikes, N·s: about 7.5 m/s on
/// the player, the hardest blow of any creature, which knocks them down
/// for about four and a half seconds.
const PECK_BLOW: f32 = 420.0;

/// Radius of each eye, m; where the left one sits in the head, the right
/// its mirror; and how far either can turn off the way the head faces,
/// radians.
const EYE_RADIUS: f32 = 0.014;
const EYE_SOCKET: Vector3<f32> = Vector3::new(0.05, 0.022, 0.05);
const EYE_SWIVEL: f32 = 0.6;
/// Radius of the lids' skin over an eye, m.
const LID_RADIUS: f32 = 1.13 * EYE_RADIUS;

/// The beak: how long it is, m; where its root is in the head, on the seam
/// between its halves; and how far its tip hangs below the way the head
/// faces, radians.
const BEAK_LENGTH: f32 = 0.56;
const BEAK_ROOT: Vector3<f32> = Vector3::new(0.0, -0.014, 0.085);
const BEAK_DROOP: f32 = 0.09;
/// Half the beak's width at its root, m, and how fast that narrows toward
/// the tip, m to the metre.
const BEAK_HALF_WIDTH: f32 = 0.034;
const BEAK_TAPER: f32 = 0.052;
/// Where the jaw hinges, along the beak from its root, m, and how far it
/// drops open as the head rears back, radians.
const JAW_HINGE: Vector3<f32> = Vector3::new(0.0, -0.006, -0.03);
const JAW_OPEN: f32 = 0.42;
/// Side of the grid the beak is skinned from, m.
const BEAK_GRAIN: f32 = 0.0052;

/// Horn of the beak at its root, along its middle and at its tip (linear).
const HORN_ROOT: Vector3<f32> = Vector3::new(0.3, 0.2, 0.085);
const HORN: Vector3<f32> = Vector3::new(0.21, 0.18, 0.125);
const HORN_TIP: Vector3<f32> = Vector3::new(0.045, 0.035, 0.03);
/// What has dried on the end of it.
const GORE: Vector3<f32> = Vector3::new(0.17, 0.03, 0.025);
/// The inside of its mouth, and its tongue (linear).
const MOUTH: Vector3<f32> = Vector3::new(0.1, 0.022, 0.022);
const TONGUE: (f32, f32, f32) = (0.25, 0.075, 0.075);
/// Bare skin of the head and the neck, the crop that hangs from the neck,
/// and the lids (linear).
const BARE_SKIN: (f32, f32, f32) = (0.5, 0.31, 0.26);
const CROP: (f32, f32, f32) = (0.5, 0.24, 0.23);
const LID_SKIN: (f32, f32, f32) = (0.4, 0.25, 0.23);
/// The wisps of down left on its nape (linear).
const DOWN: (f32, f32, f32) = (0.62, 0.6, 0.54);
/// Tin (linear), and the rust that multiplies it.
const TIN: (f32, f32, f32) = (0.34, 0.33, 0.3);
const RUST: Vector3<f32> = Vector3::new(0.42, 0.2, 0.09);
/// Cloth of the robe and of the cowl over it (linear).
const ROBE: (f32, f32, f32) = (0.036, 0.031, 0.03);
const COWL: (f32, f32, f32) = (0.17, 0.042, 0.034);
/// The beads it tells, and the key hung on them (linear).
const BEAD: (f32, f32, f32) = (0.14, 0.07, 0.035);
const KEY: (f32, f32, f32) = (0.42, 0.4, 0.37);

/// How thick the robe's cloth is, m, and the side of the grid it is
/// skinned from.
const ROBE_THICK: f32 = 0.038;
const ROBE_GRAIN: f32 = 0.018;
/// How far the slice that closes the robe's inside from sight is drawn in
/// toward the robe's middle, as a share of its width: enough to lie within
/// the cloth.
const LID_DRAWN_IN: f32 = 0.93;

/// The back of the robe hangs free of the hips from this high in their
/// frame down to the hem, and either side of straight behind as far round
/// as this, radians: the hocks swing back further than any robe falls.
const DRAPE_SEWN: f32 = 0.2;
const DRAPE_ROUND: f32 = 1.2;
/// How far below its seam, m, and round from its sides, radians, it takes
/// the cloth to hang wholly free.
const DRAPE_EASE_DOWN: f32 = 0.15;
const DRAPE_EASE_ROUND: f32 = 0.35;
/// Air kept between a hock and the cloth over it, and how wide the bulge
/// it makes spreads either side, m.
const DRAPE_AIR: f32 = 0.015;
const DRAPE_SPREAD: f32 = 0.05;

/// Its legs: a wader's, dark and heavy.
const LEGS: BirdLeg = BirdLeg {
    girth: 0.075,
    shank: 0.027,
    toe: 0.23,
    skin: (0.42, 0.3, 0.25),
    hide: (0.21, 0.185, 0.15),
    claw: (0.05, 0.045, 0.04),
    grain: 0.007,
    // Its drumsticks are up under the robe.
    drumstick_grain: 0.013,
};

/// 0 at or below `from`, 1 at or above `to`, and smooth between.
fn ease(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A ball of `radius` at `centre`.
fn ball(radius: f32, centre: Vector3<f32>) -> Lump {
    Lump::Ball(Ellipsoid::ball(radius, centre))
}

/// A limb from `from` to `to`.
fn limb(from: Vector3<f32>, to: Vector3<f32>, radii: (f32, f32)) -> Lump {
    Lump::Limb { from, to, radii }
}

/// Where an eye sits in the head: `side` +1 for the left, −1 for the right.
fn socket(side: f32) -> Vector3<f32> {
    Vector3::new(side * EYE_SOCKET.x, EYE_SOCKET.y, EYE_SOCKET.z)
}

/// How the beak, modelled lying along +Z from its root with the seam
/// between its halves level, is set in the head.
fn beak_set() -> (UnitQuaternion<f32>, Vector3<f32>) {
    (
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), BEAK_DROOP),
        BEAK_ROOT,
    )
}

/// One half of the beak, modelled along +Z from its root: `upper` the half
/// above the seam, with its ridge and nostrils, or else the half below.
fn beak_half(upper: bool) -> MeshPrimitive {
    let long = if upper {
        BEAK_LENGTH
    } else {
        0.93 * BEAK_LENGTH
    };
    let up = if upper { 1.0 } else { -1.0 };
    let deep = if upper { 0.05 } else { 0.037 };
    // A cone lying along the beak, cut flat at the seam and down its two
    // sides, which draw together toward the tip.
    let mut horn = Clay::new(0.01)
        .add(
            limb(
                Vector3::new(0.0, up * 0.004, 0.0),
                Vector3::new(0.0, 0.0, long),
                (deep, 0.005),
            ),
            (HORN.x, HORN.y, HORN.z),
        )
        // The ridge down its back, or the keel down its chin.
        .smoothing(0.007)
        .add(
            limb(
                Vector3::new(0.0, up * (deep - 0.006), 0.01),
                Vector3::new(0.0, up * 0.004, 0.97 * long),
                (0.012, 0.0035),
            ),
            (HORN.x, HORN.y, HORN.z),
        )
        .cut(
            Lump::half_space(Vector3::zeros(), Vector3::y() * up),
            0.0015,
        );
    for side in [-1.0, 1.0] {
        horn = horn.cut(
            Lump::half_space(
                Vector3::new(side * BEAK_HALF_WIDTH, 0.0, 0.0),
                Vector3::new(-side, 0.0, -BEAK_TAPER),
            ),
            0.009,
        );
    }
    let flank = |z: f32| BEAK_HALF_WIDTH - BEAK_TAPER * z;
    if upper {
        // Nostrils: a slit each side, near the root.
        for side in [-1.0, 1.0] {
            horn = horn.cut(
                limb(
                    Vector3::new(side * flank(0.07), 0.026, 0.07),
                    Vector3::new(side * flank(0.13), 0.02, 0.13),
                    (0.0055, 0.0035),
                ),
                0.002,
            );
        }
    }
    // Chips out of its cutting edge, as (how far along, which side, how
    // big).
    let chips: &[(f32, f32, f32)] = if upper {
        &[
            (0.19, 1.0, 0.011),
            (0.33, -1.0, 0.015),
            (0.36, -1.0, 0.009),
            (0.47, 1.0, 0.008),
        ]
    } else {
        &[(0.26, 1.0, 0.012), (0.41, -1.0, 0.009)]
    };
    for &(along, side, size) in chips {
        horn = horn.cut(
            ball(
                size,
                Vector3::new(side * (flank(along) + 0.3 * size), -up * 0.2 * size, along),
            ),
            0.001,
        );
    }
    // It grows in layers, and they show as lines along it.
    const GROOVE: f32 = 0.0013;
    let grooved = horn.clone().relief(Relief::Folds {
        along: Vector3::z(),
        frequency: 46.0,
        depth: GROOVE,
        seed: if upper { 301 } else { 302 },
    });
    let mut mesh = grooved.skin(BEAK_GRAIN, Paint::Horn.id());
    for vertex in &mut mesh.vertices {
        // Yellow at the root, bone along its middle, and dark at the tip
        // with what it has been driven into.
        let along = vertex.pos.z / long;
        let stain = fbm_perlin_3d(
            vertex.pos.x * 40.0,
            vertex.pos.y * 40.0,
            vertex.pos.z * 9.0,
            3,
            0.5,
            303,
        );
        // Streaks along it, as horn has that grows from its root.
        let streak = fbm_perlin_3d(
            vertex.pos.x * 75.0,
            vertex.pos.y * 75.0,
            vertex.pos.z * 3.5,
            2,
            0.5,
            308,
        );
        let colour = HORN_ROOT
            .lerp(&HORN, ease(0.02, 0.3, along + 0.1 * stain))
            .lerp(&HORN_TIP, ease(0.45, 0.95, along + 0.22 * stain))
            .lerp(&GORE, 0.7 * ease(0.7, 0.9, along + 0.3 * stain.abs()))
            * (1.0 + 0.3 * streak);
        // Dark along the edge it cuts with.
        let edge = 1.0 - 0.45 * ease(0.009, 0.002, up * vertex.pos.y);
        let colour = colour * edge;
        // The face that meets the other half is the inside of its mouth.
        let within = ease(0.5, 0.85, -up * vertex.normal.y) * ease(0.004, 0.001, up * vertex.pos.y);
        let colour = colour.lerp(&MOUTH, within);
        vertex.color.x = colour.x;
        vertex.color.y = colour.y;
        vertex.color.z = colour.z;
    }
    shade_relief(
        &mut mesh,
        &horn,
        (-GROOVE, GROOVE),
        Vector3::new(0.62, 0.56, 0.5),
        Vector3::new(1.08, 1.06, 1.02),
    );
    Mottle {
        frequency: 16.0,
        value: 0.22,
        warmth: 0.1,
        seed: 304,
    }
    .paint(&mut mesh);
    // Cracks, and the dirt in them.
    Veins {
        frequency: 17.0,
        width: 0.02,
        tint: Vector3::new(0.5, 0.44, 0.4),
        seed: 305,
    }
    .paint(&mut mesh);
    mesh
}

/// The upper half of the beak, the hook at its tip and the roof of its
/// mouth, in the head's frame.
fn upper_beak() -> Vec<MeshPrimitive> {
    let (turn, root) = beak_set();
    let long = BEAK_LENGTH;
    // The tip turns down past the lower half, to a point.
    let mut hook = Clay::new(0.006)
        .add(
            limb(
                Vector3::new(0.0, 0.006, long - 0.05),
                Vector3::new(0.0, -0.004, long + 0.004),
                (0.0085, 0.0055),
            ),
            (HORN_TIP.x, HORN_TIP.y, HORN_TIP.z),
        )
        .add(
            limb(
                Vector3::new(0.0, -0.004, long + 0.004),
                Vector3::new(0.0, -0.026, long + 0.016),
                (0.0055, 0.0012),
            ),
            (HORN_TIP.x, HORN_TIP.y, HORN_TIP.z),
        )
        .skin(0.002, Paint::Horn.id());
    Mottle {
        frequency: 60.0,
        value: 0.25,
        warmth: 0.1,
        seed: 306,
    }
    .paint(&mut hook);
    // The roof of its mouth: wet, and seen only when the beak gapes. Shut,
    // it lies inside the lower half.
    let palate = Clay::new(0.0)
        .add(
            limb(
                Vector3::new(0.0, 0.005, 0.01),
                Vector3::new(0.0, 0.002, 0.46),
                (0.017, 0.0035),
            ),
            (MOUTH.x, MOUTH.y, MOUTH.z),
        )
        .relief(Relief::Lumps {
            frequency: 60.0,
            depth: 0.001,
            seed: 309,
        })
        .skin(0.006, Paint::Wet.id());
    vec![
        placed(beak_half(true), turn, root),
        placed(hook, turn, root),
        placed(palate, turn, root),
    ]
}

/// Where the jaw hinges, in the head's frame.
fn jaw_pivot() -> Vector3<f32> {
    let (turn, root) = beak_set();
    turn * JAW_HINGE + root
}

/// The lower half of the beak and the tongue lying in it, about the jaw's
/// hinge.
fn lower_beak() -> Vec<MeshPrimitive> {
    let (turn, root) = beak_set();
    let about_hinge = root - jaw_pivot();
    let tongue = Clay::new(0.008)
        .add(
            limb(
                Vector3::new(0.0, 0.002, 0.0),
                Vector3::new(0.0, 0.004, 0.16),
                (0.014, 0.009),
            ),
            TONGUE,
        )
        .add(
            limb(
                Vector3::new(0.0, 0.004, 0.16),
                Vector3::new(0.0, 0.003, 0.33),
                (0.009, 0.0025),
            ),
            TONGUE,
        )
        .relief(Relief::Lumps {
            frequency: 70.0,
            depth: 0.0012,
            seed: 307,
        })
        .skin(0.004, Paint::Wet.id());
    vec![
        placed(beak_half(false), turn, about_hinge),
        placed(tongue, turn, about_hinge),
    ]
}

/// The skull: bare, scabbed skin over a small cranium, a mound round each
/// eye under a heavy brow, and the dark of its gullet behind the beak. Its
/// skin, its gullet, then the down left on it. In the head's frame.
fn skull() -> Vec<MeshPrimitive> {
    let mut smooth = Clay::new(0.03)
        .add(
            Lump::mass((0.07, 0.075, 0.09), (0.0, 0.01, -0.03)),
            BARE_SKIN,
        )
        .add(
            Lump::mass((0.066, 0.052, 0.06), (0.0, -0.01, 0.045)),
            BARE_SKIN,
        )
        .add(
            Lump::mass((0.045, 0.04, 0.055), (0.0, -0.05, -0.015)),
            BARE_SKIN,
        )
        // The knob at the back of it.
        .add(ball(0.03, Vector3::new(0.0, 0.032, -0.105)), BARE_SKIN)
        .smoothing(0.014);
    for side in [-1.0f32, 1.0] {
        let at = |x: f32, y: f32, z: f32| Vector3::new(side * x, y, z);
        smooth = smooth
            // The brow over an eye, level and heavy, and the bone under
            // it.
            .add(
                limb(
                    at(0.02, 0.052, 0.066),
                    at(0.082, 0.04, 0.022),
                    (0.0095, 0.008),
                ),
                BARE_SKIN,
            )
            .add(ball(0.015, at(0.056, -0.006, 0.054)), BARE_SKIN)
            // A fold of skin from the corner of the beak back along the jaw.
            .add(
                limb(
                    at(0.036, -0.03, 0.07),
                    at(0.05, -0.05, -0.02),
                    (0.008, 0.006),
                ),
                BARE_SKIN,
            );
    }
    // Wrinkles across its crown and down its nape.
    smooth = smooth.smoothing(0.006);
    for (back, thick) in [(0.0f32, 0.006f32), (-0.035, 0.007), (-0.07, 0.006)] {
        // Round the cranium where it is that far back.
        let girth = (1.0 - ((back + 0.02) / 0.09).powi(2)).sqrt();
        let wrinkle: Vec<_> = (-5..=5)
            .map(|step| {
                let round = step as f32 * 0.22;
                let centre = Vector3::new(0.0, 0.01, back - 0.01);
                let out = Vector3::new(0.069 * round.sin(), 0.074 * round.cos(), 0.0);
                (centre + out * girth, thick)
            })
            .collect();
        smooth = cord(smooth, &wrinkle, BARE_SKIN);
    }
    for side in [-1.0, 1.0] {
        smooth = smooth.cut(ball(1.03 * LID_RADIUS, socket(side)), 0.003);
    }
    const WART: f32 = 0.0018;
    let warty = smooth
        .clone()
        .relief(Relief::Scales {
            size: 0.0105,
            depth: WART,
        })
        .relief(Relief::Lumps {
            frequency: 55.0,
            depth: 0.0016,
            seed: 311,
        });
    let mut skin = plucked(&warty, 0.0045);
    shade_relief(
        &mut skin,
        &smooth,
        (-WART, 0.0),
        Vector3::new(0.6, 0.5, 0.5),
        Vector3::new(1.05, 1.0, 0.96),
    );
    // Scabs, where it has been at itself.
    Spots {
        frequency: 55.0,
        presence: 0.3,
        radius: (0.2, 0.45),
        tint: Vector3::new(0.34, 0.24, 0.22),
        salt: 312,
    }
    .paint(&mut skin);
    // Raw round the root of the beak and round the eyes.
    Blush {
        at: Vector3::new(0.0, -0.01, 0.09),
        radius: 0.085,
        tint: Vector3::new(0.9, 0.5, 0.48),
    }
    .paint(&mut skin);
    for side in [-1.0, 1.0] {
        Blush {
            at: socket(side),
            radius: 0.038,
            tint: Vector3::new(0.5, 0.36, 0.42),
        }
        .paint(&mut skin);
    }

    let gullet = Clay::new(0.0)
        .add(
            ball(0.034, Vector3::new(0.0, -0.016, 0.07)),
            (MOUTH.x, MOUTH.y, MOUTH.z),
        )
        .skin(0.005, Paint::Wet.id());

    // A few wisps of down on its nape, where it cannot reach to pull them.
    let middle = Vector3::new(0.0, 0.0, -0.03);
    let mut wisps = Vec::new();
    for wisp in 0..40u32 {
        let way = Vector3::new(
            0.8 * scatter(wisp, 313),
            -0.3 + 0.9 * (0.5 + 0.5 * scatter(wisp, 314)),
            -1.0,
        );
        let long = 0.02 + 0.035 * (0.5 + 0.5 * scatter(wisp, 315));
        wisps.push(Bristle {
            root: surface_toward(&smooth, middle, way, 0.3) - way.normalize() * 0.002,
            grows: (way.normalize() + Vector3::new(0.0, -0.4, -0.3)).normalize() * long,
            bend: Vector3::new(0.3 * long * scatter(wisp, 316), -0.5 * long, 0.0),
            thick: 0.0009,
        });
    }
    vec![skin, gullet, bristles(&wisps, DOWN, Paint::Feather.id())]
}

/// One lid of one eye, about that eye's middle: a slice of a ball a little
/// larger than the eye that turns about the line through its corners.
/// Angles are about that line, from straight ahead up toward the crown.
struct Lid {
    /// Where its free edge lies with the eye open.
    edge: f32,
    /// Where it ends, out of sight inside the skull.
    back: f32,
    /// How far it turns to shut: positive brings the crown forward.
    shut: f32,
}

/// The upper lid, which hangs over half the iris, and the lower.
const UPPER_LID: Lid = Lid {
    edge: 0.58,
    back: 2.6,
    shut: 0.73,
};
const LOWER_LID: Lid = Lid {
    edge: -0.5,
    back: -2.2,
    shut: -0.37,
};

impl Lid {
    /// The joint it rides, over the eye on `side`.
    fn hinge(&self, side: f32) -> Joint {
        Joint::Hinge {
            pivot: socket(side),
            axis: Vector3::x(),
            drive: Drive::Lid { shut: self.shut },
        }
    }

    /// Its skin.
    fn mesh(&self) -> MeshPrimitive {
        let up = (self.back - self.edge).signum();
        let above = |lift: f32| Vector3::new(0.0, lift.cos(), -lift.sin());
        // A roll of skin along its edge, and a fold above that.
        let mut slice = Clay::new(0.001).add(ball(LID_RADIUS, Vector3::zeros()), LID_SKIN);
        for (over, thick) in [(0.1f32, 0.0014f32), (0.42, 0.0012)] {
            let lift = self.edge + up * over;
            let roll: Vec<_> = (-6..=6)
                .map(|step| {
                    let round = step as f32 * 0.24;
                    let on = Vector3::new(
                        round.sin(),
                        round.cos() * lift.sin(),
                        round.cos() * lift.cos(),
                    );
                    (on * (LID_RADIUS - 0.0006), thick)
                })
                .collect();
            slice = cord(slice, &roll, LID_SKIN);
        }
        let slice = slice
            .cut(
                Lump::half_space(Vector3::zeros(), above(self.edge) * up),
                0.0008,
            )
            .cut(
                Lump::half_space(Vector3::zeros(), -above(self.back) * up),
                0.0,
            );
        let mut skin = anatomy::flesh(&slice, 0.0014);
        // Kept only where it lies clear of the eye, and raw along its
        // edge.
        skin.indices = skin
            .indices
            .chunks_exact(3)
            .filter(|triangle| {
                triangle
                    .iter()
                    .any(|index| skin.vertices[*index as usize].pos.norm() > 0.9 * EYE_RADIUS)
            })
            .flatten()
            .copied()
            .collect();
        for vertex in &mut skin.vertices {
            let out = vertex.pos.normalize();
            let from_edge = (out.y.atan2(out.z) - self.edge) * up;
            let raw = 1.0 - ease(0.0, 0.14, from_edge);
            vertex.color.x *= 1.0 - 0.08 * raw;
            vertex.color.y *= 1.0 - 0.5 * raw;
            vertex.color.z *= 1.0 - 0.48 * raw;
        }
        skin
    }
}

/// The hat's funnel: the radius of its rim, how thick its wall is, and how
/// high its cone runs before the spout begins, m.
const HAT_RIM: f32 = 0.09;
const HAT_WALL: f32 = 0.018;
const HAT_CONE: f32 = 0.17;
/// Side of the grid the hat is skinned from, m: fine enough that its rolled
/// rim and the wall of its spout come out whole.
const HAT_GRAIN: f32 = 0.0045;
/// How fast the funnel's side narrows with height, m to the metre.
const HAT_NARROWING: f32 = (HAT_RIM - 0.022) / (HAT_CONE + 0.02);

/// A dent in the hat: where a ball has been pressed a little way into its
/// side. Broad and shallow: tin is a mirror, and a small deep dent shows
/// only the dark under the hat, like a hole.
struct Dent {
    /// How far round the funnel it is, radians, and how high above the rim,
    /// m.
    round: f32,
    height: f32,
    /// Radius of the ball, and how deep it went, m.
    size: f32,
    deep: f32,
}

const DENTS: [Dent; 4] = [
    Dent {
        round: 0.7,
        height: 0.07,
        size: 0.07,
        deep: 0.0065,
    },
    Dent {
        round: 2.6,
        height: 0.045,
        size: 0.075,
        deep: 0.005,
    },
    Dent {
        round: 4.4,
        height: 0.1,
        size: 0.06,
        deep: 0.006,
    },
    Dent {
        round: 5.5,
        height: 0.05,
        size: 0.085,
        deep: 0.004,
    },
];

impl Dent {
    /// The way out of the funnel's side where it is.
    fn out(&self) -> Vector3<f32> {
        Vector3::new(self.round.sin(), HAT_NARROWING, self.round.cos()).normalize()
    }

    /// The middle of the ball that made it.
    fn ball_at(&self) -> Vector3<f32> {
        let girth = HAT_RIM - HAT_NARROWING * self.height;
        let on = Vector3::new(
            self.round.sin() * girth,
            self.height,
            self.round.cos() * girth,
        );
        on + self.out() * (self.size - self.deep)
    }
}

/// The hat before it is set on the head: a funnel standing on its rim
/// about the Y axis, hollow, its rim rolled and its side dented.
fn funnel() -> Clay {
    let neck_of_it = Vector3::new(0.0, HAT_CONE, 0.0);
    let mut funnel = Clay::new(0.008)
        .add(
            limb(
                Vector3::new(0.0, -0.02, 0.0),
                neck_of_it,
                (HAT_RIM + 0.006, 0.022),
            ),
            TIN,
        )
        .add(
            limb(neck_of_it, Vector3::new(0.004, 0.29, 0.0), (0.022, 0.019)),
            TIN,
        );
    // The rim, rolled over a wire.
    let rim: Vec<_> = (0..=20)
        .map(|step| {
            let round = step as f32 / 20.0 * TAU;
            (
                Vector3::new(
                    round.sin() * (HAT_RIM + 0.002),
                    0.006,
                    round.cos() * (HAT_RIM + 0.002),
                ),
                0.0062,
            )
        })
        .collect();
    funnel = cord(funnel.smoothing(0.004), &rim, TIN);
    funnel = funnel
        .hollow(HAT_WALL)
        .cut(Lump::half_space(Vector3::zeros(), Vector3::y()), 0.0)
        // The bore of the spout.
        .cut(
            limb(
                Vector3::new(0.0, 0.1, 0.0),
                Vector3::new(0.004, 0.4, 0.0),
                (0.009, 0.0085),
            ),
            0.0,
        );
    for dent in &DENTS {
        funnel = funnel.cut(ball(dent.size, dent.ball_at()), 0.006);
    }
    funnel
}

/// The hat: a tin funnel upside down, its spout in the air, dented, its
/// rim rolled, rust running down it; set on the crown askew. In the head's
/// frame.
fn hat() -> MeshPrimitive {
    let beaten = funnel().relief(Relief::Lumps {
        frequency: 13.0,
        depth: 0.002,
        seed: 321,
    });
    let mut mesh = beaten.skin(HAT_GRAIN, Paint::Tin.id());
    for vertex in &mut mesh.vertices {
        // Dull inside, where no weather reaches and nothing rubs.
        let axis_out = Vector3::new(vertex.pos.x, 0.0, vertex.pos.z);
        let inside = vertex.normal.dot(&axis_out) < 0.0 && vertex.pos.y < HAT_CONE;
        if inside {
            vertex.color.x *= 0.25;
            vertex.color.y *= 0.24;
            vertex.color.z *= 0.22;
            continue;
        }
        // Rust in runs down it, thickest low down where the wet sits.
        let run = fbm_perlin_3d(
            vertex.pos.x * 34.0,
            vertex.pos.y * 5.0,
            vertex.pos.z * 34.0,
            3,
            0.55,
            322,
        );
        let low = 1.0 - ease(0.0, 0.2, vertex.pos.y);
        let rust = ease(0.02, 0.5, run + 0.45 * low - 0.1);
        let wash = Vector3::repeat(1.0).lerp(&RUST, 0.9 * rust);
        vertex.color.x *= wash.x;
        vertex.color.y *= wash.y;
        vertex.color.z *= wash.z;
    }
    Spots {
        frequency: 70.0,
        presence: 0.35,
        radius: (0.15, 0.4),
        tint: Vector3::new(0.45, 0.3, 0.2),
        salt: 323,
    }
    .paint(&mut mesh);
    // Jammed on askew: tipped back and to one side.
    let askew = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.16)
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.17);
    placed(mesh, askew, Vector3::new(0.004, 0.04, -0.03))
}

/// The neck: bare and plucked, the skin of it fallen in rolls, a windpipe
/// down its front, a crop hanging low on it and two wattles under the jaw.
/// Its skin, then the down left on it. In the neck's frame, the head's
/// middle `rig.neck_length` up it.
fn neck(rig: &StiltRigConfig) -> Vec<MeshPrimitive> {
    let top = rig.neck_length;
    // The line it follows, as (height, how far forward, how thick): out of
    // the collar, back, and up under the skull.
    let line = [
        (-0.05, 0.0, 0.064),
        (0.03, -0.012, 0.057),
        (0.11, -0.036, 0.05),
        (0.2, -0.04, 0.044),
        (0.29, -0.02, 0.039),
        (0.37, -0.004, 0.036),
        (top - 0.07, -0.012, 0.035),
        (top - 0.02, -0.04, 0.038),
    ];
    let along = |height: f32| {
        let after = line
            .iter()
            .position(|point| point.0 >= height)
            .unwrap_or(line.len() - 1)
            .max(1);
        let (a, b) = (line[after - 1], line[after]);
        let t = ((height - a.0) / (b.0 - a.0)).clamp(0.0, 1.0);
        (
            Vector3::new(0.0, height, a.1 + (b.1 - a.1) * t),
            a.2 + (b.2 - a.2) * t,
        )
    };
    // The core of it, then the rolls of slack skin down it.
    let mut smooth = Clay::new(0.02);
    for pair in line.windows(2) {
        let (from, thick_from) = along(pair[0].0);
        let (to, thick_to) = along(pair[1].0);
        smooth = smooth.add(
            limb(from, to, (0.86 * thick_from, 0.86 * thick_to)),
            BARE_SKIN,
        );
    }
    smooth = smooth.smoothing(0.0075);
    const ROLLS: u32 = 25;
    for roll in 0..ROLLS {
        let height =
            -0.02 + (top - 0.04) * roll as f32 / (ROLLS - 1) as f32 + 0.004 * scatter(roll, 331);
        let (at, thick) = along(height);
        let wide = thick * (1.0 + 0.07 * scatter(roll, 332));
        let turn = UnitQuaternion::from_euler_angles(
            0.16 * scatter(roll, 333),
            0.0,
            0.12 * scatter(roll, 334),
        );
        smooth = smooth.add(
            Lump::Ball(Ellipsoid {
                radii: Vector3::new(wide, 0.0145, wide),
                centre: at + Vector3::new(0.003 * scatter(roll, 335), 0.0, 0.0),
                turn,
            }),
            BARE_SKIN,
        );
    }
    // The windpipe, standing under the skin of its upper half.
    let pipe: Vec<_> = (0..=8)
        .map(|step| {
            let (at, thick) = along(0.19 + 0.03 * step as f32);
            (at + Vector3::new(0.0, 0.0, thick - 0.004), 0.0105)
        })
        .collect();
    smooth = cord(smooth.smoothing(0.012), &pipe, BARE_SKIN);
    // The crop, hanging heavy at its bottom, and the wattles.
    smooth = smooth
        .smoothing(0.022)
        .add(
            Lump::mass((0.028, 0.05, 0.032), (0.004, 0.165, 0.022)),
            CROP,
        )
        .add(
            Lump::mass((0.037, 0.046, 0.042), (0.007, 0.108, 0.04)),
            CROP,
        )
        .smoothing(0.008)
        .add(
            Lump::mass((0.0075, 0.034, 0.022), (0.012, top - 0.105, 0.03)),
            CROP,
        )
        .add(
            Lump::mass((0.0065, 0.026, 0.018), (-0.011, top - 0.098, 0.034)),
            CROP,
        );
    const PIMPLE: f32 = 0.0016;
    let pimpled = smooth.clone().relief(Relief::Scales {
        size: 0.014,
        depth: PIMPLE,
    });
    let mut skin = plucked(&pimpled, 0.0062);
    shade_relief(
        &mut skin,
        &smooth,
        (-PIMPLE, 0.0),
        Vector3::new(0.66, 0.56, 0.56),
        Vector3::new(1.06, 0.98, 0.94),
    );
    Spots {
        frequency: 48.0,
        presence: 0.22,
        radius: (0.2, 0.45),
        tint: Vector3::new(0.4, 0.28, 0.26),
        salt: 336,
    }
    .paint(&mut skin);
    // Grey with dirt where it goes into the collar.
    Grime {
        below: 0.02,
        fade: 0.1,
        tint: Vector3::new(0.6, 0.56, 0.52),
    }
    .paint(&mut skin);

    let mut wisps = Vec::new();
    for wisp in 0..46u32 {
        let height = 0.08 + 0.36 * (0.5 + 0.5 * scatter(wisp, 337));
        let (at, _) = along(height);
        let round = 3.14 + 1.3 * scatter(wisp, 338);
        let way = Vector3::new(round.sin(), 0.0, round.cos());
        let long = 0.018 + 0.03 * (0.5 + 0.5 * scatter(wisp, 339));
        wisps.push(Bristle {
            root: surface_toward(&smooth, at, way, 0.2) - way * 0.002,
            grows: (way + Vector3::new(0.0, -0.7, 0.0)).normalize() * long,
            bend: Vector3::new(0.0, -0.5 * long, 0.0),
            thick: 0.0009,
        });
    }
    vec![skin, bristles(&wisps, DOWN, Paint::Feather.id())]
}

/// The skirt of the robe: where its middle is and how wide and how deep it
/// is, `down` of the way from the shoulders (0) to the hem in front (1). It
/// widens as it falls, and sweeps back behind the legs.
fn skirt(rig: &StiltRigConfig, down: f32) -> (Vector3<f32>, f32, f32) {
    let top = rig.haunch_height - 0.12;
    let middle = Vector3::new(0.0, top - (top + 0.5) * down, -0.04 - 0.17 * down);
    (middle, 0.15 + 0.1 * down, 0.15 + 0.17 * down)
}

/// The point of the skirt's cloth `round` radians round it from straight
/// ahead, `down` of the way to the hem and `inset` metres in from its
/// surface.
fn on_skirt(rig: &StiltRigConfig, round: f32, down: f32, inset: f32) -> Vector3<f32> {
    let (middle, wide, deep) = skirt(rig, down);
    middle
        + Vector3::new(
            round.sin() * (wide - inset),
            0.0,
            round.cos() * (deep - inset),
        )
}

/// Where round the skirt `at` is, in the hips' frame: radians from
/// straight ahead, positive toward +X.
fn round_on_skirt(rig: &StiltRigConfig, at: Vector3<f32>) -> f32 {
    let (top, _, _) = skirt(rig, 0.0);
    let (bottom, _, _) = skirt(rig, 1.0);
    let down = (top.y - at.y) / (top.y - bottom.y);
    let (middle, wide, deep) = skirt(rig, down);
    ((at.x - middle.x) / wide).atan2((at.z - middle.z) / deep)
}

/// Take the back of `robe` below the hips off it, to hang as a drape the
/// legs push out.
fn back_of_robe(rig: &StiltRigConfig, robe: &mut [MeshPrimitive]) -> Drape {
    let behind = |at: Vector3<f32>| round_on_skirt(rig, at).abs() - (PI - DRAPE_ROUND);
    // A margin in metres, round the skirt where it is narrowest is the
    // most radians it takes.
    let (_, narrowest, _) = skirt(rig, 0.0);
    let draped = |at: Vector3<f32>, margin: f32| {
        at.y < DRAPE_SEWN - margin && behind(at) > margin / narrowest
    };
    // The robe keeps the drape's edge as well, a cell wide, where the drape
    // hardly moves: drawn in two frames, the two would otherwise crack
    // apart along the seam and show what is behind.
    let back = robe
        .iter_mut()
        .map(|mesh| {
            let drape = mesh.clone().split_off(|at| draped(at, 0.0));
            mesh.split_off(|at| draped(at, ROBE_GRAIN));
            drape
        })
        .collect();
    let freedom = |at: Vector3<f32>| {
        ease(DRAPE_SEWN, DRAPE_SEWN - DRAPE_EASE_DOWN, at.y)
            * ease(0.0, DRAPE_EASE_ROUND, behind(at))
    };
    Drape::new(
        back,
        DRAPE_SEWN,
        freedom,
        LEGS.hock_radius() + DRAPE_AIR,
        DRAPE_SPREAD,
    )
}

/// The robe: heavy cloth falling in pleats from a hunched back to a hem
/// that is longer behind, torn and trailing threads, and one sleeve. Its
/// cloth, what closes it from sight within, then its threads. In the hips'
/// frame.
fn robe(rig: &StiltRigConfig) -> Vec<MeshPrimitive> {
    let shoulder = rig.haunch_height;
    // The hem: a plane lower behind than in front.
    let hem_at = Vector3::new(0.0, -0.5, -0.1);
    let hem_slope = 0.45;
    let hem_height = |z: f32| hem_at.y + (z - hem_at.z) * hem_slope;
    let sleeve_from = Vector3::new(0.13, shoulder - 0.13, 0.0);
    let sleeve_to = Vector3::new(0.09, 0.13, 0.165);

    // The body under the cloth: the hunch of its back and shoulders, and
    // the skirt as a plain cone.
    let (top, _, _) = skirt(rig, 0.0);
    let (low, _, _) = skirt(rig, 1.3);
    let body = Clay::new(0.08)
        .add(
            Lump::mass((0.17, 0.12, 0.16), (0.0, shoulder - 0.08, -0.05)),
            ROBE,
        )
        .add(
            Lump::mass((0.13, 0.1, 0.12), (0.0, shoulder - 0.03, -0.13)),
            ROBE,
        )
        .add(limb(top, low, (0.135, 0.3)), ROBE);
    // The pleats: tubes of cloth from under the cowl to past the hem, each
    // its own width.
    const PLEATS: u32 = 15;
    let mut solid = body.clone().smoothing(0.032);
    for pleat in 0..PLEATS {
        let round = (pleat as f32 + 0.3 * scatter(pleat, 341)) / PLEATS as f32 * TAU;
        let full = 1.0 + 0.3 * scatter(pleat, 342);
        let thick = |down: f32| (0.026 + 0.036 * down) * full;
        // It breaks once on the way down, and falls a little aside of
        // where it began.
        let kink = 0.45 + 0.25 * (0.5 + 0.5 * scatter(pleat, 343));
        let sway = 0.06 * scatter(pleat, 337);
        let tube = [
            (on_skirt(rig, round, 0.0, 0.01), thick(0.0)),
            (on_skirt(rig, round + sway, kink, 0.01), thick(kink)),
            (on_skirt(rig, round + 0.4 * sway, 1.3, 0.01), thick(1.3)),
        ];
        solid = cord(solid, &tube, ROBE);
    }
    let skirts = solid.clone();
    let sleeve = Clay::new(0.0).add(limb(sleeve_from, sleeve_to, (0.08, 0.052)), ROBE);
    let solid = solid
        .smoothing(0.04)
        .add(limb(sleeve_from, sleeve_to, (0.08, 0.052)), ROBE);
    let mut draped = solid
        .clone()
        .hollow(ROBE_THICK)
        .cut(
            Lump::half_space(hem_at, Vector3::new(0.0, 1.0, -hem_slope)),
            0.006,
        )
        // The sleeve's mouth.
        .cut(
            ball(
                0.046,
                sleeve_to + (sleeve_to - sleeve_from).normalize() * 0.036,
            ),
            0.006,
        );
    // The hem is torn: bites out of it, and rents running up from it. Each
    // as how far round the skirt it is.
    for bite in 0..9u32 {
        let round = (bite as f32 + 0.4 * scatter(bite, 344)) / 9.0 * TAU;
        let rough = on_skirt(rig, round, 1.0, 0.0);
        let at = Vector3::new(rough.x, hem_height(rough.z) - 0.02, rough.z);
        let size = 0.035 + 0.035 * (0.5 + 0.5 * scatter(bite, 345));
        draped = draped.cut(ball(size, at), 0.004);
    }
    for (round, long) in [(0.6f32, 0.22f32), (2.3, 0.34), (3.7, 0.18), (5.2, 0.27)] {
        let rough = on_skirt(rig, round, 1.0, 0.0);
        let from = Vector3::new(rough.x, hem_height(rough.z) - 0.04, rough.z);
        let to = on_skirt(rig, round + 0.05, 1.0 - 1.3 * long, 0.0);
        draped = draped.cut(limb(from, to, (0.035, 0.004)), 0.003);
    }
    // Crumpled, as cloth is that has been slept in.
    let crumpled = draped.clone().relief(Relief::Lumps {
        frequency: 21.0,
        depth: 0.0045,
        seed: 346,
    });
    let mut cloth = crumpled.skin(ROBE_GRAIN, Paint::Cloth.id());

    // Its inside is kept only as far up as it can be seen from outside,
    // and all the way up the sleeve.
    const SEEN_WITHIN: f32 = 0.24;
    let within = |at: Vector3<f32>| solid.distance(at) < -0.5 * ROBE_THICK;
    let lost = |at: Vector3<f32>| {
        within(at) && at.y > hem_height(at.z) + SEEN_WITHIN && sleeve.distance(at) > 0.01
    };
    cloth.indices = cloth
        .indices
        .chunks_exact(3)
        .filter(|triangle| {
            !triangle
                .iter()
                .all(|index| lost(cloth.vertices[*index as usize].pos))
        })
        .flatten()
        .copied()
        .collect();

    // Faded in patches, greasy in others.
    Mottle {
        frequency: 3.2,
        value: 0.3,
        warmth: 0.18,
        seed: 347,
    }
    .paint(&mut cloth);
    Mottle {
        frequency: 26.0,
        value: 0.16,
        warmth: 0.05,
        seed: 348,
    }
    .paint(&mut cloth);
    for vertex in &mut cloth.vertices {
        // Dark inside, where no light gets up under it.
        if within(vertex.pos) {
            vertex.color.x *= 0.3;
            vertex.color.y *= 0.3;
            vertex.color.z *= 0.3;
            continue;
        }
        // Rubbed pale along the crown of each pleat, and dark with grease
        // in the valleys between.
        let proud = ease(0.0, 0.05, body.distance(vertex.pos));
        let wear = 0.65 + 0.6 * proud;
        // Dust and dried mud up from the hem: paler than the cloth.
        let above_hem = vertex.pos.y - hem_height(vertex.pos.z);
        let mud = 1.0 - ease(0.0, 0.22, above_hem);
        let wash = Vector3::repeat(wear).lerp(&Vector3::new(2.4, 2.0, 1.5), 0.6 * mud * mud);
        vertex.color.x *= wash.x;
        vertex.color.y *= wash.y;
        vertex.color.z *= wash.z;
    }

    // What closes it from sight above that: a slice of the same shape
    // across it, drawn in so that it lies within the cloth, of which only
    // the underside is kept.
    let ceiling = hem_at + Vector3::y() * (SEEN_WITHIN - 0.02);
    let across = Vector3::new(0.0, 1.0, -hem_slope);
    let mut lid = skirts
        .cut(Lump::half_space(ceiling, across), 0.0)
        .cut(
            Lump::half_space(ceiling + Vector3::y() * 0.06, -across),
            0.0,
        )
        .skin(0.024, Paint::Cloth.id());
    lid.indices = lid
        .indices
        .chunks_exact(3)
        .filter(|triangle| {
            triangle
                .iter()
                .all(|index| lid.vertices[*index as usize].normal.y < 0.3)
        })
        .flatten()
        .copied()
        .collect();
    let (middle, _, _) = skirt(rig, 0.6);
    for vertex in &mut lid.vertices {
        vertex.pos.x = middle.x + LID_DRAWN_IN * (vertex.pos.x - middle.x);
        vertex.pos.z = middle.z + LID_DRAWN_IN * (vertex.pos.z - middle.z);
        vertex.color = nalgebra::Vector4::new(0.012, 0.01, 0.01, 1.0);
    }

    // Threads hanging from the torn hem.
    let mut threads = Vec::new();
    for thread in 0..90u32 {
        let round = thread as f32 / 90.0 * TAU + 0.05 * scatter(thread, 349);
        let rough = on_skirt(rig, round, 1.0, 0.0);
        let height = hem_height(rough.z) + 0.012;
        let inward = Vector3::new(-round.sin(), 0.0, -round.cos());
        let outside = Vector3::new(rough.x, height, rough.z) - inward * 0.14;
        // Nothing to hang from where the hem is bitten away.
        let Some(root) = surface_inward(&crumpled, outside, inward, 0.24) else {
            continue;
        };
        let long = 0.025 + 0.07 * (0.5 + 0.5 * scatter(thread, 350)).powi(2);
        threads.push(Bristle {
            root,
            grows: Vector3::new(0.0, -long, 0.0),
            bend: Vector3::new(
                0.25 * long * scatter(thread, 351),
                0.0,
                -0.3 * long + 0.2 * long * scatter(thread, 352),
            ),
            thick: 0.0014,
        });
    }
    vec![
        cloth,
        lid,
        bristles(&threads, (0.12, 0.1, 0.08), Paint::Cloth.id()),
    ]
}

/// The cowl: a short cape of faded red over the shoulders, falling in
/// pleats to a ragged hem that stands off the robe, its hood in a heap
/// behind the neck and a thick collar round it. In the hips' frame.
fn cowl(rig: &StiltRigConfig) -> MeshPrimitive {
    let shoulder = rig.haunch_height;
    let neck_at = Vector3::new(0.0, shoulder + 0.03, -0.03);
    let hem_middle = Vector3::new(0.0, shoulder - 0.33, -0.08);
    // The shoulders under it, and the hood in a heap high on its back.
    let under = Clay::new(0.05)
        .add(
            Lump::mass((0.19, 0.14, 0.18), (0.0, shoulder - 0.075, -0.05)),
            COWL,
        )
        .add(
            limb(
                Vector3::new(0.0, shoulder - 0.06, -0.05),
                hem_middle + Vector3::y() * 0.03,
                (0.18, 0.215),
            ),
            COWL,
        )
        .add(
            Lump::mass((0.165, 0.14, 0.15), (0.0, shoulder + 0.01, -0.15)),
            COWL,
        )
        .add(
            Lump::mass((0.11, 0.075, 0.1), (0.03, shoulder + 0.1, -0.17)),
            COWL,
        );
    // The pleats: each from by the collar, over the shoulder, and down past
    // the hem.
    let mut cape = under.clone().smoothing(0.028);
    const PLEATS: u32 = 13;
    for pleat in 0..PLEATS {
        let round = (pleat as f32 + 0.35 * scatter(pleat, 353)) / PLEATS as f32 * TAU;
        let out = Vector3::new(round.sin(), 0.0, round.cos());
        let full = 1.0 + 0.3 * scatter(pleat, 354);
        let drape = [
            (neck_at + out * 0.1, 0.02 * full),
            (neck_at + out * 0.2 - Vector3::y() * 0.16, 0.03 * full),
            (
                hem_middle + out * (0.235 + 0.02 * scatter(pleat, 355)),
                0.045 * full,
            ),
        ];
        cape = cord(cape, &drape, COWL);
    }
    // The collar: a roll of it round the neck, gaping in front.
    let collar: Vec<_> = (0..=14)
        .map(|step| {
            let round = 0.35 + (TAU - 0.7) * step as f32 / 14.0;
            let sag = 0.02 * (round * 0.5).sin();
            let at = Vector3::new(
                round.sin() * 0.082,
                shoulder + sag,
                round.cos() * 0.08 - 0.012,
            );
            (at, 0.026)
        })
        .collect();
    cape = cord(cape, &collar, COWL);
    // Its hem: shorter in front than over the hump behind, and gnawed.
    let hem_at = Vector3::new(0.0, shoulder - 0.22, 0.0);
    let hem_slope = 0.25;
    let mut cape = cape.cut(
        Lump::half_space(hem_at, Vector3::new(0.0, 1.0, -hem_slope)),
        0.008,
    );
    for bite in 0..8u32 {
        let round = (bite as f32 + 0.4 * scatter(bite, 356)) / 8.0 * TAU;
        let out = Vector3::new(round.sin(), 0.0, round.cos()) * 0.23;
        let at = Vector3::new(
            out.x,
            hem_at.y + hem_slope * (hem_middle.z + out.z) - 0.012,
            hem_middle.z + out.z,
        );
        let size = 0.025 + 0.03 * (0.5 + 0.5 * scatter(bite, 357));
        cape = cape.cut(ball(size, at), 0.004);
    }
    let crumpled = cape.clone().relief(Relief::Lumps {
        frequency: 24.0,
        depth: 0.0045,
        seed: 358,
    });
    let mut cloth = crumpled.skin(0.012, Paint::Cloth.id());
    Mottle {
        frequency: 7.0,
        value: 0.34,
        warmth: 0.12,
        seed: 359,
    }
    .paint(&mut cloth);
    Mottle {
        frequency: 30.0,
        value: 0.18,
        warmth: 0.04,
        seed: 360,
    }
    .paint(&mut cloth);
    for vertex in &mut cloth.vertices {
        // Rubbed pale along the crown of each pleat, and dark between them.
        let proud = ease(0.0, 0.035, under.distance(vertex.pos));
        let wear = 0.55 + 0.75 * proud;
        vertex.color.x *= wear;
        vertex.color.y *= wear;
        vertex.color.z *= wear;
    }
    // Black with grease round the collar, and faded along its hem.
    Blush {
        at: Vector3::new(0.0, shoulder + 0.02, -0.02),
        radius: 0.2,
        tint: Vector3::new(0.4, 0.42, 0.45),
    }
    .paint(&mut cloth);
    Grime {
        below: shoulder - 0.26,
        fade: 0.12,
        tint: Vector3::new(1.45, 1.5, 1.4),
    }
    .paint(&mut cloth);
    cloth
}

/// What it has that is a man's: a hand out of the sleeve, hanging, and a
/// string of beads through its fingers with a key at the end. The hand,
/// the beads, then the key. In the hips' frame.
fn hand_and_beads() -> Vec<MeshPrimitive> {
    const SIZE: f32 = 0.19;
    let wrist = Vector3::new(0.092, 0.155, 0.158);
    // Fingers down, the palm turned back toward the body.
    let hanging = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.25)
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_2 - 0.25);
    let mut hand = anatomy::flesh(&anatomy::hand(SIZE, 0.8, FootSide::Right), DIGIT_GRAIN);
    // The sleeve hides where the wrist ends, so only a little of it is
    // kept.
    hand.indices = hand
        .indices
        .chunks_exact(3)
        .filter(|triangle| {
            triangle
                .iter()
                .any(|index| hand.vertices[*index as usize].pos.z > -0.1 * SIZE)
        })
        .flatten()
        .copied()
        .collect();
    for vertex in &mut hand.vertices {
        // Bloodless, and black under the nails.
        let along = (vertex.pos.z / SIZE - 0.6).clamp(0.0, 1.0);
        let dirt = 1.0 - 0.4 * along;
        vertex.color.x *= 0.86 * dirt;
        vertex.color.y *= 0.9 * dirt;
        vertex.color.z *= 0.92 * dirt;
    }
    let hand = placed(hand, hanging, wrist);

    // The beads: a string over the fingers, hanging in a loop.
    let hold = wrist + hanging * Vector3::new(0.0, -0.012, 0.5 * SIZE);
    const BEADS: u32 = 27;
    let mut string = Clay::new(0.0);
    let mut low = hold;
    for bead in 0..BEADS {
        let t = bead as f32 / (BEADS - 1) as f32;
        // Down one side of the loop and up the other.
        let round = t * TAU;
        let at = hold
            + Vector3::new(
                0.03 * round.sin(),
                -0.075 * (1.0 - round.cos()),
                0.012 * round.sin().abs() + 0.01,
            );
        if at.y < low.y {
            low = at;
        }
        let size = if bead % 9 == 4 { 0.0082 } else { 0.0062 };
        string = string.add(ball(size, at), BEAD);
    }
    let string = string.skin(0.003, Paint::Wood.id());

    // The key, on a short cord from the bottom of the loop.
    let top = low - Vector3::y() * 0.012;
    let end = top - Vector3::y() * 0.085;
    let bow: Vec<_> = (0..=10)
        .map(|step| {
            let round = step as f32 / 10.0 * TAU;
            (
                top + Vector3::new(0.013 * round.sin(), -0.013 * (1.0 - round.cos()), 0.0),
                0.0032,
            )
        })
        .collect();
    let mut key = cord(Clay::new(0.002), &bow, KEY)
        .add(limb(top - Vector3::y() * 0.026, end, (0.0034, 0.0034)), KEY);
    for (down, wide) in [(0.0f32, 0.014f32), (0.012, 0.01)] {
        let tooth = end + Vector3::y() * down;
        key = key.add(
            limb(tooth, tooth + Vector3::x() * wide, (0.003, 0.003)),
            KEY,
        );
    }
    let mut key = key.skin(0.0016, Paint::Tin.id());
    Mottle {
        frequency: 90.0,
        value: 0.3,
        warmth: 0.25,
        seed: 361,
    }
    .paint(&mut key);
    vec![hand, string, key]
}

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
            ankle_height: LEGS.ankle_height(),
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
            carriage: Carriage::default(),
        }
    }

    /// Its dress, against the palette's slots.
    pub fn sculpt() -> Dress {
        let rig = Self::rig();
        let mut head = skull();
        head.extend(upper_beak());
        head.push(hat());
        let mut body = robe(&rig);
        let drape = back_of_robe(&rig, &mut body);
        body.push(cowl(&rig));
        body.extend(hand_and_beads());
        let jaw = Joint::Hinge {
            pivot: jaw_pivot(),
            axis: Vector3::x(),
            drive: Drive::Strike { open: JAW_OPEN },
        };
        let mut pieces = vec![
            (Worn::on(Joint::Hips).breathing(0.012), body),
            (Worn::on(Joint::Neck), neck(&rig)),
            (Worn::on(Joint::Head), head),
            (Worn::on(jaw), lower_beak()),
        ];
        // Pale eyes, close-set: a man's, and no kinder for it.
        let eye = anatomy::eyeball(
            EYE_RADIUS,
            EyeLook {
                iris: (0.4, 0.5, 0.54),
                iris_angle: 0.5,
                pupil_share: 0.32,
                bloodshot: 0.8,
            },
        );
        let (upper, lower) = (UPPER_LID.mesh(), LOWER_LID.mesh());
        for side in [1.0, -1.0] {
            let turning = Joint::Eye {
                socket: socket(side),
                swivel: EYE_SWIVEL,
            };
            pieces.push((Worn::on(turning), vec![eye.clone()]));
            pieces.push((Worn::on(UPPER_LID.hinge(side)), vec![upper.clone()]));
            pieces.push((Worn::on(LOWER_LID.hinge(side)), vec![lower.clone()]));
        }
        pieces.extend(LEGS.legs(&rig));
        Dress::of(pieces).draped(drape)
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
                blow: PECK_BLOW,
                windup: PECK_WINDUP,
                recovery: PECK_RECOVERY,
            }),
            harm: Some(HarmConfig::default()),
            shell: None,
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

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn its_dress_is_tens_of_thousands_of_triangles_and_every_part_is_worn() {
        let started = Instant::now();
        let dress = StalkerDef::sculpt();
        println!("a stalker is sculpted in {:?}", started.elapsed());
        assert_eq!(dress.worn.len(), dress.model.parts.len());
        for (worn, part) in dress.worn.iter().zip(&dress.model.parts) {
            let triangles: usize = part.primitives.iter().map(|m| m.indices.len() / 3).sum();
            println!("  {:?}: {triangles}", worn.joint);
        }
        let triangles = dress.triangles();
        println!("a stalker is {triangles} triangles");
        assert!(
            (30_000..210_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn a_dent_in_its_hat_is_a_dent_and_not_a_hole() {
        let funnel = funnel();
        for dent in &DENTS {
            // Walking in through the dent from the ball that made it: tin,
            // and then the hollow of the hat.
            let (from, inward) = (dent.ball_at(), -dent.out());
            let tin = (0..80)
                .map(|step| from + inward * (dent.size + 0.0005 * step as f32))
                .filter(|at| funnel.distance(*at) < 0.0)
                .count();
            println!(
                "under the dent at {}: {} mm of tin",
                dent.round,
                0.5 * tin as f32
            );
            assert!(tin >= 12, "the dent at {} goes through", dent.round);
        }
        // Skinned, it has one way through it and no more: in at the rim
        // and out at the spout.
        let mesh = hat();
        let mut edges = std::collections::HashSet::new();
        for triangle in mesh.indices.chunks_exact(3) {
            for k in 0..3 {
                let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        let faces = mesh.indices.len() / 3;
        let euler = mesh.vertices.len() as i64 - edges.len() as i64 + faces as i64;
        assert_eq!(euler, 0, "its skin has {} holes too many", -euler / 2);
    }

    #[test]
    fn its_lids_meet_when_they_shut() {
        let upper = UPPER_LID.edge - UPPER_LID.shut;
        let lower = LOWER_LID.edge - LOWER_LID.shut;
        assert!(
            upper <= lower,
            "a gap between the lids: {upper} over {lower}"
        );
        assert!(lower - upper < 0.1, "the lids cross: {upper} over {lower}");
    }

    #[test]
    fn its_beak_reaches_ahead_of_its_head_and_its_jaw_hangs_from_its_root() {
        let upper = upper_beak();
        let tip = upper[0]
            .vertices
            .iter()
            .map(|vertex| vertex.pos.z)
            .fold(f32::MIN, f32::max);
        assert!((0.6..0.7).contains(&tip), "its beak reaches {tip}");
        // About its hinge, the lower half starts just ahead of the origin
        // and lies below the seam.
        let lower = &lower_beak()[0];
        let (back, top) = lower
            .vertices
            .iter()
            .fold((f32::MAX, f32::MIN), |(back, top), vertex| {
                (back.min(vertex.pos.z), top.max(vertex.pos.y))
            });
        assert!((-0.03..0.03).contains(&back), "its jaw starts at {back}");
        assert!(top < 0.02, "its jaw rises to {top}");
    }
}
