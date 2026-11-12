//! The parts of bodies creatures are put together from: bare human legs and
//! feet, hands that grip, and eyes.
//!
//! ```text
//!   thigh, shin, foot   modelled for the stilt rig's bones (`animation::
//!                       stilts::Joint`): each about its own joint
//!   hand                about its wrist, fingers along +Z, curling to −Y
//!   eyeball             about its middle, looking along +Z
//!   flesh()             a clay form skinned and painted as living skin
//! ```
//!
//! Bosch's creatures unsettle because the parts are a person's and the
//! whole is not, so these are modelled as a person's: a kneecap and a calf,
//! an ankle bone, five toes, knuckles, a wet eye with a ring round its iris.

use std::f32::consts::PI;

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};

use super::palette::Paint;
use crate::animation::FootSide;
use crate::app::sculpt::{
    Blush, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::MeshPrimitive;
use crate::rendering::colour::Colour;
use crate::utils::noise::fbm_perlin_3d;

/// Skin as it is pressed in clay (linear): pale, where the blood is near
/// the surface, and on a sole or a palm.
pub const FLESH: (f32, f32, f32) = (0.5, 0.37, 0.3);
pub const FLUSH: (f32, f32, f32) = (0.52, 0.27, 0.23);
pub const SOLE: (f32, f32, f32) = (0.55, 0.36, 0.28);
/// A nail.
pub const NAIL: (f32, f32, f32) = (0.5, 0.42, 0.32);

/// How far above the sole an ankle turns, m: where a shin ends and where
/// a foot's ankle bone sits, so the two stay joined however the shin leans
/// over the foot.
pub const ANKLE_RISE: f32 = 0.05;

/// How far forward of the ankle a foot's middle is, as a share of its
/// length: the heel is short behind the leg and the toes long ahead, so
/// the back of the leg runs down into the heel.
const FOOT_AHEAD: f32 = 0.1;

/// Side of the grid flesh is skinned from, m, for a limb and for a hand or
/// a foot, whose toes need more.
pub const LIMB_GRAIN: f32 = 0.009;
pub const DIGIT_GRAIN: f32 = 0.0042;

/// The uneven colour of skin.
const SKIN_MOTTLE: Mottle = Mottle {
    frequency: 11.0,
    value: 0.16,
    warmth: 0.12,
    seed: 81,
};

/// Veins under it: few, faint and blue.
const SKIN_VEINS: Veins = Veins {
    frequency: 9.0,
    width: 0.03,
    tint: Vector3::new(0.78, 0.84, 0.95),
    seed: 82,
};

/// Blotches: broad patches where the blood sits nearer the skin.
const SKIN_BLOTCHES: Mottle = Mottle {
    frequency: 3.5,
    value: 0.16,
    warmth: 0.3,
    seed: 84,
};

/// Stubble: close-set dark points, seen only where the skin is skinned
/// finely enough to hold them.
const SKIN_STUBBLE: Spots = Spots {
    frequency: 110.0,
    presence: 0.55,
    radius: (0.2, 0.34),
    tint: Vector3::new(0.72, 0.66, 0.62),
    salt: 85,
};

/// Moles and freckles.
const SKIN_MOLES: Spots = Spots {
    frequency: 26.0,
    presence: 0.16,
    radius: (0.12, 0.3),
    tint: Vector3::new(0.55, 0.38, 0.3),
    salt: 83,
};

/// `clay` skinned from a grid of `grain` metres and painted as living
/// skin.
pub fn flesh(clay: &Clay, grain: f32) -> MeshPrimitive {
    let mut mesh = clay.skin(grain, Paint::Skin.id());
    SKIN_MOTTLE.paint(&mut mesh);
    SKIN_BLOTCHES.paint(&mut mesh);
    SKIN_VEINS.paint(&mut mesh);
    SKIN_STUBBLE.paint(&mut mesh);
    SKIN_MOLES.paint(&mut mesh);
    mesh
}

/// +1 for the side whose inner edge (its big toe, its thumb) lies toward
/// −X, −1 for the other. The rig hangs the left leg on +X.
fn inward(side: FootSide) -> f32 {
    match side {
        FootSide::Left => -1.0,
        FootSide::Right => 1.0,
    }
}

/// A bare thigh `length` long and `girth` thick at the hip: the hip at its
/// origin, the knee `length` below it, the kneecap toward +Z.
pub fn thigh(length: f32, girth: f32) -> Clay {
    Clay::new(0.6 * girth)
        .add(
            Lump::limb((0.0, 0.04, 0.0), (0.0, -length, 0.0), (girth, 0.62 * girth)),
            FLESH,
        )
        // The muscle over the front of it, and the hamstring behind.
        .add(
            Lump::mass(
                (0.8 * girth, 0.34 * length, 0.7 * girth),
                (0.0, -0.4 * length, 0.3 * girth),
            ),
            FLESH,
        )
        .add(
            Lump::mass(
                (0.7 * girth, 0.3 * length, 0.6 * girth),
                (0.0, -0.45 * length, -0.32 * girth),
            ),
            FLESH,
        )
        // The kneecap, proud of the joint.
        .smoothing(0.22 * girth)
        .add(
            Lump::mass(
                (0.36 * girth, 0.4 * girth, 0.26 * girth),
                (0.0, -length + 0.1 * girth, 0.36 * girth),
            ),
            FLUSH,
        )
}

/// A bare shin `length` long under a thigh of `girth`: the knee at its
/// origin, the ground `length` below it. It ends in an ankle,
/// [`ANKLE_RISE`] short of the ground.
pub fn shin(length: f32, girth: f32) -> Clay {
    let ankle = -length + ANKLE_RISE;
    Clay::new(0.5 * girth)
        .add(
            Lump::limb(
                (0.0, 0.0, 0.0),
                (0.0, ankle, 0.0),
                (0.6 * girth, 0.3 * girth),
            ),
            FLESH,
        )
        // The calf, high on the back.
        .add(
            Lump::mass(
                (0.5 * girth, 0.24 * length, 0.52 * girth),
                (0.0, -0.3 * length, -0.3 * girth),
            ),
            FLESH,
        )
        .smoothing(0.2 * girth)
        // The edge of the shin bone down the front.
        .add(
            Lump::limb(
                (0.0, -0.1 * length, 0.42 * girth),
                (0.0, ankle + 0.05, 0.2 * girth),
                (0.16 * girth, 0.1 * girth),
            ),
            FLESH,
        )
        // The tendon from the calf down the back to the heel.
        .add(
            Lump::limb(
                (0.0, -0.45 * length, -0.42 * girth),
                (0.0, ankle + 0.01, -0.36 * girth),
                (0.16 * girth, 0.12 * girth),
            ),
            FLESH,
        )
        .add(Lump::ball(0.2 * girth, (0.3 * girth, ankle, -0.02)), FLUSH)
        .add(
            Lump::ball(0.2 * girth, (-0.3 * girth, ankle + 0.012, -0.02)),
            FLUSH,
        )
}

/// A bare foot `length` long, standing on y = 0 with the leg coming down
/// onto its origin: toes toward +Z, the big toe on the inner side.
pub fn foot(length: f32, side: FootSide) -> MeshPrimitive {
    let l = length;
    let inner = inward(side);
    let ahead = FOOT_AHEAD * l;
    let mut clay = Clay::new(0.07 * l)
        // Heel, the column of the ankle over it, and the instep running
        // down to the ball of the foot.
        .add(Lump::ball(0.14 * l, (0.0, 0.14 * l, ahead - 0.2 * l)), SOLE)
        // The ankle bone the shin turns on, and the tendon from the heel
        // up the back of it.
        .add(Lump::ball(0.13 * l, (0.0, ANKLE_RISE, 0.0)), FLESH)
        .add(
            Lump::limb(
                (0.0, 0.16 * l, ahead - 0.26 * l),
                (0.0, ANKLE_RISE + 0.015, -0.07 * l),
                (0.05 * l, 0.04 * l),
            ),
            FLESH,
        )
        .add(
            Lump::limb(
                (0.0, 0.22 * l, ahead - 0.06 * l),
                (0.0, 0.1 * l, ahead + 0.42 * l),
                (0.17 * l, 0.1 * l),
            ),
            FLESH,
        )
        .add(
            Lump::mass(
                (0.2 * l, 0.075 * l, 0.17 * l),
                (0.01 * l * inner, 0.08 * l, ahead + 0.4 * l),
            ),
            SOLE,
        )
        .smoothing(0.025 * l);
    // Five toes, the big one inside and each shorter and thinner outward:
    // across, how far forward its root is, how long, how thick.
    let toes: [(f32, f32, f32, f32); 5] = [
        (0.13, 0.5, 0.25, 0.062),
        (0.05, 0.53, 0.2, 0.042),
        (-0.025, 0.52, 0.18, 0.04),
        (-0.095, 0.5, 0.15, 0.037),
        (-0.155, 0.46, 0.12, 0.034),
    ];
    for (across, root, reach, thick) in toes {
        let root = root + FOOT_AHEAD;
        let x = across * l * inner;
        // Splayed a little from the foot's middle line, and bent at its
        // one knuckle so its pad rests on the ground.
        let splay = across * 0.25 * inner;
        let knuckle = (
            x + splay * 0.5 * reach * l,
            0.085 * l,
            (root + 0.55 * reach) * l,
        );
        let tip = (
            x + splay * reach * l,
            0.5 * thick * l + 0.004,
            (root + reach) * l,
        );
        clay = clay
            .add(
                Lump::limb(
                    (x, 0.075 * l, root * l),
                    knuckle,
                    (thick * l, 0.9 * thick * l),
                ),
                FLESH,
            )
            .add(
                Lump::limb(knuckle, tip, (0.9 * thick * l, 0.8 * thick * l)),
                FLUSH,
            )
            // The nail, set into the top of its tip.
            .add(
                Lump::mass(
                    (0.6 * thick * l, 0.2 * thick * l, 0.55 * thick * l),
                    (tip.0, tip.1 + 0.62 * thick * l, tip.2 - 0.25 * thick * l),
                ),
                NAIL,
            );
    }
    let mut mesh = flesh(&clay, DIGIT_GRAIN.max(0.02 * l));
    // It has walked here.
    Grime {
        below: 0.012,
        fade: 0.07 * l + 0.02,
        tint: Vector3::new(0.42, 0.34, 0.27),
    }
    .paint(&mut mesh);
    mesh
}

/// A hand `size` long from wrist to fingertip, its wrist at the origin and
/// its fingers along +Z, each joint of them curled `curl` radians toward
/// −Y: a little for a hand at rest, more to grip an edge. The thumb is on
/// the inner side.
pub fn hand(size: f32, curl: f32, side: FootSide) -> Clay {
    let s = size;
    let inner = inward(side);
    let mut clay = Clay::new(0.05 * s)
        .add(
            Lump::limb(
                (0.0, 0.0, -0.3 * s),
                (0.0, 0.0, 0.05 * s),
                (0.12 * s, 0.13 * s),
            ),
            FLESH,
        )
        .add(
            Lump::mass((0.19 * s, 0.06 * s, 0.22 * s), (0.0, 0.0, 0.26 * s)),
            FLESH,
        )
        .smoothing(0.016 * s);
    // Fingers: across the palm, how long against the middle one.
    let fingers = [(-0.15, 0.86), (-0.05, 1.0), (0.05, 0.94), (0.145, 0.76)];
    let bend = |turns: f32| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), curl * turns) * Vector3::z()
    };
    for (across, long) in fingers {
        let reach = 0.56 * s * long;
        let mut at = Vector3::new(across * s * inner, 0.0, 0.44 * s);
        let mut thick = 0.03 * s;
        // Three bones, each curled on from the last, with the knuckle each
        // turns on standing under thin skin.
        for (bone, share) in [0.45, 0.3, 0.25].into_iter().enumerate() {
            let along = bend(bone as f32 + 0.6);
            let next = at + along * reach * share;
            let colour = if bone == 0 { FLESH } else { FLUSH };
            let back = Vector3::x().cross(&along);
            let knuckle = at - back * 0.25 * thick;
            clay = clay
                .add(
                    Lump::limb(
                        (at.x, at.y, at.z),
                        (next.x, next.y, next.z),
                        (thick, 0.86 * thick),
                    ),
                    colour,
                )
                .add(
                    Lump::ball(1.16 * thick, (knuckle.x, knuckle.y, knuckle.z)),
                    FLUSH,
                );
            at = next;
            thick *= 0.86;
        }
        // The nail, laid along the back of the last bone.
        let along = bend(2.6);
        let back = Vector3::x().cross(&along);
        let nail = at - along * 0.05 * s - back * 0.9 * thick;
        clay = clay.add(
            Lump::Ball(Ellipsoid {
                radii: Vector3::new(0.026 * s, 0.009 * s, 0.045 * s),
                centre: nail,
                turn: UnitQuaternion::rotation_between(&Vector3::z(), &along)
                    .unwrap_or_else(UnitQuaternion::identity),
            }),
            NAIL,
        );
    }
    // The thumb: from the heel of the palm, tucked in under it.
    let root = Vector3::new(0.15 * s * -inner, -0.02 * s, 0.12 * s);
    let knuckle = root + Vector3::new(0.03 * s * -inner, -0.06 * s, 0.05 * s);
    let tip = knuckle + Vector3::new(0.04 * s * inner, -0.04 * s - 0.03 * s * curl, 0.07 * s);
    clay.add(
        Lump::limb(
            (root.x, root.y, root.z),
            (knuckle.x, knuckle.y, knuckle.z),
            (0.06 * s, 0.048 * s),
        ),
        FLESH,
    )
    .add(
        Lump::limb(
            (knuckle.x, knuckle.y, knuckle.z),
            (tip.x, tip.y, tip.z),
            (0.046 * s, 0.04 * s),
        ),
        FLUSH,
    )
}

/// How an eye is coloured.
#[derive(Clone, Copy, Debug)]
pub struct EyeLook {
    /// The iris (linear).
    pub iris: (f32, f32, f32),
    /// How wide the iris is, as the angle from the eye's axis to its edge,
    /// radians, and how much of that the pupil takes.
    pub iris_angle: f32,
    pub pupil_share: f32,
    /// How bloodshot the white is, 0 to 1.
    pub bloodshot: f32,
}

impl Default for EyeLook {
    fn default() -> Self {
        Self {
            iris: (0.32, 0.4, 0.3),
            iris_angle: 0.48,
            pupil_share: 0.42,
            bloodshot: 0.45,
        }
    }
}

/// Segments round an eyeball: enough to carry the iris painted on it.
const EYE_SEGMENTS: u32 = 64;

/// An eyeball of `radius` about its own middle, looking along +Z: a wet
/// white run with blood from behind, an iris of fibres with a dark ring
/// round it, a pupil, and the bulge of the cornea over them.
pub fn eyeball(radius: f32, look: EyeLook) -> MeshPrimitive {
    let white = Vector3::new(0.86, 0.82, 0.72);
    let blood = Vector3::new(0.55, 0.06, 0.05);
    let iris = Vector3::new(look.iris.0, look.iris.1, look.iris.2);
    let mut vertices = generate_sphere_vertices(1.0, EYE_SEGMENTS, EYE_SEGMENTS, Colour::WHITE);
    for vertex in &mut vertices {
        // The sphere's own pole is +Y; the eye looks along +Z.
        let at = Vector3::new(vertex.pos.x, vertex.pos.z, vertex.pos.y);
        let off = at.z.clamp(-1.0, 1.0).acos();
        let round = at.y.atan2(at.x);
        let rim = look.iris_angle;

        let colour = if off < rim * look.pupil_share {
            Vector3::repeat(0.012)
        } else if off < rim {
            // Fibres running out from the pupil, paler near it, and a dark
            // ring where the iris meets the white.
            let out = (off / rim - look.pupil_share) / (1.0 - look.pupil_share);
            let fibre = fbm_perlin_3d(round.cos() * 9.0, round.sin() * 9.0, out * 1.5, 3, 0.6, 91);
            let ring = ((out - 0.78) / 0.22).clamp(0.0, 1.0);
            iris * (1.25 - 0.5 * out + 0.45 * fibre) * (1.0 - 0.75 * ring * ring)
        } else {
            // Veins that thicken toward the back of the eye, and the whole
            // white reddening there.
            let behind = ((off - rim) / (PI * 0.75 - rim)).clamp(0.0, 1.0);
            let field = fbm_perlin_3d(at.x * 3.2, at.y * 3.2, at.z * 3.2, 3, 0.55, 92);
            let vein = (1.0 - field.abs() * 8.0).clamp(0.0, 1.0);
            let red =
                look.bloodshot * (vein * vein * (0.25 + 0.75 * behind) + 0.35 * behind * behind);
            white.lerp(&blood, red.clamp(0.0, 0.9))
        };
        // The cornea stands a little proud of the ball.
        let bulge = 1.0 + 0.07 * (1.0 - (off / (rim * 1.25)).min(1.0).powi(2)).max(0.0);
        vertex.pos = at * radius * bulge;
        vertex.normal = at;
        vertex.color = Vector4::new(colour.x, colour.y, colour.z, 1.0);
        vertex.tex_coords = Vector2::zeros();
    }
    // Swapping two axes turned the sphere inside out: wind it back.
    let mut indices = generate_sphere_indices(EYE_SEGMENTS, EYE_SEGMENTS);
    for triangle in indices.chunks_exact_mut(3) {
        triangle.swap(1, 2);
    }
    MeshPrimitive {
        vertices,
        indices,
        material: Paint::Wet.id(),
    }
}

/// A lid for an eye of `radius` at the origin looking along +Z: a cap of
/// skin over its top (or under it, for a lower lid) that reaches `cover`
/// radians down from the crown, hinged to turn about the X axis.
pub fn eyelid(radius: f32, cover: f32, lower: bool, colour: (f32, f32, f32)) -> MeshPrimitive {
    let outer = radius * 1.09;
    let sign = if lower { -1.0 } else { 1.0 };
    // A shell round the eye, with everything past the lid's edge cut away
    // by a ball so large its side is nearly a plane.
    let edge = outer * cover.cos();
    let huge = 40.0 * radius;
    let clay = Clay::new(0.0)
        .add(Lump::ball(outer, (0.0, 0.0, 0.0)), colour)
        .hollow(0.09 * radius)
        // Creased, as a lid is that has been held open too long.
        .relief(Relief::Folds {
            along: Vector3::x(),
            frequency: 3.5 / radius,
            depth: 0.012 * radius,
            seed: 86,
        })
        .cut(
            Lump::ball(huge, (0.0, sign * (edge - huge), 0.0)),
            0.03 * radius,
        );
    let mut mesh = clay.skin((0.07 * radius).max(0.0025), Paint::Skin.id());
    SKIN_MOTTLE.paint(&mut mesh);
    // Raw and wet along its edge.
    for vertex in &mut mesh.vertices {
        let from_edge = ((sign * vertex.pos.y - edge) / (0.16 * radius)).clamp(0.0, 1.0);
        let raw = 1.0 - from_edge;
        vertex.color.x *= 1.0 - 0.1 * raw;
        vertex.color.y *= 1.0 - 0.5 * raw;
        vertex.color.z *= 1.0 - 0.5 * raw;
    }
    Blush {
        at: Vector3::new(0.0, sign * outer, 0.0),
        radius: 0.9 * radius,
        tint: Vector3::new(0.86, 0.8, 0.82),
    }
    .paint(&mut mesh);
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_leg_reaches_from_hip_to_ground_and_stands_on_five_toes() {
        let thigh = flesh(&thigh(0.4, 0.07), 0.01);
        let low = thigh
            .vertices
            .iter()
            .map(|v| v.pos.y)
            .fold(f32::MAX, f32::min);
        assert!((-0.47..-0.4).contains(&low), "a thigh ends at {low}");

        let foot = foot(0.24, FootSide::Left);
        let sole = foot
            .vertices
            .iter()
            .map(|v| v.pos.y)
            .fold(f32::MAX, f32::min);
        assert!(sole.abs() < 0.012, "its sole is at {sole}");
        let toes = foot
            .vertices
            .iter()
            .map(|v| v.pos.z)
            .fold(f32::MIN, f32::max);
        assert!((0.17..0.22).contains(&toes), "its toes reach {toes}");
        // The heel is short behind the ankle, a quarter of the foot at most.
        let heel = foot
            .vertices
            .iter()
            .map(|v| v.pos.z)
            .fold(f32::MAX, f32::min);
        assert!(heel > -0.25 * (toes - heel), "its heel reaches {heel}");
        // The big toe is the furthest forward, and on the inner side: −X
        // for a left foot.
        let foremost = foot
            .vertices
            .iter()
            .max_by(|a, b| a.pos.z.total_cmp(&b.pos.z))
            .unwrap();
        assert!(
            foremost.pos.x < -0.01,
            "the big toe is at x {}",
            foremost.pos.x
        );
    }

    #[test]
    fn an_eye_is_dark_at_its_pupil_and_white_at_its_side() {
        let eye = eyeball(0.05, EyeLook::default());
        let at = |dir: Vector3<f32>| {
            let nearest = eye.vertices.iter().max_by(|a, b| {
                a.pos
                    .normalize()
                    .dot(&dir)
                    .total_cmp(&b.pos.normalize().dot(&dir))
            });
            *nearest.unwrap()
        };
        assert!(at(Vector3::z()).color.x < 0.05, "no pupil ahead");
        assert!(at(Vector3::x()).color.x > 0.6, "no white at the side");
        // Wound to face outward.
        let t = &eye.indices[..3];
        let [a, b, c] = [t[0], t[1], t[2]].map(|i| eye.vertices[i as usize].pos);
        let face = (b - a).cross(&(c - a));
        assert!(
            face.dot(&((a + b + c) / 3.0)) > 0.0,
            "the eye is inside out"
        );
    }
}
