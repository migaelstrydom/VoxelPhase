//! The ferry fish's form: the clay its body is skinned from and the places
//! on it everything else is set by, shared by its look and by the deck it
//! collides as, so that what the player stands on is what they see.
//!
//! ```text
//!   deck_form (disc + head) ──sampled in rings──► Deck::hull ──┐
//!   eye turrets ──────────────────────────────► Deck::eyes ───┴─► Deck::centre
//!   body_form = deck_form + tail stalk + eye turrets + shoulders
//!   rim(side, z), top(x, z) ──► where its fins root, its eyes sit, its wings hinge
//! ```
//!
//! Everything is sculpted in one frame, its nose along +Z and its eyes up,
//! and moved by [`Deck::centre`] so that the body's origin, which is its
//! centre of mass, is the model's.

use std::f32::consts::{PI, TAU};
use std::sync::{Arc, OnceLock};

use nalgebra::Vector3;

use super::super::fish_anatomy::{point, surface_along, surface_within};
use crate::app::sculpt::{Clay, Lump};
use crate::app::spawnables::shared::models::{build_convex_hull, SolidFace};
use crate::collision::ConvexHull;
use crate::geometry::convex_hull;

/// Its skin before it is painted (linear): the olive sand of its back.
pub const SAND: (f32, f32, f32) = (0.29, 0.245, 0.155);

/// Its body: a thin disc for its edges, a dome for the thick of its back
/// along its middle, and its head. Each is half its width, thickness and
/// length, and its middle: a lens in section, thin where its fins root.
const DISC: ((f32, f32, f32), (f32, f32, f32)) = ((1.55, 0.13, 2.0), (0.0, 0.0, -0.2));
const DOME: ((f32, f32, f32), (f32, f32, f32)) = ((0.95, 0.26, 1.75), (0.04, 0.0, -0.25));
const HEAD: ((f32, f32, f32), (f32, f32, f32)) = ((0.98, 0.24, 0.82), (0.08, 0.0, 1.28));
/// How far the disc, the dome and the head are smoothed into one another, m.
const DECK_BLEND: f32 = 0.3;
/// The stalk its tail grows from: two lumps narrowing toward the tail.
const STALK: [((f32, f32, f32), (f32, f32, f32)); 2] = [
    ((0.5, 0.12, 0.6), (0.0, 0.0, -2.1)),
    ((0.24, 0.09, 0.42), (0.0, 0.0, -2.5)),
];

/// How far forward its snout reaches, and where along it its tail fin
/// roots, m.
pub const SNOUT: f32 = 2.1;
pub const TAIL_ROOT: f32 = -2.82;

/// The radius of an eye, m.
pub const EYE_RADIUS: f32 = 0.125;
/// Each eye: where it sits across and along the head, and the way it looks
/// at rest. Both are on its upper side, close together, the front one
/// further out.
pub const EYES: [((f32, f32), (f32, f32, f32)); 2] = [
    ((0.52, 1.6), (0.75, 0.62, 0.25)),
    ((0.2, 1.28), (-0.3, 0.88, 0.2)),
];
/// How high over the skin of its head an eye's middle stands, m.
const EYE_RAISE: f32 = 0.07;
/// The radius of the ball each eye and its turret collide as, m.
pub const EYE_REACH: f32 = 0.16;

/// How far forward along its rim its wings are hinged, and how far in from
/// the rim and down from its middle, m.
pub const WING_AT: f32 = 0.75;
const WING_TUCK: (f32, f32) = (0.05, 0.03);

/// The deck is sampled in this many directions round its middle, at the
/// rim and in rings over its back (each a share of the way out to the
/// rim), and in one ring under it.
const HULL_ROUND: usize = 20;
const HULL_RINGS: [f32; 4] = [0.95, 0.85, 0.6, 0.3];
const HULL_UNDER: f32 = 0.7;

/// The lumps that bear a load: the disc and the head.
pub fn deck_form() -> Clay {
    [DISC, DOME, HEAD]
        .into_iter()
        .fold(Clay::new(DECK_BLEND), |clay, (radii, at)| {
            clay.add(Lump::mass(radii, at), SAND)
        })
}

/// Its outline: the deck and the stalk its tail grows from.
pub fn outline_form() -> Clay {
    STALK
        .iter()
        .fold(deck_form().smoothing(0.3), |clay, (radii, at)| {
            clay.add(Lump::mass(*radii, *at), SAND)
        })
}

/// Its whole body before it is carved: the outline, the turrets its eyes
/// stand in and the shoulders its wings are rooted in.
pub fn body_form() -> Clay {
    let mut clay = outline_form().smoothing(0.09);
    for eye in 0..EYES.len() {
        let at = eye_centre(eye);
        clay = clay.add(
            Lump::mass((0.18, 0.1, 0.18), (at.x, at.y - 0.05, at.z)),
            SAND,
        );
    }
    clay = clay.smoothing(0.12);
    for side in [-1.0, 1.0] {
        let root = wing_root(side);
        clay = clay.add(
            Lump::mass((0.12, 0.07, 0.3), (root.x, root.y + 0.01, root.z)),
            SAND,
        );
    }
    clay
}

/// The edge of its outline on `side` (+1 or −1 along X), `z` along it, at
/// the height its fins root at.
pub fn rim(side: f32, z: f32) -> Vector3<f32> {
    surface_along(
        &outline_form(),
        Vector3::new(0.0, 0.0, z),
        Vector3::x() * side,
    )
}

/// The skin of its back over (`x`, `z`), which must be within its deck.
pub fn top(x: f32, z: f32) -> Vector3<f32> {
    surface_along(&deck_form(), Vector3::new(x, 0.0, z), Vector3::y())
}

/// The middle of eye `eye`, in the frame it is sculpted in.
pub fn eye_centre(eye: usize) -> Vector3<f32> {
    let (across, along) = EYES[eye].0;
    top(across, along) + Vector3::y() * EYE_RAISE
}

/// The way eye `eye` looks at rest.
pub fn eye_rest(eye: usize) -> Vector3<f32> {
    point(EYES[eye].1).normalize()
}

/// Where the wing on `side` is hinged, in the frame it is sculpted in.
pub fn wing_root(side: f32) -> Vector3<f32> {
    rim(side, WING_AT) - Vector3::new(side * WING_TUCK.0, WING_TUCK.1, 0.0)
}

/// What the fish collides as: its deck, and a ball round each eye, all
/// about its centre of mass.
pub struct Deck {
    /// The deck, a hull of points on the skin of its back and belly.
    pub hull: Arc<ConvexHull>,
    /// The middle of the ball round each eye.
    pub eyes: [Vector3<f32>; 2],
    /// Where its centre of mass is in the frame it is sculpted in: what
    /// every part of its look is moved by.
    pub centre: Vector3<f32>,
    /// Half the box the deck fills, across, through and along.
    pub reach: Vector3<f32>,
}

impl Deck {
    /// The fish's deck, made the first time it is asked for.
    pub fn get() -> &'static Deck {
        static DECK: OnceLock<Deck> = OnceLock::new();
        DECK.get_or_init(Self::make)
    }

    /// `at`, in the frame it is sculpted in, as a point in its body's frame.
    pub fn placed(&self, at: Vector3<f32>) -> Vector3<f32> {
        at - self.centre
    }

    fn make() -> Self {
        let (corners, triangles) =
            convex_hull(&hull_corners()).expect("the deck encloses a volume");
        let faces: Vec<SolidFace> = triangles
            .iter()
            .map(|&[a, b, c]| {
                // Whatever lies furthest behind the face is behind it.
                let normal = (corners[b] - corners[a]).cross(&(corners[c] - corners[a]));
                let opposite = (0..corners.len())
                    .min_by(|&i, &j| {
                        let behind = |k: usize| (corners[k] - corners[a]).dot(&normal);
                        behind(i).total_cmp(&behind(j))
                    })
                    .unwrap_or(0);
                SolidFace {
                    vertex_indices: vec![a, b, c],
                    opposite_vertex: opposite,
                }
            })
            .collect();
        let hull = build_convex_hull(&corners, &faces);

        // Its weight is the hull's and the eyes' balls', all of a density.
        let ball = 4.0 / 3.0 * PI * EYE_REACH.powi(3);
        let eyes = [eye_centre(0), eye_centre(1)];
        let volume = hull.compute_volume();
        let centre = (hull.centroid() * volume + eyes.iter().sum::<Vector3<f32>>() * ball)
            / (volume + 2.0 * ball);

        let (low, high) = corners.iter().fold(
            (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN)),
            |(low, high), v| (low.inf(v), high.sup(v)),
        );
        Self {
            hull: Arc::new(hull.translated(-centre)),
            eyes: eyes.map(|eye| eye - centre),
            centre,
            reach: (high - low) * 0.5,
        }
    }
}

/// Points on the skin of the deck: round its rim, in rings over its back
/// (each a share of the way out to the rim), in one ring under it, and the
/// middle of its back and of its belly.
fn hull_corners() -> Vec<Vector3<f32>> {
    let deck = deck_form();
    let middle = Vector3::new(0.0, 0.0, DISC.1 .2);
    let mut corners = Vec::new();
    for round in 0..HULL_ROUND {
        let angle = round as f32 / HULL_ROUND as f32 * TAU;
        let way = Vector3::new(angle.sin(), 0.0, angle.cos());
        let edge = surface_within(&deck, middle, way, 4.0);
        corners.push(edge);
        let within = |share: f32| middle + (edge - middle) * share;
        for share in HULL_RINGS {
            corners.push(surface_along(&deck, within(share), Vector3::y()));
        }
        corners.push(surface_along(&deck, within(HULL_UNDER), -Vector3::y()));
    }
    corners.push(surface_along(&deck, middle, Vector3::y()));
    corners.push(surface_along(&deck, middle, -Vector3::y()));
    corners
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_deck_lies_on_the_skin_of_its_back_and_is_about_its_weight() {
        let deck = Deck::get();
        let form = deck_form();
        // Every corner of the hull is on the skin, so what is stood on is
        // what is drawn; and the faces between them sag no more than a few
        // centimetres inside it.
        for corner in &deck.hull.vertices {
            let at = *corner + deck.centre;
            assert!(
                form.distance(at).abs() < 2e-3,
                "a corner {at:?} off the skin"
            );
        }
        // Wherever it is flat enough to stand on, its faces sag no more
        // than a few centimetres inside the skin.
        let mut worst: f32 = 0.0;
        for face in deck.hull.faces.iter().filter(|face| face.normal.y > 0.8) {
            let middle = face
                .vertex_indices
                .iter()
                .map(|&i| deck.hull.vertices[i as usize])
                .sum::<Vector3<f32>>()
                / face.vertex_indices.len() as f32;
            worst = worst.max(-form.distance(middle + deck.centre));
        }
        println!("the deck sags {worst} m at worst");
        assert!(worst < 0.03, "a face sags {worst} m inside the skin");
        // It is about as big as it is sculpted: a deck three metres across
        // and four long, half a metre thick.
        assert!((deck.reach.x - 1.55).abs() < 0.08, "{:?}", deck.reach);
        assert!((0.25..0.35).contains(&deck.reach.y), "{:?}", deck.reach);
        assert!(deck.reach.z > 2.0 && deck.reach.z < 2.3, "{:?}", deck.reach);
        // The hull and the eyes' balls weigh about the origin.
        let volume = deck.hull.compute_volume();
        let ball = 4.0 / 3.0 * PI * EYE_REACH.powi(3);
        let moment = deck.hull.centroid() * volume + deck.eyes.iter().sum::<Vector3<f32>>() * ball;
        assert!(moment.norm() / volume < 1e-3, "{moment:?}");
    }

    #[test]
    fn its_eyes_stand_up_out_of_its_head_and_its_wings_hinge_at_its_edge() {
        for eye in 0..EYES.len() {
            let at = eye_centre(eye);
            let skin = top(at.x, at.z);
            assert!(at.y - skin.y > 0.05, "eye {eye} is sunk in the head");
            assert!(eye_rest(eye).y > 0.5, "eye {eye} does not look up");
        }
        for side in [-1.0, 1.0] {
            let root = wing_root(side);
            assert!(root.x * side > 1.2, "a wing is hinged {root:?}");
            assert!(
                deck_form().distance(root) < 0.0,
                "a wing's root is not in the body"
            );
        }
    }
}
