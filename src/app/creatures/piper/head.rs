//! The head sewn into the bag's mouth: bald, bowed over its pipe, its
//! cheeks blown out and its eyes starting from it.
//!
//! Everything here is modelled in the head's own frame (its middle at the
//! origin, +Z out of its face) and set on the piper by [`bow`] and
//! [`HEAD`]; [`in_head`] takes a point across.

use std::f32::consts::TAU;

use nalgebra::{UnitQuaternion, Vector3};

use super::shaping::{ease, surface, triple, turned_to, vector, wash};
use crate::app::creatures::anatomy::{self, EyeLook, FLESH, FLUSH};
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Lump, Mottle, Relief, Spots, VertexPaint,
};
use crate::model::MeshPrimitive;

/// Side of the grid the head is skinned from, m, and the grids of its
/// cheeks and its neck.
const HEAD_GRAIN: f32 = 0.005;
const CHEEK_GRAIN: f32 = 0.006;
const NECK_GRAIN: f32 = 0.011;

/// The middle of the head, how far it is bowed over its pipe, radians, and
/// the radius of its eyes, m.
pub(super) const HEAD: (f32, f32, f32) = (0.0, 0.385, 0.385);
const HEAD_BOW: f32 = 0.35;
const EYE_RADIUS: f32 = 0.024;
/// Where each eye sits in the head, mirrored for the other, and where the
/// lips close on the pipe; both in the head's own frame.
const SOCKET: (f32, f32, f32) = (0.036, 0.008, 0.074);
pub(super) const LIPS: (f32, f32, f32) = (0.0, -0.08, 0.098);
/// What the cheeks swell about, in the head's own frame.
pub(super) const CHEEK_ROOT: (f32, f32, f32) = (0.0, -0.055, 0.02);

/// The turn that bows the head over its pipe.
pub(super) fn bow() -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), HEAD_BOW)
}

/// A point of the head's own frame, in the piper's.
pub(super) fn in_head(point: (f32, f32, f32)) -> Vector3<f32> {
    vector(HEAD) + bow() * vector(point)
}

/// The neck, in the head's own frame: from under the jaw down into the
/// bag's collar, further than the bag ever sinks.
pub(super) fn neck() -> MeshPrimitive {
    let mut neck = Clay::new(0.02).add(
        Lump::limb((0.0, -0.1, -0.04), (0.0, -0.31, -0.11), (0.054, 0.064)),
        FLESH,
    );
    // The cords that stand in a neck when its owner strains, and the
    // lump in the throat between them.
    for side in [-1.0, 1.0] {
        neck = neck.add(
            Lump::limb(
                (0.05 * side, -0.07, -0.02),
                (0.018 * side, -0.3, -0.045),
                (0.013, 0.012),
            ),
            FLESH,
        );
    }
    let neck = neck.add(Lump::ball(0.02, (0.0, -0.17, 0.0)), FLUSH);
    let mut mesh = anatomy::flesh(&neck, NECK_GRAIN);
    wash(&mut mesh, |vertex| {
        let closed = (1.0 - vertex.ao) / 0.75;
        Vector3::new(
            1.02 - 0.3 * closed,
            0.92 - 0.45 * closed,
            0.86 - 0.4 * closed,
        )
    });
    mesh
}

/// The head, in its own frame: bald, its brows drawn up in the middle,
/// lids stretched round eyes that start from it, its lips a ring round
/// the pipe, a vein standing on its temple.
pub(super) fn head() -> MeshPrimitive {
    let lid = (0.47, 0.27, 0.25);
    let lip = (0.36, 0.14, 0.15);
    let vein = (0.4, 0.34, 0.44);
    let socket = vector(SOCKET);
    let eye = EYE_RADIUS;
    // The skull, with the mask of the face on the front of it.
    let skull = Clay::new(0.03)
        .add(Lump::mass((0.085, 0.096, 0.1), (0.0, 0.04, -0.015)), FLESH)
        .add(Lump::mass((0.068, 0.08, 0.07), (0.0, -0.035, 0.03)), FLESH)
        .add(
            Lump::mass((0.05, 0.036, 0.048), (0.0, -0.102, 0.034)),
            FLESH,
        );
    let mut head = skull
        .clone()
        // The top of the neck, under the jaw.
        .add(
            Lump::limb((0.0, -0.07, -0.03), (0.0, -0.13, -0.05), (0.056, 0.058)),
            FLESH,
        )
        .smoothing(0.014);
    for side in [-1.0, 1.0] {
        head = head
            // The cheekbone and the root of the cheek, and the brow,
            // drawn up at its inner end.
            .add(Lump::ball(0.02, (0.05 * side, -0.016, 0.062)), FLESH)
            .add(Lump::ball(0.034, (0.042 * side, -0.06, 0.04)), FLESH)
            .add(
                Lump::limb(
                    (0.066 * side, 0.03, 0.058),
                    (0.015 * side, 0.044, 0.087),
                    (0.014, 0.0125),
                ),
                FLESH,
            )
            // The lids: a ball of thin skin the eye sits in, and the
            // bag under it.
            .add(
                Lump::ball(1.32 * eye, (socket.x * side, socket.y, socket.z)),
                lid,
            )
            .add(
                Lump::limb(
                    (0.06 * side, -0.026, 0.066),
                    (0.02 * side, -0.03, 0.084),
                    (0.009, 0.0105),
                ),
                lid,
            )
            // A wing of the nose.
            .add(Lump::ball(0.011, (0.016 * side, -0.041, 0.099)), FLUSH)
            // An ear, standing out, and its lobe.
            .add(
                Lump::Ball(
                    Ellipsoid::new((0.011, 0.033, 0.023), (0.088 * side, 0.0, -0.012))
                        .yawed(-0.5 * side),
                ),
                FLUSH,
            )
            .add(Lump::ball(0.011, (0.087 * side, -0.034, -0.004)), FLUSH);
    }
    head = head
        .add(
            Lump::limb((0.0, 0.024, 0.088), (0.0, -0.028, 0.111), (0.011, 0.015)),
            FLESH,
        )
        .add(Lump::ball(0.016, (0.0, -0.034, 0.112)), FLUSH);
    // The lips, a ring pursed round the pipe and split with dryness.
    let lips = vector(LIPS);
    let purse = turned_to(Vector3::new(0.0, -0.5, 0.87), Vector3::y());
    let ring = |step: i32| {
        let round = TAU * step as f32 / 9.0;
        triple(lips + purse * Vector3::new(round.cos() * 0.019, 0.0, round.sin() * 0.017))
    };
    for step in 0..9 {
        let full = 0.0105 + 0.0025 * scatter(step as u32, 89);
        head = head.add(Lump::limb(ring(step), ring(step + 1), (full, full)), lip);
    }
    // Veins standing under the skin: up the temple and over the
    // skull, a branch of that toward the brow, and one down the
    // forehead. Each is a course of ways out from the skull's middle.
    head = head.smoothing(0.005);
    let courses: [&[(f32, f32, f32)]; 3] = [
        &[
            (-1.0, 0.1, 0.5),
            (-1.0, 0.36, 0.34),
            (-0.92, 0.62, 0.42),
            (-0.9, 0.95, 0.2),
            (-0.7, 1.2, 0.05),
            (-0.55, 1.3, -0.3),
        ],
        &[(-1.0, 0.4, 0.36), (-0.8, 0.5, 0.62), (-0.55, 0.62, 0.78)],
        &[
            (0.13, 0.42, 1.0),
            (0.05, 0.7, 0.9),
            (0.17, 1.0, 0.7),
            (0.08, 1.3, 0.38),
        ],
    ];
    let middle = Vector3::new(0.0, 0.01, -0.01);
    for course in courses {
        let under: Vec<_> = course
            .iter()
            .map(|way| {
                let (point, out) = surface(&skull, middle, vector(*way));
                triple(point - out * 0.0025)
            })
            .collect();
        for pair in under.windows(2) {
            head = head.add(Lump::limb(pair[0], pair[1], (0.0046, 0.004)), vein);
        }
    }
    // Lines across a forehead that has strained a long time.
    let mut head = head.relief(Relief::Folds {
        along: Vector3::x(),
        frequency: 30.0,
        depth: 0.001,
        seed: 90,
    });
    for side in [-1.0, 1.0] {
        let socket = Vector3::new(socket.x * side, socket.y, socket.z);
        head = head
            .cut(Lump::ball(1.04 * eye, triple(socket)), 0.002)
            // Between the lids, held wide.
            .cut(
                Lump::mass(
                    (0.98 * eye, 0.58 * eye, 0.9 * eye),
                    (socket.x, socket.y - 0.06 * eye, socket.z + 0.86 * eye),
                ),
                0.002,
            )
            .cut(Lump::ball(0.006, (0.01 * side, -0.047, 0.106)), 0.002)
            .cut(Lump::ball(0.012, (0.097 * side, 0.002, 0.0)), 0.004);
    }
    let mut mesh = anatomy::flesh(&head, HEAD_GRAIN);
    // Red with the effort where the blood is near the skin, dark
    // round eyes that do not shut, and an old man's spots on the skull.
    for (at, radius, tint) in [
        ((0.0, -0.034, 0.108), 0.04, (1.0, 0.72, 0.7)),
        ((0.088, 0.0, -0.01), 0.05, (1.0, 0.7, 0.7)),
        ((-0.088, 0.0, -0.01), 0.05, (1.0, 0.7, 0.7)),
        ((0.04, -0.012, 0.08), 0.042, (0.74, 0.6, 0.72)),
        ((-0.04, -0.012, 0.08), 0.042, (0.74, 0.6, 0.72)),
        ((0.0, 0.1, 0.03), 0.11, (1.06, 0.95, 0.9)),
    ] {
        Blush {
            at: vector(at),
            radius,
            tint: vector(tint),
        }
        .paint(&mut mesh);
    }
    Spots {
        frequency: 30.0,
        presence: 0.2,
        radius: (0.14, 0.32),
        tint: Vector3::new(0.62, 0.46, 0.38),
        salt: 91,
    }
    .paint(&mut mesh);
    wash(&mut mesh, |vertex| {
        let closed = (1.0 - vertex.ao) / 0.75;
        Vector3::new(
            1.06 - 0.3 * closed,
            0.96 - 0.5 * closed,
            0.9 - 0.45 * closed,
        )
    });
    mesh
}

/// The cheeks, in the head's own frame: blown out as far as skin
/// goes, the blood driven to the crown of each.
pub(super) fn cheeks() -> MeshPrimitive {
    let mut cheeks = Clay::new(0.02);
    for side in [-1.0, 1.0] {
        cheeks = cheeks
            .add(
                Lump::mass((0.043, 0.042, 0.041), (0.052 * side, -0.06, 0.052)),
                FLESH,
            )
            .add(Lump::ball(0.026, (0.04 * side, -0.09, 0.048)), FLESH);
    }
    let mut mesh = anatomy::flesh(&cheeks, CHEEK_GRAIN);
    Mottle {
        frequency: 45.0,
        value: 0.12,
        warmth: 0.2,
        seed: 92,
    }
    .paint(&mut mesh);
    // Flushed at the crown of each, where it is drawn thinnest, and in
    // the shadow of the face where it leaves it.
    let root = vector(CHEEK_ROOT);
    wash(&mut mesh, |vertex| {
        let out = ease(0.05, 0.085, (vertex.pos - root).norm());
        let shade = 0.62 + 0.38 * ease(0.045, 0.062, (vertex.pos - root).norm());
        Vector3::new(1.06, 0.96 - 0.2 * out, 0.9 - 0.16 * out) * shade
    });
    mesh
}

/// One of its eyes.
pub(super) fn eye() -> MeshPrimitive {
    anatomy::eyeball(
        EYE_RADIUS,
        EyeLook {
            iris: (0.3, 0.26, 0.16),
            iris_angle: 0.46,
            pupil_share: 0.34,
            bloodshot: 1.0,
        },
    )
}

/// Where each of its eyes sits and looks at rest, in the piper's frame.
pub(super) fn sockets() -> [(Vector3<f32>, Vector3<f32>); 2] {
    [1.0, -1.0].map(|side| {
        (
            in_head((SOCKET.0 * side, SOCKET.1, SOCKET.2)),
            bow() * Vector3::new(0.14 * side, 0.1, 1.0).normalize(),
        )
    })
}
