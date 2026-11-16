//! The bag: a belly of skin like a stomach, what is sewn and tied onto it,
//! and its painted hide.
//!
//! ```text
//!   belly ──form──▶ the bag as seen: a teat round each stock, the collar
//!     │             round the neck, rolls where it sags, the seam's welt,
//!     │             the pleats of its tied-off neck ──hide──▶ painted skin
//!     └──thread──▶ the stitches over the seam and the cord round the neck
//! ```

use std::f32::consts::{PI, TAU};

use nalgebra::Vector3;

use super::frame::RADIUS;
use super::pipes::Pipe;
use super::shaping::{ease, surface, triple, turned_to, vector, wash};
use crate::app::creatures::palette::Paint;
use crate::app::sculpt::{
    scatter, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::model::MeshPrimitive;

/// Side of the grid the bag is skinned from, m, and the grid of its thread.
const BAG_GRAIN: f32 = 0.016;
const THREAD_GRAIN: f32 = 0.006;

/// The bag's skin (linear): where it is full of blood, and where it is
/// drawn thin over a swelling or gathered at a tie.
const HIDE: (f32, f32, f32) = (0.42, 0.17, 0.16);
const HIDE_THIN: (f32, f32, f32) = (0.54, 0.31, 0.24);
/// Coarse waxed thread and cord (linear).
const THREAD: (f32, f32, f32) = (0.13, 0.09, 0.055);

/// The middle of the bag's weight, which its swellings are found from.
pub(super) const CORE: (f32, f32, f32) = (0.0, -0.05, -0.02);
/// Where the bag's mouth is gathered round the head's neck: the top of the
/// collar, a neck's length under the head.
pub(super) const THROAT: (f32, f32, f32) = (0.0, 0.26, 0.35);
/// Where the collar round the head's neck rises from the belly.
pub(super) const COLLAR: (f32, f32, f32) = (0.0, 0.14, 0.24);
/// Where the bag's neck is tied off, behind it.
const TIE: (f32, f32, f32) = (0.12, 0.13, -0.6);

/// The bag before anything is sewn or tied to it: a belly sagging onto
/// the ground, swollen unevenly as a stomach is, and drawn out behind
/// into the neck it is tied off by.
pub(super) fn belly() -> Clay {
    let tie = vector(TIE);
    Clay::new(0.14)
        .add(Lump::mass((0.4, 0.33, 0.45), (0.0, -0.05, -0.02)), HIDE)
        .add(Lump::mass((0.43, 0.16, 0.47), (0.01, -0.26, 0.0)), HIDE)
        .add(Lump::ball(0.25, (-0.14, 0.14, -0.2)), HIDE_THIN)
        .add(Lump::ball(0.21, (0.19, 0.08, 0.14)), HIDE_THIN)
        .add(Lump::ball(0.2, (-0.06, -0.12, 0.3)), HIDE)
        .smoothing(0.1)
        .add(
            Lump::limb((0.06, 0.02, -0.36), triple(tie), (0.15, 0.044)),
            HIDE_THIN,
        )
}

/// The bag as it is seen: the belly with a teat drawn up round every
/// stock and bound there, a collar gathered round the head's neck,
/// rolls where it sags onto the ground, the welt of its seam, and the
/// pleats where it is tied off.
pub(super) fn form(belly: &Clay, stocks: &[&Pipe]) -> Clay {
    let core = vector(CORE);
    let mut bag = belly.clone();

    // A teat drawn up round each stock, its edge rolled over where it
    // is bound.
    for pipe in stocks {
        let along = (pipe.tip - pipe.foot).normalize();
        let (wide, narrow) = (4.2 * pipe.girth, 2.5 * pipe.girth);
        bag = bag
            .smoothing(0.06)
            .add(
                Lump::limb(
                    triple(pipe.foot - along * 0.1),
                    triple(pipe.foot + along * 0.05),
                    (wide, narrow),
                ),
                HIDE_THIN,
            )
            .smoothing(0.012)
            .add(
                Lump::Ball(Ellipsoid {
                    radii: Vector3::new(1.22 * narrow, 0.016, 1.22 * narrow),
                    centre: pipe.foot + along * (0.05 + 0.6 * narrow),
                    turn: turned_to(along, Vector3::z()),
                }),
                HIDE,
            );
    }

    // The collar: the bag's own mouth, drawn up round the neck of the
    // head and gathered there in a ring of puckers.
    let throat = vector(THROAT);
    bag = bag
        .smoothing(0.08)
        .add(Lump::limb(COLLAR, triple(throat), (0.17, 0.082)), HIDE_THIN);
    bag = bag.smoothing(0.01);
    for pucker in 0..13 {
        let round = TAU * pucker as f32 / 13.0;
        let out = Vector3::new(round.cos(), 0.0, round.sin());
        let fat = 0.017 + 0.004 * scatter(pucker as u32, 94);
        bag = bag.add(
            Lump::ball(fat, triple(throat + out * 0.08 + Vector3::y() * 0.012)),
            HIDE,
        );
    }

    // Where it sags: rolls of it lying on one another round its foot,
    // none of them running all the way round.
    bag = bag.smoothing(0.035);
    for (height, fat, from, to) in [
        (-0.335, 0.05, 0.3f32, 3.6f32),
        (-0.26, 0.042, 2.2, 5.9),
        (-0.37, 0.04, 3.9, 6.6),
        (-0.2, 0.034, 0.9, 2.5),
    ] {
        let steps = ((to - from) / 0.3).ceil() as i32;
        let at = |step: i32| {
            let round = from + (to - from) * step as f32 / steps as f32;
            let droop = 0.025 * (round * 2.3 + height * 40.0).sin();
            let level = Vector3::new(core.x, height + droop, core.z);
            let (point, out) = surface(belly, level, Vector3::new(round.sin(), 0.0, round.cos()));
            point - out * 0.35 * fat
        };
        for step in 0..steps {
            // Thin at its ends, where it runs back into the bag.
            let taper = |step: i32| {
                let along = step as f32 / steps as f32;
                fat * (0.45 + 0.55 * (along * PI).sin())
            };
            bag = bag.add(
                Lump::limb(
                    triple(at(step)),
                    triple(at(step + 1)),
                    (taper(step), taper(step + 1)),
                ),
                HIDE,
            );
        }
    }

    // The seam: a welt pinched up into a row of swellings by its
    // stitches.
    bag = bag.smoothing(0.014);
    let seam = seam(belly);
    for pair in seam.windows(2) {
        let ((from, out), (to, _)) = (pair[0], pair[1]);
        let middle = 0.5 * (from + to) + out * 0.004;
        bag = bag.add(Lump::ball(0.021, triple(middle)), HIDE_THIN);
    }

    // The neck: pleats run into the tie, and past it the end of the
    // skin stands out in a rosette.
    let tie = vector(TIE);
    let back = (tie - Vector3::new(0.06, 0.02, -0.36)).normalize();
    let turn = turned_to(back, Vector3::y());
    bag = bag.smoothing(0.01);
    for pleat in 0..9 {
        let round = TAU * pleat as f32 / 9.0 + 0.3;
        let out = turn * Vector3::new(round.cos(), 0.0, round.sin());
        bag = bag
            .add(
                Lump::limb(
                    triple(tie - back * 0.17 + out * 0.105),
                    triple(tie - back * 0.01 + out * 0.036),
                    (0.02, 0.011),
                ),
                HIDE,
            )
            .add(
                Lump::limb(
                    triple(tie + back * 0.012 + out * 0.03),
                    triple(tie + back * 0.06 + out * 0.062),
                    (0.012, 0.019),
                ),
                HIDE_THIN,
            );
    }
    bag.add(Lump::ball(0.034, triple(tie + back * 0.03)), HIDE)
        // Nothing that was once inside something else is quite even.
        .relief(Relief::Lumps {
            frequency: 5.0,
            depth: 0.012,
            seed: 74,
        })
}

/// The line the bag is sewn up along, over its top from the collar to
/// the neck: a point for each stitch, and the way out of the bag there.
fn seam(belly: &Clay) -> Vec<(Vector3<f32>, Vector3<f32>)> {
    let core = vector(CORE);
    let stitches = 18;
    (0..=stitches)
        .map(|stitch| {
            let along = stitch as f32 / stitches as f32;
            let over = -0.2 + 1.5 * along;
            let wander = 0.34 + 0.09 * (along * 7.0).sin();
            surface(
                belly,
                core,
                Vector3::new(wander * over.cos().max(0.3), over.cos(), -over.sin()),
            )
        })
        .collect()
}

/// The bag's thread: a stitch over the seam's welt between each of its
/// swellings, and the cord its neck is tied off with.
pub(super) fn thread(belly: &Clay) -> MeshPrimitive {
    let mut thread = Clay::new(0.002);
    let seam = seam(belly);
    for (stitch, pair) in seam.windows(2).enumerate().skip(1) {
        let ((at, out), (next, _)) = (pair[0], pair[1]);
        let along = (next - at).normalize();
        let across = out.cross(&along).normalize();
        // Not sewn by anyone who cared: each a little askew.
        let skew = along * 0.012 * scatter(stitch as u32, 75);
        let wide = 0.034 + 0.008 * scatter(stitch as u32, 76);
        let run = [
            at - across * wide - out * 0.008 + skew,
            at - across * 0.55 * wide + out * 0.012 + 0.5 * skew,
            at + out * 0.019,
            at + across * 0.55 * wide + out * 0.012 - 0.5 * skew,
            at + across * wide - out * 0.008 - skew,
        ];
        for piece in run.windows(2) {
            thread = thread.add(
                Lump::limb(triple(piece[0]), triple(piece[1]), (0.0052, 0.0052)),
                THREAD,
            );
        }
    }
    // Two turns of cord round the neck, and its two ends hanging.
    let tie = vector(TIE);
    let back = (tie - Vector3::new(0.06, 0.02, -0.36)).normalize();
    let turn = turned_to(back, Vector3::y());
    let wind = |step: i32| {
        let round = TAU * step as f32 / 9.0;
        let out = turn * Vector3::new(round.cos(), 0.0, round.sin());
        tie + back * (-0.01 + 0.0014 * step as f32) + out * 0.044
    };
    for step in 0..18 {
        thread = thread.add(
            Lump::limb(triple(wind(step)), triple(wind(step + 1)), (0.0065, 0.0065)),
            THREAD,
        );
    }
    for (aside, drop) in [(0.03, 0.13), (-0.018, 0.085)] {
        let knot = wind(18);
        let end = knot + Vector3::new(aside, -drop, 0.01);
        thread = thread
            .add(
                Lump::limb(triple(knot), triple(end), (0.006, 0.005)),
                THREAD,
            )
            .add(Lump::ball(0.009, triple(end)), THREAD);
    }
    let mut mesh = thread.skin(THREAD_GRAIN, Paint::Husk.id());
    Mottle {
        frequency: 60.0,
        value: 0.3,
        warmth: 0.1,
        seed: 77,
    }
    .paint(&mut mesh);
    mesh
}

/// The bag's skin, painted: blotched, run with veins, dark and
/// bruise-coloured in its creases, pale where it is drawn tight, and
/// dirty where it sits.
pub(super) fn hide(bag: &Clay) -> MeshPrimitive {
    let mut mesh = bag.skin(BAG_GRAIN, Paint::Horn.id());
    // Great patches where the blood sits, and where it does not.
    Mottle {
        frequency: 2.6,
        value: 0.24,
        warmth: 0.22,
        seed: 78,
    }
    .paint(&mut mesh);
    Mottle {
        frequency: 9.0,
        value: 0.14,
        warmth: 0.1,
        seed: 79,
    }
    .paint(&mut mesh);
    // The great veins, blue under the skin.
    Veins {
        frequency: 3.6,
        width: 0.045,
        tint: Vector3::new(0.46, 0.42, 0.74),
        seed: 80,
    }
    .paint(&mut mesh);
    Veins {
        frequency: 7.5,
        width: 0.03,
        tint: Vector3::new(0.72, 0.6, 0.78),
        seed: 87,
    }
    .paint(&mut mesh);
    Spots {
        frequency: 9.0,
        presence: 0.2,
        radius: (0.2, 0.42),
        tint: Vector3::new(0.7, 0.52, 0.5),
        salt: 88,
    }
    .paint(&mut mesh);
    wash(&mut mesh, |vertex| {
        let closed = (1.0 - vertex.ao) / 0.75;
        // Blood pools in a crease; a swelling's crown is drawn thin.
        let crease = Vector3::repeat(1.0).lerp(&Vector3::new(0.36, 0.2, 0.3), closed);
        let drawn = 0.3 * ease(0.2, 0.95, vertex.normal.y) * (1.0 - closed);
        crease.component_mul(&Vector3::new(
            1.0 + 0.35 * drawn,
            1.0 + 1.0 * drawn,
            1.0 + 0.8 * drawn,
        ))
    });
    Grime {
        below: -RADIUS + 0.04,
        fade: 0.2,
        tint: Vector3::new(0.48, 0.4, 0.33),
    }
    .paint(&mut mesh);
    mesh
}
