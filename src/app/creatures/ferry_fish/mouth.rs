//! The ferry fish's mouth: thick lips round a wet pocket, a chin under
//! them and a gilt ring through the lower lip, all one skin with its face.
//!
//! ```text
//!          upper lip       share 0    sculpted with the jaw a little
//!   hinge ●═══════════                dropped, as a fish breathing holds
//!          ╲ lower lip     share 1    it: its lips parted, its corners,
//!           ╲____chin____╱            on the hinge, joined; opened further
//!                 ╲ throat  fading    by turning each vertex about the
//!                                     hinge by its share (`model::Jaw`)
//! ```
//!
//! Its muzzle is skinned finer than its body and laid over it
//! (`sculpt::patch`), its share fading to nothing well inside its edge, so
//! that the edge stays on the body's skin however wide the jaw opens.

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector3};

use super::super::fish_anatomy::point;
use super::super::palette::Paint;
use super::form::{Deck, SAND};
use super::sculpt::{carved, dressed, paint_skin, patch, placed, smooth, CREVICES};
use crate::app::sculpt::{Clay, Ellipsoid, Lump, Mottle};
use crate::model::{Jaw, MeshPrimitive};

/// The crooked line of its mouth: the corner on −X and on +X, which its
/// jaw turns on; how far forward and how high each lip pouts at its
/// middle, and how thick it is there, with the jaw shut.
const MOUTH_CORNERS: [(f32, f32, f32); 2] = [(-0.12, 0.05, 1.98), (0.22, -0.06, 1.96)];
const UPPER_LIP: ((f32, f32, f32), f32) = ((0.04, 0.07, 2.24), 0.065);
const LOWER_LIP: ((f32, f32, f32), f32) = ((0.07, -0.07, 2.22), 0.078);

/// How far its jaw is dropped as sculpted, radians: far enough that its
/// lips stand clear of each other, so that they can part.
const REST_GAPE: f32 = 0.2;

/// Ellipsoids in the frame of its mouth (across it toward its +X corner,
/// up, and forward, about the middle of its corners), radii and centre:
/// its muzzle, which holds the whole of its mouth however wide; its chin,
/// under its lower lip and jutting below and ahead of its snout; and, in
/// the frame half dropped so that they lie between its lips as sculpted,
/// the pocket of its mouth behind its lips and the slit its lips part
/// along, which stops short of its corners.
const MUZZLE: ((f32, f32, f32), (f32, f32, f32)) = ((0.38, 0.3, 0.38), (0.0, -0.04, 0.06));
const CHIN: ((f32, f32, f32), (f32, f32, f32)) = ((0.21, 0.12, 0.19), (0.0, -0.1, 0.0));
const POCKET: ((f32, f32, f32), (f32, f32, f32)) = ((0.16, 0.07, 0.15), (0.0, 0.0, -0.02));
const SLIT: ((f32, f32, f32), (f32, f32, f32)) = ((0.17, 0.018, 0.16), (0.0, 0.0, 0.12));

/// How a vertex's share of its jaw's turn fades: across the line its lips
/// part along, m either side; behind the hinge toward its throat, from and
/// to m behind; past its corners, from and to m across; and toward the edge
/// of its muzzle, from and to the share of the way out to it.
const PARTING: f32 = 0.012;
const THROAT: (f32, f32) = (0.04, 0.2);
const CHEEKS: (f32, f32) = (0.16, 0.28);
const EDGE: (f32, f32) = (0.8, 0.95);

/// Its lips (linear), and the gilt of its ring and the grid it is skinned
/// from, m.
const LIP: (f32, f32, f32) = (0.6, 0.36, 0.3);
const GILT: (f32, f32, f32) = (0.85, 0.6, 0.28);
const RING_GRAIN: f32 = 0.0045;

/// The inside of its mouth (linear): raw at its lips, dark down its
/// gullet; how far forward its lips are, in the frame half dropped, and
/// how far behind them the dark is, m.
const MOUTH_RAW: (f32, f32, f32) = (0.42, 0.1, 0.09);
const MOUTH_DEEP: (f32, f32, f32) = (0.12, 0.016, 0.02);
const LIPS_FRONT: f32 = 0.2;
const MOUTH_DEPTH: f32 = 0.2;
/// Where the flesh of its mouth shows: how deep inside its skin it starts
/// and where it is all flesh, and how near the walls of the slit and
/// pocket cut between its lips, m.
const FLESH: (f32, f32) = (0.012, 0.03);
const WALLS: (f32, f32) = (0.008, 0.016);

/// The frame of its mouth: the middle of its corners, and the turn from
/// the body's axes to the mouth's.
fn mouth_frame() -> (Vector3<f32>, UnitQuaternion<f32>) {
    let [left, right] = MOUTH_CORNERS.map(point);
    let turn = UnitQuaternion::rotation_between(&Vector3::x(), &(right - left))
        .unwrap_or_else(UnitQuaternion::identity);
    ((left + right) * 0.5, turn)
}

/// The frame of its mouth with its jaw dropped `gape` radians.
fn dropped_frame(gape: f32) -> (Vector3<f32>, UnitQuaternion<f32>) {
    let (middle, turn) = mouth_frame();
    (
        middle,
        turn * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), gape),
    )
}

/// An ellipsoid laid out in the frame of its mouth with its jaw dropped
/// `gape` radians.
fn in_mouth((radii, at): ((f32, f32, f32), (f32, f32, f32)), gape: f32) -> Ellipsoid {
    let (middle, turn) = dropped_frame(gape);
    Ellipsoid {
        radii: point(radii),
        centre: middle + turn * point(at),
        turn,
    }
}

/// `p` carried with its jaw as it is sculpted, dropped.
fn dropped(p: Vector3<f32>) -> Vector3<f32> {
    let (middle, turn) = mouth_frame();
    let (_, drop) = dropped_frame(REST_GAPE);
    middle + drop * turn.inverse() * (p - middle)
}

/// Its muzzle, in the frame it is sculpted in.
pub(super) fn muzzle_region() -> Ellipsoid {
    in_mouth(MUZZLE, 0.0)
}

/// The line of a lip pouting through `pout`, `t` from its −X corner (0)
/// to its +X corner (1): a curve through both and the pout.
fn lip_line(pout: Vector3<f32>, t: f32) -> Vector3<f32> {
    let [left, right] = MOUTH_CORNERS.map(point);
    let (a, b) = (left.lerp(&pout, t), pout.lerp(&right, t));
    a.lerp(&b, t)
}

/// A lip from corner to corner of its mouth, pouting through `pout`, as
/// limbs, each end carried by `carry`.
fn lip_limbs(
    (pout, thick): ((f32, f32, f32), f32),
    carry: impl Fn(Vector3<f32>) -> Vector3<f32>,
) -> Vec<Lump> {
    let pout = point(pout);
    let joints = 8;
    let at = |t: f32| carry(lip_line(pout, t));
    let girth = |t: f32| thick * (0.55 + 0.45 * (PI * t).sin());
    (0..joints)
        .map(|joint| {
            let (from, to) = (
                joint as f32 / joints as f32,
                (joint + 1) as f32 / joints as f32,
            );
            Lump::Limb {
                from: at(from),
                to: at(to),
                radii: (girth(from), girth(to)),
            }
        })
        .collect()
}

/// `clay` with its upper lip added.
pub(super) fn upper_lip(clay: Clay) -> Clay {
    lip_limbs(UPPER_LIP, |p| p)
        .into_iter()
        .fold(clay, |clay, limb| clay.add(limb, LIP))
}

/// `clay` with its lower lip added, dropped with its jaw.
fn lower_lip(clay: Clay) -> Clay {
    lip_limbs(LOWER_LIP, dropped)
        .into_iter()
        .fold(clay, |clay, limb| clay.add(limb, LIP))
}

/// Both its lips as sculpted, for the paint that leaves them be.
pub(super) fn lips() -> Clay {
    lower_lip(upper_lip(Clay::new(0.0)))
}

/// Its body with its mouth in it: its chin smoothed into its head, its
/// lower lip on its chin, the slit between its lips, and the pocket behind
/// them.
fn muzzle_form() -> Clay {
    let clay = carved()
        .smoothing(0.04)
        .add(Lump::Ball(in_mouth(CHIN, REST_GAPE)), SAND)
        .smoothing(0.012);
    let between = REST_GAPE * 0.5;
    lower_lip(clay)
        .cut(Lump::Ball(in_mouth(SLIT, between)), 0.012)
        .cut(Lump::Ball(in_mouth(POCKET, between)), 0.03)
}

/// How much of its jaw's turn the skin at `at` (as sculpted) takes: all
/// of it below the line its lips part along and ahead of the hinge, none
/// above it, fading toward its throat, past its corners, and to nothing
/// well inside the edge of its muzzle.
fn share(at: Vector3<f32>) -> f32 {
    let (middle, half) = dropped_frame(REST_GAPE * 0.5);
    let local = half.inverse() * (at - middle);
    let below = 1.0 - smooth(-PARTING, PARTING, local.y);
    let ahead = smooth(-THROAT.1, -THROAT.0, local.z);
    let between = 1.0 - smooth(CHEEKS.0, CHEEKS.1, local.x.abs());
    let muzzle = muzzle_region();
    let out = (muzzle.turn.inverse() * (at - muzzle.centre))
        .component_div(&muzzle.radii)
        .norm();
    let within = 1.0 - smooth(EDGE.0, EDGE.1, out);
    below * ahead * between * within
}

/// What tells the inside of its mouth from its skin.
struct Inside {
    /// Its body without its mouth.
    whole: Clay,
    /// Its muzzle, beyond which nothing is mouth.
    muzzle: Clay,
    /// What is cut out between its lips and behind them.
    cut: Clay,
}

impl Inside {
    fn new() -> Self {
        let between = REST_GAPE * 0.5;
        Self {
            whole: carved(),
            muzzle: Clay::new(0.0).add(Lump::Ball(muzzle_region()), SAND),
            cut: Clay::new(0.0)
                .add(Lump::Ball(in_mouth(SLIT, between)), SAND)
                .add(Lump::Ball(in_mouth(POCKET, between)), SAND),
        }
    }

    /// How much `at` is of the flesh of its mouth, 0 to 1: whatever lies
    /// inside its body, and the walls of what was cut between its lips.
    fn flesh(&self, at: Vector3<f32>) -> f32 {
        if self.muzzle.distance(at) > -FLESH.0 {
            return 0.0;
        }
        let inside = smooth(FLESH.0, FLESH.1, -self.whole.distance(at));
        let walls = 1.0 - smooth(WALLS.0, WALLS.1, self.cut.distance(at).abs());
        inside.max(walls)
    }

    /// Paint the inside of its mouth on `mesh`, raw at its lips and dark
    /// down its gullet.
    fn paint(&self, mesh: &mut MeshPrimitive) {
        let (raw, deep) = (point(MOUTH_RAW), point(MOUTH_DEEP));
        let (middle, half) = dropped_frame(REST_GAPE * 0.5);
        for vertex in &mut mesh.vertices {
            let shown = self.flesh(vertex.pos);
            if shown > 0.0 {
                let behind = LIPS_FRONT - (half.inverse() * (vertex.pos - middle)).z;
                let flesh = raw.lerp(&deep, (behind / MOUTH_DEPTH).clamp(0.0, 1.0).sqrt());
                let colour = vertex.color.xyz().lerp(&flesh, shown);
                vertex.color.x = colour.x;
                vertex.color.y = colour.y;
                vertex.color.z = colour.z;
            }
        }
    }

    /// Which vertices of `mesh` are mostly the flesh of its mouth, in order.
    fn of(&self, mesh: &MeshPrimitive) -> Vec<bool> {
        mesh.vertices
            .iter()
            .map(|vertex| self.flesh(vertex.pos) > 0.5)
            .collect()
    }
}

/// The gilt ring through its lower lip, hung from it as sculpted.
fn ring() -> MeshPrimitive {
    let (radius, joints) = (0.075, 20);
    let middle = dropped(point(LOWER_LIP.0)) - Vector3::new(0.0, radius, 0.02);
    let at = |j: usize| {
        let angle = j as f32 / joints as f32 * TAU;
        middle + Vector3::new(0.0, angle.cos(), angle.sin()) * radius
    };
    let clay = (0..joints).fold(Clay::new(0.004), |clay, j| {
        clay.add(
            Lump::Limb {
                from: at(j),
                to: at(j + 1),
                radii: (0.014, 0.014),
            },
            GILT,
        )
    });
    let mut mesh = clay.skin(RING_GRAIN, Paint::Bronze.id());
    Mottle {
        frequency: 40.0,
        value: 0.2,
        warmth: 0.15,
        seed: 206,
    }
    .paint(&mut mesh);
    placed(mesh)
}

/// Its muzzle in its body's frame, against the palette's slots: its lips,
/// chin and the inside of its mouth, laid over its body, and the ring
/// through its lip, which turns whole with it; hinged across the corners of
/// its mouth.
pub fn muzzle() -> Jaw {
    let inside = Inside::new();
    let skin = patch(muzzle_form(), muzzle_region(), |mesh| {
        paint_skin(mesh);
        inside.paint(mesh);
        CREVICES.paint(mesh);
    });
    let inside = inside.of(&skin);
    let mut meshes = dressed(&skin, &inside);
    let centre = Deck::get().centre;
    let mut shares: Vec<Vec<f32>> = meshes
        .iter()
        .map(|mesh| {
            mesh.vertices
                .iter()
                .map(|vertex| share(vertex.pos + centre))
                .collect()
        })
        .collect();
    let ring = ring();
    shares.push(vec![1.0; ring.vertices.len()]);
    meshes.push(ring);

    let [left, right] = MOUTH_CORNERS.map(point);
    let (middle, _) = mouth_frame();
    Jaw::new(
        meshes,
        shares,
        Deck::get().placed(middle),
        (right - left).normalize(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn its_lips_part_but_meet_at_the_corners_and_behind_them_it_is_hollow() {
        let form = muzzle_form();
        let solid = |at: Vector3<f32>| form.distance(at) < 0.0;
        let lower = |t: f32| dropped(lip_line(point(LOWER_LIP.0), t));
        let between = |t: f32| (lip_line(point(UPPER_LIP.0), t) + lower(t)) * 0.5;
        for t in [0.35, 0.5, 0.65] {
            assert!(!solid(between(t)), "its lips meet at {t}");
        }
        for t in [0.03, 0.97] {
            assert!(solid(between(t)), "its mouth is open at its corner {t}");
        }
        let (middle, turn) = dropped_frame(REST_GAPE * 0.5);
        let pocket = middle + turn * Vector3::new(0.0, 0.0, -0.04);
        assert!(!solid(pocket), "its mouth is solid behind its lips");
    }

    #[test]
    fn its_lower_lip_and_chin_turn_whole_and_its_upper_lip_and_edge_not_at_all() {
        let lower = dropped(lip_line(point(LOWER_LIP.0), 0.5));
        let upper = lip_line(point(UPPER_LIP.0), 0.5);
        assert!(share(lower) > 0.99, "its lower lip takes {}", share(lower));
        assert!(share(upper) < 0.01, "its upper lip takes {}", share(upper));
        let chin = in_mouth(CHIN, REST_GAPE);
        let under = chin.centre - chin.turn * Vector3::y() * chin.radii.y;
        assert!(share(under) > 0.99, "its chin takes {}", share(under));
        let muzzle = muzzle_region();
        for way in [
            Vector3::x(),
            -Vector3::x(),
            Vector3::z(),
            -Vector3::z(),
            -Vector3::y(),
        ] {
            let edge = muzzle.centre + muzzle.turn * way.component_mul(&muzzle.radii);
            assert_eq!(share(edge), 0.0, "its muzzle's edge moves at {way:?}");
        }
    }

    #[test]
    fn opened_wide_its_lip_drops_and_its_edge_stays_on_its_body() {
        let jaw = muzzle();
        let mut shaped = jaw.rest().to_vec();
        jaw.shape(0.5, &mut shaped);
        let deck = Deck::get();
        let lip = deck.placed(dropped(point(LOWER_LIP.0)));
        let muzzle = muzzle_region();
        let mut dropped_lip = false;
        for (rest, shaped) in jaw.rest().iter().zip(&shaped) {
            for (before, after) in rest.vertices.iter().zip(&shaped.vertices) {
                let at = before.pos + deck.centre;
                let out = (muzzle.turn.inverse() * (at - muzzle.centre))
                    .component_div(&muzzle.radii)
                    .norm();
                if out > EDGE.1 {
                    assert!((after.pos - before.pos).norm() < 1e-6, "its edge moved");
                }
                if (before.pos - lip).norm() < 0.1 && after.pos.y < before.pos.y - 0.04 {
                    dropped_lip = true;
                }
            }
        }
        assert!(dropped_lip, "its lower lip did not drop");
    }
}
