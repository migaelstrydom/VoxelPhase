//! The blocks a fountain tower is built from: each one collider, set
//! somewhere in the tower's frame, and for those that can shatter the stone
//! they shatter as.

use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::sync::Arc;

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use crate::app::spawnables::shared::models::{build_convex_hull, SolidFace};
use crate::collision::convex_hull::ConvexHull;
use crate::physics::ColliderDesc;

/// Sides round a turned block's hull, and rings from pole to pole of a
/// ball's: enough that a wedge's outline reads as round, and few enough
/// that a ball cut three ways stays well inside a hull's vertex limit.
const SIDES: usize = 14;
const BALL_RINGS: usize = 6;
/// Rings in each rounded end of a rod's hull.
const CAP_RINGS: usize = 3;
/// How much of a disc's half-thickness its upright rim takes; the rest is
/// bevel.
const DISC_RIM: f32 = 0.4;

/// One of the tower's colliders and where it sits in the tower's frame.
pub(super) struct Block {
    pub collider: ColliderDesc,
    pub frame: Isometry3<f32>,
    /// The shape it shatters as, in its own frame; `None` for a block
    /// that never does.
    pub stone: Option<Arc<ConvexHull>>,
}

fn placed(centre: Vector3<f32>) -> Isometry3<f32> {
    Isometry3::from_parts(Translation3::from(centre), UnitQuaternion::identity())
}

impl Block {
    /// A ball that never shatters.
    pub fn ball(radius: f32, centre: Vector3<f32>) -> Self {
        Self {
            collider: ColliderDesc::sphere(radius),
            frame: placed(centre),
            stone: None,
        }
    }

    /// A rod of `radius` from `foot` to `top`, ends rounded, that never
    /// shatters.
    pub fn rod(radius: f32, foot: Vector3<f32>, top: Vector3<f32>) -> Self {
        let turn = UnitQuaternion::rotation_between(&Vector3::y(), &(top - foot))
            .unwrap_or_else(UnitQuaternion::identity);
        Self {
            collider: ColliderDesc::capsule(0.5 * (top - foot).norm(), radius),
            frame: Isometry3::from_parts(Translation3::from(0.5 * (foot + top)), turn),
            stone: None,
        }
    }

    /// A block of stone shaped `hull`, its frame's origin at `centre`.
    fn of_stone(hull: ConvexHull, centre: Vector3<f32>) -> Self {
        let hull = Arc::new(hull);
        Self {
            collider: ColliderDesc::convex_hull(hull.clone()),
            frame: placed(centre),
            stone: Some(hull),
        }
    }

    /// A ball of stone.
    pub fn stone_ball(radius: f32, centre: Vector3<f32>) -> Self {
        let rings: Vec<(f32, f32)> = (1..=BALL_RINGS)
            .map(|ring| {
                let latitude = -FRAC_PI_2 + PI * ring as f32 / (BALL_RINGS + 1) as f32;
                (radius * latitude.sin(), radius * latitude.cos())
            })
            .collect();
        Self::of_stone(turned_hull(-radius, radius, &rings), centre)
    }

    /// An upright rod of stone of `radius`, ends rounded, from height
    /// `foot` to height `top` on the tower's axis.
    pub fn stone_rod(radius: f32, foot: Vector3<f32>, top: Vector3<f32>) -> Self {
        let half = 0.5 * (top.y - foot.y);
        let shoulder = half - radius;
        // Each end's rings, from its pole towards the straight.
        let cap = |sign: f32| {
            (1..=CAP_RINGS).map(move |ring| {
                let latitude = FRAC_PI_2 * ring as f32 / CAP_RINGS as f32;
                (
                    sign * (shoulder + radius * latitude.cos()),
                    radius * latitude.sin(),
                )
            })
        };
        let mut rings: Vec<(f32, f32)> = cap(-1.0).collect();
        let upper: Vec<(f32, f32)> = cap(1.0).collect();
        rings.extend(upper.into_iter().rev());
        Self::of_stone(turned_hull(-half, half, &rings), 0.5 * (foot + top))
    }

    /// A level disc of stone of `radius`, `half_thickness` thick either
    /// side of `centre`, its rim bevelled.
    pub fn stone_disc(radius: f32, half_thickness: f32, centre: Vector3<f32>) -> Self {
        let rim = DISC_RIM * half_thickness;
        let face = radius - (half_thickness - rim);
        Self::of_stone(
            capped_hull(&[
                (-half_thickness, face),
                (-rim, radius),
                (rim, radius),
                (half_thickness, face),
            ]),
            centre,
        )
    }
}

/// Corner `side` of ring `ring` of a turned hull whose rings start at
/// vertex `first`.
fn corner(first: usize, ring: usize, side: usize) -> usize {
    first + ring * SIDES + side % SIDES
}

/// The corners of `rings` of (height, radius), each of [`SIDES`] corners.
fn ring_corners(rings: &[(f32, f32)]) -> impl Iterator<Item = Vector3<f32>> + '_ {
    rings.iter().flat_map(|&(height, radius)| {
        (0..SIDES).map(move |side| {
            let azimuth = TAU * side as f32 / SIDES as f32;
            Vector3::new(radius * azimuth.cos(), height, radius * azimuth.sin())
        })
    })
}

/// The faces between each ring and the next, for rings starting at vertex
/// `first`.
fn ring_faces(first: usize, rings: usize) -> impl Iterator<Item = SolidFace> {
    (0..rings.saturating_sub(1)).flat_map(move |ring| {
        (0..SIDES).map(move |side| SolidFace {
            vertex_indices: vec![
                corner(first, ring, side),
                corner(first, ring, side + 1),
                corner(first, ring + 1, side + 1),
                corner(first, ring + 1, side),
            ],
            opposite_vertex: corner(first, ring, side + SIDES / 2),
        })
    })
}

/// The hull of a shape turned on a lathe about +y and cut off flat at
/// both ends: `rings` of (height, radius) rising, the lowest and highest
/// each a flat face.
fn capped_hull(rings: &[(f32, f32)]) -> ConvexHull {
    let vertices: Vec<Vector3<f32>> = ring_corners(rings).collect();
    let last = rings.len() - 1;
    let mut faces: Vec<SolidFace> = ring_faces(0, rings.len()).collect();
    faces.push(SolidFace {
        vertex_indices: (0..SIDES).map(|side| corner(0, 0, side)).collect(),
        opposite_vertex: corner(0, last, 0),
    });
    faces.push(SolidFace {
        vertex_indices: (0..SIDES).rev().map(|side| corner(0, last, side)).collect(),
        opposite_vertex: corner(0, 0, 0),
    });
    build_convex_hull(&vertices, &faces)
}

/// The hull of a shape turned on a lathe about +y: a pole at `bottom`,
/// `rings` of (height, radius) rising from it, each of [`SIDES`] corners,
/// and a pole at `top`.
fn turned_hull(bottom: f32, top: f32, rings: &[(f32, f32)]) -> ConvexHull {
    let mut vertices = vec![Vector3::new(0.0, bottom, 0.0)];
    vertices.extend(ring_corners(rings));
    let north = vertices.len();
    vertices.push(Vector3::new(0.0, top, 0.0));
    let last = rings.len() - 1;

    let mut faces: Vec<SolidFace> = ring_faces(1, rings.len()).collect();
    for side in 0..SIDES {
        faces.push(SolidFace {
            vertex_indices: vec![0, corner(1, 0, side + 1), corner(1, 0, side)],
            opposite_vertex: north,
        });
        faces.push(SolidFace {
            vertex_indices: vec![north, corner(1, last, side), corner(1, last, side + 1)],
            opposite_vertex: 0,
        });
    }
    build_convex_hull(&vertices, &faces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cleave::CleaveRule;

    fn hull(block: &Block) -> &ConvexHull {
        block.stone.as_ref().unwrap()
    }

    #[test]
    fn a_stone_disc_is_as_thick_and_wide_as_asked() {
        let block = Block::stone_disc(1.2, 0.1, Vector3::zeros());
        let hull = hull(&block);
        assert!((hull.box_half_extents.y - 0.1).abs() < 1.0e-5);
        assert!((hull.box_half_extents.x - 1.2).abs() < 1.0e-5);
        assert_eq!(hull.faces.len(), 3 * SIDES + 2);
    }

    #[test]
    fn a_stone_ball_fills_most_of_its_sphere() {
        let block = Block::stone_ball(1.0, Vector3::zeros());
        let sphere = 4.0 / 3.0 * PI;
        let volume = hull(&block).compute_volume();
        assert!(volume > 0.7 * sphere && volume < sphere, "volume {volume}");
        assert!((hull(&block).bounding_radius - 1.0).abs() < 1.0e-4);
    }

    #[test]
    fn a_stone_rod_reaches_from_foot_to_top() {
        let block = Block::stone_rod(
            0.3,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 3.0, 0.0),
        );
        assert!((block.frame.translation.vector.y - 2.0).abs() < 1.0e-6);
        let (low, high) = hull(&block)
            .vertices
            .iter()
            .fold((f32::MAX, f32::MIN), |(low, high), v| {
                (low.min(v.y), high.max(v.y))
            });
        assert!((low + 1.0).abs() < 1.0e-5 && (high - 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn every_face_of_a_turned_hull_faces_outward() {
        for block in [
            Block::stone_ball(0.7, Vector3::zeros()),
            Block::stone_rod(0.2, Vector3::zeros(), Vector3::y() * 2.0),
            Block::stone_disc(1.2, 0.1, Vector3::zeros()),
        ] {
            let hull = hull(&block);
            for face in &hull.faces {
                let on = hull.vertices[face.vertex_indices[0] as usize];
                assert!(face.normal.dot(&on) > 0.0, "a face looks inward");
            }
        }
    }

    #[test]
    fn a_stone_ball_cleaves_from_every_side() {
        let rule = CleaveRule {
            threshold: 1.0,
            pieces: 3,
            tilt: 18.0,
            min_volume: 0.01,
        };
        let block = Block::stone_ball(0.72, Vector3::zeros());
        let cleaved = (0..200u32)
            .filter(|&k| {
                let (round, up) = (k as f32 * 0.7, k as f32 * 0.37);
                let from = Vector3::new(round.cos() * up.cos(), up.sin(), round.sin() * up.cos());
                rule.cleave(&block.collider.shape, from * 1.5, k).is_some()
            })
            .count();
        assert_eq!(cleaved, 200, "sides it cleaved from");
    }
}
