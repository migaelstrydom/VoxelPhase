//! `Path` — a walkable deck swept along a polyline.
//!
//! The workhorse primitive: catwalks, bridges, ledges cut into a cliff, and —
//! because every waypoint carries its own height — spiral ramps around a tower
//! as a single feature rather than a construction of many.

use nalgebra::Point3;

use crate::collision::AABB;
use crate::level::PathProfile;

use super::solid::{above, below, Sample, TraversalSolid};

/// A deck of constant width swept along a polyline.
///
/// ```text
///   plan                              cross-section
///                                       Flat            Rounded
///   p0 ●────────────● p1              ┌────────┐      ╭────────╮
///                    ╲                │  deck  │      ╰──────╯
///                     ● p2            └────────┘
///                                     ▲ surface at the authored y
/// ```
///
/// A waypoint's `y` is the **walking surface**, not the middle of the deck, so
/// the top lands exactly on the authored height and the thickness hangs below
/// it. That is the difference between authoring "the bridge is at 6 m" and
/// having to remember to add half a deck.
///
/// Each leg is swept independently and the results unioned. A leg's ends are
/// capped with a half-round of the deck's own half-width, which is also what
/// carries the outside of a corner: two legs meeting at a waypoint overlap in
/// a rounded joint without the author placing anything there. The cost is that
/// the two open ends of the whole path overhang their waypoints by a
/// half-width, so author the endpoints where the deck should meet what it
/// joins, not half a deck short of it.
/// How much of a rounded deck's half-thickness goes into its edge radius.
const ROUNDED_EDGE_FRACTION: f32 = 0.6;

pub struct PathSolid {
    /// Waypoints in segment-local coordinates; `y` is the walking surface.
    points: Vec<Point3<f32>>,
    /// Half the authored deck width.
    half_width: f32,
    /// How far the deck extends below its walking surface.
    thickness: f32,
    /// Cross-section shape.
    profile: PathProfile,
}

impl PathSolid {
    /// Build a path, or `None` if it describes nothing: fewer than two
    /// waypoints, or a non-positive width or thickness.
    pub fn new(
        points: &[(f32, f32, f32)],
        width: f32,
        thickness: f32,
        profile: PathProfile,
    ) -> Option<Self> {
        if points.len() < 2 || width <= 0.0 || thickness <= 0.0 {
            return None;
        }
        Some(Self {
            points: points.iter().map(|p| Point3::new(p.0, p.1, p.2)).collect(),
            half_width: width * 0.5,
            thickness,
            profile,
        })
    }

    /// Radius the deck's long edges are rounded to.
    ///
    /// A fraction rather than the full half-extent, so a rounded deck keeps a
    /// flat strip down the middle to walk on instead of becoming a log.
    fn corner_radius(&self) -> f32 {
        self.half_width.min(self.thickness * 0.5) * ROUNDED_EDGE_FRACTION
    }

    /// Signed distance within one leg's cross-section.
    ///
    /// `lateral` is the unsigned distance from the leg's centreline in the
    /// horizontal plane; `v` is the height above the walking surface, so `v`
    /// is negative inside the deck.
    fn cross_section(&self, lateral: f32, v: f32) -> f32 {
        match self.profile {
            // A box, in the max-of-half-spaces form: exact on each face, which
            // is all the density encoding reads.
            PathProfile::Flat => (lateral - self.half_width)
                .max(below(v, 0.0))
                .max(above(v, -self.thickness)),

            // The same deck with its four long edges rounded off: a stone
            // bridge, or a ledge weathered out of a cliff. This is the exact
            // rounded-box distance, not a shaped approximation, and that
            // matters — a profile built by intersecting an ellipse with a
            // half-space meets the top plane tangentially, and a knife edge
            // that fine is where marching cubes cracks.
            PathProfile::Rounded => {
                let r = self.corner_radius();
                let qu = lateral - (self.half_width - r);
                let qv = (v + self.thickness * 0.5).abs() - (self.thickness * 0.5 - r);
                let outside = (qu.max(0.0).powi(2) + qv.max(0.0).powi(2)).sqrt();
                outside + qu.max(qv).min(0.0) - r
            }
        }
    }
}

impl TraversalSolid for PathSolid {
    fn sample(&self, p: Point3<f32>) -> Sample {
        let mut distance = f32::INFINITY;
        let mut surface_y = p.y;

        for leg in self.points.windows(2) {
            let (a, b) = (leg[0], leg[1]);
            let (t, lateral) = project_horizontal(p, a, b);
            let top = a.y + (b.y - a.y) * t;
            let d = self.cross_section(lateral, p.y - top);
            if d < distance {
                distance = d;
                surface_y = top;
            }
        }

        Sample {
            distance,
            surface_y,
        }
    }

    fn bounds(&self, voxel_size: f32) -> AABB {
        let pad = self.half_width + voxel_size;
        let mut min = self.points[0];
        let mut max = self.points[0];
        for p in &self.points[1..] {
            min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
        AABB::new(
            Point3::new(
                min.x - pad,
                min.y - self.thickness - voxel_size,
                min.z - pad,
            ),
            Point3::new(max.x + pad, max.y + voxel_size, max.z + pad),
        )
    }
}

/// Project `p` onto the horizontal projection of the leg `a`→`b`.
///
/// Returns the parameter along the leg, clamped to `[0, 1]`, and the horizontal
/// distance from the clamped point. Clamping is what produces the half-round
/// end caps, and therefore the rounded outside of every corner.
fn project_horizontal(p: Point3<f32>, a: Point3<f32>, b: Point3<f32>) -> (f32, f32) {
    let (abx, abz) = (b.x - a.x, b.z - a.z);
    let len_sq = abx * abx + abz * abz;
    if len_sq < 1e-10 {
        let (dx, dz) = (p.x - a.x, p.z - a.z);
        return (0.0, (dx * dx + dz * dz).sqrt());
    }
    let t = (((p.x - a.x) * abx + (p.z - a.z) * abz) / len_sq).clamp(0.0, 1.0);
    let (dx, dz) = (p.x - (a.x + t * abx), p.z - (a.z + t * abz));
    (t, (dx * dx + dz * dz).sqrt())
}
