//! The area a body shows along a direction: the union of its envelope's
//! parts' outlines, projected across the view.
//!
//! ```text
//!   EnvelopePart (shape, offset)
//!        │  projected onto the plane across the view
//!        ▼
//!   one convex polygon per part
//!        │  slabs between the polygons' vertex heights
//!        ▼
//!   each slab's covered width (the union of the polygons' spans across
//!   its middle) × its height, summed
//! ```
//!
//! Measured as a union, a body shows the same area however it is cut up, a
//! gap between its parts lets the view through, and a part behind another
//! adds nothing. Within a slab every polygon's span changes linearly with
//! height, so the middle row gives the slab's area exactly unless two spans'
//! ends cross inside it. A body of many parts has too many vertex heights to
//! slab at each: past [`MAX_SLABS`] the slabs are even rows instead, and a
//! row holding a vertex is off by a sliver. Round shapes are drawn as
//! polygons circumscribing them, a fraction of a percent over.
//!
//! What a blast strikes ([`struck_area`]) weighs that union by the parts'
//! solidities: each solidity among them times the union of its parts,
//! summed, and never more than the whole union.

use std::f32::consts::{PI, TAU};
use std::ops::Range;

use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;

/// Directions the share a body's parts show is averaged over: enough that it
/// comes within a percent of its value over hundreds, for cut cubes and
/// boulders, crosses, dumbbells and Ls.
pub const MEAN_DIRECTIONS: usize = 16;
/// Points round each projected circle (a sphere's, a capsule's ends).
const CIRCLE_POINTS: usize = 24;
/// Most slabs one silhouette is measured in: what bounds its cost for a
/// body of many parts. A compound of 64 boxes measures the share it shows
/// in about half a millisecond, within a tenth of a percent of a quarter.
pub const MAX_SLABS: usize = 64;

/// The area a body whose envelope is `parts` shows to `view`, a unit
/// direction in the body's frame: what a blast travelling along it strikes.
pub fn exposed_area<'a>(
    parts: impl IntoIterator<Item = EnvelopePart<'a>>,
    view: Vector3<f32>,
) -> f32 {
    let parts: SmallVec<[EnvelopePart; 8]> = parts.into_iter().collect();
    match parts.as_slice() {
        [] => 0.0,
        [part] => part
            .shape
            .projected_area(part.offset.rotation.inverse() * view),
        _ => Silhouette::default().area(&parts, view),
    }
}

/// The area of a blast's loading the body whose envelope is `parts` catches
/// along `view`, a unit direction in the body's frame: for each solidity
/// among the parts, that solidity times the union of those parts'
/// silhouettes, summed, and capped at the whole union's area. Parts that
/// share one solidity catch that share of their union.
pub fn struck_area<'a>(
    parts: impl IntoIterator<Item = EnvelopePart<'a>>,
    view: Vector3<f32>,
) -> f32 {
    let parts: SmallVec<[EnvelopePart; 8]> = parts.into_iter().collect();
    let mut solidities: SmallVec<[f32; 2]> = SmallVec::new();
    for part in &parts {
        if !solidities.contains(&part.solidity) {
            solidities.push(part.solidity);
        }
    }
    match solidities.as_slice() {
        [] => 0.0,
        [solidity] => solidity * exposed_area(parts, view),
        _ => {
            let caught: f32 = solidities
                .iter()
                .map(|&solidity| {
                    let alike = parts.iter().filter(|p| p.solidity == solidity).copied();
                    solidity * exposed_area(alike, view)
                })
                .sum();
            caught.min(exposed_area(parts, view))
        }
    }
}

/// Measures silhouettes, keeping its scratch between measurements.
#[derive(Debug, Default)]
pub struct Silhouette {
    /// One part's outline points, before they are reduced to its hull.
    points: Vec<Vector2<f32>>,
    /// Every part's convex outline, end to end.
    vertices: Vec<Vector2<f32>>,
    /// Each part's outline: its range in `vertices` and its height span.
    polygons: Vec<Polygon>,
    /// Heights the slabs break at: the polygons' vertices', or even rows
    /// past [`MAX_SLABS`].
    heights: Vec<f32>,
    /// The polygons in the order the slabs reach them, lowest first.
    rising: Vec<usize>,
    /// The polygons the current slab's middle crosses, each walking up its
    /// two sides.
    active: Vec<Climb>,
    /// The spans across one slab's middle.
    spans: Vec<(f32, f32)>,
}

/// One part's convex outline in a [`Silhouette`].
#[derive(Debug)]
struct Polygon {
    /// Its vertices' range in [`Silhouette::vertices`].
    vertices: Range<usize>,
    /// Its lowest height.
    low: f32,
    /// Its highest height.
    high: f32,
    /// Where its lowest vertex is in its own outline.
    lowest: usize,
}

/// A polygon a rising row crosses, and the vertex below the row on each of
/// its two sides: walking up from its lowest vertex one way round, and the
/// other way round. A convex outline's sides only climb, so the walk only
/// moves on.
#[derive(Debug, Clone, Copy)]
struct Climb {
    /// The polygon, in [`Silhouette::polygons`].
    polygon: usize,
    /// The vertex below the row going forwards round the outline.
    forwards: usize,
    /// The vertex below the row going backwards round the outline.
    backwards: usize,
}

impl Silhouette {
    /// The area `parts` show to `view`, a unit direction in their frame.
    pub fn area(&mut self, parts: &[EnvelopePart], view: Vector3<f32>) -> f32 {
        self.project(parts, view);
        match self.polygons.as_slice() {
            [] => 0.0,
            [one] => polygon_area(&self.vertices[one.vertices.clone()]),
            _ => self.union_area(),
        }
    }

    /// The share of their own silhouettes `parts` show together, over
    /// [`MEAN_DIRECTIONS`] directions spread evenly over a hemisphere (a view
    /// and its opposite show the same): the union's area over the sum of the
    /// parts' outlines, each summed over the same directions and drawn the
    /// same way, so the directions' sampling and the drawing of round shapes
    /// cancel where the parts hide each other alike from every side.
    pub fn mean_shown(&mut self, parts: &[EnvelopePart]) -> f32 {
        let golden = PI * (3.0 - 5.0f32.sqrt());
        let (mut union, mut each) = (0.0, 0.0);
        for i in 0..MEAN_DIRECTIONS {
            let z = 1.0 - (i as f32 + 0.5) / MEAN_DIRECTIONS as f32;
            let ring = (1.0 - z * z).sqrt();
            let azimuth = golden * i as f32;
            let view = Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z);
            union += self.area(parts, view);
            each += self
                .polygons
                .iter()
                .map(|polygon| polygon_area(&self.vertices[polygon.vertices.clone()]))
                .sum::<f32>();
        }
        if each > 0.0 {
            union / each
        } else {
            0.0
        }
    }

    /// Each part's convex outline seen along `view`.
    fn project(&mut self, parts: &[EnvelopePart], view: Vector3<f32>) {
        let across = view
            .cross(&Vector3::x())
            .try_normalize(1e-6)
            .unwrap_or_else(|| view.cross(&Vector3::y()).normalize());
        let up = view.cross(&across);
        self.vertices.clear();
        self.polygons.clear();
        for part in parts {
            self.points.clear();
            outline(part.shape, &part.offset, &across, &up, &mut self.points);
            let start = self.vertices.len();
            convex_hull(&mut self.points, &mut self.vertices);
            let hull = &self.vertices[start..];
            if hull.len() < 3 {
                self.vertices.truncate(start);
                continue;
            }
            let lowest = (0..hull.len())
                .min_by(|&a, &b| hull[a].y.total_cmp(&hull[b].y))
                .unwrap_or(0);
            let high = hull.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
            self.polygons.push(Polygon {
                vertices: start..self.vertices.len(),
                low: hull[lowest].y,
                high,
                lowest,
            });
        }
    }

    /// The area of the union of the projected polygons, slab by slab.
    fn union_area(&mut self) -> f32 {
        self.heights.clear();
        self.heights.extend(self.vertices.iter().map(|v| v.y));
        self.heights.sort_unstable_by(f32::total_cmp);
        self.heights.dedup();
        if self.heights.len() > MAX_SLABS + 1 {
            let (lowest, highest) = (self.heights[0], self.heights[self.heights.len() - 1]);
            let row = (highest - lowest) / MAX_SLABS as f32;
            self.heights.clear();
            self.heights
                .extend((0..=MAX_SLABS).map(|i| lowest + row * i as f32));
        }
        self.rising.clear();
        self.rising.extend(0..self.polygons.len());
        let polygons = &self.polygons;
        self.rising
            .sort_unstable_by(|&a, &b| polygons[a].low.total_cmp(&polygons[b].low));
        self.active.clear();
        let mut reached = 0;
        let mut area = 0.0;
        for slab in self.heights.windows(2) {
            let (low, high) = (slab[0], slab[1]);
            let middle = 0.5 * (low + high);
            while let Some(&next) = self.rising.get(reached) {
                let polygon = &polygons[next];
                if polygon.low >= middle {
                    break;
                }
                self.active.push(Climb {
                    polygon: next,
                    forwards: polygon.lowest,
                    backwards: polygon.lowest,
                });
                reached += 1;
            }
            self.active
                .retain(|climb| polygons[climb.polygon].high > middle);
            self.spans.clear();
            for climb in &mut self.active {
                let outline = &self.vertices[polygons[climb.polygon].vertices.clone()];
                self.spans.push(climb.span_at(outline, middle));
            }
            area += covered_width(&mut self.spans) * (high - low);
        }
        area
    }
}

impl Climb {
    /// The polygon's span across `height`, which is above its lowest vertex
    /// and below its highest, and no lower than the last height asked.
    fn span_at(&mut self, outline: &[Vector2<f32>], height: f32) -> (f32, f32) {
        let n = outline.len();
        let forwards = climb(outline, &mut self.forwards, height, 1);
        let backwards = climb(outline, &mut self.backwards, height, n - 1);
        (forwards.min(backwards), forwards.max(backwards))
    }
}

/// Walks `below` round `outline` by `step` until the next vertex is at or
/// above `height`, and gives where the side between them crosses it.
fn climb(outline: &[Vector2<f32>], below: &mut usize, height: f32, step: usize) -> f32 {
    let n = outline.len();
    let mut above = (*below + step) % n;
    while outline[above].y < height {
        *below = above;
        above = (above + step) % n;
    }
    let (a, b) = (outline[*below], outline[above]);
    let rise = b.y - a.y;
    if rise > 0.0 {
        a.x + (height - a.y) * (b.x - a.x) / rise
    } else {
        a.x
    }
}

/// The points of `shape`'s outline at `offset`, projected onto the plane
/// spanned by `across` and `up`.
fn outline(
    shape: &ColliderShape,
    offset: &Isometry3<f32>,
    across: &Vector3<f32>,
    up: &Vector3<f32>,
    out: &mut Vec<Vector2<f32>>,
) {
    let project = |p: Point3<f32>| Vector2::new(p.coords.dot(across), p.coords.dot(up));
    // Out to the circumscribed polygon, so its area is the circle's to
    // within a fraction of a percent, not under it.
    let circumscribed = 1.0 / (PI / CIRCLE_POINTS as f32).cos();
    let mut circle = |centre: Point3<f32>, radius: f32| {
        let c = project(centre);
        out.extend((0..CIRCLE_POINTS).map(|k| {
            let angle = TAU * k as f32 / CIRCLE_POINTS as f32;
            c + Vector2::new(angle.cos(), angle.sin()) * radius * circumscribed
        }));
    };
    match shape {
        ColliderShape::Sphere { radius } => circle(offset * Point3::origin(), *radius),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let half = (half_height - radius).max(0.0);
            circle(offset * Point3::new(0.0, half, 0.0), *radius);
            circle(offset * Point3::new(0.0, -half, 0.0), *radius);
        }
        ColliderShape::Box { half_extents } => {
            for corner in 0..8 {
                let sign = |bit: usize| if corner & bit == 0 { -1.0 } else { 1.0 };
                let local = Point3::new(
                    sign(1) * half_extents.x,
                    sign(2) * half_extents.y,
                    sign(4) * half_extents.z,
                );
                out.push(project(offset * local));
            }
        }
        ColliderShape::ConvexHull { hull } => {
            out.extend(
                hull.vertices
                    .iter()
                    .map(|v| project(offset * Point3::from(*v))),
            );
        }
    }
}

/// Appends the convex hull of `points` to `out`, counter-clockwise
/// (Andrew's monotone chain). Reorders `points`.
fn convex_hull(points: &mut [Vector2<f32>], out: &mut Vec<Vector2<f32>>) {
    points.sort_unstable_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let lower = out.len();
    for &p in points.iter() {
        push_turning_left(out, lower, p);
    }
    out.pop();
    let upper = out.len();
    for &p in points.iter().rev() {
        push_turning_left(out, upper, p);
    }
    out.pop();
}

/// Pushes `p` onto the chain that starts at `out[from]`, first dropping the
/// points it would leave on a right turn or a straight line.
fn push_turning_left(out: &mut Vec<Vector2<f32>>, from: usize, p: Vector2<f32>) {
    while out.len() >= from + 2 {
        let (o, a) = (out[out.len() - 2], out[out.len() - 1]);
        if (a - o).perp(&(p - o)) > 0.0 {
            break;
        }
        out.pop();
    }
    out.push(p);
}

/// The area of the polygon `outline` (the shoelace formula).
fn polygon_area(outline: &[Vector2<f32>]) -> f32 {
    let n = outline.len();
    let twice: f32 = (0..n).map(|i| outline[i].perp(&outline[(i + 1) % n])).sum();
    0.5 * twice.abs()
}

/// The width `spans` cover together, counting overlaps once. Reorders them.
fn covered_width(spans: &mut [(f32, f32)]) -> f32 {
    spans.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    let mut width = 0.0;
    let mut reached = f32::NEG_INFINITY;
    for &(start, end) in spans.iter() {
        let from = start.max(reached);
        if end > from {
            width += end - from;
            reached = end;
        }
    }
    width
}

#[cfg(test)]
mod tests {
    use super::*;

    fn part<'a>(shape: &'a ColliderShape, offset: &Isometry3<f32>) -> EnvelopePart<'a> {
        porous(shape, offset, 1.0)
    }

    fn porous<'a>(
        shape: &'a ColliderShape,
        offset: &Isometry3<f32>,
        solidity: f32,
    ) -> EnvelopePart<'a> {
        EnvelopePart {
            shape,
            offset: *offset,
            drag_coefficient: None,
            solidity,
        }
    }

    fn block(half: f32) -> ColliderShape {
        ColliderShape::Box {
            half_extents: Vector3::repeat(half),
        }
    }

    /// Four squares tiling a square show the square they tile, and two
    /// apart show the two of them, not the hull round both.
    #[test]
    fn a_silhouette_is_the_union_of_its_parts() {
        let quarter = block(0.25);
        let tiled: Vec<Isometry3<f32>> =
            [(-0.25, -0.25), (-0.25, 0.25), (0.25, -0.25), (0.25, 0.25)]
                .iter()
                .map(|&(x, y)| Isometry3::translation(x, y, 0.0))
                .collect();
        let parts: Vec<EnvelopePart> = tiled.iter().map(|o| part(&quarter, o)).collect();
        let tiled_area = Silhouette::default().area(&parts, Vector3::z());
        assert!((tiled_area - 1.0).abs() < 1e-5, "tiled {tiled_area}");

        let apart = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let parts: Vec<EnvelopePart> = apart.iter().map(|o| part(&quarter, o)).collect();
        let apart_area = Silhouette::default().area(&parts, Vector3::z());
        assert!((apart_area - 0.5).abs() < 1e-5, "apart {apart_area}");
    }

    /// Two squares overlapping by half show one and a half squares.
    #[test]
    fn overlapping_parts_count_once() {
        let unit = block(0.5);
        let offsets = [
            Isometry3::translation(0.0, 0.0, 0.0),
            Isometry3::translation(0.5, 0.0, 0.3),
        ];
        let parts: Vec<EnvelopePart> = offsets.iter().map(|o| part(&unit, o)).collect();
        let area = Silhouette::default().area(&parts, Vector3::z());
        assert!((area - 1.5).abs() < 1e-5, "{area}");
    }

    /// Two balls apart show two discs from the side; end on, one hides the
    /// other.
    #[test]
    fn a_part_behind_another_adds_nothing() {
        let ball = ColliderShape::Sphere { radius: 0.1 };
        let offsets = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let parts: Vec<EnvelopePart> = offsets.iter().map(|o| part(&ball, o)).collect();
        let disc = PI * 0.01;
        let side = Silhouette::default().area(&parts, Vector3::z());
        let end_on = Silhouette::default().area(&parts, Vector3::x());
        // The circles are drawn circumscribed, about half a percent over.
        assert!((side - 2.0 * disc).abs() < 0.02 * disc, "side {side}");
        assert!((end_on - disc).abs() < 0.01 * disc, "end on {end_on}");
    }

    /// Parts of one solidity catch that share of their union, as a body of
    /// one solidity always has.
    #[test]
    fn a_uniformly_porous_body_catches_its_share_of_its_union() {
        let unit = block(0.5);
        let offsets = [
            Isometry3::translation(0.0, 0.0, 0.0),
            Isometry3::translation(0.5, 0.0, 0.3),
        ];
        let parts: Vec<EnvelopePart> = offsets.iter().map(|o| porous(&unit, o, 0.3)).collect();
        let union = exposed_area(parts.iter().copied(), Vector3::z());
        let struck = struck_area(parts, Vector3::z());
        assert_eq!(struck, 0.3 * union);
    }

    /// Apart, a solid square and a porous one each catch their own share;
    /// overlapping wholly, the two shares together are capped at the one
    /// square they show.
    #[test]
    fn mixed_solidities_sum_their_unions_up_to_the_whole() {
        let unit = block(0.5);
        let apart = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let parts = [porous(&unit, &apart[0], 1.0), porous(&unit, &apart[1], 0.3)];
        let struck = struck_area(parts, Vector3::z());
        assert!((struck - 1.3).abs() < 1e-5, "apart {struck}");

        let together = Isometry3::identity();
        let parts = [porous(&unit, &together, 1.0), porous(&unit, &together, 0.3)];
        let struck = struck_area(parts, Vector3::z());
        assert!((struck - 1.0).abs() < 1e-5, "together {struck}");

        let half = [
            Isometry3::translation(0.0, 0.0, 0.0),
            Isometry3::translation(0.5, 0.0, 0.3),
        ];
        let parts = [porous(&unit, &half[0], 0.5), porous(&unit, &half[1], 0.2)];
        let struck = struck_area(parts, Vector3::z());
        assert!((struck - 0.7).abs() < 1e-5, "half over {struck}");
    }
}
