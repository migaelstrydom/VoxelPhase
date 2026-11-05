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

use std::f32::consts::{PI, TAU};
use std::ops::Range;

use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;

/// Directions a mean silhouette is averaged over: enough that a box's mean
/// comes within a percent of Cauchy's exact quarter-surface.
pub const MEAN_DIRECTIONS: usize = 32;
/// Points round each projected circle (a sphere's, a capsule's ends).
const CIRCLE_POINTS: usize = 24;
/// Most slabs one silhouette is measured in: what bounds its cost for a
/// body of many parts. A compound of 64 boxes measures its mean silhouette
/// in about 1.5 ms, within a percent of the one box they fill.
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

    /// The area `parts` show averaged over [`MEAN_DIRECTIONS`] directions
    /// spread evenly over a hemisphere (a view and its opposite show the
    /// same).
    pub fn mean_area(&mut self, parts: &[EnvelopePart]) -> f32 {
        let golden = PI * (3.0 - 5.0f32.sqrt());
        let mut sum = 0.0;
        for i in 0..MEAN_DIRECTIONS {
            let z = 1.0 - (i as f32 + 0.5) / MEAN_DIRECTIONS as f32;
            let ring = (1.0 - z * z).sqrt();
            let azimuth = golden * i as f32;
            let view = Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z);
            sum += self.area(parts, view);
        }
        sum / MEAN_DIRECTIONS as f32
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
            outline(part.shape, part.offset, &across, &up, &mut self.points);
            let start = self.vertices.len();
            convex_hull(&mut self.points, &mut self.vertices);
            let hull = &self.vertices[start..];
            if hull.len() < 3 {
                self.vertices.truncate(start);
                continue;
            }
            let (low, high) = hull
                .iter()
                .fold((f32::INFINITY, f32::NEG_INFINITY), |(low, high), v| {
                    (low.min(v.y), high.max(v.y))
                });
            self.polygons.push(Polygon {
                vertices: start..self.vertices.len(),
                low,
                high,
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
        let mut area = 0.0;
        for slab in self.heights.windows(2) {
            let (low, high) = (slab[0], slab[1]);
            let middle = 0.5 * (low + high);
            self.spans.clear();
            for polygon in &self.polygons {
                if polygon.low < middle && middle < polygon.high {
                    let outline = &self.vertices[polygon.vertices.clone()];
                    if let Some(span) = span_at(outline, middle) {
                        self.spans.push(span);
                    }
                }
            }
            area += covered_width(&mut self.spans) * (high - low);
        }
        area
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

/// Where the convex polygon `outline` spans at `height`, from its least to
/// its greatest across.
fn span_at(outline: &[Vector2<f32>], height: f32) -> Option<(f32, f32)> {
    let n = outline.len();
    let mut span = (f32::INFINITY, f32::NEG_INFINITY);
    for i in 0..n {
        let (a, b) = (outline[i], outline[(i + 1) % n]);
        if (a.y <= height) != (b.y <= height) {
            let x = a.x + (height - a.y) * (b.x - a.x) / (b.y - a.y);
            span = (span.0.min(x), span.1.max(x));
        }
    }
    (span.0 <= span.1).then_some(span)
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

    fn part<'a>(shape: &'a ColliderShape, offset: &'a Isometry3<f32>) -> EnvelopePart<'a> {
        EnvelopePart {
            shape,
            offset,
            drag_coefficient: None,
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
}
