//! How a fluid drags a shape moving through it: the quadratic law
//! `F = ½ρ·Cd·A·|v|·v`, and its turning twin `τ = ½ρ·Cω·A·r²·|ω|·ω`, with the
//! drag coefficient and areas each collider shape presents.
//!
//! Every medium uses it: water (`water::buoyancy`) and the world's air
//! ([`super::Atmosphere`]) differ only in density and in how much of the body
//! is in them.

use std::f32::consts::PI;

use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use smallvec::SmallVec;

use super::collider::ColliderShape;

/// Drag coefficient of a sphere.
const SPHERE: f32 = 0.47;
/// Drag coefficient of a box, and of a convex hull.
const BOX: f32 = 1.05;
/// Drag coefficient of a capsule.
const CAPSULE: f32 = 0.82;
/// Rotational drag multiplier `Cω`.
pub const ANGULAR_COEFFICIENT: f32 = 2.0;

/// What a shape presents to a fluid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeDrag {
    /// Drag coefficient `Cd`.
    pub coefficient: f32,
    /// Projected area averaged over every direction, m².
    pub mean_area: f32,
    /// Square of the arm spin drag acts at, m².
    pub spin_radius_sq: f32,
}

impl ShapeDrag {
    /// What `shape` presents.
    pub fn of(shape: &ColliderShape) -> Self {
        let (coefficient, mean_area, spin_radius_sq) = match shape {
            ColliderShape::Sphere { radius } => {
                let r = *radius;
                (SPHERE, PI * r * r, r * r)
            }
            ColliderShape::Box { half_extents } => {
                let (hx, hy, hz) = (half_extents.x, half_extents.y, half_extents.z);
                let mean_area = 2.0 * (hx * hy + hy * hz + hx * hz);
                (BOX, mean_area, hx * hx + hy * hy + hz * hz)
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let r = *radius;
                let cylinder = (2.0 * (half_height - r)).max(0.0);
                let surface = 2.0 * PI * r * cylinder + 4.0 * PI * r * r;
                (CAPSULE, surface * 0.25, half_height * half_height)
            }
            ColliderShape::ConvexHull { hull } => {
                // A convex body's projected area, averaged over every
                // direction, is a quarter of its surface (Cauchy), as the
                // box's is above.
                let radius = hull.bounding_radius;
                (BOX, hull.surface_area() * 0.25, radius * radius)
            }
        };
        Self {
            coefficient,
            mean_area,
            spin_radius_sq,
        }
    }

    /// `Cd·A` over the mean area, m²: the linear drag's area.
    pub fn linear_area(&self) -> f32 {
        self.coefficient * self.mean_area
    }

    /// `Cω·A·r²`, m⁴: the spin drag's area.
    pub fn angular_area(&self) -> f32 {
        ANGULAR_COEFFICIENT * self.mean_area * self.spin_radius_sq
    }
}

/// A body's drag areas: what turns a fluid's density and the body's speed
/// into drag.
///
/// A property of the body's shape as a whole, not of how it is cut up: a
/// crate of eight boxes drags as the one box they fill. The linear area is
/// the body's silhouette, averaged over directions: the 2D convex hull of
/// every part's projected outline, for each of [`SILHOUETTE_DIRECTIONS`]
/// directions. Its drag coefficient is the parts' own, weighted by their
/// area. The spin area takes that silhouette at the parts' arm about the
/// body's centre of mass, their own spin radius plus their offset squared,
/// so a dumbbell turns against the air as a dumbbell.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DragArea {
    /// `Cd·A`, m².
    pub linear: f32,
    /// `Cω·A·r²`, m⁴.
    pub angular: f32,
}

/// Directions a body's silhouette is averaged over: enough that a box's
/// mean comes within a percent or two of Cauchy's exact quarter-surface.
pub const SILHOUETTE_DIRECTIONS: usize = 32;
/// Points round each projected circle (a sphere's, a capsule's ends).
const CIRCLE_POINTS: usize = 24;

impl DragArea {
    /// The drag areas of a body made of `parts`, each a shape placed in the
    /// body's frame, measured from its centre of mass.
    pub fn of<'a>(
        parts: impl IntoIterator<Item = (&'a ColliderShape, &'a Isometry3<f32>)>,
    ) -> Self {
        let parts: SmallVec<[(&ColliderShape, &Isometry3<f32>); 8]> = parts.into_iter().collect();
        if parts.is_empty() {
            return Self::default();
        }
        let mut weighted_coefficient = 0.0;
        let mut weighted_arm = 0.0;
        let mut total = 0.0;
        for (shape, offset) in &parts {
            let drag = ShapeDrag::of(shape);
            weighted_coefficient += drag.coefficient * drag.mean_area;
            weighted_arm +=
                (drag.spin_radius_sq + offset.translation.vector.norm_squared()) * drag.mean_area;
            total += drag.mean_area;
        }
        if total <= 0.0 {
            return Self::default();
        }
        // One convex part's mean silhouette is Cauchy's quarter of its
        // surface exactly; only a body of several needs measuring.
        let area = if parts.len() == 1 {
            total
        } else {
            mean_silhouette(&parts)
        };
        let coefficient = weighted_coefficient / total;
        let arm_sq = weighted_arm / total;
        Self {
            linear: coefficient * area,
            angular: ANGULAR_COEFFICIENT * area * arm_sq,
        }
    }
}

/// The parts' silhouette area averaged over directions spread evenly over a
/// hemisphere (a silhouette and its opposite are the same).
fn mean_silhouette(parts: &[(&ColliderShape, &Isometry3<f32>)]) -> f32 {
    let golden = PI * (3.0 - 5.0f32.sqrt());
    let mut points = Vec::new();
    let mut sum = 0.0;
    for i in 0..SILHOUETTE_DIRECTIONS {
        let z = 1.0 - (i as f32 + 0.5) / SILHOUETTE_DIRECTIONS as f32;
        let ring = (1.0 - z * z).sqrt();
        let azimuth = golden * i as f32;
        let view = Vector3::new(ring * azimuth.cos(), ring * azimuth.sin(), z);
        let across = view
            .cross(&Vector3::x())
            .try_normalize(1e-6)
            .unwrap_or_else(|| view.cross(&Vector3::y()).normalize());
        let up = view.cross(&across);
        points.clear();
        for (shape, offset) in parts {
            outline(shape, offset, &across, &up, &mut points);
        }
        sum += hull_area(&mut points);
    }
    sum / SILHOUETTE_DIRECTIONS as f32
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
    let mut circle = |centre: Point3<f32>, radius: f32| {
        let c = project(centre);
        out.extend((0..CIRCLE_POINTS).map(|k| {
            let angle = std::f32::consts::TAU * k as f32 / CIRCLE_POINTS as f32;
            // Out to the circumscribed polygon, so its area is the circle's
            // to within a fraction of a percent, not under it.
            let r = radius / (PI / CIRCLE_POINTS as f32).cos();
            c + Vector2::new(angle.cos(), angle.sin()) * r
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

/// Area of the convex hull of `points` (Andrew's monotone chain). Reorders
/// them.
fn hull_area(points: &mut [Vector2<f32>]) -> f32 {
    if points.len() < 3 {
        return 0.0;
    }
    points.sort_unstable_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let cross = |o: Vector2<f32>, a: Vector2<f32>, b: Vector2<f32>| {
        (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
    };
    let mut hull: Vec<Vector2<f32>> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &Vector2<f32>>> = if pass == 0 {
            Box::new(points.iter())
        } else {
            Box::new(points.iter().rev())
        };
        for &p in ordered {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    let n = hull.len();
    (0..n)
        .map(|i| {
            let (a, b) = (hull[i], hull[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum::<f32>()
        .abs()
        * 0.5
}

#[cfg(test)]
mod tests {
    use nalgebra::Translation3;

    use super::*;

    fn cube(pieces_per_side: usize) -> f32 {
        let half = 0.5 / pieces_per_side as f32;
        let shape = ColliderShape::Box {
            half_extents: Vector3::repeat(half),
        };
        let offsets: Vec<Isometry3<f32>> = (0..pieces_per_side.pow(3))
            .map(|i| {
                let at = |k: usize| -0.5 + half * (2 * k + 1) as f32;
                let n = pieces_per_side;
                Isometry3::from_parts(
                    Translation3::new(at(i % n), at(i / n % n), at(i / (n * n))),
                    nalgebra::UnitQuaternion::identity(),
                )
            })
            .collect();
        DragArea::of(offsets.iter().map(|o| (&shape, o))).linear
    }

    /// A metre cube drags the same however it is cut: eight boxes and
    /// twenty-seven measure the same silhouette, and that is the one box's
    /// exact quarter-surface to within the directions' sampling.
    #[test]
    fn a_body_drags_as_its_shape_however_it_is_cut() {
        let whole = cube(1);
        let cauchy = BOX * 6.0 / 4.0;
        assert!((whole - cauchy).abs() < 1e-5, "{whole} against {cauchy}");
        let (eight, twenty_seven) = (cube(2), cube(3));
        assert!(
            (eight - twenty_seven).abs() < 1e-4 * eight,
            "8 pieces {eight}, 27 pieces {twenty_seven}"
        );
        assert!(
            (eight - whole).abs() < 0.01 * whole,
            "8 pieces {eight} against {whole}"
        );
    }

    /// A sphere's silhouette is its great circle, from every side.
    #[test]
    fn a_sphere_drags_by_its_great_circle() {
        let sphere = ColliderShape::Sphere { radius: 0.3 };
        let area = DragArea::of([(&sphere, &Isometry3::identity())]).linear;
        let exact = SPHERE * PI * 0.09;
        assert!(
            (area - exact).abs() < 0.01 * exact,
            "{area} against {exact}"
        );
    }

    /// Two balls on a bar turn against the air at their arm, not as if both
    /// sat at the centre.
    #[test]
    fn a_dumbbell_turns_against_the_air_at_its_arm() {
        let ball = ColliderShape::Sphere { radius: 0.1 };
        let centred = Isometry3::identity();
        let ends = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let together = DragArea::of([(&ball, &centred), (&ball, &centred)]).angular;
        let apart = DragArea::of(ends.iter().map(|o| (&ball, o))).angular;
        assert!(
            apart > 50.0 * together,
            "apart {apart}, together {together}"
        );
    }
}
