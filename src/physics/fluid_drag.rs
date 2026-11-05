//! How a fluid drags a shape moving through it: the quadratic law
//! `F = ½ρ·Cd·A·|v|·v`, and its turning twin `τ = ½ρ·Cω·A·r²·|ω|·ω`, with the
//! drag coefficient and areas each collider shape presents.
//!
//! Every medium uses it: water (`water::buoyancy`) and the world's air
//! ([`super::Atmosphere`]) differ only in density and in how much of the body
//! is in them.

use std::f32::consts::PI;

use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;
use super::silhouette::Silhouette;

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
/// the body's silhouette averaged over directions ([`Silhouette`]: the union
/// of its parts' outlines, so a gap between parts lets the flow through).
/// Its drag coefficient is the parts' own, weighted by their area, or the
/// one the body's bulk declares. The spin area takes that silhouette at the
/// body's reach about its centre of mass, the furthest any part reaches,
/// as one part's spin radius is its own furthest reach: a dumbbell turns
/// against the air at its ends.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DragArea {
    /// `Cd·A`, m².
    pub linear: f32,
    /// `Cω·A·r²`, m⁴.
    pub angular: f32,
}

impl DragArea {
    /// The drag areas of a body whose envelope is `parts`, each placed in
    /// the body's frame, measured from its centre of mass.
    pub fn of<'a>(parts: impl IntoIterator<Item = EnvelopePart<'a>>) -> Self {
        let parts: SmallVec<[EnvelopePart; 8]> = parts.into_iter().collect();
        let mut weighted_coefficient = 0.0;
        let mut total = 0.0;
        let mut reach: f32 = 0.0;
        for part in &parts {
            let drag = part.drag();
            weighted_coefficient += drag.coefficient * drag.mean_area;
            total += drag.mean_area;
            let arm = part.offset.translation.vector.norm();
            reach = reach.max(arm + drag.spin_radius_sq.sqrt());
        }
        if total <= 0.0 {
            return Self::default();
        }
        // One convex part's mean silhouette is Cauchy's quarter of its
        // surface exactly; only a body of several needs measuring.
        let area = if parts.len() == 1 {
            total
        } else {
            Silhouette::default().mean_area(&parts)
        };
        let coefficient = weighted_coefficient / total;
        Self {
            linear: coefficient * area,
            angular: ANGULAR_COEFFICIENT * area * reach * reach,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Isometry3, Vector3};

    use super::*;

    /// A metre cube cut into `pieces_per_side`³ boxes.
    fn cube(pieces_per_side: usize) -> DragArea {
        let half = 0.5 / pieces_per_side as f32;
        let shape = ColliderShape::Box {
            half_extents: Vector3::repeat(half),
        };
        let at = |k: usize| -0.5 + half * (2 * k + 1) as f32;
        let n = pieces_per_side;
        let offsets: Vec<Isometry3<f32>> = (0..n.pow(3))
            .map(|i| Isometry3::translation(at(i % n), at(i / n % n), at(i / (n * n))))
            .collect();
        DragArea::of(offsets.iter().map(|o| part(&shape, o, None)))
    }

    /// A metre cube drags and turns the same however it is cut: eight boxes
    /// and twenty-seven measure the same silhouette and reach, and that is
    /// the one box's exact quarter-surface to within the directions'
    /// sampling.
    #[test]
    fn a_body_drags_as_its_shape_however_it_is_cut() {
        let whole = cube(1);
        let cauchy = BOX * 6.0 / 4.0;
        assert!(
            (whole.linear - cauchy).abs() < 1e-5,
            "{} against {cauchy}",
            whole.linear
        );
        for cut in [cube(2), cube(3)] {
            assert!(
                (cut.linear - whole.linear).abs() < 0.01 * whole.linear,
                "cut {cut:?} against {whole:?}"
            );
            let (cut_reach, whole_reach) = (cut.angular / cut.linear, whole.angular / whole.linear);
            assert!(
                (cut_reach - whole_reach).abs() < 1e-4 * whole_reach,
                "cut {cut:?} against {whole:?}"
            );
        }
    }

    /// A sphere's silhouette is its great circle, from every side.
    #[test]
    fn a_sphere_drags_by_its_great_circle() {
        let sphere = ColliderShape::Sphere { radius: 0.3 };
        let area = DragArea::of([part(&sphere, &Isometry3::identity(), None)]).linear;
        let exact = SPHERE * PI * 0.09;
        assert!(
            (area - exact).abs() < 0.01 * exact,
            "{area} against {exact}"
        );
    }

    /// Two balls on a bar drag by their two discs, not by the hull round
    /// both, and turn against the air at their ends.
    #[test]
    fn a_dumbbell_drags_by_its_balls_and_turns_at_its_ends() {
        let ball = ColliderShape::Sphere { radius: 0.1 };
        let one = DragArea::of([part(&ball, &Isometry3::identity(), None)]);
        let ends = [
            Isometry3::translation(-1.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
        ];
        let dumbbell = DragArea::of(ends.iter().map(|o| part(&ball, o, None)));
        // The balls' circles are drawn circumscribed, about half a percent
        // over, and hide each other only end on.
        assert!(
            (dumbbell.linear - 2.0 * one.linear).abs() < 0.02 * one.linear,
            "dumbbell {dumbbell:?}, one ball {one:?}"
        );
        let reach_sq = dumbbell.angular / (ANGULAR_COEFFICIENT * dumbbell.linear / SPHERE);
        assert!((reach_sq - 1.21).abs() < 1e-4, "reach² {reach_sq}");
    }

    /// A body that declares its drag coefficient drags by it over the same
    /// silhouette, and turns as before: spin drag has its own coefficient.
    #[test]
    fn a_declared_coefficient_replaces_the_shapes() {
        let ball = ColliderShape::Sphere { radius: 0.3 };
        let at = Isometry3::identity();
        let own = DragArea::of([part(&ball, &at, None)]);
        let porous = DragArea::of([part(&ball, &at, Some(1.0))]);
        assert!((porous.linear - own.linear / SPHERE).abs() < 1e-6);
        assert_eq!(porous.angular, own.angular);
    }

    fn part<'a>(
        shape: &'a ColliderShape,
        offset: &'a Isometry3<f32>,
        drag_coefficient: Option<f32>,
    ) -> EnvelopePart<'a> {
        EnvelopePart {
            shape,
            offset,
            drag_coefficient,
        }
    }
}
