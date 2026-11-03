//! How a fluid drags a shape moving through it: the quadratic law
//! `F = ½ρ·Cd·A·|v|·v`, and its turning twin `τ = ½ρ·Cω·A·r²·|ω|·ω`, with the
//! drag coefficient and areas each collider shape presents.
//!
//! Every medium uses it: water (`water::buoyancy`) and the world's air
//! ([`super::Atmosphere`]) differ only in density and in how much of the body
//! is in them.

use std::f32::consts::PI;

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

/// A body's drag areas summed over what fluids see of it: what turns a
/// fluid's density and the body's speed into drag.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DragArea {
    /// `Σ Cd·A`, m².
    pub linear: f32,
    /// `Σ Cω·A·r²`, m⁴.
    pub angular: f32,
}

impl DragArea {
    /// The areas of `shapes` together.
    pub fn of<'a>(shapes: impl IntoIterator<Item = &'a ColliderShape>) -> Self {
        shapes
            .into_iter()
            .map(ShapeDrag::of)
            .fold(Self::default(), |sum, drag| Self {
                linear: sum.linear + drag.linear_area(),
                angular: sum.angular + drag.angular_area(),
            })
    }
}
