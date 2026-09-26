//! The barrel's shape: a body of revolution about `+Y` whose radius swells
//! from the heads to the belly.
//!
//! One profile answers for everything that has to agree on the shape — the
//! contact hull, the volume the bulk weighs and floats, and the render mesh —
//! so none of them can drift from the others.

use std::f32::consts::TAU;

use nalgebra::Vector3;

use crate::app::spawnables::shared::models::{build_convex_hull, SolidFace};
use crate::collision::convex_hull::ConvexHull;

/// A barrel's outline, measured from its centre.
#[derive(Debug, Clone, Copy)]
pub struct BarrelProfile {
    /// Half the distance from head to head, along `+Y`.
    pub half_height: f32,
    /// Radius at the widest point, halfway up.
    pub belly_radius: f32,
    /// Radius at either head.
    pub head_radius: f32,
}

impl BarrelProfile {
    /// Radius at height `y`: a parabola through the belly and both heads,
    /// which is the curve a bent stave takes.
    pub fn radius_at(&self, y: f32) -> f32 {
        let t = y / self.half_height;
        self.head_radius + (self.belly_radius - self.head_radius) * (1.0 - t * t)
    }

    /// `d radius / d y` at height `y`, for the render mesh's normals.
    pub fn slope_at(&self, y: f32) -> f32 {
        -2.0 * (self.belly_radius - self.head_radius) * y / (self.half_height * self.half_height)
    }

    /// The same barrel shrunk by `thickness` all round: the inside face of
    /// staves and heads that thick.
    ///
    /// Radii shrink by `thickness` measured flat, which on a sloping stave is
    /// slightly more than `thickness` measured square to it, so the result
    /// always lies inside `self`.
    pub fn inset(&self, thickness: f32) -> Self {
        Self {
            half_height: self.half_height - thickness,
            belly_radius: self.belly_radius - thickness,
            head_radius: self.head_radius - thickness,
        }
    }

    /// A convex hull of `sides` flat staves, each bent through `rings` rings
    /// from head to head.
    ///
    /// Convex because the profile bulges: every chord of it lies inside it.
    pub fn hull(&self, sides: usize, rings: usize) -> ConvexHull {
        let (vertices, faces) = self.hull_geometry(sides, rings);
        build_convex_hull(&vertices, &faces)
    }

    /// Vertices and faces of [`hull`](Self::hull): ring-major vertices, one
    /// quad per stave per band between rings, and one polygon per head.
    fn hull_geometry(&self, sides: usize, rings: usize) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
        let index = |ring: usize, side: usize| ring * sides + side % sides;
        let far_side = sides / 2;

        let mut vertices = Vec::with_capacity(sides * rings);
        for ring in 0..rings {
            let y = self.ring_height(ring, rings);
            let radius = self.radius_at(y);
            for side in 0..sides {
                let angle = side as f32 * TAU / sides as f32;
                vertices.push(Vector3::new(radius * angle.cos(), y, radius * angle.sin()));
            }
        }

        let mut faces = Vec::with_capacity(sides * (rings - 1) + 2);
        for band in 0..rings - 1 {
            for side in 0..sides {
                faces.push(SolidFace {
                    vertex_indices: vec![
                        index(band, side),
                        index(band + 1, side),
                        index(band + 1, side + 1),
                        index(band, side + 1),
                    ],
                    opposite_vertex: index(band, side + far_side),
                });
            }
        }
        faces.push(SolidFace {
            vertex_indices: (0..sides).map(|side| index(0, side)).collect(),
            opposite_vertex: index(rings - 1, 0),
        });
        faces.push(SolidFace {
            vertex_indices: (0..sides).map(|side| index(rings - 1, side)).collect(),
            opposite_vertex: index(0, 0),
        });

        (vertices, faces)
    }

    /// Height of ring `ring` of `rings`, spaced evenly from the bottom head
    /// to the top one.
    pub fn ring_height(&self, ring: usize, rings: usize) -> f32 {
        -self.half_height + 2.0 * self.half_height * ring as f32 / (rings - 1) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> BarrelProfile {
        BarrelProfile {
            half_height: 0.45,
            belly_radius: 0.3,
            head_radius: 0.26,
        }
    }

    #[test]
    fn the_belly_is_wider_than_the_heads() {
        let p = profile();
        assert!((p.radius_at(0.0) - p.belly_radius).abs() < 1e-6);
        assert!((p.radius_at(p.half_height) - p.head_radius).abs() < 1e-6);
        assert!((p.radius_at(-p.half_height) - p.head_radius).abs() < 1e-6);
    }

    /// Every inside corner lies behind every outside face, by close to the
    /// wall's thickness: the shell never has a negative thickness anywhere.
    #[test]
    fn the_inset_hull_lies_inside_the_outer_one() {
        let thickness = 0.02;
        let outer = profile().hull(16, 5);
        let inner = profile().inset(thickness).hull(16, 5);

        for vertex in &inner.vertices {
            for face in &outer.faces {
                let on_face = outer.vertices[face.vertex_indices[0] as usize];
                let height = face.normal.dot(&(vertex - on_face));
                assert!(
                    height < -0.8 * thickness,
                    "inner vertex {vertex:?} is {height} from an outer face"
                );
            }
        }
    }

    #[test]
    fn the_hull_volume_is_close_to_the_smooth_barrel() {
        let p = profile();
        let hull = p.hull(16, 5);

        // ∫ π r(y)² dy over the height, by the midpoint rule.
        let steps = 1000;
        let dy = 2.0 * p.half_height / steps as f32;
        let smooth: f32 = (0..steps)
            .map(|i| {
                let r = p.radius_at(-p.half_height + (i as f32 + 0.5) * dy);
                std::f32::consts::PI * r * r * dy
            })
            .sum();

        let ratio = hull.compute_volume() / smooth;
        assert!(
            (0.95..1.0).contains(&ratio),
            "hull holds {ratio} of the smooth barrel"
        );
    }
}
