//! Warps that bend a body part off the ellipsoid it starts as: a beak that
//! tapers and hooks, a flipper that thins and sweeps back, a frill whose
//! edge runs in lobes, a shell that is not quite smooth. A perfect
//! ellipsoid is what makes a creature built from them read as a toy.
//!
//! A warp works in the part's own frame (its axes, its centre, metres), so
//! the same warp means the same thing on a part however it is turned.

use std::collections::HashMap;

use nalgebra::{UnitQuaternion, Vector3};

use crate::model::MeshPrimitive;
use crate::utils::noise::fbm_perlin_3d;

/// Grid that vertices sharing a position are welded on when normals are
/// rebuilt, m: a sphere's seam and poles repeat vertices.
const WELD_GRID: f32 = 1e-5;

/// One way of bending a part off its ellipsoid.
#[derive(Clone, Copy, Debug)]
pub enum Warp {
    /// Narrows towards one end. The cross-section is whole at the far end
    /// from `along` and `tip` of itself at the `along` end.
    Taper {
        /// The narrow end's direction, a unit vector in the part's frame.
        along: Vector3<f32>,
        /// Share of the cross-section left at the narrow end.
        tip: f32,
    },
    /// Bends towards one side: the `along` end is moved `amount` towards
    /// `toward`, the move growing with the square of the way along, so the
    /// other end stays put.
    Bend {
        /// The end that bends, a unit vector in the part's frame.
        along: Vector3<f32>,
        /// Which way it bends, a unit vector in the part's frame.
        toward: Vector3<f32>,
        /// How far the end moves, m.
        amount: f32,
    },
    /// Lobes round the part's edge about its own up axis: each lobe pushed
    /// out and lifted, the edge between them left where it was, like a
    /// ruff. Only the rim moves; the top and bottom stay put.
    Scallop {
        /// Lobes round the edge.
        lobes: u32,
        /// How far a lobe reaches out, as a share of the part's width.
        bulge: f32,
        /// How far a lobe lifts, m.
        lift: f32,
    },
    /// Lumps and hollows: the surface pushed out or in along its normal.
    Lumps {
        /// Lumps per metre.
        frequency: f32,
        /// Furthest a lump stands out or a hollow sinks, m.
        amplitude: f32,
        /// Seed of the lumps.
        seed: u32,
    },
}

impl Warp {
    /// Where `local`, a point on a part of semi-axes `radii` with outward
    /// normal `normal`, both in the part's frame, moves to.
    pub fn apply(
        &self,
        local: Vector3<f32>,
        normal: Vector3<f32>,
        radii: Vector3<f32>,
    ) -> Vector3<f32> {
        match *self {
            Warp::Taper { along, tip } => {
                let length = local.dot(&along);
                let across = local - along * length;
                let share = share_along(local, along, radii);
                along * length + across * (1.0 + (tip - 1.0) * share)
            }
            Warp::Bend {
                along,
                toward,
                amount,
            } => {
                let share = share_along(local, along, radii);
                local + toward * amount * share * share
            }
            Warp::Scallop { lobes, bulge, lift } => {
                let angle = local.x.atan2(local.z);
                let edge = (local.x / radii.x).hypot(local.z / radii.z).powi(4);
                let lobe = 0.5 + 0.5 * (lobes as f32 * angle).cos();
                let out = 1.0 + bulge * lobe * edge;
                Vector3::new(local.x * out, local.y + lift * lobe * edge, local.z * out)
            }
            Warp::Lumps {
                frequency,
                amplitude,
                seed,
            } => {
                let at = local * frequency;
                local + normal * amplitude * fbm_perlin_3d(at.x, at.y, at.z, 2, 0.5, seed)
            }
        }
    }
}

/// How far along `along` a point lies on a part: 0 at the far end, 1 at the
/// `along` end.
fn share_along(local: Vector3<f32>, along: Vector3<f32>, radii: Vector3<f32>) -> f32 {
    let extent = radii.component_mul(&along).norm();
    ((local.dot(&along) / extent + 1.0) * 0.5).clamp(0.0, 1.0)
}

/// `warps`, in order, applied to a point of a part at `centre` turned by
/// `turn`, with outward normal `normal` there; both the point and the
/// result in the body's frame.
pub fn warp_point(
    warps: &[Warp],
    centre: Vector3<f32>,
    turn: UnitQuaternion<f32>,
    radii: Vector3<f32>,
    point: Vector3<f32>,
    normal: Vector3<f32>,
) -> Vector3<f32> {
    let local_normal = turn.inverse() * normal;
    let local = warps
        .iter()
        .fold(turn.inverse() * (point - centre), |at, warp| {
            warp.apply(at, local_normal, radii)
        });
    turn * local + centre
}

/// Rebuild `mesh`'s normals from its triangles, after its vertices have
/// moved by less than turns a face over: each the area-weighted sum of the faces round it, summed across
/// vertices that share a position, so a seam does not show.
pub fn rebuild_normals(mesh: &mut MeshPrimitive) {
    let key = |p: Vector3<f32>| {
        (
            (p.x / WELD_GRID).round() as i64,
            (p.y / WELD_GRID).round() as i64,
            (p.z / WELD_GRID).round() as i64,
        )
    };
    let mut sums: HashMap<(i64, i64, i64), Vector3<f32>> = HashMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|i| mesh.vertices[triangle[i] as usize].pos);
        let face = (b - a).cross(&(c - a));
        for p in [a, b, c] {
            *sums.entry(key(p)).or_insert_with(Vector3::zeros) += face;
        }
    }
    for vertex in &mut mesh.vertices {
        if let Some(sum) = sums
            .get(&key(vertex.pos))
            .and_then(|s| s.try_normalize(1e-12))
        {
            // The winding is the sphere generator's; the outward side is
            // whichever the normal before the warp faced.
            vertex.normal = if sum.dot(&vertex.normal) < 0.0 {
                -sum
            } else {
                sum
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::creatures::body_parts::Ellipsoid;
    use crate::rendering::colour::Colour;
    use crate::rendering::material::MaterialId;

    #[test]
    fn rebuilt_normals_of_an_unwarped_ellipsoid_match_its_own() {
        let part = Ellipsoid::new((0.3, 0.2, 0.4), (0.1, 0.0, -0.2)).yawed(0.4);
        let analytic = part.mesh(Colour::WHITE, MaterialId(0));
        let mut rebuilt = analytic.clone();
        rebuild_normals(&mut rebuilt);
        let worst = analytic
            .vertices
            .iter()
            .zip(&rebuilt.vertices)
            .map(|(a, b)| a.normal.dot(&b.normal))
            .fold(1.0_f32, f32::min);
        assert!(worst > 0.97, "worst agreement {worst}");
    }

    #[test]
    fn a_taper_narrows_only_the_tip_end() {
        let radii = Vector3::new(0.05, 0.05, 0.2);
        let taper = Warp::Taper {
            along: Vector3::z(),
            tip: 0.25,
        };
        let at = |z: f32| taper.apply(Vector3::new(0.05, 0.0, z), Vector3::x(), radii);
        assert!(
            (at(-0.2).x - 0.05).abs() < 1e-6,
            "the far end keeps its width"
        );
        assert!(
            (at(0.2).x - 0.0125).abs() < 1e-6,
            "the tip is a quarter as wide"
        );
        assert!(
            (at(0.0).x - 0.03125).abs() < 1e-6,
            "halfway is half narrowed"
        );
    }

    #[test]
    fn a_bend_moves_the_end_and_leaves_the_root() {
        let radii = Vector3::new(0.05, 0.05, 0.2);
        let bend = Warp::Bend {
            along: Vector3::z(),
            toward: -Vector3::y(),
            amount: 0.04,
        };
        let root = bend.apply(Vector3::new(0.0, 0.0, -0.2), Vector3::z(), radii);
        let tip = bend.apply(Vector3::new(0.0, 0.0, 0.2), Vector3::z(), radii);
        assert!(root.y.abs() < 1e-6);
        assert!((tip.y + 0.04).abs() < 1e-6);
    }
}
