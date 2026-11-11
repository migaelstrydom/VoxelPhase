//! Strands: thorns, threads and stems, too thin or too sharp to cut from
//! clay's grid at any spacing a creature can afford.
//!
//! ```text
//!   path (points, each with a radius) ──smoothed──▶ stations along a curve
//!   station ──▶ a ring of vertices round it ──▶ a tube, capped at each end
//!                                               it does not come to a point
//! ```
//!
//! Like clay, a strand has no texture coordinates: its colour is given
//! along its length and rides on its vertices.

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3, Vector4};

use crate::model::MeshPrimitive;
use crate::rendering::vertex::Vertex;

/// A radius below which an end of a strand is a point and needs no cap, m.
const POINT: f32 = 1e-4;

/// Something long and round in section: a thorn, a thread, a stem.
#[derive(Clone, Debug)]
pub struct Strand {
    /// The points it runs through, and its radius at each, m.
    path: Vec<(Vector3<f32>, f32)>,
    /// Sides round it.
    sides: u32,
    /// Rings of vertices from one point of the path to the next.
    rings: u32,
}

/// One ring of a strand: where its middle is, the way the strand runs
/// there, its radius, and how far along the strand it is, 0 to 1.
struct Station {
    centre: Vector3<f32>,
    tangent: Vector3<f32>,
    radius: f32,
    along: f32,
}

impl Strand {
    /// A strand through `path`, curving smoothly from each point to the
    /// next: points, each with the strand's radius there.
    pub fn through(path: Vec<(Vector3<f32>, f32)>) -> Self {
        Self {
            path,
            sides: 8,
            rings: 4,
        }
    }

    /// With `sides` sides round it.
    pub fn sides(mut self, sides: u32) -> Self {
        self.sides = sides.max(3);
        self
    }

    /// With `rings` rings of vertices from one point of its path to the
    /// next.
    pub fn rings(mut self, rings: u32) -> Self {
        self.rings = rings.max(1);
        self
    }

    /// Lay the strand onto `mesh`. `look` gives its colour (linear) and how
    /// open to the light it is (1 in the open) for a share of the way
    /// along it.
    pub fn lay(&self, mesh: &mut MeshPrimitive, look: impl Fn(f32) -> (Vector3<f32>, f32)) {
        let stations = self.stations();
        if stations.len() < 2 {
            return;
        }
        let sides = self.sides;
        let first = mesh.vertices.len() as u32;
        let vertex = |pos: Vector3<f32>, normal: Vector3<f32>, along: f32| {
            let (colour, open) = look(along);
            Vertex {
                pos,
                color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
                tex_coords: Vector2::zeros(),
                normal,
                ao: open,
            }
        };

        // A side that keeps its place round the strand as the strand turns.
        let mut side = perpendicular(stations[0].tangent);
        let mut rims = Vec::with_capacity(stations.len());
        for (index, station) in stations.iter().enumerate() {
            side = (side - station.tangent * side.dot(&station.tangent))
                .try_normalize(1e-6)
                .unwrap_or_else(|| perpendicular(station.tangent));
            let other = station.tangent.cross(&side);
            // A strand that narrows has its skin leaning toward its tip.
            let (before, after) = (
                &stations[index.saturating_sub(1)],
                &stations[(index + 1).min(stations.len() - 1)],
            );
            let run = (after.centre - before.centre).norm().max(1e-9);
            let taper = (after.radius - before.radius) / run;
            let rim: Vec<(Vector3<f32>, Vector3<f32>)> = (0..sides)
                .map(|step| {
                    let round = step as f32 / sides as f32 * TAU;
                    let out = side * round.cos() + other * round.sin();
                    (
                        station.centre + out * station.radius,
                        (out - station.tangent * taper).normalize(),
                    )
                })
                .collect();
            for (pos, normal) in &rim {
                mesh.vertices.push(vertex(*pos, *normal, station.along));
            }
            rims.push(rim);
        }
        for ring in 0..stations.len() as u32 - 1 {
            for step in 0..sides {
                let next = (step + 1) % sides;
                let (a, b) = (first + ring * sides + step, first + ring * sides + next);
                let (c, d) = (b + sides, a + sides);
                mesh.indices.extend([a, b, d, b, c, d]);
            }
        }

        // Each end that is not a point, closed flat.
        let ends = [(0, -1.0), (stations.len() - 1, 1.0)];
        for (index, facing) in ends {
            let station = &stations[index];
            if station.radius < POINT {
                continue;
            }
            let normal = station.tangent * facing;
            let middle = mesh.vertices.len() as u32;
            mesh.vertices
                .push(vertex(station.centre, normal, station.along));
            for (pos, _) in &rims[index] {
                mesh.vertices.push(vertex(*pos, normal, station.along));
            }
            for step in 0..sides {
                let (a, b) = (middle + 1 + step, middle + 1 + (step + 1) % sides);
                if facing > 0.0 {
                    mesh.indices.extend([middle, a, b]);
                } else {
                    mesh.indices.extend([middle, b, a]);
                }
            }
        }
    }

    /// The rings the strand is skinned over, from its first point to its
    /// last.
    fn stations(&self) -> Vec<Station> {
        let points = &self.path;
        if points.len() < 2 {
            return Vec::new();
        }
        // The path carried on past each end, so the curve arrives straight.
        let at = |index: isize| -> (Vector3<f32>, f32) {
            let last = points.len() as isize - 1;
            if index < 0 {
                (points[0].0 * 2.0 - points[1].0, points[0].1)
            } else if index > last {
                let (end, before) = (points[last as usize], points[last as usize - 1]);
                (end.0 * 2.0 - before.0, end.1)
            } else {
                points[index as usize]
            }
        };
        let mut placed: Vec<(Vector3<f32>, f32)> = Vec::new();
        for span in 0..points.len() as isize - 1 {
            let (p0, p1, p2, p3) = (at(span - 1), at(span), at(span + 1), at(span + 2));
            for step in 0..self.rings {
                let t = step as f32 / self.rings as f32;
                placed.push((
                    catmull_rom(p0.0, p1.0, p2.0, p3.0, t),
                    p1.1 + (p2.1 - p1.1) * t,
                ));
            }
        }
        placed.push(points[points.len() - 1]);

        let mut travelled = vec![0.0f32];
        for pair in placed.windows(2) {
            travelled.push(travelled[travelled.len() - 1] + (pair[1].0 - pair[0].0).norm());
        }
        let length = travelled[travelled.len() - 1].max(1e-9);
        (0..placed.len())
            .map(|index| {
                let before = placed[index.saturating_sub(1)].0;
                let after = placed[(index + 1).min(placed.len() - 1)].0;
                Station {
                    centre: placed[index].0,
                    tangent: (after - before)
                        .try_normalize(1e-9)
                        .unwrap_or_else(Vector3::y),
                    radius: placed[index].1.max(0.0),
                    along: travelled[index] / length,
                }
            })
            .collect()
    }
}

/// The point `t` of the way from `p1` to `p2` on the smooth curve through
/// all four.
fn catmull_rom(
    p0: Vector3<f32>,
    p1: Vector3<f32>,
    p2: Vector3<f32>,
    p3: Vector3<f32>,
    t: f32,
) -> Vector3<f32> {
    let (t2, t3) = (t * t, t * t * t);
    (p1 * 2.0
        + (p2 - p0) * t
        + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2
        + (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3)
        * 0.5
}

/// Some unit vector square to `direction`.
fn perpendicular(direction: Vector3<f32>) -> Vector3<f32> {
    let aside = if direction.x.abs() < 0.8 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    direction.cross(&aside).normalize()
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use rustc_hash::FxHashMap;

    use super::*;
    use crate::rendering::material::MaterialId;

    fn laid(strand: &Strand) -> MeshPrimitive {
        let mut mesh = MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: MaterialId(0),
        };
        strand.lay(&mut mesh, |along| (Vector3::repeat(along), 1.0));
        mesh
    }

    /// The volume a closed mesh encloses: positive if it faces outward.
    fn enclosed(mesh: &MeshPrimitive) -> f32 {
        let volumes = mesh.indices.chunks_exact(3).map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize].pos);
            a.dot(&b.cross(&c)) / 6.0
        });
        volumes.sum()
    }

    #[test]
    fn a_straight_strand_is_a_capped_tube_facing_out() {
        let rod = Strand::through(vec![
            (Vector3::new(1.0, 0.0, 0.0), 0.1),
            (Vector3::new(1.0, 2.0, 0.0), 0.1),
        ])
        .sides(24);
        let mesh = laid(&rod);
        let cylinder = PI * 0.1 * 0.1 * 2.0;
        let volume = enclosed(&mesh);
        assert!(
            (volume / cylinder - 1.0).abs() < 0.03,
            "{volume} m³ of {cylinder}"
        );
        // Its side looks straight out, and its far cap along it.
        let side = mesh.vertices[0];
        let out = (side.pos - Vector3::new(1.0, side.pos.y, 0.0)).normalize();
        assert!(side.normal.dot(&out) > 0.99);
        let cap = mesh.vertices.last().unwrap();
        assert!(cap.normal.y > 0.99 && (cap.pos.y - 2.0).abs() < 1e-5);
    }

    #[test]
    fn a_thorn_comes_to_a_point_and_is_coloured_along_its_length() {
        let thorn = Strand::through(vec![
            (Vector3::zeros(), 0.05),
            (Vector3::new(0.0, 0.2, 0.05), 0.02),
            (Vector3::new(0.0, 0.3, 0.2), 0.0),
        ]);
        let mesh = laid(&thorn);
        let tip = mesh
            .vertices
            .iter()
            .max_by(|a, b| a.pos.z.total_cmp(&b.pos.z))
            .unwrap();
        assert!((tip.pos - Vector3::new(0.0, 0.3, 0.2)).norm() < 1e-4);
        assert!(tip.color.x > 0.99, "the tip is coloured {}", tip.color.x);
        assert!(mesh.vertices[0].color.x < 0.01);
        // Sealed: no edge is left with a triangle on one side only, but
        // for those round the point, where a ring has no size.
        let mut edges: FxHashMap<(u32, u32), u32> = FxHashMap::default();
        for t in mesh.indices.chunks_exact(3) {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let open = edges
            .iter()
            .filter(|(_, shared)| **shared == 1)
            .filter(|((a, b), _)| {
                let length =
                    (mesh.vertices[*a as usize].pos - mesh.vertices[*b as usize].pos).norm();
                length > 1e-6
            })
            .count();
        // The root's cap is its own ring of vertices, so its rim is open
        // by the count though closed in space.
        assert_eq!(open, 2 * 8, "{open} open edges");
        assert!(enclosed(&mesh) > 0.0, "the thorn is inside out");
    }
}
