//! A clump's drawn surface: each lobe a ball of rounded nubs, the way a
//! clipped shrub reads from a few metres off.
//!
//! ```text
//!    ╭╮╭─╮╭╮        a lobe is a geodesic sphere pushed out into nubs:
//!   ╭╯╰╯ ╰╯╰╮       tops at the lobe's radius, valleys a share below it.
//!   ╰ crevice╯      Valleys and the folds between lobes are baked darker.
//! ```
//!
//! The silhouette is in the mesh, because a smooth ball with leafy shading
//! still reads as a plastic ball. Built once, in the clump's frame, and drawn
//! as it was built.

use nalgebra::{Point3, Vector2, Vector3, Vector4};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::shape::{FoliageShape, Lobe};
use crate::geometry::generate_icosphere;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::perlin_3d;

/// The greens a skin is coloured in, linear RGB.
#[derive(Debug, Clone, Copy)]
pub struct LeafPalette {
    /// Leaves deep in a valley between nubs.
    pub shade: Vector3<f32>,
    /// Leaves on a nub's top.
    pub tip: Vector3<f32>,
    /// How far each nub's hue strays from the others, as a share of the
    /// colour: what makes the clump read as many sprigs, not one surface.
    pub sprig_variation: f32,
    /// Colour of a blossom on a nub's tip.
    pub blossom: Vector3<f32>,
    /// Share of nubs carrying a blossom.
    pub blossom_share: f32,
}

impl Default for LeafPalette {
    /// The garden's clipped box green: olive in the shade, sap green on top.
    fn default() -> Self {
        Self {
            shade: Vector3::new(0.045, 0.085, 0.020),
            tip: Vector3::new(0.20, 0.33, 0.065),
            sprig_variation: 0.1,
            blossom: Vector3::new(0.90, 0.82, 0.78),
            blossom_share: 0.0,
        }
    }
}

/// How a skin is grown over a shape.
#[derive(Debug, Clone, Copy)]
pub struct SkinSpec {
    /// Geodesic subdivisions of each lobe's sphere: 4 is 2562 vertices a lobe.
    pub subdivisions: u32,
    /// Radius of one nub, in metres.
    pub nub_radius: f32,
    /// Depth of the valleys between nubs, as a share of the lobe's radius.
    pub nub_depth: f32,
    /// Irregularity laid over the nubs, as a share of the lobe's radius.
    pub roughness: f32,
    /// Depth of the leaf-sized relief over everything, in metres: what
    /// breaks a nub's surface into leaves.
    pub leaf_relief: f32,
    /// The colours.
    pub palette: LeafPalette,
}

impl Default for SkinSpec {
    fn default() -> Self {
        Self {
            subdivisions: 4,
            nub_radius: 0.075,
            nub_depth: 0.09,
            roughness: 0.035,
            leaf_relief: 0.012,
            palette: LeafPalette::default(),
        }
    }
}

/// A grown skin, in the clump's frame.
#[derive(Debug, Clone)]
pub struct Skin {
    /// The vertices at rest.
    pub vertices: Vec<Vertex>,
    /// Triangle list.
    pub indices: Vec<u32>,
}

/// One nub on a lobe: where it points, how big it is and how its sprig is
/// coloured.
struct Nub {
    direction: Vector3<f32>,
    /// Its radius as a share of the spec's.
    size: f32,
    tint: f32,
    blossom: bool,
}

impl SkinSpec {
    /// The skin over `shape`, its nubs placed from `seed`.
    pub fn grow(&self, shape: &FoliageShape, seed: u32) -> Skin {
        let sphere = generate_icosphere(self.subdivisions);
        let mut rng = StdRng::seed_from_u64(seed as u64 ^ 0x5eed_1eaf);
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for (index, lobe) in shape.lobes.iter().enumerate() {
            let nubs = self.nubs(lobe, &mut rng);
            let base = vertices.len() as u32;
            let mut hidden = Vec::with_capacity(sphere.positions.len());
            for &direction in &sphere.positions {
                let (height, nub) = self.nub_height(direction, lobe, &nubs);
                let wobble = perlin_3d(
                    direction.x * 3.0 + index as f32 * 7.1,
                    direction.y * 3.0,
                    direction.z * 3.0,
                    seed,
                ) * self.roughness;
                let at = lobe.surface_point(direction, 1.0).coords * LEAF_FREQUENCY;
                let leaves = self.leaf_relief * perlin_3d(at.x, at.y, at.z, seed ^ 0xa5a5);
                let normal = lobe.normal_at(direction);
                let scale = 1.0 - self.nub_depth * (1.0 - height) + wobble;
                let position = lobe.surface_point(direction, scale) + normal * leaves;
                hidden.push(self.is_buried(position, index, shape));
                vertices.push(Vertex {
                    pos: position.coords,
                    color: opaque(self.colour(height, nub)),
                    tex_coords: Vector2::zeros(),
                    normal,
                    ao: self.crevice(height) * lobe_occlusion(position, normal, index, shape),
                });
            }
            for face in sphere.indices.chunks_exact(3) {
                if face.iter().all(|&i| hidden[i as usize]) {
                    continue;
                }
                indices.extend(face.iter().map(|&i| base + i));
            }
        }
        smooth_normals(&mut vertices, &indices);
        compact(Skin { vertices, indices })
    }

    /// Nubs spread evenly over a lobe, as many as fit their size.
    fn nubs(&self, lobe: &Lobe, rng: &mut StdRng) -> Vec<Nub> {
        let count = (lobe.area() / (std::f32::consts::PI * self.nub_radius * self.nub_radius) * 1.1)
            .round()
            .max(8.0) as usize;
        let golden = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
        let jitter = 0.35 / (count as f32).sqrt();
        (0..count)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as f32 + 0.5) / count as f32;
                let ring = (1.0 - y * y).sqrt();
                let azimuth = golden * i as f32;
                let even = Vector3::new(ring * azimuth.cos(), y, ring * azimuth.sin());
                let nudge = Vector3::new(
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                ) * jitter;
                Nub {
                    direction: (even + nudge).normalize(),
                    size: rng.gen_range(0.75..1.3),
                    tint: rng.gen_range(-1.0..1.0),
                    blossom: rng.gen::<f32>() < self.palette.blossom_share,
                }
            })
            .collect()
    }

    /// How far up a nub a point in `direction` from the lobe's centre is,
    /// 0 in a valley and 1 on a top, and which nub it is on.
    fn nub_height<'a>(
        &self,
        direction: Vector3<f32>,
        lobe: &Lobe,
        nubs: &'a [Nub],
    ) -> (f32, Option<&'a Nub>) {
        // Chord distance on the unit sphere, against a nub's radius there.
        let reach = (self.nub_radius / lobe.mean_radius()) * 1.15;
        let mut best = (0.0f32, None);
        for nub in nubs {
            let chord2 = 2.0 * (1.0 - direction.dot(&nub.direction));
            let nub_reach = reach * nub.size;
            let cap = 1.0 - chord2 / (nub_reach * nub_reach);
            if cap > best.0 {
                best = (cap, Some(nub));
            }
        }
        (best.0.max(0.0).sqrt(), best.1)
    }

    /// Whether a point is inside another lobe's valleys: hidden whatever
    /// way the clump is turned.
    fn is_buried(&self, point: Point3<f32>, index: usize, shape: &FoliageShape) -> bool {
        let inner = (1.0 - self.nub_depth) * 0.97;
        shape
            .lobes
            .iter()
            .enumerate()
            .any(|(other, lobe)| other != index && lobe.contains_scaled(point, inner))
    }

    fn colour(&self, height: f32, nub: Option<&Nub>) -> Vector3<f32> {
        let palette = &self.palette;
        let tint = nub.map_or(0.0, |n| n.tint) * palette.sprig_variation;
        let leaf = palette.shade.lerp(&palette.tip, height.powf(0.7));
        // A sprig's tint leans it yellow or blue-green, not lighter or darker.
        let leaf = leaf.component_mul(&Vector3::new(1.0 + tint, 1.0, 1.0 - tint));
        match nub {
            Some(nub) if nub.blossom && height > 0.75 => palette.blossom,
            _ => leaf,
        }
    }

    /// Light reaching a point between nubs: little in a valley.
    fn crevice(&self, height: f32) -> f32 {
        0.45 + 0.55 * height
    }
}

/// Leaf-sized relief per metre: a leaf or two across a nub.
const LEAF_FREQUENCY: f32 = 22.0;

/// Light the other lobes keep from a point on one: each a ball of its mean
/// radius seen from the point, weighted by how squarely the point's surface
/// faces it.
fn lobe_occlusion(
    point: Point3<f32>,
    normal: Vector3<f32>,
    index: usize,
    shape: &FoliageShape,
) -> f32 {
    let mut blocked = 0.0;
    for (other, lobe) in shape.lobes.iter().enumerate() {
        if other == index {
            continue;
        }
        let to = lobe.centre - point;
        let radius = lobe.mean_radius();
        let distance = to.magnitude().max(radius);
        let facing = (normal.dot(&to) / distance).max(0.0);
        blocked += (radius / distance).powi(2) * facing;
    }
    (1.0 - 0.7 * blocked).clamp(0.3, 1.0)
}

/// Vertex normals from the faces round each vertex, area-weighted.
pub(crate) fn smooth_normals(vertices: &mut [Vertex], indices: &[u32]) {
    let mut sums = vec![Vector3::zeros(); vertices.len()];
    for face in indices.chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|i| face[i] as usize);
        let n = (vertices[b].pos - vertices[a].pos).cross(&(vertices[c].pos - vertices[a].pos));
        sums[a] += n;
        sums[b] += n;
        sums[c] += n;
    }
    for (vertex, sum) in vertices.iter_mut().zip(sums) {
        if let Some(n) = sum.try_normalize(1e-12) {
            vertex.normal = n;
        }
    }
}

/// The skin without the vertices no kept triangle uses.
fn compact(skin: Skin) -> Skin {
    let mut remap = vec![u32::MAX; skin.vertices.len()];
    let mut vertices = Vec::new();
    let indices = skin
        .indices
        .iter()
        .map(|&i| {
            let slot = &mut remap[i as usize];
            if *slot == u32::MAX {
                *slot = vertices.len() as u32;
                vertices.push(skin.vertices[i as usize]);
            }
            *slot
        })
        .collect();
    Skin { vertices, indices }
}

fn opaque(rgb: Vector3<f32>) -> Vector4<f32> {
    Vector4::new(rgb.x, rgb.y, rgb.z, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foliage::shape::ShrubSpec;

    /// The skin reaches the lobes' radius on its nub tops and no further,
    /// keeps out of other lobes' insides, and every triangle faces out.
    #[test]
    fn the_skin_wraps_the_lobes() {
        let shape = ShrubSpec::with_radius(0.6).grow(3);
        let spec = SkinSpec::default();
        let skin = spec.grow(&shape, 3);
        assert!(!skin.indices.is_empty());
        let reach = shape.bounding_radius() * (1.0 + spec.roughness) + spec.leaf_relief;
        for v in &skin.vertices {
            assert!(v.pos.magnitude() <= reach + 1e-4);
            assert!((0.0..=1.0).contains(&v.ao));
        }
        // Buried faces are dropped: fewer than every lobe's whole sphere.
        let whole = shape.lobes.len() * generate_icosphere(spec.subdivisions).indices.len();
        assert!(skin.indices.len() < whole);
        let outward = skin
            .indices
            .chunks_exact(3)
            .filter(|f| {
                let [a, b, c] = [0, 1, 2].map(|i| skin.vertices[f[i] as usize].pos);
                (b - a)
                    .cross(&(c - a))
                    .dot(&skin.vertices[f[0] as usize].normal)
                    > 0.0
            })
            .count();
        assert!(
            outward * 100 > skin.indices.len() / 3 * 98,
            "faces wind inward"
        );
    }

    /// A tall lobe's skin is stretched to its height: its nub tops reach its
    /// semi-axes and no further.
    #[test]
    fn a_tall_lobe_grows_a_tall_skin() {
        let lobe = Lobe {
            radii: Vector3::new(0.4, 1.5, 0.4),
            ..Lobe::round(Point3::origin(), 0.4)
        };
        let shape = FoliageShape { lobes: vec![lobe] };
        let spec = SkinSpec::default();
        let skin = spec.grow(&shape, 1);
        let top = skin
            .vertices
            .iter()
            .map(|v| v.pos.y)
            .fold(f32::MIN, f32::max);
        let side = skin
            .vertices
            .iter()
            .map(|v| v.pos.x)
            .fold(f32::MIN, f32::max);
        let slack = 1.0 + spec.roughness;
        assert!(
            top > 1.4 && top < 1.5 * slack + spec.leaf_relief,
            "top {top}"
        );
        assert!(
            side > 0.36 && side < 0.4 * slack + spec.leaf_relief,
            "side {side}"
        );
    }
}
