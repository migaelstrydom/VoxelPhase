//! How a plant's limbs are drawn: the bark's painted texture and the limb's
//! mesh, in the manner of the painted gardens' trees, slender, smooth and
//! pale, warm grey-ochre with a blush of pink.
//!
//! ```text
//!   BarkSpec ──texture(seed)──▶ RGBA tile: colour, and height in alpha
//!            │                    (fibre streaks, shallow fissures, ring marks)
//!            └──mesh(limb)─────▶ a tapered, gently curving tube, flared at a
//!                                 trunk's foot and collared where a branch joins
//! ```
//!
//! The mesh is drawn only: the limb collides as its straight capsule
//! (`Limb::collider`), which the curve and flare stay close to. The texture
//! carries the detail a mesh of a few hundred triangles cannot: its alpha is
//! height, which the shader turns into a detail normal
//! (`Material::with_relief`).

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};

use super::limbs::Limb;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::{fbm_perlin_2d_periodic_xy, perlin_2d_periodic_xy};

/// How a limb's bark looks.
#[derive(Debug, Clone, Copy)]
pub struct BarkSpec {
    /// Texels along each side of the square tile.
    pub texture_size: u32,
    /// The bark's palest colour, and its warmer, pinker one, mixed in broad
    /// patches across the tile.
    pub pale: [f32; 3],
    pub blush: [f32; 3],
    /// The colour of a fissure's floor.
    pub fissure: [f32; 3],
    /// How deep the deepest fissure cuts, in tile widths: a tile is once
    /// round a limb, so on a trunk a few millimetres.
    pub relief_depth: f32,
    /// Sides round a limb's tube.
    pub sides: u32,
    /// Most distance between a tube's rings along a limb, m.
    pub ring_spacing: f32,
    /// A limb's radius at its tip as a share of its radius at its base.
    pub taper: f32,
    /// How far a limb's line strays from straight at its middle, as a share
    /// of its radius: gently, so its capsule still holds it.
    pub sway: f32,
    /// How much wider a trunk is at the ground, as a share of its radius.
    pub flare: f32,
    /// How high above the ground the flare dies away, m.
    pub flare_height: f32,
    /// How much darker the bark is at the ground than a metre up.
    pub foot_shade: f32,
}

impl Default for BarkSpec {
    fn default() -> Self {
        Self {
            texture_size: 256,
            pale: [0.90, 0.83, 0.74],
            blush: [0.92, 0.76, 0.68],
            fissure: [0.68, 0.56, 0.48],
            relief_depth: 0.012,
            sides: 10,
            ring_spacing: 0.3,
            taper: 0.7,
            sway: 0.6,
            flare: 0.9,
            flare_height: 0.14,
            foot_shade: 0.25,
        }
    }
}

/// Frequencies, in features per tile, of the texture's fields. Whole
/// numbers, so each tiles across the tile's edges.
const STREAKS: (i32, i32) = (18, 2);
const PATCHES: (i32, i32) = (3, 3);
const FISSURES: (i32, i32) = (12, 2);
const RINGS: (i32, i32) = (3, 14);

impl BarkSpec {
    /// The bark tile, RGBA by rows: colour, and height in alpha, `255` at the
    /// surface and `0` at the deepest fissure. It tiles both ways.
    pub fn texture(&self, seed: u32) -> Vec<u8> {
        let size = self.texture_size;
        let mut pixels = Vec::with_capacity((size * size * 4) as usize);
        let field = |(fu, fv): (i32, i32), u: f32, v: f32, octaves: u32, salt: u32| {
            fbm_perlin_2d_periodic_xy(
                u * fu as f32,
                v * fv as f32,
                octaves,
                0.5,
                2.0,
                seed.wrapping_add(salt),
                Some((fu, fv)),
            )
        };
        for y in 0..size {
            for x in 0..size {
                let (u, v) = (x as f32 / size as f32, y as f32 / size as f32);
                let streak = field(STREAKS, u, v, 3, 1);
                let patch = field(PATCHES, u, v, 2, 2);
                // A fissure is where a wandering ridge of noise peaks: thin,
                // long, mostly along the limb.
                let ridge = 1.0 - (2.0 * field(FISSURES, u, v, 2, 3) - 1.0).abs();
                let fissure = smoothstep(0.94, 0.99, ridge);
                // Ring marks: short dashes across the limb.
                let ring_field = perlin_2d_periodic_xy(
                    u * RINGS.0 as f32,
                    v * RINGS.1 as f32,
                    seed.wrapping_add(4),
                    Some(RINGS),
                );
                let ring = smoothstep(0.78, 0.9, ring_field);

                let blush = smoothstep(0.35, 0.65, patch);
                let fibre = 0.95 + 0.1 * streak;
                let colour: [f32; 3] = std::array::from_fn(|c| {
                    let base = lerp(self.pale[c], self.blush[c], blush) * fibre;
                    let marked = lerp(base, self.fissure[c], 0.25 * ring);
                    lerp(marked, self.fissure[c], fissure)
                });
                let height = 1.0 - 0.5 * fissure - 0.15 * ring - 0.1 * (1.0 - streak);
                pixels.extend(colour.iter().map(|c| to_byte(*c)));
                pixels.push(to_byte(height));
            }
        }
        pixels
    }

    /// `limb` drawn as a tube, centred on its midpoint, `seed` choosing the
    /// way it curves. A trunk (a limb with no parent) flares where it meets
    /// the ground, at height 0 in the plant's frame; a branch thickens where
    /// it joins. Its ends are open: a trunk's foot is in the ground and every
    /// tip is inside leaves.
    pub fn mesh(&self, limb: &Limb, seed: u32) -> (Vec<Vertex>, Vec<u32>) {
        let along = limb.tip - limb.base;
        let length = along.magnitude();
        let axis = along / length;
        let turn = UnitQuaternion::rotation_between(&Vector3::y(), &axis)
            .unwrap_or_else(UnitQuaternion::identity);
        let (side, across) = (turn * Vector3::x(), turn * Vector3::z());
        let middle = limb.midpoint().coords;
        // Square texels: a tile is once round the limb at its base, and the
        // same length along it.
        let circumference = TAU * limb.radius;
        let phase = (seed % 1000) as f32 * 0.37;

        let stations = self.stations(limb, length);
        let radius_at = |s: f32| {
            let t = s / length;
            let mut r = limb.radius * lerp(1.0, self.taper, t);
            let height = (limb.base + axis * s).y;
            if limb.parent.is_none() {
                r *= 1.0 + self.flare * (-(height.max(0.0)) / self.flare_height).exp();
            } else {
                r *= 1.0 + 0.3 * (-s / (3.0 * limb.radius)).exp();
            }
            r
        };
        let centre_at = |s: f32| {
            let t = s / length;
            let envelope = (PI * t).sin();
            let wander = side * (TAU * t + phase).sin() + across * (PI * t + 0.6 * phase).cos();
            limb.base.coords + axis * s + wander * (self.sway * limb.radius * envelope)
        };

        let ring = self.sides + 1;
        let mut vertices = Vec::with_capacity(stations.len() * ring as usize);
        for (k, &s) in stations.iter().enumerate() {
            let r = radius_at(s);
            let (before, after) = (
                stations[k.saturating_sub(1)],
                stations[(k + 1).min(stations.len() - 1)],
            );
            let slope = (radius_at(after) - radius_at(before)) / (after - before).max(1e-4);
            let height = (limb.base + axis * s).y;
            let shade = 1.0 - self.foot_shade * (1.0 - height.clamp(0.0, 1.0));
            let centre = centre_at(s);
            for j in 0..ring {
                let angle = TAU * j as f32 / self.sides as f32;
                let radial = side * angle.cos() + across * angle.sin();
                let normal = (radial - axis * slope).normalize();
                vertices.push(Vertex {
                    pos: centre + radial * r - middle,
                    color: Vector4::new(shade, shade, shade, 1.0),
                    tex_coords: Vector2::new(j as f32 / self.sides as f32, s / circumference),
                    normal,
                    ao: 1.0,
                });
            }
        }
        let mut indices = Vec::with_capacity((stations.len() - 1) * self.sides as usize * 6);
        for k in 0..stations.len() as u32 - 1 {
            for j in 0..self.sides {
                let a = k * ring + j;
                let b = a + ring;
                indices.extend_from_slice(&[a, b, a + 1, b, b + 1, a + 1]);
            }
        }
        (vertices, indices)
    }

    /// Where along a limb its rings stand, base to tip: evenly at most
    /// `ring_spacing` apart, and closer where a trunk flares.
    fn stations(&self, limb: &Limb, length: f32) -> Vec<f32> {
        let count = (length / self.ring_spacing).ceil().max(2.0) as usize;
        let mut stations: Vec<f32> = (0..=count)
            .map(|i| length * i as f32 / count as f32)
            .collect();
        if limb.parent.is_none() {
            let axis_y = (limb.tip - limb.base).y / length;
            if axis_y > 0.0 {
                let ground = -limb.base.y / axis_y;
                for height in [0.0, 0.5, 1.0, 2.0] {
                    let s = ground + height * self.flare_height / axis_y;
                    if s > 0.0 && s < length {
                        stations.push(s);
                    }
                }
            }
        }
        stations.sort_by(f32::total_cmp);
        stations.dedup_by(|a, b| (*a - *b).abs() < 0.02);
        stations
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn to_byte(value: f32) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0).round() as u8
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    fn trunk() -> Limb {
        Limb {
            base: Point3::new(0.0, -0.3, 0.0),
            tip: Point3::new(0.1, 2.7, 0.0),
            radius: 0.07,
            parent: None,
        }
    }

    /// Every triangle faces out, the mesh is a few hundred triangles, and it
    /// stays near the capsule it collides as, but for the flare at the
    /// ground.
    #[test]
    fn a_trunk_is_a_light_tube_round_its_capsule() {
        let limb = trunk();
        let (vertices, indices) = BarkSpec::default().mesh(&limb, 7);
        assert!(indices.len() / 3 < 400, "{} triangles", indices.len() / 3);
        let axis = (limb.tip - limb.base).normalize();
        let middle = limb.midpoint();
        for face in indices.chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|i| vertices[face[i] as usize].pos);
            let n = (b - a).cross(&(c - a));
            let out = (a + b + c) / 3.0;
            let radial = out - axis * out.dot(&axis);
            assert!(n.dot(&radial) > 0.0, "a face winds inward");
        }
        for v in &vertices {
            let world = middle + v.pos;
            let along = (world - limb.base).dot(&axis);
            let off = ((world - limb.base) - axis * along).magnitude();
            if world.y > 0.4 {
                assert!(off < 2.0 * limb.radius, "{off:.3} m off the capsule's line");
            }
        }
    }

    /// The tile wraps: its left column continues its right, its top row its
    /// bottom.
    #[test]
    fn the_bark_tiles() {
        let spec = BarkSpec {
            texture_size: 64,
            ..BarkSpec::default()
        };
        let pixels = spec.texture(3);
        let size = spec.texture_size as usize;
        let at = |x: usize, y: usize| pixels[(y * size + x) * 4] as i32;
        let seam = |a: i32, b: i32| (a - b).abs();
        let worst_wrap = (0..size)
            .map(|y| seam(at(0, y), at(size - 1, y)))
            .chain((0..size).map(|x| seam(at(x, 0), at(x, size - 1))))
            .max()
            .unwrap();
        let worst_inside = (0..size)
            .map(|y| seam(at(size / 2, y), at(size / 2 - 1, y)))
            .max()
            .unwrap();
        assert!(
            worst_wrap <= worst_inside + 8,
            "seam {worst_wrap} against {worst_inside} inside"
        );
    }
}
