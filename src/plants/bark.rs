//! How a plant's limbs are drawn: the bark's painted texture and the limb's
//! mesh, in the manner of the painted gardens' trees, slender, smooth and
//! pale, warm grey-ochre with a blush of pink.
//!
//! ```text
//!   BarkSpec ──texture(seed)──▶ RGBA tile: colour, and height in alpha
//!            │                    (fibre streaks, shallow fissures, ring marks)
//!            └──mesh(limb)─────▶ a tapered, gently curving tube, closed at both
//!                                 ends, flared at a trunk's foot, collared where a
//!                                 branch joins, splintered where one snapped off
//! ```
//!
//! The mesh is drawn only: the limb collides as its straight capsule
//! (`Limb::collider`), which the curve and flare stay close to. The texture
//! carries the detail a mesh of a few hundred triangles cannot: its alpha is
//! height, which the shader turns into a detail normal
//! (`Material::with_relief`).

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::limbs::{Limb, LimbKind};
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
    /// A root's radius at its tip as a share of its radius at the trunk.
    pub root_taper: f32,
    /// How far the longest splinter of a snapped branch stands proud of the
    /// break, as a share of the branch's radius.
    pub splinter: f32,
    /// The colour of fresh wood where a branch snapped, multiplying the
    /// bark tile as the bark's own shade does.
    pub heartwood: [f32; 3],
    /// How far a limb's line strays from straight at its middle, as a share
    /// of its radius: gently, so its capsule still holds it.
    pub sway: f32,
    /// How much wider a trunk is at the ground, as a share of its radius.
    pub flare: f32,
    /// How far above and below the ground the flare dies away, m: below
    /// it, the trunk narrows into a tap root among its roots.
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
            root_taper: 0.3,
            splinter: 1.0,
            heartwood: [1.0, 0.94, 0.72],
            sway: 0.6,
            flare: 0.9,
            flare_height: 0.14,
            foot_shade: 0.25,
        }
    }
}

/// How a limb's base is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimbBase {
    /// Rounded shut: buried, or inside what it grows from.
    Rounded,
    /// Snapped off what it grew from: splinters round a broken face of
    /// fresh wood.
    Snapped,
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

    /// `limb` drawn as a closed tube, centred on its midpoint, `seed`
    /// choosing the way it curves and splinters, its base drawn as `base`
    /// says and its tip rounded shut. A trunk flares where it meets the
    /// ground, at height 0 in the plant's frame; a branch thickens where it
    /// joins; a root tapers.
    pub fn mesh(&self, limb: &Limb, seed: u32, base: LimbBase) -> (Vec<Vertex>, Vec<u32>) {
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

        let radius_at = |s: f32| {
            let t = s / length;
            let height = (limb.base + axis * s).y;
            match limb.kind {
                LimbKind::Trunk => {
                    limb.radius
                        * lerp(1.0, self.taper, t)
                        * (1.0 + self.flare * (-height.abs() / self.flare_height).exp())
                }
                LimbKind::Branch => {
                    limb.radius
                        * lerp(1.0, self.taper, t)
                        * (1.0 + 0.3 * (-s / (3.0 * limb.radius)).exp())
                }
                LimbKind::Root => limb.radius * lerp(1.0, self.root_taper, t),
            }
        };
        let centre_at = |s: f32| {
            let t = s / length;
            let envelope = (PI * t).sin();
            let wander = side * (TAU * t + phase).sin() + across * (PI * t + 0.6 * phase).cos();
            limb.base.coords + axis * s + wander * (self.sway * limb.radius * envelope)
        };
        let shade_at = |centre: Vector3<f32>| {
            let shade = 1.0 - self.foot_shade * (1.0 - centre.y.clamp(0.0, 1.0));
            Vector4::new(shade, shade, shade, 1.0)
        };

        // The rings, base to tip: a rounded or snapped base, the tube, and a
        // rounded tip.
        let stations = self.stations(limb, length);
        let mut rings = Vec::with_capacity(stations.len() + 2 * DOME_RINGS + 1);
        let (r0, r1) = (radius_at(0.0), radius_at(length));
        let (c0, c1) = (centre_at(0.0), centre_at(length));
        let snapped = base == LimbBase::Snapped;
        if !snapped {
            rings.extend(
                dome(c0, axis, -1.0, r0, 1.0 / circumference)
                    .into_iter()
                    .rev(),
            );
        }
        for (k, &s) in stations.iter().enumerate() {
            let (before, after) = (
                stations[k.saturating_sub(1)],
                stations[(k + 1).min(stations.len() - 1)],
            );
            let slope = (radius_at(after) - radius_at(before)) / (after - before).max(1e-4);
            rings.push(Ring {
                centre: centre_at(s),
                radius: radius_at(s),
                normal: (1.0, -slope),
                v: s / circumference,
            });
        }
        rings.extend(
            dome(c1, axis, 1.0, r1, 1.0 / circumference)
                .into_iter()
                .map(|ring| Ring {
                    v: ring.v + length / circumference,
                    ..ring
                }),
        );

        let ring_size = self.sides + 1;
        let mut vertices = Vec::with_capacity((rings.len() + 1) * ring_size as usize);
        let mut indices = Vec::new();
        let teeth = snapped.then(|| self.teeth(r0, seed));
        if let Some(teeth) = &teeth {
            // The bark runs on into the splinters' points.
            for j in 0..ring_size {
                let radial = radial_at(side, across, j, self.sides);
                let tooth = teeth[(j % self.sides) as usize];
                vertices.push(Vertex {
                    pos: c0 - axis * tooth + radial * r0 - middle,
                    color: shade_at(c0),
                    tex_coords: Vector2::new(j as f32 / self.sides as f32, -tooth / circumference),
                    normal: radial,
                    ao: 1.0,
                });
            }
        }
        for ring in &rings {
            for j in 0..ring_size {
                let radial = radial_at(side, across, j, self.sides);
                vertices.push(Vertex {
                    pos: ring.centre + radial * ring.radius - middle,
                    color: shade_at(ring.centre),
                    tex_coords: Vector2::new(j as f32 / self.sides as f32, ring.v),
                    normal: (radial * ring.normal.0 + axis * ring.normal.1).normalize(),
                    ao: 1.0,
                });
            }
        }
        let ring_count = rings.len() as u32 + teeth.is_some() as u32;
        for k in 0..ring_count - 1 {
            for j in 0..self.sides {
                let a = k * ring_size + j;
                let b = a + ring_size;
                indices.extend_from_slice(&[a, b, a + 1, b, b + 1, a + 1]);
            }
        }
        if let Some(teeth) = &teeth {
            self.break_face(
                &mut vertices,
                &mut indices,
                BreakFace {
                    centre: c0,
                    axis,
                    side,
                    across,
                    radius: r0,
                    teeth,
                    middle,
                    texel: circumference,
                },
            );
        }
        (vertices, indices)
    }

    /// How far each splinter round a snapped end stands proud of the break,
    /// one per side: a few long, most short.
    fn teeth(&self, radius: f32, seed: u32) -> Vec<f32> {
        let mut rng = StdRng::seed_from_u64(seed as u64 ^ 0x5a1_1e7);
        (0..self.sides)
            .map(|_| {
                let t: f32 = rng.gen_range(0.0..1.0);
                radius * self.splinter * (0.15 + 0.85 * t * t)
            })
            .collect()
    }

    /// The broken face of a snapped end: fresh wood, flat-shaded facets from
    /// the splinters' points down to a hollow in the middle.
    fn break_face(&self, vertices: &mut Vec<Vertex>, indices: &mut Vec<u32>, face: BreakFace) {
        let BreakFace {
            centre,
            axis,
            side,
            across,
            radius,
            teeth,
            middle,
            texel,
        } = face;
        let mean = teeth.iter().sum::<f32>() / teeth.len() as f32;
        let hollow = centre - axis * (0.4 * mean);
        let rim = |j: u32| {
            let radial = radial_at(side, across, j, self.sides);
            centre - axis * teeth[(j % self.sides) as usize] + radial * radius
        };
        let colour = Vector4::new(self.heartwood[0], self.heartwood[1], self.heartwood[2], 1.0);
        let uv = |p: Vector3<f32>| {
            let d = p - centre;
            Vector2::new(d.dot(&side) / texel + 0.5, d.dot(&across) / texel + 0.5)
        };
        for j in 0..self.sides {
            let mut corners = [hollow, rim(j), rim(j + 1)];
            let mut normal = (corners[1] - corners[0]).cross(&(corners[2] - corners[0]));
            if normal.dot(&axis) > 0.0 {
                corners.swap(1, 2);
                normal = -normal;
            }
            let normal = normal.normalize();
            let first = vertices.len() as u32;
            for corner in corners {
                vertices.push(Vertex {
                    pos: corner - middle,
                    color: colour,
                    tex_coords: uv(corner),
                    normal,
                    ao: 1.0,
                });
            }
            indices.extend_from_slice(&[first, first + 1, first + 2]);
        }
    }

    /// Where along a limb its rings stand, base to tip: evenly at most
    /// `ring_spacing` apart, and closer where a trunk flares.
    fn stations(&self, limb: &Limb, length: f32) -> Vec<f32> {
        let count = (length / self.ring_spacing).ceil().max(2.0) as usize;
        let mut stations: Vec<f32> = (0..=count)
            .map(|i| length * i as f32 / count as f32)
            .collect();
        if limb.kind == LimbKind::Trunk {
            let axis_y = (limb.tip - limb.base).y / length;
            if axis_y > 0.0 {
                let ground = -limb.base.y / axis_y;
                for height in [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0] {
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

/// Rings in each rounded end, between the tube's last ring and the pole.
const DOME_RINGS: usize = 3;
/// How far a rounded end stands past the tube's last ring, as a share of
/// its radius: a flattened cap, not a hemisphere.
const DOME_HEIGHT: f32 = 0.6;

/// One ring of a limb's mesh: a circle round its line.
#[derive(Debug, Clone, Copy)]
struct Ring {
    centre: Vector3<f32>,
    radius: f32,
    /// The surface's normal there, unnormalised: how much of it points out
    /// from the limb's line, and how much along it towards the tip.
    normal: (f32, f32),
    /// Texture coordinate along the limb.
    v: f32,
}

/// A snapped end's broken face, as `BarkSpec::break_face` draws it.
struct BreakFace<'a> {
    /// The centre of the tube's last whole ring.
    centre: Vector3<f32>,
    /// Along the limb, base to tip.
    axis: Vector3<f32>,
    /// The two directions across the limb the rings are laid out by.
    side: Vector3<f32>,
    across: Vector3<f32>,
    /// The tube's radius there.
    radius: f32,
    /// How far each splinter stands proud.
    teeth: &'a [f32],
    /// Subtracted from every position: the limb's midpoint.
    middle: Vector3<f32>,
    /// Metres per texture tile.
    texel: f32,
}

/// The rings rounding off an end at `centre` of a tube of `radius` along
/// `axis`, facing `axis * sign` (`sign` is 1 at the tip, -1 at the base):
/// half an ellipsoid, from beside the tube's last ring to the pole, a ring
/// of radius zero. `v_per_metre` carries the texture on.
fn dome(
    centre: Vector3<f32>,
    axis: Vector3<f32>,
    sign: f32,
    radius: f32,
    v_per_metre: f32,
) -> Vec<Ring> {
    (1..=DOME_RINGS)
        .map(|i| {
            let angle = FRAC_PI_2 * i as f32 / DOME_RINGS as f32;
            let out = DOME_HEIGHT * radius * angle.sin();
            Ring {
                centre: centre + axis * (sign * out),
                radius: radius * angle.cos(),
                normal: (angle.cos(), sign * angle.sin() / DOME_HEIGHT),
                v: sign * out * v_per_metre,
            }
        })
        .collect()
}

/// The unit direction out from a limb's line to the `j`th of `sides`
/// vertices round a ring.
fn radial_at(side: Vector3<f32>, across: Vector3<f32>, j: u32, sides: u32) -> Vector3<f32> {
    let angle = TAU * j as f32 / sides as f32;
    side * angle.cos() + across * angle.sin()
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
            kind: LimbKind::Trunk,
        }
    }

    fn branch() -> Limb {
        Limb {
            base: Point3::new(0.0, 1.5, 0.0),
            tip: Point3::new(1.2, 2.3, 0.3),
            radius: 0.045,
            parent: Some(0),
            kind: LimbKind::Branch,
        }
    }

    fn root() -> Limb {
        Limb {
            base: Point3::new(0.0, -0.04, 0.0),
            tip: Point3::new(0.5, -0.25, 0.2),
            radius: 0.042,
            parent: Some(0),
            kind: LimbKind::Root,
        }
    }

    /// A mesh's triangles by welded corners, the degenerate ones (a pole's)
    /// left out.
    fn welded(vertices: &[Vertex], indices: &[u32]) -> Vec<[(i64, i64, i64); 3]> {
        let key = |i: u32| {
            let p = vertices[i as usize].pos * 1e4;
            (p.x.round() as i64, p.y.round() as i64, p.z.round() as i64)
        };
        indices
            .chunks_exact(3)
            .map(|face| [key(face[0]), key(face[1]), key(face[2])])
            .filter(|[a, b, c]| a != b && b != c && c != a)
            .collect()
    }

    /// Every limb's mesh is a closed solid, wound outward: each edge is met
    /// once each way, and what it encloses is positive and about its
    /// capsule's volume.
    #[test]
    fn every_limb_is_drawn_closed() {
        let spec = BarkSpec::default();
        for (limb, base) in [
            (trunk(), LimbBase::Rounded),
            (branch(), LimbBase::Rounded),
            (branch(), LimbBase::Snapped),
            (root(), LimbBase::Rounded),
        ] {
            let (vertices, indices) = spec.mesh(&limb, 7, base);
            let faces = welded(&vertices, &indices);
            let mut edges = std::collections::HashMap::new();
            for face in &faces {
                for k in 0..3 {
                    *edges.entry((face[k], face[(k + 1) % 3])).or_insert(0) += 1;
                }
            }
            for (&(a, b), &count) in &edges {
                assert_eq!(
                    count, 1,
                    "{:?} {base:?}: an edge met twice one way",
                    limb.kind
                );
                assert_eq!(
                    edges.get(&(b, a)),
                    Some(&1),
                    "{:?} {base:?}: an open edge",
                    limb.kind
                );
            }
            let volume: f32 = indices
                .chunks_exact(3)
                .map(|f| {
                    let [a, b, c] = [0, 1, 2].map(|i| vertices[f[i] as usize].pos);
                    a.dot(&b.cross(&c)) / 6.0
                })
                .sum();
            let length = (limb.tip - limb.base).magnitude();
            let capsule = PI * limb.radius * limb.radius * length;
            assert!(
                volume > 0.2 * capsule && volume < 4.0 * capsule,
                "{:?} {base:?}: encloses {volume:.5} m³ against {capsule:.5}",
                limb.kind
            );
        }
    }

    /// A trunk is a few hundred triangles and stays near the capsule it
    /// collides as, but for the flare at the ground.
    #[test]
    fn a_trunk_is_a_light_tube_round_its_capsule() {
        let limb = trunk();
        let (vertices, indices) = BarkSpec::default().mesh(&limb, 7, LimbBase::Rounded);
        assert!(indices.len() / 3 < 500, "{} triangles", indices.len() / 3);
        let axis = (limb.tip - limb.base).normalize();
        let middle = limb.midpoint();
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
