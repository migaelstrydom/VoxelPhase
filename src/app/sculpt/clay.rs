//! Clay: a form modelled as lumps pressed together and smoothed, hollowed
//! and carved, then skinned with a dense mesh.
//!
//! ```text
//!   lumps (balls, limbs) ──smooth union──▶ a solid, as a distance field
//!        ├── relief: folds, lumps and scales raised on its surface
//!        ├── hollow: only a shell of it kept
//!        └── cuts: lumps carved out of it (sockets, mouths, breaks)
//!   field ──skin(spacing)──▶ mesh: a vertex per cell the surface crosses,
//!                            its normal the field's gradient, its colour the
//!                            lumps' colours blended, its occlusion how
//!                            closed-in the surface is there
//! ```
//!
//! Where [`super::parts`] sets ellipsoids side by side and shades the crease
//! between them, clay has no crease: a toe runs into a foot and a foot into
//! an ankle as flesh does. It has no texture coordinates either, so what is
//! painted on it is painted on its vertices ([`super::paint`]) and its fine
//! grain comes from its material.

use nalgebra::{Vector2, Vector3, Vector4};
use rayon::prelude::*;
use rustc_hash::FxHashMap;

use super::parts::Ellipsoid;
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

/// Cells of the coarse grid that finds where the surface is, to a side, in
/// cells of the grid the skin is cut from.
const COARSE: usize = 4;

/// Most samples a skin's grid may hold: past this the spacing is opened up.
const MOST_SAMPLES: usize = 60_000_000;

/// How far over the surface a lump's colour carries into its neighbour's, as
/// a multiple of the blend between them.
const COLOUR_CARRY: f32 = 0.6;

/// Distances along the normal at which a vertex's occlusion is judged, in
/// cells, and how much of the light a wholly closed-in vertex keeps.
const OCCLUSION_REACH: [f32; 4] = [1.5, 3.0, 6.0, 12.0];
const OCCLUSION_FLOOR: f32 = 0.25;

/// Radius of the ball that stands for a half-space, m: its side bows less
/// than a millimetre across half a metre.
const HALF_SPACE_RADIUS: f32 = 50.0;

/// One lump of clay.
#[derive(Clone, Copy, Debug)]
pub enum Lump {
    /// A rounded mass.
    Ball(Ellipsoid),
    /// A limb: round in section, `radii.0` thick at `from` and `radii.1` at
    /// `to`, rounded at both ends.
    Limb {
        from: Vector3<f32>,
        to: Vector3<f32>,
        radii: (f32, f32),
    },
}

impl Lump {
    /// A ball of `radius` at `centre`.
    pub fn ball(radius: f32, centre: (f32, f32, f32)) -> Self {
        Lump::Ball(Ellipsoid::ball(
            radius,
            Vector3::new(centre.0, centre.1, centre.2),
        ))
    }

    /// An ellipsoid of `radii` at `centre`, on the form's axes.
    pub fn mass(radii: (f32, f32, f32), centre: (f32, f32, f32)) -> Self {
        Lump::Ball(Ellipsoid::new(radii, centre))
    }

    /// A limb from `from` to `to`.
    pub fn limb(from: (f32, f32, f32), to: (f32, f32, f32), radii: (f32, f32)) -> Self {
        Lump::Limb {
            from: Vector3::new(from.0, from.1, from.2),
            to: Vector3::new(to.0, to.1, to.2),
            radii,
        }
    }

    /// Everything behind the plane through `point` that faces along
    /// `facing`, for cutting a form off flat: a ball so large that its side
    /// is a plane across anything a creature is made of. Only for
    /// [`Clay::cut`]; pressed on, it would be the whole form.
    pub fn half_space(point: Vector3<f32>, facing: Vector3<f32>) -> Self {
        let behind = point - facing.normalize() * HALF_SPACE_RADIUS;
        Lump::Ball(Ellipsoid::ball(HALF_SPACE_RADIUS, behind))
    }

    /// How far outside the lump `p` is; negative inside.
    fn distance(&self, p: Vector3<f32>) -> f32 {
        match *self {
            Lump::Ball(ball) => ball.distance_outside(p),
            Lump::Limb { from, to, radii } => round_cone(p, from, to, radii.0, radii.1),
        }
    }

    /// A sphere that holds it: its centre and radius.
    fn bounds(&self) -> (Vector3<f32>, f32) {
        match *self {
            Lump::Ball(ball) => (ball.centre, ball.radii.max()),
            Lump::Limb { from, to, radii } => (
                0.5 * (from + to),
                0.5 * (to - from).norm() + radii.0.max(radii.1),
            ),
        }
    }
}

/// Distance to a cone with rounded ends (Quilez's exact form).
fn round_cone(p: Vector3<f32>, a: Vector3<f32>, b: Vector3<f32>, r1: f32, r2: f32) -> f32 {
    let ba = b - a;
    let l2 = ba.norm_squared();
    if l2 < 1e-12 {
        return (p - a).norm() - r1.max(r2);
    }
    let rr = r1 - r2;
    let a2 = l2 - rr * rr;
    let il2 = 1.0 / l2;
    let pa = p - a;
    let y = pa.dot(&ba);
    let z = y - l2;
    let x2 = (pa * l2 - ba * y).norm_squared();
    let y2 = y * y * l2;
    let z2 = z * z * l2;
    let k = rr.signum() * rr * rr * x2;
    if z.signum() * a2 * z2 > k {
        return (x2 + z2).sqrt() * il2 - r2;
    }
    if y.signum() * a2 * y2 < k {
        return (x2 + y2).sqrt() * il2 - r1;
    }
    ((x2 * a2 * il2).sqrt() + y * rr) * il2 - r1
}

/// The lesser of two distances, rounded off over `blend` where they meet.
fn smooth_min(a: f32, b: f32, blend: f32) -> f32 {
    if blend <= 0.0 {
        return a.min(b);
    }
    let h = (blend - (a - b).abs()).max(0.0) / blend;
    a.min(b) - h * h * blend * 0.25
}

/// Something raised on, or pressed into, a form's surface.
#[derive(Clone, Copy, Debug)]
pub enum Relief {
    /// Soft swellings in every direction: fat, lumps, an uneven casting.
    Lumps {
        /// Swellings to the metre.
        frequency: f32,
        /// How far they stand out and sink in, m.
        depth: f32,
        seed: u32,
    },
    /// Ridges that run along `along`: folds of cloth, wrinkles of skin.
    Folds {
        /// The way the folds run.
        along: Vector3<f32>,
        /// Folds to the metre across them.
        frequency: f32,
        /// How far a fold stands out, m.
        depth: f32,
        seed: u32,
    },
    /// Close-set bumps with creases between: scales, pebbled hide.
    Scales {
        /// Width of a scale, m.
        size: f32,
        /// How deep the creases between them are, m.
        depth: f32,
    },
    /// Ridges in rings about a point, each rising gently outward and
    /// falling away at its outer edge: the growth lines of a shell.
    Rings {
        /// The point the rings are about.
        about: Vector3<f32>,
        /// From one ridge to the next, m.
        spacing: f32,
        /// How far the proudest ridge stands out, m.
        depth: f32,
        seed: u32,
    },
}

/// Of a ring of [`Relief::Rings`]: the share of its width over which it
/// falls away, how far the rings wander from round in ring widths, and how
/// many ring widths a wander is long.
const RING_FALL: f32 = 0.3;
const RING_WANDER: f32 = 0.8;
const RING_WANDER_LENGTH: f32 = 3.0;

impl Relief {
    /// How far the surface is raised at `p`, m.
    fn raise(&self, p: Vector3<f32>) -> f32 {
        match *self {
            Relief::Lumps {
                frequency,
                depth,
                seed,
            } => {
                let at = p * frequency;
                depth * fbm_perlin_3d(at.x, at.y, at.z, 3, 0.5, seed)
            }
            Relief::Folds {
                along,
                frequency,
                depth,
                seed,
            } => {
                // Squeezed along the folds, so the noise's features run that
                // way; a fold is where the field crosses zero.
                let along = along.normalize();
                let across = p - along * p.dot(&along) * 0.85;
                let at = across * frequency;
                let field = fbm_perlin_3d(at.x, at.y, at.z, 2, 0.5, seed);
                // Rounded at its crest and in its trough: a fold with an
                // edge to it is a crack.
                let off = (field.abs() * 2.5).min(1.0);
                depth * (1.0 - 2.0 * off * off * (3.0 - 2.0 * off))
            }
            Relief::Scales { size, depth } => {
                // The crease is the edge of a cell: far from the nearest
                // cell's middle.
                let at = p / size;
                let cell = at.map(f32::floor);
                let mut nearest = f32::MAX;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            let c = cell + Vector3::new(dx as f32, dy as f32, dz as f32);
                            let jitter = Vector3::new(hash(c, 1), hash(c, 2), hash(c, 3));
                            nearest = nearest.min((at - c - jitter).norm());
                        }
                    }
                }
                -depth * (nearest * 1.6).clamp(0.0, 1.0).powi(2)
            }
            Relief::Rings {
                about,
                spacing,
                depth,
                seed,
            } => {
                let at = p / (spacing * RING_WANDER_LENGTH);
                let wander = fbm_perlin_3d(at.x, at.y, at.z, 2, 0.5, seed);
                let ring = (p - about).norm() / spacing + RING_WANDER * wander;
                let within = ring - ring.floor();
                // Level with the form at both its edges, so a ring may stand
                // prouder than its neighbour without a step between them.
                let rise = (within / (1.0 - RING_FALL)).min(1.0);
                let fall = ((1.0 - within) / RING_FALL).min(1.0);
                let proud = 0.4 + 0.6 * hash(Vector3::new(ring.floor(), 0.0, 0.0), seed);
                depth * proud * rise * fall * fall * (3.0 - 2.0 * fall)
            }
        }
    }

    /// The most the raising can change over a metre, m: how much steeper
    /// than a plain distance it can make the field.
    fn steepness(&self) -> f32 {
        match *self {
            Relief::Lumps {
                frequency, depth, ..
            } => depth.abs() * frequency * 2.5,
            Relief::Folds {
                frequency, depth, ..
            } => depth.abs() * frequency * 12.5,
            Relief::Scales { size, depth } => depth.abs() / size * 3.2,
            // A ring falls away over a share of its width, and the wander
            // crowds the rings as much again.
            Relief::Rings { spacing, depth, .. } => {
                depth.abs() / spacing * (1.5 / RING_FALL) * (1.0 + RING_WANDER)
            }
        }
    }

    /// The furthest it moves the surface either way, m.
    fn reach(&self) -> f32 {
        match *self {
            Relief::Lumps { depth, .. }
            | Relief::Folds { depth, .. }
            | Relief::Scales { depth, .. }
            | Relief::Rings { depth, .. } => depth.abs(),
        }
    }
}

/// A number in 0..1 for a cell and a salt.
fn hash(cell: Vector3<f32>, salt: u32) -> f32 {
    let [x, y, z] = [cell.x, cell.y, cell.z].map(|c| c as i32 as u32);
    let mut h = x.wrapping_mul(73_856_093)
        ^ y.wrapping_mul(19_349_663)
        ^ z.wrapping_mul(83_492_791)
        ^ salt.wrapping_mul(0x9E37_79B1);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    (h >> 8) as f32 / (1u32 << 24) as f32
}

/// A lump pressed onto the form.
#[derive(Clone, Copy, Debug)]
struct Pressed {
    lump: Lump,
    /// Its colour (linear).
    colour: Vector3<f32>,
    /// How far its join with what is already there is rounded off, m.
    blend: f32,
}

/// A form in clay.
#[derive(Clone, Debug, Default)]
pub struct Clay {
    lumps: Vec<Pressed>,
    /// Lumps carved out, each with how far its edge is rounded off.
    cuts: Vec<(Lump, f32)>,
    relief: Vec<Relief>,
    /// Thickness of the shell kept, if the form is hollow.
    shell: Option<f32>,
    /// How far the join of each lump pressed on from here is rounded off.
    blend: f32,
}

impl Clay {
    /// Nothing yet, whose lumps will join over `blend` metres.
    pub fn new(blend: f32) -> Self {
        Self {
            blend,
            ..Self::default()
        }
    }

    /// From here on, lumps join over `blend` metres.
    pub fn smoothing(mut self, blend: f32) -> Self {
        self.blend = blend;
        self
    }

    /// Press `lump` on, in `colour` (linear red, green, blue).
    pub fn add(mut self, lump: Lump, colour: (f32, f32, f32)) -> Self {
        self.lumps.push(Pressed {
            lump,
            colour: Vector3::new(colour.0, colour.1, colour.2),
            blend: self.blend,
        });
        self
    }

    /// Carve `lump` out, its edge rounded off over `blend` metres.
    pub fn cut(mut self, lump: Lump, blend: f32) -> Self {
        self.cuts.push((lump, blend));
        self
    }

    /// Raise `relief` on the surface.
    pub fn relief(mut self, relief: Relief) -> Self {
        self.relief.push(relief);
        self
    }

    /// Keep only a shell `thickness` metres deep under the surface. Cuts go
    /// right through it.
    pub fn hollow(mut self, thickness: f32) -> Self {
        self.shell = Some(thickness);
        self
    }

    /// How far outside the form `p` is; negative inside.
    pub fn distance(&self, p: Vector3<f32>) -> f32 {
        let mut d = f32::MAX;
        for pressed in &self.lumps {
            d = smooth_min(d, pressed.lump.distance(p), pressed.blend);
        }
        for relief in &self.relief {
            d -= relief.raise(p);
        }
        if let Some(thickness) = self.shell {
            d = d.max(-(d + thickness));
        }
        for (cut, blend) in &self.cuts {
            d = -smooth_min(-d, cut.distance(p), *blend);
        }
        d
    }

    /// The colour of the surface at `p`: each lump's, weighed by how near
    /// its own surface is.
    fn colour(&self, p: Vector3<f32>) -> Vector3<f32> {
        let near: Vec<f32> = self.lumps.iter().map(|l| l.lump.distance(p)).collect();
        let nearest = near.iter().copied().fold(f32::MAX, f32::min);
        let (mut sum, mut weight) = (Vector3::zeros(), 0.0);
        for (pressed, d) in self.lumps.iter().zip(&near) {
            let carry = (pressed.blend * COLOUR_CARRY).max(1e-4);
            let w = (-(d - nearest) / carry).exp();
            sum += pressed.colour * w;
            weight += w;
        }
        sum / weight.max(1e-6)
    }

    /// The box that holds the form: its low and high corners.
    fn bounds(&self) -> (Vector3<f32>, Vector3<f32>) {
        let (mut low, mut high) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
        for pressed in &self.lumps {
            let (centre, radius) = pressed.lump.bounds();
            low = low.inf(&(centre - Vector3::repeat(radius)));
            high = high.sup(&(centre + Vector3::repeat(radius)));
        }
        let swell: f32 = self.relief.iter().map(Relief::reach).sum();
        (low.add_scalar(-swell), high.add_scalar(swell))
    }

    /// The form's skin, cut from a grid of about `spacing` metres, in
    /// `material`. Finer is smoother and carries more painted detail, at
    /// four times the triangles for half the spacing.
    pub fn skin(&self, spacing: f32, material: MaterialId) -> MeshPrimitive {
        let (low, high) = self.bounds();
        let extent = high - low;
        // Opened up if the grid would be too big to hold.
        let cells = |h: f32| extent.map(|side| (side / h).ceil() as usize + 2 * COARSE);
        let mut h = spacing;
        while cells(h).iter().map(|n| n + 1).product::<usize>() > MOST_SAMPLES {
            h *= 1.15;
        }
        // Whole coarse cells, with one to spare all round.
        let dims = cells(h).map(|n| n.div_ceil(COARSE) * COARSE);
        let origin = low - Vector3::repeat(COARSE as f32 * h);
        let grid = Grid { origin, h, dims };
        let samples = grid.sample(self);
        let (positions, indices) = grid.net(&samples);

        let vertices: Vec<Vertex> = positions
            .par_iter()
            .map(|&pos| {
                let step = 0.5 * h;
                let slope = |axis: Vector3<f32>| {
                    self.distance(pos + axis * step) - self.distance(pos - axis * step)
                };
                let normal = Vector3::new(
                    slope(Vector3::x()),
                    slope(Vector3::y()),
                    slope(Vector3::z()),
                )
                .try_normalize(1e-12)
                .unwrap_or_else(Vector3::y);
                // Open where the field grows as fast as the distance walked
                // out along the normal; closed in where something else is
                // met on the way.
                let mut closed = 0.0;
                let mut weight = 0.0;
                for (step, cells) in OCCLUSION_REACH.iter().enumerate() {
                    let reach = cells * h;
                    let w = 0.5f32.powi(step as i32);
                    closed +=
                        w * (1.0 - self.distance(pos + normal * reach) / reach).clamp(0.0, 1.0);
                    weight += w;
                }
                let colour = self.colour(pos);
                Vertex {
                    pos,
                    color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
                    tex_coords: Vector2::zeros(),
                    normal,
                    ao: 1.0 - (1.0 - OCCLUSION_FLOOR) * closed / weight,
                }
            })
            .collect();
        MeshPrimitive {
            vertices,
            indices,
            material,
        }
    }
}

/// The grid a skin is cut from.
struct Grid {
    /// Its low corner.
    origin: Vector3<f32>,
    /// The side of a cell, m.
    h: f32,
    /// Cells along each axis: a whole number of coarse cells.
    dims: Vector3<usize>,
}

impl Grid {
    fn point(&self, x: usize, y: usize, z: usize) -> Vector3<f32> {
        self.origin + Vector3::new(x as f32, y as f32, z as f32) * self.h
    }

    /// Samples along x and y, and in a slab of constant z.
    fn row(&self) -> usize {
        self.dims.x + 1
    }

    fn slab(&self) -> usize {
        self.row() * (self.dims.y + 1)
    }

    /// The form's distance at every point of the grid, exact wherever the
    /// surface is near and merely of the right sign elsewhere.
    fn sample(&self, clay: &Clay) -> Vec<f32> {
        // The coarse grid first: a coarse cell can hold surface only if a
        // corner of it is within its own diagonal of the surface.
        let coarse = self.dims.map(|n| n / COARSE);
        let coarse_row = coarse.x + 1;
        let coarse_slab = coarse_row * (coarse.y + 1);
        let stride = COARSE as f32 * self.h;
        let coarse_samples: Vec<f32> = (0..coarse_slab * (coarse.z + 1))
            .into_par_iter()
            .map(|index| {
                let (x, y, z) = (
                    index % coarse_row,
                    (index % coarse_slab) / coarse_row,
                    index / coarse_slab,
                );
                clay.distance(self.origin + Vector3::new(x as f32, y as f32, z as f32) * stride)
            })
            .collect();
        // The field changes by at most `steep` to the metre, so a corner
        // further than this from zero has no surface in its cell.
        let steep = 1.0 + clay.relief.iter().map(Relief::steepness).sum::<f32>();
        let swell: f32 = clay.relief.iter().map(Relief::reach).sum();
        let near = stride * 3.0f32.sqrt() * 1.5 * steep + swell;

        let (row, slab) = (self.row(), self.slab());
        (0..slab * (self.dims.z + 1))
            .into_par_iter()
            .map(|index| {
                let (x, y, z) = (index % row, (index % slab) / row, index / slab);
                let corner = (x / COARSE).min(coarse.x)
                    + (y / COARSE).min(coarse.y) * coarse_row
                    + (z / COARSE).min(coarse.z) * coarse_slab;
                let rough = coarse_samples[corner];
                if rough.abs() > near {
                    rough
                } else {
                    clay.distance(self.point(x, y, z))
                }
            })
            .collect()
    }

    /// The surface as a net: a vertex in every cell the surface crosses,
    /// where its crossings of the cell's edges average, and a quad round
    /// every edge of the grid it crosses.
    fn net(&self, samples: &[f32]) -> (Vec<Vector3<f32>>, Vec<u32>) {
        let (row, slab) = (self.row(), self.slab());
        let at = |x: usize, y: usize, z: usize| samples[x + y * row + z * slab];
        const CORNERS: [(usize, usize, usize); 8] = [
            (0, 0, 0),
            (1, 0, 0),
            (0, 1, 0),
            (1, 1, 0),
            (0, 0, 1),
            (1, 0, 1),
            (0, 1, 1),
            (1, 1, 1),
        ];
        const EDGES: [(usize, usize); 12] = [
            (0, 1),
            (2, 3),
            (4, 5),
            (6, 7),
            (0, 2),
            (1, 3),
            (4, 6),
            (5, 7),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];

        // Vertices, a slab of cells to a core.
        let found: Vec<Vec<((usize, usize, usize), Vector3<f32>)>> = (0..self.dims.z)
            .into_par_iter()
            .map(|z| {
                let mut slab_vertices = Vec::new();
                for y in 0..self.dims.y {
                    for x in 0..self.dims.x {
                        let corner = CORNERS.map(|(dx, dy, dz)| at(x + dx, y + dy, z + dz));
                        let inside = corner.iter().filter(|d| **d < 0.0).count();
                        if inside == 0 || inside == 8 {
                            continue;
                        }
                        let (mut sum, mut crossings) = (Vector3::zeros(), 0.0);
                        for (a, b) in EDGES {
                            let (da, db) = (corner[a], corner[b]);
                            if (da < 0.0) == (db < 0.0) {
                                continue;
                            }
                            let t = da / (da - db);
                            let (pa, pb) = (CORNERS[a], CORNERS[b]);
                            let from = Vector3::new(pa.0 as f32, pa.1 as f32, pa.2 as f32);
                            let to = Vector3::new(pb.0 as f32, pb.1 as f32, pb.2 as f32);
                            sum += from + (to - from) * t;
                            crossings += 1.0;
                        }
                        let within = sum / crossings;
                        slab_vertices.push(((x, y, z), self.point(x, y, z) + within * self.h));
                    }
                }
                slab_vertices
            })
            .collect();

        let mut positions = Vec::new();
        let mut vertex_of: FxHashMap<(usize, usize, usize), u32> = FxHashMap::default();
        for (cell, position) in found.into_iter().flatten() {
            vertex_of.insert(cell, positions.len() as u32);
            positions.push(position);
        }

        // A quad round each crossed edge, joining the four cells that share
        // it, wound to face out of the solid.
        let mut indices = Vec::new();
        let mut quad = |cells: [(usize, usize, usize); 4], flipped: bool| {
            let Some(corners) = cells
                .iter()
                .map(|cell| vertex_of.get(cell).copied())
                .collect::<Option<Vec<u32>>>()
            else {
                return;
            };
            let [a, b, c, d] = [corners[0], corners[1], corners[2], corners[3]];
            if flipped {
                indices.extend([a, c, b, a, d, c]);
            } else {
                indices.extend([a, b, c, a, c, d]);
            }
        };
        for z in 1..self.dims.z {
            for y in 1..self.dims.y {
                for x in 1..self.dims.x {
                    let here = at(x, y, z) < 0.0;
                    if here != (at(x + 1, y, z) < 0.0) {
                        quad(
                            [(x, y - 1, z - 1), (x, y, z - 1), (x, y, z), (x, y - 1, z)],
                            !here,
                        );
                    }
                    if here != (at(x, y + 1, z) < 0.0) {
                        quad(
                            [(x - 1, y, z - 1), (x - 1, y, z), (x, y, z), (x, y, z - 1)],
                            !here,
                        );
                    }
                    if here != (at(x, y, z + 1) < 0.0) {
                        quad(
                            [(x - 1, y - 1, z), (x, y - 1, z), (x, y, z), (x - 1, y, z)],
                            !here,
                        );
                    }
                }
            }
        }
        (positions, indices)
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;

    const WHITE: (f32, f32, f32) = (1.0, 1.0, 1.0);

    /// The volume a closed mesh encloses.
    fn enclosed(mesh: &MeshPrimitive) -> f32 {
        let volumes = mesh.indices.chunks_exact(3).map(|t| {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize].pos);
            a.dot(&b.cross(&c)) / 6.0
        });
        volumes.sum()
    }

    /// How many triangles share each edge of the mesh.
    fn sharing(mesh: &MeshPrimitive) -> Vec<u32> {
        let mut edges: FxHashMap<(u32, u32), u32> = FxHashMap::default();
        for t in mesh.indices.chunks_exact(3) {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        edges.into_values().collect()
    }

    /// Whether every edge of the mesh is shared by exactly two triangles.
    fn closed(mesh: &MeshPrimitive) -> bool {
        let sharing = sharing(mesh);
        !sharing.is_empty() && sharing.iter().all(|shared| *shared == 2)
    }

    /// Whether the mesh has no open edge. Where two sheets of surface pass
    /// through one cell they share its vertex, and an edge there is shared
    /// by four triangles: sealed, though not a simple surface.
    fn watertight(mesh: &MeshPrimitive) -> bool {
        let sharing = sharing(mesh);
        !sharing.is_empty() && sharing.iter().all(|shared| shared % 2 == 0)
    }

    #[test]
    fn a_ball_skins_as_a_closed_sphere_facing_out() {
        let ball = Clay::new(0.0).add(Lump::ball(0.5, (1.0, 2.0, 3.0)), WHITE);
        let mesh = ball.skin(0.02, MaterialId(0));
        assert!(closed(&mesh));
        let volume = enclosed(&mesh);
        let sphere = 4.0 / 3.0 * PI * 0.5f32.powi(3);
        assert!(
            (volume / sphere - 1.0).abs() < 0.01,
            "{volume} m³ of {sphere}"
        );
        let centre = Vector3::new(1.0, 2.0, 3.0);
        for vertex in &mesh.vertices {
            let out = (vertex.pos - centre).normalize();
            assert!(
                vertex.normal.dot(&out) > 0.99,
                "a normal looks {:?}",
                vertex.normal
            );
            assert!(
                (vertex.ao - 1.0).abs() < 0.05,
                "open ground shaded {}",
                vertex.ao
            );
        }
    }

    #[test]
    fn a_limb_is_as_thick_as_asked_at_each_end() {
        let limb = Lump::limb((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.2, 0.05));
        assert!((limb.distance(Vector3::new(0.3, 0.0, 0.0)) - 0.1).abs() < 0.02);
        assert!((limb.distance(Vector3::new(0.1, 1.0, 0.0)) - 0.05).abs() < 0.02);
        assert!(limb.distance(Vector3::new(0.0, 0.5, 0.0)) < 0.0);
    }

    #[test]
    fn two_lumps_blend_into_one_closed_skin_with_a_shaded_waist() {
        let form = Clay::new(0.08)
            .add(Lump::ball(0.3, (0.0, 0.0, 0.0)), (1.0, 0.0, 0.0))
            .add(Lump::ball(0.3, (0.5, 0.0, 0.0)), (0.0, 0.0, 1.0));
        let mesh = form.skin(0.015, MaterialId(0));
        assert!(closed(&mesh));
        // Merely overlapping, the two would meet in a crease 0.166 m from
        // their axis. Blended, the waist is fuller than that.
        let waist_girth = mesh
            .vertices
            .iter()
            .filter(|v| (v.pos.x - 0.25).abs() < 0.01)
            .map(|v| v.pos.yz().norm())
            .fold(f32::MAX, f32::min);
        assert!(waist_girth > 0.175, "a waist {waist_girth} m thick");
        // Red at one end, blue at the other, and the waist between them in
        // shade.
        let end = |x: f32| {
            let nearest = mesh
                .vertices
                .iter()
                .min_by(|a, b| (a.pos.x - x).abs().total_cmp(&(b.pos.x - x).abs()));
            *nearest.unwrap()
        };
        assert!(end(-0.3).color.x > 0.95 && end(0.8).color.z > 0.95);
        let waist = mesh
            .vertices
            .iter()
            .filter(|v| (v.pos.x - 0.25).abs() < 0.02)
            .map(|v| v.ao)
            .fold(1.0, f32::min);
        assert!(waist < 0.9, "the waist is lit {waist}");
    }

    #[test]
    fn a_hollow_form_cut_open_shows_its_inside() {
        let whole = Clay::new(0.0).add(Lump::ball(0.4, (0.0, 0.0, 0.0)), WHITE);
        let shell = whole
            .clone()
            .hollow(0.03)
            .cut(Lump::ball(0.2, (0.0, 0.4, 0.0)), 0.01);
        let mesh = shell.skin(0.01, MaterialId(0));
        assert!(closed(&mesh));
        // A shell holds far less than the ball it was.
        assert!(enclosed(&mesh) < 0.3 * enclosed(&whole.skin(0.01, MaterialId(0))));
        // Nothing is left where the cut went, and the inside wall is there.
        assert!(mesh
            .vertices
            .iter()
            .all(|v| (v.pos - Vector3::y() * 0.4).norm() > 0.18));
        let inner = mesh
            .vertices
            .iter()
            .filter(|v| v.pos.norm() < 0.385)
            .count();
        assert!(inner > mesh.vertices.len() / 4, "no inside wall");
    }

    #[test]
    fn folds_stand_out_of_the_surface_they_are_raised_on() {
        let smooth = Clay::new(0.0).add(Lump::ball(0.4, (0.0, 0.0, 0.0)), WHITE);
        let folded = smooth.clone().relief(Relief::Folds {
            along: Vector3::y(),
            frequency: 12.0,
            depth: 0.02,
            seed: 3,
        });
        let mesh = folded.skin(0.01, MaterialId(0));
        assert!(watertight(&mesh));
        let radii: Vec<f32> = mesh.vertices.iter().map(|v| v.pos.norm()).collect();
        let (low, high) = radii
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), r| (lo.min(*r), hi.max(*r)));
        assert!(high - low > 0.02, "folds only {} m deep", high - low);
        assert!(high < 0.43 && low > 0.37);
    }

    #[test]
    fn rings_only_raise_the_surface_and_no_further_than_their_depth() {
        let ringed = Clay::new(0.0)
            .add(Lump::ball(0.4, (0.0, 0.0, 0.0)), WHITE)
            .relief(Relief::Rings {
                about: Vector3::new(0.0, 0.4, 0.0),
                spacing: 0.06,
                depth: 0.012,
                seed: 5,
            });
        let mesh = ringed.skin(0.008, MaterialId(0));
        assert!(watertight(&mesh));
        let radii: Vec<f32> = mesh.vertices.iter().map(|v| v.pos.norm()).collect();
        let (low, high) = radii
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), r| (lo.min(*r), hi.max(*r)));
        assert!(low > 0.395, "a ring sinks to {low}");
        assert!((0.406..0.416).contains(&high), "the proudest stands {high}");
    }
}
