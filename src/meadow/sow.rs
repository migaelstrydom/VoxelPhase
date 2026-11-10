//! Sowing: which cells of the ground bear what.
//!
//! ```text
//!   triangle of level turf ──▶ the cells whose middles lie on it
//!        cell ── hash of where it is ──▶ nothing, a tuft, a flower, a berry
//!                (weighed by how lush the lawn is there, and whether a
//!                 drift of flowers runs through)
//! ```

use nalgebra::{Vector2, Vector3};
use rayon::prelude::*;
use rustc_hash::FxHashMap;

use super::config::MeadowConfig;
use crate::rendering::vertex::Vertex;

/// Side of a cell, m: no two sprigs stand closer than about this.
const CELL: f32 = 0.125;

/// Side of a tile, m: the sprigs of one are drawn as one mesh.
pub const TILE: f32 = 16.0;

/// Least height of a triangle's normal for anything to grow on it: steeper
/// turf is thin and patchy (`shader/slope_zoning.glsl`).
const LEVEL_ENOUGH: f32 = 0.8;

/// Least pile at each corner of a triangle for it to count as lawn: short of
/// the turf's edge, where the blend with bare ground begins.
const ALL_TURF: f32 = 0.99;

/// Patches of lushness, and drifts of flowers, to the metre.
const LUSH_FREQUENCY: f32 = 0.3;
const DRIFT_FREQUENCY: f32 = 0.09;

/// The share of a drift's noise over which flowers come in.
const DRIFT_EDGE: (f32, f32) = (0.5, 0.7);

/// How many times thicker flowers grow in a drift than outside one.
const DRIFT_RICHNESS: f32 = 14.0;

/// Share of blooms that are berries; the rest are flowers.
const BERRY_SHARE: f32 = 0.25;

/// A tile, by its place in the grid of tiles.
pub type TileCoord = (i32, i32);

/// What grows in a cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SprigKind {
    Tuft,
    Flower,
    Berry,
}

/// One thing growing in the meadow.
#[derive(Clone, Copy, Debug)]
pub struct Sprig {
    /// The cell it grows in.
    pub cell: (i32, i32),
    /// Where it stands on the ground.
    pub at: Vector3<f32>,
    pub kind: SprigKind,
    /// Its own chance: what makes it unlike the next of its kind.
    pub chance: u32,
}

/// An integer hash of a cell and a salt.
fn hash(x: i32, z: i32, salt: u32) -> u32 {
    let mut h = (x as u32).wrapping_mul(0x9E37_79B1)
        ^ (z as u32).wrapping_mul(0x85EB_CA77)
        ^ salt.wrapping_mul(0xC2B2_AE3D);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    h = h.wrapping_mul(0x297A_2D39);
    h ^ (h >> 15)
}

/// A hash as a share, 0 to 1.
pub fn share(h: u32) -> f32 {
    (h >> 8) as f32 / (1u32 << 24) as f32
}

/// Smooth noise over the ground, 0 to 1, `frequency` features to the metre.
fn patches(at: Vector2<f32>, frequency: f32, salt: u32) -> f32 {
    let p = at * frequency;
    let (x, z) = (p.x.floor(), p.y.floor());
    let (fx, fz) = (p.x - x, p.y - z);
    let (ux, uz) = (fx * fx * (3.0 - 2.0 * fx), fz * fz * (3.0 - 2.0 * fz));
    let corner = |dx: i32, dz: i32| share(hash(x as i32 + dx, z as i32 + dz, salt));
    let near = corner(0, 0) + (corner(1, 0) - corner(0, 0)) * ux;
    let far = corner(0, 1) + (corner(1, 1) - corner(0, 1)) * ux;
    near + (far - near) * uz
}

fn smoothstep(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// What cell `(x, z)` bears on a lawn, if anything.
fn bears(cell: (i32, i32), config: &MeadowConfig) -> Option<(SprigKind, u32)> {
    let area = CELL * CELL;
    // Most cells bear nothing wherever they lie: those are turned away
    // before the lawn is read at all.
    let chance = hash(cell.0, cell.1, config.seed);
    let drawn = share(chance);
    let richest = area
        * (config.blooms * (1.0 + DRIFT_RICHNESS) / (1.0 + 0.2 * DRIFT_RICHNESS)
            + config.tufts * 4.0);
    if drawn >= richest {
        return None;
    }

    let at = Vector2::new(cell.0 as f32 + 0.5, cell.1 as f32 + 0.5) * CELL;
    let lush = patches(at, LUSH_FREQUENCY, config.seed ^ 0x51);
    let drift = smoothstep(
        DRIFT_EDGE.0,
        DRIFT_EDGE.1,
        patches(at, DRIFT_FREQUENCY, config.seed ^ 0xA7),
    );
    // Thinnest outside a drift, so that over a lawn that is a fifth drift
    // the mean comes out about what was asked.
    let blooms =
        config.blooms * area * (1.0 + DRIFT_RICHNESS * drift) / (1.0 + 0.2 * DRIFT_RICHNESS);
    // Gathered hard into the lushest ground; lushness cubed has a mean of
    // a quarter.
    let tufts = config.tufts * area * 4.0 * lush * lush * lush;

    let own = hash(cell.0, cell.1, config.seed ^ 0x3D);
    if drawn < blooms {
        let kind = if share(own) < BERRY_SHARE {
            SprigKind::Berry
        } else {
            SprigKind::Flower
        };
        Some((kind, own))
    } else if drawn < blooms + tufts {
        Some((SprigKind::Tuft, own))
    } else {
        None
    }
}

/// A level rectangle of ground, by its corners in x and z.
#[derive(Clone, Copy, Debug)]
pub struct Plot {
    pub min: Vector2<f32>,
    pub max: Vector2<f32>,
}

impl Plot {
    fn holds(&self, at: Vector2<f32>) -> bool {
        at.x >= self.min.x && at.x <= self.max.x && at.y >= self.min.y && at.y <= self.max.y
    }

    /// Whether `other` reaches into it.
    pub fn meets(&self, other: &Plot) -> bool {
        other.max.x >= self.min.x
            && other.min.x <= self.max.x
            && other.max.y >= self.min.y
            && other.min.y <= self.max.y
    }
}

/// Sprigs by tile, each tile's in the order of their cells.
pub type Tiles = FxHashMap<TileCoord, Vec<Sprig>>;

/// The middle of a cell, on the ground.
fn middle_of(cell: (i32, i32)) -> Vector2<f32> {
    Vector2::new(cell.0 as f32 + 0.5, cell.1 as f32 + 0.5) * CELL
}

/// The tile a cell lies in.
fn tile_of(cell: (i32, i32)) -> TileCoord {
    let middle = middle_of(cell);
    (
        (middle.x / TILE).floor() as i32,
        (middle.y / TILE).floor() as i32,
    )
}

/// The sprigs on the mesh's triangles, in the cells `plots` hold if any are
/// given and in every cell if not: in no order, a cell on an edge two
/// triangles share perhaps twice.
fn sprigs_on(
    vertices: &[Vertex],
    indices: &[u32],
    config: &MeadowConfig,
    plots: Option<&[Plot]>,
) -> Vec<Sprig> {
    // A level's mesh is hundreds of thousands of triangles, and this runs
    // in the frame a blast lands: every core takes a share of them.
    indices
        .par_chunks(3 * TRIANGLES_A_SHARE)
        .flat_map_iter(|share| sow_triangles(vertices, share, config, plots))
        .collect()
}

/// Put `sprigs` into the tiles they lie in, and those tiles back in order.
fn plant(tiles: &mut Tiles, sprigs: Vec<Sprig>) -> Vec<TileCoord> {
    let mut planted = Vec::new();
    for sprig in sprigs {
        let tile = tile_of(sprig.cell);
        tiles.entry(tile).or_default().push(sprig);
        planted.push(tile);
    }
    planted.sort_unstable();
    planted.dedup();
    for tile in &planted {
        let sprigs = tiles.get_mut(tile).expect("a tile just planted");
        sprigs.sort_unstable_by_key(|sprig| sprig.cell);
        sprigs.dedup_by_key(|sprig| sprig.cell);
    }
    planted
}

/// Everything that grows on the terrain mesh `vertices`, `indices` (world
/// space, pile in each vertex's second texture coordinate).
pub fn sow(vertices: &[Vertex], indices: &[u32], config: &MeadowConfig) -> Tiles {
    let mut tiles = Tiles::default();
    plant(&mut tiles, sprigs_on(vertices, indices, config, None));
    tiles
}

/// A piece of the terrain's mesh: its vertices, and the triangles among
/// them.
pub type MeshPiece<'a> = (&'a [Vertex], &'a [u32]);

/// Sow `plots` again, on the ground as it is now: whatever `tiles` held in
/// them goes, and what `meshes` bear there takes its place. `meshes` must
/// between them hold all the ground over the plots, at every height; the
/// rest of the terrain is not looked at. Returns the tiles that lost or
/// gained a sprig.
pub fn resow(
    tiles: &mut Tiles,
    meshes: &[MeshPiece],
    config: &MeadowConfig,
    plots: &[Plot],
) -> Vec<TileCoord> {
    let within = |sprig: &Sprig| plots.iter().any(|plot| plot.holds(middle_of(sprig.cell)));
    let mut touched = Vec::new();
    for (tile, sprigs) in tiles.iter_mut() {
        let before = sprigs.len();
        sprigs.retain(|sprig| !within(sprig));
        if sprigs.len() != before {
            touched.push(*tile);
        }
    }
    let fresh = meshes
        .iter()
        .flat_map(|(vertices, indices)| sprigs_on(vertices, indices, config, Some(plots)))
        .collect();
    touched.extend(plant(tiles, fresh));
    tiles.retain(|_, sprigs| !sprigs.is_empty());
    touched.sort_unstable();
    touched.dedup();
    touched
}

/// Triangles sown by one core at a time.
const TRIANGLES_A_SHARE: usize = 4096;

/// [`sprigs_on`], for one core's share of the triangles.
fn sow_triangles(
    vertices: &[Vertex],
    indices: &[u32],
    config: &MeadowConfig,
    plots: Option<&[Plot]>,
) -> Vec<Sprig> {
    let mut sprigs = Vec::new();
    for triangle in indices.chunks_exact(3) {
        let [a, b, c] = [triangle[0], triangle[1], triangle[2]].map(|i| &vertices[i as usize]);
        if a.tex_coords.y.min(b.tex_coords.y).min(c.tex_coords.y) < ALL_TURF {
            continue;
        }
        let normal = (b.pos - a.pos).cross(&(c.pos - a.pos));
        let twice_area = normal.norm();
        if twice_area <= 0.0 || normal.y < LEVEL_ENOUGH * twice_area {
            continue;
        }

        // The triangle seen from above, and the cells its box covers.
        let flat = |v: &Vertex| Vector2::new(v.pos.x, v.pos.z);
        let (pa, pb, pc) = (flat(a), flat(b), flat(c));
        let across = (pb - pa).perp(&(pc - pa));
        if across == 0.0 {
            continue;
        }
        let (low, high) = (pa.inf(&pb).inf(&pc), pa.sup(&pb).sup(&pc));
        let reach = Plot {
            min: low,
            max: high,
        };
        if plots.is_some_and(|plots| !plots.iter().any(|plot| plot.meets(&reach))) {
            continue;
        }
        let (low, high) = (low / CELL, high / CELL);
        for x in low.x.floor() as i32..=high.x.floor() as i32 {
            for z in low.y.floor() as i32..=high.y.floor() as i32 {
                let Some((kind, chance)) = bears((x, z), config) else {
                    continue;
                };
                let middle = middle_of((x, z));
                if plots.is_some_and(|plots| !plots.iter().any(|plot| plot.holds(middle))) {
                    continue;
                }
                let wb = (middle - pa).perp(&(pc - pa)) / across;
                let wc = (pb - pa).perp(&(middle - pa)) / across;
                if wb < 0.0 || wc < 0.0 || wb + wc > 1.0 {
                    continue;
                }
                // Stood a little off its cell's middle, on the triangle's
                // plane.
                let nudge =
                    Vector2::new(share(chance) - 0.5, share(chance >> 7) - 0.5) * 0.7 * CELL;
                let at = middle + nudge;
                let height = a.pos.y
                    - (normal.x * (at.x - a.pos.x) + normal.z * (at.y - a.pos.z)) / normal.y;
                sprigs.push(Sprig {
                    cell: (x, z),
                    at: Vector3::new(at.x, height, at.y),
                    kind,
                    chance,
                });
            }
        }
    }
    sprigs
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector4;

    use super::*;

    /// A level square of ground `side` metres across with a corner at the
    /// origin, at `height`, of the given `pile`.
    fn square(side: f32, height: f32, pile: f32) -> (Vec<Vertex>, Vec<u32>) {
        let corner = |x: f32, z: f32| Vertex {
            pos: Vector3::new(x, height, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::new(0.2, pile),
            normal: Vector3::y(),
            ao: 1.0,
        };
        (
            vec![
                corner(0.0, 0.0),
                corner(side, 0.0),
                corner(side, side),
                corner(0.0, side),
            ],
            vec![0, 2, 1, 0, 3, 2],
        )
    }

    /// A blast is told to the meadow as the plot it changed. Sown again
    /// there alone, the meadow must come out as if sown whole.
    #[test]
    fn a_plot_sown_again_leaves_the_meadow_as_if_sown_whole() {
        let config = MeadowConfig::default();
        // Four squares of lawn in a row, 8 m each.
        let lawn = |bare: Option<usize>| {
            let (mut vertices, mut indices) = (Vec::new(), Vec::new());
            for k in 0..4 {
                let pile = if bare == Some(k) { 0.0 } else { 1.0 };
                let (square_vertices, square_indices) = square(8.0, 1.0, pile);
                let first = vertices.len() as u32;
                vertices.extend(square_vertices.into_iter().map(|mut vertex| {
                    vertex.pos.x += 8.0 * k as f32 + 4.0;
                    vertex
                }));
                indices.extend(square_indices.iter().map(|i| i + first));
            }
            (vertices, indices)
        };
        let (vertices, indices) = lawn(None);
        let mut tiles = sow(&vertices, &indices, &config);
        let before: usize = tiles.values().map(Vec::len).sum();

        // The third square is blasted bare: it straddles two tiles.
        let (vertices, indices) = lawn(Some(2));
        let plot = Plot {
            min: Vector2::new(19.5, -0.5),
            max: Vector2::new(28.5, 8.5),
        };
        let touched = resow(&mut tiles, &[(&vertices, &indices)], &config, &[plot]);
        let whole = sow(&vertices, &indices, &config);

        let cells = |tiles: &Tiles| {
            let mut cells: Vec<_> = tiles.values().flatten().map(|s| (s.cell, s.kind)).collect();
            cells.sort_unstable_by_key(|(cell, _)| *cell);
            cells
        };
        assert_eq!(cells(&tiles), cells(&whole));
        assert!(cells(&tiles).len() < before, "the blast cleared nothing");
        assert_eq!(touched, vec![(1, 0)], "tiles the blast touched");
    }

    fn count(tiles: &FxHashMap<TileCoord, Vec<Sprig>>, kind: SprigKind) -> usize {
        tiles
            .values()
            .flatten()
            .filter(|sprig| sprig.kind == kind)
            .count()
    }

    #[test]
    fn a_lawn_bears_about_what_was_asked() {
        let (vertices, indices) = square(64.0, 3.0, 1.0);
        let config = MeadowConfig::default();
        let tiles = sow(&vertices, &indices, &config);
        let area = 64.0 * 64.0;
        let tufts = count(&tiles, SprigKind::Tuft) as f32 / area;
        let blooms =
            (count(&tiles, SprigKind::Flower) + count(&tiles, SprigKind::Berry)) as f32 / area;
        assert!((tufts / config.tufts - 1.0).abs() < 0.4, "{tufts} tufts/m²");
        assert!(
            (blooms / config.blooms - 1.0).abs() < 0.6,
            "{blooms} blooms/m²"
        );
        assert_eq!(tiles.len(), 16, "tiles of a 64 m square");
        for sprig in tiles.values().flatten() {
            assert!((sprig.at.y - 3.0).abs() < 1.0e-4);
        }
    }

    #[test]
    fn bare_ground_and_the_turfs_edge_bear_nothing() {
        let config = MeadowConfig::default();
        let (vertices, indices) = square(32.0, 0.0, 0.0);
        assert!(sow(&vertices, &indices, &config).is_empty());
        let (mut vertices, indices) = square(32.0, 0.0, 1.0);
        vertices[0].tex_coords.y = 0.0;
        vertices[2].tex_coords.y = 0.0;
        assert!(sow(&vertices, &indices, &config).is_empty());
    }

    #[test]
    fn a_wall_of_turf_bears_nothing() {
        let (mut vertices, indices) = square(32.0, 0.0, 1.0);
        for vertex in &mut vertices {
            vertex.pos = Vector3::new(vertex.pos.x, vertex.pos.z, 0.0);
        }
        assert!(sow(&vertices, &indices, &MeadowConfig::default()).is_empty());
    }

    /// The ground alone decides: however it is cut into triangles, and
    /// whatever lies beside it.
    #[test]
    fn the_same_ground_bears_the_same_sprigs_however_it_is_meshed() {
        let config = MeadowConfig::default();
        let (vertices, indices) = square(16.0, 1.0, 1.0);
        let whole = sow(&vertices, &indices, &config);
        // The other diagonal.
        let other = sow(&vertices, &[0, 3, 1, 1, 3, 2], &config);
        let cells = |tiles: &FxHashMap<TileCoord, Vec<Sprig>>| -> Vec<((i32, i32), SprigKind)> {
            tiles[&(0, 0)].iter().map(|s| (s.cell, s.kind)).collect()
        };
        assert!(!cells(&whole).is_empty());
        assert_eq!(cells(&whole), cells(&other));
    }
}
