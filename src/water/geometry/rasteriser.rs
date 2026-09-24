//! Turns terrain into spans, and keeps them current as terrain is blown apart.
//!
//! ```text
//!   terrain chunk ──rasterise──▶ CrossingCache ──pair──▶ SpanGraph
//!   (rebuilt)                    per (chunk, column)      per column
//!                                                          └─▶ SpanRemap
//! ```
//!
//! Recomputing crossings needs only the changed terrain chunk; pairing them
//! into spans needs the whole vertical stack, so a stale column re-pairs the
//! cached crossings of every chunk and segment above and below it.

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::collision::AABB;
use crate::terrain::{TerrainChunkId, TerrainWorld};

use super::crossings::{rasterise_by_tile, Crossing, Facing, FloorPiece, TileCrossings};
use super::span::{
    Column, Span, SpanChunk, SpanChunkCoord, SpanOwner, CHUNK_COLUMNS, COLUMNS_PER_CHUNK,
    COLUMN_SIZE,
};
use super::span_graph::SpanGraph;

/// A floor and a ceiling closer than this are one surface seen twice, at a
/// silhouette edge, and are dropped as a pair.
const ZERO_HEIGHT: f32 = 1e-4;

/// Height above an old floor at which the remap looks for its new span. Air
/// on both sides of any edit (§3.2).
pub const REMAP_EPSILON: f32 = 1e-3;

/// How far a floor may rise across an edit, as a fraction of the local voxel
/// size, before it breaks the removal-only invariant.
///
/// Carving only lowers densities, but marching cubes does not turn that into
/// a surface that only falls: when a cell near a crater's rim changes case,
/// its triangles connect different edge vertices, and the surface over a
/// fixed point can rise by up to a voxel: the surface stays within the cell
/// it crosses. Grenades on the surface of the five water levels raised it at
/// most 0.3 of a voxel; `water_fuzz`, blasting into test_arena's pool floor
/// and the caves under it, 0.75. A rise within this tolerance is clamped: the
/// span keeps its old floor, so everything downstream still sees floors that
/// only drop.
pub const RETRIANGULATION_TOLERANCE: f32 = 1.0;

/// Where each span of the rebuilt columns went.
///
/// Owners are already rewritten when the remap is returned: each new span
/// belongs to the owner of the lowest old span that landed in it. Entries
/// sharing a new span are the conflicts of §8.3, for the caller to act on.
#[derive(Debug, Clone, Default)]
pub struct SpanRemap {
    /// Every re-paired column, including those that had no spans before.
    pub columns: Vec<Column>,
    /// One per old span of every re-paired column, bottom-up within a column.
    pub entries: Vec<RemapEntry>,
    /// Span chunks rebuilt. Every ref into them is stale.
    pub chunks: Vec<SpanChunkCoord>,
}

/// One old span and the new span containing `(centre, old.floor_c + ε)`.
#[derive(Debug, Clone, Copy)]
pub struct RemapEntry {
    pub column: Column,
    pub old_ordinal: u8,
    pub old: Span,
    /// Who owned the old span.
    pub old_owner: SpanOwner,
    /// `None` where the edit blew away every floor below the old one: the
    /// span now opens onto the void, and its water leaves the world.
    pub new_ordinal: Option<u8>,
    /// Whether the new span still rests on this span's floor. Not for a
    /// sliver thinner than a voxel that the re-mesh closed: marching cubes
    /// does not resolve it, so it can vanish with no material added, and the
    /// span above takes its air. Owners, holes, drains and the floor hold go
    /// by the lowest span that rests.
    pub rests: bool,
}

/// What pairing one column found.
#[derive(Debug, Clone, Default)]
struct PairedColumn {
    spans: Vec<Span>,
    /// The crossings alternated wrongly and had to be resolved by solidity
    /// tests.
    repaired: bool,
}

/// Where one rebuild's time went.
#[derive(Debug, Clone, Copy, Default)]
pub struct RebuildTimings {
    /// Rasterising the rebuilt terrain chunks.
    pub rasterise: Duration,
    /// Finding the columns whose crossings changed.
    pub diff: Duration,
    /// Pairing the stale columns.
    pub pair: Duration,
    /// Remapping, committing and rewriting owners.
    pub commit: Duration,
    pub stale_columns: usize,
}

/// Counters from building or rebuilding spans.
#[derive(Debug, Clone, Copy, Default)]
pub struct RasterStats {
    pub columns_paired: usize,
    pub parity_repairs: usize,
    /// Floors that rose by less than the re-triangulation tolerance and were
    /// held at their old height.
    pub floor_clamps: usize,
    /// The largest clamped rise, in voxels.
    pub worst_clamp: f32,
    /// Spans whose `floor_min` rose because a span opened below them, held
    /// at the old height.
    pub floor_min_holds: usize,
    /// Slivers thinner than a voxel that a re-mesh closed.
    pub closed_slivers: usize,
    pub invariant_violations: usize,
    /// The most recent violation: the column, its old floor and its new.
    pub last_violation: Option<(Column, f32, f32)>,
}

/// Rasterises terrain into a span graph and keeps it current.
pub struct SpanRasteriser {
    /// Crossings per terrain chunk, per tile.
    cache: FxHashMap<(TerrainChunkId, SpanChunkCoord), TileCrossings>,
    /// Terrain chunks holding crossings in each tile.
    by_span_chunk: FxHashMap<SpanChunkCoord, BTreeSet<TerrainChunkId>>,
    /// Columns whose last pairing needed a parity repair.
    repaired: BTreeSet<Column>,
    /// Totals since construction.
    stats: RasterStats,
    last_rebuild: RebuildTimings,
}

impl SpanRasteriser {
    /// Rasterise every terrain chunk and pair every column, in parallel.
    pub fn build(terrain: &TerrainWorld) -> (Self, SpanGraph) {
        let ids = terrain.chunk_ids();
        let tiles: Vec<(TerrainChunkId, FxHashMap<SpanChunkCoord, TileCrossings>)> = ids
            .par_iter()
            .filter_map(|&id| {
                let geometry = terrain.chunk_geometry(id)?;
                let point = |i: u32| {
                    let v = geometry.vertices[i as usize].pos;
                    nalgebra::Point3::new(v.x, v.y, v.z)
                };
                let triangles = geometry
                    .indices
                    .chunks_exact(3)
                    .map(|tri| [point(tri[0]), point(tri[1]), point(tri[2])]);
                Some((id, rasterise_by_tile(triangles)))
            })
            .collect();

        let mut rasteriser = Self {
            cache: FxHashMap::default(),
            by_span_chunk: FxHashMap::default(),
            repaired: BTreeSet::new(),
            stats: RasterStats::default(),
            last_rebuild: RebuildTimings::default(),
        };
        for (id, by_tile) in tiles {
            for (tile, crossings) in by_tile {
                rasteriser.install(id, tile, crossings);
            }
        }

        let (min, max) = rasteriser.extent(terrain);
        let mut graph = SpanGraph::new(min, max);
        let coords: Vec<SpanChunkCoord> = graph.chunk_coords().collect();
        let built: Vec<(SpanChunkCoord, Vec<PairedColumn>)> = coords
            .par_iter()
            .map(|&coord| (coord, rasteriser.pair_chunk(coord, terrain, |_| true, None)))
            .collect();
        for (coord, columns) in built {
            rasteriser.commit(&mut graph, coord, columns, 0);
        }
        (rasteriser, graph)
    }

    /// Re-rasterise what the terrain rebuilt and re-pair every column whose
    /// crossings changed. Returns where the old spans went.
    ///
    /// Only the tiles of each rebuilt chunk that a changed region touches are
    /// re-rasterised: outside those boxes the surface has not moved, and
    /// marching cubes gives identical triangles for identical voxels.
    pub fn rebuild(
        &mut self,
        terrain: &TerrainWorld,
        graph: &mut SpanGraph,
        rebuilt: &[TerrainChunkId],
        changed: &[AABB],
    ) -> SpanRemap {
        let started = Instant::now();
        let mut jobs: Vec<(TerrainChunkId, SpanChunkCoord, AABB)> = Vec::new();
        let mut vanished: Vec<TerrainChunkId> = Vec::new();
        for &id in rebuilt {
            let Some(bounds) = terrain.chunk_world_bounds(id) else {
                vanished.push(id);
                continue;
            };
            if terrain.chunk_geometry(id).is_none() {
                vanished.push(id);
                continue;
            }
            let mut tiles: BTreeSet<SpanChunkCoord> = BTreeSet::new();
            for region in changed.iter().filter(|r| r.intersects(&bounds)) {
                tiles.extend(tiles_over(region, COLUMN_SIZE));
            }
            for tile in tiles {
                let (x0, z0) = tile.column(0).min_corner();
                let extent = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
                let footprint = AABB::new(
                    nalgebra::Point3::new(x0, bounds.min.y, z0),
                    nalgebra::Point3::new(x0 + extent, bounds.max.y, z0 + extent),
                );
                jobs.push((id, tile, footprint));
            }
        }
        let fresh: Vec<(TerrainChunkId, SpanChunkCoord, TileCrossings)> = jobs
            .par_iter()
            .map(|&(id, tile, footprint)| {
                let triangles = terrain
                    .chunk_triangles_in(id, &footprint)
                    .into_iter()
                    .map(|t| [t.v0, t.v1, t.v2]);
                (id, tile, TileCrossings::for_tile(triangles, tile))
            })
            .collect();
        let rasterise = started.elapsed();

        let started = Instant::now();
        let mut stale: BTreeSet<Column> = BTreeSet::new();
        for (id, tile, crossings) in fresh {
            let old = self.cache.remove(&(id, tile)).unwrap_or_default();
            stale.extend(old.changed_columns(&crossings));
            self.install(id, tile, crossings);
        }
        for id in vanished {
            let keys: Vec<_> = self.cache.keys().filter(|k| k.0 == id).copied().collect();
            for key in keys {
                if let Some(old) = self.cache.remove(&key) {
                    stale.extend(old.touched());
                }
            }
        }

        let mut by_chunk: FxHashMap<SpanChunkCoord, BTreeSet<Column>> = FxHashMap::default();
        for column in stale {
            if graph.contains_column(column) {
                by_chunk.entry(column.chunk()).or_default().insert(column);
            }
        }
        let mut coords: Vec<SpanChunkCoord> = by_chunk.keys().copied().collect();
        coords.sort();
        let diff = started.elapsed();
        let stale_columns = by_chunk.values().map(BTreeSet::len).sum();

        let started = Instant::now();
        let this = &*self;
        let graph_ref = &*graph;
        let paired: Vec<(SpanChunkCoord, Vec<PairedColumn>)> = coords
            .par_iter()
            .map(|coord| {
                let columns = &by_chunk[coord];
                let paired = this.pair_chunk(
                    *coord,
                    terrain,
                    |column| columns.contains(&column),
                    Some(graph_ref),
                );
                (*coord, paired)
            })
            .collect();
        let pair = started.elapsed();

        let started = Instant::now();
        let mut remap = SpanRemap::default();
        for (coord, columns) in paired {
            let stale = &by_chunk[&coord];
            let generation = graph.chunk(coord).map_or(0, |c| c.generation()) + 1;
            let old_owners: Vec<Vec<SpanOwner>> = (0..COLUMNS_PER_CHUNK)
                .map(|local| {
                    graph
                        .chunk(coord)
                        .map(|c| c.column_owners(local).to_vec())
                        .unwrap_or_default()
                })
                .collect();
            let first_entry = remap.entries.len();
            let mut columns = columns;
            for column in stale {
                remap.columns.push(*column);
                let old = graph.spans(*column).to_vec();
                let owners = old_owners[column.local_index()].clone();
                let (x, z) = column.centre();
                let voxel = terrain.voxel_size_at(nalgebra::Point3::new(
                    x,
                    old.first().map_or(0.0, |s| s.floor_c),
                    z,
                ));
                let new = &mut columns[column.local_index()].spans;
                self.remap_column(*column, &old, &owners, new, voxel, &mut remap);
            }
            self.commit(graph, coord, columns, generation);

            for (local, owners) in old_owners.iter().enumerate() {
                let column = coord.column(local);
                if stale.contains(&column) {
                    continue;
                }
                for (ordinal, owner) in owners.iter().enumerate() {
                    *graph.owner_mut(graph.make_ref(column, ordinal as u8)) = *owner;
                }
            }
            // Old spans are remapped bottom-up, so the lowest old span to land
            // in a new span claims it first: the one it still rests on (§8.3).
            // A closed sliver claims only a span nothing resting landed in.
            let mut claimed: BTreeSet<(Column, u8)> = BTreeSet::new();
            let entries = &remap.entries[first_entry..];
            for entry in entries
                .iter()
                .filter(|e| e.rests)
                .chain(entries.iter().filter(|e| !e.rests))
            {
                let Some(new_ordinal) = entry.new_ordinal else {
                    continue;
                };
                if !claimed.insert((entry.column, new_ordinal)) {
                    continue;
                }
                let owner = old_owners[entry.column.local_index()][entry.old_ordinal as usize];
                *graph.owner_mut(graph.make_ref(entry.column, new_ordinal)) = owner;
            }
            remap.chunks.push(coord);
        }
        self.last_rebuild = RebuildTimings {
            rasterise,
            diff,
            pair,
            commit: started.elapsed(),
            stale_columns,
        };
        remap
    }

    /// Where the most recent rebuild's time went.
    pub fn last_rebuild(&self) -> RebuildTimings {
        self.last_rebuild
    }

    /// Totals since construction.
    pub fn stats(&self) -> RasterStats {
        self.stats
    }

    /// Columns whose current spans needed a parity repair.
    pub fn repaired_columns(&self) -> impl Iterator<Item = Column> + '_ {
        self.repaired.iter().copied()
    }

    fn install(&mut self, id: TerrainChunkId, tile: SpanChunkCoord, crossings: TileCrossings) {
        if crossings.is_empty() {
            return;
        }
        self.by_span_chunk.entry(tile).or_default().insert(id);
        self.cache.insert((id, tile), crossings);
    }

    /// The span chunks covering the terrain, padded by one chunk so that the
    /// void past every edge is inside the graph.
    fn extent(&self, terrain: &TerrainWorld) -> (SpanChunkCoord, SpanChunkCoord) {
        let bounds = terrain.bounds();
        let min = Column::containing(bounds.min.x, bounds.min.z).chunk();
        let max = Column::containing(bounds.max.x, bounds.max.z).chunk();
        (
            SpanChunkCoord {
                x: min.x - 1,
                z: min.z - 1,
            },
            SpanChunkCoord {
                x: max.x + 1,
                z: max.z + 1,
            },
        )
    }

    /// Pair every selected column of a span chunk. Unselected columns keep
    /// their current spans from `graph`.
    fn pair_chunk(
        &self,
        coord: SpanChunkCoord,
        terrain: &TerrainWorld,
        selected: impl Fn(Column) -> bool,
        graph: Option<&SpanGraph>,
    ) -> Vec<PairedColumn> {
        let sources: Vec<&TileCrossings> = self
            .by_span_chunk
            .get(&coord)
            .map(|ids| {
                ids.iter()
                    .filter_map(|id| self.cache.get(&(*id, coord)))
                    .collect()
            })
            .unwrap_or_default();
        coord
            .columns()
            .map(|column| {
                if !selected(column) {
                    return PairedColumn {
                        spans: graph.map(|g| g.spans(column).to_vec()).unwrap_or_default(),
                        repaired: self.repaired.contains(&column),
                    };
                }
                let mut crossings: Vec<Crossing> = Vec::new();
                let mut pieces: Vec<FloorPiece> = Vec::new();
                for source in &sources {
                    let c = source.column(column);
                    crossings.extend(c.crossings.iter().map(|e| e.1));
                    pieces.extend(c.pieces.iter().map(|e| e.1));
                }
                pair_column(column, crossings, &pieces, terrain)
            })
            .collect()
    }

    fn commit(
        &mut self,
        graph: &mut SpanGraph,
        coord: SpanChunkCoord,
        columns: Vec<PairedColumn>,
        generation: u32,
    ) {
        let mut lists = Vec::with_capacity(COLUMNS_PER_CHUNK);
        for (local, paired) in columns.into_iter().enumerate() {
            let column = coord.column(local);
            if paired.repaired {
                if self.repaired.insert(column) {
                    self.stats.parity_repairs += 1;
                }
            } else {
                self.repaired.remove(&column);
            }
            self.stats.columns_paired += 1;
            lists.push(paired.spans);
        }
        graph.replace_chunk(coord, SpanChunk::from_columns(&lists, generation));
    }

    /// Map each old span to the new span containing `(centre, floor_c + ε)`,
    /// and hold the new span's floor no higher than the old span it rests on.
    ///
    /// Two things can lift a floor without adding material. Re-triangulation
    /// can raise `floor_c` by a fraction of a voxel (see
    /// [`RETRIANGULATION_TOLERANCE`]); a larger rise is a broken invariant.
    /// And a new span opening below, a pocket blown under a ledge, takes the
    /// floor pieces under its ceiling into its own band, which lifts the old
    /// span's `floor_min` to the ledge. Both are held at the old height, so
    /// that everything downstream can rely on floors and saddles that only
    /// drop (§3.2).
    fn remap_column(
        &mut self,
        column: Column,
        old: &[Span],
        old_owners: &[SpanOwner],
        new: &mut [Span],
        voxel_size: f32,
        remap: &mut SpanRemap,
    ) {
        let mut claimed = [false; u8::MAX as usize + 1];
        for (ordinal, span) in old.iter().enumerate() {
            let y = span.floor_c + REMAP_EPSILON;
            let new_ordinal = new.iter().position(|s| s.ceiling > y);
            let rests = new_ordinal.is_none_or(|n| {
                let sliver = span.ceiling - span.floor_c < voxel_size;
                !(sliver && new[n].floor_c > y)
            });
            if let Some(n) = new_ordinal.filter(|_| rests) {
                // Old spans are visited bottom-up: the first to land in a new
                // span is the one it rests on.
                if !std::mem::replace(&mut claimed[n], true) {
                    self.hold_floor(column, span, &mut new[n], voxel_size);
                }
            } else if new_ordinal.is_some() {
                self.stats.closed_slivers += 1;
            }
            remap.entries.push(RemapEntry {
                column,
                old_ordinal: ordinal as u8,
                old: *span,
                old_owner: old_owners.get(ordinal).copied().unwrap_or_default(),
                new_ordinal: new_ordinal.map(|n| n as u8),
                rests,
            });
        }
    }

    /// Keep a re-paired span's floor from rising above the old span it rests
    /// on.
    fn hold_floor(&mut self, column: Column, old: &Span, new: &mut Span, voxel_size: f32) {
        if new.floor_c > old.floor_c {
            let rise = (new.floor_c - old.floor_c) / voxel_size;
            if rise <= RETRIANGULATION_TOLERANCE {
                self.stats.floor_clamps += 1;
                self.stats.worst_clamp = self.stats.worst_clamp.max(rise);
                new.floor_c = old.floor_c;
            } else {
                self.report_violation(column, old.floor_c, new.floor_c);
            }
        }
        if new.floor_min > old.floor_min {
            self.stats.floor_min_holds += 1;
            new.floor_min = old.floor_min;
        }
        new.floor_max = new.floor_max.max(new.floor_c);
        new.floor_min = new.floor_min.min(new.floor_c);
    }

    fn report_violation(&mut self, column: Column, old: f32, new: f32) {
        self.stats.invariant_violations += 1;
        self.stats.last_violation = Some((column, old, new));
        log::error!(
            "terrain edit raised a floor at column ({}, {}): {old:.3} -> {new:.3}; \
             terrain edits must only remove material",
            column.i,
            column.k,
        );
        debug_assert!(false, "removal-only invariant broken at {column:?}");
    }
}

/// Tiles whose squares overlap a box's XZ footprint grown by `margin`.
fn tiles_over(region: &AABB, margin: f32) -> impl Iterator<Item = SpanChunkCoord> {
    let min = Column::containing(region.min.x - margin, region.min.z - margin).chunk();
    let max = Column::containing(region.max.x + margin, region.max.z + margin).chunk();
    (min.z..=max.z).flat_map(move |z| (min.x..=max.x).map(move |x| SpanChunkCoord { x, z }))
}

/// Pair one column's crossings into spans.
fn pair_column(
    column: Column,
    mut crossings: Vec<Crossing>,
    pieces: &[FloorPiece],
    terrain: &TerrainWorld,
) -> PairedColumn {
    crossings.sort_by(|a, b| a.y.total_cmp(&b.y).then(a.facing.cmp(&b.facing)));
    let crossings = drop_zero_height_pairs(crossings);

    let (intervals, repaired) = match alternating_intervals(&crossings) {
        Some(intervals) => (intervals, false),
        None => (repaired_intervals(column, &crossings, terrain), true),
    };

    let mut spans: Vec<Span> = intervals
        .into_iter()
        .map(|(floor, ceiling)| Span {
            floor_c: floor,
            floor_min: floor,
            floor_max: floor,
            ceiling,
        })
        .collect();
    fold_floor_bands(&mut spans, pieces);
    PairedColumn { spans, repaired }
}

/// Remove floor/ceiling pairs that meet at one height: a silhouette edge seen
/// from both of its triangles.
fn drop_zero_height_pairs(crossings: Vec<Crossing>) -> Vec<Crossing> {
    let mut out: Vec<Crossing> = Vec::with_capacity(crossings.len());
    for crossing in crossings {
        if let Some(last) = out.last() {
            if last.facing != crossing.facing && (crossing.y - last.y).abs() < ZERO_HEIGHT {
                out.pop();
                continue;
            }
        }
        out.push(crossing);
    }
    out
}

/// Air intervals from crossings that alternate correctly, or `None` if they
/// do not. Air below the lowest crossing has no floor and is not a span.
fn alternating_intervals(crossings: &[Crossing]) -> Option<Vec<(f32, f32)>> {
    let mut intervals = Vec::new();
    let mut floor: Option<f32> = None;
    for (index, crossing) in crossings.iter().enumerate() {
        match (crossing.facing, floor) {
            (Facing::Floor, None) => floor = Some(crossing.y),
            (Facing::Ceiling, Some(f)) => {
                intervals.push((f, crossing.y));
                floor = None;
            }
            // Only the very first crossing may be a ceiling: the underside of
            // the world, with floorless air below it.
            (Facing::Ceiling, None) if index == 0 => {}
            _ => return None,
        }
    }
    if let Some(f) = floor {
        intervals.push((f, f32::INFINITY));
    }
    Some(intervals)
}

/// Air intervals decided by testing each gap between crossings for solidity
/// against the mesh itself.
fn repaired_intervals(
    column: Column,
    crossings: &[Crossing],
    terrain: &TerrainWorld,
) -> Vec<(f32, f32)> {
    let (x, z) = column.centre();
    let mut intervals: Vec<(f32, f32)> = Vec::new();
    for (index, crossing) in crossings.iter().enumerate() {
        let above = crossings.get(index + 1).map_or(f32::INFINITY, |c| c.y);
        let probe = if above.is_finite() {
            (crossing.y + above) * 0.5
        } else {
            crossing.y + 1.0
        };
        if terrain.is_mesh_solid_at(x, probe, z) {
            continue;
        }
        match intervals.last_mut() {
            Some(last) if last.1 == crossing.y => last.1 = above,
            _ => intervals.push((crossing.y, above)),
        }
    }
    intervals
}

/// Fold each clipped floor piece into the span whose band it lies in. A span's
/// band runs from the ceiling of the span below (−∞ for the lowest) up to its
/// own ceiling.
fn fold_floor_bands(spans: &mut [Span], pieces: &[FloorPiece]) {
    let mut band_lo = f32::NEG_INFINITY;
    for span in spans.iter_mut() {
        let band_hi = span.ceiling;
        for piece in pieces {
            let lo = piece.y_min.max(band_lo);
            let hi = piece.y_max.min(band_hi);
            if lo <= hi {
                span.floor_min = span.floor_min.min(lo);
                span.floor_max = span.floor_max.max(hi);
            }
        }
        band_lo = band_hi;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floor(y: f32) -> Crossing {
        Crossing {
            y,
            facing: Facing::Floor,
        }
    }

    fn ceiling(y: f32) -> Crossing {
        Crossing {
            y,
            facing: Facing::Ceiling,
        }
    }

    #[test]
    fn an_island_over_ground_is_two_spans() {
        let intervals = alternating_intervals(&[floor(0.0), ceiling(4.0), floor(8.0)]).unwrap();
        assert_eq!(intervals, vec![(0.0, 4.0), (8.0, f32::INFINITY)]);
    }

    #[test]
    fn the_underside_of_the_world_is_not_a_span() {
        let intervals = alternating_intervals(&[ceiling(-16.0), floor(0.0)]).unwrap();
        assert_eq!(intervals, vec![(0.0, f32::INFINITY)]);
    }

    #[test]
    fn two_floors_in_a_row_break_parity() {
        assert!(alternating_intervals(&[floor(0.0), floor(1.0)]).is_none());
    }

    #[test]
    fn a_silhouette_pair_is_dropped() {
        let out = drop_zero_height_pairs(vec![floor(0.0), ceiling(3.0), floor(3.0)]);
        assert_eq!(out, vec![floor(0.0)]);
    }

    #[test]
    fn floor_pieces_fold_into_their_own_band() {
        let mut spans = vec![
            Span {
                floor_c: 0.0,
                floor_min: 0.0,
                floor_max: 0.0,
                ceiling: 4.0,
            },
            Span {
                floor_c: 8.0,
                floor_min: 8.0,
                floor_max: 8.0,
                ceiling: f32::INFINITY,
            },
        ];
        let pieces = [
            FloorPiece {
                y_min: -0.3,
                y_max: 0.2,
            },
            FloorPiece {
                y_min: 7.9,
                y_max: 8.1,
            },
        ];
        fold_floor_bands(&mut spans, &pieces);
        assert_eq!((spans[0].floor_min, spans[0].floor_max), (-0.3, 0.2));
        assert_eq!((spans[1].floor_min, spans[1].floor_max), (7.9, 8.1));
    }
}
