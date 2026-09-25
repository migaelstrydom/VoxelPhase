//! The flood that finds a basin's region, its crests and its hierarchy (§7.2).
//!
//! ```text
//!   seeds ──wet flood at `level`──▶ everything under the water
//!         └─climb above it─────────▶ the dry banks up to the spill (+ headroom)
//!                                     every descent ends the region: a crest
//! ```
//!
//! **Lumped, so connected.** One level stands over the whole region, which is
//! exact only if at every level the spans below it form one connected set.
//! The flood enforces that the way Priority-Flood does: climbing out from the
//! water, any step *down* into a span ends the region there and becomes a
//! crest, classified by where water crossing it would go:
//!
//! - **outlet** if the far side drains away below the saddle,
//! - **pothole** if it is a tiny pit (§7.4); it is absorbed, its floor raised
//!   to the saddle and its volume held as dead storage that fills there,
//! - **child** otherwise: a depression of its own.
//!
//! A region span that opens onto the void is an outlet crest of its own, at
//! its floor. The climb stops a little above the lowest crest of any kind:
//! water spills there, and never stands much higher.
//!
//! Under the water the question is different: everything connected below the
//! level is one body already, so the wet flood takes it all, stopping only at
//! outlets. The ridges inside it are found afterwards by merging its spans in
//! order of saddle: each merge of two real depressions is a merge saddle, where
//! the lake splits when it drains below it.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

use rustc_hash::{FxHashMap, FxHashSet};

use crate::water::geometry::{DrainageField, Span, SpanGraph, SpanRef, COLUMN_SIZE};
use crate::water::ids::StoreId;

use super::hypsometry::DeadStorage;

/// A pit no deeper than this, and holding no more than
/// [`POTHOLE_VOLUME`], is absorbed rather than made a basin (§7.4).
pub const POTHOLE_DEPTH: f32 = 0.3;
pub const POTHOLE_VOLUME: f64 = 1.0;

/// How far above its lowest crest (or its level, if higher) a region's banks
/// are flooded, so that a basin rising over its lip finds the crest cells
/// beside it already recorded. A basin nearing the top of its headroom is
/// re-flooded. Kept small: a bank a few centimetres above the lip can be a
/// whole plateau.
pub const HEADROOM: f32 = 0.05;

/// The furthest above its level a region's banks are flooded, however far
/// off its lowest crest. The banks matter only once the water reaches them,
/// and a basin rising towards the top of its region is re-flooded then; a
/// pond drained into a cave system ten metres down need not claim the plain
/// its crests lie on.
pub const CLIMB_LIMIT: f32 = 0.5;

const CELL_AREA: f64 = (COLUMN_SIZE * COLUMN_SIZE) as f64;

/// Where water crossing a crest goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrestKind {
    /// Drains away below the saddle.
    Outlet,
    /// Into a depression of its own, owned by `owner` if a store holds it.
    Child { owner: Option<StoreId> },
}

/// One cell of a crest: a step down from a region span across its boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrestCell {
    pub inside: SpanRef,
    /// The span beyond the lip; the same as `inside` where the lip is the
    /// edge of the world, or an outlet the drainage field was given.
    pub outside: SpanRef,
    /// The lip: the lowest level water crosses here.
    pub saddle: f32,
    pub kind: CrestKind,
}

/// A ridge inside a region where two real depressions meet. The lake splits
/// here when it drains below the saddle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MergeSaddle {
    pub saddle: f32,
    /// A span on each side of the ridge.
    pub a: SpanRef,
    pub b: SpanRef,
}

/// One span of a region, as the hypsometry sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RegionSpan {
    pub span: SpanRef,
    /// The span's geometry, with an absorbed pothole's floor raised to its
    /// rim.
    pub shape: Span,
}

/// What a flood found.
#[derive(Debug, Clone, Default)]
pub struct Flood {
    pub region: Vec<RegionSpan>,
    pub dead: Vec<DeadStorage>,
    pub crests: Vec<CrestCell>,
    pub merge_saddles: Vec<MergeSaddle>,
    /// The highest level the region was flooded to.
    pub cap: f32,
}

impl Flood {
    /// The lowest outlet saddle, if the region has an outlet.
    pub fn spill(&self) -> Option<f32> {
        self.crests
            .iter()
            .filter(|c| c.kind == CrestKind::Outlet)
            .map(|c| c.saddle)
            .reduce(f32::min)
    }
}

/// What a flood is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloodMode {
    /// Placing water: everything connected under the level is one body.
    Create,
    /// Re-flooding a basin after an edit: its own spans are one body, but a
    /// dry depression it can now reach under its level fills over a weir.
    Reregion,
}

/// Finds basin regions over a span graph.
pub struct DepressionFinder<'a> {
    pub graph: &'a SpanGraph,
    pub drainage: &'a DrainageField,
    /// The store the region is for, if it exists yet: spans it owns are its
    /// own, spans any other store owns are a boundary.
    pub store: Option<StoreId>,
    /// Which store owns a span, if any.
    pub owner: &'a dyn Fn(SpanRef) -> Option<StoreId>,
    pub mode: FloodMode,
}

/// A frontier entry for the climbing flood, lowest saddle first.
#[derive(Debug, Clone, Copy)]
struct Step {
    saddle: f32,
    to: SpanRef,
    from: SpanRef,
}

impl PartialEq for Step {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Step {}
impl PartialOrd for Step {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Step {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .saddle
            .total_cmp(&self.saddle)
            .then_with(|| other.to.cmp(&self.to))
            .then_with(|| other.from.cmp(&self.from))
    }
}

impl DepressionFinder<'_> {
    /// Flood from `seeds` with water standing at `level`.
    pub fn flood(&self, seeds: &[SpanRef], level: f32) -> Flood {
        let mut flood = Flood::default();
        let mut index: FxHashMap<SpanRef, usize> = FxHashMap::default();
        index.reserve(seeds.len() * 2);
        flood.region.reserve(seeds.len() * 2);
        let add = |flood: &mut Flood,
                   index: &mut FxHashMap<SpanRef, usize>,
                   span: SpanRef,
                   shape: Span| {
            index.insert(span, flood.region.len());
            flood.region.push(RegionSpan { span, shape });
        };

        // Under the water: everything connected below the level.
        let mut frontier: BinaryHeap<Step> = BinaryHeap::new();
        let mut queue: VecDeque<SpanRef> = VecDeque::new();
        let mut crossed: FxHashSet<(SpanRef, SpanRef)> = FxHashSet::default();
        for &seed in seeds {
            if !index.contains_key(&seed) {
                add(&mut flood, &mut index, seed, *self.graph.span(seed));
                self.void_crest(&mut flood, &mut crossed, seed);
                queue.push_back(seed);
            }
        }
        while let Some(s) = queue.pop_front() {
            for n in self.graph.orthogonal_neighbours(s) {
                if index.contains_key(&n.span) {
                    continue;
                }
                if n.saddle >= level {
                    frontier.push(Step {
                        saddle: n.saddle,
                        to: n.span,
                        from: s,
                    });
                    continue;
                }
                if let Some(other) = self.foreign(n.span) {
                    self.crest(
                        &mut flood,
                        &mut crossed,
                        s,
                        n.span,
                        n.saddle,
                        CrestKind::Child { owner: Some(other) },
                    );
                    continue;
                }
                let far = *self.graph.span(n.span);
                let descent = far.floor_min < n.saddle;
                if descent && self.drainage.fill(self.graph, n.span) < n.saddle {
                    self.crest(
                        &mut flood,
                        &mut crossed,
                        s,
                        n.span,
                        n.saddle,
                        CrestKind::Outlet,
                    );
                    continue;
                }
                // A basin re-flooding keeps its own spans and any new air
                // at its level, but a dry depression it can now reach is a
                // depression of its own, filled over a weir (§7.2).
                let unowned = (self.owner)(n.span) != self.store || self.store.is_none();
                if descent && unowned && self.mode == FloodMode::Reregion {
                    if let Some((pit, volume)) = self.pothole(n.span, n.saddle, &index) {
                        flood.dead.push(DeadStorage {
                            rim: n.saddle,
                            volume,
                        });
                        for span in pit {
                            let raised = Span {
                                floor_c: n.saddle,
                                floor_min: n.saddle,
                                floor_max: n.saddle.max(self.graph.span(span).floor_max),
                                ..*self.graph.span(span)
                            };
                            add(&mut flood, &mut index, span, raised);
                            queue.push_back(span);
                        }
                        continue;
                    }
                    self.crest(
                        &mut flood,
                        &mut crossed,
                        s,
                        n.span,
                        n.saddle,
                        CrestKind::Child { owner: None },
                    );
                    continue;
                }
                add(&mut flood, &mut index, n.span, far);
                self.void_crest(&mut flood, &mut crossed, n.span);
                queue.push_back(n.span);
            }
        }

        // Above it: climb the banks, ending the region at every descent, and
        // stopping a little above the lowest crest.
        let mut cap = if flood.crests.is_empty() {
            level + CLIMB_LIMIT
        } else {
            level
        };
        let mut absorbed: FxHashSet<SpanRef> = FxHashSet::default();
        while let Some(Step { saddle, to, from }) = frontier.pop() {
            if saddle > cap + HEADROOM {
                break;
            }
            if index.contains_key(&to) || absorbed.contains(&to) {
                continue;
            }
            if let Some(other) = self.foreign(to) {
                self.crest(
                    &mut flood,
                    &mut crossed,
                    from,
                    to,
                    saddle,
                    CrestKind::Child { owner: Some(other) },
                );
                cap = cap.min(saddle.max(level));
                continue;
            }
            let far = *self.graph.span(to);
            if far.floor_min < saddle {
                if self.drainage.fill(self.graph, to) < saddle {
                    self.crest(
                        &mut flood,
                        &mut crossed,
                        from,
                        to,
                        saddle,
                        CrestKind::Outlet,
                    );
                    cap = cap.min(saddle.max(level));
                    continue;
                }
                if let Some((pit, volume)) = self.pothole(to, saddle, &index) {
                    flood.dead.push(DeadStorage {
                        rim: saddle,
                        volume,
                    });
                    for span in pit {
                        let raised = Span {
                            floor_c: saddle,
                            floor_min: saddle,
                            floor_max: saddle.max(self.graph.span(span).floor_max),
                            ..*self.graph.span(span)
                        };
                        add(&mut flood, &mut index, span, raised);
                        absorbed.insert(span);
                        self.push_frontier(&mut frontier, span, &index);
                    }
                    continue;
                }
                self.crest(
                    &mut flood,
                    &mut crossed,
                    from,
                    to,
                    saddle,
                    CrestKind::Child { owner: None },
                );
                cap = cap.min(saddle.max(level));
                continue;
            }
            add(&mut flood, &mut index, to, far);
            if self.void_crest(&mut flood, &mut crossed, to) {
                cap = cap.min(far.floor_min.max(level));
            }
            self.push_frontier(&mut frontier, to, &index);
        }
        flood.cap = if cap.is_finite() {
            cap + HEADROOM
        } else {
            f32::INFINITY
        };
        flood.merge_saddles = self.merge_saddles(&flood.region, &index, level);
        flood
    }

    /// The store owning a span, if it is not this one.
    fn foreign(&self, span: SpanRef) -> Option<StoreId> {
        (self.owner)(span).filter(|owner| Some(*owner) != self.store)
    }

    fn crest(
        &self,
        flood: &mut Flood,
        crossed: &mut FxHashSet<(SpanRef, SpanRef)>,
        inside: SpanRef,
        outside: SpanRef,
        saddle: f32,
        kind: CrestKind,
    ) {
        if crossed.insert((inside, outside)) {
            flood.crests.push(CrestCell {
                inside,
                outside,
                saddle,
                kind,
            });
        }
    }

    /// Record a region span that opens onto the void as an outlet crest at
    /// its floor. Returns whether it does.
    fn void_crest(
        &self,
        flood: &mut Flood,
        crossed: &mut FxHashSet<(SpanRef, SpanRef)>,
        span: SpanRef,
    ) -> bool {
        let Some(level) = self.drainage.outlets().level(self.graph, span) else {
            return false;
        };
        self.crest(flood, crossed, span, span, level, CrestKind::Outlet);
        true
    }

    fn push_frontier(
        &self,
        frontier: &mut BinaryHeap<Step>,
        from: SpanRef,
        index: &FxHashMap<SpanRef, usize>,
    ) {
        for n in self.graph.orthogonal_neighbours(from) {
            if !index.contains_key(&n.span) {
                frontier.push(Step {
                    saddle: n.saddle,
                    to: n.span,
                    from,
                });
            }
        }
    }

    /// The pit behind a descent into `span` at `rim`, if it is a pothole: its
    /// spans and the volume they hold below the rim.
    fn pothole(
        &self,
        span: SpanRef,
        rim: f32,
        region: &FxHashMap<SpanRef, usize>,
    ) -> Option<(Vec<SpanRef>, f64)> {
        let mut pit = vec![span];
        let mut seen = FxHashSet::from_iter([span]);
        let mut volume = 0.0f64;
        let mut cursor = 0;
        while cursor < pit.len() {
            let s = pit[cursor];
            cursor += 1;
            let shape = self.graph.span(s);
            if rim - shape.floor_min > POTHOLE_DEPTH || self.foreign(s).is_some() {
                return None;
            }
            let mean = 0.5 * (shape.floor_min + shape.floor_max.min(rim));
            volume += ((rim - mean).max(0.0)) as f64 * CELL_AREA;
            if volume > POTHOLE_VOLUME {
                return None;
            }
            for n in self.graph.orthogonal_neighbours(s) {
                if n.saddle < rim
                    && self.graph.span(n.span).floor_min < rim
                    && !region.contains_key(&n.span)
                    && seen.insert(n.span)
                {
                    pit.push(n.span);
                }
            }
        }
        Some((pit, volume))
    }

    /// Merge the wet spans in order of saddle; every merge of two real
    /// depressions is a ridge the lake splits at.
    fn merge_saddles(
        &self,
        region: &[RegionSpan],
        index: &FxHashMap<SpanRef, usize>,
        level: f32,
    ) -> Vec<MergeSaddle> {
        let mut edges: Vec<(f32, usize, usize)> = Vec::new();
        for (i, r) in region.iter().enumerate() {
            if r.shape.floor_min >= level {
                continue;
            }
            for n in self.graph.orthogonal_neighbours(r.span) {
                if let Some(&j) = index.get(&n.span) {
                    if i < j && n.saddle < level && region[j].shape.floor_min < level {
                        edges.push((n.saddle, i, j));
                    }
                }
            }
        }
        edges.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));

        let mut parent: Vec<usize> = (0..region.len()).collect();
        let mut lowest: Vec<f32> = region.iter().map(|r| r.shape.floor_min).collect();
        let mut count = vec![1usize; region.len()];
        let mut floor_sum: Vec<f64> = region.iter().map(|r| r.shape.floor_min as f64).collect();
        let mut out = Vec::new();
        for (saddle, i, j) in edges {
            let (a, b) = (root(&mut parent, i), root(&mut parent, j));
            if a == b {
                continue;
            }
            let real = |k: usize| {
                let depth = saddle - lowest[k];
                let volume = (saddle as f64 * count[k] as f64 - floor_sum[k]) * CELL_AREA;
                depth > POTHOLE_DEPTH || volume > POTHOLE_VOLUME
            };
            if real(a) && real(b) {
                out.push(MergeSaddle {
                    saddle,
                    a: region[i].span,
                    b: region[j].span,
                });
            }
            let (keep, gone) = if count[a] >= count[b] { (a, b) } else { (b, a) };
            parent[gone] = keep;
            lowest[keep] = lowest[keep].min(lowest[gone]);
            count[keep] += count[gone];
            floor_sum[keep] += floor_sum[gone];
        }
        out
    }
}

fn root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// Whether the pit water at `span` would fill to `rim` is a pothole (§7.4):
/// no deeper than [`POTHOLE_DEPTH`] and holding no more than
/// [`POTHOLE_VOLUME`].
pub fn is_pothole(graph: &SpanGraph, span: SpanRef, rim: f32) -> bool {
    let mut pit = vec![span];
    let mut seen = FxHashSet::from_iter([span]);
    let mut volume = 0.0f64;
    let mut cursor = 0;
    while cursor < pit.len() {
        let s = pit[cursor];
        cursor += 1;
        let shape = graph.span(s);
        if rim - shape.floor_min > POTHOLE_DEPTH {
            return false;
        }
        let mean = 0.5 * (shape.floor_min + shape.floor_max.min(rim));
        volume += ((rim - mean).max(0.0)) as f64 * CELL_AREA;
        if volume > POTHOLE_VOLUME {
            return false;
        }
        for n in graph.orthogonal_neighbours(s) {
            if n.saddle < rim && graph.span(n.span).floor_min < rim && seen.insert(n.span) {
                pit.push(n.span);
            }
        }
    }
    true
}

/// The bottom of the pit a span lies in: walk to the lowest neighbouring
/// floor until none is lower.
pub fn pit_bottom(graph: &SpanGraph, span: SpanRef) -> SpanRef {
    pit_bottom_within(graph, span, |_| true)
}

/// As [`pit_bottom`], walking only onto spans `within` admits: the bottom of
/// a pit opened into another's side is its own, not the other's.
pub fn pit_bottom_within(
    graph: &SpanGraph,
    mut span: SpanRef,
    within: impl Fn(SpanRef) -> bool,
) -> SpanRef {
    while let Some(next) = downhill_within(graph, span, &within) {
        span = next;
    }
    span
}

/// The lowest neighbour water on `span` runs down to, over the real floor
/// rather than the filled surface: the way down a pit's dry side. `None` at
/// the bottom.
pub fn downhill(graph: &SpanGraph, span: SpanRef) -> Option<SpanRef> {
    downhill_within(graph, span, &|_| true)
}

fn downhill_within(
    graph: &SpanGraph,
    span: SpanRef,
    within: &dyn Fn(SpanRef) -> bool,
) -> Option<SpanRef> {
    let here = graph.span(span).floor_min;
    graph
        .orthogonal_neighbours(span)
        .into_iter()
        .filter(|n| {
            graph.span(n.span).floor_min < here && n.saddle <= here + 1e-6 && within(n.span)
        })
        .min_by(|a, b| {
            graph
                .span(a.span)
                .floor_min
                .total_cmp(&graph.span(b.span).floor_min)
                .then(a.span.cmp(&b.span))
        })
        .map(|n| n.span)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::{Column, Outlets, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};

    fn open(floor: f32) -> Span {
        Span {
            floor_c: floor,
            floor_min: floor,
            floor_max: floor,
            ceiling: f32::INFINITY,
        }
    }

    fn graph(f: impl Fn(i32, i32) -> Option<f32>) -> SpanGraph {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let c = coord.column(local);
                f(c.i, c.k).map(open).into_iter().collect()
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        graph
    }

    /// Two 1 m-deep pits on a plateau at 2 m, joined by a ridge at 1.5 m,
    /// with a notch at 1.8 m out of the west pit to the void.
    fn twin_pits() -> SpanGraph {
        graph(|i, k| {
            if i == 0 || k == 0 || i == 15 || k == 15 {
                return None;
            }
            if (3..=6).contains(&i) && (5..=9).contains(&k) {
                Some(1.0)
            } else if i == 7 && (5..=9).contains(&k) {
                Some(1.5)
            } else if (8..=11).contains(&i) && (5..=9).contains(&k) {
                Some(1.0)
            } else if k == 7 && (1..=2).contains(&i) {
                Some(1.8)
            } else {
                Some(2.0)
            }
        })
    }

    fn finder<'a>(
        g: &'a SpanGraph,
        d: &'a DrainageField,
        owner: &'a dyn Fn(SpanRef) -> Option<StoreId>,
    ) -> DepressionFinder<'a> {
        DepressionFinder {
            graph: g,
            drainage: d,
            store: None,
            owner,
            mode: FloodMode::Create,
        }
    }

    #[test]
    fn a_lake_over_twin_pits_records_the_ridge() {
        let g = twin_pits();
        let d = DrainageField::build(&g, Outlets::default());
        let none = |_: SpanRef| None;
        let seed = g.make_ref(Column::new(4, 7), 0);
        let flood = finder(&g, &d, &none).flood(&[seed], 1.7);
        let wet = flood
            .region
            .iter()
            .filter(|r| r.shape.floor_min < 1.7)
            .count();
        assert_eq!(wet, 4 * 5 * 2 + 5, "both pits and the ridge");
        assert_eq!(flood.merge_saddles.len(), 1);
        assert_eq!(flood.merge_saddles[0].saddle, 1.5);
        assert_eq!(flood.spill(), Some(1.8));
    }

    #[test]
    fn a_dry_pit_climbs_to_its_ridge_and_stops() {
        let g = twin_pits();
        let d = DrainageField::build(&g, Outlets::default());
        let none = |_: SpanRef| None;
        // The east pit, empty. Its lowest crest is the ridge into the west
        // pit, which fills to 1.8 before it spills: a child.
        let seed = pit_bottom(&g, g.make_ref(Column::new(9, 7), 0));
        let flood = finder(&g, &d, &none).flood(&[seed], 1.0);
        let child = flood
            .crests
            .iter()
            .find(|c| matches!(c.kind, CrestKind::Child { .. }))
            .expect("the ridge is a child crest");
        assert_eq!(child.saddle, 1.5);
        assert!(flood.region.iter().all(|r| r.span.column.i >= 7));
    }
}
