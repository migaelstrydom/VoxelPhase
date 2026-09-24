//! The drainage field: for every span, how high still water would stand there
//! at steady state, and which way moving water leaves it.
//!
//! ```text
//!   outlets ──Priority-Flood──▶ fill levels ──D8 over the filled surface──▶ drains
//!                                             └─ flats: Barnes' flat resolution
//! ```
//!
//! **Fill level.** The lowest achievable maximum floor along any open path to
//! an outlet: minimax, computed by a Dijkstra over `max`. Outlets are spans
//! that open onto the void (every open world edge, and any hole to below the
//! world) at their floor, plus any the caller adds: the sea, authored sinks.
//!
//! **Drain.** Steepest descent over the filled surface, to a neighbour whose
//! fill is lower and whose saddle water can pass at this span's fill. Spans
//! with no such neighbour sit on a flat, and flats are resolved as Barnes et
//! al. (2014) do: towards the flat's lower edge and away from its higher one,
//! so water on a flat runs down its middle rather than hugging one side.
//!
//! **Repair.** Terrain edits only remove material (§3.2), so fill levels only
//! drop. A decrease-only Dijkstra seeded from the re-paired columns relaxes a
//! neighbour only while its fill actually drops; drains are then recomputed
//! around every span it touched. A flat those drains land on must be resolved
//! again as a whole, and a flat can be most of a level, so it is marked
//! pending rather than resolved on the blast frame: the router resolves it
//! the first time a channel asks which way water leaves one of its spans.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

use nalgebra::Point3;
use rustc_hash::{FxHashMap, FxHashSet};

use super::rasteriser::SpanRemap;
use super::span::{Column, SpanChunkCoord, SpanRef, COLUMNS_PER_CHUNK};
use super::span_graph::{Neighbour, SpanGraph, Step, EIGHT, ORTHOGONAL};

/// Which way water leaves a span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drain {
    /// No path to any outlet: a sealed pocket.
    Sealed,
    /// The span is itself an outlet.
    Outlet,
    /// On a flat whose resolution is pending; see
    /// [`DrainageField::downstream_resolved`].
    Pending,
    /// Down to the `ordinal`-th span of the column one `EIGHT[step]` away.
    To { step: u8, ordinal: u8 },
}

/// An authored box that swallows water: a span whose column centre lies
/// inside it in plan, with its floor inside it in height, drains away.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SinkBox {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl SinkBox {
    fn swallows(&self, column: Column, floor: f32) -> bool {
        let (x, z) = column.centre();
        (self.min.x..=self.max.x).contains(&x)
            && (self.min.z..=self.max.z).contains(&z)
            && (self.min.y..=self.max.y).contains(&floor)
    }
}

/// The sea beyond some edges of the map (§12). A span on an open edge drains
/// into it at sea level, or at its own floor where that stands higher.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeaEdges {
    pub level: f32,
    /// Which edges open onto it: −x, +x, −z, +z.
    pub open: [bool; 4],
}

impl SeaEdges {
    /// Whether the edge a step of `(di, dk)` crosses opens onto the sea.
    pub fn opens(&self, di: i32, dk: i32) -> bool {
        match (di, dk) {
            (-1, 0) => self.open[0],
            (1, 0) => self.open[1],
            (0, -1) => self.open[2],
            (0, 1) => self.open[3],
            _ => false,
        }
    }
}

/// Where water may leave the world, beyond the void.
///
/// Sinks are held as boxes and tested against the span in hand, so they
/// hold through every rebuild of the spans under them.
#[derive(Debug, Clone, Default)]
pub struct Outlets {
    /// Extra outlets and the level each drains at: the sea.
    extra: FxHashMap<SpanRef, f32>,
    sinks: Vec<SinkBox>,
    sea: Option<SeaEdges>,
}

impl Outlets {
    pub fn add(&mut self, span: SpanRef, level: f32) {
        let entry = self.extra.entry(span).or_insert(level);
        *entry = entry.min(level);
    }

    pub fn add_sink(&mut self, sink: SinkBox) {
        self.sinks.push(sink);
    }

    pub fn set_sea(&mut self, sea: SeaEdges) {
        self.sea = Some(sea);
    }

    pub fn sea(&self) -> Option<SeaEdges> {
        self.sea
    }

    /// Whether a column stands on an edge of the map that opens onto the sea.
    pub fn on_sea_edge(&self, graph: &SpanGraph, column: Column) -> bool {
        let Some(sea) = self.sea else {
            return false;
        };
        ORTHOGONAL.iter().any(|step| {
            let next = column.offset(step.di, step.dk);
            sea.opens(step.di, step.dk)
                && (!graph.contains_column(next) || graph.spans(next).is_empty())
        })
    }

    /// The level a span drains away at, if it is an outlet.
    pub fn level(&self, graph: &SpanGraph, span: SpanRef) -> Option<f32> {
        let floor = graph.span(span).floor_min;
        let edge = if self.on_sea_edge(graph, span.column) {
            let sea = self.sea.map_or(floor, |s| s.level);
            Some(floor.max(sea))
        } else {
            (graph.borders_void(span.column)
                || self.sinks.iter().any(|s| s.swallows(span.column, floor)))
            .then_some(floor)
        };
        match (edge, self.extra.get(&span)) {
            (Some(a), Some(b)) => Some(a.min(*b)),
            (a, b) => a.or(b.copied()),
        }
    }
}

/// One span chunk's drainage, parallel to its spans.
#[derive(Debug, Clone, Default)]
struct DrainChunk {
    /// The CSR offsets these arrays were laid out for.
    offsets: Vec<u16>,
    fill: Vec<f32>,
    drain: Vec<Drain>,
}

/// Counters from one repair.
#[derive(Debug, Clone, Copy, Default)]
pub struct RepairStats {
    /// Spans whose fill level dropped.
    pub lowered: usize,
    /// Spans whose drain was recomputed.
    pub redirected: usize,
    /// Flats resolved again.
    pub flats: usize,
}

/// Fill levels and drains for every span.
#[derive(Debug, Clone)]
pub struct DrainageField {
    /// Parallel to the graph's chunk slots.
    chunks: Vec<DrainChunk>,
    outlets: Outlets,
    /// Flats whose drains are out of date, by level (as bits), each named by
    /// the members found next to what changed. A span on one of them has a
    /// stale drain until it is resolved.
    pending: FxHashMap<u32, Vec<SpanRef>>,
}

/// A priority-queue entry, ordered by level then span, lowest first.
#[derive(Debug, Clone, Copy)]
struct Key {
    level: f32,
    span: SpanRef,
}

impl PartialEq for Key {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    /// Reversed, so that `BinaryHeap` pops the lowest level first.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .level
            .total_cmp(&self.level)
            .then_with(|| other.span.cmp(&self.span))
    }
}

impl DrainageField {
    /// Flood the whole graph from its outlets.
    pub fn build(graph: &SpanGraph, outlets: Outlets) -> Self {
        let mut field = Self {
            chunks: (0..graph.slot_count())
                .map(|slot| Self::empty_chunk(graph, slot))
                .collect(),
            outlets,
            pending: FxHashMap::default(),
        };
        let outlets = field.outlets.clone();
        let mut heap = BinaryHeap::new();
        for span in graph.all_refs() {
            if let Some(level) = outlets.level(graph, span) {
                field.set_fill(graph, span, level);
                heap.push(Key { level, span });
            }
        }
        field.flood(graph, heap, None);
        let all: Vec<SpanRef> = graph.all_refs().collect();
        field.redirect(graph, &outlets, &all);
        field
    }

    /// The outlets this field drains to.
    pub fn outlets(&self) -> &Outlets {
        &self.outlets
    }

    /// Bring the field up to date with a rebuild of the span graph.
    ///
    /// Re-paired spans carry over the drain of the old span they rest on. A
    /// crater in a plain leaves the plain's flat as it was, so its carried
    /// drains stay right, and re-resolving a flat the size of the level for
    /// every blast in it would be wasted. Flats are resolved again only where
    /// a fill level dropped, or where a carried drain no longer leads
    /// anywhere valid.
    pub fn repair(&mut self, graph: &SpanGraph, remap: &SpanRemap) -> RepairStats {
        let outlets = self.outlets.clone();
        let outlets = &outlets;
        let stale: FxHashSet<Column> = remap.columns.iter().copied().collect();
        let moved: FxHashMap<(Column, u8), Option<u8>> = remap
            .entries
            .iter()
            .map(|e| ((e.column, e.old_ordinal), e.new_ordinal))
            .collect();
        for &coord in &remap.chunks {
            self.relayout(graph, coord, remap, &stale);
        }

        // Seeds: every span in a re-paired column, and in the ring around it,
        // whose outlets can change when a column opens onto the void.
        let mut ring: Vec<Column> = stale
            .iter()
            .flat_map(|c| {
                std::iter::once(*c).chain(ORTHOGONAL.iter().map(move |s| c.offset(s.di, s.dk)))
            })
            .collect();
        ring.sort();
        ring.dedup();
        let seeds: Vec<SpanRef> = ring.iter().flat_map(|c| graph.refs(*c)).collect();
        for &span in &seeds {
            self.retarget(graph, span, &stale, &moved);
        }

        // Pull each seed down to what its neighbours and outlets allow, then
        // push every seed: its floor may have dropped under its neighbours.
        let before: Vec<f32> = seeds.iter().map(|s| self.fill(graph, *s)).collect();
        let mut heap = BinaryHeap::new();
        for &span in &seeds {
            let mut level = self.fill(graph, span);
            if let Some(outlet) = outlets.level(graph, span) {
                level = level.min(outlet);
            }
            for n in graph.neighbours(span) {
                level = level.min(self.fill(graph, n.span).max(n.saddle));
            }
            self.set_fill(graph, span, level);
            if level.is_finite() {
                heap.push(Key { level, span });
            }
        }
        let mut lowered: Vec<SpanRef> = Vec::new();
        self.flood(graph, heap, Some(&mut lowered));
        for (span, was) in seeds.iter().zip(&before) {
            if self.fill(graph, *span) < *was {
                lowered.push(*span);
            }
        }

        // Around every lowered span, drains and flats are recomputed outright.
        let mut must: Vec<SpanRef> = Vec::new();
        let mut in_must: FxHashSet<SpanRef> = FxHashSet::default();
        for &span in &lowered {
            for s in std::iter::once(span).chain(graph.neighbours(span).iter().map(|n| n.span)) {
                if in_must.insert(s) {
                    must.push(s);
                }
            }
        }
        let mut flat_cells: Vec<SpanRef> = Vec::new();
        for &span in &must {
            match self.steepest(graph, outlets, span) {
                Some(drain) => self.set_drain(graph, span, drain),
                None => flat_cells.push(span),
            }
        }
        // Elsewhere a seed keeps its carried drain if it is on a flat and the
        // drain is still valid.
        for &span in seeds.iter().filter(|s| !in_must.contains(s)) {
            match self.steepest(graph, outlets, span) {
                Some(drain) => self.set_drain(graph, span, drain),
                None if self.carried_drain_valid(graph, span) => {}
                None => flat_cells.push(span),
            }
        }
        // A pending flat's seed in a re-paired column follows its span; one
        // that fell into the void is dropped, and the edit that did it marks
        // the flat's cells around the hole pending again.
        for seeds in self.pending.values_mut() {
            seeds.retain_mut(|seed| {
                if !stale.contains(&seed.column) {
                    *seed = graph.make_ref(seed.column, seed.ordinal);
                    return true;
                }
                match moved.get(&(seed.column, seed.ordinal)).copied().flatten() {
                    Some(ordinal) => {
                        *seed = graph.make_ref(seed.column, ordinal);
                        true
                    }
                    None => false,
                }
            });
        }
        self.pending.retain(|_, seeds| !seeds.is_empty());
        let flats = flat_cells.len();
        for cell in flat_cells {
            let level = self.fill(graph, cell);
            self.set_drain(graph, cell, Drain::Pending);
            self.pending.entry(level.to_bits()).or_default().push(cell);
        }
        if !lowered.is_empty() {
            self.rekey_pending(graph);
        }
        RepairStats {
            lowered: lowered.len(),
            redirected: must.len() + seeds.len(),
            flats,
        }
    }

    /// The span water leaving `span` flows into, resolving the flat it lies
    /// on first if that flat is pending.
    pub fn downstream_resolved(&mut self, graph: &SpanGraph, span: SpanRef) -> Option<SpanRef> {
        let level = self.fill(graph, span);
        if self.pending.contains_key(&level.to_bits()) {
            self.resolve_containing(graph, span);
        }
        self.downstream(graph, span)
    }

    /// Resolve every pending flat, lowest level first.
    pub fn resolve_pending(&mut self, graph: &SpanGraph) {
        let mut levels: Vec<u32> = self.pending.keys().copied().collect();
        levels.sort_by(|a, b| f32::from_bits(*a).total_cmp(&f32::from_bits(*b)));
        for level in levels {
            while let Some(seed) = self.pending.get(&level).and_then(|s| s.first().copied()) {
                self.resolve_containing(graph, seed);
                if let Some(seeds) = self.pending.get_mut(&level) {
                    seeds.retain(|s| *s != seed);
                }
            }
            self.pending.remove(&level);
        }
    }

    /// Levels with flats still waiting to be resolved.
    pub fn pending_flats(&self) -> usize {
        self.pending.len()
    }

    /// Resolve the flat holding `span`, if it is on one, and strike its
    /// members from the pending list.
    fn resolve_containing(&mut self, graph: &SpanGraph, span: SpanRef) {
        let outlets = self.outlets.clone();
        let level = self.fill(graph, span).to_bits();
        let members: Vec<SpanRef> = if self.steepest(graph, &outlets, span).is_some() {
            vec![span]
        } else {
            self.resolve_flat(graph, &outlets, span)
        };
        if let Some(seeds) = self.pending.get_mut(&level) {
            let members: FxHashSet<SpanRef> = members.into_iter().collect();
            seeds.retain(|seed| !members.contains(seed));
            if seeds.is_empty() {
                self.pending.remove(&level);
            }
        }
    }

    /// File every pending seed under its current fill: a later repair may
    /// have lowered the flat it was found on.
    fn rekey_pending(&mut self, graph: &SpanGraph) {
        let mut rekeyed: FxHashMap<u32, Vec<SpanRef>> = FxHashMap::default();
        let mut levels: Vec<u32> = self.pending.keys().copied().collect();
        levels.sort_unstable();
        for level in levels {
            for seed in self.pending.remove(&level).unwrap_or_default() {
                let fill = self.fill(graph, seed).to_bits();
                rekeyed.entry(fill).or_default().push(seed);
            }
        }
        for seeds in rekeyed.values_mut() {
            seeds.sort_unstable();
            seeds.dedup();
        }
        self.pending = rekeyed;
    }

    /// Point a carried drain whose target column was re-paired at the span
    /// its old target became.
    fn retarget(
        &mut self,
        graph: &SpanGraph,
        span: SpanRef,
        stale: &FxHashSet<Column>,
        moved: &FxHashMap<(Column, u8), Option<u8>>,
    ) {
        let Drain::To { step, ordinal } = self.drain(graph, span) else {
            return;
        };
        let offset = EIGHT[step as usize];
        let target = span.column.offset(offset.di, offset.dk);
        if !stale.contains(&target) {
            return;
        }
        let drain = match moved.get(&(target, ordinal)).copied().flatten() {
            Some(ordinal) => Drain::To { step, ordinal },
            None => Drain::Sealed,
        };
        self.set_drain(graph, span, drain);
    }

    /// Whether a flat span's drain still leads to a passable neighbour at or
    /// below its own fill.
    fn carried_drain_valid(&self, graph: &SpanGraph, span: SpanRef) -> bool {
        let fill = self.fill(graph, span);
        let Some(target) = self.downstream(graph, span) else {
            return false;
        };
        graph
            .neighbours(span)
            .iter()
            .any(|n| n.span == target && n.saddle <= fill && self.fill(graph, n.span) <= fill)
    }

    /// The fill level of a span: where still water would stand there.
    /// `f32::INFINITY` for a sealed pocket.
    pub fn fill(&self, graph: &SpanGraph, span: SpanRef) -> f32 {
        graph.index_of(span).map_or(f32::INFINITY, |i| {
            self.chunks[i.slot as usize].fill[i.flat as usize]
        })
    }

    pub fn drain(&self, graph: &SpanGraph, span: SpanRef) -> Drain {
        graph.index_of(span).map_or(Drain::Sealed, |i| {
            self.chunks[i.slot as usize].drain[i.flat as usize]
        })
    }

    /// The span water leaving `span` flows into, if it flows to a neighbour.
    pub fn downstream(&self, graph: &SpanGraph, span: SpanRef) -> Option<SpanRef> {
        match self.drain(graph, span) {
            Drain::To { step, ordinal } => {
                let step = EIGHT[step as usize];
                Some(graph.make_ref(span.column.offset(step.di, step.dk), ordinal))
            }
            _ => None,
        }
    }

    fn empty_chunk(graph: &SpanGraph, slot: usize) -> DrainChunk {
        let chunk = graph.chunk_at_slot(slot);
        let count = chunk.span_count();
        DrainChunk {
            offsets: chunk_offsets(graph, slot),
            fill: vec![f32::INFINITY; count],
            drain: vec![Drain::Sealed; count],
        }
    }

    fn set_fill(&mut self, graph: &SpanGraph, span: SpanRef, level: f32) {
        if let Some(i) = graph.index_of(span) {
            self.chunks[i.slot as usize].fill[i.flat as usize] = level;
        }
    }

    fn set_drain(&mut self, graph: &SpanGraph, span: SpanRef, drain: Drain) {
        if let Some(i) = graph.index_of(span) {
            self.chunks[i.slot as usize].drain[i.flat as usize] = drain;
        }
    }

    /// Minimax Dijkstra: relax neighbours only while their fill drops.
    fn flood(
        &mut self,
        graph: &SpanGraph,
        mut heap: BinaryHeap<Key>,
        mut lowered: Option<&mut Vec<SpanRef>>,
    ) {
        while let Some(Key { level, span }) = heap.pop() {
            if level > self.fill(graph, span) {
                continue;
            }
            for n in graph.neighbours(span) {
                let candidate = level.max(n.saddle);
                if candidate < self.fill(graph, n.span) {
                    self.set_fill(graph, n.span, candidate);
                    if let Some(lowered) = lowered.as_deref_mut() {
                        lowered.push(n.span);
                    }
                    heap.push(Key {
                        level: candidate,
                        span: n.span,
                    });
                }
            }
        }
    }

    /// Recompute the drains of `spans`, then resolve every flat any of them
    /// lies on. Returns the number of flats resolved.
    fn redirect(&mut self, graph: &SpanGraph, outlets: &Outlets, spans: &[SpanRef]) -> usize {
        let mut flat_cells: Vec<SpanRef> = Vec::new();
        for &span in spans {
            match self.steepest(graph, outlets, span) {
                Some(drain) => self.set_drain(graph, span, drain),
                None => flat_cells.push(span),
            }
        }
        self.resolve_flats(graph, outlets, flat_cells)
    }

    /// Resolve every flat holding one of `cells`, each once.
    fn resolve_flats(
        &mut self,
        graph: &SpanGraph,
        outlets: &Outlets,
        cells: Vec<SpanRef>,
    ) -> usize {
        let mut resolved: FxHashSet<SpanRef> = FxHashSet::default();
        let mut flats = 0;
        for cell in cells {
            if resolved.contains(&cell) {
                continue;
            }
            let members = self.resolve_flat(graph, outlets, cell);
            resolved.extend(members);
            flats += 1;
        }
        flats
    }

    /// The drain of a span that is an outlet, sealed, or has a lower
    /// neighbour it can pass to; `None` for a span on a flat.
    fn steepest(&self, graph: &SpanGraph, outlets: &Outlets, span: SpanRef) -> Option<Drain> {
        self.steepest_among(graph, outlets, span, &graph.neighbours(span))
    }

    /// As [`Self::steepest`], over neighbours already fetched.
    fn steepest_among(
        &self,
        graph: &SpanGraph,
        outlets: &Outlets,
        span: SpanRef,
        neighbours: &[Neighbour],
    ) -> Option<Drain> {
        let fill = self.fill(graph, span);
        if !fill.is_finite() {
            return Some(Drain::Sealed);
        }
        if outlets.level(graph, span).is_some_and(|l| l <= fill) {
            return Some(Drain::Outlet);
        }
        let mut best: Option<(f32, Neighbour)> = None;
        for &n in neighbours {
            let there = self.fill(graph, n.span);
            if there >= fill || n.saddle > fill {
                continue;
            }
            let slope = (fill - there) / n.step.length();
            if best.as_ref().map_or(true, |(s, _)| slope > *s) {
                best = Some((slope, n));
            }
        }
        best.map(|(_, n)| to(n))
    }

    /// Resolve the flat containing `seed`: every span reachable from it at
    /// the same fill. Returns its members.
    fn resolve_flat(
        &mut self,
        graph: &SpanGraph,
        outlets: &Outlets,
        seed: SpanRef,
    ) -> Vec<SpanRef> {
        let level = self.fill(graph, seed);

        // One pass fetches each member's neighbours once, and records what the
        // rest needs of them: whether the member drains off the flat (a low
        // edge), whether it borders higher ground (a high edge), and its
        // passable links to other members. A flat can be most of a level.
        let mut members: Vec<SpanRef> = vec![seed];
        let mut index: FxHashMap<SpanRef, u32> = FxHashMap::default();
        index.insert(seed, 0);
        let mut low_edges: Vec<usize> = Vec::new();
        let mut high_edges: Vec<usize> = Vec::new();
        let mut link_start: Vec<u32> = vec![0];
        let mut links: Vec<(SpanRef, u8)> = Vec::new();
        let mut cursor = 0;
        while cursor < members.len() {
            let span = members[cursor];
            let neighbours = graph.neighbours(span);
            if let Some(drain) = self.steepest_among(graph, outlets, span, &neighbours) {
                self.set_drain(graph, span, drain);
                low_edges.push(cursor);
            }
            let mut higher = false;
            for n in &neighbours {
                let there = self.fill(graph, n.span);
                higher |= there > level;
                if n.saddle <= level && there == level {
                    links.push((n.span, step_index(n.step)));
                    if !index.contains_key(&n.span) {
                        index.insert(n.span, members.len() as u32);
                        members.push(n.span);
                    }
                }
            }
            if higher {
                high_edges.push(cursor);
            }
            link_start.push(links.len() as u32);
            cursor += 1;
        }

        let adjacency = FlatAdjacency {
            start: link_start,
            target: links.iter().map(|(span, _)| index[span]).collect(),
        };
        let to_lower = adjacency.bfs(&low_edges);
        let from_higher = adjacency.bfs(&high_edges);
        let highest = from_higher
            .iter()
            .copied()
            .filter(|d| *d != u32::MAX)
            .max()
            .unwrap_or(0);
        let mask: Vec<u64> = to_lower
            .iter()
            .zip(&from_higher)
            .map(|(&low, &high)| {
                let away = if high == u32::MAX { 0 } else { highest - high };
                2 * low as u64 + away as u64
            })
            .collect();

        let is_low_edge: FxHashSet<usize> = low_edges.iter().copied().collect();
        for (i, &span) in members.iter().enumerate() {
            if is_low_edge.contains(&i) {
                continue;
            }
            let range = adjacency.start[i] as usize..adjacency.start[i + 1] as usize;
            let best = range
                .filter(|&l| mask[adjacency.target[l] as usize] < mask[i])
                .min_by_key(|&l| mask[adjacency.target[l] as usize]);
            let drain = match best {
                Some(l) => Drain::To {
                    step: links[l].1,
                    ordinal: links[l].0.ordinal,
                },
                None => Drain::Sealed,
            };
            self.set_drain(graph, span, drain);
        }
        members
    }

    /// Lay out a rebuilt chunk's arrays for its new spans. Untouched columns
    /// keep their values; each re-paired span starts from the lowest fill of
    /// the old spans that landed in it, an upper bound on its true fill.
    fn relayout(
        &mut self,
        graph: &SpanGraph,
        coord: SpanChunkCoord,
        remap: &SpanRemap,
        stale: &FxHashSet<Column>,
    ) {
        let Some(slot) = graph.slot(coord) else {
            return;
        };
        let old = std::mem::take(&mut self.chunks[slot]);
        let mut fresh = Self::empty_chunk(graph, slot);
        // Each new span takes the lowest fill of the old spans that landed in
        // it, and the drain of the lowest of them: the one it rests on.
        let mut landed: FxHashMap<(usize, u8), (f32, Drain)> = FxHashMap::default();
        for entry in remap.entries.iter().filter(|e| e.column.chunk() == coord) {
            let local = entry.column.local_index();
            let Some(new_ordinal) = entry.new_ordinal else {
                continue;
            };
            let old_index = old.offsets[local] as usize + entry.old_ordinal as usize;
            let old_fill = old.fill.get(old_index).copied().unwrap_or(f32::INFINITY);
            let old_drain = old.drain.get(old_index).copied().unwrap_or(Drain::Sealed);
            landed
                .entry((local, new_ordinal))
                .and_modify(|(fill, _)| *fill = fill.min(old_fill))
                .or_insert((old_fill, old_drain));
        }
        for local in 0..COLUMNS_PER_CHUNK {
            let column = coord.column(local);
            let new_start = fresh.offsets[local] as usize;
            let new_len = fresh.offsets[local + 1] as usize - new_start;
            if stale.contains(&column) {
                for ordinal in 0..new_len {
                    let (fill, drain) = landed
                        .get(&(local, ordinal as u8))
                        .copied()
                        .unwrap_or((f32::INFINITY, Drain::Sealed));
                    fresh.fill[new_start + ordinal] = fill;
                    fresh.drain[new_start + ordinal] = drain;
                }
            } else {
                let old_start = old.offsets[local] as usize;
                fresh.fill[new_start..new_start + new_len]
                    .copy_from_slice(&old.fill[old_start..old_start + new_len]);
                fresh.drain[new_start..new_start + new_len]
                    .copy_from_slice(&old.drain[old_start..old_start + new_len]);
            }
        }
        self.chunks[slot] = fresh;
    }
}

/// The drain to a neighbour, named by step index and ordinal.
fn to(n: Neighbour) -> Drain {
    Drain::To {
        step: step_index(n.step),
        ordinal: n.span.ordinal,
    }
}

fn step_index(step: Step) -> u8 {
    EIGHT
        .iter()
        .position(|s| *s == step)
        .expect("every neighbour step is one of the eight") as u8
}

/// Passable links between the members of one flat, in CSR form.
struct FlatAdjacency {
    /// `target[start[i]..start[i + 1]]` are member `i`'s neighbours.
    start: Vec<u32>,
    target: Vec<u32>,
}

impl FlatAdjacency {
    /// Breadth-first distances from a set of sources; `u32::MAX` where
    /// unreachable.
    fn bfs(&self, sources: &[usize]) -> Vec<u32> {
        let mut distance = vec![u32::MAX; self.start.len() - 1];
        let mut queue = VecDeque::new();
        for &s in sources {
            distance[s] = 0;
            queue.push_back(s);
        }
        while let Some(i) = queue.pop_front() {
            for &j in &self.target[self.start[i] as usize..self.start[i + 1] as usize] {
                let j = j as usize;
                if distance[j] == u32::MAX {
                    distance[j] = distance[i] + 1;
                    queue.push_back(j);
                }
            }
        }
        distance
    }
}

/// A chunk's CSR offsets, read back through its public column view.
fn chunk_offsets(graph: &SpanGraph, slot: usize) -> Vec<u16> {
    let chunk = graph.chunk_at_slot(slot);
    let mut offsets = Vec::with_capacity(COLUMNS_PER_CHUNK + 1);
    let mut total = 0u16;
    offsets.push(0);
    for local in 0..COLUMNS_PER_CHUNK {
        total += chunk.column_spans(local).len() as u16;
        offsets.push(total);
    }
    offsets
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::span::{Column, Span, SpanChunk};

    fn open(floor: f32) -> Span {
        Span {
            floor_c: floor,
            floor_min: floor,
            floor_max: floor,
            ceiling: f32::INFINITY,
        }
    }

    /// One chunk, 16 × 16 columns, floors from `f(i, k)`; `None` is void.
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

    /// A 14 × 14 plateau at 2 m, ringed by void, with a 1 m-deep pit.
    fn pit() -> SpanGraph {
        graph(|i, k| {
            if i == 0 || k == 0 || i == 15 || k == 15 {
                None
            } else if (6..=9).contains(&i) && (6..=9).contains(&k) {
                Some(1.0)
            } else {
                Some(2.0)
            }
        })
    }

    #[test]
    fn a_pit_fills_to_its_rim() {
        let g = pit();
        let field = DrainageField::build(&g, Outlets::default());
        let bottom = g.make_ref(Column::new(7, 7), 0);
        assert_eq!(field.fill(&g, bottom), 2.0);
        let rim = g.make_ref(Column::new(3, 3), 0);
        assert_eq!(field.fill(&g, rim), 2.0);
    }

    #[test]
    fn every_drain_path_reaches_an_outlet() {
        let g = pit();
        let field = DrainageField::build(&g, Outlets::default());
        for start in g.all_refs() {
            let mut span = start;
            let mut steps = 0;
            while let Some(next) = field.downstream(&g, span) {
                span = next;
                steps += 1;
                assert!(steps < 1000, "cycle from {start:?}");
            }
            assert_eq!(field.drain(&g, span), Drain::Outlet, "from {start:?}");
        }
    }

    #[test]
    fn a_flat_drains_down_its_middle() {
        // A dead-flat strip 1 m below its banks, open to the void only at
        // i = 0. Water on the strip should run along it, not to a bank.
        let g = graph(|i, k| {
            if i == 0 {
                (5..=9).contains(&k).then_some(0.0)
            } else if (5..=9).contains(&k) && i < 14 {
                Some(0.0)
            } else if i < 15 && (1..15).contains(&k) {
                Some(1.0)
            } else {
                None
            }
        });
        let field = DrainageField::build(&g, Outlets::default());
        let middle = g.make_ref(Column::new(10, 7), 0);
        let next = field.downstream(&g, middle).expect("drains");
        assert_eq!(next.column.k, 7, "left the centreline: {next:?}");
        assert!(next.column.i < 10);
    }

    #[test]
    fn repair_after_a_breach_lowers_the_pit() {
        let mut g = pit();
        let outlets = Outlets::default();
        let mut field = DrainageField::build(&g, outlets.clone());

        // Cut a notch from the pit to the void along k = 7.
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| g.chunk_at_slot(0).column_spans(local).to_vec())
            .collect();
        let mut remap = SpanRemap::default();
        for i in 1..6 {
            let column = Column::new(i, 7);
            let old = columns[column.local_index()][0];
            columns[column.local_index()][0] = open(0.5);
            remap.columns.push(column);
            remap.entries.push(crate::water::geometry::RemapEntry {
                column,
                old_ordinal: 0,
                old,
                old_owner: Default::default(),
                new_ordinal: Some(0),
            });
        }
        g.replace_chunk(coord, SpanChunk::from_columns(&columns, 2));
        remap.chunks.push(coord);
        field.repair(&g, &remap);
        field.resolve_pending(&g);

        let fresh = DrainageField::build(&g, outlets);
        for span in g.all_refs() {
            assert_eq!(field.fill(&g, span), fresh.fill(&g, span), "{span:?}");
        }
        assert_eq!(field.fill(&g, g.make_ref(Column::new(7, 7), 0)), 1.0);
    }
}
