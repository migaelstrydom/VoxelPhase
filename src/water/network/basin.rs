//! A basin: still water with one level over a region of spans.

use rustc_hash::{FxHashMap, FxHashSet};

use crate::water::geometry::{Column, COLUMN_SIZE};
use crate::water::ids::{LinkId, StoreId};

use super::depression::{CrestCell, CrestKind, Flood, MergeSaddle, RegionSpan};
use super::hypsometry::Hypsometry;

/// A hole blown through a basin's floor into whatever lies below: the
/// columns it covers and the floor it was blown through.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoleColumn {
    pub column: Column,
    /// The old floor at the hole: the rim water pours over into it.
    pub lip: f32,
}

/// Plan geometry of a hole, for the orifice law.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoleGeometry {
    pub area: f64,
    pub perimeter: f64,
    pub lip: f32,
}

/// One group of crest cells that sends its water to one place: a notch, a
/// ridge into a neighbouring pit, a hole in the floor.
#[derive(Debug, Clone)]
pub struct Outflow {
    pub kind: CrestKind,
    pub cells: Vec<CrestCell>,
    /// The lowest saddle among the cells.
    pub lip: f32,
    /// Set when the cells lead through a hole in the floor: an orifice.
    pub hole: Option<HoleGeometry>,
    /// The link carrying the water, once the basin has risen to the lip.
    pub link: Option<LinkId>,
    /// The store that link leads to.
    pub target: Option<StoreId>,
}

/// Still water over a region: a pond, a lake, a puddle on a drained bed.
#[derive(Debug, Clone)]
pub struct Basin {
    /// Water held, m³.
    pub volume: f64,
    /// Every span the basin claims, wet or dry, with pothole floors raised.
    pub region: Vec<RegionSpan>,
    pub hypsometry: Hypsometry,
    /// The steps down out of the region, lowest first.
    pub crests: Vec<CrestCell>,
    /// The crests grouped by where their water goes, lowest lip first.
    pub outflows: Vec<Outflow>,
    /// Holes blown through the floor, kept across re-floods.
    pub holes: Vec<HoleColumn>,
    /// Ridges under the water where the basin splits as it drains.
    pub merge_saddles: Vec<MergeSaddle>,
    /// The level the region was flooded to. A basin rising near it must be
    /// re-flooded to find its higher banks.
    pub cap: f32,
    /// Loses water to the ground and air (§7.7).
    pub minor: bool,
    /// Bumped whenever the region changes, so its mesh knows to rebuild.
    pub region_version: u32,
}

impl Basin {
    /// A basin over a flood's region, holding `volume`.
    pub fn from_flood(flood: Flood, volume: f64) -> Self {
        Self::with_holes(flood, volume, Vec::new())
    }

    /// A basin over a flood's region, holding `volume`, with holes blown
    /// through its floor at `holes`.
    pub fn with_holes(flood: Flood, volume: f64, holes: Vec<HoleColumn>) -> Self {
        let hypsometry = Hypsometry::new(
            flood.region.iter().map(|r| r.shape),
            COLUMN_SIZE * COLUMN_SIZE,
            flood.dead.clone(),
        );
        let mut crests = flood.crests;
        crests.sort_by(|a, b| a.saddle.total_cmp(&b.saddle));
        let outflows = group_outflows(&crests, &holes);
        Self {
            volume,
            region: flood.region,
            hypsometry,
            crests,
            outflows,
            holes,
            merge_saddles: flood.merge_saddles,
            cap: flood.cap,
            minor: false,
            region_version: 0,
        }
    }

    /// Replace the region with a new flood's, keeping the volume, the holes
    /// and the loss gate. The outflows are regrouped with no links: the
    /// caller removes the old ones.
    pub fn reregion(&mut self, flood: Flood) {
        let volume = self.volume;
        let version = self.region_version.wrapping_add(1);
        let minor = self.minor;
        let holes = std::mem::take(&mut self.holes);
        *self = Self::with_holes(flood, volume, holes);
        self.region_version = version;
        self.minor = minor;
    }

    /// Record a hole blown through the floor; takes effect at the next
    /// re-flood.
    pub fn add_hole(&mut self, hole: HoleColumn) {
        if !self.holes.iter().any(|h| h.column == hole.column) {
            self.holes.push(hole);
        }
    }

    /// The water surface.
    pub fn level(&self) -> f32 {
        self.hypsometry.level(self.volume)
    }

    /// The lowest outlet saddle: above it, water leaves the basin.
    pub fn spill(&self) -> Option<f32> {
        self.crests
            .iter()
            .find(|c| c.kind == CrestKind::Outlet)
            .map(|c| c.saddle)
    }

    /// The lowest floor in the region.
    pub fn deepest(&self) -> f32 {
        self.hypsometry.bottom()
    }

    /// Columns of the region, each once, row-major.
    pub fn columns(&self) -> Vec<Column> {
        let mut columns: Vec<Column> = self.region.iter().map(|r| r.span.column).collect();
        columns.sort_unstable();
        columns.dedup();
        columns
    }

    /// Links carrying water out of the basin.
    pub fn links(&self) -> impl Iterator<Item = LinkId> + '_ {
        self.outflows.iter().filter_map(|o| o.link)
    }
}

/// Group crest cells into outflows: cells of one kind whose inside columns
/// touch, so one notch is one outflow; and every cell leading into a hole is
/// one orifice, however it lies.
fn group_outflows(crests: &[CrestCell], holes: &[HoleColumn]) -> Vec<Outflow> {
    let hole_columns: FxHashSet<Column> = holes.iter().map(|h| h.column).collect();
    let into_hole = |c: &CrestCell| hole_columns.contains(&c.outside.column);

    let mut parent: Vec<usize> = (0..crests.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut by_column: FxHashMap<Column, Vec<usize>> = FxHashMap::default();
    for (i, c) in crests.iter().enumerate() {
        by_column.entry(c.inside.column).or_default().push(i);
    }
    for (i, c) in crests.iter().enumerate() {
        for dk in -1..=1 {
            for di in -1..=1 {
                let Some(near) = by_column.get(&c.inside.column.offset(di, dk)) else {
                    continue;
                };
                for &j in near {
                    let other = &crests[j];
                    let same = (into_hole(c) && into_hole(other))
                        || (!into_hole(c) && !into_hole(other) && c.kind == other.kind);
                    if same {
                        let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                        parent[a.max(b)] = a.min(b);
                    }
                }
            }
        }
    }
    // All hole cells are one orifice, even where they do not touch.
    let first_hole = crests.iter().position(into_hole);
    if let Some(first) = first_hole {
        for (i, c) in crests.iter().enumerate() {
            if into_hole(c) {
                let (a, b) = (root(&mut parent, i), root(&mut parent, first));
                parent[a.max(b)] = a.min(b);
            }
        }
    }

    let mut groups: FxHashMap<usize, Vec<CrestCell>> = FxHashMap::default();
    for (i, c) in crests.iter().enumerate() {
        let r = root(&mut parent, i);
        groups.entry(r).or_default().push(*c);
    }
    let mut roots: Vec<usize> = groups.keys().copied().collect();
    roots.sort_unstable();
    let mut outflows: Vec<Outflow> = roots
        .into_iter()
        .map(|r| {
            let cells = groups.remove(&r).expect("root from this map");
            let lip = cells.iter().map(|c| c.saddle).fold(f32::INFINITY, f32::min);
            let hole = into_hole(&cells[0]).then(|| hole_geometry(holes));
            Outflow {
                kind: cells[0].kind,
                cells,
                lip: hole.map_or(lip, |h| h.lip.min(lip)),
                hole,
                link: None,
                target: None,
            }
        })
        .collect();
    outflows.sort_by(|a, b| a.lip.total_cmp(&b.lip));
    outflows
}

/// The plan area, rim length and lip of a set of hole columns.
fn hole_geometry(holes: &[HoleColumn]) -> HoleGeometry {
    let columns: FxHashSet<Column> = holes.iter().map(|h| h.column).collect();
    let mut edges = 0usize;
    for h in holes {
        for (di, dk) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            if !columns.contains(&h.column.offset(di, dk)) {
                edges += 1;
            }
        }
    }
    HoleGeometry {
        area: holes.len() as f64 * (COLUMN_SIZE * COLUMN_SIZE) as f64,
        perimeter: edges as f64 * COLUMN_SIZE as f64,
        lip: holes.iter().map(|h| h.lip).fold(f32::INFINITY, f32::min),
    }
}
