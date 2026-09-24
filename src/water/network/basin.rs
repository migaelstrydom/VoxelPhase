//! A basin: still water with one level over a region of spans.

use crate::water::geometry::{Column, COLUMN_SIZE};

use super::depression::{CrestCell, CrestKind, Flood, MergeSaddle, RegionSpan};
use super::hypsometry::Hypsometry;

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
        let hypsometry = Hypsometry::new(
            flood.region.iter().map(|r| r.shape),
            COLUMN_SIZE * COLUMN_SIZE,
            flood.dead.clone(),
        );
        let mut crests = flood.crests;
        crests.sort_by(|a, b| a.saddle.total_cmp(&b.saddle));
        Self {
            volume,
            region: flood.region,
            hypsometry,
            crests,
            merge_saddles: flood.merge_saddles,
            cap: flood.cap,
            minor: false,
            region_version: 0,
        }
    }

    /// Replace the region with a new flood's, keeping the volume.
    pub fn reregion(&mut self, flood: Flood) {
        let volume = self.volume;
        let version = self.region_version.wrapping_add(1);
        let minor = self.minor;
        *self = Self::from_flood(flood, volume);
        self.region_version = version;
        self.minor = minor;
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
}
