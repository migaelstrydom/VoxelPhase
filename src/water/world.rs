//! The water façade: the single ECS resource, and the one thing physics,
//! rendering and gameplay hold.
//!
//! ```text
//!   WaterWorld
//!     geometry   WaterGeometry   spans + drainage, kept current with terrain
//!     network    Network         stores and links
//!     topology   TopologyBuilder the only writer of topology
//!     solver     HydrologySolver integrates volumes at a fixed tick
//!     ledger     VolumeLedger    where every m³ came from and went
//! ```
//!
//! Each frame: catch up with terrain edits, settle topology between ticks,
//! then run the ticks the frame has accumulated.

use std::time::{Duration, Instant};

use crate::debug::DebugLog;
use crate::level::{WaterBody, WaterConfig};
use crate::terrain::TerrainWorld;

use super::geometry::{GeometryUpdate, Outlets, WaterGeometry};
use super::ids::StoreId;
use super::network::{Basin, LossLaw, Network, Store};
use super::query::WaterQuery;
use super::solver::{Balance, HydrologySolver, VolumeLedger};
use super::topology::{PoolError, Topology, TopologyBuilder};

/// Per-level dials of the hydrology.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HydrologyConfig {
    /// Scales weir and orifice discharge (§7.3).
    pub drain_gain: f32,
    pub loss: LossLaw,
}

impl HydrologyConfig {
    pub fn from_level(config: &WaterConfig) -> Self {
        Self {
            drain_gain: config.drain_gain,
            loss: LossLaw::from_mm_per_hour(config.loss_rate as f64),
        }
    }
}

/// Where one frame's water time went.
#[derive(Debug, Clone, Copy, Default)]
pub struct WaterTimings {
    /// Re-pairing spans and repairing drainage after a terrain edit.
    pub geometry: Duration,
    /// Re-flooding basins the edit touched.
    pub reregion: Duration,
    /// Between-tick topology.
    pub settle: Duration,
    /// Solver ticks.
    pub solve: Duration,
    pub ticks: u32,
}

impl WaterTimings {
    pub fn total(&self) -> Duration {
        self.geometry + self.reregion + self.settle + self.solve
    }
}

/// All of a level's water.
pub struct WaterWorld {
    geometry: WaterGeometry,
    network: Network,
    ledger: VolumeLedger,
    solver: HydrologySolver,
    topology: TopologyBuilder,
    config: HydrologyConfig,
    /// Each store's surface level after the last change, by store slot, so
    /// that queries in every physics substep do not invert a hypsometry.
    levels: Vec<Option<f32>>,
    /// This frame's timings so far: an edit's, until the step that follows.
    last_timings: WaterTimings,
    /// The most recent step's timings, edit included.
    last_step: WaterTimings,
    /// Timings of the most recent frame that had a terrain edit.
    last_edit: Option<WaterTimings>,
}

impl WaterWorld {
    /// A level's water, placed from its config over its terrain. Pools that
    /// cannot be placed are reported and skipped.
    pub fn from_config(config: &WaterConfig, terrain: &TerrainWorld) -> (Self, Vec<PoolError>) {
        Self::build(config, terrain, TopologyBuilder::default())
    }

    /// As [`Self::from_config`], logging every topology edit.
    pub fn recording(config: &WaterConfig, terrain: &TerrainWorld) -> (Self, Vec<PoolError>) {
        Self::build(config, terrain, TopologyBuilder::recording())
    }

    fn build(
        config: &WaterConfig,
        terrain: &TerrainWorld,
        topology: TopologyBuilder,
    ) -> (Self, Vec<PoolError>) {
        let mut world = Self {
            geometry: WaterGeometry::build(terrain, Outlets::default()),
            network: Network::default(),
            ledger: VolumeLedger::default(),
            solver: HydrologySolver::default(),
            topology,
            config: HydrologyConfig::from_level(config),
            levels: Vec::new(),
            last_timings: WaterTimings::default(),
            last_step: WaterTimings::default(),
            last_edit: None,
        };
        let mut errors = Vec::new();
        for body in &config.bodies {
            match body {
                WaterBody::Pool {
                    seed,
                    surface_level,
                } => {
                    let mut t = Topology {
                        network: &mut world.network,
                        ledger: &mut world.ledger,
                        geometry: &mut world.geometry,
                    };
                    if let Err(e) = world.topology.create_pool(&mut t, *seed, *surface_level) {
                        log::warn!("{e}");
                        errors.push(e);
                    }
                }
            }
        }
        world.refresh_levels();
        (world, errors)
    }

    /// Catch up with the terrain's most recent update.
    pub fn on_terrain_update(&mut self, terrain: &TerrainWorld) -> Option<GeometryUpdate> {
        let started = Instant::now();
        let update = self.geometry.update(terrain)?;
        let geometry = started.elapsed();
        let started = Instant::now();
        let mut t = Topology {
            network: &mut self.network,
            ledger: &mut self.ledger,
            geometry: &mut self.geometry,
        };
        self.topology
            .after_terrain_update(&mut t, &update.remap.columns);
        self.refresh_levels();
        self.last_timings.geometry = geometry;
        self.last_timings.reregion = started.elapsed();
        Some(update)
    }

    /// Advance by one frame: settle topology, then run the ticks due.
    pub fn step(&mut self, frame_dt: f32) {
        let edited = self.last_timings.geometry > Duration::ZERO;
        let started = Instant::now();
        let loss = self.config.loss;
        let mut t = Topology {
            network: &mut self.network,
            ledger: &mut self.ledger,
            geometry: &mut self.geometry,
        };
        self.topology.settle(&mut t, &loss);
        let settle = started.elapsed();

        let started = Instant::now();
        let ticks =
            self.solver
                .advance(&mut self.network, &mut self.ledger, &loss, frame_dt as f64);
        self.refresh_levels();
        self.last_timings.settle = settle;
        self.last_timings.solve = started.elapsed();
        self.last_timings.ticks = ticks;
        if edited {
            self.last_edit = Some(self.last_timings);
        }
        let balance = self.balance();
        debug_assert!(balance.is_balanced(), "water ledger unbalanced: {balance}");
        self.last_step = self.last_timings;
        self.last_timings = WaterTimings::default();
    }

    fn refresh_levels(&mut self) {
        self.levels = (0..self.network.store_slots())
            .map(|i| match self.network.store(StoreId(i as u32)) {
                Some(Store::Basin(b)) => Some(b.level()),
                _ => None,
            })
            .collect();
    }

    /// Re-flood a basin in place, as a terrain edit would: for measuring the
    /// worst case of §9.2.
    pub fn reregion(&mut self, id: StoreId) {
        let mut t = Topology {
            network: &mut self.network,
            ledger: &mut self.ledger,
            geometry: &mut self.geometry,
        };
        self.topology.reregion(&mut t, id);
        self.refresh_levels();
    }

    /// Point queries against the water: buoyancy, currents, the player.
    pub fn query(&self) -> WaterQuery<'_> {
        WaterQuery::new(self)
    }

    /// A store's surface level, as of the last change.
    pub fn level(&self, id: StoreId) -> Option<f32> {
        self.levels.get(id.0 as usize).copied().flatten()
    }

    /// Every basin, in id order.
    pub fn basins(&self) -> impl Iterator<Item = (StoreId, &Basin)> {
        self.network
            .stores()
            .filter_map(|(id, s)| s.as_basin().map(|b| (id, b)))
    }

    pub fn network(&self) -> &Network {
        &self.network
    }

    pub fn geometry(&self) -> &WaterGeometry {
        &self.geometry
    }

    pub fn ledger(&self) -> &VolumeLedger {
        &self.ledger
    }

    pub fn config(&self) -> &HydrologyConfig {
        &self.config
    }

    /// The ledger against what the stores hold now.
    pub fn balance(&self) -> Balance {
        self.ledger.check(self.network.held_volume())
    }

    /// Water held in every finite store, m³.
    pub fn volume(&self) -> f64 {
        self.network.held_volume()
    }

    pub fn topology_log(&self) -> &[super::topology::TopologyEdit] {
        self.topology.log()
    }

    /// Timings of the most recent frame with a terrain edit.
    pub fn last_edit_timings(&self) -> Option<WaterTimings> {
        self.last_edit
    }

    /// This frame's edit timings, before [`Self::step`] completes the frame.
    pub fn pending_timings(&self) -> WaterTimings {
        self.last_timings
    }

    /// The most recent frame's timings, edit and step.
    pub fn last_step_timings(&self) -> WaterTimings {
        self.last_step
    }

    /// Write the F3 statistics.
    pub fn debug_log(&self, log: &mut DebugLog) {
        let ledger = self.ledger;
        let balance = self.balance();
        log.add("Water/Ledger/Held", format!("{:.3} m³", balance.held));
        log.add("Water/Ledger/Error", format!("{:+.3e} m³", balance.error()));
        log.add("Water/Ledger/Emitted", format!("{:.3}", ledger.emitted));
        log.add("Water/Ledger/Sunk", format!("{:.3}", ledger.sunk));
        log.add("Water/Ledger/Lost", format!("{:.3}", ledger.lost));
        log.add("Water/Ledger/Discarded", format!("{:.3}", ledger.discarded));
        log.add(
            "Water/Stores",
            format!(
                "{} basins, {} stores",
                self.basins().count(),
                self.network.stores().count()
            ),
        );
        log.add("Water/Links", self.network.links().count().to_string());
        let stats = self.geometry.stats();
        log.add(
            "Water/Geometry/ParityRepairs",
            stats.parity_repairs.to_string(),
        );
        log.add(
            "Water/Geometry/FloorHolds",
            format!(
                "{} (+{} floor_min)",
                stats.floor_clamps, stats.floor_min_holds
            ),
        );
        if let Some(t) = self.last_edit {
            log.add(
                "Water/Time/LastEdit",
                format!(
                    "{:.3} ms (geometry {:.3}, reregion {:.3})",
                    t.total().as_secs_f64() * 1000.0,
                    t.geometry.as_secs_f64() * 1000.0,
                    t.reregion.as_secs_f64() * 1000.0
                ),
            );
        }
    }
}
