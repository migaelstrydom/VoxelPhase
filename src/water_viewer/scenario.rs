//! What a water scenario is: terrain, water, and a script of things to do to it.

use nalgebra::Point3;

use crate::level::loader::parse_level;
use crate::level::Level;
use crate::level_check::build_terrain;
use crate::terrain::TerrainWorld;

/// A named point whose water level the report follows.
#[derive(Debug, Clone, Copy)]
pub struct Probe {
    pub name: &'static str,
    /// World position. `y` picks the span on multi-layer terrain: the probe
    /// reads the water whose column range contains it.
    pub at: Point3<f32>,
}

/// Something the script does to the world at a moment in time.
#[derive(Debug, Clone, Copy)]
pub enum Action {
    /// A charge that cuts a crater of exactly this radius.
    Blast { centre: Point3<f32>, radius: f32 },
}

/// One scripted action and when it happens, in simulated seconds.
#[derive(Debug, Clone, Copy)]
pub struct Beat {
    pub at: f32,
    pub action: Action,
}

/// One named thing to watch water do.
pub struct Scenario {
    pub name: &'static str,
    /// One line saying what this scenario is for. Printed by `--list`.
    pub description: &'static str,
    /// The terrain and water, as level RON. Scenarios keep their levels in
    /// code so a scenario is one self-contained definition.
    pub level: &'static str,
    /// Simulated seconds the scenario runs for.
    pub duration: f32,
    pub beats: Vec<Beat>,
    pub probes: Vec<Probe>,
}

impl Scenario {
    /// The scenario's level, parsed and placed.
    pub fn level(&self) -> Result<Level, String> {
        parse_level(self.level).map_err(|e| format!("scenario {}: {e}", self.name))
    }

    /// The scenario's terrain, built headlessly.
    pub fn terrain(&self) -> Result<(Level, TerrainWorld), String> {
        let level = self.level()?;
        let terrain = build_terrain(&level);
        Ok((level, terrain))
    }
}
