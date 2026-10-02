//! What a rubble scenario is: terrain, blasts, and what should come loose.

use nalgebra::Point3;

use super::driver::Run;
use crate::level::loader::parse_level;
use crate::level_check::build_terrain;
use crate::terrain::{BlastConfig, TerrainWorld};

/// One charge set off at a point.
#[derive(Debug, Clone, Copy)]
pub struct Blast {
    pub centre: Point3<f32>,
    pub charge: BlastConfig,
}

impl Blast {
    /// A charge that cuts a crater of exactly `radius`.
    pub fn sized(centre: Point3<f32>, radius: f32) -> Self {
        Self {
            centre,
            charge: BlastConfig::fixed_radius(radius),
        }
    }

    /// The game's grenade, which spends its budget against what it hits.
    pub fn grenade(centre: Point3<f32>) -> Self {
        Self {
            centre,
            charge: BlastConfig::default(),
        }
    }
}

/// One named thing to blow up.
pub struct Scenario {
    pub name: &'static str,
    /// One line saying what this scenario is for. Printed by `--list`.
    pub description: &'static str,
    /// The terrain, as level RON, so a scenario is one self-contained
    /// definition.
    pub level: &'static str,
    /// Set off one after another, in order.
    pub blasts: Vec<Blast>,
    /// What this scenario in particular should have done, beyond the
    /// invariants every run is held to.
    pub expect: fn(&Run) -> Result<(), String>,
    /// A limit of the design this scenario exists to record, if any. Such a
    /// scenario is expected to break the invariants; when it stops, the limit
    /// has gone and the scenario needs revisiting.
    pub known_gap: Option<&'static str>,
}

impl Scenario {
    /// Whether `run` did what this scenario says it should: the invariants
    /// held (or, for a known gap, broke) and its own expectation was met.
    pub fn verdict(&self, run: &Run) -> Result<(), Vec<String>> {
        let violations = run.violations();
        let mut problems = match self.known_gap {
            None => violations,
            Some(gap) if violations.is_empty() => {
                vec![format!("the known gap no longer shows: {gap}")]
            }
            Some(_) => Vec::new(),
        };
        if let Err(problem) = (self.expect)(run) {
            problems.push(problem);
        }
        if problems.is_empty() {
            Ok(())
        } else {
            Err(problems)
        }
    }
}

impl Scenario {
    /// The scenario's terrain, built headlessly.
    pub fn terrain(&self) -> Result<TerrainWorld, String> {
        let level = parse_level(self.level).map_err(|e| format!("scenario {}: {e}", self.name))?;
        Ok(build_terrain(&level))
    }
}
