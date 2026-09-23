use std::path::Path;

use nalgebra::Point3;

use crate::level::{load_level, Level};
use crate::level_check::build_terrain;
use crate::terrain::TerrainWorld;

/// Real level terrain to run a scenario on, and the spot to run it at.
///
/// The collision cost of a body depends on how many triangles its queries
/// return and how the terrain answers them, and the cost of a blast depends on
/// the material and chunk layout it lands in, so the benches measure against
/// the same `TerrainWorld` the game runs rather than a stand-in.
pub struct Ground {
    /// The level the terrain was built from, kept so a bench that damages its
    /// terrain can start every run from an untouched copy.
    level: Level,
    /// The level's terrain, built headlessly. Never modified.
    terrain: TerrainWorld,
    /// Horizontal position (x, z) the scenario is centred on.
    site: (f32, f32),
    /// Level name, for the report.
    label: String,
}

impl Ground {
    pub fn load(level_path: &Path, site: (f32, f32)) -> Result<Self, String> {
        let level = load_level(level_path).map_err(|e| e.to_string())?;
        let terrain = build_terrain(&level);
        let label = level_path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default();
        Ok(Self {
            level,
            terrain,
            site,
            label,
        })
    }

    pub fn terrain(&self) -> &TerrainWorld {
        &self.terrain
    }

    /// A newly built copy of the level's terrain, for a run that modifies it.
    pub fn fresh_terrain(&self) -> TerrainWorld {
        build_terrain(&self.level)
    }

    pub fn label(&self) -> String {
        format!("{} @ ({}, {})", self.label, self.site.0, self.site.1)
    }

    /// The level name alone, for a bench that is not centred on one site.
    pub fn level_name(&self) -> &str {
        &self.label
    }

    /// The point on the terrain surface `offset` metres from the site.
    pub fn surface_point(&self, offset_x: f32, offset_z: f32) -> Point3<f32> {
        let x = self.site.0 + offset_x;
        let z = self.site.1 + offset_z;
        self.surface_at(x, z).unwrap_or_else(|| {
            let y = self.terrain.approx_surface_height_at(x, z).unwrap_or(0.0);
            Point3::new(x, y, z)
        })
    }

    /// The top of the meshed terrain surface at world column (x, z), if the
    /// column has one.
    pub fn surface_at(&self, x: f32, z: f32) -> Option<Point3<f32>> {
        self.terrain
            .mesh_surface_height_at(x, z)
            .map(|y| Point3::new(x, y, z))
    }
}
