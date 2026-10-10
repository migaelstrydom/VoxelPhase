//! River surfaces kept apart by id, so that only those whose version moved
//! are touched, and joined into the one mesh the renderer uploads.
//!
//! A surface whose version moved is first read again in place
//! (`WaterScene::refresh_river`), every sync, so moving water is drawn as it
//! moves. Only one whose shape changed is built again, and under a budget at
//! most that many in one sync, stalest first: a shape that changes
//! everywhere at once shows a few frames late in places rather than costing
//! the frame. A surface with no mesh yet, or whose cells were laid out again
//! so it cannot be read, is always built.

use std::collections::BTreeMap;

use crate::water::ids::SurfaceId;

use super::river_mesh::RiverMesh;
use super::scene::{Refreshed, WaterScene};

/// Most surfaces built again in one sync, by default.
const BUILDS_PER_SYNC: usize = 8;

/// One surface as built.
#[derive(Debug)]
struct Part {
    /// The version it was built at.
    version: u32,
    /// The sync it was built in.
    built: u64,
    /// Its first vertex in the joined mesh.
    joined_at: usize,
    mesh: RiverMesh,
}

/// Every river surface, apart and joined.
#[derive(Debug)]
pub struct RiverParts {
    parts: BTreeMap<SurfaceId, Part>,
    /// Every surface joined, in id order.
    joined: RiverMesh,
    /// Syncs so far.
    syncs: u64,
    /// After the last sync: surfaces still behind their version, and the
    /// most syncs any of them has gone unbuilt.
    lag: (usize, u64),
    /// Most surfaces built again in one sync; `None` for every one that
    /// moved.
    budget: Option<usize>,
}

impl Default for RiverParts {
    fn default() -> Self {
        Self {
            parts: BTreeMap::new(),
            joined: RiverMesh::default(),
            syncs: 0,
            lag: (0, 0),
            budget: Some(BUILDS_PER_SYNC),
        }
    }
}

impl RiverParts {
    /// Most surfaces built again in one sync; `None` builds every one that
    /// moved, as a tool drawing single frames needs.
    pub fn set_budget(&mut self, budget: Option<usize>) {
        self.budget = budget;
    }

    /// Read again every surface whose version moved, build again the
    /// stalest of those whose shape changed, drop those gone, and join them
    /// all. Returns whether anything changed.
    pub fn sync(&mut self, water: &dyn WaterScene) -> bool {
        self.syncs += 1;
        let wanted: BTreeMap<SurfaceId, u32> = water.river_key().into_iter().collect();
        let before = self.parts.len();
        self.parts.retain(|id, _| wanted.contains_key(id));
        let mut reshaped = self.parts.len() != before;

        let mut refreshed: Vec<SurfaceId> = Vec::new();
        // Surfaces missing or unreadable first, all built now: the cells they
        // read were laid out again, so they cannot be read until built. Then
        // those built longest ago.
        let mut stale: Vec<(u64, SurfaceId, u32)> = Vec::new();
        for (&id, &version) in &wanted {
            match self.parts.get_mut(&id) {
                None => stale.push((0, id, version)),
                Some(part) if part.version != version => {
                    match water.refresh_river(id, &mut part.mesh) {
                        Refreshed::Held => part.version = version,
                        Refreshed::Reshaped => stale.push((part.built, id, version)),
                        Refreshed::Unreadable => stale.push((0, id, version)),
                    }
                    refreshed.push(id);
                }
                Some(_) => {}
            }
        }
        stale.sort_unstable();
        let missing = stale.iter().take_while(|s| s.0 == 0).count();
        let allowed = self.budget.map_or(stale.len(), |b| missing + b);
        self.lag = stale
            .iter()
            .skip(allowed)
            .fold((0, 0), |(n, age), s| (n + 1, age.max(self.syncs - s.0)));
        for &(_, id, version) in stale.iter().take(allowed) {
            let part = Part {
                version,
                built: self.syncs,
                joined_at: 0,
                mesh: water.build_river(id),
            };
            self.parts.insert(id, part);
            reshaped = true;
        }

        if reshaped {
            self.join();
        } else {
            for id in &refreshed {
                let part = &self.parts[id];
                let vertices = &part.mesh.vertices;
                self.joined.vertices[part.joined_at..part.joined_at + vertices.len()]
                    .copy_from_slice(vertices);
            }
        }
        reshaped || !refreshed.is_empty()
    }

    /// Join every surface again, in id order.
    fn join(&mut self) {
        self.joined.vertices.clear();
        self.joined.indices.clear();
        self.joined.draws.clear();
        for part in self.parts.values_mut() {
            part.joined_at = self.joined.vertices.len();
            self.joined.append(&part.mesh);
        }
    }

    /// Surfaces the last sync left behind their version, and the most syncs
    /// any of them has gone unbuilt.
    pub fn lag(&self) -> (usize, u64) {
        self.lag
    }

    /// Every surface joined.
    pub fn mesh(&self) -> &RiverMesh {
        &self.joined
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use nalgebra::Point3;

    use super::*;
    use crate::level::load_level;
    use crate::level_check::build_terrain;
    use crate::rendering::water::field_sources::VertexSource;
    use crate::terrain::BlastConfig;
    use crate::water::geometry::{CHUNK_COLUMNS, COLUMN_SIZE};
    use crate::water::WaterWorld;

    /// Farthest a surface read again may stand from one built afresh, m:
    /// the versions' tolerance, below which a surface is not read again.
    const MATCH: f32 = 2.0e-3;

    /// After a dam on water_park is blasted and its water runs, every
    /// surface read again in place matches the same surface built afresh.
    #[test]
    #[ignore = "release: opens water_park"]
    fn refreshed_surfaces_match_fresh_builds() {
        let level = load_level(Path::new("levels/water_park.level.ron")).unwrap();
        let mut terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let mut parts = RiverParts::default();
        parts.set_budget(Some(0));
        let mut refreshed = 0;
        for frame in 0..180 {
            if frame == 30 {
                terrain.detonate(Point3::new(-52.2, 2.0, 49.8), &BlastConfig::default());
            }
            terrain.update();
            water.on_terrain_update(&terrain);
            water.step(1.0 / 60.0);
            parts.sync(&water);
            refreshed += parts
                .parts
                .values()
                .filter(|p| p.built < parts.syncs && p.version != 0)
                .count();
        }
        assert!(refreshed > 0, "nothing was read again");

        let versions: BTreeMap<SurfaceId, u32> = water.river_key().into_iter().collect();
        let mut compared = 0;
        for (id, part) in &parts.parts {
            if part.version != versions[id] {
                continue;
            }
            let fresh = water.build_river(*id);
            assert_eq!(fresh.vertices.len(), part.mesh.vertices.len(), "{id:?}");
            for (a, b) in fresh.vertices.iter().zip(&part.mesh.vertices) {
                assert_eq!(a.xz, b.xz, "{id:?}");
                assert!(
                    (a.surface - b.surface).abs() <= MATCH,
                    "{id:?}: {} vs {}",
                    a.surface,
                    b.surface
                );
                assert!(
                    (a.depth - b.depth).abs() <= MATCH,
                    "{id:?}: {} vs {}",
                    a.depth,
                    b.depth
                );
            }
            compared += 1;
        }
        assert!(compared > 0);
    }

    /// Farthest apart two surfaces may stand where they meet on a chunk's
    /// border, m: twice the versions' tolerance, as far as a border cell's
    /// drawn depth may drift before its neighbours are drawn again.
    const SEAM: f32 = 2.1e-3;

    /// Nearer than this two border vertices are taken for the same surface,
    /// m; further, they are two surfaces over one point (a curtain's top and
    /// foot, water over a tunnel's).
    const SAME_SURFACE: f32 = 0.25;

    /// After a blast on a level's water, every surface meets its
    /// neighbours on their shared borders, frame by frame.
    #[test]
    #[ignore = "release: opens water_park and thin_ice"]
    fn neighbouring_surfaces_meet() {
        for (level, at) in [
            ("water_park", Point3::new(-52.2, 2.0, 49.8)),
            ("thin_ice", Point3::new(45.0, 3.5, 60.0)),
        ] {
            let (parted, worst) = parted_after_blast(level, at);
            assert_eq!(parted, 0, "{level}: surfaces parted by up to {worst} m");
        }
    }

    /// Run a level's water through a blast at `at`, as the game draws it,
    /// and count the border corners that stand apart from the neighbouring
    /// surface's, and how far.
    fn parted_after_blast(level: &str, at: Point3<f32>) -> (usize, f32) {
        let level = load_level(Path::new(&format!("levels/{level}.level.ron"))).unwrap();
        let mut terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let mut parts = RiverParts::default();
        let extent = CHUNK_COLUMNS as f32 * COLUMN_SIZE;
        let on = |c: f32| (c / extent - (c / extent).round()).abs() < 1.0e-4;
        let (mut parted, mut worst) = (0, 0.0f32);
        for frame in 0..240 {
            if frame == 30 {
                terrain.detonate(at, &BlastConfig::default());
            }
            terrain.update();
            water.on_terrain_update(&terrain);
            water.step(1.0 / 60.0);
            parts.sync(&water);
            // Every quad corner on a chunk's border, by where it stands.
            let mut border: BTreeMap<(i64, i64), Vec<(SurfaceId, f32)>> = BTreeMap::new();
            for (id, part) in &parts.parts {
                for (i, v) in part.mesh.vertices.iter().enumerate() {
                    let corner =
                        matches!(part.mesh.sources.vertices[i], VertexSource::Corner { .. });
                    if corner && (on(v.xz.x) || on(v.xz.y)) {
                        let key = (
                            (v.xz.x * 1000.0).round() as i64,
                            (v.xz.y * 1000.0).round() as i64,
                        );
                        border.entry(key).or_default().push((*id, v.surface));
                    }
                }
            }
            for found in border.values() {
                for &(id, height) in found {
                    let gap = found
                        .iter()
                        .filter(|(other, _)| *other != id)
                        .map(|&(_, h)| (h - height).abs())
                        .min_by(f32::total_cmp);
                    if let Some(gap) = gap.filter(|&g| g > SEAM && g <= SAME_SURFACE) {
                        parted += 1;
                        worst = worst.max(gap);
                    }
                }
            }
        }
        (parted, worst)
    }
}
