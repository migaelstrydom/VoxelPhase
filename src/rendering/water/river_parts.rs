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
    /// The joined mesh's index buffer as uploaded: every surface's
    /// triangles, then every falling sheet's.
    index_buffer: Vec<u32>,
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
            index_buffer: Vec::new(),
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

    /// Drop every surface, keeping the budget.
    pub fn forget(&mut self) {
        *self = Self {
            budget: self.budget,
            ..Self::default()
        };
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
        self.joined.sheet_indices.clear();
        self.joined.draws.clear();
        for part in self.parts.values_mut() {
            part.joined_at = self.joined.vertices.len();
            self.joined.append(&part.mesh);
        }
        self.index_buffer.clear();
        self.index_buffer.extend_from_slice(&self.joined.indices);
        self.index_buffer
            .extend_from_slice(&self.joined.sheet_indices);
    }

    /// The index buffer to upload with the joined mesh's vertices: its
    /// surfaces' indices, then its sheets', which start at [`Self::sheet_base`].
    pub fn index_buffer(&self) -> &[u32] {
        &self.index_buffer
    }

    /// Where the sheets' indices start in [`Self::index_buffer`].
    pub fn sheet_base(&self) -> u32 {
        self.joined.indices.len() as u32
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

#[cfg(all(test, feature = "water_scenarios"))]
mod in_a_level {
    use std::path::Path;

    use nalgebra::{Point3, Vector3};
    use smallvec::{smallvec, SmallVec};

    use super::*;
    use crate::level::load_level;
    use crate::level_check::build_terrain;
    use crate::rendering::water::field_sources::VertexSource;
    use crate::terrain::BlastConfig;
    use crate::water::geometry::{Column, SpanIndex, CHUNK_COLUMNS, COLUMN_SIZE};
    use crate::water::WaterWorld;

    /// Farthest a surface read again may stand from one built afresh, m:
    /// the versions' tolerance, below which a surface is not read again.
    const MATCH: f32 = 2.0e-3;

    /// Farthest a curtain's vertex read again may lie from one built afresh,
    /// m. Both are drawn along the same fall's arc from the surface they hang
    /// from, which may stand the surface's tolerance apart.
    const THROWN_MATCH: f32 = 1.0e-2;

    /// After a dam on water_park is blasted and its water runs, every
    /// surface read again in place matches the same surface built afresh.
    #[test]
    fn refreshed_surfaces_match_fresh_builds() {
        let dam = [Point3::new(-52.2, 2.0, 49.8)];
        assert_refreshes_match("levels/water_park.level.ron", &dam, 180, Some(0));
    }

    /// The same, over a ditch blasted from thin_ice's lake to the map's
    /// edge, where the water falls off the map.
    #[test]
    fn refreshed_sheets_off_the_map_match_fresh_builds() {
        let ditch: Vec<_> = (0..10)
            .map(|i| Point3::new(42.0, 5.0, 82.0 + 1.5 * i as f32))
            .collect();
        let (parts, water) = assert_refreshes_match("levels/thin_ice.level.ron", &ditch, 600, None);
        let off_the_map: Vec<_> = (water.field().falls().iter())
            .filter(|fall| fall.into.is_none())
            .map(|fall| (fall.from, fall.direction))
            .collect();
        assert!(!off_the_map.is_empty(), "no water falls off the map");
        let drawn = (parts.parts.values())
            .flat_map(|p| &p.mesh.sources.curtains)
            .filter(|curtain| off_the_map.contains(&curtain.fall))
            .count();
        assert_eq!(
            drawn,
            off_the_map.len(),
            "a fall off the map has no curtain"
        );
    }

    /// Blast `blasts` into `level` at frame 30 and run it for `frames`,
    /// building at most `budget` surfaces afresh a sync beyond those that
    /// cannot be read; then every surface read again matches the same
    /// surface built afresh.
    fn assert_refreshes_match(
        level: &str,
        blasts: &[Point3<f32>],
        frames: usize,
        budget: Option<usize>,
    ) -> (RiverParts, WaterWorld) {
        let level = load_level(Path::new(level)).unwrap();
        let mut terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let mut parts = RiverParts::default();
        parts.set_budget(budget);
        let mut refreshed = 0;
        for frame in 0..frames {
            if frame == 30 {
                for &blast in blasts {
                    terrain.detonate(blast, &BlastConfig::default());
                }
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
                assert!(
                    (a.xz - b.xz).norm() <= THROWN_MATCH,
                    "{id:?}: {} vs {}",
                    a.xz,
                    b.xz
                );
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
        let joined = parts.mesh();
        let sheets: usize = (parts.parts.values())
            .map(|p| p.mesh.sheet_indices.len())
            .sum();
        assert_eq!(joined.sheet_indices.len(), sheets, "each sheet joined once");
        assert!(crate::rendering::water::audit::indexed_within(joined));
        (parts, water)
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

    /// After a blast, two cells of a surface that each count the other in
    /// the group at a corner they share lay that corner at the same height,
    /// to the last bit: two corners a hair apart open a crack between the
    /// quads. W1's breach and water_park's undercroft have water in a hole
    /// under a roof beside open water, whose columns hold dry ground over
    /// the hole's water.
    #[test]
    fn a_corner_shared_by_cells_of_one_surface_is_laid_once() {
        for (level, at, frames) in [
            ("w1", Point3::new(12.2, 8.0, 20.0), 600),
            ("water_park", Point3::new(-64.0, 6.0, -6.0), 1800),
        ] {
            let (apart, worst) = corners_laid_apart(level, at, frames);
            assert_eq!(apart, 0, "{level}: corners laid up to {worst:e} m apart");
        }
    }

    /// Run a level's water through a blast at `at` for `frames`, as the game
    /// draws it, and count the corners two cells of one group laid apart in
    /// the surfaces up to date, and the worst.
    fn corners_laid_apart(level: &str, at: Point3<f32>, frames: usize) -> (usize, f32) {
        let path = match level {
            "w1" => "levels/rooms/w1_flood_the_hollow.level.ron".to_string(),
            level => format!("levels/{level}.level.ron"),
        };
        let level = load_level(Path::new(&path)).unwrap();
        let mut terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        let mut parts = RiverParts::default();
        let (mut apart, mut worst) = (0, 0.0f32);
        for frame in 0..frames {
            if frame == 30 {
                terrain.detonate(at, &BlastConfig::default());
            }
            terrain.update();
            water.on_terrain_update(&terrain);
            water.step(1.0 / 60.0);
            parts.sync(&water);
            // A surface waiting to be built again keeps the shapes it was
            // built with while its heights move on.
            let versions: BTreeMap<SurfaceId, u32> = water.river_key().into_iter().collect();
            let current = (parts.parts.iter()).filter(|(id, part)| part.version == versions[*id]);
            for (_, part) in current {
                // Each corner by where it lies: the height it was laid at,
                // the cell that laid it, and the cells of its group.
                let mut laid: BTreeMap<(u32, u32), Vec<(f32, SpanIndex, Vec<SpanIndex>)>> =
                    BTreeMap::new();
                for (v, source) in part.mesh.vertices.iter().zip(&part.mesh.sources.vertices) {
                    if let VertexSource::Corner { cells, shape } = source {
                        let group = (cells.iter().enumerate())
                            .filter(|&(i, _)| shape.joins(i))
                            .map(|(_, &c)| c)
                            .collect();
                        laid.entry((v.xz.x.to_bits(), v.xz.y.to_bits()))
                            .or_default()
                            .push((v.surface, cells[0], group));
                    }
                }
                for corners in laid.values() {
                    for (i, (a, a_cell, a_group)) in corners.iter().enumerate() {
                        for (b, b_cell, b_group) in &corners[i + 1..] {
                            let one = a_group.contains(b_cell) && b_group.contains(a_cell);
                            if one && a != b {
                                apart += 1;
                                worst = worst.max((a - b).abs());
                            }
                        }
                    }
                }
            }
        }
        (apart, worst)
    }

    /// Floating Island's brook pours off its rim into the void, and no
    /// surface's skirt is laid out over the void beside its lip: no ground
    /// stands there to hide one, and it stood as a shelf of flat water over
    /// the fall.
    #[test]
    fn no_skirt_is_laid_over_the_void() {
        let level = load_level(Path::new("levels/floating_island.level.ron")).unwrap();
        let mut terrain = build_terrain(&level);
        let mut water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
        for _ in 0..60 {
            terrain.update();
            water.on_terrain_update(&terrain);
            water.step(1.0 / 60.0);
        }
        let mut parts = RiverParts::default();
        parts.set_budget(None);
        parts.sync(&water);
        let field = water.field();
        assert!(
            field.falls().iter().any(|fall| fall.into_void),
            "nothing pours into the void"
        );
        let graph = field.graph();
        let mut skirts = 0;
        for part in parts.parts.values() {
            let mesh = &part.mesh;
            let skirt = |v: u32| {
                matches!(
                    mesh.sources.vertices[v as usize],
                    VertexSource::Skirt { .. }
                )
            };
            for triangle in mesh.indices.chunks(3) {
                if !triangle.iter().all(|&v| skirt(v)) {
                    continue;
                }
                skirts += 1;
                let centre = triangle
                    .iter()
                    .map(|&v| {
                        let v = &mesh.vertices[v as usize];
                        Vector3::new(v.xz.x, v.surface, v.xz.y)
                    })
                    .sum::<Vector3<f32>>()
                    / 3.0;
                let column = Column::containing(centre.x, centre.z);
                assert!(
                    !graph.spans(column).is_empty() && centre.y >= graph.underside(column),
                    "a skirt over the void at {centre:?}"
                );
            }
        }
        assert!(skirts > 0, "no skirts laid at all");
    }

    /// Every cell a level's surfaces read again lies in a chunk whose
    /// layout the surface notes, so a blast that lays that chunk out again
    /// has the surface built afresh instead of read from a cell gone.
    #[test]
    fn every_cell_a_surface_reads_again_is_in_a_layout_it_notes() {
        for level in [
            "levels/water_park.level.ron",
            "levels/rooms/c11_washed_off.level.ron",
            "levels/rooms/o2_raft.level.ron",
        ] {
            let level = load_level(Path::new(level)).unwrap();
            let terrain = build_terrain(&level);
            let water = WaterWorld::from_config(level.water.as_ref().unwrap(), &terrain);
            let mut parts = RiverParts::default();
            parts.set_budget(None);
            parts.sync(&water);
            for (id, part) in &parts.parts {
                let sources = &part.mesh.sources;
                let noted = |c: &SpanIndex| sources.layouts.iter().any(|&(s, _)| s == c.slot);
                for source in &sources.vertices {
                    let read: SmallVec<[SpanIndex; 4]> = match source {
                        VertexSource::Corner { cells, .. } => cells.clone(),
                        VertexSource::Skirt { dry, .. } => dry.clone(),
                        VertexSource::Jet { landing, .. }
                        | VertexSource::JetPlunge { landing, .. } => smallvec![*landing],
                        _ => SmallVec::new(),
                    };
                    let unnoted = read.iter().find(|c| !noted(c));
                    assert!(
                        unnoted.is_none(),
                        "{}: {id:?} reads {unnoted:?} through {source:?}",
                        level.name
                    );
                }
                for curtain in &sources.curtains {
                    let ends = curtain.beside.iter().flatten().map(|&(from, _)| from);
                    let unnoted = std::iter::once(curtain.fall.0)
                        .chain(ends)
                        .find(|c| !noted(c));
                    assert!(
                        unnoted.is_none(),
                        "{}: {id:?}'s curtain reads {unnoted:?}",
                        level.name
                    );
                }
            }
        }
    }
}
