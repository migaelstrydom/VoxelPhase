//! River surfaces kept apart by id, so that only those whose version moved
//! are built again, and joined into the one mesh the renderer uploads.
//!
//! Under a budget, at most that many surfaces that already have a mesh are
//! built again in one sync, stalest first: water that keeps moving
//! everywhere at once is drawn a few frames behind in places rather than
//! costing the frame. A surface with no mesh yet is always built.

use std::collections::BTreeMap;

use crate::water::ids::StoreId;

use super::basin_mesher::WaterScene;
use super::reach_mesher::RiverMesh;

/// Most surfaces built again in one sync, by default.
const BUILDS_PER_SYNC: usize = 8;

/// One surface as built.
#[derive(Debug)]
struct Part {
    /// The version it was built at.
    version: u32,
    /// The sync it was built in.
    built: u64,
    mesh: RiverMesh,
}

/// Every river surface, apart and joined.
#[derive(Debug)]
pub struct RiverParts {
    parts: BTreeMap<StoreId, Part>,
    /// Every surface joined, in id order.
    joined: RiverMesh,
    /// Syncs so far.
    syncs: u64,
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

    /// Build again the stalest surfaces whose version moved, drop those
    /// gone, and join them all. Returns whether anything changed.
    pub fn sync(&mut self, water: &dyn WaterScene) -> bool {
        self.syncs += 1;
        let wanted: BTreeMap<StoreId, u32> = water.river_key().into_iter().collect();
        let before = self.parts.len();
        self.parts.retain(|id, _| wanted.contains_key(id));
        let mut changed = self.parts.len() != before;

        // Missing surfaces first, then those built longest ago.
        let mut stale: Vec<(u64, StoreId, u32)> = wanted
            .iter()
            .filter_map(|(&id, &version)| match self.parts.get(&id) {
                None => Some((0, id, version)),
                Some(part) if part.version != version => Some((part.built, id, version)),
                Some(_) => None,
            })
            .collect();
        stale.sort_unstable();
        let missing = stale.iter().take_while(|s| s.0 == 0).count();
        let allowed = self.budget.map_or(stale.len(), |b| missing + b);
        for &(_, id, version) in stale.iter().take(allowed) {
            let part = Part {
                version,
                built: self.syncs,
                mesh: water.build_river(id),
            };
            self.parts.insert(id, part);
            changed = true;
        }
        if changed {
            self.joined.vertices.clear();
            self.joined.indices.clear();
            self.joined.draws.clear();
            for part in self.parts.values() {
                self.joined.append(&part.mesh);
            }
        }
        changed
    }

    /// Every surface joined.
    pub fn mesh(&self) -> &RiverMesh {
        &self.joined
    }
}
