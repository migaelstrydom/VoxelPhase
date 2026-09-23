use std::time::Duration;

use crate::terrain::UpdateTimings;

/// One leaf of a blast's cost, in the order the frame pays them.
///
/// The leaves partition `UpdateTimings::total()`: the parts `UpdateTimings`
/// does not split further are kept as their own "other" leaf, so the shares a
/// report prints always add to the whole.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainStage {
    Detonate,
    GridAlloc,
    Sample,
    MarchingCubes,
    AmbientOcclusion,
    OctreeInsert,
    NeighbourRefs,
    Collect,
    /// Remesh time outside the timed build phases: the render cache entry,
    /// the mesh swap and the vacant-chunk prune.
    RemeshOther,
    Qualify,
    Unlink,
    Link,
    Concat,
}

impl TerrainStage {
    pub const ALL: [TerrainStage; 13] = [
        TerrainStage::Detonate,
        TerrainStage::GridAlloc,
        TerrainStage::Sample,
        TerrainStage::MarchingCubes,
        TerrainStage::AmbientOcclusion,
        TerrainStage::OctreeInsert,
        TerrainStage::NeighbourRefs,
        TerrainStage::Collect,
        TerrainStage::RemeshOther,
        TerrainStage::Qualify,
        TerrainStage::Unlink,
        TerrainStage::Link,
        TerrainStage::Concat,
    ];

    pub fn label(self) -> &'static str {
        match self {
            TerrainStage::Detonate => "detonate",
            TerrainStage::GridAlloc => "remesh/grid alloc",
            TerrainStage::Sample => "remesh/sample",
            TerrainStage::MarchingCubes => "remesh/marching cubes",
            TerrainStage::AmbientOcclusion => "remesh/ambient occl.",
            TerrainStage::OctreeInsert => "remesh/octree insert",
            TerrainStage::NeighbourRefs => "remesh/neighbour refs",
            TerrainStage::Collect => "remesh/collect",
            TerrainStage::RemeshOther => "remesh/other",
            TerrainStage::Qualify => "adjacency/qualify",
            TerrainStage::Unlink => "adjacency/unlink",
            TerrainStage::Link => "adjacency/link",
            TerrainStage::Concat => "concat",
        }
    }

    /// This stage's share of one update.
    pub fn of(self, t: &UpdateTimings) -> Duration {
        match self {
            TerrainStage::Detonate => t.detonate,
            TerrainStage::GridAlloc => t.build.grid_alloc,
            TerrainStage::Sample => t.build.sample,
            TerrainStage::MarchingCubes => t.build.marching_cubes,
            TerrainStage::AmbientOcclusion => t.build.ambient_occlusion,
            TerrainStage::OctreeInsert => t.build.insert,
            TerrainStage::NeighbourRefs => t.build.neighbor_refs,
            TerrainStage::Collect => t.collect,
            TerrainStage::RemeshOther => t.remesh.saturating_sub(t.build.total() + t.collect),
            TerrainStage::Qualify => t.adjacency_split.qualify,
            TerrainStage::Unlink => t.adjacency_split.unlink,
            TerrainStage::Link => t.adjacency_split.link,
            TerrainStage::Concat => t.concat,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::AdjacencyTimings;

    #[test]
    fn the_leaves_add_up_to_the_total() {
        let ms = Duration::from_millis;
        let mut t = UpdateTimings {
            detonate: ms(1),
            remesh: ms(20),
            collect: ms(2),
            concat: ms(3),
            ..Default::default()
        };
        t.build.sample = ms(4);
        t.build.ambient_occlusion = ms(5);
        t.adjacency_split = AdjacencyTimings {
            qualify: ms(1),
            unlink: ms(2),
            link: ms(3),
        };
        t.adjacency = t.adjacency_split.total();

        let sum: Duration = TerrainStage::ALL.iter().map(|s| s.of(&t)).sum();
        assert_eq!(sum, t.total());
    }
}
