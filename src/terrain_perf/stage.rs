use std::time::Duration;

use crate::terrain::{ChunkBuildTimings, UpdateTimings};

/// One leaf of a blast's wall-clock cost, in the order the frame pays them.
///
/// The leaves partition `UpdateTimings::total()`, so the shares a report
/// prints add to the whole.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainStage {
    Detonate,
    /// Every dirty chunk's replacement mesh, built in parallel.
    Build,
    Commit,
    Unlink,
    Link,
    Concat,
}

impl TerrainStage {
    pub const ALL: [TerrainStage; 6] = [
        TerrainStage::Detonate,
        TerrainStage::Build,
        TerrainStage::Commit,
        TerrainStage::Unlink,
        TerrainStage::Link,
        TerrainStage::Concat,
    ];

    pub fn label(self) -> &'static str {
        match self {
            TerrainStage::Detonate => "detonate",
            TerrainStage::Build => "build (parallel)",
            TerrainStage::Commit => "commit",
            TerrainStage::Unlink => "adjacency/unlink",
            TerrainStage::Link => "adjacency/link",
            TerrainStage::Concat => "concat",
        }
    }

    /// This stage's share of one update.
    pub fn of(self, t: &UpdateTimings) -> Duration {
        match self {
            TerrainStage::Detonate => t.detonate,
            TerrainStage::Build => t.build,
            TerrainStage::Commit => t.commit,
            TerrainStage::Unlink => t.adjacency_split.unlink,
            TerrainStage::Link => t.adjacency_split.link,
            TerrainStage::Concat => t.concat,
        }
    }
}

/// One phase of building a chunk, as CPU time summed over chunks.
///
/// These partition `ChunkBuildTimings::total()`. They say where the build's
/// work goes, not how long the frame waited for it: chunks build on several
/// threads, so the sum exceeds the `Build` stage's wall clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildPhase {
    GridAlloc,
    Sample,
    MarchingCubes,
    AmbientOcclusion,
    OctreeInsert,
    NeighbourRefs,
    Collect,
    /// Linking the chunk's own triangles; seams are linked at commit.
    Adjacency,
    RenderData,
}

impl BuildPhase {
    pub const ALL: [BuildPhase; 9] = [
        BuildPhase::GridAlloc,
        BuildPhase::Sample,
        BuildPhase::MarchingCubes,
        BuildPhase::AmbientOcclusion,
        BuildPhase::OctreeInsert,
        BuildPhase::NeighbourRefs,
        BuildPhase::Collect,
        BuildPhase::Adjacency,
        BuildPhase::RenderData,
    ];

    pub fn label(self) -> &'static str {
        match self {
            BuildPhase::GridAlloc => "grid alloc",
            BuildPhase::Sample => "sample",
            BuildPhase::MarchingCubes => "marching cubes",
            BuildPhase::AmbientOcclusion => "ambient occlusion",
            BuildPhase::OctreeInsert => "octree insert",
            BuildPhase::NeighbourRefs => "neighbour refs",
            BuildPhase::Collect => "collect triangles",
            BuildPhase::Adjacency => "chunk adjacency",
            BuildPhase::RenderData => "render data",
        }
    }

    pub fn of(self, t: &ChunkBuildTimings) -> Duration {
        match self {
            BuildPhase::GridAlloc => t.mesh.grid_alloc,
            BuildPhase::Sample => t.mesh.sample,
            BuildPhase::MarchingCubes => t.mesh.marching_cubes,
            BuildPhase::AmbientOcclusion => t.mesh.ambient_occlusion,
            BuildPhase::OctreeInsert => t.mesh.insert,
            BuildPhase::NeighbourRefs => t.mesh.neighbor_refs,
            BuildPhase::Collect => t.collect,
            BuildPhase::Adjacency => t.adjacency,
            BuildPhase::RenderData => t.render_data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::AdjacencyTimings;

    #[test]
    fn the_stages_add_up_to_the_total() {
        let ms = Duration::from_millis;
        let mut t = UpdateTimings {
            detonate: ms(1),
            build: ms(20),
            commit: ms(2),
            concat: ms(3),
            adjacency_split: AdjacencyTimings {
                unlink: ms(2),
                link: ms(3),
            },
            ..Default::default()
        };
        t.adjacency = t.adjacency_split.total();

        let sum: Duration = TerrainStage::ALL.iter().map(|s| s.of(&t)).sum();
        assert_eq!(sum, t.total());
    }

    #[test]
    fn the_build_phases_add_up_to_the_build_cpu_total() {
        let ms = Duration::from_millis;
        let mut t = ChunkBuildTimings {
            collect: ms(2),
            adjacency: ms(6),
            render_data: ms(3),
            ..Default::default()
        };
        t.mesh.sample = ms(4);
        t.mesh.ambient_occlusion = ms(5);

        let sum: Duration = BuildPhase::ALL.iter().map(|p| p.of(&t)).sum();
        assert_eq!(sum, t.total());
    }
}
