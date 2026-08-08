//! One deterministic piece of real marching-cubes terrain, shared by the scenes
//! that need to look at voxel geometry rather than at primitives.
//!
//! Every other bench scene stands its subject on a flat quad, which is a
//! comfortable place for a renderer to sit and a misleading one to judge it in.
//! This module builds terrain through the *real* path — `generate_terrain` into
//! a `ChunkGrid`, a `Segment` to mesh it, `damage_sphere` for the crater — so a
//! sheet drawn from it exercises the same vertices, normals and material
//! colours the game does.
//!
//! ```text
//!   Terrain (authored features)
//!        │  generate_terrain
//!        ▼
//!    ChunkGrid ──▶ Segment::new ──▶ update() ──▶ damage_sphere() ──▶ update()
//!                                                                      │
//!                                            append_render_data ───────┘
//!                                                    │
//!                                                    ▼
//!                                      Vec<Vertex> + Vec<u32>  ──▶ SceneMesh
//! ```
//!
//! # What is in the plot, and why
//!
//! The features are chosen to be *unflattering* — each one is a case that
//! breaks a renderer, an occlusion bake or a shadow filter, rather than a
//! pretty vista:
//!
//! | Feature | What it catches |
//! |---|---|
//! | [`CREVICE`] — a 1 m slot between two 3.5 m walls | acute inside corners; must darken |
//! | [`OVERHANG`] — a lip jutting 3 m off the cliff top | a genuine downward-facing underside, which no heightfield can produce |
//! | [`RIDGE`] and [`BOULDER`] | convex forms, which must *not* darken |
//! | [`CRATER`] — carved by `damage_sphere` | destruction geometry, a different code path from generated terrain |
//! | [`CLIFF_FACE`] — a sigmoid from flat to ~74° | a continuous slope sweep in one surface |
//! | [`SPIRES`] — a 0.9 m pillar and a 0.7 m arch | the picket-row lesson: without something thin, a sheet cannot catch a filter or a bake that erases fine features |
//!
//! # Coordinates
//!
//! The segment frame is the identity, so grid-local, segment-local and world
//! coordinates are all the same thing here. The plot spans
//! `0..PLOT_EXTENT` in x and z; the low ground sits at [`GROUND_HEIGHT`] and
//! the plateau behind the cliff at [`PLATEAU_HEIGHT`].
//!
//! Feature positions are exported as constants rather than written down twice,
//! so a camera aims at where a feature *is* instead of at where it was when the
//! shot was framed.

use nalgebra::Point3;

use crate::collision::AABB;
use crate::level::{
    Extent, MaterialLayer, Terrain, TerrainFeature, VolumeFeature, VoxelMaterialId,
};
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::SceneMesh;
use crate::terrain::{generate_terrain, ChunkGrid, DurabilityConfig, Segment, SegmentFrame};

/// Edge length of one voxel, in metres.
///
/// Fine enough that the thin features are several voxels across — a 0.9 m
/// pillar is 3.6 voxels — without making the plot expensive to mesh.
pub const VOXEL_SIZE: f32 = 0.25;

/// Side length of the square plot, in metres.
pub const PLOT_EXTENT: f32 = 32.0;

/// Height of the low ground in front of the cliff.
pub const GROUND_HEIGHT: f32 = 2.0;

/// Height of the plateau behind the cliff.
pub const PLATEAU_HEIGHT: f32 = 8.0;

/// Bottom of the generated volume. Well below the ground so the bedrock layer
/// (which `damage_sphere` cannot touch) stays far from the crater.
const FLOOR_HEIGHT: f32 = -2.0;

/// z of the cliff edge line. Everything at larger z is plateau.
const CLIFF_Z: f32 = 21.0;

/// How sharply the cliff's height transition is compressed.
///
/// The face is a sigmoid, so its slope sweeps continuously from flat through a
/// maximum of `(PLATEAU_HEIGHT - GROUND_HEIGHT) * steepness / 4` and back to
/// flat — 3.6, or 74°, at this value. That single surface is the slope sweep.
const CLIFF_STEEPNESS: f32 = 2.4;

// === Where the features are ===

/// Centre of the slot between the two crevice walls, at its floor.
pub const CREVICE: Point3<f32> = Point3::new(9.8, GROUND_HEIGHT, 9.5);

/// Half the gap between the crevice walls' centre lines.
const CREVICE_HALF_GAP: f32 = 1.2;

/// Thickness of one crevice wall. Its height falls off to nothing over half
/// this distance, so a wall face rises `CREVICE_DEPTH` over 0.8 m — about 77°.
const CREVICE_THICKNESS: f32 = 1.6;

/// How far the crevice walls rise above the ground.
const CREVICE_DEPTH: f32 = 3.5;

/// Under the middle of the overhanging lip, at ground level.
pub const OVERHANG: Point3<f32> = Point3::new(10.0, GROUND_HEIGHT, 19.4);

/// Centre of the broad convex ridge, at its crest.
pub const RIDGE: Point3<f32> = Point3::new(15.5, GROUND_HEIGHT + 3.0, 9.5);

/// Centre of the boulder.
pub const BOULDER: Point3<f32> = Point3::new(20.5, 2.6, 8.0);

/// Centre of the crater carved by `damage_sphere`.
pub const CRATER: Point3<f32> = Point3::new(4.2, 1.7, 9.0);

/// Radius of that carve.
const CRATER_RADIUS: f32 = 2.4;

/// A point on the cliff face, halfway up.
pub const CLIFF_FACE: Point3<f32> =
    Point3::new(16.0, (GROUND_HEIGHT + PLATEAU_HEIGHT) * 0.5, CLIFF_Z);

/// Between the thin pillar and the thin arch.
pub const SPIRES: Point3<f32> = Point3::new(25.0, GROUND_HEIGHT + 1.5, 9.5);

/// Base of the thin pillar.
const PILLAR: (f32, f32) = (25.0, 6.0);

/// Radius of the thin pillar — 0.9 m across, under four voxels.
const PILLAR_RADIUS: f32 = 0.45;

/// Cross-section of the thin arch, in metres.
const ARCH_THICKNESS: f32 = 0.7;

/// A meshed piece of terrain, in world space.
pub struct TerrainTableau {
    /// Vertices as marching cubes emitted them: unwelded, vertex-coloured by
    /// voxel material.
    pub vertices: Vec<Vertex>,

    /// Triangle indices into [`Self::vertices`].
    pub indices: Vec<u32>,
}

impl TerrainTableau {
    /// Wrap the geometry as a matte drawable at the origin.
    ///
    /// Takes `&self` and copies, because a sheet draws the same plot from
    /// several cameras and meshing it once per shot is pure waste.
    pub fn mesh(&self) -> SceneMesh {
        SceneMesh::new(self.vertices.clone(), self.indices.clone())
            .with_surface(SurfaceParams::MATTE)
    }

    /// The same geometry with every vertex reading as fully open to the sky.
    ///
    /// The control for any AO comparison. Neutralising the bake in the vertex
    /// data rather than in the shader means the two tiles go down the same
    /// pipeline with the same shaders, so nothing but the occlusion term
    /// differs between them.
    pub fn mesh_without_occlusion(&self) -> SceneMesh {
        let mut vertices = self.vertices.clone();
        for vertex in &mut vertices {
            vertex.ao = 1.0;
        }
        SceneMesh::new(vertices, self.indices.clone()).with_surface(SurfaceParams::MATTE)
    }
}

/// Generate, mesh and damage the plot, returning its world-space geometry.
///
/// Deterministic: every feature is authored, every noise seed is fixed, and the
/// segment iterates its chunks in sorted order. Two calls produce byte-identical
/// output, which is the property the whole bench depends on.
pub fn build() -> TerrainTableau {
    let terrain = description();
    let bounds = terrain.bounds.to_aabb();
    let durability = DurabilityConfig::default();

    let mut grid = ChunkGrid::new(terrain.voxel_size);
    generate_terrain(&mut grid, &terrain, &durability, &bounds);

    let mut segment = Segment::new("visual_bench", SegmentFrame::identity(), grid, Vec::new());
    let mut rebuilt: Vec<AABB> = Vec::new();
    segment.update(&mut rebuilt);

    // The crater goes in through the destruction path rather than through the
    // generator, because that is the path a grenade takes and it authors voxels
    // differently: a binary carve to air rather than a signed distance.
    segment.damage_sphere(CRATER, CRATER_RADIUS, u8::MAX);
    rebuilt.clear();
    segment.update(&mut rebuilt);

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    segment.append_render_data(&mut vertices, &mut indices);

    TerrainTableau { vertices, indices }
}

/// The authored plot.
///
/// Feature order matters: `Cliff` sets an *absolute* height, so it has to come
/// before anything that adds to it. Listed after it, the walls and the
/// roughness would be flattened away across the whole plot.
fn description() -> Terrain {
    Terrain {
        voxel_size: VOXEL_SIZE,
        bounds: Extent {
            min: (0.0, FLOOR_HEIGHT, 0.0),
            max: (PLOT_EXTENT, PLATEAU_HEIGHT + 4.0, PLOT_EXTENT),
        },
        base_height: GROUND_HEIGHT,
        material_layers: vec![
            MaterialLayer {
                depth: 1.2,
                material: VoxelMaterialId::Grass,
            },
            MaterialLayer {
                depth: 3.0,
                material: VoxelMaterialId::Dirt,
            },
            MaterialLayer {
                depth: 64.0,
                material: VoxelMaterialId::Rock,
            },
        ],
        features: vec![
            cliff(),
            TerrainFeature::TerrainRoughness {
                frequency: 0.07,
                amplitude: 0.12,
                octaves: 3,
                seed: 1337,
            },
            crevice_wall(CREVICE.x - CREVICE_HALF_GAP),
            crevice_wall(CREVICE.x + CREVICE_HALF_GAP),
            // The convex case. Broad and smooth, so nothing about it should
            // pick up occlusion.
            TerrainFeature::Wall {
                from: (RIDGE.x, 3.0),
                to: (RIDGE.x, 16.0),
                height: RIDGE.y - GROUND_HEIGHT,
                thickness: 5.0,
            },
        ],
        volumes: vec![
            // The underside. A lip of solid rock hanging off the cliff top with
            // nothing beneath it for six metres.
            VolumeFeature::Overhang {
                from: (6.0, CLIFF_Z),
                to: (14.0, CLIFF_Z),
                height: PLATEAU_HEIGHT,
                depth: 3.2,
                thickness: 1.8,
                direction: (0.0, -1.0),
                noise: 0.4,
                noise_seed: 11,
            },
            VolumeFeature::Island {
                center: (BOULDER.x, BOULDER.y, BOULDER.z),
                half_extents: (1.2, 1.0, 1.2),
                edge_noise: 0.3,
            },
            VolumeFeature::Pillar {
                center: PILLAR,
                height: GROUND_HEIGHT + 4.0,
                radius: PILLAR_RADIUS,
            },
            VolumeFeature::Arch {
                from: (22.8, GROUND_HEIGHT - 0.8, 12.5),
                to: (27.8, GROUND_HEIGHT - 0.8, 12.5),
                radius: 2.2,
                thickness: ARCH_THICKNESS,
            },
        ],
    }
}

/// The cliff, and with it the slope sweep.
fn cliff() -> TerrainFeature {
    TerrainFeature::Cliff {
        from: (1.0, CLIFF_Z),
        to: (31.0, CLIFF_Z),
        low_height: GROUND_HEIGHT,
        high_height: PLATEAU_HEIGHT,
        high_side: (0.0, 1.0),
        steepness: CLIFF_STEEPNESS,
        end_falloff: 4.0,
        roughness: 0.8,
        roughness_seed: 7,
    }
}

/// One side of the crevice: a raised strip whose cosine falloff gives the slot
/// its steep inner face.
fn crevice_wall(x: f32) -> TerrainFeature {
    TerrainFeature::Wall {
        from: (x, 4.0),
        to: (x, 15.0),
        height: CREVICE_DEPTH,
        thickness: CREVICE_THICKNESS,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench compares a sheet against its own past, which is worthless if
    /// the subject wanders. Everything here is authored or seeded, so two builds
    /// must agree bit for bit — including the crater, whose carve runs through
    /// the SVO's sphere traversal.
    #[test]
    fn the_plot_is_identical_across_builds() {
        let first = build();
        let second = build();

        assert!(!first.vertices.is_empty(), "the plot meshed to nothing");
        assert_eq!(first.indices, second.indices);
        assert_eq!(first.vertices.len(), second.vertices.len());

        for (index, (a, b)) in first.vertices.iter().zip(&second.vertices).enumerate() {
            assert_eq!(a.pos, b.pos, "vertex {index} position");
            assert_eq!(a.normal, b.normal, "vertex {index} normal");
            assert_eq!(a.color, b.color, "vertex {index} colour");
        }
    }

    /// The declared voxel size has been a factor of two out elsewhere in the
    /// engine, so the scale the cameras are framed against is asserted rather
    /// than assumed: the plot must be `PLOT_EXTENT` metres across and its
    /// plateau must reach `PLATEAU_HEIGHT`.
    #[test]
    fn the_plot_is_the_size_it_declares() {
        let tableau = build();
        let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
        for vertex in &tableau.vertices {
            for axis in 0..3 {
                lo[axis] = lo[axis].min(vertex.pos[axis]);
                hi[axis] = hi[axis].max(vertex.pos[axis]);
            }
        }

        assert!(
            lo[0] >= -VOXEL_SIZE && hi[0] <= PLOT_EXTENT + VOXEL_SIZE,
            "x span {lo:?}..{hi:?}"
        );
        assert!(
            lo[2] >= -VOXEL_SIZE && hi[2] <= PLOT_EXTENT + VOXEL_SIZE,
            "z span {lo:?}..{hi:?}"
        );
        assert!(
            (hi[1] - PLATEAU_HEIGHT).abs() < 1.5,
            "plateau reached {} rather than {PLATEAU_HEIGHT}",
            hi[1]
        );
    }

    /// The overhang is the one feature a heightfield cannot express, so it is
    /// worth proving it actually became one: there must be geometry with a
    /// downward-facing normal well above the ground and well away from the
    /// bottom of the generated volume.
    #[test]
    fn the_overhang_has_a_downward_facing_underside() {
        let tableau = build();
        let underside = tableau
            .vertices
            .iter()
            .filter(|v| v.normal.y < -0.5)
            .filter(|v| v.pos.y > GROUND_HEIGHT + 2.0)
            .filter(|v| (v.pos.x - OVERHANG.x).abs() < 5.0 && (v.pos.z - OVERHANG.z).abs() < 3.0)
            .count();

        assert!(
            underside > 50,
            "only {underside} downward-facing vertices under the lip"
        );
    }
}
