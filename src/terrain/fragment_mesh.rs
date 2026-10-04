//! A fragment's surface, rebuilt from its own samples for drawing it in flight.
//!
//! The fragment keeps its samples on the lattice it was cut from, so the
//! terrain's own mesher run over them reproduces the ground's triangles
//! exactly wherever the fragment was not joined to anything, and closes it
//! with new faces where it broke. AO is baked on the fragment alone: a rock in
//! the air is not shaded by the socket it came out of.
//!
//! ```text
//!   Fragment ──▶ MeshOctree::generate_block (marching cubes + AO), over a
//!            │   block of its own samples with air everywhere else
//!            ▼
//!   FragmentMesh: vertices around the fragment's world centroid, turned into
//!                 world axes, so model position + centroid = world position
//! ```

use nalgebra::Point3;

use super::fragment::{overlay, Fragment};
use super::mesh_octree::MeshOctree;
use super::voxel_block::{VoxelBlock, VoxelSource};
use crate::collision::AABB;
use crate::rendering::vertex::Vertex;

/// A fragment's render mesh, placed around where it was when it broke.
pub struct FragmentMesh {
    /// Positions relative to [`Self::origin`], in world axes.
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    /// The fragment's world centroid at the moment of the blast: where the
    /// mesh's own origin was then.
    pub origin: Point3<f32>,
}

/// AO's coarse lattice is anchored at sample indices divisible by this, and a
/// block's cell count must divide by it too. It covers every divisor
/// `terrain::ao` resolves to.
const OCCLUSION_ALIGNMENT: i32 = 2;

impl Fragment {
    /// The fragment's surface as the terrain's mesher draws it.
    pub fn mesh(&self) -> FragmentMesh {
        let voxels = self.voxels();
        let lattice = voxels.lattice();
        let step = lattice.spacing();

        // One cubic block over the fragment's samples, aligned for AO. Its
        // first cell sits one sample in from the fragment block's air padding,
        // which the mesher reads as the halo either side of it.
        let align = |i: i32| i.div_euclid(OCCLUSION_ALIGNMENT) * OCCLUSION_ALIGNMENT;
        let first = lattice.base().map(align);
        let last = [0, 1, 2].map(|a| lattice.base()[a] + lattice.dims()[a] as i32 - 1);
        let span = (0..3).map(|a| last[a] - first[a]).max().unwrap_or(0);
        let cells = (span + OCCLUSION_ALIGNMENT - 1) / OCCLUSION_ALIGNMENT * OCCLUSION_ALIGNMENT;

        let at = |i: [i32; 3]| Point3::new(i[0] as f32, i[1] as f32, i[2] as f32) * step;
        let bounds = AABB::new(at(first), at(first.map(|i| i + cells)));
        let mut octree = MeshOctree::new(bounds);
        octree.generate_block(
            Point3::origin(),
            first,
            cells as usize,
            step,
            &Alone(voxels),
        );
        let (vertices, indices) = octree.get_render_data();

        let pose = self.pose();
        let origin = self.world_centroid();
        let vertices = vertices
            .into_iter()
            .map(|mut vertex| {
                let world = pose * Point3::from(vertex.pos);
                vertex.pos = world - origin;
                vertex.normal = pose.rotation * vertex.normal;
                vertex
            })
            .collect();
        FragmentMesh {
            vertices,
            indices,
            origin,
        }
    }
}

/// A fragment's samples with nothing around them: every sample off its block
/// reads as air.
struct Alone<'a>(&'a VoxelBlock);

impl VoxelSource for Alone<'_> {
    fn fill_block(&self, block: &mut VoxelBlock) {
        overlay(block, self.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::chunk_grid::ChunkGrid;
    use crate::terrain::fragment::Search;
    use crate::terrain::frame::SegmentFrame;
    use crate::terrain::voxel::{Voxel, VoxelMaterial};

    /// Bedrock, and a rounded rock standing free above it, as authored: the
    /// audit holds up only what an indestructible sample does.
    fn rock_over_ground() -> ChunkGrid {
        let mut grid = ChunkGrid::new(1.0);
        for x in -12..=12 {
            for z in -12..=12 {
                for y in -4..=0 {
                    let at = Point3::new(x as f32, y as f32, z as f32);
                    grid.set(
                        at,
                        Voxel {
                            density: 1.0,
                            material: VoxelMaterial::Bedrock,
                        },
                    );
                }
            }
        }
        let centre = Point3::new(0.3, 6.2, -0.4);
        for x in -4..=4 {
            for y in 2..=10 {
                for z in -4..=4 {
                    let at = Point3::new(x as f32, y as f32, z as f32);
                    let density = (2.6 - (at - centre).norm()).clamp(-1.0, 1.0);
                    if density > 0.0 {
                        grid.set(
                            at,
                            Voxel {
                                density,
                                material: VoxelMaterial::Rock,
                            },
                        );
                    }
                }
            }
        }
        grid
    }

    /// The ground's triangles near `around`, as the chunk mesher drew them:
    /// each corner's position and normal, quantised so equal floats compare.
    fn ground_triangles(
        grid: &ChunkGrid,
        around: &AABB,
        frame: &SegmentFrame,
    ) -> Vec<[[i64; 6]; 3]> {
        let mut octree = MeshOctree::new(AABB::new(
            Point3::new(-16.0, -8.0, -16.0),
            Point3::new(16.0, 24.0, 16.0),
        ));
        octree.generate_block(Point3::origin(), [-16, -8, -16], 32, 1.0, grid);
        let (vertices, indices) = octree.get_render_data();
        let mut triangles: Vec<_> = indices
            .chunks(3)
            .filter(|t| {
                t.iter()
                    .all(|&i| around.contains_point(Point3::from(vertices[i as usize].pos)))
            })
            .map(|t| {
                t.iter()
                    .map(|&i| {
                        let v = vertices[i as usize];
                        key(
                            frame.to_world(Point3::from(v.pos)),
                            frame.rotate_to_world(v.normal),
                        )
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap()
            })
            .collect();
        triangles.sort_unstable();
        triangles
    }

    fn mesh_triangles(mesh: &FragmentMesh) -> Vec<[[i64; 6]; 3]> {
        let mut triangles: Vec<_> = mesh
            .indices
            .chunks(3)
            .map(|t| {
                t.iter()
                    .map(|&i| {
                        let v = mesh.vertices[i as usize];
                        key(mesh.origin + v.pos, v.normal)
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap()
            })
            .collect();
        triangles.sort_unstable();
        triangles
    }

    fn key(p: Point3<f32>, n: nalgebra::Vector3<f32>) -> [i64; 6] {
        let q = |v: f32| (v * 1.0e4).round() as i64;
        [q(p.x), q(p.y), q(p.z), q(n.x), q(n.y), q(n.z)]
    }

    /// A piece that broke away from nothing is drawn in flight exactly as the
    /// ground drew it: the same triangles in the same places, facing the same
    /// way, and in a yawed segment as in an unrotated one.
    #[test]
    fn a_free_piece_is_drawn_as_the_ground_drew_it() {
        for frame in [
            SegmentFrame::identity(),
            SegmentFrame::new(Point3::new(40.0, -3.0, 12.0), 1),
        ] {
            let grid = rock_over_ground();
            let audit = Search::audit(&grid);
            let mut fragments = audit.into_fragments(&frame);
            assert_eq!(
                fragments.len(),
                1,
                "the rock is the one piece standing free"
            );
            let mesh = fragments.remove(0).mesh();

            let around = AABB::new(Point3::new(-5.0, 1.0, -5.0), Point3::new(5.0, 11.0, 5.0));
            let drawn = ground_triangles(&grid, &around, &frame);
            assert!(!drawn.is_empty());
            assert_eq!(mesh_triangles(&mesh), drawn);
        }
    }

    /// The mesh's origin is the fragment's centroid, so it turns about its
    /// middle.
    #[test]
    fn the_mesh_is_centred_on_the_fragment() {
        let grid = rock_over_ground();
        let fragment = Search::audit(&grid)
            .into_fragments(&SegmentFrame::identity())
            .remove(0);
        let mesh = fragment.mesh();
        assert_eq!(mesh.origin, fragment.world_centroid());
        let mean = mesh
            .vertices
            .iter()
            .fold(nalgebra::Vector3::zeros(), |sum, v| sum + v.pos)
            / mesh.vertices.len() as f32;
        assert!(
            mean.norm() < 0.3,
            "vertices centred on the origin, mean {mean:?}"
        );
    }
}
