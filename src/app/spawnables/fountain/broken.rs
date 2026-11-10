//! How a piece cleaved off one of a fountain's stone blocks is drawn: the
//! carved surface it kept in the block's colour, rounded as the block was,
//! and its breaks flat and pale.
//!
//! ```text
//!   piece hull ──face by face──▶ on the whole block's surface?
//!                                  yes: the block's colour, smooth
//!                                  no:  fresh stone, flat
//! ```

use std::sync::Arc;

use nalgebra::{Vector2, Vector3};

use super::blocks::Stone;
use crate::app::spawnables::shared::finish::ColliderSurface;
use crate::app::spawnables::shared::rose_stone::{self, STONE};
use crate::collision::convex_hull::ConvexHull;
use crate::fracture::FaceMesher;
use crate::model::{Look, MeshPrimitive};
use crate::physics::ColliderDesc;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// The inside of the stone, where it has just broken.
const FRESH: Colour = Colour::new(0.68, 0.47, 0.44, 1.0);

/// How nearly a piece's face must lie along a face of the whole block to
/// be the block's surface: the cosine between their normals, and the gap
/// between their planes, m. A piece is drawn in by a few millimetres when
/// it is cut, which the gap allows for.
const SAME_FACING: f32 = 0.9995;
const SAME_PLANE: f32 = 0.02;

/// A whole block a piece may have come from.
struct Whole {
    stone: Stone,
    /// Where its frame's origin sits in the tower's frame.
    at: Vector3<f32>,
}

impl Whole {
    /// How far outside the block `point`, in the tower's frame, lies:
    /// negative inside.
    fn outside(&self, point: Vector3<f32>) -> f32 {
        let hull = &self.stone.hull;
        let local = point - self.at;
        hull.faces
            .iter()
            .map(|face| {
                face.normal
                    .dot(&(local - hull.vertices[face.vertex_indices[0] as usize]))
            })
            .fold(f32::MIN, f32::max)
    }

    /// Whether the plane through `on` (in the tower's frame) facing
    /// `normal` is one of the block's faces.
    fn has_face(&self, on: Vector3<f32>, normal: Vector3<f32>) -> bool {
        let hull = &self.stone.hull;
        let local = on - self.at;
        hull.faces.iter().any(|face| {
            let corner = hull.vertices[face.vertex_indices[0] as usize];
            face.normal.dot(&normal) > SAME_FACING
                && face.normal.dot(&(local - corner)).abs() < SAME_PLANE
        })
    }
}

/// Makes and draws the pieces of a fountain's broken blocks.
pub(super) struct BrokenStone {
    wholes: Vec<Whole>,
    material: MaterialId,
    /// The tower's scale, which sets the size of its stone's mottling.
    scale: f32,
}

impl BrokenStone {
    /// For a tower of `scale` whose stone is drawn in `material`, with
    /// each block that can break and where it sits in the tower's frame.
    pub fn new(
        blocks: impl IntoIterator<Item = (Stone, Vector3<f32>)>,
        material: MaterialId,
        scale: f32,
    ) -> Self {
        Self {
            wholes: blocks
                .into_iter()
                .map(|(stone, at)| Whole { stone, at })
                .collect(),
            material,
            scale,
        }
    }

    /// The block a piece whose frame sits at `at` was cut from: the one it
    /// lies deepest inside.
    fn whole_of(&self, at: Vector3<f32>) -> Option<&Whole> {
        self.wholes
            .iter()
            .min_by(|a, b| a.outside(at).total_cmp(&b.outside(at)))
    }
}

impl FaceMesher for BrokenStone {
    fn collider(&self, hull: Arc<ConvexHull>) -> ColliderDesc {
        ColliderDesc::convex_hull(hull).with_physical_surface(STONE)
    }

    fn look(&self, hull: &ConvexHull, at: Vector3<f32>) -> Look {
        let whole = self.whole_of(at);
        let carved: Vec<bool> = hull
            .faces
            .iter()
            .map(|face| {
                let on = at + hull.vertices[face.vertex_indices[0] as usize];
                whole.is_some_and(|whole| whole.has_face(on, face.normal))
            })
            .collect();
        // A corner's normal on the carved surface: across the carved faces
        // that meet there, so the surface reads as round as it was.
        let mut rounded = vec![Vector3::zeros(); hull.vertices.len()];
        for (face, _) in hull.faces.iter().zip(&carved).filter(|(_, &carved)| carved) {
            for &corner in &face.vertex_indices {
                rounded[corner as usize] += face.normal;
            }
        }

        let surface = whole.map_or(FRESH, |whole| whole.stone.colour);
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for (face, &carved) in hull.faces.iter().zip(&carved) {
            let first = vertices.len() as u32;
            for &corner in &face.vertex_indices {
                let normal = if carved {
                    rounded[corner as usize].normalize()
                } else {
                    face.normal
                };
                vertices.push(Vertex {
                    // In the tower's frame for now, so the mottling runs on
                    // from piece to piece.
                    pos: at + hull.vertices[corner as usize],
                    color: if carved { surface } else { FRESH }.to_vec4(),
                    tex_coords: Vector2::zeros(),
                    normal,
                    ao: 1.0,
                });
            }
            for corner in 1..face.vertex_indices.len() as u32 - 1 {
                indices.extend([first, first + corner, first + corner + 1]);
            }
        }
        let mut mesh = MeshPrimitive {
            vertices,
            indices,
            material: self.material,
        };
        rose_stone::mottle(self.scale).paint(&mut mesh);
        for vertex in &mut mesh.vertices {
            vertex.pos -= at;
        }
        Look {
            primitives: vec![mesh],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::blocks::Block;
    use super::*;
    use crate::collision::hull_split::{split_hull, Plane};

    /// A ball of stone standing off the tower's axis, and its mesher.
    fn ball() -> (Stone, Vector3<f32>, BrokenStone) {
        let at = Vector3::new(0.0, 4.0, 0.0);
        let block = Block::stone_ball(0.7, at, Colour::new(0.5, 0.2, 0.2, 1.0));
        let stone = block.stone.unwrap();
        let mesher = BrokenStone::new([(stone.clone(), at)], MaterialId(0), 1.0);
        (stone, at, mesher)
    }

    /// Whether a vertex is drawn as fresh stone: redder-pale, unlike the
    /// carved colour, whatever the mottling did to both.
    fn is_fresh(vertex: &Vertex) -> bool {
        vertex.color.y / vertex.color.x > 0.55
    }

    #[test]
    fn a_whole_block_is_all_carved_surface() {
        let (stone, at, mesher) = ball();
        let look = mesher.look(&stone.hull, at);
        assert!(look.primitives[0].vertices.iter().all(|v| !is_fresh(v)));
    }

    #[test]
    fn half_a_block_shows_its_break_and_keeps_its_surface() {
        let (stone, at, mesher) = ball();
        let cut = Plane {
            normal: Vector3::new(1.0, 0.3, 0.0).normalize(),
            offset: 0.0,
        };
        let halves = split_hull(&stone.hull, cut).unwrap();
        let centre = halves.front.centroid();
        let piece = halves.front.translated(-centre);
        let look = mesher.look(&piece, at + centre);
        let vertices = &look.primitives[0].vertices;
        let fresh: Vec<&Vertex> = vertices.iter().filter(|v| is_fresh(v)).collect();
        assert!(!fresh.is_empty(), "the break is not drawn");
        assert!(fresh.len() < vertices.len() / 2, "its surface went too");
        // The break is one flat face: every fresh vertex faces along the cut.
        for vertex in fresh {
            assert!(vertex.normal.dot(&cut.normal).abs() > 0.999);
        }
    }
}
