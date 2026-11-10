//! Pieces cut from carved blocks, each drawn as its share of the block's own
//! mesh: the carving it kept, and a fresh face where it broke.
//!
//! ```text
//!   piece hull ──▶ the whole block it lies inside
//!        │
//!        ▼  face by face: on the block's surface, or a cut?
//!   the block's mesh ──slice at each cut──▶ what the piece kept of it
//!                    ──cap each cut──────▶ its breaks, in fresh material
//! ```
//!
//! A block is carved however its spawnable likes: its mesh may stand proud
//! of its hull, carry ornaments, and wear any paint. The mesh is only ever
//! cut where the hull was, so a piece keeps all of it that lay on its side.
//! A piece cut again is drawn from the whole block again, by every cut it
//! now has.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector2, Vector3, Vector4};

use super::cutting::FaceMesher;
use crate::collision::convex_hull::ConvexHull;
use crate::model::{cap, slice, Look, MeshPrimitive};
use crate::physics::ColliderDesc;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// How nearly a piece's face must lie along a face of its whole block to be
/// the block's surface: the cosine between their normals, and the gap
/// between their planes as a share of the block's reach. Loose enough for a
/// piece drawn in by a hair when it was cut and for a corner the cutter
/// closed up, and far tighter than any cut that leaves two pieces worth
/// keeping.
const SAME_FACING: f32 = 0.995;
const SAME_PLANE: f32 = 0.02;

/// A whole block that pieces may be cut from.
pub struct CarvedBlock {
    /// Its shape, in its own frame: what it is cut as.
    pub hull: Arc<ConvexHull>,
    /// Where its frame sits in the frame its object's texture was laid in.
    pub frame: Isometry3<f32>,
    /// How it looks whole, in its own frame.
    pub look: Arc<Look>,
}

impl CarvedBlock {
    /// How far outside the block `point`, in the block's frame, lies:
    /// negative inside.
    fn outside(&self, point: Point3<f32>) -> f32 {
        let faces = self.hull.faces.iter().map(|face| {
            let on = self.hull.vertices[face.vertex_indices[0] as usize];
            face.normal.dot(&(point.coords - on))
        });
        faces.fold(f32::MIN, f32::max)
    }

    /// Whether the plane through `on` facing `normal`, both in the block's
    /// frame, is one of the block's faces.
    fn has_face(&self, on: Point3<f32>, normal: Vector3<f32>) -> bool {
        let gap = SAME_PLANE * self.hull.bounding_radius;
        self.hull.faces.iter().any(|face| {
            let corner = self.hull.vertices[face.vertex_indices[0] as usize];
            face.normal.dot(&normal) > SAME_FACING
                && face.normal.dot(&(on.coords - corner)).abs() < gap
        })
    }
}

/// How a fresh break is drawn.
#[derive(Clone, Copy)]
pub struct BreakFace {
    /// The material of the inside of the thing.
    pub material: MaterialId,
    /// Its colour there.
    pub colour: Vector4<f32>,
    /// How far across the break one repeat of the material's texture
    /// reaches, m.
    pub texture_span: f32,
}

/// Makes and draws the pieces of an object's carved blocks.
pub struct CarvedPieces {
    blocks: Vec<CarvedBlock>,
    breaks: BreakFace,
    /// The collider of a piece of a given shape, unplaced.
    collider: Box<dyn Fn(Arc<ConvexHull>) -> ColliderDesc + Send + Sync>,
}

impl CarvedPieces {
    /// For an object of these `blocks`, whose breaks are drawn as `breaks`
    /// and whose pieces collide as `collider` makes them.
    pub fn new(
        blocks: Vec<CarvedBlock>,
        breaks: BreakFace,
        collider: impl Fn(Arc<ConvexHull>) -> ColliderDesc + Send + Sync + 'static,
    ) -> Self {
        Self {
            blocks,
            breaks,
            collider: Box::new(collider),
        }
    }

    /// The block a piece whose frame sits at `at` was cut from: the one it
    /// lies deepest inside.
    fn block_of(&self, at: Vector3<f32>) -> Option<&CarvedBlock> {
        let outside = |block: &CarvedBlock| block.outside(block.frame.inverse() * Point3::from(at));
        self.blocks
            .iter()
            .min_by(|a, b| outside(a).total_cmp(&outside(b)))
    }

    /// The face that closes the holes whose edges are `rim`, in a plane
    /// facing `normal`, in a piece's frame; `at` is where that frame sits in
    /// the texture's.
    fn break_face(
        &self,
        rim: &[[Vector3<f32>; 2]],
        normal: Vector3<f32>,
        at: Vector3<f32>,
    ) -> MeshPrimitive {
        let (positions, indices) = cap(rim, normal);
        // The texture is laid flat on the break, in the texture's frame, so
        // the two sides of one break wear the same markings.
        let across = if normal.x.abs() < 0.9 {
            normal.cross(&Vector3::x()).normalize()
        } else {
            normal.cross(&Vector3::y()).normalize()
        };
        let along = normal.cross(&across);
        let vertices = positions
            .into_iter()
            .map(|pos| Vertex {
                pos,
                color: self.breaks.colour,
                tex_coords: Vector2::new((at + pos).dot(&across), (at + pos).dot(&along))
                    / self.breaks.texture_span,
                normal,
                ao: 1.0,
            })
            .collect();
        MeshPrimitive {
            vertices,
            indices,
            material: self.breaks.material,
        }
    }
}

impl FaceMesher for CarvedPieces {
    fn collider(&self, hull: Arc<ConvexHull>) -> ColliderDesc {
        (self.collider)(hull)
    }

    fn look(&self, hull: &ConvexHull, at: Vector3<f32>) -> Look {
        let Some(block) = self.block_of(at) else {
            return Look::default();
        };
        // The block's mesh in the piece's frame, which lies along the
        // texture's.
        let into_texture = block.frame;
        let mut meshes: Vec<MeshPrimitive> = block.look.primitives.clone();
        for vertex in meshes.iter_mut().flat_map(|mesh| &mut mesh.vertices) {
            vertex.pos = (into_texture * Point3::from(vertex.pos)).coords - at;
            vertex.normal = into_texture.rotation * vertex.normal;
        }

        let into_block = block.frame.inverse();
        for face in &hull.faces {
            let on = hull.vertices[face.vertex_indices[0] as usize];
            if block.has_face(
                into_block * Point3::from(at + on),
                into_block.rotation * face.normal,
            ) {
                continue;
            }
            let offset = face.normal.dot(&on);
            let mut rim = Vec::new();
            for mesh in &mut meshes {
                let sliced = slice(mesh, face.normal, offset);
                rim.extend(sliced.rim);
                *mesh = sliced.kept;
            }
            meshes.push(self.break_face(&rim, face.normal, at));
        }
        meshes.retain(|mesh| !mesh.indices.is_empty());
        Look { primitives: meshes }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::hull_split::{split_hull, Plane};

    const STONE: MaterialId = MaterialId(3);
    const FRESH: MaterialId = MaterialId(7);

    /// A mesh of the eight corners of a box of half-size `half`, standing
    /// `proud` of it all round.
    fn box_look(half: Vector3<f32>, proud: f32) -> Look {
        let hull = cube_hull(half + Vector3::repeat(proud));
        let (mut vertices, mut indices) = (Vec::new(), Vec::new());
        for face in &hull.faces {
            let first = vertices.len() as u32;
            for &corner in &face.vertex_indices {
                vertices.push(Vertex {
                    pos: hull.vertices[corner as usize],
                    color: Vector4::repeat(1.0),
                    tex_coords: Vector2::zeros(),
                    normal: face.normal,
                    ao: 1.0,
                });
            }
            for corner in 1..face.vertex_indices.len() as u32 - 1 {
                indices.extend([first, first + corner, first + corner + 1]);
            }
        }
        Look::single(vertices, indices, STONE)
    }

    /// A metre cube carved 5 cm proud of its hull, standing at `centre`.
    fn carved_cube(centre: Vector3<f32>) -> CarvedPieces {
        let half = Vector3::repeat(0.5);
        CarvedPieces::new(
            vec![CarvedBlock {
                hull: Arc::new(cube_hull(half)),
                frame: Isometry3::translation(centre.x, centre.y, centre.z),
                look: Arc::new(box_look(half, 0.05)),
            }],
            BreakFace {
                material: FRESH,
                colour: Vector4::repeat(1.0),
                texture_span: 1.0,
            },
            |hull| ColliderDesc::convex_hull(hull),
        )
    }

    /// The area of `look`'s faces in `material`.
    fn area(look: &Look, material: MaterialId) -> f32 {
        look.primitives
            .iter()
            .filter(|mesh| mesh.material == material)
            .flat_map(|mesh| {
                mesh.indices.chunks_exact(3).map(|t| {
                    let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize].pos);
                    0.5 * (b - a).cross(&(c - a)).norm()
                })
            })
            .sum()
    }

    #[test]
    fn a_whole_block_keeps_all_its_carving_and_shows_no_break() {
        let centre = Vector3::new(3.0, 1.0, -2.0);
        let look = carved_cube(centre).look(&cube_hull(Vector3::repeat(0.5)), centre);
        assert!((area(&look, STONE) - 6.0 * 1.1 * 1.1).abs() < 1.0e-3);
        assert_eq!(area(&look, FRESH), 0.0);
    }

    #[test]
    fn half_a_block_keeps_half_its_carving_and_shows_its_break() {
        let centre = Vector3::new(3.0, 1.0, -2.0);
        let whole = cube_hull(Vector3::repeat(0.5));
        let halves = split_hull(&whole, Plane::through(Vector3::zeros(), Vector3::x())).unwrap();
        let back = halves.back;
        let middle = back.centroid();
        let look = carved_cube(centre).look(&back.translated(-middle), centre + middle);

        // The carving stands proud of the hull, and the break runs right
        // across it: the piece is cut where the hull was and nowhere else.
        assert!((area(&look, FRESH) - 1.1 * 1.1).abs() < 1.0e-3);
        assert!((area(&look, STONE) - (1.1 * 1.1 + 4.0 * 1.1 * 0.55)).abs() < 1.0e-3);
        for vertex in look.primitives.iter().flat_map(|mesh| &mesh.vertices) {
            assert!(vertex.pos.x + middle.x <= 1.0e-4, "carving past the break");
        }
    }

    #[test]
    fn a_piece_cut_twice_shows_both_breaks() {
        let whole = cube_hull(Vector3::repeat(0.5));
        let halves = split_hull(&whole, Plane::through(Vector3::zeros(), Vector3::x())).unwrap();
        let quarters =
            split_hull(&halves.back, Plane::through(Vector3::zeros(), Vector3::y())).unwrap();
        let quarter = quarters.front;
        let middle = quarter.centroid();
        let look = carved_cube(Vector3::zeros()).look(&quarter.translated(-middle), middle);
        assert!((area(&look, FRESH) - 2.0 * 1.1 * 0.55).abs() < 1.0e-3);
    }
}
