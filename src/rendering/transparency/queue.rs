//! Putting blended draws in the right order.
//!
//! Alpha blending is not commutative: `over(a, over(b, dst))` is not
//! `over(b, over(a, dst))`, so two blended surfaces composited in the wrong
//! order give the wrong colour. Opaque geometry does not care — the depth
//! buffer resolves it per pixel, whatever order the draws arrive in — but a
//! blended surface does not write depth, and so has nothing to resolve it with.
//! The order it is *recorded* in is the order it is composited in.
//!
//! So blended geometry cannot be drawn in the order it is encountered. It is
//! collected here instead, sorted, and recorded after the last opaque draw:
//!
//! ```text
//!   draw_mesh ─┬─ opaque  ──▶ held, in issue order ──────────▶ recorded at flush, first
//!              └─ blended ──▶ TransparentQueue ──▶ sorted ──▶ recorded at flush, after
//!                                  (far to near)
//! ```
//!
//! # What the ordering does and does not fix
//!
//! Sorting is per draw, by the distance from the camera to the mesh's centre.
//! That resolves the case that matters — several separate transparent objects
//! at different depths — and is what a per-object sort can resolve. It cannot
//! order two triangles *within* one mesh, and no amount of sorting draws will:
//! the back face and the front face of an ice cube are one draw. That half is
//! handled by drawing each blended mesh twice, back faces then front faces,
//! which is exact for a convex shape and is what the renderer does with what
//! this queue hands back.
//!
//! The frame's particles are ordered against these draws rather than through
//! this queue: they are already sorted among themselves, so the renderer merges
//! the two streams at flush time by the distance key both are keyed on.
//!
//! Objects that interpenetrate, or a concave blended mesh, will still composite
//! wrong in places. Fixing those needs order-independent transparency (depth
//! peeling, or a weighted blend), which is a different and much more expensive
//! apparatus than this one.

use nalgebra::Vector3;

use crate::rendering::geometry_draw::GeometryDraw;
use crate::rendering::vertex::Vertex;

/// One blended draw, held back for the sorted flush.
///
/// Holding a draw costs no borrow of the mesh it came from: the geometry is
/// already committed to the frame, and this is all the flush needs to record
/// it and put it in order.
#[derive(Clone, Copy, Debug)]
pub struct BlendedDraw {
    /// The draw itself, as recorded at the flush.
    pub geometry: GeometryDraw,

    /// Squared distance from the camera to the mesh's world-space centre.
    ///
    /// Squared because the sort only ever compares it: taking the root would
    /// be a wasted `sqrt` per draw that cannot change any ordering.
    depth_key: f32,
}

impl BlendedDraw {
    /// Sort this draw by the distance from `camera_pos` to the centre of
    /// `bounds`, the mesh's model-space bounding box.
    ///
    /// The box centre rather than the average of the vertices: a mesh with
    /// dense detail at one end — a tapered stone, a character's head — drags
    /// its vertex average towards the detail and away from where the object
    /// visibly is.
    pub fn sorted_from(mut self, camera_pos: &Vector3<f32>, bounds: &MeshBounds) -> Self {
        let centre = self.geometry.model.transform_point(&bounds.centre().into());
        self.depth_key = (centre.coords - camera_pos).norm_squared();
        self
    }

    /// Squared distance from the camera to this draw, for a caller that has
    /// to place something else in the same order — the frame's particles are
    /// interleaved with these draws by comparing against it.
    pub fn depth_key(&self) -> f32 {
        self.depth_key
    }

    /// A draw with no sort key yet. Recorded first if never given one, which
    /// is the safe default: a queue that silently dropped an unsorted draw
    /// would be much harder to notice than one that mis-orders it.
    pub fn new(geometry: GeometryDraw) -> Self {
        Self {
            geometry,
            depth_key: f32::INFINITY,
        }
    }
}

/// A mesh's extent in its own model space.
#[derive(Clone, Copy, Debug)]
pub struct MeshBounds {
    min: Vector3<f32>,
    max: Vector3<f32>,
}

impl MeshBounds {
    /// The box enclosing every vertex. Empty input gives a box at the origin,
    /// which sorts the draw as if it sat on the object's own position — the
    /// best available answer for geometry that describes no extent.
    pub fn of(vertices: &[Vertex]) -> Self {
        let mut min = Vector3::repeat(f32::INFINITY);
        let mut max = Vector3::repeat(f32::NEG_INFINITY);
        for vertex in vertices {
            min = min.inf(&vertex.pos);
            max = max.sup(&vertex.pos);
        }
        if vertices.is_empty() {
            min = Vector3::zeros();
            max = Vector3::zeros();
        }
        Self { min, max }
    }

    /// The box's centre, in model space.
    pub fn centre(&self) -> Vector3<f32> {
        (self.min + self.max) * 0.5
    }

    /// Half the box's diagonal: the radius of the sphere around it.
    pub fn half_diagonal(&self) -> f32 {
        (self.max - self.min).norm() * 0.5
    }
}

/// The frame's blended draws, collected during recording and flushed in order.
///
/// Frame-scoped like the surface table and the vertex buffer it indexes into:
/// cleared at the start of every frame, and never outlives the buffers its
/// entries point at.
#[derive(Default)]
pub struct TransparentQueue {
    draws: Vec<BlendedDraw>,
}

impl TransparentQueue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Drop last frame's draws. Called from `begin_frame` alongside the
    /// rewind of the buffers these entries index into — the two must happen
    /// together, since a kept entry would point into rewound storage.
    pub fn begin_frame(&mut self) {
        self.draws.clear();
    }

    /// Hold a draw back for the flush.
    pub fn push(&mut self, draw: BlendedDraw) {
        self.draws.push(draw);
    }

    /// The frame's blended draws, farthest from the camera first.
    ///
    /// Sorting here rather than on insert because a draw's place depends on
    /// every other draw, and the comparison is only needed once, when the
    /// whole set is known.
    pub fn sorted(&mut self) -> &[BlendedDraw] {
        // Unstable: there is no meaningful tie to preserve between two draws at
        // exactly the same distance, and they cannot be ordered correctly by
        // anything this queue knows.
        self.draws.sort_unstable_by(|a, b| {
            b.depth_key
                .partial_cmp(&a.depth_key)
                // A NaN key means a degenerate transform upstream. Treating it
                // as equal keeps the sort total rather than letting it panic
                // on an inconsistent comparator.
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        &self.draws
    }

    pub fn is_empty(&self) -> bool {
        self.draws.is_empty()
    }

    /// How many blended draws this frame holds. Exposed for the debug overlay.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.draws.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ash::vk;
    use nalgebra::{Matrix4, Vector2, Vector4};

    use crate::rendering::frame::DrawInfo;
    use crate::rendering::mesh_source::MeshSource;
    use crate::rendering::surface_buffer::SurfaceIndex;

    fn vertex(pos: Vector3<f32>) -> Vertex {
        Vertex {
            pos,
            color: Vector4::new(1.0, 1.0, 1.0, 1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::y(),
            ao: 1.0,
        }
    }

    fn draw_at(position: Vector3<f32>, camera: Vector3<f32>) -> BlendedDraw {
        let cube = [
            vertex(Vector3::new(-1.0, -1.0, -1.0)),
            vertex(Vector3::new(1.0, 1.0, 1.0)),
        ];
        BlendedDraw::new(GeometryDraw {
            model: Matrix4::new_translation(&position),
            draw: DrawInfo {
                index_count: 3,
                first_index: 0,
                vertex_offset: 0,
                source: MeshSource::Frame,
            },
            surface_index: SurfaceIndex(0),
            texture_set: vk::DescriptorSet::null(),
        })
        .sorted_from(&camera, &MeshBounds::of(&cube))
    }

    /// The property the whole queue exists for. Pushed near-first, and the
    /// flush must still composite the far one underneath.
    #[test]
    fn the_farthest_draw_is_recorded_first() {
        let camera = Vector3::zeros();
        let mut queue = TransparentQueue::new();
        queue.push(draw_at(Vector3::new(0.0, 0.0, 5.0), camera));
        queue.push(draw_at(Vector3::new(0.0, 0.0, 50.0), camera));
        queue.push(draw_at(Vector3::new(0.0, 0.0, 20.0), camera));

        let order: Vec<f32> = queue
            .sorted()
            .iter()
            .map(|d| d.geometry.model[(2, 3)])
            .collect();
        assert_eq!(order, vec![50.0, 20.0, 5.0]);
    }

    /// Distance from the camera, not distance along any one axis: a cube off
    /// to the side at the same z is nearer than one straight ahead further
    /// out, and an axis-only key would get that backwards.
    #[test]
    fn the_key_is_a_distance_rather_than_a_depth_along_an_axis() {
        let camera = Vector3::new(0.0, 0.0, -10.0);
        let mut queue = TransparentQueue::new();
        let near = Vector3::new(0.0, 0.0, 0.0);
        let far = Vector3::new(40.0, 0.0, 0.0);
        queue.push(draw_at(near, camera));
        queue.push(draw_at(far, camera));

        let first = queue.sorted()[0].geometry.model;
        assert_eq!(first[(0, 3)], 40.0);
    }

    /// A mesh's sort position follows its geometry, not just its transform:
    /// a bar modelled entirely off to one side of its own origin sits where
    /// its triangles are.
    #[test]
    fn the_bounds_centre_the_key_on_the_geometry() {
        let offset = [
            vertex(Vector3::new(9.0, 0.0, 0.0)),
            vertex(Vector3::new(11.0, 0.0, 0.0)),
        ];
        let bounds = MeshBounds::of(&offset);
        assert_eq!(bounds.centre(), Vector3::new(10.0, 0.0, 0.0));
    }

    /// Nothing to sort is not a special case anyone should have to handle.
    #[test]
    fn an_empty_queue_flushes_to_nothing() {
        let mut queue = TransparentQueue::new();
        assert!(queue.is_empty());
        assert!(queue.sorted().is_empty());
    }

    /// The frame boundary is the one invariant that is not merely cosmetic:
    /// a kept entry points into a vertex buffer that has since been rewound.
    #[test]
    fn a_new_frame_starts_with_nothing_held_back() {
        let mut queue = TransparentQueue::new();
        queue.push(draw_at(Vector3::zeros(), Vector3::zeros()));
        queue.begin_frame();
        assert!(queue.is_empty());
    }
}
