//! A frame's worth of overlay triangles.

use nalgebra::{Vector2, Vector4};

use super::vertex::OverlayVertex;

/// Screen-space triangles waiting to be drawn by the overlay pipeline.
///
/// Everything the overlay draws — debug text, HUD shapes — lands in one of
/// these and is uploaded once. That is not only a saving: the overlay owns a
/// single vertex buffer, and a second upload into it before the first draw has
/// executed would silently redraw the first batch with the second's contents.
/// One buffer, one upload, one draw.
#[derive(Debug, Default, Clone)]
pub struct OverlayGeometry {
    vertices: Vec<OverlayVertex>,
    indices: Vec<u32>,
}

impl OverlayGeometry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a quad from four corners, given clockwise from the top-left in
    /// screen pixels, each with its own UV.
    pub fn push_quad(
        &mut self,
        corners: [Vector2<f32>; 4],
        uvs: [Vector2<f32>; 4],
        colour: Vector4<f32>,
    ) {
        let base = self.vertices.len() as u32;
        for (pos, uv) in corners.into_iter().zip(uvs) {
            self.vertices.push(OverlayVertex {
                pos,
                uv,
                color: colour,
            });
        }
        self.indices
            .extend_from_slice(&[base, base + 1, base + 2, base + 2, base + 3, base]);
    }

    /// Append another batch, rebasing its indices onto this one.
    pub fn append(&mut self, other: &OverlayGeometry) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.indices.extend(other.indices.iter().map(|i| i + base));
    }

    /// Append vertices and indices produced elsewhere (text layout), rebasing
    /// the indices onto this batch.
    pub fn extend(&mut self, vertices: &[OverlayVertex], indices: &[u32]) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(vertices);
        self.indices.extend(indices.iter().map(|i| i + base));
    }

    pub fn vertices(&self) -> &[OverlayVertex] {
        &self.vertices
    }

    pub fn indices(&self) -> &[u32] {
        &self.indices
    }

    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Drop everything, keeping the allocation for the next frame.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_quad() -> OverlayGeometry {
        let mut geometry = OverlayGeometry::new();
        let corner = Vector2::zeros();
        geometry.push_quad(
            [corner; 4],
            [Vector2::zeros(); 4],
            Vector4::new(1.0, 1.0, 1.0, 1.0),
        );
        geometry
    }

    #[test]
    fn a_quad_is_four_vertices_and_two_triangles() {
        let geometry = a_quad();
        assert_eq!(geometry.vertices().len(), 4);
        assert_eq!(geometry.indices().len(), 6);
    }

    #[test]
    fn appending_rebases_the_indices_of_the_second_batch() {
        let mut first = a_quad();
        first.append(&a_quad());

        assert_eq!(first.vertices().len(), 8);
        assert_eq!(
            first.indices()[6..],
            [4, 5, 6, 6, 7, 4],
            "the second quad must index its own vertices"
        );
    }
}
