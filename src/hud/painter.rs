//! Somewhere for an element to put rectangles.

use nalgebra::{Vector2, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::overlay::OverlayGeometry;

/// Collects screen-space shapes into overlay geometry.
///
/// Every shape is a quad sampling the atlas's solid texel, so a filled
/// rectangle costs exactly what a glyph does and goes out in the same draw.
/// The vocabulary is deliberately small — a rotated rectangle is enough to
/// build bars, brackets, ticks and dots, and anything richer belongs in the
/// element that wants it rather than here.
pub struct HudPainter {
    geometry: OverlayGeometry,
    solid_uv: Vector2<f32>,
}

impl HudPainter {
    /// `solid_uv` comes from the overlay's font atlas: the UV of a texel that
    /// is fully opaque, which turns a textured quad into a flat one.
    pub fn new(solid_uv: Vector2<f32>) -> Self {
        Self {
            geometry: OverlayGeometry::new(),
            solid_uv,
        }
    }

    /// An axis-aligned filled rectangle.
    pub fn rect(&mut self, centre: Vector2<f32>, half_extents: Vector2<f32>, colour: Colour) {
        self.rotated_rect(centre, half_extents, 0.0, colour);
    }

    /// A filled rectangle turned `rotation` radians clockwise on screen.
    pub fn rotated_rect(
        &mut self,
        centre: Vector2<f32>,
        half_extents: Vector2<f32>,
        rotation: f32,
        colour: Colour,
    ) {
        if colour.a <= 0.0 || half_extents.x <= 0.0 || half_extents.y <= 0.0 {
            return;
        }

        let (sin, cos) = rotation.sin_cos();
        let x_axis = Vector2::new(cos, sin) * half_extents.x;
        let y_axis = Vector2::new(-sin, cos) * half_extents.y;

        let corners = [
            centre - x_axis - y_axis,
            centre + x_axis - y_axis,
            centre + x_axis + y_axis,
            centre - x_axis + y_axis,
        ];

        self.geometry
            .push_quad(corners, [self.solid_uv; 4], colour_to_vec4(colour));
    }

    /// A bar of `thickness` pixels spanning `from` to `to`.
    pub fn bar(&mut self, from: Vector2<f32>, to: Vector2<f32>, thickness: f32, colour: Colour) {
        let span = to - from;
        let length = span.magnitude();
        if length <= 1e-4 {
            return;
        }

        self.rotated_rect(
            from + span * 0.5,
            Vector2::new(length * 0.5, thickness * 0.5),
            span.y.atan2(span.x),
            colour,
        );
    }

    /// The geometry painted so far.
    pub fn geometry(&self) -> &OverlayGeometry {
        &self.geometry
    }

    /// Take the geometry, consuming the painter.
    pub fn into_geometry(self) -> OverlayGeometry {
        self.geometry
    }
}

fn colour_to_vec4(colour: Colour) -> Vector4<f32> {
    colour.to_vec4()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fully_transparent_shape_costs_no_geometry() {
        let mut painter = HudPainter::new(Vector2::zeros());
        painter.rect(
            Vector2::new(10.0, 10.0),
            Vector2::new(4.0, 4.0),
            Colour::WHITE.with_alpha(0.0),
        );

        assert!(painter.geometry().is_empty());
    }

    #[test]
    fn a_bar_spans_the_points_it_was_given() {
        let mut painter = HudPainter::new(Vector2::zeros());
        painter.bar(
            Vector2::new(0.0, 0.0),
            Vector2::new(10.0, 0.0),
            2.0,
            Colour::WHITE,
        );

        let xs: Vec<f32> = painter
            .geometry()
            .vertices()
            .iter()
            .map(|v| v.pos.x)
            .collect();
        let min = xs.iter().cloned().fold(f32::MAX, f32::min);
        let max = xs.iter().cloned().fold(f32::MIN, f32::max);

        assert!((min - 0.0).abs() < 1e-4, "starts at {}", min);
        assert!((max - 10.0).abs() < 1e-4, "ends at {}", max);
    }

    #[test]
    fn a_degenerate_bar_draws_nothing() {
        let mut painter = HudPainter::new(Vector2::zeros());
        painter.bar(Vector2::zeros(), Vector2::zeros(), 2.0, Colour::WHITE);

        assert!(painter.geometry().is_empty());
    }
}
