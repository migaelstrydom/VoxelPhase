//! How a force field looks: its colours, proportions and pace, each as a
//! range from a pod at rest to one at its force cap.

use nalgebra::Vector3;

/// The heat at which the field is `warm`: it stays cyan through a landing
/// and only goes white toward the cap.
const WARM_AT: f32 = 0.6;

/// A value that runs from `rest` to `full` as a pod's heat goes from 0 to 1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    /// At no load.
    pub rest: f32,
    /// At the suspension's force cap.
    pub full: f32,
}

impl Span {
    pub const fn new(rest: f32, full: f32) -> Self {
        Self { rest, full }
    }

    /// The value at `heat`, 0..1.
    pub fn at(&self, heat: f32) -> f32 {
        self.rest + (self.full - self.rest) * heat.clamp(0.0, 1.0)
    }
}

/// The look of one kind of force field. Distances are in metres, times in
/// seconds, glows in luminance (above about 1.3 an opaque surface blooms).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FieldStyle {
    /// The field's colour at rest: a cool cyan.
    pub cool: Vector3<f32>,
    /// Its colour carrying about three fifths of its cap.
    pub warm: Vector3<f32>,
    /// Its colour at the cap: white-hot.
    pub hot: Vector3<f32>,
    /// Radiance of the column's glow, multiplied into its colour.
    pub column_radiance: Span,
    /// Radius of the column where it leaves the pod.
    pub column_radius: f32,
    /// How far the column's foot spreads, as a multiple of `column_radius`:
    /// `rest` with the spring at its rest length, `full` pressed right in.
    pub column_flare: Span,
    /// Distance between bands down the column: they crowd as the load grows.
    pub band_spacing: Span,
    /// Bands passing a point each second.
    pub band_rate: Span,
    /// Radius of the emissive ring on the pod's face.
    pub pod_ring_radius: f32,
    /// Glow of the pod's ring.
    pub pod_glow: Span,
    /// Radius of the ring on the ground, at rest length and pressed right in.
    pub ground_ring_radius: Span,
    /// Glow of the ground ring.
    pub ground_glow: Span,
    /// Extra glow on both rings, and the column, when a pod bottoms out.
    pub flash_glow: f32,
    /// How far a strain pulse carries the ring outward, as a multiple of its
    /// radius, and how long it takes.
    pub pulse_reach: f32,
    pub pulse_time: f32,
    /// Intensity of the light under the field, as luminance, and its range.
    pub light_intensity: Span,
    pub light_range: f32,
}

impl FieldStyle {
    /// A hover pod's field: cyan bands in a slim column over a ring on the
    /// ground, hot white when it carries a landing.
    pub const HOVER: Self = Self {
        cool: Vector3::new(0.2, 0.78, 1.0),
        warm: Vector3::new(0.45, 0.9, 1.0),
        hot: Vector3::new(1.0, 0.96, 0.9),
        column_radiance: Span::new(1.2, 3.0),
        column_radius: 0.06,
        column_flare: Span::new(2.4, 3.6),
        band_spacing: Span::new(0.11, 0.055),
        band_rate: Span::new(1.6, 9.0),
        pod_ring_radius: 0.1,
        pod_glow: Span::new(2.0, 6.0),
        ground_ring_radius: Span::new(0.18, 0.3),
        ground_glow: Span::new(1.8, 4.0),
        flash_glow: 4.0,
        pulse_reach: 0.9,
        pulse_time: 0.3,
        light_intensity: Span::new(1.2, 3.5),
        light_range: 3.5,
    };

    /// The field's colour at `heat`, 0..1: cool, through warm, to hot.
    pub fn colour(&self, heat: f32) -> Vector3<f32> {
        let heat = heat.clamp(0.0, 1.0);
        if heat < WARM_AT {
            self.cool.lerp(&self.warm, heat / WARM_AT)
        } else {
            self.warm
                .lerp(&self.hot, (heat - WARM_AT) / (1.0 - WARM_AT))
        }
    }
}

impl Default for FieldStyle {
    fn default() -> Self {
        Self::HOVER
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_colour_runs_from_cool_to_hot() {
        let style = FieldStyle::HOVER;
        assert_eq!(style.colour(0.0), style.cool);
        assert_eq!(style.colour(WARM_AT), style.warm);
        assert_eq!(style.colour(1.0), style.hot);
        assert_eq!(style.colour(2.0), style.hot);
    }

    #[test]
    fn a_span_is_clamped_to_its_ends() {
        let span = Span::new(1.0, 3.0);
        assert_eq!(span.at(-1.0), 1.0);
        assert_eq!(span.at(0.5), 2.0);
        assert_eq!(span.at(4.0), 3.0);
    }
}
