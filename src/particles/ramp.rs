//! Colour ramps: how a particle's colour changes over its life.
//!
//! A two-colour lerp is enough for a spark, but not for fire. A fireball puff
//! is white-hot for the first instant, spends most of its life cooling through
//! yellow and orange, and ends as dark smoke that then fades out — four
//! distinct colours over one lifetime, unevenly spaced. Splitting that into
//! separate fire and smoke effects never looks right, because the handover
//! between the two is visible; the same particle has to do the whole journey.

use nalgebra::{Vector3, Vector4};

/// A colour the ramp passes through, and how far into the particle's life it
/// is reached.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColourStop {
    /// Normalised age at which this colour is exact, 0 (birth) to 1 (death).
    pub at: f32,
    /// Linear RGBA. Alpha is the particle's opacity at this point.
    pub colour: Vector4<f32>,
}

impl ColourStop {
    pub const fn new(at: f32, colour: Vector4<f32>) -> Self {
        Self { at, colour }
    }
}

/// An ordered set of colours a particle interpolates through as it ages.
///
/// Fixed capacity and `Copy`: every live particle carries its own ramp, and
/// particles are spawned in the hundreds per frame, so this must not allocate.
#[derive(Debug, Clone, Copy)]
pub struct ColourRamp {
    stops: [ColourStop; Self::MAX_STOPS],
    /// How many entries of `stops` are meaningful. Never zero.
    count: usize,
}

impl ColourRamp {
    /// Stops a single ramp may hold. Five covers the longest curve in use —
    /// white-hot, yellow, orange, deep red, smoke — with nothing to spare on
    /// purpose: a ramp needing more is describing two effects.
    pub const MAX_STOPS: usize = 5;

    /// A ramp built from stops given in ascending `at` order.
    ///
    /// Stops past [`MAX_STOPS`](Self::MAX_STOPS) are dropped rather than
    /// panicking, since a ramp is authored data and a missing shade is a far
    /// better failure than a crash mid-explosion.
    pub fn new(stops: &[ColourStop]) -> Self {
        debug_assert!(!stops.is_empty(), "a ramp needs at least one colour");
        debug_assert!(
            stops.len() <= Self::MAX_STOPS,
            "ramp has {} stops, only {} are kept",
            stops.len(),
            Self::MAX_STOPS
        );
        debug_assert!(
            stops.windows(2).all(|pair| pair[0].at <= pair[1].at),
            "ramp stops must be in ascending order"
        );

        let mut ramp = Self {
            stops: [ColourStop::new(0.0, Vector4::zeros()); Self::MAX_STOPS],
            count: stops.len().clamp(1, Self::MAX_STOPS),
        };
        for (slot, stop) in ramp.stops.iter_mut().zip(stops) {
            *slot = *stop;
        }
        ramp
    }

    /// A ramp that never changes colour.
    pub fn solid(colour: Vector4<f32>) -> Self {
        Self::new(&[ColourStop::new(0.0, colour)])
    }

    /// The familiar two-colour ramp: `from` at birth, `to` at death.
    pub fn fade(from: Vector4<f32>, to: Vector4<f32>) -> Self {
        Self::new(&[ColourStop::new(0.0, from), ColourStop::new(1.0, to)])
    }

    /// The same ramp in another hue: each stop takes `colour`, scaled by how
    /// bright that stop is beside the first, and keeps its alpha. A ramp that
    /// darkens and fades as it ages still does, in the new colour.
    pub fn recoloured(&self, colour: Vector3<f32>) -> Self {
        let luminance = |c: Vector4<f32>| 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
        let base = luminance(self.stops[0].colour).max(f32::EPSILON);
        let mut ramp = *self;
        for stop in &mut ramp.stops[..self.count] {
            let rgb = colour * (luminance(stop.colour) / base);
            stop.colour = Vector4::new(rgb.x, rgb.y, rgb.z, stop.colour.w);
        }
        ramp
    }

    /// The colour at normalised age `t`.
    ///
    /// Ages outside 0..1 clamp to the end stops, so a particle is never left
    /// without a colour.
    pub fn sample(&self, t: f32) -> Vector4<f32> {
        let stops = &self.stops[..self.count];

        let Some(next) = stops.iter().position(|stop| t <= stop.at) else {
            return stops[stops.len() - 1].colour;
        };
        if next == 0 {
            return stops[0].colour;
        }

        let from = stops[next - 1];
        let to = stops[next];
        let span = to.at - from.at;
        if span <= f32::EPSILON {
            return to.colour;
        }
        from.colour.lerp(&to.colour, (t - from.at) / span)
    }
}

impl Default for ColourRamp {
    fn default() -> Self {
        Self::solid(Vector4::new(1.0, 1.0, 1.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Recoloured, a ramp starts in the new colour and keeps its darkening
    /// and its fade.
    #[test]
    fn a_recoloured_ramp_keeps_its_shape() {
        let ramp = ColourRamp::new(&[
            ColourStop::new(0.0, Vector4::new(0.4, 0.4, 0.4, 1.0)),
            ColourStop::new(1.0, Vector4::new(0.2, 0.2, 0.2, 0.0)),
        ])
        .recoloured(Vector3::new(0.2, 0.6, 0.1));
        assert!((ramp.sample(0.0) - Vector4::new(0.2, 0.6, 0.1, 1.0)).norm() < 1e-5);
        assert!((ramp.sample(1.0) - Vector4::new(0.1, 0.3, 0.05, 0.0)).norm() < 1e-5);
    }

    fn grey(value: f32, alpha: f32) -> Vector4<f32> {
        Vector4::new(value, value, value, alpha)
    }

    #[test]
    fn a_solid_ramp_holds_its_colour_throughout() {
        let ramp = ColourRamp::solid(grey(0.5, 1.0));
        for step in 0..=10 {
            assert_eq!(ramp.sample(step as f32 / 10.0), grey(0.5, 1.0));
        }
    }

    #[test]
    fn a_fade_runs_end_to_end_and_passes_through_the_middle() {
        let ramp = ColourRamp::fade(grey(1.0, 1.0), grey(0.0, 0.0));

        assert_eq!(ramp.sample(0.0), grey(1.0, 1.0));
        assert_eq!(ramp.sample(1.0), grey(0.0, 0.0));
        assert_eq!(ramp.sample(0.5), grey(0.5, 0.5));
    }

    #[test]
    fn stops_are_reached_exactly_at_the_age_they_are_authored_for() {
        let ramp = ColourRamp::new(&[
            ColourStop::new(0.0, grey(1.0, 1.0)),
            ColourStop::new(0.1, grey(0.8, 1.0)),
            ColourStop::new(0.9, grey(0.2, 0.5)),
        ]);

        assert_eq!(ramp.sample(0.1), grey(0.8, 1.0));
        assert_eq!(ramp.sample(0.9), grey(0.2, 0.5));

        // Uneven spacing must be honoured: half way through life is most of
        // the way along the long second segment, not the middle of the ramp.
        let middle = ramp.sample(0.5);
        assert!(middle.x < 0.8 && middle.x > 0.2);
    }

    #[test]
    fn ages_outside_the_ramp_clamp_to_its_ends() {
        let ramp = ColourRamp::new(&[
            ColourStop::new(0.2, grey(1.0, 1.0)),
            ColourStop::new(0.8, grey(0.0, 0.0)),
        ]);

        assert_eq!(ramp.sample(-1.0), grey(1.0, 1.0));
        assert_eq!(ramp.sample(0.0), grey(1.0, 1.0));
        assert_eq!(ramp.sample(1.0), grey(0.0, 0.0));
        assert_eq!(ramp.sample(2.0), grey(0.0, 0.0));
    }
}
