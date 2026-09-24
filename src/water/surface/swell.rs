//! Swell: the analytic long waves on every body of still water.
//!
//! A vertical sum of a few sines, not Gerstner waves: Gerstner displaces
//! horizontally, so the CPU would have to invert it to find the height over
//! a point, and at these amplitudes the two look the same. The sum is
//! evaluated identically on the CPU (buoyancy) and the GPU (the surface), so
//! a float bobs on exactly the wave that is drawn. The spectrum below is
//! mirrored in `shader/swell.glsl`, and a test holds the two together.
//!
//! Each body scales the shared spectrum by its own amplitude, which grows
//! with fetch (√area), and offsets its phase so neighbouring ponds do not
//! heave in step. Near a shore the swell fades out over the last metre of
//! depth.

use nalgebra::Vector2;

/// Gravity for the dispersion relation, m/s².
const GRAVITY: f32 = 9.81;

/// One component of the spectrum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwellWave {
    /// Direction of travel, radians from +x towards +z.
    pub heading: f32,
    pub wavelength: f32,
    /// Share of the body's amplitude.
    pub share: f32,
}

/// The shared spectrum. Mirrored in `shader/swell.glsl`.
pub const SPECTRUM: [SwellWave; 5] = [
    SwellWave {
        heading: 0.0,
        wavelength: 7.3,
        share: 0.45,
    },
    SwellWave {
        heading: 0.7,
        wavelength: 4.1,
        share: 0.25,
    },
    SwellWave {
        heading: -0.9,
        wavelength: 2.6,
        share: 0.15,
    },
    SwellWave {
        heading: 1.9,
        wavelength: 1.7,
        share: 0.10,
    },
    SwellWave {
        heading: -2.4,
        wavelength: 1.1,
        share: 0.05,
    },
];

/// Swell amplitude per metre of fetch (√area).
pub const AMPLITUDE_PER_FETCH: f32 = 0.0015;

/// The largest amplitude a basin's swell reaches, m.
pub const MAX_BASIN_AMPLITUDE: f32 = 0.15;

/// Depth over which swell fades out towards a shore, m.
pub const SHORE_FADE: f32 = 1.0;

/// One body's swell.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Swell {
    /// Height of the whole spectrum, m.
    pub amplitude: f32,
    /// Phase offset, radians, so bodies do not heave in step.
    pub phase: f32,
}

impl Swell {
    /// The swell of a basin with this wetted area.
    pub fn for_basin(area: f64, seed: u32) -> Self {
        let fetch = (area as f32).max(0.0).sqrt();
        Self {
            amplitude: (AMPLITUDE_PER_FETCH * fetch).min(MAX_BASIN_AMPLITUDE),
            phase: phase_for(seed),
        }
    }

    /// Height above the level at (x, z), time `t`, over water `depth` deep.
    pub fn height(&self, x: f32, z: f32, t: f32, depth: f32) -> f32 {
        if self.amplitude <= 0.0 {
            return 0.0;
        }
        let mut h = 0.0;
        for wave in &SPECTRUM {
            let (k, omega, d) = wave.terms();
            h += wave.share * (k * (d.x * x + d.y * z) - omega * t + self.phase).sin();
        }
        self.amplitude * h * shore_fade(depth)
    }

    /// d(height)/dx and d(height)/dz at (x, z), time `t`, ignoring how the
    /// shore fade itself varies.
    pub fn gradient(&self, x: f32, z: f32, t: f32, depth: f32) -> Vector2<f32> {
        if self.amplitude <= 0.0 {
            return Vector2::zeros();
        }
        let mut g = Vector2::zeros();
        for wave in &SPECTRUM {
            let (k, omega, d) = wave.terms();
            let c = (k * (d.x * x + d.y * z) - omega * t + self.phase).cos();
            g += d * (wave.share * k * c);
        }
        g * (self.amplitude * shore_fade(depth))
    }
}

impl SwellWave {
    /// Wavenumber, angular frequency by deep-water dispersion, direction.
    fn terms(&self) -> (f32, f32, Vector2<f32>) {
        let k = std::f32::consts::TAU / self.wavelength;
        let omega = (GRAVITY * k).sqrt();
        (
            k,
            omega,
            Vector2::new(self.heading.cos(), self.heading.sin()),
        )
    }
}

/// smoothstep(0, SHORE_FADE, depth).
pub fn shore_fade(depth: f32) -> f32 {
    let t = (depth / SHORE_FADE).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A phase in [0, 2π) from a seed, spread by the golden ratio.
fn phase_for(seed: u32) -> f32 {
    (seed as f32 * 0.618_034).fract() * std::f32::consts::TAU
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gradient_is_the_derivative_of_the_height() {
        let swell = Swell {
            amplitude: 0.1,
            phase: 0.3,
        };
        let (x, z, t, depth) = (1.7, -2.2, 3.1, 5.0);
        let e = 1e-3;
        let g = swell.gradient(x, z, t, depth);
        let dx = (swell.height(x + e, z, t, depth) - swell.height(x - e, z, t, depth)) / (2.0 * e);
        let dz = (swell.height(x, z + e, t, depth) - swell.height(x, z - e, t, depth)) / (2.0 * e);
        assert!(
            (g.x - dx).abs() < 1e-3 && (g.y - dz).abs() < 1e-3,
            "{g:?} vs ({dx}, {dz})"
        );
    }

    #[test]
    fn swell_fades_out_at_the_shore() {
        let swell = Swell {
            amplitude: 0.1,
            phase: 0.0,
        };
        assert_eq!(swell.height(0.3, 0.2, 1.0, 0.0), 0.0);
        let deep = (0..50)
            .map(|i| swell.height(i as f32 * 0.37, 0.0, 0.0, 3.0).abs())
            .fold(0.0f32, f32::max);
        assert!(deep > 0.03);
    }

    #[test]
    fn a_bigger_basin_swells_more_up_to_a_limit() {
        let small = Swell::for_basin(100.0, 1).amplitude;
        let large = Swell::for_basin(4000.0, 1).amplitude;
        let huge = Swell::for_basin(1.0e8, 1).amplitude;
        assert!(small < large && large <= huge && huge == MAX_BASIN_AMPLITUDE);
    }

    /// The shader's spectrum must be this one, wave for wave.
    #[test]
    fn the_shader_spectrum_matches() {
        let glsl =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/shader/swell.glsl"))
                .expect("shader/swell.glsl");
        for wave in &SPECTRUM {
            let line = format!(
                "SwellWave({:.1}, {:.1}, {:.2})",
                wave.heading, wave.wavelength, wave.share
            );
            assert!(glsl.contains(&line), "swell.glsl lacks {line}");
        }
        assert!(glsl.contains(&format!("SWELL_WAVES = {}", SPECTRUM.len())));
    }
}
