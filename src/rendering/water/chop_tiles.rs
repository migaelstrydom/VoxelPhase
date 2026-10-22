//! The chop's wave tiles: the wind waves on every body of water, as a few
//! hundred random waves per octave of wavelength, summed by an FFT each frame.
//!
//! A sum of a dozen sines reads as a lattice wherever two or three of them
//! dominate, however their wavelengths are chosen; real water has thousands
//! of waves, so none do. Each band here is a square tile of the slope field
//! of every wave whose wavelength falls in its octave, at random amplitude and
//! phase about the wind, each travelling at its deep-water speed. The tiles
//! wrap, and the shader (`chop.glsl`) sums the bands a pixel's water raises:
//! a pond only the short ones, the open sea all.
//!
//! ```text
//!   seeded amplitudes per band ─► at time t: phases advance by ω = √(g k)
//!     ─► inverse FFT (both slopes at once) ─► tiles ─► storage buffer
//! ```

use std::f32::consts::TAU;

use nalgebra::{Complex, Vector2};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rayon::prelude::*;

/// Octave bands of wavelength, longest first.
pub const CHOP_BANDS: usize = 4;

/// Texels along each side of a band's tile.
pub const CHOP_TILE_TEXELS: usize = 128;

/// The longest wave of the longest band, m: the chop of the open sea. Each
/// band below it runs from half the one above.
pub const CHOP_LONGEST: f32 = 3.3;

/// Each band's tile side, in its longest wavelengths. Unrelated, so that the
/// bands' tiles do not repeat together.
const TILE_SPANS: [f32; CHOP_BANDS] = [8.0, 8.9, 7.3, 8.4];

/// The wind's heading over the water. Mirrors the drift in `chop.glsl`.
const WIND: Vector2<f32> = Vector2::new(0.921, 0.390);

/// How narrowly the waves keep to the wind: power of the cosine of their
/// heading off it.
const SPREAD_POWER: i32 = 4;

/// Share of the energy of waves across the wind left in waves running
/// against it, which a real sea all but lacks; none would let opposed waves
/// stand still.
const AGAINST_WIND: f32 = 0.02;

/// Gravity, m/s².
const GRAVITY: f32 = 9.81;

/// Seed of the bands' amplitudes and phases.
const SEED: u64 = 7;

/// Every band's tile at one time.
pub struct ChopTiles {
    bands: Vec<Band>,
    fft: Fft,
    /// When `slopes` was last summed, s.
    time: Option<f32>,
    /// Each band's tile in turn, row-major, both slopes per texel.
    slopes: Vec<[f32; 2]>,
}

/// One octave of the chop's waves.
struct Band {
    /// Tile side, m.
    span: f32,
    /// Its longest wavelength, m.
    longest: f32,
    /// Per texel of the spectrum, the wave there: its wavevector, 1/m, its
    /// angular frequency, rad/s, and its complex amplitude at time zero,
    /// zero outside the band.
    waves: Vec<(Vector2<f32>, f32, Complex<f32>)>,
    /// The spectrum at the frame's time, then the slopes it sums to.
    work: Vec<Complex<f32>>,
}

impl ChopTiles {
    pub fn new() -> Self {
        let fft = Fft::new(CHOP_TILE_TEXELS);
        let mut rng = StdRng::seed_from_u64(SEED);
        let bands = (0..CHOP_BANDS)
            .map(|b| {
                let longest = CHOP_LONGEST / (1 << b) as f32;
                Band::new(longest, longest * TILE_SPANS[b], &mut rng, &fft)
            })
            .collect();
        Self {
            bands,
            fft,
            time: None,
            slopes: vec![[0.0; 2]; CHOP_BANDS * CHOP_TILE_TEXELS * CHOP_TILE_TEXELS],
        }
    }

    /// Each band's tile side, m.
    pub fn spans(&self) -> [f32; CHOP_BANDS] {
        std::array::from_fn(|b| self.bands[b].span)
    }

    /// Each band's longest wavelength, m.
    pub fn longest(&self) -> [f32; CHOP_BANDS] {
        std::array::from_fn(|b| self.bands[b].longest)
    }

    /// Every band's slopes at `time`, s: each band's tile in turn, row-major,
    /// with an rms slope of one.
    pub fn at(&mut self, time: f32) -> &[[f32; 2]] {
        if self.time != Some(time) {
            let fft = &self.fft;
            self.bands
                .par_iter_mut()
                .zip(
                    self.slopes
                        .par_chunks_mut(CHOP_TILE_TEXELS * CHOP_TILE_TEXELS),
                )
                .for_each(|(band, out)| band.sum(time, fft, out));
            self.time = Some(time);
        }
        &self.slopes
    }
}

impl Default for ChopTiles {
    fn default() -> Self {
        Self::new()
    }
}

impl Band {
    fn new(longest: f32, span: f32, rng: &mut StdRng, fft: &Fft) -> Self {
        let n = CHOP_TILE_TEXELS;
        let (k_min, k_max) = (TAU / longest, 2.0 * TAU / longest);
        let mut waves = Vec::with_capacity(n * n);
        for row in 0..n {
            for col in 0..n {
                let k = Vector2::new(signed(col, n), signed(row, n)) * (TAU / span);
                let length = k.norm();
                let amplitude = if length >= k_min && length < k_max {
                    let along = k.dot(&WIND) / length;
                    let spread = along.max(0.0).powi(SPREAD_POWER) + AGAINST_WIND;
                    // Equal slope per octave: as many waves in an annulus
                    // as its area, so each one's slope variance ∝ 1/k².
                    let height = spread.sqrt() / (length * length);
                    Complex::new(gaussian(rng), gaussian(rng)) * height
                } else {
                    Complex::new(0.0, 0.0)
                };
                waves.push((k, (GRAVITY * length).sqrt(), amplitude));
            }
        }
        let mut band = Self {
            span,
            longest,
            waves,
            work: vec![Complex::new(0.0, 0.0); n * n],
        };
        // Scale the amplitudes to an rms slope of one.
        let mut out = vec![[0.0; 2]; n * n];
        band.sum(0.0, fft, &mut out);
        let rms = (out.iter().map(|s| s[0] * s[0] + s[1] * s[1]).sum::<f32>() / (n * n) as f32)
            .sqrt()
            .max(1.0e-12);
        for wave in &mut band.waves {
            wave.2 /= rms;
        }
        band
    }

    /// Its slopes at `time` into `out`.
    fn sum(&mut self, time: f32, fft: &Fft, out: &mut [[f32; 2]]) {
        let n = CHOP_TILE_TEXELS;
        for row in 0..n {
            for col in 0..n {
                let (k, omega, a) = self.waves[row * n + col];
                if k == Vector2::zeros() {
                    self.work[row * n + col] = Complex::new(0.0, 0.0);
                    continue;
                }
                // Each wave and its mirror, so the surface is real.
                let mirror = self.waves[((n - row) % n) * n + (n - col) % n].2;
                let phase = ((omega as f64 * time as f64) % std::f64::consts::TAU) as f32;
                let turn = Complex::new(phase.cos(), -phase.sin());
                let height = a * turn + mirror.conj() * turn.conj();
                // Both slopes in one transform: x real, z imaginary.
                let i = Complex::new(0.0, 1.0);
                self.work[row * n + col] = i * height * k.x - height * k.y;
            }
        }
        fft.inverse_2d(&mut self.work);
        for (o, w) in out.iter_mut().zip(&self.work) {
            *o = [w.re, w.im];
        }
    }
}

/// Index `i` of `n` as a signed frequency, -n/2 ..< n/2.
fn signed(i: usize, n: usize) -> f32 {
    if i < n / 2 {
        i as f32
    } else {
        i as f32 - n as f32
    }
}

/// A standard normal draw.
fn gaussian(rng: &mut StdRng) -> f32 {
    let u: f32 = rng.gen_range(1.0e-7..1.0);
    let v: f32 = rng.gen();
    (-2.0 * u.ln()).sqrt() * (TAU * v).cos()
}

/// A radix-2 complex FFT of one power-of-two length.
struct Fft {
    n: usize,
    /// e^(+2πi j/n) for j < n/2.
    twiddles: Vec<Complex<f32>>,
}

impl Fft {
    fn new(n: usize) -> Self {
        assert!(n.is_power_of_two());
        let twiddles = (0..n / 2)
            .map(|j| {
                let a = TAU * j as f32 / n as f32;
                Complex::new(a.cos(), a.sin())
            })
            .collect();
        Self { n, twiddles }
    }

    /// The unnormalised inverse transform of `data` in place.
    fn inverse(&self, data: &mut [Complex<f32>]) {
        let n = self.n;
        let bits = n.trailing_zeros();
        for i in 0..n {
            let j = i.reverse_bits() >> (usize::BITS - bits);
            if j > i {
                data.swap(i, j);
            }
        }
        let mut size = 2;
        while size <= n {
            let half = size / 2;
            let step = n / size;
            for start in (0..n).step_by(size) {
                for j in 0..half {
                    let t = self.twiddles[j * step] * data[start + j + half];
                    let u = data[start + j];
                    data[start + j] = u + t;
                    data[start + j + half] = u - t;
                }
            }
            size *= 2;
        }
    }

    /// The unnormalised inverse transform of an n×n row-major grid in place.
    fn inverse_2d(&self, data: &mut [Complex<f32>]) {
        let n = self.n;
        for row in data.chunks_mut(n) {
            self.inverse(row);
        }
        let mut column = vec![Complex::new(0.0, 0.0); n];
        for col in 0..n {
            for row in 0..n {
                column[row] = data[row * n + col];
            }
            self.inverse(&mut column);
            for row in 0..n {
                data[row * n + col] = column[row];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One wave on the grid sums to a cosine of unit amplitude per n² and the
    /// right wavenumber.
    #[test]
    fn the_transform_of_one_mode_is_its_wave() {
        let n = 16;
        let fft = Fft::new(n);
        let mut data = vec![Complex::new(0.0, 0.0); n * n];
        data[3] = Complex::new(1.0, 0.0);
        fft.inverse_2d(&mut data);
        for col in 0..n {
            let expected = (TAU * 3.0 * col as f32 / n as f32).cos();
            assert!((data[5 * n + col].re - expected).abs() < 1e-5);
        }
    }

    #[test]
    fn every_band_has_unit_rms_slope() {
        let mut tiles = ChopTiles::new();
        let n = CHOP_TILE_TEXELS;
        let slopes = tiles.at(12.5).to_vec();
        for band in slopes.chunks(n * n) {
            let rms = (band.iter().map(|s| s[0] * s[0] + s[1] * s[1]).sum::<f32>()
                / (n * n) as f32)
                .sqrt();
            assert!((rms - 1.0).abs() < 0.25, "rms slope {rms}");
        }
    }

    /// The waves run downwind: their slope pattern moves along the wind.
    #[test]
    fn the_waves_run_downwind() {
        let mut tiles = ChopTiles::new();
        let n = CHOP_TILE_TEXELS;
        let before = tiles.at(0.0)[..n * n].to_vec();
        let after = tiles.at(0.1)[..n * n].to_vec();
        // Correlate `after` against `before` shifted a texel downwind and upwind.
        let shifted = |dx: i32, dz: i32| -> f32 {
            let mut sum = 0.0;
            for row in 0..n {
                for col in 0..n {
                    let r = (row as i32 - dz).rem_euclid(n as i32) as usize;
                    let c = (col as i32 - dx).rem_euclid(n as i32) as usize;
                    sum += after[row * n + col][0] * before[r * n + c][0];
                }
            }
            sum
        };
        assert!(shifted(1, 0) > shifted(-1, 0));
    }
}
