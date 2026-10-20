//! Wind exposure: how long the wind's waves can grow over each patch of water.
//!
//! Wind waves grow over their fetch, the stretch of open water the wind has
//! blown across to reach them: a pond's only ever ripple, the sea's run to
//! metres. Under a roof there is no wind at all. This map holds, per metre of
//! the field, the peak wavelength of the wind sea there, which the chop
//! (`shader/chop.glsl`) takes as the longest wave it draws.
//!
//! ```text
//!   field cells ─▶ open / roofed / dry, per texel (the top wet cell of each)
//!               ─▶ fetch along 8 directions (a run of open water, the sea's
//!                  past an open map edge)
//!               ─▶ peak wavelength of each fetch, averaged ─▶ blurred
//! ```
//!
//! The fetch is taken from every direction, not from one wind's: water open to
//! the sea on any side reads as the sea's, and a long narrow lake neither as
//! calm as across it nor as rough as along it. The blur fades the waves out
//! over a few metres where open water runs under a roof, rather than at a line.

use crate::water::field::{WaterField, WET_DEPTH};
use crate::water::geometry::{Column, CHUNK_COLUMNS, COLUMN_SIZE};

/// Columns along each side of a texel: one texel is 1 m.
const COLUMNS_PER_TEXEL: i32 = 2;

/// Side of a texel, m.
pub const TEXEL_SIZE: f32 = COLUMNS_PER_TEXEL as f32 * COLUMN_SIZE;

/// Longest fetch counted, m: past this the wind sea is fully grown for the
/// waves the chop draws.
pub const FETCH_CAP: f32 = 2000.0;

/// Peak wavelength of the wind sea over 100 m of fetch, m, under the breeze
/// the chop is drawn for (about 5 m/s). From the fetch-limited growth of
/// JONSWAP, ω_p = 22 (g² / (U F))^(1/3), so the peak wavelength grows as
/// F^(2/3): about 9 cm over a 10 m pond, 1.8 m over a kilometre.
const PEAK_AT_100M: f32 = 0.38;

/// Texels the map is blurred over, each way.
const BLUR_RADIUS: i32 = 2;

/// The eight directions fetch is measured along, as texel steps.
const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (-1, -1),
    (1, -1),
    (-1, 1),
];

/// The wind sea's peak wavelength over a fetch, m.
pub fn peak_wavelength(fetch: f32) -> f32 {
    PEAK_AT_100M * (fetch.clamp(0.0, FETCH_CAP) / 100.0).powf(2.0 / 3.0)
}

/// What a texel's water is to the wind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Cover {
    /// No terrain under it at all: past the map's edge, where the sea runs
    /// on if there is one.
    Void,
    Dry,
    Open,
    Roofed,
}

/// The peak wavelength of the wind sea over the field, per texel.
#[derive(Debug, Clone, PartialEq)]
pub struct WindExposure {
    /// World (x, z) of texel (0, 0)'s minimum corner.
    pub origin: (f32, f32),
    pub width: usize,
    pub depth: usize,
    /// Peak wavelength, m, row-major (x fastest).
    pub peak: Vec<f32>,
    /// Peak wavelength past the map's edges, m: the open sea's where the
    /// field opens onto one, else none.
    pub beyond: f32,
    /// Moves whenever `peak` does.
    pub version: u32,
}

impl Default for WindExposure {
    fn default() -> Self {
        Self {
            origin: (0.0, 0.0),
            width: 0,
            depth: 0,
            peak: Vec::new(),
            beyond: 0.0,
            version: 0,
        }
    }
}

impl WindExposure {
    /// The exposure of the field's water as it stands, `sea` if its open map
    /// edges look out over the sea.
    pub fn of(field: &WaterField, sea: bool) -> Self {
        let mut exposure = Self::default();
        exposure.refresh(field, sea);
        exposure
    }

    /// Measure the field's water again; the version moves only if the map
    /// changed.
    pub fn refresh(&mut self, field: &WaterField, sea: bool) {
        let (min, max) = field.graph().column_bounds();
        let width = ((max.i - min.i + 1) / COLUMNS_PER_TEXEL).max(0) as usize;
        let depth = ((max.k - min.k + 1) / COLUMNS_PER_TEXEL).max(0) as usize;
        let cover = covers(field, min, width, depth);
        let edge_fetch = if sea { FETCH_CAP } else { 0.0 };
        let peak = blur(&mean_peak(&cover, width, depth, edge_fetch), width, depth);
        let origin = min.min_corner();
        let beyond = if sea { peak_wavelength(FETCH_CAP) } else { 0.0 };
        if (origin, width, depth, beyond) != (self.origin, self.width, self.depth, self.beyond)
            || peak != self.peak
        {
            *self = Self {
                origin,
                width,
                depth,
                peak,
                beyond,
                version: self.version.wrapping_add(1),
            };
        }
    }
}

/// Each texel's cover, from the top wet cell of the columns in it: open if
/// any of them stands under open sky, else roofed if any is wet.
fn covers(field: &WaterField, min: Column, width: usize, depth: usize) -> Vec<Cover> {
    let mut cover = vec![Cover::Void; width * depth];
    let cells = field.cells();
    let graph = field.graph();
    for (slot, coord) in graph.chunk_coords().enumerate() {
        let chunk = &cells.chunks[slot];
        for local in 0..(CHUNK_COLUMNS * CHUNK_COLUMNS) as usize {
            let (start, end) = (
                chunk.offsets[local] as usize,
                chunk.offsets[local + 1] as usize,
            );
            if start == end {
                continue;
            }
            let i = coord.x * CHUNK_COLUMNS + local as i32 % CHUNK_COLUMNS - min.i;
            let k = coord.z * CHUNK_COLUMNS + local as i32 / CHUNK_COLUMNS - min.k;
            let (x, z) = (
                (i / COLUMNS_PER_TEXEL) as usize,
                (k / COLUMNS_PER_TEXEL) as usize,
            );
            if x >= width || z >= depth {
                continue;
            }
            let here = match (start..end).rev().find(|&c| chunk.depth[c] > WET_DEPTH) {
                None => Cover::Dry,
                Some(top) if chunk.ceiling[top].is_finite() => Cover::Roofed,
                Some(_) => Cover::Open,
            };
            // Open water anywhere in the texel makes it open; else any water
            // makes it roofed; else any terrain makes it dry.
            let rank = |c: Cover| match c {
                Cover::Void => 0,
                Cover::Dry => 1,
                Cover::Roofed => 2,
                Cover::Open => 3,
            };
            let texel = &mut cover[z * width + x];
            if rank(here) > rank(*texel) {
                *texel = here;
            }
        }
    }
    cover
}

/// Each open texel's peak wavelength, averaged over the fetch along every
/// direction; zero over dry and roofed texels, and over the void that of
/// `edge_fetch`. Fetch along a direction is the run of open texels behind a
/// texel, so it is found in one sweep per direction, each texel extending the
/// run of the one behind it. A run reaching back to the void, or past the
/// map's edge, starts there with `edge_fetch`: the sea's, if the field opens
/// onto one.
fn mean_peak(cover: &[Cover], width: usize, depth: usize, edge_fetch: f32) -> Vec<f32> {
    let mut sum = vec![0.0f32; cover.len()];
    let mut run = vec![0.0f32; cover.len()];
    for &(dx, dz) in &DIRECTIONS {
        let step = TEXEL_SIZE * ((dx * dx + dz * dz) as f32).sqrt();
        // Visit texels so the one behind each, at (x - dx, z - dz), comes
        // first.
        let xs: Vec<usize> = if dx >= 0 {
            (0..width).collect()
        } else {
            (0..width).rev().collect()
        };
        let zs: Vec<usize> = if dz >= 0 {
            (0..depth).collect()
        } else {
            (0..depth).rev().collect()
        };
        for &z in &zs {
            for &x in &xs {
                let here = z * width + x;
                match cover[here] {
                    Cover::Open => {}
                    Cover::Void => {
                        run[here] = edge_fetch;
                        sum[here] += peak_wavelength(edge_fetch);
                        continue;
                    }
                    Cover::Dry | Cover::Roofed => {
                        run[here] = 0.0;
                        continue;
                    }
                }
                let behind = (x as i32 - dx, z as i32 - dz);
                let inside = behind.0 >= 0
                    && behind.1 >= 0
                    && (behind.0 as usize) < width
                    && (behind.1 as usize) < depth;
                let before = if inside {
                    run[behind.1 as usize * width + behind.0 as usize]
                } else {
                    edge_fetch
                };
                run[here] = (before + step).min(FETCH_CAP);
                sum[here] += peak_wavelength(run[here]);
            }
        }
    }
    let directions = DIRECTIONS.len() as f32;
    sum.iter_mut().for_each(|s| *s /= directions);
    sum
}

/// A box blur over `BLUR_RADIUS` texels each way, one axis at a time.
fn blur(map: &[f32], width: usize, depth: usize) -> Vec<f32> {
    let along = |map: &[f32], stride: usize, count: usize, lines: usize, line_stride: usize| {
        let mut out = vec![0.0f32; map.len()];
        for line in 0..lines {
            for n in 0..count {
                let lo = n.saturating_sub(BLUR_RADIUS as usize);
                let hi = (n + BLUR_RADIUS as usize).min(count - 1);
                let total: f32 = (lo..=hi)
                    .map(|m| map[line * line_stride + m * stride])
                    .sum();
                out[line * line_stride + n * stride] = total / (hi - lo + 1) as f32;
            }
        }
        out
    };
    if width == 0 || depth == 0 {
        return map.to_vec();
    }
    let rows = along(map, 1, width, depth, width);
    along(&rows, width, depth, width, 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pond(width: usize, depth: usize, open: impl Fn(usize, usize) -> Cover) -> Vec<Cover> {
        (0..depth)
            .flat_map(|z| (0..width).map(move |x| (x, z)))
            .map(|(x, z)| open(x, z))
            .collect()
    }

    #[test]
    fn the_peak_wavelength_grows_with_fetch_as_jonswap_has_it() {
        assert!((peak_wavelength(10.0) - 0.082).abs() < 0.005);
        assert!((peak_wavelength(100.0) - 0.38).abs() < 1e-6);
        assert!((peak_wavelength(1000.0) - 1.76).abs() < 0.02);
        assert_eq!(peak_wavelength(1.0e6), peak_wavelength(FETCH_CAP));
    }

    #[test]
    fn a_bigger_pond_grows_longer_waves() {
        let small = mean_peak(&pond(8, 8, |_, _| Cover::Open), 8, 8, 0.0);
        let large = mean_peak(&pond(64, 64, |_, _| Cover::Open), 64, 64, 0.0);
        assert!(large[32 * 64 + 32] > 2.0 * small[4 * 8 + 4]);
    }

    #[test]
    fn the_sea_runs_on_past_the_void_at_the_maps_edge() {
        let cover = pond(16, 1, |x, _| if x < 4 { Cover::Void } else { Cover::Open });
        let sea = mean_peak(&cover, 16, 1, FETCH_CAP);
        assert!(sea[4] > 0.5 * peak_wavelength(FETCH_CAP));
        let no_sea = mean_peak(&cover, 16, 1, 0.0);
        assert!(no_sea[4] < 0.1);
    }

    #[test]
    fn roofed_and_dry_water_has_no_wind_sea() {
        let cover = pond(8, 1, |x, _| match x {
            0..=3 => Cover::Open,
            4..=5 => Cover::Roofed,
            _ => Cover::Dry,
        });
        let peak = mean_peak(&cover, 8, 1, FETCH_CAP);
        assert!(peak[3] > 0.0);
        assert!(peak[4..].iter().all(|&p| p == 0.0));
    }

    #[test]
    fn water_open_to_the_sea_takes_the_seas_waves() {
        let cover = pond(16, 16, |_, _| Cover::Open);
        let sea = mean_peak(&cover, 16, 16, FETCH_CAP);
        let pond = mean_peak(&cover, 16, 16, 0.0);
        assert!(sea[8 * 16 + 8] > 5.0 * pond[8 * 16 + 8]);
    }

    #[test]
    fn the_blur_fades_a_step_over_its_radius() {
        let step: Vec<f32> = (0..16).map(|x| if x < 8 { 1.0 } else { 0.0 }).collect();
        let blurred = blur(&step, 16, 1);
        assert_eq!(blurred[0], 1.0);
        assert!(blurred[7] < 1.0 && blurred[8] > 0.0);
        assert_eq!(blurred[15], 0.0);
    }
}
