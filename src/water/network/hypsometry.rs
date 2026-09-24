//! A basin's hypsometry: how much water it holds at each level, and the level
//! any volume stands at.
//!
//! Each span of the region wets gradually as the level rises through its
//! floor, as though its floor heights were spread evenly between `floor_min`
//! and `floor_max`, and stops adding surface once the level passes its
//! ceiling. So the wetted area is piecewise linear in the level and the
//! volume piecewise quadratic, stored as prefix sums at every breakpoint.
//!
//! A flat floor or a ceiling would make the area jump, and the volume kink;
//! every jump is spread over [`WET_BAND`] instead, so the law is C¹ and the
//! solver's Newton steps see no kinks. Absorbed potholes add their dead
//! volume as a smoothed step at their own rim.

use crate::water::geometry::Span;

/// Height over which a flat floor wets, or a ceiling closes, in metres.
pub const WET_BAND: f32 = 0.01;

/// Half-width of the smoothed step a pothole fills over, in metres (§7.2's δ).
pub const POTHOLE_STEP: f32 = 0.02;

/// A pothole absorbed into a region: it holds `volume` below `rim` and fills
/// as the level passes the rim.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeadStorage {
    pub rim: f32,
    pub volume: f64,
}

/// Volume and wetted area as functions of level, for one region.
#[derive(Debug, Clone, Default)]
pub struct Hypsometry {
    /// Levels where the area's slope changes, ascending.
    breaks: Vec<f64>,
    /// Wetted area at each break, m².
    area: Vec<f64>,
    /// d(area)/d(level) from each break to the next.
    slope: Vec<f64>,
    /// Volume below each break, m³, excluding dead storage.
    volume: Vec<f64>,
    dead: Vec<DeadStorage>,
}

impl Hypsometry {
    /// The hypsometry of a set of spans, each contributing `cell_area` of
    /// surface once wet.
    pub fn new(spans: impl Iterator<Item = Span>, cell_area: f32, dead: Vec<DeadStorage>) -> Self {
        let a = cell_area as f64;
        // (level, slope change) events: each span's area ramps up over its
        // floor band and back down over its ceiling band.
        let mut events: Vec<(f64, f64)> = Vec::new();
        for span in spans {
            let lo = span.floor_min as f64;
            let hi = (span.floor_max as f64).max(lo + WET_BAND as f64);
            let ceiling = span.ceiling as f64;
            if ceiling <= lo {
                continue;
            }
            let rise = a / (hi - lo);
            events.push((lo, rise));
            events.push((hi, -rise));
            if ceiling.is_finite() {
                // The span's area falls to zero over the ceiling band. If the
                // ceiling is inside the floor band, it closes a partial area.
                let open = if ceiling < hi {
                    rise * (ceiling - lo)
                } else {
                    a
                };
                let band = WET_BAND as f64;
                let fall = open / band;
                if ceiling < hi {
                    events.push((ceiling, -rise));
                    events.push((hi, rise));
                }
                events.push((ceiling, -fall));
                events.push((ceiling + band, fall));
            }
        }
        events.sort_unstable_by(|x, y| x.0.total_cmp(&y.0));

        let mut out = Self {
            dead,
            ..Self::default()
        };
        let (mut area, mut slope, mut volume) = (0.0f64, 0.0f64, 0.0f64);
        let mut last = events.first().map_or(0.0, |e| e.0);
        let mut i = 0;
        while i < events.len() {
            let level = events[i].0;
            let dl = level - last;
            volume += area * dl + 0.5 * slope * dl * dl;
            area = (area + slope * dl).max(0.0);
            while i < events.len() && events[i].0 == level {
                slope += events[i].1;
                i += 1;
            }
            out.breaks.push(level);
            out.area.push(area);
            out.slope.push(slope);
            out.volume.push(volume);
            last = level;
        }
        out
    }

    /// The lowest level any water stands at.
    pub fn bottom(&self) -> f32 {
        self.breaks.first().copied().unwrap_or(0.0) as f32
    }

    /// Volume held with the surface at `level`, m³.
    pub fn volume(&self, level: f32) -> f64 {
        let l = level as f64;
        let smooth: f64 = self.dead.iter().map(|d| d.volume * step(l, d.rim)).sum();
        self.live_volume(l) + smooth
    }

    /// Wetted surface area at `level`, m²: dV/dL.
    pub fn area(&self, level: f32) -> f64 {
        let l = level as f64;
        let smooth: f64 = self
            .dead
            .iter()
            .map(|d| d.volume * step_slope(l, d.rim))
            .sum();
        self.live_area(l) + smooth
    }

    /// The level `volume` stands at.
    pub fn level(&self, volume: f64) -> f32 {
        if self.breaks.is_empty() {
            return 0.0;
        }
        if volume <= 0.0 {
            return self.bottom();
        }
        if let Some(capacity) = self.capacity() {
            if volume >= capacity {
                return self.top() as f32;
            }
        }
        // Bracket, then safeguarded Newton: the volume rises monotonically.
        let mut lo = self.breaks[0];
        let mut hi = self.top().max(lo + 1.0);
        while self.volume(hi as f32) < volume {
            hi = lo + (hi - lo) * 2.0;
        }
        let mut l = 0.5 * (lo + hi);
        for _ in 0..60 {
            let f = self.volume(l as f32) - volume;
            if f.abs() <= 1e-9 * volume.max(1.0) {
                break;
            }
            if f > 0.0 {
                hi = l;
            } else {
                lo = l;
            }
            let area = self.area(l as f32);
            let newton = l - f / area.max(1e-12);
            l = if newton > lo && newton < hi {
                newton
            } else {
                0.5 * (lo + hi)
            };
            if hi - lo < 1e-7 {
                break;
            }
        }
        l as f32
    }

    /// The most the region can hold, if every column of it is capped by a
    /// ceiling: a sealed pocket. `None` if water can always rise further.
    pub fn capacity(&self) -> Option<f64> {
        let last = self.area.len().checked_sub(1)?;
        (self.area[last] <= 1e-9 && self.slope[last].abs() <= 1e-9)
            .then(|| self.volume[last] + self.dead.iter().map(|d| d.volume).sum::<f64>())
    }

    fn top(&self) -> f64 {
        self.breaks.last().copied().unwrap_or(0.0)
    }

    fn segment(&self, level: f64) -> Option<usize> {
        if self.breaks.is_empty() || level <= self.breaks[0] {
            return None;
        }
        Some(self.breaks.partition_point(|b| *b <= level) - 1)
    }

    fn live_volume(&self, level: f64) -> f64 {
        let Some(k) = self.segment(level) else {
            return 0.0;
        };
        let dl = level - self.breaks[k];
        self.volume[k] + self.area[k] * dl + 0.5 * self.slope[k] * dl * dl
    }

    fn live_area(&self, level: f64) -> f64 {
        let Some(k) = self.segment(level) else {
            return 0.0;
        };
        (self.area[k] + self.slope[k] * (level - self.breaks[k])).max(0.0)
    }
}

/// A smoothstep from 0 to 1 across `rim ± POTHOLE_STEP`.
fn step(level: f64, rim: f32) -> f64 {
    let d = POTHOLE_STEP as f64;
    let t = ((level - (rim as f64 - d)) / (2.0 * d)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// d(step)/d(level).
fn step_slope(level: f64, rim: f32) -> f64 {
    let d = POTHOLE_STEP as f64;
    let t = (level - (rim as f64 - d)) / (2.0 * d);
    if !(0.0..=1.0).contains(&t) {
        return 0.0;
    }
    6.0 * t * (1.0 - t) / (2.0 * d)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span(floor_min: f32, floor_max: f32, ceiling: f32) -> Span {
        Span {
            floor_c: floor_min,
            floor_min,
            floor_max,
            ceiling,
        }
    }

    #[test]
    fn a_flat_square_holds_area_times_depth() {
        let h = Hypsometry::new(
            (0..100).map(|_| span(0.0, 0.0, f32::INFINITY)),
            0.25,
            Vec::new(),
        );
        // 100 columns of 0.25 m²: 25 m². A metre of water is 25 m³, less the
        // half of the wet band that is not yet full.
        let v = h.volume(1.0);
        assert!((v - 25.0).abs() < 0.2, "{v}");
        assert!((h.level(v) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn a_sloped_column_wets_gradually() {
        let h = Hypsometry::new([span(0.0, 1.0, f32::INFINITY)].into_iter(), 1.0, Vec::new());
        assert!((h.area(0.5) - 0.5).abs() < 1e-6);
        // ∫₀^½ L dL = 1/8.
        assert!((h.volume(0.5) - 0.125).abs() < 1e-6);
        assert!((h.level(0.125) - 0.5).abs() < 1e-5);
    }

    #[test]
    fn a_sealed_pocket_has_a_capacity() {
        let h = Hypsometry::new((0..4).map(|_| span(0.0, 0.0, 2.0)), 1.0, Vec::new());
        let capacity = h.capacity().expect("sealed");
        assert!((capacity - 8.0).abs() < 0.1, "{capacity}");
        assert!(h.level(capacity * 2.0) >= 2.0);
    }

    #[test]
    fn a_pothole_fills_at_its_rim() {
        let dead = vec![DeadStorage {
            rim: 1.0,
            volume: 0.5,
        }];
        let h = Hypsometry::new((0..4).map(|_| span(0.0, 0.0, f32::INFINITY)), 1.0, dead);
        let below = h.volume(0.9);
        let above = h.volume(1.1);
        assert!((above - below - 0.8 - 0.5).abs() < 1e-3, "{below} {above}");
        // The level is continuous through the step.
        let mut last = h.level(below);
        let mut v = below;
        while v < above {
            v += 0.01;
            let l = h.level(v);
            assert!(l >= last - 1e-5);
            last = l;
        }
    }
}
