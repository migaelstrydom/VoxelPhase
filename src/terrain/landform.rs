//! Landforms: heightfield shapes with natural outlines.
//!
//! Each shape is set in absolute heights and moves the ground one way only: a
//! [`Landmass`] raises it, a [`Channel`] or [`Basin`] lowers it. Order in the
//! feature list then reads like union and carve, and a later shape never
//! undoes an earlier one by accident:
//!
//! ```text
//!   Landmass (low, wide)  ── beaches and lowland
//!   Landmass (high, small) ── a massif standing on it, cliffs where `rise` is short
//!   Basin                 ── a tarn in its top
//!   Channel               ── the stream out of the tarn, falling where its bed steps
//! ```
//!
//! Outlines are kept from looking drawn by a domain warp: the sample point is
//! moved by smooth noise before its distance to the outline is measured, so a
//! straight edge wanders by up to `warp` metres.

use super::generation::point_to_segment_projection;
use crate::utils::noise::fbm_2d_periodic;

/// Wavelength of the warp's coarsest octave, in metres.
const WARP_WAVELENGTH: f32 = 12.0;
/// Wavelength of a landmass's relief's coarsest octave, in metres.
const RELIEF_WAVELENGTH: f32 = 16.0;
/// Seed offset between the two warp axes, so they move independently.
const WARP_AXIS_SEED: u32 = 101;
/// Seed offset of the relief from the warp.
const RELIEF_SEED: u32 = 211;

/// Land rising inside an outline. See `TerrainFeature::Landmass`.
pub struct Landmass<'a> {
    pub outline: &'a [(f32, f32)],
    pub shore_height: f32,
    pub height: f32,
    pub rise: f32,
    pub skirt: f32,
    pub warp: f32,
    pub relief: f32,
    pub seed: u32,
}

impl Landmass<'_> {
    /// Height to add at (x, z) over the ground so far, never negative.
    pub fn contribute(&self, x: f32, z: f32, current: f32) -> f32 {
        let (wx, wz) = warp(x, z, self.warp, self.seed);
        let inland = signed_distance_to_outline(wx, wz, self.outline);
        let (target, weight) = if inland >= 0.0 {
            let risen = ramp(inland, self.rise);
            let relief = if self.relief > 0.0 {
                let f = 1.0 / RELIEF_WAVELENGTH;
                let n = fbm_2d_periodic(x * f, z * f, 4, 0.5, 2.0, self.seed + RELIEF_SEED, None);
                (n - 0.5) * 2.0 * self.relief * risen
            } else {
                0.0
            };
            (
                self.shore_height + (self.height - self.shore_height) * risen + relief,
                1.0,
            )
        } else if self.skirt > 0.0 {
            (self.shore_height, 1.0 - ramp(-inland, self.skirt))
        } else {
            return 0.0;
        };
        (weight * (target - current)).max(0.0)
    }
}

/// A watercourse cut along a polyline. See `TerrainFeature::Channel`.
pub struct Channel<'a> {
    pub points: &'a [(f32, f32, f32)],
    pub width: f32,
    pub bank: f32,
    pub warp: f32,
    pub seed: u32,
}

impl Channel<'_> {
    /// Height to add at (x, z) over the ground so far, never positive.
    pub fn contribute(&self, x: f32, z: f32, current: f32) -> f32 {
        let (wx, wz) = warp(x, z, self.warp, self.seed);
        let Some((distance, bed)) = self.nearest_bed(wx, wz) else {
            return 0.0;
        };
        let weight = 1.0 - ramp(distance - self.width * 0.5, self.bank);
        (weight * (bed - current)).min(0.0)
    }

    /// Distance to the centreline and the bed's height at its nearest point.
    fn nearest_bed(&self, x: f32, z: f32) -> Option<(f32, f32)> {
        self.points
            .windows(2)
            .map(|pair| {
                let ((ax, ay, az), (bx, by, bz)) = (pair[0], pair[1]);
                let (t, distance) = point_to_segment_projection(x, z, ax, az, bx, bz);
                (distance, ay + (by - ay) * t)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }
}

/// A hollow with a level floor and an irregular rim. See `TerrainFeature::Basin`.
pub struct Basin {
    pub center: (f32, f32),
    pub radius: f32,
    pub floor: f32,
    pub warp: f32,
    pub seed: u32,
}

impl Basin {
    /// Height to add at (x, z) over the ground so far, never positive.
    pub fn contribute(&self, x: f32, z: f32, current: f32) -> f32 {
        let (wx, wz) = warp(x, z, self.warp, self.seed);
        let distance = (wx - self.center.0).hypot(wz - self.center.1);
        let half = self.radius * 0.5;
        let weight = 1.0 - ramp(distance - half, half);
        (weight * (self.floor - current)).min(0.0)
    }
}

/// The point (x, z) moved by smooth noise up to `amplitude` on each axis.
fn warp(x: f32, z: f32, amplitude: f32, seed: u32) -> (f32, f32) {
    if amplitude <= 0.0 {
        return (x, z);
    }
    let f = 1.0 / WARP_WAVELENGTH;
    let along = |seed| (fbm_2d_periodic(x * f, z * f, 3, 0.5, 2.0, seed, None) - 0.5) * 2.0;
    (
        x + along(seed) * amplitude,
        z + along(seed + WARP_AXIS_SEED) * amplitude,
    )
}

/// Smoothstep of `distance / span`: 0 at or before zero, 1 at or past `span`.
/// A span of zero is a step.
fn ramp(distance: f32, span: f32) -> f32 {
    if span <= 0.0 {
        return if distance > 0.0 { 1.0 } else { 0.0 };
    }
    let t = (distance / span).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Distance from (x, z) to a closed outline: positive inside, negative outside.
fn signed_distance_to_outline(x: f32, z: f32, outline: &[(f32, f32)]) -> f32 {
    let mut nearest = f32::INFINITY;
    let mut inside = false;
    for (i, &(ax, az)) in outline.iter().enumerate() {
        let (bx, bz) = outline[(i + 1) % outline.len()];
        nearest = nearest.min(point_to_segment_projection(x, z, ax, az, bx, bz).1);
        // Even-odd rule: count the edges a ray towards +x crosses.
        if (az > z) != (bz > z) && x < ax + (z - az) * (bx - ax) / (bz - az) {
            inside = !inside;
        }
    }
    if inside {
        nearest
    } else {
        -nearest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: [(f32, f32); 4] = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)];

    fn plain_landmass(skirt: f32) -> Landmass<'static> {
        Landmass {
            outline: &SQUARE,
            shore_height: 1.0,
            height: 9.0,
            rise: 4.0,
            skirt,
            warp: 0.0,
            relief: 0.0,
            seed: 0,
        }
    }

    #[test]
    fn a_landmass_stands_at_its_shore_on_the_outline_and_its_height_inland() {
        let land = plain_landmass(3.0);
        assert!((land.contribute(10.0, 0.0, -5.0) - 6.0).abs() < 1e-4);
        assert!((land.contribute(0.0, 0.0, -5.0) - 14.0).abs() < 1e-4);
        assert_eq!(land.contribute(14.0, 0.0, -5.0), 0.0, "past the skirt");
    }

    #[test]
    fn a_landmass_never_lowers_the_ground() {
        let land = plain_landmass(3.0);
        assert_eq!(land.contribute(0.0, 0.0, 20.0), 0.0);
    }

    #[test]
    fn a_channel_cuts_to_its_bed_and_steps_where_its_points_do() {
        // A fall from 5 m to 1 m over half a metre, at x = 10.
        let points = [
            (0.0, 5.0, 0.0),
            (10.0, 5.0, 0.0),
            (10.5, 1.0, 0.0),
            (20.0, 1.0, 0.0),
        ];
        let channel = Channel {
            points: &points,
            width: 2.0,
            bank: 1.0,
            warp: 0.0,
            seed: 0,
        };
        let ground = 8.0;
        assert!((ground + channel.contribute(5.0, 0.0, ground) - 5.0).abs() < 1e-4);
        assert!((ground + channel.contribute(15.0, 0.5, ground) - 1.0).abs() < 1e-4);
        assert_eq!(channel.contribute(5.0, 3.0, ground), 0.0, "past the bank");
        assert_eq!(
            channel.contribute(5.0, 0.0, 2.0),
            0.0,
            "below the bed it is left alone"
        );
    }

    #[test]
    fn a_basin_is_level_over_its_inner_half() {
        let basin = Basin {
            center: (0.0, 0.0),
            radius: 6.0,
            floor: -2.0,
            warp: 0.0,
            seed: 0,
        };
        assert!((basin.contribute(2.9, 0.0, 3.0) + 5.0).abs() < 1e-4);
        assert_eq!(basin.contribute(6.0, 0.0, 3.0), 0.0);
    }
}
