//! The part of the screen a frame's water can cover.
//!
//! The water refracts a copy of the scene, and the copy costs in proportion to
//! the pixels it moves. Only the pixels the water draws over, and those its
//! refraction can reach from them, are ever read, so the copy is limited to
//! the screen rectangle around the planned draws' bounds:
//!
//! ```text
//!   world box per draw ──project──▶ NDC rectangle ──union──▶ + refraction reach
//!                                                            ──▶ pixel rectangle
//! ```
//!
//! A box reaching behind the camera cannot be projected, and a draw with no
//! bounds cannot be placed at all; either makes the footprint the whole screen.

use ash::vk;
use nalgebra::{Matrix4, Vector2, Vector3};

/// How far the water's refraction can shift a lookup, in UV: its strength
/// times the most `water.frag` scales it by.
const REFRACTION_REACH: f32 = 0.01 * 3.0;

/// Extra pixels around the rectangle, for the sampler's filter footprint.
const FILTER_PIXELS: i32 = 2;

/// The screen rectangle a frame's water can read, in normalised device
/// coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScreenFootprint {
    min: Vector2<f32>,
    max: Vector2<f32>,
    whole_screen: bool,
}

impl Default for ScreenFootprint {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl ScreenFootprint {
    pub const EMPTY: Self = Self {
        min: Vector2::new(f32::INFINITY, f32::INFINITY),
        max: Vector2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
        whole_screen: false,
    };

    /// Cover everything: for a draw whose bounds are not known.
    pub fn cover_all(&mut self) {
        self.whole_screen = true;
    }

    /// Add a world-space box, seen through `clip` (projection × view).
    pub fn add_box(&mut self, clip: &Matrix4<f32>, min: Vector3<f32>, max: Vector3<f32>) {
        if self.whole_screen {
            return;
        }
        for corner in 0..8 {
            let pick = |bit: usize, low: f32, high: f32| if corner & bit == 0 { low } else { high };
            let point = clip
                * Vector3::new(
                    pick(1, min.x, max.x),
                    pick(2, min.y, max.y),
                    pick(4, min.z, max.z),
                )
                .push(1.0);
            if point.w <= f32::EPSILON {
                self.whole_screen = true;
                return;
            }
            let ndc = Vector2::new(point.x / point.w, point.y / point.w);
            self.min = self.min.inf(&ndc);
            self.max = self.max.sup(&ndc);
        }
    }

    /// The pixels of an `extent` target the water can read, or `None` when it
    /// reads none.
    pub fn pixels(&self, extent: vk::Extent2D) -> Option<vk::Rect2D> {
        let size = Vector2::new(extent.width as f32, extent.height as f32);
        let (min, max) = if self.whole_screen {
            (Vector2::zeros(), size)
        } else {
            let reach = Vector2::repeat(REFRACTION_REACH * 2.0);
            let to_pixels =
                |ndc: Vector2<f32>| (ndc + Vector2::repeat(1.0)).component_mul(&size) * 0.5;
            (to_pixels(self.min - reach), to_pixels(self.max + reach))
        };
        let x0 = (min.x.floor() as i32 - FILTER_PIXELS).max(0);
        let y0 = (min.y.floor() as i32 - FILTER_PIXELS).max(0);
        let x1 = (max.x.ceil() as i32 + FILTER_PIXELS).min(extent.width as i32);
        let y1 = (max.y.ceil() as i32 + FILTER_PIXELS).min(extent.height as i32);
        (x1 > x0 && y1 > y0).then(|| vk::Rect2D {
            offset: vk::Offset2D { x: x0, y: y0 },
            extent: vk::Extent2D {
                width: (x1 - x0) as u32,
                height: (y1 - y0) as u32,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    const EXTENT: vk::Extent2D = vk::Extent2D {
        width: 800,
        height: 600,
    };

    fn clip() -> Matrix4<f32> {
        let view = Matrix4::look_at_rh(
            &Point3::new(0.0, 5.0, 10.0),
            &Point3::origin(),
            &Vector3::y(),
        );
        Matrix4::new_perspective(800.0 / 600.0, 1.0, 0.1, 100.0) * view
    }

    #[test]
    fn nothing_added_reads_nothing() {
        assert_eq!(ScreenFootprint::EMPTY.pixels(EXTENT), None);
    }

    #[test]
    fn a_small_pond_ahead_reads_part_of_the_screen() {
        let mut footprint = ScreenFootprint::EMPTY;
        footprint.add_box(
            &clip(),
            Vector3::new(-1.0, 0.0, -1.0),
            Vector3::new(1.0, 0.0, 1.0),
        );
        let rect = footprint.pixels(EXTENT).unwrap();
        assert!(rect.extent.width < EXTENT.width / 2);
        assert!(rect.extent.height < EXTENT.height / 2);
    }

    #[test]
    fn water_reaching_behind_the_camera_reads_the_whole_screen() {
        let mut footprint = ScreenFootprint::EMPTY;
        footprint.add_box(
            &clip(),
            Vector3::new(-50.0, 0.0, -50.0),
            Vector3::new(50.0, 0.0, 50.0),
        );
        let rect = footprint.pixels(EXTENT).unwrap();
        assert_eq!(
            (rect.extent.width, rect.extent.height),
            (EXTENT.width, EXTENT.height)
        );
    }

    #[test]
    fn water_beside_the_screen_is_clamped_to_it() {
        let mut footprint = ScreenFootprint::EMPTY;
        footprint.add_box(
            &clip(),
            Vector3::new(4.0, 0.0, -1.0),
            Vector3::new(30.0, 0.0, 1.0),
        );
        let rect = footprint.pixels(EXTENT).unwrap();
        assert!(rect.offset.x as u32 + rect.extent.width <= EXTENT.width);
    }
}
