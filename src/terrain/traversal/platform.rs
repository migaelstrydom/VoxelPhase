//! `Platform` — a free-standing slab at a height.

use nalgebra::Point3;

use crate::collision::AABB;

use super::solid::{above, below, slab, Sample, TraversalSolid};

/// A rectangular slab whose top face is the walking surface.
///
/// The atom of a jump sequence, and trivial next to [`PathSolid`]: a box, with
/// `center.y` on the surface rather than in the middle so a platform is
/// authored at the height the player stands on.
///
/// [`PathSolid`]: super::PathSolid
pub struct PlatformSolid {
    /// Centre of the walking surface, in segment-local coordinates.
    center: Point3<f32>,
    /// Half-extent of the deck along local x.
    half_x: f32,
    /// Half-extent of the deck along local z.
    half_z: f32,
    /// How far the slab extends below its walking surface.
    thickness: f32,
}

impl PlatformSolid {
    /// Build a platform, or `None` if any authored extent is non-positive.
    pub fn new(center: (f32, f32, f32), half_extents: (f32, f32), thickness: f32) -> Option<Self> {
        if half_extents.0 <= 0.0 || half_extents.1 <= 0.0 || thickness <= 0.0 {
            return None;
        }
        Some(Self {
            center: Point3::new(center.0, center.1, center.2),
            half_x: half_extents.0,
            half_z: half_extents.1,
            thickness,
        })
    }
}

impl TraversalSolid for PlatformSolid {
    fn sample(&self, p: Point3<f32>) -> Sample {
        let distance = slab(p.x, self.center.x, self.half_x)
            .max(slab(p.z, self.center.z, self.half_z))
            .max(below(p.y, self.center.y))
            .max(above(p.y, self.center.y - self.thickness));
        Sample {
            distance,
            surface_y: self.center.y,
        }
    }

    fn bounds(&self, voxel_size: f32) -> AABB {
        AABB::new(
            Point3::new(
                self.center.x - self.half_x - voxel_size,
                self.center.y - self.thickness - voxel_size,
                self.center.z - self.half_z - voxel_size,
            ),
            Point3::new(
                self.center.x + self.half_x + voxel_size,
                self.center.y + voxel_size,
                self.center.z + self.half_z + voxel_size,
            ),
        )
    }
}
