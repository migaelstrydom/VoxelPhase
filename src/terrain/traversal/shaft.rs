//! `Shaft` — a vertical bore, optionally with a helical ledge down its wall.
//!
//! Two fields rather than one, applied in order: the bore is **subtracted**
//! from whatever the shaft is sunk into, then the ledge is **added** back
//! inside it. Splitting them this way is what lets a tower descent be authored
//! as one feature — the bore does not need to know the ledge exists, and the
//! ledge is an ordinary solid that happens to spiral.

use std::f32::consts::TAU;

use nalgebra::Point3;

use crate::collision::AABB;

use super::solid::{above, below, Sample, TraversalSolid};

/// The cylindrical void of a shaft.
pub struct ShaftBore {
    center_x: f32,
    center_z: f32,
    /// Bottom of the bore.
    from_y: f32,
    /// Top of the bore.
    to_y: f32,
    radius: f32,
}

impl ShaftBore {
    /// Build a bore, or `None` if it has no radius or no height.
    pub fn new(center: (f32, f32), from_y: f32, to_y: f32, radius: f32) -> Option<Self> {
        if radius <= 0.0 || to_y <= from_y {
            return None;
        }
        Some(Self {
            center_x: center.0,
            center_z: center.1,
            from_y,
            to_y,
            radius,
        })
    }

    /// Horizontal distance from the bore axis.
    fn radial(&self, p: Point3<f32>) -> f32 {
        let (dx, dz) = (p.x - self.center_x, p.z - self.center_z);
        (dx * dx + dz * dz).sqrt()
    }
}

impl TraversalSolid for ShaftBore {
    fn sample(&self, p: Point3<f32>) -> Sample {
        let distance = (self.radial(p) - self.radius)
            .max(above(p.y, self.from_y))
            .max(below(p.y, self.to_y));
        Sample {
            distance,
            surface_y: self.to_y,
        }
    }

    fn bounds(&self, voxel_size: f32) -> AABB {
        let pad = self.radius + voxel_size;
        AABB::new(
            Point3::new(
                self.center_x - pad,
                self.from_y - voxel_size,
                self.center_z - pad,
            ),
            Point3::new(
                self.center_x + pad,
                self.to_y + voxel_size,
                self.center_z + pad,
            ),
        )
    }
}

/// A helical ledge hugging the inside of a shaft wall.
///
/// ```text
///   elevation                      plan
///    ┃      ▂▂▂▂ ┃                  ╭─────╮
///    ┃ ▂▂▂▂      ┃                 │ ╭───╮ │   ledge occupies the annulus
///    ┃      ▂▂▂▂ ┃                 │ │   │ │   from radius−width to radius
///    ┃ ▂▂▂▂      ┃                  ╰─────╯
/// ```
///
/// The ledge surface is the helix `y = from_y + pitch · turns`, and each sample
/// resolves to the turn nearest its own height, so the field is continuous
/// where the angle wraps rather than seaming at an arbitrary direction.
pub struct ShaftLedgeSolid {
    center_x: f32,
    center_z: f32,
    /// Bottom of the bore — the height the helix starts from.
    from_y: f32,
    /// Top of the bore. The helix is cut off above this.
    to_y: f32,
    /// Outer radius: the ledge meets the bore wall here.
    radius: f32,
    /// Width measured inward from the bore wall.
    width: f32,
    /// Vertical thickness of the ledge slab.
    thickness: f32,
    /// Height gained per full turn.
    pitch: f32,
    /// Angle, in radians, at which the helix passes through `from_y`.
    start_angle: f32,
}

impl ShaftLedgeSolid {
    /// Build a ledge, or `None` if it would not be walkable: no width, no
    /// thickness, a width exceeding the bore, or a pitch of zero (which is a
    /// disc sealing the shaft, not a ledge).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        center: (f32, f32),
        from_y: f32,
        to_y: f32,
        radius: f32,
        width: f32,
        thickness: f32,
        pitch: f32,
        start_angle_degrees: f32,
    ) -> Option<Self> {
        if width <= 0.0 || thickness <= 0.0 || width > radius || pitch.abs() < 1e-3 {
            return None;
        }
        Some(Self {
            center_x: center.0,
            center_z: center.1,
            from_y,
            to_y,
            radius,
            width,
            thickness,
            pitch,
            start_angle: start_angle_degrees.to_radians(),
        })
    }

    /// Height of the helix turn passing nearest to `y` at angle `theta`.
    fn helix_top(&self, theta: f32, y: f32) -> f32 {
        let fraction = (theta - self.start_angle) / TAU;
        let turns = ((y - self.from_y) / self.pitch - fraction).round() + fraction;
        self.from_y + self.pitch * turns
    }
}

impl TraversalSolid for ShaftLedgeSolid {
    fn sample(&self, p: Point3<f32>) -> Sample {
        let (dx, dz) = (p.x - self.center_x, p.z - self.center_z);
        let radial = (dx * dx + dz * dz).sqrt();
        let theta = dz.atan2(dx);
        let top = self.helix_top(theta, p.y);
        let v = p.y - top;

        let distance = below(radial, self.radius)
            .max(above(radial, self.radius - self.width))
            .max(below(v, 0.0))
            .max(above(v, -self.thickness))
            // Cut the helix off outside the bore it runs in, so it neither
            // dives below the floor nor emerges above the mouth.
            .max(above(top, self.from_y))
            .max(below(top, self.to_y));

        Sample {
            distance,
            surface_y: top,
        }
    }

    fn bounds(&self, voxel_size: f32) -> AABB {
        let pad = self.radius + voxel_size;
        AABB::new(
            Point3::new(
                self.center_x - pad,
                self.from_y - self.thickness - voxel_size,
                self.center_z - pad,
            ),
            Point3::new(
                self.center_x + pad,
                self.to_y + voxel_size,
                self.center_z + pad,
            ),
        )
    }
}
