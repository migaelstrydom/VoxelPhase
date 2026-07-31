//! `Staircase` — discrete steps between two heights.

use nalgebra::Point3;

use crate::collision::AABB;

use super::solid::{above, below, Sample, TraversalSolid};

/// A straight flight of treads climbing from one height to another.
///
/// ```text
///                              ┌───┐  ◀ to.y
///                       ┌──────┘   │
///                ┌──────┘          │     rise = (to.y − from.y) / steps
///   from.y ▶ ────┘                 │
///           └─────────────────────-┘  ◀ thickness below from.y
/// ```
///
/// Distinct from a ramp because the **rise per step** is what the player has to
/// clear, and unlike a slope that is a number a check can compare against the
/// jump apex.
///
/// The field is the union of one box per tread, each running from that tread's
/// near edge to the head of the flight. The boxes therefore **overlap**, and
/// deliberately: a set of abutting boxes would have a zero-distance plane at
/// every shared face, which reads as neither solid nor air and cracks the mesh
/// along every riser. Overlapping ones cover each other's faces, and the
/// tallest box reaching a point is the one that wins, which is exactly the
/// staircase profile because the treads ascend.
///
/// Boxes also matter for the riser itself: encoding the flight as a
/// piecewise-constant height would put every riser wherever the densities
/// either side happened to interpolate to, whereas a box has an exact face
/// there, so a riser lands on its authored run boundary.
///
/// A flight authored downward is normalised to an ascending one at
/// construction; the geometry is identical and the overlap argument only holds
/// one way round.
pub struct StaircaseSolid {
    /// Foot of the flight: the level the first riser rises *from*.
    foot: Point3<f32>,
    /// Unit run direction in the horizontal plane.
    dir_x: f32,
    dir_z: f32,
    /// Horizontal length of the whole flight.
    run: f32,
    /// Horizontal length of one tread, along the run.
    tread: f32,
    /// Half the authored tread width.
    half_width: f32,
    /// Number of treads.
    steps: u32,
    /// Height gained per step; negative for a flight authored downward.
    rise: f32,
    /// Underside of the flight.
    bottom: f32,
}

impl StaircaseSolid {
    /// Build a staircase, or `None` if it describes nothing: no steps, no
    /// horizontal run, or a non-positive width or thickness.
    pub fn new(
        from: (f32, f32, f32),
        to: (f32, f32, f32),
        width: f32,
        steps: u32,
        thickness: f32,
    ) -> Option<Self> {
        if steps == 0 || width <= 0.0 || thickness <= 0.0 {
            return None;
        }
        // Normalise to an ascending flight: the union below relies on later
        // treads being higher.
        let (from, to) = if to.1 < from.1 {
            (to, from)
        } else {
            (from, to)
        };

        let (dx, dz) = (to.0 - from.0, to.2 - from.2);
        let run = (dx * dx + dz * dz).sqrt();
        if run < 1e-6 {
            return None;
        }
        Some(Self {
            foot: Point3::new(from.0, from.1, from.2),
            dir_x: dx / run,
            dir_z: dz / run,
            run,
            tread: run / steps as f32,
            half_width: width * 0.5,
            steps,
            rise: (to.1 - from.1) / steps as f32,
            bottom: from.1 - thickness,
        })
    }

    /// Height of the tread `i` steps up from the foot.
    fn tread_top(&self, i: u32) -> f32 {
        self.foot.y + self.rise * (i + 1) as f32
    }

    /// Position along the run and unsigned distance from its centreline.
    fn axis_frame(&self, p: Point3<f32>) -> (f32, f32) {
        let (rx, rz) = (p.x - self.foot.x, p.z - self.foot.z);
        let along = rx * self.dir_x + rz * self.dir_z;
        let lateral = (rx * self.dir_z - rz * self.dir_x).abs();
        (along, lateral)
    }
}

impl TraversalSolid for StaircaseSolid {
    fn sample(&self, p: Point3<f32>) -> Sample {
        let (along, lateral) = self.axis_frame(p);

        let mut distance = f32::INFINITY;
        for i in 0..self.steps {
            let d = above(along, i as f32 * self.tread)
                .max(below(along, self.run))
                .max(lateral - self.half_width)
                .max(below(p.y, self.tread_top(i)))
                .max(above(p.y, self.bottom));
            distance = distance.min(d);
        }

        // The tread the player would be standing on here, for durability.
        let index = ((along / self.tread).floor() as i32).clamp(0, self.steps as i32 - 1) as u32;
        Sample {
            distance,
            surface_y: self.tread_top(index),
        }
    }

    fn bounds(&self, voxel_size: f32) -> AABB {
        let head_x = self.foot.x + self.dir_x * self.run;
        let head_z = self.foot.z + self.dir_z * self.run;
        let pad = self.half_width + voxel_size;
        let top = self.foot.y.max(self.tread_top(self.steps - 1));
        AABB::new(
            Point3::new(
                self.foot.x.min(head_x) - pad,
                self.bottom - voxel_size,
                self.foot.z.min(head_z) - pad,
            ),
            Point3::new(
                self.foot.x.max(head_x) + pad,
                top + voxel_size,
                self.foot.z.max(head_z) + pad,
            ),
        )
    }
}
