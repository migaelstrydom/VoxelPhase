//! A trampoline's dimensions, laid out once from its size, and the paths of
//! its rods: what the colliders, the frame model and the per-frame bed mesh
//! all read, so the three cannot disagree.
//!
//! ```text
//!        hoop (centreline radius R, at hoop_height)
//!   ◯━━━━┓~~~~~~~~~~~~~~~~~~~~~~~~~~~~┏━━━━◯   springs fan in to the bed
//!    \    ┗━━━━━━━━━━ bed ━━━━━━━━━━━┛    /    edge, which rides with it
//!     \                                  /     legs splay out to the ground
//!      ●                                ●
//! ```
//!
//! Everything is in the trampoline's ground frame: origin on the ground under
//! the hoop's centre, +Y up.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector3};

/// Height of the hoop's centreline above the ground, in metres.
const HOOP_HEIGHT: f32 = 0.9;
/// Radius of the hoop's tube.
const HOOP_TUBE_RADIUS: f32 = 0.075;
/// Horizontal span of the springs between the hoop and the bed's edge.
const SPRING_SPAN: f32 = 0.38;
/// Thickness of the bed's collider, which hangs below the drawn fabric. A
/// light bed driven down at landing speed lets a thin collider be passed
/// through in a substep; this much cannot be.
const BED_THICKNESS: f32 = 0.2;
/// How far in from the hoop's inner edge the bed's collider stops. It still
/// covers most of the springs, so a landing among them is a landing on the
/// bed, and clears the hoop by more than the contact margin, so the two never
/// touch and the bed rides free.
const BED_COLLIDER_INSET: f32 = 0.1;
/// Radius of a leg's tube.
const LEG_TUBE_RADIUS: f32 = 0.045;
/// How far out from the hoop a leg's foot stands.
const LEG_SPLAY: f32 = 0.22;
/// Radius of the ball on each foot.
const FOOT_RADIUS: f32 = 0.08;

/// A trampoline's dimensions, from which every part is laid out.
#[derive(Debug, Clone)]
pub struct TrampolineShape {
    /// Radius of the hoop's centreline, in metres.
    pub hoop_radius: f32,
    /// Height of the hoop's centreline above the ground.
    pub hoop_height: f32,
    /// Radius of the hoop's tube.
    pub hoop_tube_radius: f32,
    /// Radius of the drawn bed, where the springs take hold of it.
    pub bed_radius: f32,
    /// Radius of the bed's collider, reaching in under the springs.
    pub bed_collider_radius: f32,
    /// Thickness of the bed's collider.
    pub bed_thickness: f32,
    /// How many springs ring the bed.
    pub spring_count: u32,
    /// How many legs hold the hoop up.
    pub leg_count: u32,
    /// Radius of a leg's tube.
    pub leg_tube_radius: f32,
    /// Radius of the ball on each foot.
    pub foot_radius: f32,
}

impl TrampolineShape {
    /// A trampoline whose hoop has a centreline of `hoop_radius`.
    pub fn with_radius(hoop_radius: f32) -> Self {
        let bed_radius = hoop_radius - SPRING_SPAN;
        Self {
            hoop_radius,
            hoop_height: HOOP_HEIGHT,
            hoop_tube_radius: HOOP_TUBE_RADIUS,
            bed_radius,
            bed_collider_radius: hoop_radius - HOOP_TUBE_RADIUS - BED_COLLIDER_INSET,
            bed_thickness: BED_THICKNESS,
            spring_count: (hoop_radius * 20.0).round().max(12.0) as u32,
            leg_count: 6,
            leg_tube_radius: LEG_TUBE_RADIUS,
            foot_radius: FOOT_RADIUS,
        }
    }

    /// How far from the centre its feet reach: the radius of the ground it
    /// stands on.
    pub fn footprint_radius(&self) -> f32 {
        self.hoop_radius + LEG_SPLAY + self.foot_radius
    }

    /// The centre of the hoop's circle.
    pub fn hoop_centre(&self) -> Point3<f32> {
        Point3::new(0.0, self.hoop_height, 0.0)
    }

    /// Where the bed's collider centre rests, with nothing on it: its top
    /// level with the hoop's centreline.
    pub fn bed_rest_centre(&self) -> Point3<f32> {
        self.hoop_centre() - Vector3::y() * (self.bed_thickness * 0.5)
    }

    /// The hoop's centreline, `points` around.
    pub fn hoop_path(&self, points: u32) -> Vec<Point3<f32>> {
        (0..points)
            .map(|i| self.on_circle(self.hoop_radius, i as f32 / points as f32 * TAU, 0.0))
            .collect()
    }

    /// The bearing of leg `index` around the hoop, in radians.
    pub fn leg_bearing(&self, index: u32) -> f32 {
        (index as f32 + 0.5) / self.leg_count as f32 * TAU
    }

    /// Leg `index`'s centreline, from the hoop out and down to the centre of
    /// its foot ball: straight down a little way, then splayed out.
    pub fn leg_path(&self, index: u32) -> Vec<Point3<f32>> {
        let bearing = self.leg_bearing(index);
        let foot_height = self.foot_radius - self.hoop_height;
        vec![
            self.on_circle(self.hoop_radius, bearing, 0.0),
            self.on_circle(self.hoop_radius + 0.02, bearing, -0.18),
            self.on_circle(
                self.hoop_radius + LEG_SPLAY * 0.7,
                bearing,
                foot_height * 0.6,
            ),
            self.on_circle(self.hoop_radius + LEG_SPLAY, bearing, foot_height),
        ]
    }

    /// The bearing of spring `index` around the hoop, in radians.
    pub fn spring_bearing(&self, index: u32) -> f32 {
        index as f32 / self.spring_count as f32 * TAU
    }

    /// Where spring `index` hooks onto the hoop: on its inner side.
    pub fn spring_hoop_end(&self, index: u32) -> Point3<f32> {
        self.on_circle(
            self.hoop_radius - self.hoop_tube_radius * 0.6,
            self.spring_bearing(index),
            0.0,
        )
    }

    /// The point at `radius` from the hoop's axis, `bearing` round it and
    /// `rise` above the hoop's centreline.
    pub fn on_circle(&self, radius: f32, bearing: f32, rise: f32) -> Point3<f32> {
        Point3::new(
            radius * bearing.cos(),
            self.hoop_height + rise,
            radius * bearing.sin(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bed_collider_clears_the_hoop_and_covers_the_springs() {
        let shape = TrampolineShape::with_radius(1.8);
        assert!(shape.bed_collider_radius < shape.hoop_radius - shape.hoop_tube_radius);
        assert!(shape.bed_collider_radius > shape.bed_radius);
    }

    #[test]
    fn the_feet_stand_on_the_ground() {
        let shape = TrampolineShape::with_radius(1.8);
        for leg in 0..shape.leg_count {
            let foot = *shape.leg_path(leg).last().unwrap();
            assert!((foot.y - shape.foot_radius).abs() < 1.0e-5);
        }
    }
}
