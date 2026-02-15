use crate::components::{Acceleration, Gravity};
use specs::{Join, ReadStorage, System, WriteStorage};

/// Applies gravity to entities that have the Gravity component.
/// Sets the Y component of acceleration based on the gravity value.
pub struct GravitySystem;

impl<'a> System<'a> for GravitySystem {
    type SystemData = (ReadStorage<'a, Gravity>, WriteStorage<'a, Acceleration>);

    fn run(&mut self, (gravities, mut accelerations): Self::SystemData) {
        for (gravity, accel) in (&gravities, &mut accelerations).join() {
            // Gravity is a downward force (negative Y in our coordinate system)
            accel.0.y = -gravity.0;
        }
    }
}
