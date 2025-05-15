use crate::components::{Rotation, SpinSpeed};
use specs::{Join, ReadStorage, System, WriteStorage};

pub struct SpinningSystem;

impl<'a> System<'a> for SpinningSystem {
    type SystemData = (WriteStorage<'a, Rotation>, ReadStorage<'a, SpinSpeed>);

    fn run(&mut self, (mut rotations, speeds): Self::SystemData) {
        for (rotation, speed) in (&mut rotations, &speeds).join() {
            rotation.0 += speed.0; // Simple increment, assumes fixed delta time for now
        }
    }
}
