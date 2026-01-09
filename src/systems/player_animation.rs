//! System for updating player procedural animations (breathing, blinking, etc.)

use crate::player::PlayerAnimationState;
use crate::time::Time;
use specs::{Join, ReadExpect, System, WriteStorage};

pub struct PlayerAnimationSystem;

impl<'a> System<'a> for PlayerAnimationSystem {
    type SystemData = (ReadExpect<'a, Time>, WriteStorage<'a, PlayerAnimationState>);

    fn run(&mut self, (time, mut animations): Self::SystemData) {
        let delta = time.delta_seconds();

        for anim in (&mut animations).join() {
            anim.update(delta);
        }
    }
}
