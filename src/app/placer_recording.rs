//! Which set of feet F4 records.
//!
//! The foot placer's recorder lives on every [`LeggedLocomotion`], but the
//! recording it writes goes to the single path `PLACER_REC` names, so only
//! one rig can be recording at a time. This is the part that decides which.
//!
//! ```text
//!   F4 ──▶ toggle_nearest_recording
//!            │  nearest peeper to the player?
//!            ├── yes ──▶ PeeperAnimator::locomotion.toggle_recording()
//!            └── no  ──▶ CharacterAnimator::toggle_recording()
//! ```
//!
//! The peeper wins because it is the one whose gait is under suspicion and
//! the one you cannot drive by hand: the player's rig is always a keypress
//! away from any manoeuvre you like, a creature's only does the thing when
//! it is chasing you. With no peeper in the world F4 falls back to the
//! player, which is what it has always done.

use specs::{Join, World, WorldExt};

use crate::animation::peeper::PeeperAnimator;
use crate::animation::CharacterAnimator;
use crate::components::Position;
use crate::player::Player;

/// Start or stop recording placer input for the peeper nearest the player.
///
/// Falls back to the player's own rig when no peeper is in the world.
/// Nothing at all happens unless `PLACER_REC` is set, which is the recorder's
/// own rule.
pub fn toggle_nearest_recording(world: &World) {
    match nearest_peeper(world) {
        Some(entity) => {
            let mut animators = world.write_storage::<PeeperAnimator>();
            if let Some(animator) = animators.get_mut(entity) {
                animator.locomotion.toggle_recording();
            }
        }
        None => {
            let mut animators = world.write_storage::<CharacterAnimator>();
            for animator in (&mut animators).join() {
                animator.toggle_recording();
            }
        }
    }
}

/// The peeper closest to the player, or `None` when there is no peeper or
/// no player to measure from.
fn nearest_peeper(world: &World) -> Option<specs::Entity> {
    let entities = world.entities();
    let positions = world.read_storage::<Position>();
    let players = world.read_storage::<Player>();
    let animators = world.read_storage::<PeeperAnimator>();

    let player_pos = (&players, &positions).join().next()?.1 .0;

    (&entities, &positions, &animators)
        .join()
        .map(|(entity, pos, _)| (entity, (pos.0 - player_pos).norm_squared()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(entity, _)| entity)
}
