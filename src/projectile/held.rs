//! The grenade in the hand while a throw winds up.
//!
//! A prop, not a projectile: a model with no body, carried in the throwing
//! hand from the click until the release, when `GrenadeSpawnSystem` swaps it
//! for the grenade that flies.

use specs::{
    Component, DenseVecStorage, Entities, Entity, Join, ReadExpect, ReadStorage, System,
    WriteStorage,
};

use super::config::GrenadeConfig;
use crate::animation::CharacterAnimator;
use crate::character::ThrowingHand;
use crate::components::Position;

/// A grenade being wound up to throw.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct HeldGrenade {
    /// Who is throwing it.
    pub thrower: Entity,
    /// The throw it belongs to: the id of the thrower's `ThrowStroke`.
    pub stroke: u32,
    /// The hand it is in.
    pub hand: ThrowingHand,
}

/// Keeps each held grenade in its thrower's hand, as the animator drew it
/// this frame. Runs after character animation.
pub struct HeldGrenadeSystem;

impl<'a> System<'a> for HeldGrenadeSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, GrenadeConfig>,
        ReadStorage<'a, HeldGrenade>,
        ReadStorage<'a, CharacterAnimator>,
        WriteStorage<'a, Position>,
    );

    fn run(&mut self, (entities, config, held, animators, mut positions): Self::SystemData) {
        for (entity, held) in (&entities, &held).join() {
            let Some(animator) = animators.get(held.thrower) else {
                // The thrower is gone; nothing is holding it.
                let _ = entities.delete(entity);
                continue;
            };
            let (hand, along_forearm) = animator.grip(held.hand);
            // In the palm: the grenade sits beyond the hand, along the forearm.
            let centre = hand + along_forearm * config.radius;
            if let Some(position) = positions.get_mut(entity) {
                position.0 = centre.coords;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::animation::{BodyReading, CharacterRigConfig};
    use crate::character::grab::GrabConfig;
    use crate::character::{
        CharacterIntent, CharacterState, Grounding, Immersion, LocomotionConfig, ThrowStroke,
    };
    use crate::projectile::grenade_launch;

    /// The grenade flies from the launch origin the aiming cursor predicts
    /// from, so the hand has to let go of it there; anywhere else and the
    /// grenade jumps out of the hand as it is thrown.
    #[test]
    fn the_hand_lets_go_where_the_grenade_starts_its_arc() {
        const DT: f32 = 1.0 / 60.0;
        let config = GrenadeConfig::default();
        let clearance = LocomotionConfig::player().collider_half_height;
        let body = Point3::new(0.0, clearance, 0.0);
        let mut animator =
            CharacterAnimator::new(CharacterRigConfig::default(), body, clearance, 0.0);
        let grounding = Grounding::on(Vector3::y());
        let immersion = Immersion::default();

        // Up to the last frame the grenade is drawn in the hand: the one
        // before it is let go.
        let frames = (config.release_delay / DT).ceil() as usize;
        for frame in 0..frames {
            let stroke = ThrowStroke {
                id: 0,
                hand: ThrowingHand::Right,
                elapsed: frame as f32 * DT,
                release_at: config.release_delay,
                aim: Vector3::z(),
            };
            animator.observe_throws([&stroke], DT);
            let reading = BodyReading {
                pelvis: animator.pelvis_for(body, Vector3::y()),
                yaw: 0.0,
                velocity: Vector3::zeros(),
                up: Vector3::y(),
                grounding: &grounding,
                immersion: &immersion,
            };
            animator.update(
                DT,
                &reading,
                &CharacterState::default(),
                &CharacterIntent::default(),
                &GrabConfig::default(),
                &[],
            );
        }

        let (hand, along_forearm) = animator.grip(ThrowingHand::Right);
        let held = hand + along_forearm * config.radius;
        let launch = grenade_launch(&config, body, 0.0, 0.0, Vector3::new(0.0, -9.81, 0.0));
        let gap = (held - launch.origin).magnitude();
        assert!(
            gap < 0.1,
            "the grenade leaves the hand {gap:.2} m from where its arc starts \
             (held at {held:?}, arc from {:?})",
            launch.origin
        );
    }
}
