use nalgebra::Vector3;
use specs::{Join, ReadStorage, System, Write, WriteStorage};

use super::grounding::Grounding;
use crate::animation::CharacterAnimator;
use crate::components::RigidBodyComponent;
use crate::systems::PhysicsResource;

/// Cosine of the steepest slope that still counts as ground. A contact normal
/// within 50 degrees of world up is something you can stand on; anything
/// steeper is a wall you are pressed against.
const COS_MAX_GROUND_SLOPE: f32 = 0.642; // cos(50 degrees)

/// Derives [`Grounding`] from physics contacts for characters that have no
/// skeleton to probe with.
///
/// Rigged humanoids are deliberately skipped: `CharacterAnimationSystem` writes
/// their grounding from the foot probes, which resolve ledges and steps far
/// better than a capsule's contact set does. This system is the fallback for
/// everything else — rollers, drones on the ground, anything without an
/// animator.
pub struct ContactGroundingSystem;

impl<'a> System<'a> for ContactGroundingSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, CharacterAnimator>,
        WriteStorage<'a, Grounding>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physics_res, rigid_bodies, animators, mut groundings) = data;

        let grounded = physics_res.world.grounded_handles();

        // Steepest upward-facing contact normal per body, so a character
        // wedged between a wall and the floor reports the floor.
        let mut normals: Vec<(crate::physics::RigidBodyHandle, Vector3<f32>)> = Vec::new();
        for event in physics_res.world.contact_events() {
            // `normal` points from body_a to body_b, so for the character to be
            // the one being held up it must be body_b and the normal must point
            // up. Contacts against static geometry have no body_a.
            if event.normal.y > COS_MAX_GROUND_SLOPE {
                normals.push((event.body_b, event.normal));
            }
            if let Some(body_a) = event.body_a {
                if -event.normal.y > COS_MAX_GROUND_SLOPE {
                    normals.push((body_a, -event.normal));
                }
            }
        }

        for (rb, _, grounding) in (&rigid_bodies, !&animators, &mut groundings).join() {
            let best = normals
                .iter()
                .filter(|(handle, _)| *handle == rb.0)
                .map(|(_, n)| *n)
                .max_by(|a, b| a.y.total_cmp(&b.y));

            *grounding = match best {
                Some(normal) => Grounding::on(normal),
                // A body resting still enough to sleep stops emitting contact
                // events, but `grounded_handles` carries its support forward.
                None if grounded.contains(&rb.0) => Grounding::on(Vector3::y()),
                None => Grounding::airborne(),
            };
        }
    }
}
