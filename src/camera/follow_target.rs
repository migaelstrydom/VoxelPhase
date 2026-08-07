use specs::{Component, DenseVecStorage, Entity};

/// Component that makes a camera follow a target entity.
/// Attach this to a camera entity to make it follow another entity.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct FollowTarget {
    /// The entity to follow
    pub target: Entity,

    /// Current orbit angle around the target (radians, 0 = behind target on +Z)
    pub orbit_angle: f32,

    /// Current pitch angle (radians, 0 = horizontal, positive = looking down from above)
    pub pitch: f32,

    /// Current distance from target
    pub distance: f32,
}

impl FollowTarget {
    pub fn new(target: Entity, initial_distance: f32) -> Self {
        Self {
            target,
            orbit_angle: 0.0,
            pitch: 0.25, // Start looking slightly down
            distance: initial_distance,
        }
    }
}
