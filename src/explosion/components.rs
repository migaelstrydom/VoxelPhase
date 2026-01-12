//! Explosion-related ECS components.

use nalgebra::Point3;
use specs::{Component, VecStorage};

/// Component representing an explosion event.
///
/// When an explosion is created (e.g., from a grenade detonation),
/// the ExplosionSystem processes it to:
/// 1. Carve a crater in the terrain
/// 2. Apply knockback force to nearby entities
/// 3. Spawn particle effects
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Explosion {
    /// Center point of the explosion in world space.
    pub center: Point3<f32>,
    /// Radius for terrain destruction (crater size).
    pub crater_radius: f32,
    /// Radius for knockback effect (can be larger than crater).
    pub blast_radius: f32,
    /// Force magnitude for knockback.
    pub force: f32,
    /// Whether this explosion has been processed.
    pub processed: bool,
}

impl Explosion {
    /// Create a new explosion at the given position with default parameters.
    pub fn new(center: Point3<f32>) -> Self {
        Self {
            center,
            crater_radius: 2.5,
            blast_radius: 5.0,
            force: 25.0,
            processed: false,
        }
    }
}
