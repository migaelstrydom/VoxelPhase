//! Explosion-related ECS components.

use nalgebra::Point3;
use specs::{Component, VecStorage};

use crate::physics::PhysicsImpulse;
use crate::terrain::BlastConfig;

/// Upward share of an explosion's shove, as a fraction of its outward strength.
pub const UPWARD_BOOST: f32 = 0.5;

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
    /// How this charge converts into removed terrain. The crater's size is not
    /// fixed here — it falls out of what the charge can afford to cut through,
    /// so the same grenade scoops sand and dents granite.
    pub blast: BlastConfig,
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
            blast: BlastConfig::default(),
            blast_radius: 5.0,
            force: 1000.0,
            processed: false,
        }
    }

    /// The shove this explosion gives rigid bodies: outward from the centre
    /// with linear falloff to the knockback radius, plus half again upward.
    pub fn physics_impulse(&self) -> PhysicsImpulse {
        PhysicsImpulse::radial(self.center, self.blast_radius, self.force, UPWARD_BOOST)
    }
}
