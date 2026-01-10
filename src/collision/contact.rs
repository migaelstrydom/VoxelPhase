//! Contact manifold structures for collision response.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;
use specs::Entity;

/// A single contact point between two objects.
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// The contact point in world space (on the surface of the colliding object).
    pub point: Point3<f32>,
    /// Contact normal pointing from the terrain/obstacle toward the entity.
    pub normal: Vector3<f32>,
    /// Penetration depth (positive means overlapping).
    pub depth: f32,
}

impl ContactPoint {
    pub fn new(point: Point3<f32>, normal: Vector3<f32>, depth: f32) -> Self {
        Self {
            point,
            normal,
            depth,
        }
    }
}

/// A collection of contact points for a collision between an entity and terrain/other entities.
#[derive(Debug, Clone)]
pub struct ContactManifold {
    /// The entity that is colliding.
    pub entity: Entity,
    /// The other entity (None if colliding with terrain).
    pub other_entity: Option<Entity>,
    /// Contact points (typically 1-4 for most collisions).
    pub contacts: SmallVec<[ContactPoint; 4]>,
}

impl ContactManifold {
    /// Create a new empty contact manifold.
    pub fn new(entity: Entity, other_entity: Option<Entity>) -> Self {
        Self {
            entity,
            other_entity,
            contacts: SmallVec::new(),
        }
    }

    /// Create a manifold for terrain collision.
    pub fn terrain(entity: Entity) -> Self {
        Self::new(entity, None)
    }

    /// Add a contact point to the manifold.
    pub fn add_contact(&mut self, contact: ContactPoint) {
        self.contacts.push(contact);
    }

    /// Check if there are any contacts.
    pub fn has_contacts(&self) -> bool {
        !self.contacts.is_empty()
    }

    /// Get the deepest penetration among all contacts.
    pub fn max_penetration(&self) -> f32 {
        self.contacts.iter().map(|c| c.depth).fold(0.0, f32::max)
    }

    /// Get the average contact normal (useful for simple response).
    pub fn average_normal(&self) -> Option<Vector3<f32>> {
        if self.contacts.is_empty() {
            return None;
        }
        let sum: Vector3<f32> = self.contacts.iter().map(|c| c.normal).sum();
        let avg = sum / self.contacts.len() as f32;
        if avg.magnitude_squared() > 1e-6 {
            Some(avg.normalize())
        } else {
            None
        }
    }

    /// Check if any contact normal points mostly upward (for ground detection).
    pub fn has_ground_contact(&self, up_threshold: f32) -> bool {
        self.contacts.iter().any(|c| c.normal.y > up_threshold)
    }
}
