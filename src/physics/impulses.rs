use nalgebra::{Point3, Vector3};

use crate::collision::AABB;

// === One-shot impulses (explosions, jump pads, etc.) ===

/// A one-shot impulse event applied to nearby bodies and then discarded.
#[derive(Debug, Clone, Copy)]
pub enum PhysicsImpulse {
    /// Radial impulse expanding outward from a point with linear falloff.
    Radial {
        /// Center of the impulse in world space.
        center: Point3<f32>,
        /// Radius of effect in meters.
        radius: f32,
        /// Impulse magnitude (N·s) at the center, falling off linearly to zero at the radius.
        strength: f32,
        /// Upward impulse factor (0 = none, 0.5 = half of strength added as vertical boost).
        upward_boost: f32,
    },
}

impl PhysicsImpulse {
    /// Create a radial impulse (e.g. explosion).
    pub fn radial(center: Point3<f32>, radius: f32, strength: f32, upward_boost: f32) -> Self {
        Self::Radial {
            center,
            radius,
            strength,
            upward_boost,
        }
    }

    /// Compute the impulse vector to apply to a body at the given position.
    /// Returns `None` if the body is outside the impulse's area of effect.
    pub fn impulse_at(&self, body_pos: Point3<f32>) -> Option<Vector3<f32>> {
        match *self {
            Self::Radial {
                center,
                radius,
                strength,
                upward_boost,
            } => {
                if radius <= 0.0 || strength.abs() < 1e-6 {
                    return None;
                }
                let delta = body_pos - center;
                let distance = delta.magnitude();
                if distance >= radius {
                    return None;
                }
                let (direction, falloff) = if distance < 1e-4 {
                    (Vector3::y(), 1.0)
                } else {
                    (delta / distance, 1.0 - (distance / radius))
                };
                let impulse = direction * strength * falloff
                    + Vector3::new(0.0, strength * falloff * upward_boost, 0.0);
                Some(impulse)
            }
        }
    }
}

/// Queue of one-shot impulse events consumed each physics step.
#[derive(Default)]
pub struct PhysicsImpulseQueue {
    events: Vec<PhysicsImpulse>,
}

impl PhysicsImpulseQueue {
    pub fn push(&mut self, event: PhysicsImpulse) {
        self.events.push(event);
    }

    pub fn drain(&mut self) -> impl Iterator<Item = PhysicsImpulse> + '_ {
        self.events.drain(..)
    }
}

// === Persistent force fields (boundary springs, wind zones, etc.) ===

/// A persistent spatial force applied to bodies every physics step.
#[derive(Debug, Clone)]
pub enum ForceField {
    /// Spring boundary that pushes bodies back inside an AABB.
    ///
    /// For each axis where the body is outside the AABB, a spring force
    /// proportional to penetration depth pushes it back inward:
    /// `impulse = spring_k * penetration_depth * dt`
    Boundary {
        /// The region bodies should stay inside.
        bounds: AABB,
        /// Spring stiffness in N/m. Higher values push bodies back more aggressively.
        spring_k: f32,
    },
}

impl ForceField {
    /// Create a boundary spring field from an AABB.
    pub fn boundary(bounds: AABB, spring_k: f32) -> Self {
        Self::Boundary { bounds, spring_k }
    }

    /// Compute the impulse to apply to a body at the given position for a time step `dt`.
    /// Returns `None` if the body is unaffected by this field.
    pub fn impulse_at(&self, body_pos: Point3<f32>, dt: f32) -> Option<Vector3<f32>> {
        match self {
            Self::Boundary { bounds, spring_k } => {
                let mut push = Vector3::zeros();

                let y_push = 1.0;

                if body_pos.x < bounds.min.x {
                    push.x = bounds.min.x - body_pos.x;
                    push.y = y_push;
                } else if body_pos.x > bounds.max.x {
                    push.x = bounds.max.x - body_pos.x;
                    push.y = y_push;
                }

                if body_pos.y < bounds.min.y {
                    push.y = bounds.min.y - body_pos.y;
                } else if body_pos.y > bounds.max.y {
                    push.y = bounds.max.y - body_pos.y;
                }

                if body_pos.z < bounds.min.z {
                    push.z = bounds.min.z - body_pos.z;
                    push.y = y_push;
                } else if body_pos.z > bounds.max.z {
                    push.z = bounds.max.z - body_pos.z;
                    push.y = y_push;
                }

                if push.magnitude_squared() < 1e-10 {
                    return None;
                }

                Some(push * *spring_k * dt)
            }
        }
    }
}

/// Registry of persistent force fields applied every physics step.
#[derive(Default)]
pub struct ForceFieldRegistry {
    fields: Vec<ForceField>,
}

impl ForceFieldRegistry {
    /// Add a force field. Returns its index for later removal.
    pub fn add(&mut self, field: ForceField) -> usize {
        let idx = self.fields.len();
        self.fields.push(field);
        idx
    }

    pub fn iter(&self) -> impl Iterator<Item = &ForceField> {
        self.fields.iter()
    }
}
