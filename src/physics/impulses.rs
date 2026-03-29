use nalgebra::{Point3, Vector3};

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
///
/// After draining, a snapshot of the drained impulses is kept in
/// `last_impulses` so that downstream systems (e.g. fracture) can
/// query what was applied this frame.
#[derive(Default)]
pub struct PhysicsImpulseQueue {
    events: Vec<PhysicsImpulse>,
    last_impulses: Vec<PhysicsImpulse>,
}

impl PhysicsImpulseQueue {
    pub fn push(&mut self, event: PhysicsImpulse) {
        self.events.push(event);
    }

    pub fn drain(&mut self) -> impl Iterator<Item = PhysicsImpulse> + '_ {
        self.last_impulses.clear();
        self.last_impulses.extend(self.events.iter().copied());
        self.events.drain(..)
    }

    /// Impulses that were applied during the most recent physics step.
    pub fn last_impulses(&self) -> &[PhysicsImpulse] {
        &self.last_impulses
    }
}
