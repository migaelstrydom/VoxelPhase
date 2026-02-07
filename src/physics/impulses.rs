use nalgebra::Point3;

#[derive(Debug, Clone, Copy)]
pub struct PhysicsImpulse {
    /// Center of the radial impulse in world space.
    pub center: Point3<f32>,
    /// Radius of effect in meters.
    pub radius: f32,
    /// Impulse magnitude (N·s) applied with linear falloff.
    pub strength: f32,
    /// Upward impulse factor (0 = none, 0.5 = half of strength).
    pub upward_boost: f32,
}

impl PhysicsImpulse {
    pub fn new(center: Point3<f32>, radius: f32, strength: f32, upward_boost: f32) -> Self {
        Self {
            center,
            radius,
            strength,
            upward_boost,
        }
    }
}

#[derive(Default)]
pub struct PhysicsImpulseQueue {
    /// Pending impulse events for the next physics step.
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
