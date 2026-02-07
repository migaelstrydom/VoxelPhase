use crate::physics::handle::RigidBodyHandle;

#[derive(Debug, Clone, Copy)]
pub struct WakeEvent {
    /// Body to wake.
    pub body: RigidBodyHandle,
}

pub struct WakeEvents {
    /// Accumulated wake requests for the current frame.
    events: Vec<WakeEvent>,
}

impl WakeEvents {
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    pub fn push(&mut self, body: RigidBodyHandle) {
        self.events.push(WakeEvent { body });
    }

    pub fn drain(&mut self) -> impl Iterator<Item = WakeEvent> + '_ {
        self.events.drain(..)
    }

    pub fn clear(&mut self) {
        self.events.clear();
    }
}
