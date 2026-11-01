/// How far a takeoff has got, shared by Hop and Jump so that "down" means
/// the same for both.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TakeoffPhase {
    /// Still in contact, for at most `remaining` more seconds. Leaving the
    /// ground is the normal exit, but it cannot be the only one: a grounding
    /// signal that never goes false would otherwise hold the body here.
    Launch { remaining: f32 },
    /// Off the ground.
    Flight,
}

impl TakeoffPhase {
    /// A takeoff from the ground, held in contact for at most `window`.
    pub fn launch(window: f32) -> Self {
        Self::Launch { remaining: window }
    }

    /// Whether the takeoff is over: grounded in flight. A body still on the
    /// ground at takeoff, or a stance foot left on a lip, has not landed.
    pub fn down(&self, grounded: bool) -> bool {
        matches!(self, Self::Flight) && grounded
    }

    /// The phase `dt` seconds on. The window expiring while still grounded
    /// is a takeoff that never got off the floor: it is a flight anyway, and
    /// the next tick lands it, which hands back control.
    pub fn advanced(self, grounded: bool, dt: f32) -> Self {
        match self {
            Self::Launch { remaining } if grounded && remaining - dt > 0.0 => Self::Launch {
                remaining: remaining - dt,
            },
            Self::Launch { .. } | Self::Flight => Self::Flight,
        }
    }
}
