/// What entering a state does, returned by `LocomotionState::enter`.
///
/// The driver applies the impulse, the air speed and the cut; the
/// conditioning stage applies the rest (`Conditioning::apply`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EntryEffects {
    /// Upward speed to leave with, in m/s: a jump's or a hop's impulse.
    pub vertical_speed: Option<f32>,
    /// The horizontal speed cap air steering latches, in m/s.
    pub air_speed: Option<f32>,
    /// The buffered jump press was spent.
    pub consume_jump: bool,
    /// Leaving the ground on purpose: no loss of support to forgive.
    pub cancel_forgiveness: bool,
    /// A buffered tap, the key already released: cut the rise up front so
    /// the jump is short.
    pub cut_rise: bool,
    /// Landing from a long jump: a still-held crouch is ignored for a while.
    pub arm_crouch_lockout: bool,
}
