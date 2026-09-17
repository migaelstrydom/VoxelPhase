//! When the cursor is wanted, and when it has outstayed its welcome.

/// Decides whether the player is aiming this frame.
///
/// Aiming is the grab button held down: that button either means "I am about
/// to pick that up" or "I am holding this", and both want to see where a throw
/// would go.
///
/// The one rule that needs memory is what happens after a throw. Throwing the
/// held object empties the hand while the button is still down, and putting
/// the cursor straight back up for the grenade that could now be thrown reads
/// as a cursor that refuses to leave. So a grab-throw closes the gate until the
/// button is released. A grenade throw does not — the next grenade is thrown
/// from the same stance, and the cursor is wanted for it.
#[derive(Debug, Default, Clone, Copy)]
pub struct AimGate {
    suppressed_until_release: bool,
}

impl AimGate {
    /// Advance the gate with this frame's intent and report whether to aim.
    ///
    /// `throw_grenade` is the resolution the character control system already
    /// made: a throw it left for the grenade spawner is not a grab-throw. That
    /// decision is not re-derived here, so there is only ever one place that
    /// decides what a click meant.
    pub fn update(&mut self, grab_held: bool, throw: bool, throw_grenade: bool) -> bool {
        if !grab_held {
            self.suppressed_until_release = false;
        }
        if throw && !throw_grenade {
            self.suppressed_until_release = true;
        }

        grab_held && !self.suppressed_until_release
    }

    /// Forget any suppression — for when there is no player to aim.
    pub fn reset(&mut self) {
        self.suppressed_until_release = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holding_the_button_aims() {
        let mut gate = AimGate::default();
        assert!(gate.update(true, false, false));
    }

    #[test]
    fn releasing_the_button_stops_aiming() {
        let mut gate = AimGate::default();
        gate.update(true, false, false);
        assert!(!gate.update(false, false, false));
    }

    #[test]
    fn throwing_the_held_object_puts_the_cursor_away_for_the_rest_of_the_hold() {
        let mut gate = AimGate::default();
        gate.update(true, false, false);

        assert!(!gate.update(true, true, false), "the throw frame itself");
        assert!(!gate.update(true, false, false), "and every frame after it");
    }

    #[test]
    fn pressing_again_brings_the_cursor_back() {
        let mut gate = AimGate::default();
        gate.update(true, true, false);
        gate.update(false, false, false);

        assert!(gate.update(true, false, false));
    }

    #[test]
    fn throwing_a_grenade_leaves_the_cursor_up() {
        let mut gate = AimGate::default();
        gate.update(true, false, false);

        assert!(gate.update(true, true, true));
        assert!(gate.update(true, false, false));
    }
}
