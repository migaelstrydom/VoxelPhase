/// Holds a character's grounding answer true for a moment after support is
/// lost.
///
/// Contact grounding is the truthful answer and a chattery one: a walking
/// capsule genuinely leaves the floor between footfalls, and marching terrain
/// cubes genuinely open seams under it. The foot probes that used to answer
/// this question were lenient by accident — they asked "is there ground within
/// `probe_length` of where I am about to step", so their reach was a
/// forgiveness window nobody had chosen. This is that window, chosen: a named
/// duration on the locomotion side, applied to the one contact-derived answer.
///
/// It only ever extends a `true`. Support that was never there is not
/// forgiven, so this cannot ground an airborne character.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GroundForgiveness {
    /// Seconds of forgiveness left. Zero once the window has closed.
    remaining: f32,
}

impl GroundForgiveness {
    /// Advance by `dt` and report the forgiven grounding for this frame.
    ///
    /// Real support re-arms the window; its absence spends it.
    pub fn observe(&mut self, supported: bool, dt: f32, window: f32) -> bool {
        if supported {
            self.remaining = window.max(0.0);
        } else {
            self.remaining = (self.remaining - dt).max(0.0);
        }
        supported || self.remaining > 0.0
    }

    /// Close the window immediately.
    ///
    /// A jump is a deliberate departure from the ground, so there is nothing
    /// left to forgive: without this, a jump whose contact ends inside the
    /// window would keep reporting support into the air, and a long enough
    /// window would let `Airborne` see ground and land the character on
    /// nothing.
    pub fn cancel(&mut self) {
        self.remaining = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;
    const WINDOW: f32 = 0.1;

    #[test]
    fn real_support_is_reported_as_it_is() {
        let mut f = GroundForgiveness::default();
        for _ in 0..30 {
            assert!(f.observe(true, DT, WINDOW));
        }
    }

    #[test]
    fn support_that_was_never_there_is_not_forgiven() {
        let mut f = GroundForgiveness::default();
        for _ in 0..10 {
            assert!(
                !f.observe(false, DT, WINDOW),
                "grounded without ever landing"
            );
        }
    }

    #[test]
    fn lost_support_is_held_for_the_window_and_no_longer() {
        let mut f = GroundForgiveness::default();
        f.observe(true, DT, WINDOW);

        let mut forgiven_frames = 0;
        let mut elapsed = 0.0;
        while f.observe(false, DT, WINDOW) {
            forgiven_frames += 1;
            elapsed += DT;
            assert!(forgiven_frames < 100, "the window never closed");
        }
        assert!(
            (elapsed - WINDOW).abs() <= DT,
            "held for {elapsed}s, expected {WINDOW}s within one frame"
        );
    }

    #[test]
    fn a_frame_of_chatter_is_absorbed() {
        let mut f = GroundForgiveness::default();
        // Walking: contact drops for a frame between footfalls, twice.
        for supported in [true, true, false, true, false, false, true] {
            assert!(
                f.observe(supported, DT, WINDOW),
                "a one-frame contact loss reached the FSM"
            );
        }
    }

    #[test]
    fn the_window_re_arms_on_the_next_real_contact() {
        let mut f = GroundForgiveness::default();
        f.observe(true, DT, WINDOW);
        for _ in 0..3 {
            f.observe(false, DT, WINDOW);
        }
        f.observe(true, DT, WINDOW);
        let mut held = 0;
        while f.observe(false, DT, WINDOW) {
            held += 1;
        }
        assert!(
            (held as f32 * DT - WINDOW).abs() <= DT,
            "a partly spent window was not refilled by real contact"
        );
    }

    #[test]
    fn a_jump_cancels_the_window() {
        let mut f = GroundForgiveness::default();
        f.observe(true, DT, WINDOW);
        f.cancel();
        assert!(
            !f.observe(false, DT, WINDOW),
            "a jump kept its takeoff contact alive in the air"
        );
    }

    #[test]
    fn a_zero_window_forgives_nothing() {
        let mut f = GroundForgiveness::default();
        f.observe(true, DT, 0.0);
        assert!(!f.observe(false, DT, 0.0));
    }
}
