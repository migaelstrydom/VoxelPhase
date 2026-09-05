use super::grounding::Grounding;

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
///
/// The whole answer is forgiven, not just its boolean: the normal held the
/// character up, and so did whatever motion the surface had. The frame a
/// walking capsule leaves the floor is not the frame it stepped off the
/// platform, and a gait that loses the platform's velocity for that frame is
/// perturbed at exactly the rate the window exists to absorb.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GroundForgiveness {
    /// Seconds of forgiveness left. Zero once the window has closed.
    remaining: f32,
    /// The last support actually observed, reported while the window is open.
    last: Grounding,
}

impl GroundForgiveness {
    /// Advance by `dt` and report the forgiven grounding for this frame.
    ///
    /// Real support re-arms the window; its absence spends it.
    pub fn observe(&mut self, grounding: &Grounding, dt: f32, window: f32) -> Grounding {
        if grounding.is_grounded {
            self.remaining = window.max(0.0);
            self.last = *grounding;
            return *grounding;
        }

        self.remaining = (self.remaining - dt).max(0.0);
        if self.remaining > 0.0 {
            self.last
        } else {
            self.last = Grounding::airborne();
            Grounding::airborne()
        }
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
        self.last = Grounding::airborne();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Vector3;

    const DT: f32 = 1.0 / 60.0;
    const WINDOW: f32 = 0.1;

    /// Standing on level ground that is going nowhere.
    fn floor() -> Grounding {
        Grounding::on(Vector3::y())
    }

    /// Standing on something moving east at 3 m/s.
    fn platform() -> Grounding {
        floor().carried_by(Vector3::new(3.0, 0.0, 0.0))
    }

    #[test]
    fn real_support_is_reported_as_it_is() {
        let mut f = GroundForgiveness::default();
        for _ in 0..30 {
            assert!(f.observe(&floor(), DT, WINDOW).is_grounded);
        }
    }

    #[test]
    fn support_that_was_never_there_is_not_forgiven() {
        let mut f = GroundForgiveness::default();
        for _ in 0..10 {
            assert!(
                !f.observe(&Grounding::airborne(), DT, WINDOW).is_grounded,
                "grounded without ever landing"
            );
        }
    }

    #[test]
    fn lost_support_is_held_for_the_window_and_no_longer() {
        let mut f = GroundForgiveness::default();
        f.observe(&floor(), DT, WINDOW);

        let mut forgiven_frames = 0;
        let mut elapsed = 0.0;
        while f.observe(&Grounding::airborne(), DT, WINDOW).is_grounded {
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
            let observed = if supported {
                floor()
            } else {
                Grounding::airborne()
            };
            assert!(
                f.observe(&observed, DT, WINDOW).is_grounded,
                "a one-frame contact loss reached the FSM"
            );
        }
    }

    #[test]
    fn the_window_re_arms_on_the_next_real_contact() {
        let mut f = GroundForgiveness::default();
        f.observe(&floor(), DT, WINDOW);
        for _ in 0..3 {
            f.observe(&Grounding::airborne(), DT, WINDOW);
        }
        f.observe(&floor(), DT, WINDOW);
        let mut held = 0;
        while f.observe(&Grounding::airborne(), DT, WINDOW).is_grounded {
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
        f.observe(&floor(), DT, WINDOW);
        f.cancel();
        assert!(
            !f.observe(&Grounding::airborne(), DT, WINDOW).is_grounded,
            "a jump kept its takeoff contact alive in the air"
        );
    }

    #[test]
    fn a_zero_window_forgives_nothing() {
        let mut f = GroundForgiveness::default();
        f.observe(&floor(), DT, 0.0);
        assert!(!f.observe(&Grounding::airborne(), DT, 0.0).is_grounded);
    }

    #[test]
    fn the_surface_a_character_was_carried_by_is_forgiven_too() {
        let mut f = GroundForgiveness::default();
        assert_eq!(
            f.observe(&platform(), DT, WINDOW).surface_velocity,
            Vector3::new(3.0, 0.0, 0.0)
        );
        // One footfall's worth of lost contact must not read as stepping off.
        let blip = f.observe(&Grounding::airborne(), DT, WINDOW);
        assert!(blip.is_grounded);
        assert_eq!(blip.surface_velocity, Vector3::new(3.0, 0.0, 0.0));

        while f.observe(&Grounding::airborne(), DT, WINDOW).is_grounded {}
        assert_eq!(
            f.observe(&Grounding::airborne(), DT, WINDOW)
                .surface_velocity,
            Vector3::zeros(),
            "once the window closes nothing is carrying the character"
        );
    }
}
