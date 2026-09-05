//! What the character is asked to do, and when.
//!
//! A script is a list of beats, each holding one steady intent for a duration.
//! It is written in exactly the vocabulary `CharacterIntent` uses — a direction
//! and three modifier flags — because that is the seam the game drives the
//! character through. Scripting anything richer would be scripting something
//! the player cannot express.
//!
//! Transitions are not a beat type. A transition is what happens *between* two
//! beats, and the animator sees the same edge it would see from a keyboard.

use nalgebra::Vector3;

/// One steady intent, held for `duration`.
#[derive(Clone, Debug)]
pub struct Beat {
    /// Appears in the report and on filmstrip captions. Keep it one word.
    pub label: &'static str,
    pub duration: f32,
    /// Desired direction of travel in world space. Zero means "stand".
    pub direction: Vector3<f32>,
    pub sprint: bool,
    pub crouch: bool,
    /// Press jump on the first frame of this beat.
    pub jump: bool,
    /// Rotate `direction` about Y at this rate, in radians per second. Turns a
    /// straight walk into a circle without needing a per-frame script.
    pub turn_rate: f32,
}

impl Beat {
    /// Stand still for `duration`.
    pub fn stand(duration: f32) -> Self {
        Self {
            label: "stand",
            duration,
            direction: Vector3::zeros(),
            sprint: false,
            crouch: false,
            jump: false,
            turn_rate: 0.0,
        }
    }

    /// Walk in +x for `duration`.
    pub fn walk(duration: f32) -> Self {
        Self {
            label: "walk",
            ..Self::stand(duration)
        }
        .towards(Vector3::x())
    }

    pub fn run(duration: f32) -> Self {
        Self {
            label: "run",
            sprint: true,
            ..Self::walk(duration)
        }
    }

    pub fn crouch_walk(duration: f32) -> Self {
        Self {
            label: "crouch",
            crouch: true,
            ..Self::walk(duration)
        }
    }

    pub fn crouch_still(duration: f32) -> Self {
        Self {
            label: "crouch_idle",
            crouch: true,
            ..Self::stand(duration)
        }
    }

    /// Jump on the first frame, holding whatever this beat's direction is.
    pub fn jump(duration: f32) -> Self {
        Self {
            label: "jump",
            jump: true,
            ..Self::walk(duration)
        }
    }

    pub fn towards(mut self, direction: Vector3<f32>) -> Self {
        self.direction = direction;
        self
    }

    pub fn turning(mut self, radians_per_second: f32) -> Self {
        self.turn_rate = radians_per_second;
        self
    }

    pub fn named(mut self, label: &'static str) -> Self {
        self.label = label;
        self
    }
}

/// A whole scenario's worth of intent.
#[derive(Clone, Debug, Default)]
pub struct Script {
    pub beats: Vec<Beat>,
}

impl Script {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn then(mut self, beat: Beat) -> Self {
        self.beats.push(beat);
        self
    }

    pub fn duration(&self) -> f32 {
        self.beats.iter().map(|b| b.duration).sum()
    }

    /// The beat running at `time`, and whether this is its first frame.
    ///
    /// Returns `None` past the end of the script, which is how the driver knows
    /// to stop.
    pub fn at(&self, time: f32, dt: f32) -> Option<(&Beat, bool)> {
        let mut start = 0.0;
        for beat in &self.beats {
            let end = start + beat.duration;
            if time < end {
                return Some((beat, time - start < dt));
            }
            start = end;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beats_run_in_order_and_report_their_first_frame() {
        let script = Script::new()
            .then(Beat::stand(0.5))
            .then(Beat::walk(1.0))
            .then(Beat::run(0.5));
        let dt = 1.0 / 60.0;

        assert_eq!(script.at(0.0, dt).unwrap().0.label, "stand");
        assert!(script.at(0.0, dt).unwrap().1, "t=0 is a beat's first frame");
        assert!(!script.at(0.25, dt).unwrap().1);
        assert_eq!(script.at(0.6, dt).unwrap().0.label, "walk");
        assert!(script.at(1.5, dt).unwrap().1, "the run beat starts at 1.5");
        assert!(
            script.at(2.1, dt).is_none(),
            "past the end the script stops"
        );
        assert!((script.duration() - 2.0).abs() < 1e-6);
    }
}
