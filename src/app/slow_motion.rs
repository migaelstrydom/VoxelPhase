//! F9 slows the whole game down, to watch motion closely: every system that
//! reads [`Time`] sees its scaled step, and physics substeps less often.

use crate::time::Time;

/// The speeds F9 steps through, as fractions of real time.
const SPEEDS: [f32; 3] = [1.0, 0.25, 0.1];

/// Steps `time` to the next of [`SPEEDS`].
pub fn next_speed(time: &mut Time) {
    let now = SPEEDS
        .iter()
        .position(|&speed| speed == time.scale())
        .unwrap_or(0);
    time.set_scale(SPEEDS[(now + 1) % SPEEDS.len()]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn f9_steps_through_the_speeds_and_back() {
        let mut time = Time::fixed(1.0 / 60.0);
        let mut seen = vec![time.scale()];
        for _ in 0..SPEEDS.len() {
            next_speed(&mut time);
            seen.push(time.scale());
        }
        assert_eq!(seen, [1.0, 0.25, 0.1, 1.0]);
    }
}
