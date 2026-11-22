use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

use super::blow::Band;
use super::config::HarmConfig;

/// A character harm can befall: how its blows are judged, and what has
/// befallen it. Only the player has one, so a deadly blow restarts the room;
/// what harm does to creatures is for later.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct Harm {
    /// The bands, and how blows are read.
    pub config: HarmConfig,
    /// A knockdown for the character to take, at least this many seconds
    /// down: set by `HarmSystem`, taken by `CharacterControlSystem`.
    pub knock: Option<f32>,
    /// Seconds left before the room restarts, after a deadly blow.
    pub dying: Option<f32>,
    /// The body's velocity at the end of the last frame: what it moved into
    /// this frame's blows at.
    pub last_velocity: Vector3<f32>,
    /// The worst blow of the last frame that had one, m/s sudden, and its
    /// band: for the debug readout.
    pub last_blow: Option<(f32, Band)>,
}

impl Harm {
    /// Suffer a blow in `band`: a heavy one knocks the body down, a deadly
    /// one too, and starts the beat before the room restarts. A second
    /// deadly blow does not restart the beat.
    pub fn suffer(&mut self, band: Band) {
        let down = match band {
            Band::Light => return,
            Band::Heavy => self.config.down_seconds,
            Band::Deadly => {
                self.dying.get_or_insert(self.config.restart_beat);
                self.config.down_seconds.max(self.config.restart_beat)
            }
        };
        self.knock = Some(self.knock.map_or(down, |knock| knock.max(down)));
    }

    /// Count the beat down by `dt`: whether it ran out this frame.
    pub fn beat_out(&mut self, dt: f32) -> bool {
        let Some(left) = self.dying else {
            return false;
        };
        let left = left - dt;
        self.dying = Some(left);
        left <= 0.0 && left + dt > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// Frames until the beat runs out, counting how many times it says so.
    fn beat_frames(harm: &mut Harm, frames: usize) -> Vec<usize> {
        (0..frames).filter(|_| harm.beat_out(DT)).collect()
    }

    #[test]
    fn a_light_blow_befalls_nothing() {
        let mut harm = Harm::default();
        harm.suffer(Band::Light);
        assert_eq!(harm.knock, None);
        assert_eq!(harm.dying, None);
    }

    #[test]
    fn a_heavy_blow_knocks_down_for_the_longer_of_two() {
        let mut harm = Harm::default();
        harm.suffer(Band::Heavy);
        assert_eq!(harm.knock, Some(harm.config.down_seconds));
        harm.knock = Some(5.0);
        harm.suffer(Band::Heavy);
        assert_eq!(harm.knock, Some(5.0));
        assert_eq!(harm.dying, None);
    }

    /// A deadly blow knocks the body down and the room restarts once, a
    /// second later; a second deadly blow in the beat does not put it off.
    #[test]
    fn a_deadly_blow_restarts_the_room_a_beat_later() {
        let mut harm = Harm::default();
        harm.suffer(Band::Deadly);
        assert!(harm.knock.unwrap() >= harm.config.restart_beat);
        let half = beat_frames(&mut harm, 30);
        assert!(half.is_empty());
        harm.suffer(Band::Deadly);
        let rest = beat_frames(&mut harm, 120);
        assert_eq!(rest.len(), 1, "restarted once");
        assert!(
            rest[0] + 30 >= 59 && rest[0] + 30 <= 61,
            "at frame {}",
            rest[0] + 30
        );
    }
}
