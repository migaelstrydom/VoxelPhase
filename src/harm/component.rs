use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

use super::blow::{Band, Blow};
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
    /// Suffer `blow`: a heavy one knocks the body down, for longer the
    /// more sudden it was (`HarmConfig::time_down`); a deadly one too, and
    /// starts the beat before the room restarts. A second deadly blow does
    /// not restart the beat.
    pub fn suffer(&mut self, blow: &Blow) {
        let config = self.config;
        let time_down = config.time_down(blow.sudden(&config));
        let down = match blow.band(&config) {
            Band::Light => return,
            Band::Heavy => time_down,
            Band::Deadly => {
                self.dying.get_or_insert(config.restart_beat);
                time_down.max(config.restart_beat)
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

    /// A blow as sudden as `sudden`, m/s, from a hit.
    fn blow(sudden: f32) -> Blow {
        Blow {
            delta_v: sudden,
            give: HarmConfig::default().sudden_give,
            braced: false,
            at_worst: Band::Deadly,
        }
    }

    /// Frames until the beat runs out, counting how many times it says so.
    fn beat_frames(harm: &mut Harm, frames: usize) -> Vec<usize> {
        (0..frames).filter(|_| harm.beat_out(DT)).collect()
    }

    #[test]
    fn a_light_blow_befalls_nothing() {
        let mut harm = Harm::default();
        harm.suffer(&blow(4.0));
        assert_eq!(harm.knock, None);
        assert_eq!(harm.dying, None);
    }

    #[test]
    fn a_heavy_blow_knocks_down_for_the_longer_of_two() {
        let mut harm = Harm::default();
        harm.suffer(&blow(5.0));
        assert_eq!(harm.knock, Some(harm.config.least_down));
        harm.knock = Some(6.0);
        harm.suffer(&blow(5.0));
        assert_eq!(harm.knock, Some(6.0));
        assert_eq!(harm.dying, None);
    }

    /// The more sudden a heavy blow, the longer it keeps the body down: from
    /// the least time at the heavy threshold to the most at the deadly one,
    /// and no longer for a shove past it.
    #[test]
    fn a_harder_blow_keeps_the_body_down_longer() {
        let config = HarmConfig::default();
        let knock = |blow: Blow| {
            let mut harm = Harm::default();
            harm.suffer(&blow);
            harm.knock.unwrap()
        };
        let midway = 0.5 * (config.heavy + config.deadly);
        assert_eq!(knock(blow(config.heavy)), config.least_down);
        assert!((knock(blow(midway)) - 0.5 * (config.least_down + config.most_down)).abs() < 1e-5);
        let blast = Blow {
            at_worst: config.creatures_and_blasts_at_worst,
            ..blow(2.0 * config.deadly)
        };
        assert_eq!(knock(blast), config.most_down);
    }

    /// A deadly blow knocks the body down and the room restarts once, a
    /// second later; a second deadly blow in the beat does not put it off.
    #[test]
    fn a_deadly_blow_restarts_the_room_a_beat_later() {
        let mut harm = Harm::default();
        harm.suffer(&blow(9.0));
        assert!(harm.knock.unwrap() >= harm.config.restart_beat);
        let half = beat_frames(&mut harm, 30);
        assert!(half.is_empty());
        harm.suffer(&blow(9.0));
        let rest = beat_frames(&mut harm, 120);
        assert_eq!(rest.len(), 1, "restarted once");
        assert!(
            rest[0] + 30 >= 59 && rest[0] + 30 <= 61,
            "at frame {}",
            rest[0] + 30
        );
    }
}
