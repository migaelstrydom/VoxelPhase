use super::blow::Band;
use crate::physics::HARD_GIVE;

/// How far a blow to the player's body squeezes it before it is stopped, m.
/// Set on the player's capsule, so every hit on it carries it.
pub const FLESH_GIVE: f32 = 0.03;

/// The bands harm judges blows in, and how it reads them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HarmConfig {
    /// A sudden change of speed, m/s, from which a blow knocks the body down.
    /// Below the player's jump takeoff, so a blast that throws them higher
    /// than they can jump also knocks them down.
    pub heavy: f32,
    /// A sudden change of speed, m/s, from which a blow kills.
    pub deadly: f32,
    /// The worst band a blast's shove falls in, however close: heavy, so a
    /// blast knocks the body down and kills it only by what it throws it
    /// into, judged as any other hit.
    pub blast_at_worst: Band,
    /// The give the bands are stated for, m: the body struck against
    /// something hard. A blow spread over more is judged as the sudden one
    /// that stops as hard (`Blow::sudden`).
    pub sudden_give: f32,
    /// What bracing adds to a blow's give, m: the legs bending under a
    /// landing, the arms against a wall. A tuning value, not a measured one:
    /// a real knee bend would let a body land on its feet from any height.
    pub brace_give: f32,
    /// Seconds a blow that only just knocks the body down keeps it down, at
    /// least.
    pub least_down: f32,
    /// Seconds the hardest blows keep the body down, at least: any as
    /// sudden as `deadly` or more. Between the two, the time grows with how
    /// sudden the blow was (`time_down`).
    pub most_down: f32,
    /// Seconds between a deadly blow and the room restarting: the beat in
    /// which the player sees what killed them.
    pub restart_beat: f32,
}

impl Default for HarmConfig {
    fn default() -> Self {
        Self {
            heavy: 5.0,
            deadly: 8.0,
            blast_at_worst: Band::Heavy,
            sudden_give: FLESH_GIVE + HARD_GIVE,
            brace_give: 0.1,
            least_down: 2.0,
            most_down: 5.0,
            restart_beat: 1.0,
        }
    }
}

impl HarmConfig {
    /// The band of a blow as sudden as `sudden`, m/s.
    pub fn band(&self, sudden: f32) -> Band {
        if sudden >= self.deadly {
            Band::Deadly
        } else if sudden >= self.heavy {
            Band::Heavy
        } else {
            Band::Light
        }
    }

    /// Seconds a blow as sudden as `sudden`, m/s, keeps the body down, at
    /// least, if it knocks it down: from `least_down` at `heavy` to
    /// `most_down` at `deadly`, in proportion, and no more past it.
    pub fn time_down(&self, sudden: f32) -> f32 {
        let past_heavy = (sudden - self.heavy) / (self.deadly - self.heavy);
        self.least_down + (self.most_down - self.least_down) * past_heavy.clamp(0.0, 1.0)
    }
}
