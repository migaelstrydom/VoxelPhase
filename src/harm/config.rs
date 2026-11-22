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
    /// The give the bands are stated for, m: the body struck against
    /// something hard. A blow spread over more is judged as the sudden one
    /// that stops as hard (`Blow::sudden`).
    pub sudden_give: f32,
    /// What bracing adds to a blow's give, m: the legs bending under a
    /// landing, the arms against a wall. A tuning value, not a measured one:
    /// a real knee bend would let a body land on its feet from any height.
    pub brace_give: f32,
    /// Seconds a heavy blow keeps the body down, at least.
    pub down_seconds: f32,
    /// Seconds between a deadly blow and the room restarting: the beat in
    /// which the player sees what killed them.
    pub restart_beat: f32,
}

impl Default for HarmConfig {
    fn default() -> Self {
        Self {
            heavy: 5.0,
            deadly: 8.0,
            sudden_give: FLESH_GIVE + HARD_GIVE,
            brace_give: 0.1,
            down_seconds: 2.0,
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
}
