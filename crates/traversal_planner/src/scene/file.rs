use serde::{Deserialize, Serialize};

use crate::params::GaitParams;
use crate::plan::{BodyState, Limb};
use crate::section::{Section, StripLayout};

use super::authored::AuthoredGround;

/// Where a scene's section comes from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Ground {
    /// Shapes, sampled into a section by the same rules as the game's.
    Authored(AuthoredGround),
    /// A section the game sampled, verbatim.
    Captured(Section),
}

/// What the player asks for.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Intent {
    /// Speed along the path, in m/s; zero to stop.
    pub speed: f32,
}

/// Everything the planner needs for one plan, as a file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TraversalScene {
    pub ground: Ground,
    pub body: BodyState,
    /// The foot that lands next.
    pub lead: Limb,
    pub intent: Intent,
    #[serde(default)]
    pub params: GaitParams,
    #[serde(default)]
    pub layout: StripLayout,
}

impl TraversalScene {
    pub fn from_ron(text: &str) -> Result<Self, ron::error::SpannedError> {
        ron::from_str(text)
    }

    pub fn to_ron(&self) -> Result<String, ron::Error> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
    }
}
