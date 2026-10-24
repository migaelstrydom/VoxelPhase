use serde::{Deserialize, Serialize};

use crate::params::GaitParams;
use crate::plan::{BodyState, Limb};
use crate::planner::PlanRequest;
use crate::section::{sample_section, Section, StripLayout};

use super::authored::AuthoredGround;
use super::caster::ShapeCaster;

/// How deep a scene's RON nests before the rest of a value goes on one line:
/// a captured section's body floor samples and its foothold lanes' surfaces
/// sit at depth 7, so each is a line.
const RON_DEPTH: usize = 6;

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

/// What a scene's plan should do, for the fixtures that check it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Outcome {
    /// A plan reaches the end of the strip.
    Crosses,
    /// No plan on this heading: the right answer when nothing in reach
    /// gets the body across.
    Refuses,
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
    /// What the plan should do; every fixture states it, a fresh capture
    /// leaves it open.
    #[serde(default)]
    pub expect: Option<Outcome>,
}

impl TraversalScene {
    pub fn from_ron(text: &str) -> Result<Self, ron::error::SpannedError> {
        ron::from_str(text)
    }

    /// The scene as RON, each of a captured section's samples on a line of
    /// its own.
    pub fn to_ron(&self) -> Result<String, ron::Error> {
        let config = ron::ser::PrettyConfig::default().depth_limit(RON_DEPTH);
        ron::ser::to_string_pretty(self, config)
    }

    /// What the planner is asked for in this scene, with no plan before it.
    pub fn request(&self) -> PlanRequest<'static> {
        PlanRequest {
            body: self.body,
            lead: self.lead,
            target_speed: self.intent.speed,
            planted: None,
            previous: None,
        }
    }

    /// The section the planner reads: an authored scene sampled by the same
    /// rules as the game's, or a captured one as it was.
    pub fn section(&self) -> Section {
        match &self.ground {
            Ground::Captured(section) => section.clone(),
            Ground::Authored(ground) => sample_section(
                &ShapeCaster::new(ground),
                &self.layout,
                &self.params,
                self.body.s,
            ),
        }
    }
}
