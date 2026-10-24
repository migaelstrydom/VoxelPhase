use std::time::Instant;

use crate::planner::{Planner, PlannerSettings};
use crate::properties::{check_plan, check_replan, Violation};
use crate::scene::{Outcome, TraversalScene};
use crate::section::StripQuery;

/// A frame at the game's 60 Hz, in seconds: how far on the replan check
/// replans.
const FRAME: f32 = 1.0 / 60.0;

/// What planning one scene did.
#[derive(Clone, Debug)]
pub struct Trial {
    pub outcome: Outcome,
    pub violations: Vec<Violation>,
    /// Seconds the plan took.
    pub seconds: f64,
}

/// Plans `scene` and checks the plan, or a refusal's partial plan, against
/// every property, and a plan's replan a frame later.
pub fn try_scene(scene: &TraversalScene) -> Trial {
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = Planner::new(scene.params.clone(), PlannerSettings::default());
    let request = scene.request();
    let start = Instant::now();
    let outcome = planner.plan(&query, &request);
    let seconds = start.elapsed().as_secs_f64();
    match outcome {
        Ok(planned) => {
            let mut violations = check_plan(&planned.plan, &query, &scene.params);
            violations.extend(check_replan(
                &planner,
                &query,
                &request,
                &planned.plan,
                FRAME,
            ));
            Trial {
                outcome: Outcome::Crosses,
                violations,
                seconds,
            }
        }
        Err(refusal) => Trial {
            outcome: Outcome::Refuses,
            violations: check_plan(&refusal.partial, &query, &scene.params),
            seconds,
        },
    }
}
