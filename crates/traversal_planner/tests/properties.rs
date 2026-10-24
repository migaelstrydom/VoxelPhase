//! Every property shown able to fail: a good plan, broken on purpose, must be
//! caught by its rule.

use std::fs;
use std::path::Path;

use traversal_planner::plan::{Contact, Motion, Plan};
use traversal_planner::properties::{check_plan, check_replan, Rule};
use traversal_planner::section::{Footing, Section, StripQuery};
use traversal_planner::{PathVec, Planner, PlannerSettings, TraversalScene};

const FRAME: f32 = 1.0 / 60.0;

struct Case {
    scene: TraversalScene,
    section: Section,
}

impl Case {
    fn new(name: &str) -> Self {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures")
            .join(name)
            .with_extension("ron");
        let scene =
            TraversalScene::from_ron(&fs::read_to_string(path).expect("fixture")).expect("parses");
        let section = scene.section();
        Self { scene, section }
    }

    fn query(&self) -> StripQuery<'_> {
        StripQuery::new(&self.section, &self.scene.params)
    }

    fn planner(&self) -> Planner {
        Planner::new(self.scene.params.clone(), PlannerSettings::default())
    }

    fn plan(&self) -> Plan {
        match self.planner().plan(&self.query(), &self.scene.request()) {
            Ok(planned) => planned.plan,
            Err(refusal) => refusal.partial,
        }
    }

    fn rules_broken(&self, plan: &Plan) -> Vec<Rule> {
        check_plan(plan, &self.query(), &self.scene.params)
            .into_iter()
            .map(|violation| violation.rule)
            .collect()
    }
}

/// `plan` with its `index`th footfall changed by `change`, wherever it
/// holds the body.
fn with_footfall(plan: &Plan, index: usize, change: impl Fn(&mut Contact)) -> Plan {
    let target = *plan.footfalls().nth(index).expect("that many footfalls").1;
    let mut broken = plan.clone();
    for contact in broken
        .phases
        .iter_mut()
        .flat_map(|phase| phase.contacts.iter_mut())
        .filter(|contact| **contact == target)
    {
        change(contact);
    }
    broken
}

#[test]
fn the_plans_broken_below_start_out_whole() {
    for name in ["zigzag_stones", "vertical_wall", "lip_40cm"] {
        let case = Case::new(name);
        assert!(case.rules_broken(&case.plan()).is_empty(), "{name}");
    }
}

#[test]
fn a_foot_landing_twice_running_is_caught() {
    let case = Case::new("zigzag_stones");
    let plan = case.plan();
    let broken = with_footfall(&plan, 1, |contact| contact.limb = contact.limb.other());
    assert!(case.rules_broken(&broken).contains(&Rule::FeetAlternate));
}

#[test]
fn a_footfall_over_the_void_is_caught() {
    let case = Case::new("zigzag_stones");
    let broken = with_footfall(&case.plan(), 0, |contact| {
        contact.at = PathVec::new(0.6, contact.at.d, -1.0)
    });
    let rules = case.rules_broken(&broken);
    assert!(rules.contains(&Rule::OnFoothold), "{rules:?}");
    assert!(rules.contains(&Rule::FootReachesBody), "{rules:?}");
}

#[test]
fn a_footfall_on_moving_rubble_is_caught() {
    let case = Case::new("zigzag_stones");
    let broken = with_footfall(&case.plan(), 0, |contact| {
        contact.footing = Footing::Loose {
            mass: 40.0,
            resting: false,
        }
    });
    assert!(case.rules_broken(&broken).contains(&Rule::FootingHolds));
}

#[test]
fn a_step_past_the_stride_is_caught() {
    let case = Case::new("lip_40cm");
    let plan = case.plan();
    let broken = with_footfall(&plan, 1, |contact| contact.at.s += 3.0);
    assert!(case.rules_broken(&broken).contains(&Rule::WithinStride));
    let sideways = with_footfall(&plan, 1, |contact| contact.at.d = -contact.at.d * 4.0);
    assert!(case.rules_broken(&sideways).contains(&Rule::WithinStride));
}

#[test]
fn a_takeoff_past_the_gait_is_caught() {
    let case = Case::new("lip_40cm");
    let mut broken = case.plan();
    let flight = broken
        .phases
        .iter_mut()
        .find_map(|phase| match &mut phase.motion {
            Motion::Flight { takeoff, .. } => Some(takeoff),
            Motion::Grounded { .. } => None,
        })
        .expect("a hop");
    flight.climb = 5.0;
    let rules = case.rules_broken(&broken);
    assert!(rules.contains(&Rule::ClimbWithinLimits), "{rules:?}");
}

#[test]
fn riding_through_a_wall_is_caught() {
    let case = Case::new("vertical_wall");
    let mut broken = case.plan();
    let last = broken.phases.last_mut().expect("a phase");
    if let Motion::Grounded { to, .. } = &mut last.motion {
        *to = 2.5;
    }
    assert!(case.rules_broken(&broken).contains(&Rule::PhaseClears));
}

#[test]
fn a_gap_between_phases_is_caught() {
    let case = Case::new("zigzag_stones");
    let mut broken = case.plan();
    broken.phases[1].start += 0.1;
    assert!(case.rules_broken(&broken).contains(&Rule::PhasesFollowOn));
}

/// A planner whose churn costs nothing and whose commitment allows any move,
/// so a replan is free to move every footfall.
fn careless(case: &Case) -> Planner {
    let mut settings = PlannerSettings::default();
    settings.costs.churn = 0.0;
    settings.commit_tolerance = 100.0;
    Planner::new(case.scene.params.clone(), settings)
}

#[test]
fn a_replan_that_moves_footfalls_on_the_same_ground_is_caught() {
    let case = Case::new("zigzag_stones");
    let plan = case.plan();
    // Last frame's plan, with its tentative footfalls moved: a careless
    // replan lands where it likes, which is not where this plan said.
    let moved = with_footfall(&plan, 1, |contact| contact.at.s -= 0.3);
    let rules: Vec<Rule> = check_replan(
        &careless(&case),
        &case.query(),
        &case.scene.request(),
        &moved,
        FRAME,
    )
    .into_iter()
    .map(|violation| violation.rule)
    .collect();
    assert!(rules.contains(&Rule::TentativeStays), "{rules:?}");
}

#[test]
fn a_replan_that_drops_its_commitment_is_caught() {
    let case = Case::new("zigzag_stones");
    let plan = case.plan();
    let gone = with_footfall(&plan, 0, |contact| {
        contact.at = PathVec::new(0.6, 0.15, -1.0)
    });
    let rules: Vec<Rule> = check_replan(
        &case.planner(),
        &case.query(),
        &case.scene.request(),
        &gone,
        FRAME,
    )
    .into_iter()
    .map(|violation| violation.rule)
    .collect();
    assert!(rules.contains(&Rule::KeepsCommitment), "{rules:?}");
}
