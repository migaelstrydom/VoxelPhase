//! The planner on every fixture: the outcome each states, and the properties
//! every plan keeps, replanning a frame later included; then replanning
//! against a last plan whose committed footfall has gone.

use std::fs;
use std::path::Path;

use traversal_planner::plan::{Limb, Plan};

/// A frame at the game's 60 Hz, in seconds.
const FRAME: f32 = 1.0 / 60.0;
use traversal_planner::properties::{check_plan, check_replan};
use traversal_planner::scene::Outcome;
use traversal_planner::section::StripQuery;
use traversal_planner::{PathVec, Planner, PlannerSettings, Slowing, TraversalScene};

fn fixture(name: &str) -> TraversalScene {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures")
        .join(name)
        .with_extension("ron");
    TraversalScene::from_ron(&fs::read_to_string(&path).expect("fixture")).expect("parses")
}

fn fixture_names() -> Vec<String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures");
    let mut names: Vec<_> = fs::read_dir(dir)
        .expect("fixtures")
        .map(|entry| entry.expect("entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "ron"))
        .map(|path| {
            path.file_stem()
                .expect("stem")
                .to_string_lossy()
                .into_owned()
        })
        .collect();
    names.sort();
    names
}

fn planner(scene: &TraversalScene) -> Planner {
    Planner::new(scene.params.clone(), PlannerSettings::default())
}

#[test]
fn every_fixture_plans_as_it_expects_and_keeps_the_rules() {
    let mut broken = Vec::new();
    for name in fixture_names() {
        let scene = fixture(&name);
        let section = scene.section();
        let query = StripQuery::new(&section, &scene.params);
        let outcome = planner(&scene).plan(&query, &scene.request());
        let plan = match (&outcome, scene.expect.expect("fixtures state an outcome")) {
            (Ok(planned), Outcome::Crosses) => &planned.plan,
            (Err(refusal), Outcome::Refuses) => &refusal.partial,
            (outcome, expected) => panic!(
                "{name}: expected {expected:?}, got {}",
                if outcome.is_ok() {
                    "a plan"
                } else {
                    "a refusal"
                }
            ),
        };
        let mut violations = check_plan(plan, &query, &scene.params);
        if outcome.is_ok() {
            violations.extend(check_replan(
                &planner(&scene),
                &query,
                &scene.request(),
                plan,
                FRAME,
            ));
        }
        broken.extend(
            violations
                .iter()
                .map(|violation| format!("{name}: {violation}")),
        );
    }
    assert!(broken.is_empty(), "{}", broken.join("\n"));
}

#[test]
fn replanning_against_its_own_plan_keeps_it() {
    let scene = fixture("zigzag_stones");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = planner(&scene);
    let first = planner.plan(&query, &scene.request()).expect("crosses");
    let again = planner
        .plan(
            &query,
            &traversal_planner::PlanRequest {
                previous: Some(&first.plan),
                reach_limit: None,
                ..scene.request()
            },
        )
        .expect("crosses");
    assert!(!again.diverged);
    let footfalls = |plan: &Plan| -> Vec<(Limb, PathVec)> {
        plan.footfalls()
            .map(|(_, contact)| (contact.limb, contact.at))
            .collect()
    };
    assert_eq!(footfalls(&again.plan), footfalls(&first.plan));
}

#[test]
fn a_committed_footfall_that_has_gone_is_dropped() {
    let scene = fixture("zigzag_stones");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = planner(&scene);
    let mut previous = planner
        .plan(&query, &scene.request())
        .expect("crosses")
        .plan;

    // The committed footfall's ground fell away: it now hangs over the void
    // between the first two stones.
    let gone = PathVec::new(0.6, 0.15, -1.0);
    let landing = previous
        .phases
        .iter_mut()
        .skip(1)
        .flat_map(|phase| phase.contacts.iter_mut())
        .find(|contact| contact.limb == scene.lead)
        .expect("the first footfall");
    landing.at = gone;
    assert_eq!(previous.footfalls().next().expect("a footfall").1.at, gone);

    let outcome = planner.plan(
        &query,
        &traversal_planner::PlanRequest {
            previous: Some(&previous),
            reach_limit: None,
            ..scene.request()
        },
    );
    let diverged = match outcome {
        Ok(planned) => planned.diverged,
        Err(refusal) => refusal.diverged,
    };
    assert!(diverged, "the committed footfall at {gone:?} is gone");
}

/// A steady slope the body can ride is ridden: hopping up it would look
/// cheaper only if the ride were charged for the speed the slope takes, or
/// for its climb as if it were bumps.
#[test]
fn a_steady_slope_at_the_limit_is_ridden_not_hopped() {
    let scene = fixture("slope_40deg");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    for speed in [3.0, 5.0, 8.0] {
        let request = traversal_planner::PlanRequest {
            target_speed: speed,
            ..scene.request()
        };
        let planned = planner(&scene)
            .plan(&query, &request)
            .unwrap_or_else(|_| panic!("refused at {speed} m/s"));
        let hops = planned
            .plan
            .phases
            .iter()
            .filter(|phase| matches!(phase.motion, traversal_planner::Motion::Flight { .. }))
            .count();
        assert_eq!(hops, 0, "at {speed} m/s");
    }
}

/// Over the Stack Yard's voussoir arch the body runs the crest, its seams
/// 12° corners it skims, and jumps off the far side at speed rather than
/// braking to drop off within a stride.
#[test]
fn an_arch_is_run_over_and_jumped_off_at_speed() {
    let mut scene = fixture("stack_yard_voussoir_arch");
    for speed in [5.0, 8.0] {
        scene.body.speed = speed;
        scene.intent.speed = speed;
        let section = scene.section();
        let query = StripQuery::new(&section, &scene.params);
        let planned = planner(&scene)
            .plan(&query, &scene.request())
            .unwrap_or_else(|_| panic!("refused at {speed} m/s"));
        let flights: Vec<_> = planned
            .plan
            .phases
            .iter()
            .filter_map(|phase| match phase.motion {
                traversal_planner::Motion::Flight { takeoff, .. } => Some(takeoff),
                _ => None,
            })
            .collect();
        assert_eq!(flights.len(), 1, "one jump, off the end, at {speed} m/s");
        assert!(
            flights[0].speed > 0.9 * speed && flights[0].climb < 0.5,
            "at {speed} m/s: {:?}",
            flights[0]
        );
    }
}

/// The slowest the body is planned to ride across `plan`, in m/s.
fn slowest_ride(plan: &Plan) -> f32 {
    plan.phases
        .iter()
        .filter_map(|phase| match phase.motion {
            traversal_planner::Motion::Grounded { end_speed, .. } => Some(end_speed),
            _ => None,
        })
        .fold(f32::MAX, f32::min)
}

/// Over the garden's plank bridge the costs would brake the body to a crawl
/// to step onto it; a plan that may not slow, or only to avoid failing,
/// crosses it at speed, keeping every rule.
#[test]
fn a_bridge_is_crossed_at_speed_unless_slowing_is_left_to_the_costs() {
    let scene = fixture("plank_bridge_middle");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let speed = scene.request().target_speed;
    for slowing in [Slowing::Never, Slowing::ToAvoidFailure, Slowing::ByCost] {
        let settings = PlannerSettings {
            slowing,
            ..PlannerSettings::default()
        };
        let planned = Planner::new(scene.params.clone(), settings)
            .plan(&query, &scene.request())
            .unwrap_or_else(|_| panic!("refused with {slowing:?}"));
        let slowest = slowest_ride(&planned.plan);
        assert!(
            (slowest > 0.9 * speed) == (slowing != Slowing::ByCost),
            "{slowing:?}: slowest {slowest} m/s of {speed}"
        );
        assert!(check_plan(&planned.plan, &query, &scene.params).is_empty());
    }
}

/// Standing on the steep bevel at a wall's foot and heading away from the
/// wall, the body rides down it: steeper than the drive climbs, but down a
/// face the feet stand on gravity carries it.
#[test]
fn a_steep_bevel_underfoot_is_walked_down_not_hopped() {
    let scene = fixture("wall_foot_bevel");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let plan = match planner(&scene).plan(&query, &scene.request()) {
        Ok(planned) => planned.plan,
        Err(refusal) => refusal.partial,
    };
    assert!(plan
        .phases
        .iter()
        .all(|phase| matches!(phase.motion, traversal_planner::Motion::Grounded { .. })));
    assert!(plan.phases.last().is_some_and(|phase| phase.end_s() > 1.5));
}
