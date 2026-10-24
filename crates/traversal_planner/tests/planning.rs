//! The planner on every fixture: the outcome each states, and the rules every
//! plan keeps; then replanning against the last plan.

use std::fs;
use std::path::Path;

use traversal_planner::plan::{Limb, Plan};
use traversal_planner::scene::{Ground, Outcome};
use traversal_planner::section::{Footing, SectionQuery, StripQuery};
use traversal_planner::{PathVec, Planner, PlannerSettings, TraversalScene};

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

/// The rules every plan keeps, whatever its scene.
fn check_rules(name: &str, plan: &Plan, query: &dyn SectionQuery, scene: &TraversalScene) {
    let params = &scene.params;
    let mut last = scene.lead.other();
    for (time, contact) in plan.footfalls() {
        assert_ne!(contact.limb, last, "{name}: feet alternate (t = {time})");
        last = contact.limb;
        assert!(
            !matches!(contact.footing, Footing::Loose { resting: false, .. }),
            "{name}: no footfall on moving footing"
        );
    }
    for phase in &plan.phases {
        assert!(
            query.clears(phase),
            "{name}: phase at t = {} clears",
            phase.start
        );
    }
    for pair in plan.phases.windows(2) {
        let gap = pair[1].start - (pair[0].start + pair[0].duration);
        assert!(gap.abs() < 1e-4, "{name}: phases follow on");
    }
}

#[test]
fn every_fixture_plans_as_it_expects_and_keeps_the_rules() {
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
        check_rules(&name, plan, &query, &scene);
    }
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
    let planner = planner(&scene);
    let first = planner
        .plan(&StripQuery::new(&section, &scene.params), &scene.request())
        .expect("crosses");

    // The first stone, which the committed footfall is on, falls away.
    let mut fallen = scene.clone();
    let Ground::Authored(ground) = &mut fallen.ground else {
        panic!("authored");
    };
    let committed = first.plan.footfalls().next().expect("a footfall").1.at;
    ground.shapes.retain(|shape| {
        let footprint = shape.footprint();
        let (from, to) = footprint
            .iter()
            .fold((f32::MAX, f32::MIN), |(a, b), &(s, _)| (a.min(s), b.max(s)));
        !(from <= committed.s && committed.s <= to && from > 0.5)
    });
    let fallen_section = fallen.section();
    let outcome = planner.plan(
        &StripQuery::new(&fallen_section, &fallen.params),
        &traversal_planner::PlanRequest {
            previous: Some(&first.plan),
            ..fallen.request()
        },
    );
    let diverged = match outcome {
        Ok(planned) => planned.diverged,
        Err(refusal) => refusal.diverged,
    };
    assert!(diverged, "the committed footfall at {committed:?} is gone");
}
