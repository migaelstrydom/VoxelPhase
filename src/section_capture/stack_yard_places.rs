//! Places in the Stack Yard that planned wrongly in play, captured from the
//! settled level and planned to the outcome they should have.

use std::path::Path;

use traversal_planner::plan::Limb;
use traversal_planner::scene::{Intent, Outcome};
use traversal_planner::section::StripQuery;
use traversal_planner::{Planner, PlannerSettings, TraversalScene};

use super::capture::CaptureWorld;

const STACK_YARD: &str = "levels/stack_yard.level.ron";
const RUN: f32 = 5.0;
const SPRINT: f32 = 8.0;

/// A run up to somewhere in the yard, and what a plan from there should do.
struct Place {
    name: &'static str,
    at: (f32, f32),
    heading_degrees: f32,
    expect: Outcome,
}

fn places() -> Vec<Place> {
    vec![
        // The ball pit from its south rim, between two columns of balls: the
        // trench the balls left along the rim is hopped, over the first
        // ball's bulge, onto the balls.
        Place {
            name: "ball_pit_between_columns",
            at: (53.0, 67.0),
            heading_degrees: 0.0,
            expect: Outcome::Crosses,
        },
        Place {
            name: "ball_pit_off_columns",
            at: (52.0, 67.0),
            heading_degrees: 0.0,
            expect: Outcome::Crosses,
        },
        // Along the pyramid's first step, a metre deep, with the second
        // step's face beside it: pressed 5° into the face from the middle of
        // the step and from against the face, the path slides along it.
        Place {
            name: "pyramid_step_into_the_face",
            at: (26.5, 29.6),
            heading_degrees: 85.0,
            expect: Outcome::Crosses,
        },
        Place {
            name: "pyramid_step_against_the_face",
            at: (26.5, 29.75),
            heading_degrees: 85.0,
            expect: Outcome::Crosses,
        },
        // Turned 35° toward the face, the body means to go at it.
        Place {
            name: "pyramid_step_at_the_face",
            at: (26.5, 29.6),
            heading_degrees: 55.0,
            expect: Outcome::Refuses,
        },
    ]
}

#[test]
fn stack_yard_places_plan_as_they_should() {
    let world = CaptureWorld::open(Path::new(STACK_YARD), &[]).expect("the yard opens");
    let mut wrong = Vec::new();
    for place in places() {
        for speed in [RUN, SPRINT] {
            let (captured, _) = world
                .capture(
                    place.at,
                    None,
                    place.heading_degrees,
                    speed,
                    Limb::LeftFoot,
                    None,
                )
                .expect("a floor under every start");
            let scene = TraversalScene {
                intent: Intent { speed },
                ..captured
            };
            let section = scene.section();
            let query = StripQuery::new(&section, &scene.params);
            let planner = Planner::new(scene.params.clone(), PlannerSettings::default());
            let outcome = match planner.plan(&query, &scene.request()) {
                Ok(_) => Outcome::Crosses,
                Err(_) => Outcome::Refuses,
            };
            if outcome != place.expect {
                wrong.push(format!(
                    "{} at {speed} m/s: {outcome:?}, expected {:?}",
                    place.name, place.expect
                ));
            }
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}
