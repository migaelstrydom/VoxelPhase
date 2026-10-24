//! Every Traversal Garden station, captured from the settled level and
//! planned: every plan keeps the planner's rules, and where the station has
//! an authored twin, the two plan alike.

use std::path::Path;

use traversal_planner::plan::{BodyState, Limb, Motion, Plan};
use traversal_planner::properties::check_plan;
use traversal_planner::scene::{AuthoredGround, Form, Ground, Intent, Shape};
use traversal_planner::section::{Footing, StripQuery};
use traversal_planner::{Planned, Planner, PlannerSettings, Refusal, TraversalScene};

use super::capture::CaptureWorld;

const GARDEN: &str = "levels/traversal_garden.level.ron";
/// Every lane runs along +x.
const HEADING_DEGREES: f32 = 90.0;
/// Where a station's face is, in x; the bridges' decks end at `BRIDGE_X`.
const FACE_X: f32 = 7.0;
const BRIDGE_X: f32 = 3.0;
/// How far before the face the body starts, in metres.
const RUN_UP: f32 = 1.5;
const WALK: f32 = 5.0;
const SPRINT: f32 = 8.0;
/// How far the first footfall of a capture's plan may be from its twin's.
const FOOTFALL_TOLERANCE: f32 = 0.3;
/// The garden's voxel size, in metres.
const VOXEL: f32 = 0.25;
/// Far enough along the path to outlast any strip.
const FAR: f32 = 40.0;

/// A station: its lane, where the run up to it starts, and the ground it
/// was authored as, where it has a twin.
struct Station {
    name: &'static str,
    lane_z: f32,
    start_x: f32,
    twin: Option<Vec<Form>>,
}

/// A lip as the garden's quarter-metre voxels mesh it: no upright face,
/// but a slope across the voxels either side of where the face was
/// authored.
fn lip(name: &'static str, lane_z: f32, height: f32) -> Station {
    station(
        name,
        lane_z,
        vec![
            Form::Ramp {
                from: RUN_UP - VOXEL,
                to: RUN_UP + VOXEL,
                start_top: 0.0,
                end_top: height,
            },
            Form::Block {
                from: RUN_UP + VOXEL,
                to: FAR,
                top: height,
                tilt: 0.0,
            },
        ],
    )
}

fn slope(name: &'static str, lane_z: f32, length: f32, height: f32) -> Station {
    station(
        name,
        lane_z,
        vec![
            Form::Ramp {
                from: RUN_UP,
                to: RUN_UP + length,
                start_top: 0.0,
                end_top: height,
            },
            Form::Block {
                from: RUN_UP + length,
                to: FAR,
                top: height,
                tilt: 0.0,
            },
        ],
    )
}

fn station(name: &'static str, lane_z: f32, twin: Vec<Form>) -> Station {
    Station {
        name,
        lane_z,
        start_x: FACE_X - RUN_UP,
        twin: Some(twin),
    }
}

fn captured_only(name: &'static str, lane_z: f32, start_x: f32) -> Station {
    Station {
        name,
        lane_z,
        start_x,
        twin: None,
    }
}

fn stations() -> Vec<Station> {
    vec![
        lip("lip3", -12.0, 0.03),
        lip("lip8", -8.0, 0.08),
        lip("lip15", -4.0, 0.15),
        lip("lip25", 0.0, 0.25),
        lip("lip40", 4.0, 0.40),
        captured_only("planks", 10.0, BRIDGE_X - RUN_UP),
        captured_only("planks_wide", 18.0, BRIDGE_X - RUN_UP),
        captured_only("rough", 26.0, FACE_X - RUN_UP),
        slope("slope40", 34.0, 8.0, 6.71),
        slope("slope50", 42.0, 6.0, 7.15),
        captured_only("scree", 50.0, FACE_X - RUN_UP),
        station(
            "ledge",
            58.0,
            vec![Form::Block {
                from: RUN_UP,
                to: RUN_UP + 8.0,
                top: 0.7,
                tilt: 0.0,
            }],
        ),
        station(
            "cave_lip",
            66.0,
            vec![Form::Roof {
                from: RUN_UP,
                to: RUN_UP + 12.0,
                bottom: 0.9,
            }],
        ),
    ]
}

/// The authored twin of `captured`: the same body, intent and gait over
/// `forms` on flat ground at zero.
fn twin(captured: &TraversalScene, forms: &[Form]) -> TraversalScene {
    let shapes = forms
        .iter()
        .map(|form| Shape {
            form: form.clone(),
            across: None,
            yaw: 0.0,
            footing: Footing::Fixed,
        })
        .collect();
    TraversalScene {
        ground: Ground::Authored(AuthoredGround {
            base: Some(0.0),
            shapes,
        }),
        body: BodyState {
            z: captured.params.body.radius,
            ..captured.body
        },
        ..captured.clone()
    }
}

/// What a plan did, as the comparison reads it.
#[derive(Debug)]
struct Summary {
    crosses: bool,
    flights: usize,
    first_footfall: Option<f32>,
    /// The planner's rules the plan breaks.
    broken: Vec<String>,
}

fn summarise(scene: &TraversalScene) -> Summary {
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = Planner::new(scene.params.clone(), PlannerSettings::default());
    let outcome: Result<Planned, Refusal> = planner.plan(&query, &scene.request());
    let plan: &Plan = match &outcome {
        Ok(planned) => &planned.plan,
        Err(refusal) => &refusal.partial,
    };
    let first_footfall = plan.footfalls().next().map(|(_, contact)| contact.at.s);
    let broken = check_plan(plan, &query, &scene.params)
        .iter()
        .map(|violation| format!("{violation:?}"))
        .collect();
    Summary {
        crosses: outcome.is_ok(),
        flights: plan
            .phases
            .iter()
            .filter(|phase| matches!(phase.motion, Motion::Flight { .. }))
            .count(),
        first_footfall,
        broken,
    }
}

/// Where a capture's plan breaks the planner's rules or plans unlike its
/// twin. A refusal is compared by its outcome alone: how far its partial
/// plan got depends on the order the search went in.
fn disagreement(captured: &Summary, authored: Option<&Summary>) -> Option<String> {
    if !captured.broken.is_empty() {
        return Some(format!("breaks {:?}", captured.broken));
    }
    let authored = authored?;
    let footfalls_apart = match (captured.first_footfall, authored.first_footfall) {
        (Some(a), Some(b)) => (a - b).abs() > FOOTFALL_TOLERANCE,
        (a, b) => a.is_some() != b.is_some(),
    };
    let differs = captured.crosses != authored.crosses
        || (captured.crosses
            && (captured.flights.abs_diff(authored.flights) > 1 || footfalls_apart));
    differs.then(|| format!("captured {captured:?}, authored {authored:?}"))
}

#[test]
fn garden_station_captures_plan_like_their_authored_twins() {
    let world = CaptureWorld::open(Path::new(GARDEN), &[]).expect("the garden opens");
    let mut broken = Vec::new();
    for station in stations() {
        for speed in [WALK, SPRINT] {
            let (captured, _) = world
                .capture(
                    (station.start_x, station.lane_z),
                    HEADING_DEGREES,
                    speed,
                    Limb::LeftFoot,
                    None,
                )
                .expect("a floor under every start");
            let captured = TraversalScene {
                intent: Intent { speed },
                ..captured
            };
            let authored = station
                .twin
                .as_ref()
                .map(|forms| summarise(&twin(&captured, forms)));
            let summary = summarise(&captured);
            println!("{} at {speed}: {summary:?} / {authored:?}", station.name);
            if let Some(why) = disagreement(&summary, authored.as_ref()) {
                broken.push(format!("{} at {speed} m/s: {why}", station.name));
            }
        }
    }
    assert!(broken.is_empty(), "{}", broken.join("\n"));
}
