//! The planner on every fixture: the outcome each states, and the properties
//! every plan keeps, replanning a frame later included; then replanning
//! against a last plan whose committed footfall has gone.

use std::fs;
use std::path::Path;

use traversal_planner::plan::{BodyState, Contact, Limb, Plan};

/// A frame at the game's 60 Hz, in seconds.
const FRAME: f32 = 1.0 / 60.0;
use traversal_planner::properties::{check_plan, check_replan};
use traversal_planner::scene::Outcome;
use traversal_planner::section::{SectionQuery, StripQuery};
use traversal_planner::{Motion, PathVec, PlanRequest, Planner, PlannerSettings, TraversalScene};

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

/// A flight a body following its plan left the ground on.
#[derive(Debug)]
struct LeftOn {
    takeoff: BodyState,
    /// Where it planned to come down, or to leave sight in a leap.
    landing: f32,
    /// Rolled off, unpushed, the stance foot left planted.
    rolled: bool,
}

/// The flights a body leaves the ground on following its plan, replanned
/// every frame against the last as the game does, from the scene's start
/// until it is `until` along the path: what it does across a scene, where
/// one plan sees only a few steps of it.
fn followed(scene: &TraversalScene, until: f32) -> Vec<LeftOn> {
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = planner(scene);
    let mut request = scene.request();
    let mut previous: Option<Plan> = None;
    let mut flying = false;
    let mut left = Vec::new();
    for _ in 0..600 {
        if request.body.s >= until {
            return left;
        }
        let outcome = planner.plan(
            &query,
            &PlanRequest {
                previous: previous.as_ref(),
                ..request
            },
        );
        let plan = match &outcome {
            Ok(planned) => &planned.plan,
            Err(refusal) => &refusal.partial,
        };
        let leaving = plan
            .phases
            .iter()
            .take_while(|phase| phase.start < 0.5 * FRAME)
            .find(|phase| matches!(phase.motion, Motion::Flight { .. }));
        if let (Some(phase), false) = (leaving, flying) {
            let Motion::Flight { takeoff, .. } = phase.motion else {
                unreachable!("a flight");
            };
            left.push(LeftOn {
                takeoff,
                landing: phase.end_s(),
                rolled: !phase.contacts.is_empty(),
            });
        }
        flying = leaving.is_some();
        request.body = plan
            .body_at(FRAME, &query)
            .unwrap_or_else(|| panic!("the plan ends at s {}", request.body.s));
        request.planted = outcome
            .as_ref()
            .ok()
            .and_then(|planned| planned.plan.stance_at(FRAME).copied());
        let later = plan.after(FRAME);
        request.lead = match request.planted {
            Some(stance) => stance.limb.other(),
            None => later
                .footfalls()
                .next()
                .map_or(request.lead, |(_, next)| next.limb),
        };
        previous = Some(later);
    }
    panic!("still short of s {until} after 10 s");
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

/// A last plan whose feet the legs did not follow is not kept: replanned
/// with the other foot landing next, or the planted foot somewhere else, the
/// plan starts over from the feet the body has.
#[test]
fn a_plan_the_legs_did_not_follow_is_not_kept() {
    let scene = fixture("zigzag_stones");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = planner(&scene);
    let first = planner
        .plan(&query, &scene.request())
        .expect("crosses")
        .plan;
    let stance = *first.stance_at(0.0).expect("a planted foot");
    let replanned = |lead: Limb, planted: Contact| {
        planner
            .plan(
                &query,
                &PlanRequest {
                    lead,
                    planted: Some(planted),
                    previous: Some(&first),
                    reach_limit: None,
                    ..scene.request()
                },
            )
            .map_or_else(|refusal| refusal.diverged, |planned| planned.diverged)
    };
    assert!(!replanned(scene.lead, stance), "kept as it stands");
    assert!(
        replanned(scene.lead.other(), stance),
        "the other foot leads"
    );
    let moved = Contact {
        at: PathVec::new(stance.at.s - 0.2, stance.at.d, stance.at.z),
        ..stance
    };
    assert!(replanned(scene.lead, moved), "planted 20 cm back");
}

/// A committed hop is kept through its takeoff: by the plan's clock the body
/// has just left the ground, while the legs still have the foot it pushed
/// off from down, the body not yet having lifted. That foot is not a
/// divergence; the plan stays, with its hop.
#[test]
fn a_hop_just_taken_off_is_kept_while_the_push_off_foot_is_down() {
    let mut scene = fixture("stack_yard_voussoir_arch");
    scene.body.speed = 5.0;
    scene.intent.speed = 5.0;
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let planner = planner(&scene);
    let first = planner
        .plan(&query, &scene.request())
        .expect("crosses")
        .plan;
    let (takeoff_at, takeoff) = first
        .phases
        .iter()
        .find_map(|phase| match phase.motion {
            Motion::Flight { takeoff, .. } => Some((phase.start, takeoff)),
            Motion::Grounded { .. } => None,
        })
        .expect("a hop off the end");
    let pushed_off = *first
        .stance_at(takeoff_at - FRAME)
        .expect("a foot to push off from");
    let just_after = takeoff_at + 0.5 * FRAME;
    let later = first.after(just_after);
    assert!(
        later.stance_at(0.0).is_none(),
        "in flight by the plan's clock"
    );
    let planned = planner
        .plan(
            &query,
            &PlanRequest {
                body: first.body_at(just_after, &query).expect("on the plan"),
                lead: pushed_off.limb.other(),
                planted: Some(pushed_off),
                previous: Some(&later),
                reach_limit: None,
                ..scene.request()
            },
        )
        .expect("still crosses");
    assert!(!planned.diverged, "the commitment is kept");
    let kept = planned
        .plan
        .phases
        .first()
        .and_then(|phase| match phase.motion {
            Motion::Flight { takeoff: kept, .. } => Some(kept),
            Motion::Grounded { .. } => None,
        })
        .expect("still in the hop");
    assert!(
        (kept.climb - takeoff.climb).abs() < 0.3,
        "the same hop: climb {} m/s, planned {}",
        kept.climb,
        takeoff.climb
    );
}

/// A steady slope as steep as the legs step up is ridden: hopping up it
/// would look cheaper only if the ride were charged for the speed the slope
/// takes, or for its climb as if it were bumps.
#[test]
fn a_steady_slope_at_the_limit_is_ridden_not_hopped() {
    let scene = fixture("slope_38deg");
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

/// Walking over the Stack Yard's voussoir arch the body runs up to the crest
/// and leaves it once, all but unpushed: from the crest, a centimetre's rise
/// at most, rather than a hop over the top stone. Running, it pops off a
/// corner on the way up by itself on the climb it carries, and the plan,
/// whose arc would come down across the next stone's corner, refuses there
/// and leaves the body to the drive, which this follower cannot replay.
#[test]
fn an_arch_is_run_over_and_left_unpushed() {
    let mut scene = fixture("stack_yard_voussoir_arch");
    for (speed, roll_offs) in [(5.0, 0)] {
        scene.body.speed = speed;
        scene.intent.speed = speed;
        let (rolled, hops): (Vec<_>, Vec<_>) = followed(&scene, ARCH_LEFT)
            .into_iter()
            .partition(|flight| flight.rolled);
        assert_eq!(
            hops.len(),
            1,
            "one jump, off the end, at {speed} m/s: {hops:?}"
        );
        assert!(
            hops[0].takeoff.climb < 0.5,
            "at {speed} m/s: {:?}",
            hops[0].takeoff
        );
        assert_eq!(rolled.len(), roll_offs, "roll-offs at {speed} m/s");
    }
}

/// Where along the voussoir arch's path the body has left it.
const ARCH_LEFT: f32 = 7.0;

/// Running at the garden's 40 cm lip, which its voxels mesh as a ramp, the
/// body hops onto the top rather than onto the ramp, where landing would
/// run into its face and lose speed.
#[test]
fn a_lip_is_hopped_onto_its_top_not_its_ramp_at_a_run() {
    let scene = fixture("garden_lip40_ramp");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let flights = followed(&scene, 6.0);
    assert_eq!(flights.len(), 1, "one hop");
    let landing = flights[0].landing;
    let floor = query.floor(landing).expect("lands on a floor");
    assert!(
        floor.normal.z > 10f32.to_radians().cos(),
        "lands at s {landing} on a floor {:.0}° steep",
        floor.normal.z.acos().to_degrees()
    );
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

/// Running at the end of the cave roof, the body leaves it at the edge,
/// walking and running: when the edge comes within its plan the landing
/// lies out of sight, and the plan leaps into it rather than off before it,
/// onto what it sees. At the edge is as near as takeoffs are tried, a
/// flight step apart, and a frame's travel.
#[test]
fn an_edge_whose_landing_is_out_of_sight_is_leapt_off_at_the_edge() {
    const EDGE: f32 = 5.55;
    let mut scene = fixture("garden_roof_edge");
    let flight_step = PlannerSettings::default().flight_step;
    for speed in [8.0, 5.0] {
        scene.body.speed = speed;
        scene.intent.speed = speed;
        let flights = followed(&scene, EDGE + 1.0);
        assert_eq!(flights.len(), 1, "at {speed} m/s");
        let takeoff = flights[0].takeoff.s;
        assert!(
            (takeoff - EDGE).abs() < flight_step + speed * FRAME,
            "at {speed} m/s: takes off at s {takeoff}"
        );
    }
}

/// Planned from 2 m short of the cave roof's end at a run, whose landing lies
/// past the strip sampled, the leap is the plan's last phase, with no
/// footfall after it.
#[test]
fn a_leap_ends_the_plan() {
    let mut scene = fixture("garden_roof_edge");
    scene.body.s = 3.5;
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let plan = planner(&scene)
        .plan(&query, &scene.request())
        .expect("crosses")
        .plan;
    let last = plan.phases.last().expect("phases");
    assert!(matches!(last.motion, Motion::Flight { .. }) && last.contacts.is_empty());
    let (time, _) = plan.footfalls().last().expect("footfalls");
    assert!(time <= last.start);
}

#[test]
fn feet_land_on_their_own_lines_on_open_ground() {
    let scene = fixture("lip_03cm");
    let section = scene.section();
    let query = StripQuery::new(&section, &scene.params);
    let plan = planner(&scene)
        .plan(&query, &scene.request())
        .expect("crosses")
        .plan;
    let off: Vec<_> = plan
        .footfalls()
        .map(|(_, contact)| {
            (
                contact.limb,
                contact.at.d - scene.params.foot.natural_offset(contact.limb),
            )
        })
        .filter(|(_, off)| off.abs() > 0.01)
        .collect();
    assert!(off.is_empty(), "footfalls off their foot's line: {off:?}");
}
