//! Running a scene forward the way the game will: the body tracks its plan
//! exactly, and the planner replans every frame against the last plan, so
//! commitment and churn can be watched over time. Loose footing can give
//! way under each foot that lands on it.

use traversal_planner::plan::{Contact, Plan};
use traversal_planner::scene::{Form, Ground, Shape, TraversalScene};
use traversal_planner::section::{Footing, StripQuery};
use traversal_planner::{PlanRequest, Planned, Planner, Refusal};

/// A frame at the game's 60 Hz, in seconds.
const FRAME: f32 = 1.0 / 60.0;
/// How far a loose stone drops and slides under a foot, in metres.
const GIVE_DROP: f32 = 0.04;
const GIVE_SLIDE: f32 = 0.03;
/// How far the next footfall must move between frames to be logged, in
/// metres.
const MOVE_LOGGED: f32 = 0.15;
/// How near a shape's top a footfall must be to be standing on it, in
/// metres.
const ON_SHAPE: f32 = 0.05;

/// The scene after running, and the plan made in its last frame.
pub struct Scrubbed {
    pub scene: TraversalScene,
    /// `None` when nothing ran.
    pub outcome: Option<Result<Planned, Refusal>>,
    pub events: Vec<String>,
}

/// Runs `scene` for `time` seconds, replanning every frame.
pub fn scrub(scene: &TraversalScene, planner: &Planner, time: f32, perturb: bool) -> Scrubbed {
    let mut scene = scene.clone();
    let mut events = Vec::new();
    let frames = (time / FRAME).round() as usize;
    if frames == 0 {
        return Scrubbed {
            scene,
            outcome: None,
            events,
        };
    }
    let (mut previous, mut planted, mut lead): (Option<Plan>, Option<Contact>, _) =
        (None, None, scene.lead);
    for frame in 0..=frames {
        let now = frame as f32 * FRAME;
        let section = scene.section();
        let query = StripQuery::new(&section, &scene.params);
        let request = PlanRequest {
            body: scene.body,
            lead,
            target_speed: scene.intent.speed,
            planted,
            previous: previous.as_ref(),
        };
        let outcome = planner.plan(&query, &request);
        let (plan, diverged) = match &outcome {
            Ok(planned) => (planned.plan.clone(), planned.diverged),
            Err(refusal) => (refusal.partial.clone(), refusal.diverged),
        };
        if diverged {
            events.push(format!("{now:.2} s: commitment dropped"));
        }
        if let Some(before) = &previous {
            let next = |plan: &Plan| plan.footfalls().nth(1).map(|(_, contact)| contact.at);
            if let (Some(before), Some(after)) = (next(before), next(&plan)) {
                let moved = (after - before).length();
                if moved > MOVE_LOGGED {
                    events.push(format!("{now:.2} s: next footfall moved {moved:.2} m"));
                }
            }
        }
        if frame == frames {
            return Scrubbed {
                scene,
                outcome: Some(outcome),
                events,
            };
        }

        let Some(body) = plan.body_at(FRAME, &query) else {
            events.push(format!("{now:.2} s: the plan ran out"));
            return Scrubbed {
                scene,
                outcome: Some(outcome),
                events,
            };
        };
        let landed: Vec<Contact> = plan
            .footfalls()
            .filter(|(time, _)| *time > 0.0 && *time <= FRAME)
            .map(|(_, contact)| *contact)
            .collect();
        let later = plan.after(FRAME);
        planted = plan.stance_at(FRAME).copied();
        lead = match planted {
            Some(stance) => stance.limb.other(),
            None => later.footfalls().next().map_or(lead, |(_, next)| next.limb),
        };
        previous = Some(later);
        scene.body = body;
        if perturb {
            for contact in landed {
                if let Some(given) = give_way(&mut scene, &contact) {
                    events.push(format!("{now:.2} s: {given}"));
                }
            }
        }
    }
    unreachable!("returns on the last frame")
}

/// Drops and slides the loose shape `contact` stands on; what gave way.
fn give_way(scene: &mut TraversalScene, contact: &Contact) -> Option<String> {
    if !matches!(contact.footing, Footing::Loose { .. }) {
        return None;
    }
    let Ground::Authored(ground) = &mut scene.ground else {
        return None;
    };
    let (index, shape) = ground.shapes.iter_mut().enumerate().find(|(_, shape)| {
        matches!(shape.footing, Footing::Loose { .. })
            && covers(shape, contact.at.s, contact.at.d)
            && (shape.face().at(contact.at.s, contact.at.d) - contact.at.z).abs() < ON_SHAPE
    })?;
    match &mut shape.form {
        Form::Block { from, to, top, .. } => {
            *top -= GIVE_DROP;
            *from += GIVE_SLIDE;
            *to += GIVE_SLIDE;
        }
        Form::Ramp {
            from,
            to,
            start_top,
            end_top,
        } => {
            *start_top -= GIVE_DROP;
            *end_top -= GIVE_DROP;
            *from += GIVE_SLIDE;
            *to += GIVE_SLIDE;
        }
        Form::Roof { .. } => return None,
    }
    Some(format!(
        "shape {index} gave way under the {:?}",
        contact.limb
    ))
}

/// Whether `(s, d)` lies over the shape's footprint.
fn covers(shape: &Shape, s: f32, d: f32) -> bool {
    let footprint = shape.footprint();
    (0..footprint.len()).all(|i| {
        let (start, end) = (footprint[i], footprint[(i + 1) % footprint.len()]);
        (end.0 - start.0) * (d - start.1) - (end.1 - start.1) * (s - start.0) >= -1.0e-4
    })
}
