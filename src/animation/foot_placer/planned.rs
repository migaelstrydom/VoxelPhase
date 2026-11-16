use nalgebra::{Point3, Vector2};

use super::capture_point::stance_offset;
use super::placer::{facing_from_yaw, hip_position, max_leg_extension, FootSide, PlacerCtx};
use super::timing::reach_budget;

/// How far onto the other foot's side of the body a planned step may lie
/// and still be this foot's to land, in metres: feet land near the body's
/// midline on a plank or in a tight turn, and never well across it.
const OWN_SIDE_MARGIN: f32 = 0.03;

/// The longest a swing takes to a planned step a flight comes before, in
/// seconds: a hop's flight and the stride after it. A step the body takes
/// longer to reach is not this swing's to land.
const LONGEST_FLOWN_SWING: f32 = 0.8;

/// How far short of a planned step, past the leg's reach, the body may
/// still be when the foot lands on it, in metres: the legs land a footfall
/// in their own time, and wait over it a moment for the body; air steering
/// and a plan a frame old put a flight's landing a stride out.
const STEP_SHORTFALL: f32 = 0.5;

/// How far above the hip, and how far below the leg's reach under it, a
/// planned step may lie and still be one to swing to, in metres: a body
/// rises and falls through a hop, and its landing lies off the leg's reach
/// until it comes down. A step further off is not one this body is about to
/// stand on.
const STEP_HEIGHT_SLACK: f32 = 0.6;

/// Where a plan of the way ahead has one foot land next.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlannedStep {
    /// The plan's name for this footfall, which the legs hand back when a
    /// foot lands on it; `None` for a step no plan names.
    pub id: Option<StepId>,
    /// The landing, in the world: the sole's contact point.
    pub at: Point3<f32>,
    /// Seconds from the start of the frame until the body comes over it,
    /// where a flight comes before it: a flight's landing, and the footfall
    /// the foot that pushed off comes down on after it. The foot lands as
    /// the body gets there. `None` where the legs step onto it on their own
    /// clock, as they do every step on the ground.
    pub landing: Option<f32>,
}

/// A plan's name for one of its footfalls: opaque to the legs, which only
/// carry it from the step they were handed to the foot that lands on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StepId(pub u32);

/// Where a plan has each foot land next. A foot with a planned step lands
/// on it rather than where the capture point would put it, if the body is
/// coming to where the leg can ([`PlannedStep::landing`] aside, the plan
/// says where and the legs say when: a foot lifts on the gait's own clock).
/// A foot without one steps as the capture point has it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlannedSteps {
    pub left: Option<PlannedStep>,
    pub right: Option<PlannedStep>,
}

impl PlannedSteps {
    /// The step planned for `side`.
    pub fn of(&self, side: FootSide) -> Option<PlannedStep> {
        match side {
            FootSide::Left => self.left,
            FootSide::Right => self.right,
        }
    }

    /// The same steps `elapsed` seconds into the frame.
    pub fn later(&self, elapsed: f32) -> Self {
        let later = |step: Option<PlannedStep>| {
            step.map(|step| PlannedStep {
                landing: step.landing.map(|landing| landing - elapsed),
                ..step
            })
        };
        Self {
            left: later(self.left),
            right: later(self.right),
        }
    }
}

/// The plan's next footfall for `side` and the seconds until the foot lands
/// on it, if it is one this swing can land; `None`, and the legs step as
/// they would with no plan, if it is not.
///
/// The foot lands in the swing's own time, `own`; on a step a flight comes
/// before ([`PlannedStep::landing`]), as the body comes down on it. So a
/// hop's landing foot is in the air for the hop.
///
/// A step is not this swing's to land where the body is not coming to it:
/// - it lies further to the side of the body's travel than the leg reaches;
/// - the hip will have passed it, by the time the foot lands, by more than
///   the leg reaches back: a plan laid along a heading the body has not yet
///   turned to, as when the stick is swung back, lands the feet behind a body
///   still travelling the old way;
/// - it lies on the other foot's side of the body ([`OWN_SIDE_MARGIN`]): a
///   plan is laid along the way the body travels, and a body that faces
///   another way (turned to throw behind it as it walks on) has its left
///   foot where the plan's right one goes;
/// - the body comes down on it later than [`LONGEST_FLOWN_SWING`] from now;
/// - the body will still be strides short of it when the foot lands
///   ([`STEP_SHORTFALL`]);
/// - it lies higher over the hip or further under the leg's reach than a
///   hop's rise and fall account for ([`STEP_HEIGHT_SLACK`]).
///
/// A footfall a stride ahead is the plan's, though the leg stretches to it:
/// the legs land it in their own time, and a foot that gets there before
/// the body waits over it.
pub(super) fn planned_landing(
    ctx: &PlacerCtx<'_>,
    side: FootSide,
    own: f32,
) -> Option<(PlannedStep, f32)> {
    let step = ctx.planned.of(side)?;
    let lands_in = step.landing.unwrap_or(own);
    if lands_in > LONGEST_FLOWN_SWING {
        return None;
    }
    let facing = facing_from_yaw(ctx.yaw);
    let own_side = stance_offset(facing, 1.0, side.side_sign());
    let from_pelvis = step.at - ctx.pelvis;
    if Vector2::new(from_pelvis.x, from_pelvis.z).dot(&own_side) < -OWN_SIDE_MARGIN {
        return None;
    }
    let hip = hip_position(side, ctx, facing);
    let offset = step.at - hip;
    let extension = max_leg_extension(ctx);
    if offset.y > STEP_HEIGHT_SLACK || offset.y < -(extension + STEP_HEIGHT_SLACK) {
        return None;
    }
    let reach = reach_budget(extension, ctx.standing_height);
    let across = Vector2::new(offset.x, offset.z);
    let travel = Vector2::new(ctx.velocity.x, ctx.velocity.z);
    let Some(ahead) = travel.try_normalize(1.0e-3) else {
        return Some((step, lands_in));
    };
    let along = across.dot(&ahead);
    let aside = (across - ahead * along).magnitude();
    let passed_by = travel.magnitude() * lands_in.max(0.0) - along;
    // A step is where the body is when the foot lands: one the body,
    // carried on as it moves, is still strides short of by then is a later
    // step's, or another flight's.
    let short_of = -passed_by;
    (aside <= reach && passed_by <= reach && short_of <= reach + STEP_SHORTFALL)
        .then_some((step, lands_in))
}
