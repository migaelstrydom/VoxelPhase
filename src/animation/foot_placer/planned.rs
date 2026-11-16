use nalgebra::Point3;

use super::placer::FootSide;

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
