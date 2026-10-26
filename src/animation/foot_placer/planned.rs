use nalgebra::Point3;

use super::placer::FootSide;

/// Where a plan of the way ahead has one foot land next.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlannedStep {
    /// The landing, in the world: the sole's contact point.
    pub at: Point3<f32>,
    /// Seconds from the start of the frame until the body comes down on
    /// it, where the step ends a flight: the foot lands as the body does.
    /// `None` where the legs step onto it on their own clock.
    pub landing: Option<f32>,
}

/// Where a plan has each foot land next. A foot with a planned step lands
/// on it rather than where the capture point would put it, lifting on the
/// gait's own clock; one without steps as the capture point has it.
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
