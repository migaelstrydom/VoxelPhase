use crate::plan::{BodyState, Contact};

/// The plan at a footfall, which is where the search branches: the foot that
/// just landed, and the body over the floor between it and the foot before.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanState {
    /// Seconds from the plan's start.
    pub time: f32,
    pub body: BodyState,
    /// The foot that just landed; the other one lands next.
    pub stance: Contact,
    /// Footfalls planned so far; the first is number 0.
    pub footfalls: usize,
}
