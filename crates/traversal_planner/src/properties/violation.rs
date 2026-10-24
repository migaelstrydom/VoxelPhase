use std::fmt;

/// A property a plan can break.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Rule {
    /// Each phase starts when and where the one before ends.
    PhasesFollowOn,
    /// The body clears the floor and ceiling, riding where the drive can push
    /// it.
    PhaseClears,
    /// Feet alternate.
    FeetAlternate,
    /// Every footfall is on a foothold the section has.
    OnFoothold,
    /// No footfall is on loose footing that is moving.
    FootingHolds,
    /// Steps, rises, drops and sideways reach are within the gait's limits.
    WithinStride,
    /// Every foot that lands is within reach of the body.
    FootReachesBody,
    /// Every takeoff climbs, and no faster than the gait allows.
    ClimbWithinLimits,
    /// Replanning a frame later on the same ground keeps the commitment.
    KeepsCommitment,
    /// Replanning a frame later on the same ground leaves the next tentative
    /// footfall where it was.
    TentativeStays,
}

/// A property broken, where, and how.
#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    pub rule: Rule,
    /// Seconds into the plan.
    pub time: f32,
    pub detail: String,
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:?} at t = {:.3} s: {}",
            self.rule, self.time, self.detail
        )
    }
}
