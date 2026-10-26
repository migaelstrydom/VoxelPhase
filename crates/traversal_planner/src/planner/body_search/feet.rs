use crate::planner::rules::{FOOT_SLACK, PUSH_OFF_TOLERANCE};
use crate::section::{FootholdWindow, Footing};

use super::search::{BodySearch, Node, STATION_TOLERANCE};

/// Spacing of the footholds looked for to tell whether a station is within a
/// foot's reach of any, in metres.
const SUPPORT_SPACING: f32 = 0.1;

/// What the feet need of a station.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum FootNeed {
    /// To ride through or land on it: a foothold within half a step.
    Support,
    /// To take off from it: a foothold just behind, to push off from.
    PushOff,
    /// To land on it: a foothold just ahead, to land on.
    TouchDown,
}

/// What the feet can do at each station, as it is worked out.
pub(super) struct FootReach {
    answers: [Vec<Option<bool>>; 3],
}

impl FootReach {
    pub(super) fn new(stations: usize) -> Self {
        Self {
            answers: std::array::from_fn(|_| vec![None; stations]),
        }
    }

    pub(super) fn len(&self) -> usize {
        self.answers[0].len()
    }
}

impl BodySearch<'_> {
    /// Whether a foot behind the takeoff at `station` can push the body off:
    /// the planted one, until the first flight; after it, one that landed at
    /// or after the body last touched down.
    pub(super) fn pushes_off(&self, origin: f32, at: &Node, known: &mut FootReach) -> bool {
        let (station, z, landed) = (at.station, at.z, at.landed);
        let from = self.station_s(origin, station);
        if from < landed - PUSH_OFF_TOLERANCE {
            false
        } else if from - landed >= FOOT_SLACK {
            self.feet_can(origin, station, FootNeed::PushOff, known)
        } else {
            !at.flown || self.foothold_within(landed..from + PUSH_OFF_TOLERANCE, from, z)
        }
    }

    /// Whether a foothold in `window` along the path is in reach of the body
    /// at height `body_z`, and one a foot may land on.
    pub(super) fn foothold_within(
        &self,
        window: std::ops::Range<f32>,
        body_s: f32,
        body_z: f32,
    ) -> bool {
        let mut footholds = Vec::new();
        self.query.footholds(
            &FootholdWindow {
                s: window,
                d: f32::MIN..f32::MAX,
                spacing: SUPPORT_SPACING,
                line: None,
            },
            &mut footholds,
        );
        footholds.iter().any(|foothold| {
            foothold.edge_margin >= self.params.foot.min_edge_margin
                && !matches!(foothold.footing, Footing::Loose { resting: false, .. })
                && self.params.foot_reaches(body_s, body_z, &foothold.at)
        })
    }

    /// Whether the feet can do what `need` asks at a station, worked out once
    /// per station: a foothold in reach of the body resting there, in the
    /// window `need` names, that a foot may land on.
    pub(super) fn feet_can(
        &self,
        origin: f32,
        station: usize,
        need: FootNeed,
        known: &mut FootReach,
    ) -> bool {
        let slot = &mut known.answers[need as usize][station];
        if let Some(answer) = *slot {
            return answer;
        }
        let s = self.station_s(origin, station);
        let half = 0.5 * self.params.stride.max_length;
        let window = match need {
            FootNeed::Support => s - half..s + half,
            FootNeed::PushOff => s - FOOT_SLACK..s + PUSH_OFF_TOLERANCE,
            FootNeed::TouchDown => s - STATION_TOLERANCE..s + FOOT_SLACK,
        };
        let answer = (station == 0 && need == FootNeed::Support)
            || self
                .query
                .floor(s)
                .is_some_and(|floor| self.foothold_within(window, s, floor.z));
        known.answers[need as usize][station] = Some(answer);
        answer
    }
}
