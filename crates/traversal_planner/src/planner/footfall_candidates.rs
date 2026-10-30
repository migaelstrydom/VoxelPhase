use std::ops::Range;

use crate::params::GaitParams;
use crate::plan::{Contact, Limb};
use crate::section::{Foothold, FootholdWindow, SectionQuery};

use super::cost::FootfallCosts;
use super::settings::PlannerSettings;

/// The footholds one foot may land on through a footfall search, read from
/// the section once, each with what landing on it costs whatever the step
/// to it: a footfall's window is a slice of them, not a query.
pub struct FootfallCandidates {
    candidates: Vec<Candidate>,
    /// Stretches of `candidates` each in increasing `s`, in order: the
    /// section gives footholds lane by lane.
    runs: Vec<Range<usize>>,
}

/// A foothold, and what a footfall on it costs whatever the step to it.
pub struct Candidate {
    pub foothold: Foothold,
    /// `None` where no foot may land: footing that is moving, too near an
    /// edge, or a slope the gait cannot step up.
    pub fixed: Option<FixedCost>,
}

/// The terms of a footfall's cost that depend only on its foothold, kept
/// apart so a footfall's cost sums them in the order it always has.
#[derive(Clone, Copy)]
pub struct FixedCost {
    /// The step the gait takes on this ground, in metres: its natural step,
    /// or the longest the legs reach here if that is shorter.
    pub natural: f32,
    pub sideways: f32,
    pub edge: f32,
    pub footing: f32,
}

/// What [`FootfallCandidates`] are read for.
pub struct CandidateSpec<'a> {
    pub query: &'a dyn SectionQuery,
    pub params: &'a GaitParams,
    pub settings: &'a PlannerSettings,
    pub costs: &'a FootfallCosts<'a>,
    pub target_speed: f32,
    pub spacing: f32,
    pub limb: Limb,
    /// How far along the path a footfall can land at most, in metres.
    pub until: f32,
}

impl FootfallCandidates {
    pub fn new(spec: &CandidateSpec) -> Self {
        let params = spec.params;
        let mut footholds = Vec::new();
        spec.query.footholds(
            &FootholdWindow {
                s: f32::NEG_INFINITY..spec.until,
                d: params.foot.reach(spec.limb),
                spacing: spec.spacing,
                line: Some(params.foot.natural_offset(spec.limb)),
            },
            &mut footholds,
        );
        let mut runs: Vec<Range<usize>> = Vec::new();
        for (index, pair) in footholds.windows(2).enumerate() {
            if pair[1].at.s < pair[0].at.s {
                let start = runs.last().map_or(0, |run| run.end);
                runs.push(start..index + 1);
            }
        }
        let start = runs.last().map_or(0, |run| run.end);
        runs.push(start..footholds.len());
        let candidates = footholds
            .into_iter()
            .map(|foothold| Candidate {
                fixed: fixed_cost(spec, &foothold),
                foothold,
            })
            .collect();
        Self { candidates, runs }
    }

    /// The candidates from `s.start` up to `s.end`, in the section's order.
    pub fn within(&self, s: Range<f32>) -> impl Iterator<Item = &Candidate> {
        self.runs.iter().flat_map(move |run| {
            let run = &self.candidates[run.clone()];
            let first = run.partition_point(|candidate| candidate.foothold.at.s < s.start);
            run[first..]
                .iter()
                .take_while(move |candidate| candidate.foothold.at.s < s.end)
        })
    }
}

fn fixed_cost(spec: &CandidateSpec, foothold: &Foothold) -> Option<FixedCost> {
    let (params, weights) = (spec.params, &spec.settings.costs);
    if !spec.costs.admits(foothold.footing) || foothold.edge_margin < params.foot.min_edge_margin {
        return None;
    }
    let slope = spec.query.floor(foothold.at.s).map_or(0.0, |floor| {
        -floor.normal.s / floor.normal.z.max(f32::EPSILON)
    });
    if !params.steps_up(slope, spec.target_speed) {
        return None;
    }
    // The gait's step, or the longest the legs reach on this ground if
    // that is shorter: a slope is not charged for the steps it forces.
    let grade = -foothold.normal.s / foothold.normal.z.max(f32::EPSILON);
    let natural = spec
        .settings
        .natural_step(spec.target_speed, params)
        .min(params.reachable_step(grade, spec.target_speed))
        .max(params.stride.min_length);
    let reach = params.foot.reach_in.max(params.foot.reach_out);
    let sideways_off = (foothold.at.d - params.foot.natural_offset(spec.limb)) / reach;
    let near_edge = (weights.comfortable_edge_margin - foothold.edge_margin).max(0.0)
        / weights.comfortable_edge_margin;
    let contact = Contact {
        limb: spec.limb,
        footfall: None,
        at: foothold.at,
        normal: foothold.normal,
        footing: foothold.footing,
    };
    Some(FixedCost {
        natural,
        sideways: weights.sideways * sideways_off * sideways_off,
        edge: weights.edge * near_edge * near_edge,
        footing: spec.costs.of(&contact),
    })
}
