//! Measuring a gait, and judging it against what it was planned to be.
//!
//! Most animation checks have to invent a threshold — "a walk should have a
//! duty factor near 0.6" — and an invented threshold is an argument waiting to
//! happen. This module mostly avoids the problem: the foot placer publishes the
//! cadence it planned each frame (`GaitTiming`), so the primary measurements
//! are ratios of observed to intended. A foot that stands for 1.6× the stance
//! it was scheduled is wrong by the engine's own account, whatever a
//! biomechanics textbook says.
//!
//! Two quantities do carry absolute thresholds, because they are absolute
//! facts about a rig rather than choices:
//!
//! * **Leg extension** — at the leg's full length the knee is straight and the
//!   IK has nothing left. There is no tuning under which that looks right while
//!   walking.
//! * **Overreach** — the drawn foot not being where animation asked it to be,
//!   because the leg could not reach that far. Any non-zero value is the gait
//!   asking for something the rig cannot do, and the foot being dragged short
//!   of it.
//!
//! Those two are what "sticky feet" looks like from the inside: the foot stays
//! anchored past its scheduled release, the body walks on, the leg runs out,
//! and the ankle is dragged along a straight line until the step finally fires.

use nalgebra::Vector2;

use super::take::{FrameSample, Side, Take};

/// Ratio of observed to intended above which a cadence quantity is called out.
/// Generous: gait timing legitimately varies as speed does, and the point is to
/// catch a gait that is not doing what it planned, not one that rounded.
const CADENCE_WARN: f32 = 1.25;
const CADENCE_FAIL: f32 = 1.6;

/// Fraction of leg length at which the knee is effectively locked.
const EXTENSION_WARN: f32 = 0.97;
const EXTENSION_FAIL: f32 = 0.995;

/// Metres of overreach worth mentioning. The foot capsule is 6 cm across, so a
/// centimetre of straightened-past-full leg is already visible.
const OVERREACH_WARN: f32 = 0.005;
const OVERREACH_FAIL: f32 = 0.02;

/// Metres a planted foot may travel before it is called a skate.
const SLIP_WARN: f32 = 0.01;
const SLIP_FAIL: f32 = 0.03;

/// How far the left/right takeoff split may sit from half a cycle.
const SPLIT_TOLERANCE: f32 = 0.15;

/// Speed below which the body counts as standing, for the settle window.
const REST_SPEED: f32 = 0.15;
/// Shortest run of standing frames a settle can be judged over. A settle step
/// takes at most `max_step_duration`, so a tail longer than one is long enough
/// for a foot that intends to come home to have arrived.
const REST_TAIL: f32 = 0.5;
/// How far a foot may end up from where the placer says it should stand once
/// the body has stopped. `settle_trigger` is 50 mm, so a foot that took its
/// settle steps sits inside that; these are the distances that mean it did not.
const SETTLE_WARN: f32 = 0.07;
const SETTLE_FAIL: f32 = 0.14;

/// A contiguous run of frames in one phase.
#[derive(Clone, Copy, Debug)]
pub struct Interval {
    pub start: usize,
    pub end: usize,
    /// False when the interval was cut off by the start or end of the window,
    /// in which case its duration is a lower bound and is not averaged in.
    pub complete: bool,
}

/// What one foot did.
#[derive(Clone, Debug)]
pub struct FootMetrics {
    pub side: Side,
    pub steps: usize,

    /// Fraction of the window this foot spent planted.
    pub duty_measured: f32,
    /// Mean of the duty factor the gait was planning over the same window.
    pub duty_designed: f32,

    pub stance_mean: f32,
    pub swing_mean: f32,
    /// Mean of the swing duration the gait was planning.
    pub swing_designed: f32,

    /// Peak hip-to-foot distance as a fraction of leg length, over stance.
    pub extension_peak: f32,
    /// Extension in the frame each step finally fired. A gait that releases at
    /// full stretch is being paced by its reach limit, not by its clock.
    pub extension_at_takeoff: f32,

    /// Furthest the planted foot trailed behind its hip, along travel.
    pub trail_max: f32,
    /// Furthest ahead of its hip the foot planted.
    pub lead_max: f32,

    /// Largest distance the drawn foot moved within a single stance.
    pub plant_slip_max: f32,
    /// Largest distance the placer's anchor moved within a single stance.
    pub anchor_drift_max: f32,

    /// Least hip-to-foot distance seen *while planted*, as a fraction of leg
    /// length. This is the leg's headroom: with a foot on the floor, how much
    /// bend is left before the knee locks. The rig is designed for 0.85
    /// (`standing_height_ratio`); a floor near 1.0 means a stride of any length
    /// overreaches from the first frame.
    pub stance_extension_min: f32,
    /// Furthest the foot was asked to go beyond the leg's reach, in metres.
    /// The rig clamps an unreachable target back on to the leg, so this is
    /// measured against what animation asked for: it is exactly the distance
    /// the drawn foot is dragged short of where the gait wanted it.
    pub overreach_max: f32,
    /// Fraction of the window spent past full reach.
    pub overreach_fraction: f32,

    /// Peak horizontal speed of the foot during swing, over mean body speed.
    /// A foot that dwells and then snaps carries a high ratio.
    pub swing_speed_ratio: f32,

    /// Closest this foot came to its own ideal stance position once the body
    /// had come to rest, in metres. `None` when the window never stands still
    /// long enough to ask.
    ///
    /// At rest the capture point collapses onto the pelvis, so the ideal *is*
    /// the neutral stance and this is simply how far from standing the foot
    /// ended up. The minimum over the window rather than the last frame: a
    /// foot may be mid-settle-step when the take ends, and a foot that shuffles
    /// away again after arriving has still shown it can get home.
    pub settle_offset: Option<f32>,
}

/// What both feet did together.
#[derive(Clone, Debug)]
pub struct GaitMetrics {
    pub scenario: String,
    pub window: String,
    pub ground: String,

    pub frames: usize,
    pub duration: f32,
    pub mean_speed: f32,
    /// Fraction of the window spent off the ground.
    pub airborne_fraction: f32,

    /// Pelvis height the body rode at while supported.
    pub ride_height: f32,
    /// Pelvis height the rig was designed for.
    pub designed_height: f32,
    /// Full reach of one leg.
    pub leg_length: f32,

    pub left: FootMetrics,
    pub right: FootMetrics,

    /// Fraction of the window with both feet planted.
    pub double_support: f32,
    /// Fraction with neither foot planted while still grounded — a flight
    /// phase, correct for a run and wrong for a walk.
    pub flight: f32,

    /// Distance the body travelled per full gait cycle.
    pub cycle_distance_measured: f32,
    /// Distance the gait planned to travel per cycle.
    pub cycle_distance_designed: f32,

    /// Left/right takeoff split as a fraction of a cycle. Half is antiphase.
    pub takeoff_split: Option<f32>,

    pub checks: Vec<Check>,
}

impl GaitMetrics {
    /// The worst verdict any check reached.
    pub fn verdict(&self) -> Verdict {
        self.checks
            .iter()
            .map(|c| c.verdict)
            .max()
            .unwrap_or(Verdict::Ok)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Verdict {
    Ok,
    Warn,
    Fail,
}

impl Verdict {
    pub fn tag(self) -> &'static str {
        match self {
            Verdict::Ok => "ok  ",
            Verdict::Warn => "WARN",
            Verdict::Fail => "FAIL",
        }
    }

    /// Pick a verdict from a value against a warn and a fail threshold.
    fn above(value: f32, warn: f32, fail: f32) -> Self {
        if value >= fail {
            Verdict::Fail
        } else if value >= warn {
            Verdict::Warn
        } else {
            Verdict::Ok
        }
    }
}

/// One judged property of a gait.
#[derive(Clone, Debug)]
pub struct Check {
    pub name: &'static str,
    pub verdict: Verdict,
    /// The measurement, phrased so the number that drove the verdict is visible.
    pub detail: String,
}

/// Analyse a window of frames.
///
/// `window` names the slice for the report — a beat label, or "whole take".
/// Frames where the character is airborne are kept in the window (the airborne
/// fraction is reported) but contribute no stance or swing intervals, since the
/// placer is suspended and has no opinion about feet.
pub fn analyse(take: &Take, frames: &[&FrameSample], window: &str) -> GaitMetrics {
    let duration: f32 = frames.iter().map(|f| f.dt).sum();
    let mean_speed = mean(frames.iter().map(|f| f.horizontal_speed()));
    let travel = travel_direction(frames);

    let left = foot_metrics(take, frames, Side::Left, travel, mean_speed);
    let right = foot_metrics(take, frames, Side::Right, travel, mean_speed);

    let grounded: Vec<&&FrameSample> = frames.iter().filter(|f| f.grounded).collect();
    let double_support = fraction(&grounded, |f| f.left.planted && f.right.planted);
    let flight = fraction(&grounded, |f| !f.left.planted && !f.right.planted);
    let airborne_fraction = 1.0 - fraction(&frames.iter().collect::<Vec<_>>(), |f| f.grounded);

    let cycle_distance_designed = mean(
        frames
            .iter()
            .filter_map(|f| f.timing.map(|t| t.cycle_distance)),
    );
    let cycle_distance_measured = measured_cycle_distance(frames);
    let takeoff_split = takeoff_split(frames);

    let mut metrics = GaitMetrics {
        scenario: take.scenario.clone(),
        window: window.to_string(),
        ground: take.ground.clone(),
        frames: frames.len(),
        duration,
        mean_speed,
        airborne_fraction,
        ride_height: take.ride_height,
        designed_height: take.standing_height,
        leg_length: take.leg_length,
        left,
        right,
        double_support,
        flight,
        cycle_distance_measured,
        cycle_distance_designed,
        takeoff_split,
        checks: Vec::new(),
    };
    metrics.checks = judge(&metrics);
    metrics
}

/// Analyse a whole take, plus one window per beat that lasted long enough for a
/// gait to exist in it.
pub fn analyse_take(take: &Take) -> Vec<GaitMetrics> {
    let all: Vec<&FrameSample> = take.frames.iter().collect();
    let mut out = vec![analyse(take, &all, "whole take")];

    let beats = take.beats();
    if beats.len() > 1 {
        for label in beats {
            let frames: Vec<&FrameSample> = take.beat(label).collect();
            if frames.len() >= 30 {
                out.push(analyse(take, &frames, label));
            }
        }
    }
    out
}

fn foot_metrics(
    take: &Take,
    frames: &[&FrameSample],
    side: Side,
    travel: Vector2<f32>,
    mean_speed: f32,
) -> FootMetrics {
    let planted: Vec<bool> = frames.iter().map(|f| f.foot(side).planted).collect();
    let grounded: Vec<bool> = frames.iter().map(|f| f.grounded).collect();

    // Only frames with support are phase-classified: the placer is suspended in
    // the air, so a foot's "planted" flag there is stale, not observed.
    let stances = intervals(&planted, &grounded, true);
    let swings = intervals(&planted, &grounded, false);

    let supported: Vec<&&FrameSample> = frames.iter().filter(|f| f.grounded).collect();
    let duty_measured = fraction(&supported, |f| f.foot(side).planted);
    let duty_designed = mean(
        frames
            .iter()
            .filter_map(|f| f.timing.map(|t| t.duty_factor)),
    );
    let swing_designed = mean(
        frames
            .iter()
            .filter_map(|f| f.timing.map(|t| t.swing_duration)),
    );

    let stance_mean = mean(
        stances
            .iter()
            .filter(|i| i.complete)
            .map(|i| span_seconds(frames, *i)),
    );
    let swing_mean = mean(
        swings
            .iter()
            .filter(|i| i.complete)
            .map(|i| span_seconds(frames, *i)),
    );

    let mut extension_peak: f32 = 0.0;
    let mut extension_at_takeoff: f32 = 0.0;
    let mut trail_max: f32 = 0.0;
    let mut lead_max: f32 = 0.0;
    let mut plant_slip_max: f32 = 0.0;
    let mut anchor_drift_max: f32 = 0.0;

    for stance in &stances {
        // Slip and drift are measured on the surface, not in the world: a foot
        // riding a moving platform travels metres and has not slipped a
        // millimetre, and a foot that holds still in the world while the
        // platform runs out from under it has slipped the whole way.
        let start = frames[stance.start];
        let first_rendered = start.on_surface(start.foot(side).rendered);
        let first_anchor = start.on_surface(start.foot(side).anchor);
        for frame in &frames[stance.start..=stance.end] {
            let foot = frame.foot(side);
            extension_peak = extension_peak.max(foot.extension() / take.leg_length);
            let offset = planar(foot.hip) - planar(foot.rendered);
            let along = offset.dot(&travel);
            trail_max = trail_max.max(along);
            lead_max = lead_max.max(-along);
            plant_slip_max =
                plant_slip_max.max((frame.on_surface(foot.rendered) - first_rendered).magnitude());
            anchor_drift_max =
                anchor_drift_max.max((frame.on_surface(foot.anchor) - first_anchor).magnitude());
        }
        if stance.complete {
            let last = frames[stance.end].foot(side);
            extension_at_takeoff = extension_at_takeoff.max(last.extension() / take.leg_length);
        }
    }

    let stance_extension_min = stances
        .iter()
        .flat_map(|stance| stance.start..=stance.end)
        .map(|index| frames[index].foot(side).extension() / take.leg_length)
        .fold(f32::MAX, f32::min);
    // A window with no stance at all has no headroom to report; say so with the
    // designed value rather than with `f32::MAX`.
    let stance_extension_min = if stance_extension_min.is_finite() {
        stance_extension_min
    } else {
        take.standing_height / take.leg_length
    };
    let overreach = |f: &FrameSample| f.foot(side).overreach(take.leg_length);
    let overreach_max = frames.iter().map(|f| overreach(f)).fold(0.0f32, f32::max);
    let overreach_fraction = fraction(&frames.iter().collect::<Vec<_>>(), |f| overreach(f) > 1e-4);

    let swing_speed_ratio = if mean_speed > 1e-3 {
        peak_swing_speed(frames, side, &swings) / mean_speed
    } else {
        0.0
    };

    FootMetrics {
        side,
        steps: stances.iter().filter(|i| i.complete).count(),
        duty_measured,
        duty_designed,
        stance_mean,
        swing_mean,
        swing_designed,
        extension_peak,
        extension_at_takeoff,
        stance_extension_min,
        trail_max,
        lead_max,
        plant_slip_max,
        anchor_drift_max,
        overreach_max,
        overreach_fraction,
        swing_speed_ratio,
        settle_offset: settle_offset(frames, side),
    }
}

/// Closest the foot came to its ideal over the window's trailing run of
/// standing frames.
///
/// Measured on the surface so a foot settling on a moving platform is judged
/// against the deck it is standing on rather than against the world.
fn settle_offset(frames: &[&FrameSample], side: Side) -> Option<f32> {
    let start = rest_tail_start(frames)?;
    frames[start..]
        .iter()
        .map(|f| {
            (f.on_surface(f.foot(side).rendered) - f.on_surface(f.foot(side).ideal)).magnitude()
        })
        .fold(f32::MAX, f32::min)
        .into()
}

/// First frame of the window's trailing run of grounded, standing frames, if
/// that run lasted at least `REST_TAIL`.
fn rest_tail_start(frames: &[&FrameSample]) -> Option<usize> {
    let mut seconds = 0.0;
    let mut start = None;
    for (index, frame) in frames.iter().enumerate().rev() {
        if !frame.grounded || frame.horizontal_speed() >= REST_SPEED {
            break;
        }
        seconds += frame.dt;
        start = Some(index);
    }
    start.filter(|_| seconds >= REST_TAIL)
}

/// Turn the checks on a set of measurements.
fn judge(m: &GaitMetrics) -> Vec<Check> {
    let mut checks = Vec::new();

    // Does the rig fit the body it is hung on? Asked first because every gait
    // number below is downstream of the answer: a leg with no bend left while
    // standing has no headroom to take a stride with.
    let ride_ratio = m.ride_height / m.designed_height.max(1e-3);
    let headroom = m
        .left
        .stance_extension_min
        .min(m.right.stance_extension_min);
    checks.push(Check {
        name: "rig-fit",
        verdict: Verdict::above(headroom, EXTENSION_WARN, EXTENSION_FAIL),
        detail: format!(
            "body rides {:.0} mm up, rig designed for {:.0} mm ({ride_ratio:.2}x); \
             with a foot down the leg is never less than {:.0}% extended",
            m.ride_height * 1000.0,
            m.designed_height * 1000.0,
            headroom * 100.0
        ),
    });

    // Asked before the standing early-out below, because a window that ends
    // at rest is exactly the one this judges: whether the feet came home once
    // there was no travel left to carry them there.
    for foot in [&m.left, &m.right] {
        if let Some(offset) = foot.settle_offset {
            checks.push(Check {
                name: "settle",
                verdict: Verdict::above(offset, SETTLE_WARN, SETTLE_FAIL),
                detail: format!(
                    "{}: standing still, the foot got no closer than {:.0} mm to its stance",
                    foot.side.label(),
                    offset * 1000.0
                ),
            });
        }
    }

    // Nothing below means anything for a character that never walked.
    if m.mean_speed < 0.2 {
        checks.push(Check {
            name: "motion",
            verdict: Verdict::Ok,
            detail: format!(
                "standing ({:.2} m/s mean); gait checks skipped",
                m.mean_speed
            ),
        });
        return checks;
    }

    for foot in [&m.left, &m.right] {
        let side = foot.side.label();

        if foot.duty_designed > 0.0 {
            let ratio = foot.duty_measured / foot.duty_designed;
            checks.push(Check {
                name: "stance-share",
                // Symmetric: standing too long and not standing long enough are
                // both the gait failing to keep to its own schedule.
                verdict: Verdict::above(ratio.max(1.0 / ratio), CADENCE_WARN, CADENCE_FAIL),
                detail: format!(
                    "{side}: planted {:.0}% of supported frames, gait planned {:.0}% ({ratio:.2}x)",
                    foot.duty_measured * 100.0,
                    foot.duty_designed * 100.0
                ),
            });
        }

        if foot.swing_designed > 0.0 && foot.swing_mean > 0.0 {
            let ratio = foot.swing_mean / foot.swing_designed;
            checks.push(Check {
                name: "swing-length",
                verdict: Verdict::above(ratio.max(1.0 / ratio), CADENCE_WARN, CADENCE_FAIL),
                detail: format!(
                    "{side}: swings ran {:.0} ms, gait planned {:.0} ms ({ratio:.2}x)",
                    foot.swing_mean * 1000.0,
                    foot.swing_designed * 1000.0
                ),
            });
        }

        checks.push(Check {
            name: "leg-extension",
            verdict: Verdict::above(foot.extension_peak, EXTENSION_WARN, EXTENSION_FAIL),
            detail: format!(
                "{side}: reached {:.1}% of leg length in stance ({:.1}% at takeoff)",
                foot.extension_peak * 100.0,
                foot.extension_at_takeoff * 100.0
            ),
        });

        checks.push(Check {
            name: "overreach",
            verdict: Verdict::above(foot.overreach_max, OVERREACH_WARN, OVERREACH_FAIL),
            detail: format!(
                "{side}: foot asked for {:.0} mm past full leg reach, on {:.0}% of frames",
                foot.overreach_max * 1000.0,
                foot.overreach_fraction * 100.0
            ),
        });

        checks.push(Check {
            name: "plant-slip",
            verdict: Verdict::above(foot.plant_slip_max, SLIP_WARN, SLIP_FAIL),
            detail: format!(
                "{side}: planted foot travelled up to {:.0} mm (anchor moved {:.0} mm)",
                foot.plant_slip_max * 1000.0,
                foot.anchor_drift_max * 1000.0
            ),
        });
    }

    if m.cycle_distance_designed > 0.0 && m.cycle_distance_measured > 0.0 {
        let ratio = m.cycle_distance_measured / m.cycle_distance_designed;
        checks.push(Check {
            name: "stride-length",
            verdict: Verdict::above(ratio.max(1.0 / ratio), CADENCE_WARN, CADENCE_FAIL),
            detail: format!(
                "body travelled {:.2} m per cycle, gait planned {:.2} m ({ratio:.2}x)",
                m.cycle_distance_measured, m.cycle_distance_designed
            ),
        });
    }

    if let Some(split) = m.takeoff_split {
        let error = (split - 0.5).abs();
        checks.push(Check {
            name: "antiphase",
            verdict: Verdict::above(error, SPLIT_TOLERANCE, SPLIT_TOLERANCE * 2.0),
            detail: format!(
                "feet released {:.2} of a cycle apart (half a cycle is antiphase)",
                split
            ),
        });
    }

    checks
}

/// Contiguous runs where `flags` matches `wanted`, restricted to frames with
/// support.
fn intervals(flags: &[bool], grounded: &[bool], wanted: bool) -> Vec<Interval> {
    let mut out: Vec<Interval> = Vec::new();
    let mut start: Option<usize> = None;

    for index in 0..flags.len() {
        let active = grounded[index] && flags[index] == wanted;
        match (active, start) {
            (true, None) => start = Some(index),
            (false, Some(from)) => {
                out.push(Interval {
                    start: from,
                    end: index - 1,
                    complete: from > 0,
                });
                start = None;
            }
            _ => {}
        }
    }
    if let Some(from) = start {
        out.push(Interval {
            start: from,
            end: flags.len() - 1,
            // Cut off by the end of the window, so its duration is a floor.
            complete: false,
        });
    }
    out
}

fn span_seconds(frames: &[&FrameSample], interval: Interval) -> f32 {
    (interval.start..=interval.end).map(|i| frames[i].dt).sum()
}

/// Peak horizontal foot speed within any swing.
fn peak_swing_speed(frames: &[&FrameSample], side: Side, swings: &[Interval]) -> f32 {
    let mut peak: f32 = 0.0;
    for swing in swings {
        for index in swing.start.max(1)..=swing.end {
            let dt = frames[index].dt.max(1e-6);
            // Across the floor, as `mean_speed` is: a swing on a moving
            // platform is as fast as the leg swung it, not as fast as the
            // platform was going.
            let step = frames[index].on_surface(frames[index].foot(side).rendered)
                - frames[index - 1].on_surface(frames[index - 1].foot(side).rendered);
            peak = peak.max(step.magnitude() / dt);
        }
    }
    peak
}

/// Distance the body covered per gait cycle, measured between successive
/// takeoffs of the same foot.
fn measured_cycle_distance(frames: &[&FrameSample]) -> f32 {
    let mut distances = Vec::new();
    for side in Side::BOTH {
        let takeoffs = takeoff_indices(frames, side);
        for pair in takeoffs.windows(2) {
            let from = planar(frames[pair[0]].pelvis);
            let to = planar(frames[pair[1]].pelvis);
            distances.push((to - from).magnitude());
        }
    }
    mean(distances.into_iter())
}

/// Frames where a foot left the ground: planted last frame, stepping this one.
fn takeoff_indices(frames: &[&FrameSample], side: Side) -> Vec<usize> {
    (1..frames.len())
        .filter(|&i| {
            frames[i].grounded && frames[i - 1].foot(side).planted && !frames[i].foot(side).planted
        })
        .collect()
}

/// How far apart, as a fraction of a cycle, the two feet release.
///
/// A healthy gait alternates: left, right, left, each half a cycle apart. Feet
/// that release together are the in-phase hop the placer's clock exists to
/// prevent.
fn takeoff_split(frames: &[&FrameSample]) -> Option<f32> {
    let mut events: Vec<(usize, Side)> = Vec::new();
    for side in Side::BOTH {
        events.extend(takeoff_indices(frames, side).into_iter().map(|i| (i, side)));
    }
    events.sort_by_key(|(index, _)| *index);

    let mut alternating = Vec::new();
    let mut cycles = Vec::new();
    for pair in events.windows(2) {
        let gap: f32 = (pair[0].0..pair[1].0).map(|i| frames[i].dt).sum();
        if pair[0].1 != pair[1].1 {
            alternating.push(gap);
        }
    }
    for pair in events.windows(3) {
        if pair[0].1 == pair[2].1 {
            cycles.push((pair[0].0..pair[2].0).map(|i| frames[i].dt).sum::<f32>());
        }
    }

    let split = mean(alternating.into_iter());
    let cycle = mean(cycles.into_iter());
    (cycle > 1e-3).then(|| split / cycle)
}

/// Mean direction of travel over a window, as a horizontal unit vector.
fn travel_direction(frames: &[&FrameSample]) -> Vector2<f32> {
    let sum = frames
        .iter()
        .fold(Vector2::zeros(), |acc: Vector2<f32>, f| {
            acc + Vector2::new(f.velocity.x, f.velocity.z)
        });
    sum.try_normalize(1e-4)
        .unwrap_or_else(|| Vector2::new(1.0, 0.0))
}

fn planar(point: nalgebra::Point3<f32>) -> Vector2<f32> {
    Vector2::new(point.x, point.z)
}

fn fraction<T>(items: &[T], predicate: impl Fn(&T) -> bool) -> f32 {
    if items.is_empty() {
        return 0.0;
    }
    items.iter().filter(|item| predicate(item)).count() as f32 / items.len() as f32
}

fn mean(values: impl Iterator<Item = f32>) -> f32 {
    let mut sum = 0.0;
    let mut count = 0;
    for value in values {
        sum += value;
        count += 1;
    }
    if count == 0 {
        0.0
    } else {
        sum / count as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intervals_skip_unsupported_frames() {
        //          0     1     2      3      4     5
        let flags = [true, true, false, false, true, true];
        let ground = [true, true, true, false, true, true];
        let stances = intervals(&flags, &ground, true);
        assert_eq!(stances.len(), 2);
        assert_eq!((stances[0].start, stances[0].end), (0, 1));
        assert_eq!((stances[1].start, stances[1].end), (4, 5));
        assert!(!stances[0].complete, "an interval at frame 0 is cut off");
        assert!(!stances[1].complete, "so is one running to the last frame");
    }

    #[test]
    fn verdicts_order_by_severity() {
        assert!(Verdict::Fail > Verdict::Warn);
        assert!(Verdict::Warn > Verdict::Ok);
        assert_eq!(Verdict::above(1.7, 1.25, 1.6), Verdict::Fail);
        assert_eq!(Verdict::above(1.3, 1.25, 1.6), Verdict::Warn);
        assert_eq!(Verdict::above(1.0, 1.25, 1.6), Verdict::Ok);
    }
}
