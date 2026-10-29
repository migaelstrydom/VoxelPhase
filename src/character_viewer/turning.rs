//! How the traversal plan kept up with a steady turn: the runs, on the
//! Traversal Garden's open ground west of its stations, and what they are
//! judged on.
//!
//! The stick steers the body and the plan's path is a straight line laid
//! along the heading, so while the body turns the plan's footfalls lie along
//! a line the body has turned away from until the path is laid afresh. The
//! lag between the two is what the feet show as running one way while
//! stepping another.

use std::f32::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};

use super::record::FrameRecord;
use super::scenario::{CameraRig, Judged, Scenario};
use super::traversal::FootfallSummary;

const GARDEN: &str = "levels/traversal_garden.level.ron";
/// The beat a turning run is judged over.
const TURN_BEAT: &str = "turn";

/// A run along +x from `start` for `run_up` seconds, then turning left at
/// `degrees_per_second` through `degrees`, then on straight.
fn turning(
    name: &'static str,
    summary: &'static str,
    start: (f32, f32),
    beat: fn(f32) -> Beat,
    run_up: f32,
    degrees_per_second: f32,
    degrees: f32,
) -> Scenario {
    Scenario {
        name,
        summary,
        level: GARDEN,
        start,
        yaw: FRAC_PI_2,
        script: Script::new()
            .then(Beat::stand(0.3))
            .then(beat(run_up))
            .then(
                // The pilot turns a positive rate from +x towards -z.
                beat(degrees / degrees_per_second)
                    .turning(-degrees_per_second.to_radians())
                    .named(TURN_BEAT),
            )
            .then(beat(0.5))
            .then(Beat::stand(0.5)),
        camera: CameraRig::three_quarter().with_distance(6.0),
        judged: Judged::Turn { axis: Vector3::x() },
    }
}

/// The turning runs. Each turns 120° left within the flat ground at
/// x -16..7, z -16..7: 6.4 m round at 5 m/s, 7.6 m at 8 m/s.
pub fn catalogue() -> Vec<Scenario> {
    vec![
        turning(
            "turn_walk",
            "Turning: walk a steady 45°/s turn on open ground",
            (-15.0, -14.0),
            Beat::walk,
            1.5,
            45.0,
            120.0,
        ),
        turning(
            "turn_run",
            "Turning: run a steady 60°/s turn on open ground",
            (-15.0, -15.0),
            Beat::run,
            1.2,
            60.0,
            120.0,
        ),
    ]
}

/// How the plan kept up with one turning run.
#[derive(Clone, Copy, Debug, Default)]
pub struct TurnSummary {
    /// Seconds judged: the turning beat.
    pub seconds: f32,
    /// Frames with a plan, of those judged.
    pub planned_frames: usize,
    /// Summed angle between the plan's path and the body's travel over the
    /// planned frames, in degrees.
    pub total_lag: f32,
    /// Largest such angle, in degrees.
    pub worst_lag: f32,
    /// Times the path was laid afresh.
    pub relays: usize,
    /// Summed planar speed over the frames judged, and the speed asked, in
    /// m/s: how much the turn cost the body.
    pub total_speed: f32,
    pub total_asked: f32,
    pub frames: usize,
    /// Where the feet landed.
    pub footfalls: FootfallSummary,
}

impl TurnSummary {
    /// Judge the turning beat of `frames`.
    pub fn of(frames: &[FrameRecord]) -> Self {
        let mut summary = Self::default();
        let judged = || {
            frames
                .windows(2)
                .filter(|pair| pair[1].beat == TURN_BEAT)
                .map(|pair| (&pair[1], &pair[0]))
        };
        for (frame, previous) in judged() {
            summary.frames += 1;
            summary.seconds += frame.time - previous.time;
            summary.total_speed += frame.planar_speed();
            summary.total_asked += frame.asked_speed;
            summary.relays += usize::from(frame.path_laid);
            let travel = Vector3::new(frame.velocity.x, 0.0, frame.velocity.z);
            if let (Some(forward), Some(travel)) =
                (frame.plan_forward, travel.try_normalize(1.0e-3))
            {
                let lag = forward.angle(&travel).to_degrees();
                summary.planned_frames += 1;
                summary.total_lag += lag;
                summary.worst_lag = summary.worst_lag.max(lag);
            }
        }
        summary.footfalls = FootfallSummary::of(judged());
        summary
    }

    /// Both summaries together.
    fn merged(self, other: Self) -> Self {
        Self {
            seconds: self.seconds + other.seconds,
            planned_frames: self.planned_frames + other.planned_frames,
            total_lag: self.total_lag + other.total_lag,
            worst_lag: self.worst_lag.max(other.worst_lag),
            relays: self.relays + other.relays,
            total_speed: self.total_speed + other.total_speed,
            total_asked: self.total_asked + other.total_asked,
            frames: self.frames + other.frames,
            footfalls: self.footfalls.merged(other.footfalls),
        }
    }

    /// The summaries of several runs of one scenario, taken together.
    pub fn over(summaries: &[TurnSummary]) -> Self {
        summaries
            .iter()
            .fold(Self::default(), |all, summary| all.merged(*summary))
    }

    /// Column headings for `row`.
    pub fn header() -> String {
        format!(
            "{:16} {:>6} {:>7} {:>7} {:>7} {:>6} {}",
            "scenario",
            "turn s",
            "lag",
            "lag_max",
            "relay/s",
            "speed",
            FootfallSummary::header()
        )
    }

    /// One line of the table, for the scenario `name`: the mean and worst
    /// lag in degrees, relays a second, and the body's speed as a share of
    /// what was asked.
    pub fn row(&self, name: &str) -> String {
        let lag = if self.planned_frames == 0 {
            f32::NAN
        } else {
            self.total_lag / self.planned_frames as f32
        };
        format!(
            "{name:16} {:6.2} {lag:7.2} {:7.2} {:7.2} {:6.2} {}",
            self.seconds,
            self.worst_lag,
            self.relays as f32 / self.seconds.max(f32::EPSILON),
            self.total_speed / self.total_asked.max(f32::EPSILON),
            self.footfalls.columns()
        )
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::character_viewer::FootRecord;

    fn frame(index: usize, beat: &'static str, plan_degrees: f32, laid: bool) -> FrameRecord {
        let foot = FootRecord {
            planted: true,
            anchor: Point3::origin(),
            position: Point3::origin(),
            ground: Some(0.0),
            planned: None,
        };
        let (sin, cos) = plan_degrees.to_radians().sin_cos();
        FrameRecord {
            index,
            time: index as f32 / 60.0,
            beat,
            locomotion: String::new(),
            pose: "",
            upper: String::new(),
            body: Point3::origin(),
            velocity: Vector3::new(5.0, 0.0, 0.0),
            body_pitch: 0.0,
            pitch_asked: 0.0,
            grounded: true,
            surface: None,
            level: None,
            floor: None,
            pelvis: Point3::origin(),
            head: Point3::origin(),
            bottom: 0.25,
            ride_floor: Some(0.25),
            ride_floor_behind: Some(0.25),
            asked_speed: 5.0,
            plan_forward: Some(Vector3::new(cos, 0.0, sin)),
            path_laid: laid,
            feet: [foot, foot],
        }
    }

    #[test]
    fn a_plan_off_the_travel_is_lag_only_while_turning() {
        let frames = vec![
            frame(0, "walk", 30.0, true),
            frame(1, TURN_BEAT, 4.0, false),
            frame(2, TURN_BEAT, 8.0, true),
            frame(3, "walk", 30.0, false),
        ];
        let summary = TurnSummary::of(&frames);
        assert_eq!(summary.planned_frames, 2);
        assert!((summary.total_lag - 12.0).abs() < 1.0e-3);
        assert!((summary.worst_lag - 8.0).abs() < 1.0e-3);
        assert_eq!(summary.relays, 1);
    }
}
