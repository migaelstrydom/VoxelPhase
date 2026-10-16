//! How well the character got across a station: speed lost, snags, bounce,
//! foot slip, and whether it got across at all.
//!
//! ```text
//!   Station (where, along which way) ─┐
//!                                     ├─▶ summarise ─▶ TraversalSummary ─▶ row
//!   FrameRecord per frame ────────────┘
//! ```
//!
//! Everything is read off the recorded frames: the summary is a pure function
//! of a take, so a run can be judged again with different thresholds without
//! running the game again.

use nalgebra::{Point3, Vector3};

use super::record::FrameRecord;

/// A stretch of route a traversal scenario measures the character across.
///
/// Positions are distances along `axis` from the world origin, so a station on
/// a lane running along +x is just the x range of its obstacle.
#[derive(Clone, Copy, Debug)]
pub struct Station {
    /// Horizontal unit direction the character crosses the station in.
    pub axis: Vector3<f32>,
    /// Where the station begins: the lip's face, the bridge's first beam end.
    pub enter: f32,
    /// Where it ends: past this, the character has got across.
    pub leave: f32,
}

impl Station {
    /// A station across `enter..leave` of a lane running along +x.
    pub fn along_x(enter: f32, leave: f32) -> Self {
        Self {
            axis: Vector3::x(),
            enter,
            leave,
        }
    }

    /// How far along the station's axis `point` is.
    fn along(&self, point: &Point3<f32>) -> f32 {
        point.coords.dot(&self.axis)
    }

    /// Whether `along` lies in the window the summary judges: the station and
    /// a margin either side for the approach and the recovery.
    fn in_window(&self, along: f32) -> bool {
        along >= self.enter - WINDOW_MARGIN && along <= self.leave + WINDOW_MARGIN
    }
}

/// Distance before and after a station that still counts towards it, in
/// metres: a body brakes before a lip it hits and recovers after it.
const WINDOW_MARGIN: f32 = 1.0;

/// Fraction of the commanded speed a frame may fall short by before it counts
/// as a snag.
const SNAG_SHORTFALL: f32 = 0.2;

/// Planar distance a plant may move between frames and still be the same
/// plant, in metres: the placer settles an anchor onto the ground it probes.
const POP_DISTANCE: f32 = 0.05;

/// How the character fared across one station.
#[derive(Clone, Copy, Debug)]
pub struct TraversalSummary {
    /// Whether the body ended the run past the station's far end.
    pub crossed: bool,
    /// Planar speed the intent asked for while crossing, in m/s.
    pub asked: f32,
    /// Lowest planar speed the body travelled at in the window while the
    /// intent pushed, in m/s.
    pub min_speed: f32,
    /// Seconds from the body reaching the station to leaving it, if it did.
    pub crossing_time: Option<f32>,
    /// Seconds the crossing takes at the asked speed.
    pub ideal_time: f32,
    /// Frames in the window where the intent pushed and the body travelled
    /// more than `SNAG_SHORTFALL` slower than it asked.
    pub snag_frames: usize,
    /// Runs of consecutive snag frames: how many separate times it caught.
    pub snags: usize,
    /// Frames in the window the Support Set was not holding the body up.
    pub airborne_frames: usize,
    /// Largest upward body speed in the window, in m/s: the hop a lip throws.
    pub peak_rise: f32,
    /// Largest downward body speed in the window, in m/s.
    pub peak_drop: f32,
    /// RMS of the body's vertical acceleration in the window, in m/s²: bounce
    /// and jitter. Steady climbing on an even slope contributes nothing.
    pub rms_vertical_accel: f32,
    /// Largest horizontal distance a drawn foot moved in one frame while
    /// holding the same plant on a grounded body, in m.
    pub worst_foot_slip: f32,
    /// Total horizontal distance drawn feet slid off their plants in the
    /// window, in m.
    pub total_foot_slip: f32,
    /// Times a foot's plant moved without a step: planted on both frames, but
    /// anchored somewhere else. The drawn foot jumps with it.
    pub foot_pops: usize,
    /// Frames the summary judged.
    pub window_frames: usize,
}

/// Judge `frames` across `station`.
pub fn summarise(frames: &[FrameRecord], station: &Station) -> TraversalSummary {
    let crossed = frames
        .last()
        .is_some_and(|f| station.along(&f.body) >= station.leave);
    let reached = |line: f32| {
        frames
            .iter()
            .find(|f| station.along(&f.body) >= line)
            .map(|f| f.time)
    };
    let crossing_time = reached(station.enter)
        .zip(reached(station.leave))
        .map(|(enter, leave)| leave - enter);

    // The window ends at the first frame past its far edge, so a body that
    // gets across is not judged on the run-out.
    let window: Vec<(usize, &FrameRecord)> = frames
        .iter()
        .enumerate()
        .skip_while(|(_, f)| station.along(&f.body) < station.enter - WINDOW_MARGIN)
        .take_while(|(_, f)| station.in_window(station.along(&f.body)))
        .collect();

    let asked = window
        .iter()
        .map(|(_, f)| f.asked_speed)
        .fold(0.0, f32::max);
    let pushing = |f: &FrameRecord| f.asked_speed > 0.0;

    let min_speed = window
        .iter()
        .filter(|(_, f)| pushing(f))
        .map(|&(i, _)| travel_speed(frames, i))
        .fold(f32::INFINITY, f32::min);

    let snagged = |i: usize| {
        let f = &frames[i];
        pushing(f) && travel_speed(frames, i) < f.asked_speed * (1.0 - SNAG_SHORTFALL)
    };
    let snag_frames = window.iter().filter(|&&(i, _)| snagged(i)).count();
    let snags = window
        .iter()
        .filter(|&&(i, _)| snagged(i) && (i == 0 || !snagged(i - 1)))
        .count();

    let airborne_frames = window.iter().filter(|(_, f)| !f.grounded).count();
    let peak_rise = window.iter().map(|(_, f)| f.velocity.y).fold(0.0, f32::max);
    let peak_drop = window
        .iter()
        .map(|(_, f)| -f.velocity.y)
        .fold(0.0, f32::max);

    let mut accel_squares = 0.0;
    let mut accel_samples = 0;
    let mut worst_foot_slip: f32 = 0.0;
    let mut total_foot_slip = 0.0;
    let mut foot_pops = 0;
    for &(i, frame) in window.iter().filter(|(i, _)| *i > 0) {
        let previous = &frames[i - 1];
        let dt = frame.time - previous.time;
        if dt > 0.0 {
            let accel = (frame.velocity.y - previous.velocity.y) / dt;
            accel_squares += accel * accel;
            accel_samples += 1;
        }
        for (now, before) in frame.feet.iter().zip(&previous.feet) {
            if !(now.planted && before.planted) {
                continue;
            }
            if planar_distance(&now.anchor, &before.anchor) > POP_DISTANCE {
                foot_pops += 1;
            } else if frame.grounded && previous.grounded {
                let slip = planar_distance(&now.position, &before.position);
                worst_foot_slip = worst_foot_slip.max(slip);
                total_foot_slip += slip;
            }
        }
    }
    let rms_vertical_accel = if accel_samples > 0 {
        (accel_squares / accel_samples as f32).sqrt()
    } else {
        0.0
    };

    TraversalSummary {
        crossed,
        asked,
        min_speed: if min_speed.is_finite() {
            min_speed
        } else {
            0.0
        },
        crossing_time,
        ideal_time: if asked > 0.0 {
            (station.leave - station.enter) / asked
        } else {
            0.0
        },
        snag_frames,
        snags,
        airborne_frames,
        peak_rise,
        peak_drop,
        rms_vertical_accel,
        worst_foot_slip,
        total_foot_slip,
        foot_pops,
        window_frames: window.len(),
    }
}

/// Planar speed of the body over the frame ending at `frames[i]`, from how far
/// it moved rather than the velocity the solver left it with: a body wedged
/// against a contact can keep a velocity it never travels at.
fn travel_speed(frames: &[FrameRecord], i: usize) -> f32 {
    let frame = &frames[i];
    let Some(previous) = i.checked_sub(1).map(|p| &frames[p]) else {
        return frame.planar_speed();
    };
    let dt = frame.time - previous.time;
    if dt > 0.0 {
        planar_distance(&frame.body, &previous.body) / dt
    } else {
        frame.planar_speed()
    }
}

/// Distance between two points in the horizontal plane.
fn planar_distance(a: &Point3<f32>, b: &Point3<f32>) -> f32 {
    let d = a - b;
    Vector3::new(d.x, 0.0, d.z).magnitude()
}

impl TraversalSummary {
    /// Column headings for `row`.
    pub fn header() -> String {
        format!(
            "{:18} {:>5} {:>5} {:>5} {:>11} {:>8} {:>7} {:>5} {:>5} {:>6} {:>5} {:>8} {:>4}",
            "scenario",
            "cross",
            "asked",
            "min",
            "time/ideal",
            "snags",
            "air",
            "rise",
            "drop",
            "rms_ay",
            "slip",
            "slip_sum",
            "pops"
        )
    }

    /// One line of the summary table, for the scenario `name`.
    pub fn row(&self, name: &str) -> String {
        let time = self
            .crossing_time
            .map(|t| format!("{t:5.2}/{:5.2}", self.ideal_time))
            .unwrap_or_else(|| format!("    -/{:5.2}", self.ideal_time));
        format!(
            "{:18} {:>5} {:5.2} {:5.2} {:>11} {:>3}({:>2}) {:>3}/{:<3} {:5.2} {:5.2} {:6.1} {:5.3} {:8.3} {:>4}",
            name,
            if self.crossed { "yes" } else { "NO" },
            self.asked,
            self.min_speed,
            time,
            self.snag_frames,
            self.snags,
            self.airborne_frames,
            self.window_frames,
            self.peak_rise,
            self.peak_drop,
            self.rms_vertical_accel,
            self.worst_foot_slip,
            self.total_foot_slip,
            self.foot_pops
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::character_viewer::FootRecord;

    const DT: f32 = 1.0 / 30.0;

    /// A frame of a body at `x` on a lane along +x, moving at `speed` while
    /// the intent asks for `asked`.
    fn frame(index: usize, x: f32, speed: f32, vy: f32, asked: f32) -> FrameRecord {
        let foot = FootRecord {
            planted: true,
            anchor: Point3::origin(),
            position: Point3::origin(),
        };
        FrameRecord {
            index,
            time: index as f32 * DT,
            beat: "run",
            locomotion: String::new(),
            pose: "",
            upper: String::new(),
            body: Point3::new(x, 0.75, 0.0),
            velocity: Vector3::new(speed, vy, 0.0),
            body_pitch: 0.0,
            pitch_asked: 0.0,
            grounded: true,
            surface: None,
            level: None,
            floor: None,
            pelvis: Point3::origin(),
            head: Point3::origin(),
            asked_speed: asked,
            feet: [foot, foot],
        }
    }

    /// A run at `speed` from x = 0, with `slow` replacing the speed while the
    /// body is inside `slow_at`.
    fn run(speed: f32, slow: f32, slow_at: std::ops::Range<f32>, until: f32) -> Vec<FrameRecord> {
        let mut frames = Vec::new();
        let mut x = 0.0;
        while x < until {
            let v = if slow_at.contains(&x) { slow } else { speed };
            frames.push(frame(frames.len(), x, v, 0.0, speed));
            x += v * DT;
        }
        frames
    }

    #[test]
    fn a_clean_crossing_has_no_snags_and_takes_the_ideal_time() {
        let station = Station::along_x(5.0, 7.0);
        let summary = summarise(&run(5.0, 5.0, 0.0..0.0, 12.0), &station);
        assert!(summary.crossed);
        assert_eq!(summary.snags, 0);
        assert!((summary.min_speed - 5.0).abs() < 1e-4);
        let time = summary.crossing_time.unwrap();
        assert!((time - summary.ideal_time).abs() < 2.0 * DT, "{time}");
    }

    #[test]
    fn a_slowdown_inside_the_station_is_one_snag() {
        let station = Station::along_x(5.0, 7.0);
        let summary = summarise(&run(5.0, 2.0, 5.5..6.5, 12.0), &station);
        assert!(summary.crossed);
        assert_eq!(summary.snags, 1);
        assert!(summary.snag_frames > 1);
        assert!((summary.min_speed - 2.0).abs() < 1e-4);
        assert!(summary.crossing_time.unwrap() > summary.ideal_time + 0.1);
    }

    #[test]
    fn a_wedged_body_is_judged_by_where_it_goes_not_its_velocity() {
        let station = Station::along_x(5.0, 7.0);
        let mut frames = run(5.0, 5.0, 0.0..0.0, 5.5);
        let wedged = frames.last().unwrap().body;
        for _ in 0..30 {
            let mut f = frame(frames.len(), wedged.x, 1.5, 0.0, 5.0);
            f.body = wedged;
            frames.push(f);
        }
        let summary = summarise(&frames, &station);
        assert!(!summary.crossed);
        assert_eq!(summary.min_speed, 0.0);
        assert_eq!(summary.snags, 1);
        assert!(summary.snag_frames >= 30);
    }

    #[test]
    fn a_slowdown_outside_the_window_is_not_judged() {
        let station = Station::along_x(5.0, 7.0);
        let summary = summarise(&run(5.0, 2.0, 1.0..2.0, 12.0), &station);
        assert_eq!(summary.snags, 0);
    }

    #[test]
    fn a_body_that_stops_short_has_not_crossed() {
        let station = Station::along_x(5.0, 7.0);
        let summary = summarise(&run(5.0, 5.0, 0.0..0.0, 5.5), &station);
        assert!(!summary.crossed);
        assert!(summary.crossing_time.is_none());
    }

    #[test]
    fn a_hop_shows_as_rise_and_vertical_acceleration() {
        let station = Station::along_x(5.0, 7.0);
        let mut frames = run(5.0, 5.0, 0.0..0.0, 12.0);
        let flat = summarise(&frames, &station);
        let hop = frames.iter().position(|f| f.body.x >= 5.5).unwrap();
        frames[hop].velocity.y = 1.5;
        let hopped = summarise(&frames, &station);
        assert_eq!(flat.rms_vertical_accel, 0.0);
        assert!((hopped.peak_rise - 1.5).abs() < 1e-4);
        assert!(hopped.rms_vertical_accel > 1.0);
    }

    #[test]
    fn a_planted_foot_drawn_off_its_plant_is_slip() {
        let station = Station::along_x(5.0, 7.0);
        let mut frames = run(5.0, 5.0, 0.0..0.0, 12.0);
        for frame in &mut frames {
            for foot in &mut frame.feet {
                foot.position = Point3::new(0.1 * frame.body.x, 0.0, 0.0);
            }
        }
        let summary = summarise(&frames, &station);
        assert!((summary.worst_foot_slip - 0.1 * 5.0 * DT).abs() < 1e-4);
        assert!(summary.total_foot_slip > 2.0 * 0.1 * 3.0);
        assert_eq!(summary.foot_pops, 0);
    }

    #[test]
    fn a_plant_that_moves_without_a_step_is_a_pop_not_slip() {
        let station = Station::along_x(5.0, 7.0);
        let mut frames = run(5.0, 5.0, 0.0..0.0, 12.0);
        for frame in &mut frames {
            for foot in &mut frame.feet {
                foot.anchor = Point3::new(frame.body.x.floor(), 0.0, 0.0);
                foot.position = foot.anchor;
            }
        }
        let summary = summarise(&frames, &station);
        // The window spans x 4..8: the plants jump at 5, 6, 7 and 8.
        assert_eq!(summary.foot_pops, 2 * 4);
        assert_eq!(summary.total_foot_slip, 0.0);
    }
}
