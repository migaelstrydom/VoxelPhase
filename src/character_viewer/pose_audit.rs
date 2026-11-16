//! Whether the legs a run drew look like legs.
//!
//! The game window cannot be watched from an agent shell, and a foot placer's
//! bugs are poses, not numbers: a straight leg held out level, a foot over its
//! own hip, a foot that jumps half a stride in a frame. This reads each
//! frame's drawn legs against the hips they hang from and names every stretch
//! of frames that no walking, falling or landing body draws.
//!
//! ```text
//!   FrameRecord ── upright? ──► per foot: oddities this frame
//!                                   │
//!                     runs of frames, per oddity and foot
//!                                   │
//!                      Finding { oddity, foot, start, frames, worst }
//! ```
//!
//! It judges looks, not accuracy: whether a foot stands on its footing or
//! where the plan had it land is the traversal summary's.

use std::collections::BTreeMap;
use std::fmt::Write;

use nalgebra::{Point3, Vector3};

use super::record::{FootRecord, FrameRecord};
use super::soak::SETTLE;

/// How far off straight down a leg near its full length may point, in
/// degrees: a sprint's trailing leg at toe-off is about 45, a crouch-walk's
/// about 55.
const LEVEL_DEGREES: f32 = 65.0;
/// How much of its length a leg must span to be judged as held out, rather
/// than folded: a bent leg may point anywhere a knee can.
const LEVEL_EXTENSION: f32 = 0.75;
/// How much of its length a leg may span with its foot over its own hip:
/// less is a heel tucked up behind.
const OVER_HIP_EXTENSION: f32 = 0.45;
/// How far from where it was asked for a foot may be drawn, in metres: the
/// gait lets a pushing-off leg stretch past its bones by 15%, which the rig
/// draws 7 cm short.
const REINED_BY: f32 = 0.12;
/// The fastest a drawn foot moves against the pelvis, in m/s: what the rig
/// holds its feet to (`humanoid::FootSlew`), and a little.
const SNAP_SPEED: f32 = 14.5;
/// The fastest a foot is asked to move against the pelvis, in m/s, by a
/// stride: a sprint's swing passes the hip at about 12.
const JUMP_SPEED: f32 = 12.5;
/// How far onto the other foot's side of the body a foot may be, in metres.
const CROSSED_BY: f32 = 0.05;
/// How little of its length a leg may span: a foot folded up against its hip.
const FOLDED_EXTENSION: f32 = 0.3;
/// The speed over the ground at which a body on foot has to be stepping, in
/// m/s.
const STALL_SPEED: f32 = 1.0;
/// How long both feet may stay down at that speed, in seconds: a stance.
const STALL_SECONDS: f32 = 0.35;

/// Something a leg was drawn doing that legs do not do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Oddity {
    /// A leg held out level: near its full length, further off straight down
    /// than a stride takes it.
    Level,
    /// A foot over its own hip, the leg not tucked.
    OverHip,
    /// A foot drawn well away from where it was asked for: asked for where
    /// no leg goes, or faster than a foot moves, and held to what one does
    /// by the rig's last word on its feet. Nothing odd is seen; something
    /// upstream asked for it.
    Reined,
    /// A drawn foot that jumps against the pelvis in one frame.
    Snap,
    /// A foot asked for somewhere else from one frame to the next, faster
    /// than a stride moves it: the gait, a pose or a blend changing its
    /// mind. The rig draws it getting there over a few frames.
    Jump,
    /// The feet on each other's side of the body.
    Crossed,
    /// A foot folded up against its hip.
    Folded,
    /// On foot and travelling as asked with both feet down: the legs have
    /// stopped stepping. A body carried along as it stands is asked nothing.
    Stalled,
}

impl Oddity {
    pub const ALL: [Oddity; 8] = [
        Oddity::Level,
        Oddity::OverHip,
        Oddity::Reined,
        Oddity::Snap,
        Oddity::Jump,
        Oddity::Crossed,
        Oddity::Folded,
        Oddity::Stalled,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Oddity::Level => "level",
            Oddity::OverHip => "over_hip",
            Oddity::Reined => "reined",
            Oddity::Snap => "snap",
            Oddity::Jump => "jump",
            Oddity::Crossed => "crossed",
            Oddity::Folded => "folded",
            Oddity::Stalled => "stalled",
        }
    }

    /// The shortest run of frames that counts, in seconds.
    fn least_seconds(self) -> f32 {
        match self {
            Oddity::Stalled => STALL_SECONDS,
            Oddity::Level
            | Oddity::OverHip
            | Oddity::Reined
            | Oddity::Snap
            | Oddity::Jump
            | Oddity::Crossed
            | Oddity::Folded => 0.0,
        }
    }
}

/// Which leg a finding is about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Leg {
    Left,
    Right,
    Both,
}

/// One stretch of frames a leg was drawn oddly through.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Finding {
    pub oddity: Oddity,
    pub leg: Leg,
    /// When it began, in seconds into the run.
    pub start: f32,
    /// How many frames it lasted.
    pub frames: usize,
    /// How long it lasted, in seconds.
    pub seconds: f32,
    /// The oddity's own measure at its worst: degrees off straight down,
    /// metres over the hip, metres reined in, snapped or jumped, metres
    /// crossed, the leg's span as a fraction, or zero.
    pub worst: f32,
}

/// Every oddity a run's legs were drawn with.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PoseAudit {
    /// Frames judged: the body upright on foot or in the air.
    pub judged: usize,
    pub findings: Vec<Finding>,
}

/// An oddity still running, as the frames are read.
struct Run {
    start: f32,
    frames: usize,
    seconds: f32,
    worst: f32,
}

impl PoseAudit {
    /// Judge `frames`, in order.
    pub fn of(frames: &[FrameRecord]) -> Self {
        let mut audit = PoseAudit::default();
        let mut open: BTreeMap<(Oddity, Leg), Run> = BTreeMap::new();
        let mut previous: Option<&FrameRecord> = None;
        for frame in frames {
            let dt = previous.map_or(0.0, |p| frame.time - p.time);
            let seen = if upright(frame) {
                audit.judged += 1;
                oddities(frame, previous.filter(|p| upright(p)), dt)
            } else {
                Vec::new()
            };
            let ended: Vec<_> = open
                .keys()
                .filter(|key| !seen.iter().any(|(oddity, leg, _)| (*oddity, *leg) == **key))
                .copied()
                .collect();
            for key in ended {
                let run = open.remove(&key).expect("an open run");
                audit.close(key, run);
            }
            for (oddity, leg, measure) in seen {
                let run = open.entry((oddity, leg)).or_insert(Run {
                    start: frame.time,
                    frames: 0,
                    seconds: 0.0,
                    worst: measure,
                });
                run.frames += 1;
                run.seconds += dt;
                run.worst = worse(oddity, run.worst, measure);
            }
            previous = Some(frame);
        }
        for (key, run) in open {
            audit.close(key, run);
        }
        audit
            .findings
            .sort_by(|a, b| a.start.total_cmp(&b.start).then(a.oddity.cmp(&b.oddity)));
        audit
    }

    fn close(&mut self, (oddity, leg): (Oddity, Leg), run: Run) {
        if run.seconds + 1.0e-4 < oddity.least_seconds() {
            return;
        }
        self.findings.push(Finding {
            oddity,
            leg,
            start: run.start,
            frames: run.frames,
            seconds: run.seconds,
            worst: run.worst,
        });
    }

    /// How many findings of `oddity`, the frames they cover and the longest.
    pub fn tally(&self, oddity: Oddity) -> Tally {
        self.findings
            .iter()
            .filter(|finding| finding.oddity == oddity)
            .fold(Tally::default(), |tally, finding| Tally {
                findings: tally.findings + 1,
                frames: tally.frames + finding.frames,
                longest: tally.longest.max(finding.frames),
            })
    }

    /// Frames drawn oddly, over every oddity; a frame odd two ways counts
    /// twice.
    pub fn odd_frames(&self) -> usize {
        self.findings.iter().map(|finding| finding.frames).sum()
    }

    /// Column headings for [`Self::row`].
    pub fn header() -> String {
        let mut out = format!("{:24} {:>6}", "scenario", "judged");
        for oddity in Oddity::ALL {
            let _ = write!(out, " {:>12}", oddity.name());
        }
        out
    }

    /// One line for the run `name`: per oddity, findings/frames/longest.
    pub fn row(&self, name: &str) -> String {
        let mut out = format!("{:24} {:>6}", name, self.judged);
        for oddity in Oddity::ALL {
            let tally = self.tally(oddity);
            let cell = if tally.findings == 0 {
                "-".to_string()
            } else {
                format!("{}/{}/{}", tally.findings, tally.frames, tally.longest)
            };
            let _ = write!(out, " {cell:>12}");
        }
        out
    }

    /// Every finding, a line each.
    pub fn listing(&self) -> String {
        let mut out = String::new();
        for finding in &self.findings {
            let _ = writeln!(
                out,
                "  {:6.2}s {:9} {:5} {:3} frames, worst {:.2}",
                finding.start,
                finding.oddity.name(),
                format!("{:?}", finding.leg).to_lowercase(),
                finding.frames,
                finding.worst
            );
        }
        out
    }
}

/// How often an oddity was found.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tally {
    pub findings: usize,
    pub frames: usize,
    /// The longest finding, in frames.
    pub longest: usize,
}

/// How far the body may lean from upright and its legs still be judged
/// against straight down, in degrees: a body jumping out of a stroke is
/// still coming upright through the first of its flight.
const UPRIGHT_WITHIN: f32 = 20.0;

/// Whether `frame` is judged: the rig stands upright, on foot or in the
/// air, in a run under way. A swimmer lies along its stroke and a hanger's
/// legs are a climb's, which are drawn by other rules; and a soak run is
/// first let settle where it was dropped ([`SETTLE`]).
fn upright(frame: &FrameRecord) -> bool {
    frame.beat != SETTLE
        && frame.body_pitch <= UPRIGHT_WITHIN
        && matches!(
            frame.locomotion.as_str(),
            "on_foot" | "grace" | "hop" | "jump" | "fall"
        )
}

/// The more extreme of two measures of `oddity`.
fn worse(oddity: Oddity, a: f32, b: f32) -> f32 {
    match oddity {
        Oddity::Folded => a.min(b),
        Oddity::Level
        | Oddity::OverHip
        | Oddity::Reined
        | Oddity::Snap
        | Oddity::Jump
        | Oddity::Crossed
        | Oddity::Stalled => a.max(b),
    }
}

/// The oddities `frame` draws, each with its measure. `previous` is the
/// frame before, `dt` seconds earlier, if it was judged too.
fn oddities(
    frame: &FrameRecord,
    previous: Option<&FrameRecord>,
    dt: f32,
) -> Vec<(Oddity, Leg, f32)> {
    let mut seen = Vec::new();
    for (index, leg) in [(0, Leg::Left), (1, Leg::Right)] {
        let foot = &frame.feet[index];
        let (span, off_down) = leg_pose(foot, frame.leg_length);
        let over_hip = foot.position.y - foot.hip.y;
        if over_hip > 0.0 && span > OVER_HIP_EXTENSION {
            seen.push((Oddity::OverHip, leg, over_hip));
        } else if off_down > LEVEL_DEGREES && span > LEVEL_EXTENSION {
            seen.push((Oddity::Level, leg, off_down));
        }
        if span < FOLDED_EXTENSION {
            seen.push((Oddity::Folded, leg, span));
        }
        let reined = (foot.asked - foot.position).norm();
        if reined > REINED_BY {
            seen.push((Oddity::Reined, leg, reined));
        }
        if let Some(previous) = previous.filter(|_| dt > 0.0) {
            // As each frame's body faces: a body that spins takes its feet
            // round with it, and nothing has jumped against it.
            let last = &previous.feet[index];
            let moved = |now: Point3<f32>, before: Point3<f32>| {
                (as_faced(frame, now - frame.pelvis) - as_faced(previous, before - previous.pelvis))
                    .norm()
            };
            let snapped = moved(foot.position, last.position);
            if snapped > SNAP_SPEED * dt {
                seen.push((Oddity::Snap, leg, snapped));
            }
            let jumped = moved(foot.asked, last.asked);
            if jumped > JUMP_SPEED * dt {
                seen.push((Oddity::Jump, leg, jumped));
            }
        }
    }
    let right = frame.facing.cross(&Vector3::y());
    let across = |foot: &FootRecord| (foot.position - frame.pelvis).dot(&right);
    let crossed = across(&frame.feet[0]) - across(&frame.feet[1]);
    if crossed > CROSSED_BY {
        seen.push((Oddity::Crossed, Leg::Both, crossed));
    }
    let stepping = frame.feet.iter().any(|foot| !foot.planted);
    if frame.locomotion == "on_foot"
        && frame.grounded
        && frame.asked_speed > STALL_SPEED
        && frame.planar_speed() > STALL_SPEED
        && !stepping
    {
        seen.push((Oddity::Stalled, Leg::Both, 0.0));
    }
    seen
}

/// `offset` as the rig faces in `frame`: to its right, up and ahead.
fn as_faced(frame: &FrameRecord, offset: Vector3<f32>) -> Vector3<f32> {
    let right = frame.facing.cross(&Vector3::y());
    Vector3::new(offset.dot(&right), offset.y, offset.dot(&frame.facing))
}

/// How much of the leg's length `foot` is drawn from its hip, and how far
/// off straight down the line to it points, in degrees.
fn leg_pose(foot: &FootRecord, leg_length: f32) -> (f32, f32) {
    let leg = foot.position - foot.hip;
    let reach = leg.norm();
    let off_down = if reach > 1.0e-4 {
        (-leg.y / reach).clamp(-1.0, 1.0).acos().to_degrees()
    } else {
        0.0
    };
    (reach / leg_length.max(1.0e-3), off_down)
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    const DT: f32 = 1.0 / 60.0;
    const LEG: f32 = 0.5;

    /// A frame of a body standing at the origin on foot, its feet
    /// `left` and `right` of the pelvis.
    fn frame(index: usize, left: Vector3<f32>, right: Vector3<f32>) -> FrameRecord {
        let pelvis = Point3::new(0.0, 0.45, 0.0);
        let foot = |offset: Vector3<f32>, side: f32| {
            let hip = pelvis + Vector3::new(-side * 0.12, 0.0, 0.0);
            FootRecord {
                planted: false,
                anchor: pelvis + offset,
                position: pelvis + offset,
                asked: pelvis + offset,
                hip,
                ground: None,
                planned: None,
            }
        };
        FrameRecord {
            index,
            time: index as f32 * DT,
            beat: "run",
            locomotion: "on_foot".to_string(),
            pose: "",
            upper: String::new(),
            body: pelvis,
            velocity: Vector3::zeros(),
            body_pitch: 0.0,
            pitch_asked: 0.0,
            grounded: true,
            surface: None,
            level: None,
            floor: None,
            pelvis,
            head: pelvis,
            bottom: 0.25,
            ride_floor: None,
            ride_floor_behind: None,
            asked_speed: 0.0,
            plan_forward: None,
            plan_offset: None,
            path_laid: false,
            plan_diverged: false,
            others: (0.0, 0.0),
            // The rig faces +z, so its left is +x.
            feet: [foot(left, -1.0), foot(right, 1.0)],
            leg_length: LEG,
            facing: Vector3::z(),
        }
    }

    const STAND_LEFT: Vector3<f32> = Vector3::new(0.12, -0.43, 0.0);
    const STAND_RIGHT: Vector3<f32> = Vector3::new(-0.12, -0.43, 0.0);

    #[test]
    fn a_body_standing_draws_nothing_odd() {
        let frames: Vec<_> = (0..30).map(|i| frame(i, STAND_LEFT, STAND_RIGHT)).collect();
        let audit = PoseAudit::of(&frames);
        assert_eq!(audit.judged, 30);
        assert!(audit.findings.is_empty(), "{:?}", audit.findings);
    }

    /// A sprint's trailing leg, straight and 45 degrees back, is a stride.
    #[test]
    fn a_trailing_leg_at_toe_off_is_a_stride() {
        let trailing = Vector3::new(0.12, -0.35, -0.35);
        let frames: Vec<_> = (0..10).map(|i| frame(i, trailing, STAND_RIGHT)).collect();
        assert!(PoseAudit::of(&frames).findings.is_empty());
    }

    #[test]
    fn a_straight_leg_held_out_behind_is_level_for_as_long_as_it_is_held() {
        let level = Vector3::new(0.12, -0.05, -0.49);
        let frames: Vec<_> = (0..20)
            .map(|i| {
                let left = if (5..15).contains(&i) {
                    level
                } else {
                    STAND_LEFT
                };
                frame(i, left, STAND_RIGHT)
            })
            .collect();
        let audit = PoseAudit::of(&frames);
        let held: Vec<_> = audit
            .findings
            .iter()
            .filter(|f| f.oddity == Oddity::Level)
            .collect();
        assert_eq!(held.len(), 1, "{:?}", audit.findings);
        assert_eq!((held[0].leg, held[0].frames), (Leg::Left, 10));
        assert!(held[0].worst > 80.0);
        // Going there and coming back in a frame each are snaps, of feet
        // asked to jump.
        assert_eq!(audit.tally(Oddity::Snap).findings, 2);
        assert_eq!(audit.tally(Oddity::Jump).findings, 2);
    }

    #[test]
    fn a_foot_over_its_hip_is_named_unless_the_leg_is_tucked() {
        let over = Vector3::new(0.12, 0.1, -0.45);
        let tucked = Vector3::new(0.12, 0.02, -0.2);
        let raised = PoseAudit::of(&[frame(0, over, STAND_RIGHT)]);
        assert_eq!(raised.tally(Oddity::OverHip).findings, 1);
        assert_eq!(raised.tally(Oddity::Level).findings, 0);
        let heel = PoseAudit::of(&[frame(0, tucked, STAND_RIGHT)]);
        assert_eq!(heel.tally(Oddity::OverHip).findings, 0);
    }

    #[test]
    fn a_foot_drawn_far_from_where_it_was_asked_for_was_reined_in() {
        let mut stretched = frame(0, STAND_LEFT, STAND_RIGHT);
        stretched.feet[1].asked = stretched.feet[1].hip + Vector3::new(0.0, -0.5, -0.45);
        let audit = PoseAudit::of(&[stretched]);
        assert_eq!(audit.tally(Oddity::Reined).findings, 1);
    }

    #[test]
    fn feet_on_each_others_side_are_crossed() {
        let frames = [frame(0, STAND_RIGHT, STAND_LEFT)];
        assert_eq!(PoseAudit::of(&frames).tally(Oddity::Crossed).findings, 1);
    }

    /// Both feet down while the body travels is a stance for a moment, and
    /// legs that have stopped stepping after that.
    #[test]
    fn travelling_with_both_feet_down_is_stalled_only_once_it_outlasts_a_stance() {
        let travelling = |frames: usize| -> Vec<FrameRecord> {
            (0..frames)
                .map(|i| {
                    let mut f = frame(i, STAND_LEFT, STAND_RIGHT);
                    f.velocity = Vector3::new(0.0, 0.0, 3.0);
                    f.asked_speed = 3.0;
                    for foot in &mut f.feet {
                        foot.planted = true;
                    }
                    f
                })
                .collect()
        };
        assert_eq!(
            PoseAudit::of(&travelling(10))
                .tally(Oddity::Stalled)
                .findings,
            0
        );
        assert_eq!(
            PoseAudit::of(&travelling(40))
                .tally(Oddity::Stalled)
                .findings,
            1
        );
    }

    #[test]
    fn a_swimmer_is_not_judged() {
        let mut flat = frame(0, Vector3::new(0.12, 0.0, -0.49), STAND_RIGHT);
        flat.locomotion = "swim".to_string();
        let audit = PoseAudit::of(&[flat]);
        assert_eq!(audit.judged, 0);
        assert!(audit.findings.is_empty());
    }
}
