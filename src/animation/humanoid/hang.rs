//! Hanging from a ledge and climbing onto it: the pose and the arms of a
//! body whose hands hold an edge (`LocomotionState::Ledge`).
//!
//! ```text
//!   LedgeReading (world, this frame) ──► Hang::advance ──► Hang
//!     hands on the lip, the face,          legs' swing and knees' tuck, driven by the
//!     the stand point, the phase           body's changes of velocity; the lean; the
//!                                          hands' holds along the edge; the climb
//!                                                │
//!                       Posture::of ◄────────────┤  pelvis and torso under the hands
//!        PoseState::Hanging ◄── sample_hanging ──┤  feet, pelvis offset, torso, head
//!      UpperState::Gripping ◄── sample_gripping ─┘  hands, hand over hand
//! ```
//!
//! Nothing here is a canned clip. The physics moves the body; the hands stay
//! where they hold; the figure is placed between them, its arms only as long
//! as they are; the legs swing as a damped pendulum from the hips, set going
//! by every change in the body's velocity (the catch, a platform's turn).
//! A climb is the one exception: the capsule must clear the lip, which puts
//! its pelvis higher than hands on the lip can reach, so the figure follows
//! a path of its own through key poses, held to the edge.

use std::f32::consts::PI;

use nalgebra::{Point3, Vector2, Vector3};

use crate::animation::config::CharacterRigConfig;
use crate::animation::pose::{FeetPose, FootAnchor, HandsPose, PoseFragment};
use crate::animation::state::AnimationState;

/// What the animation reads of a hold on a ledge this frame, in the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LedgeReading {
    pub phase: HangPhase,
    /// Where the hands hold, on the top behind the lip, in no set order.
    pub hands: [Point3<f32>; 2],
    /// Level and unit length, out of the face.
    pub outward: Vector3<f32>,
    /// A point on the face, under the lip.
    pub face: Point3<f32>,
    /// Where the body's centre stands after a climb, if it can climb.
    pub stand: Option<Point3<f32>>,
    /// The velocity of the edge's body at the grip; zero for terrain.
    pub edge_velocity: Vector3<f32>,
    /// Whether a face stands below the lip where the feet hang.
    pub wall_below: bool,
}

/// Where in a hold the body is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HangPhase {
    /// The hands landing on the edge, `progress` of the way through the
    /// settle.
    Catch { progress: f32 },
    /// Hanging. `lean` is a drop (towards -1) or a pull-up (towards +1)
    /// being held, as a fraction of the hold that commits it.
    Hang { lean: f32 },
    /// Climbing onto the top, `progress` of the way.
    Climb { progress: f32 },
}

impl LedgeReading {
    /// Midway between the hands.
    pub fn grip(&self) -> Point3<f32> {
        nalgebra::center(&self.hands[0], &self.hands[1])
    }

    /// Along the edge, level: the body's right as it faces the wall.
    pub fn right(&self) -> Vector3<f32> {
        Vector3::y().cross(&self.outward).normalize()
    }

    /// The lip's height under the hand `along` metres to the right of the
    /// grip: the hands' heights, carried on along the edge.
    fn lip_at(&self, along: f32) -> f32 {
        let right = self.right();
        let (a, b) = (self.hands[0], self.hands[1]);
        let (sa, sb) = ((a - self.grip()).dot(&right), (b - self.grip()).dot(&right));
        if (sb - sa).abs() < 1.0e-4 {
            return self.grip().y;
        }
        a.y + (b.y - a.y) * (along - sa) / (sb - sa)
    }

    /// Half the hands' spacing along the edge.
    fn hand_offset(&self) -> f32 {
        0.5 * (self.hands[1] - self.hands[0]).dot(&self.right()).abs()
    }

    /// How far the lip stands over the top a climber stands on: the top is
    /// as high as the lip, give or take a voxel's rounding.
    fn top(&self) -> f32 {
        self.grip().y
    }
}

/// A hold on a ledge as the animation carries it from frame to frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Hang {
    pub reading: LedgeReading,
    /// The legs' swing from hanging straight down, in radians: out from the
    /// wall (x) and to the body's right (y).
    swing: Vector2<f32>,
    swing_rate: Vector2<f32>,
    /// How far the knees are drawn up, 0 straight to 1 tucked.
    tuck: f32,
    tuck_rate: f32,
    /// The lean, eased.
    lean: f32,
    /// The hands' holds, left then right.
    holds: [HandHold; 2],
    /// How far the figure has shifted its weight along the edge, to the
    /// body's right, in metres.
    sway: f32,
    /// The climb, once one has begun.
    climb: Option<ClimbStart>,
    /// The body's velocity relative to the edge last frame.
    velocity: Vector3<f32>,
    /// Whether this hold began as a climb from the ground (a mantle).
    mantle: bool,
}

/// Where one hand holds, along the edge from the grip, and its move to a
/// new hold if it is making one.
#[derive(Debug, Clone, Copy, PartialEq)]
struct HandHold {
    along: f32,
    shift: Option<HandShift>,
    /// Seconds since the hand last landed.
    planted: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct HandShift {
    from: f32,
    to: f32,
    elapsed: f32,
    /// Seconds the move takes.
    duration: f32,
}

impl HandShift {
    /// How far through the move the hand is, 0 to 1.
    fn progress(&self) -> f32 {
        (self.elapsed / self.duration).clamp(0.0, 1.0)
    }
}

/// How the figure stood when its climb began, relative to the grip.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ClimbStart {
    pelvis: Vector3<f32>,
    torso_pitch: f32,
    feet: [Vector3<f32>; 2],
    hands: [Vector3<f32>; 2],
    /// The leg that steps up first: true for the right.
    lead_right: bool,
}

/// The legs as a pendulum from the hips: natural frequency in rad/s and
/// damping ratio. Short legs on a small body swing quickly.
const SWING_FREQUENCY: f32 = 5.5;
const SWING_DAMPING: f32 = 0.22;
/// Radians of swing a 1 m/s change of the body's velocity sets going:
/// about the angular speed a leg's length picks up from it.
const SWING_KICK: f32 = 1.6;
/// Furthest the legs swing, in radians.
const SWING_LIMIT: f32 = 1.1;
/// The knees: frequency, damping, the tuck a 1 m/s jolt upward gives, and
/// the tuck a hanger rests at.
const TUCK_FREQUENCY: f32 = 9.0;
const TUCK_DAMPING: f32 = 0.45;
const TUCK_KICK: f32 = 0.35;
const TUCK_REST: f32 = 0.12;
/// Seconds the lean takes to follow the stick.
const LEAN_EASE: f32 = 0.08;
/// How far a still body's hand may lie off its place before it moves there,
/// and how long a hand's move takes at most, in metres and seconds.
const HAND_SLACK: f32 = 0.07;
const HAND_SHIFT: f32 = 0.16;
/// Shimmying, how far past its place the leading hand lands, in metres:
/// past the shoulder, as far as arms near straight reach out to the side.
const HAND_REACH: f32 = 0.18;
/// Shimmying, how much nearer the leading hand the trailing one lands than
/// the hands' spacing, in metres. The shoulders stand wider than the hands,
/// so closing them in further crosses the trailing arm under the head.
const HAND_CLOSE: f32 = 0.05;
/// A shimmy's hand moves take this share of the time the body takes to
/// catch up with the leading hand, no longer than `HAND_SHIFT` nor shorter
/// than `HAND_QUICKEST`.
const HAND_SHARE: f32 = 0.35;
const HAND_QUICKEST: f32 = 0.06;
/// The leading hand bears the weight for this share of a hand move before
/// the trailing hand follows.
const WEIGHT_SHARE: f32 = 0.5;
/// How far the figure shifts along the edge towards where its hands hold,
/// as a share of the holds' midpoint; at most `SWAY_LIMIT` metres, and
/// eased over `SWAY_EASE` seconds.
const SWAY_GAIN: f32 = 0.5;
const SWAY_LIMIT: f32 = 0.08;
const SWAY_EASE: f32 = 0.06;
/// How high a moving hand lifts off the lip, and how far out from the wall.
const HAND_LIFT: f32 = 0.05;
const HAND_CLEAR: f32 = 0.03;

impl Hang {
    /// A hold begun this frame: the legs still, the hands where the probe
    /// found the edge. One that begins as a climb is a mantle.
    pub fn begin(reading: LedgeReading, velocity: Vector3<f32>) -> Self {
        let offset = reading.hand_offset();
        Self {
            reading,
            swing: Vector2::zeros(),
            swing_rate: Vector2::zeros(),
            tuck: TUCK_REST,
            tuck_rate: 0.0,
            lean: 0.0,
            holds: [
                HandHold {
                    along: -offset,
                    shift: None,
                    planted: 0.0,
                },
                HandHold {
                    along: offset,
                    shift: None,
                    planted: 0.0,
                },
            ],
            sway: 0.0,
            climb: None,
            velocity: velocity - reading.edge_velocity,
            mantle: matches!(reading.phase, HangPhase::Climb { .. }),
        }
    }

    /// One frame on: the legs and knees answer the change in the body's
    /// velocity, the lean follows the stick, the hands keep their holds on
    /// the edge as the body moves along it and move hand over hand when it
    /// leaves them behind, and a climb beginning takes the figure as it is.
    pub fn advance(
        &mut self,
        reading: LedgeReading,
        velocity: Vector3<f32>,
        figure: &FigureNow,
        rig: &CharacterRigConfig,
        dt: f32,
    ) {
        self.reading = reading;
        if dt <= 0.0 {
            return;
        }
        let relative = velocity - reading.edge_velocity;
        let jolt = relative - self.velocity;
        self.velocity = relative;
        let (out, right) = (reading.outward, reading.right());

        // A climb is a pull held: the lean it began with stays.
        let lean_target = match reading.phase {
            HangPhase::Hang { lean } => lean.clamp(-1.0, 1.0),
            HangPhase::Catch { .. } => 0.0,
            HangPhase::Climb { .. } => self.lean,
        };
        self.lean += (lean_target - self.lean) * (1.0 - (-dt / LEAN_EASE).exp());

        // The pivot jolted one way leaves the legs swinging the other.
        self.swing_rate -= Vector2::new(jolt.dot(&out), jolt.dot(&right)) * SWING_KICK;
        let rest = Vector2::new(self.swing_rest(), 0.0);
        let accel = (rest - self.swing) * SWING_FREQUENCY.powi(2)
            - self.swing_rate * (2.0 * SWING_DAMPING * SWING_FREQUENCY);
        self.swing_rate += accel * dt;
        self.swing += self.swing_rate * dt;
        self.swing = self.swing.map(|a| a.clamp(-SWING_LIMIT, SWING_LIMIT));

        // Caught from a fall, the knees give.
        self.tuck_rate += jolt.y.max(0.0) * TUCK_KICK * TUCK_FREQUENCY;
        let tuck_rest = TUCK_REST + 0.45 * self.lean.max(0.0);
        let tuck_accel = (tuck_rest - self.tuck) * TUCK_FREQUENCY.powi(2)
            - self.tuck_rate * (2.0 * TUCK_DAMPING * TUCK_FREQUENCY);
        self.tuck_rate += tuck_accel * dt;
        self.tuck = (self.tuck + self.tuck_rate * dt).clamp(0.0, 1.0);

        let moved = relative.dot(&right) * dt;
        let climbing = matches!(reading.phase, HangPhase::Climb { .. });
        self.keep_holds(moved, relative.dot(&right), climbing, dt);
        let sway = (SWAY_GAIN * self.weight_along()).clamp(-SWAY_LIMIT, SWAY_LIMIT);
        self.sway += (sway - self.sway) * (1.0 - (-dt / SWAY_EASE).exp());

        if climbing && self.climb.is_none() {
            self.climb = Some(self.climb_start(figure, rig));
        }
    }

    /// Where the legs hang at rest: a little in towards a wall below, to rest
    /// the toes on it; out from it when leaning to drop.
    fn swing_rest(&self) -> f32 {
        let towards_wall = if self.reading.wall_below { -0.22 } else { 0.0 };
        towards_wall + 0.35 * (-self.lean).max(0.0)
    }

    /// Hands keep their place on the edge as the body slides along it by
    /// `moved`. On the move they go hand over hand, leading hand first
    /// ([`Self::shimmy_holds`]); still, a hand left `HAND_SLACK` off its place
    /// moves to it, one hand at a time.
    fn keep_holds(&mut self, moved: f32, speed: f32, climbing: bool, dt: f32) {
        for hold in &mut self.holds {
            hold.along -= moved;
            hold.planted += dt;
            if let Some(shift) = &mut hold.shift {
                shift.from -= moved;
                shift.to -= moved;
                shift.elapsed += dt;
            }
        }
        for hold in &mut self.holds {
            if let Some(shift) = hold.shift.filter(|s| s.elapsed >= s.duration) {
                hold.along = shift.to;
                hold.shift = None;
                hold.planted = 0.0;
            }
        }
        if climbing || self.holds.iter().any(|h| h.shift.is_some()) {
            return;
        }
        let offset = self.reading.hand_offset();
        let places = [-offset, offset];
        if speed.abs() > 0.05 {
            self.shimmy_holds(places, speed);
            return;
        }
        let lagging = (0..2)
            .map(|i| (i, (self.holds[i].along - places[i]).abs()))
            .filter(|(_, off)| *off > HAND_SLACK)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((i, _)) = lagging {
            self.move_hand(i, places[i], HAND_SHIFT);
        }
    }

    /// Hand over hand along the edge at `speed`: once the body has caught up
    /// with the leading hand, it reaches out `HAND_REACH` past its place;
    /// when it has borne the weight a moment and the figure has shifted
    /// towards it, the trailing hand follows, `HAND_CLOSE` nearer it than the
    /// hands' spacing. Neither hand trails behind the figure, and the moves
    /// quicken as the shimmy does.
    fn shimmy_holds(&mut self, places: [f32; 2], speed: f32) {
        let way = speed.signum();
        let (lead, trail) = if way > 0.0 { (1, 0) } else { (0, 1) };
        let duration = (HAND_SHARE * HAND_REACH / speed.abs()).clamp(HAND_QUICKEST, HAND_SHIFT);
        let closed = places[lead] - places[trail] - way * HAND_CLOSE;
        let behind = self.holds[lead].along - closed;
        if (behind - self.holds[trail].along) * way > HAND_SLACK * 0.5
            && self.holds[lead].planted >= WEIGHT_SHARE * duration
        {
            self.move_hand(trail, behind, duration);
        } else if (self.holds[lead].along - places[lead]) * way <= 0.0 {
            let to = places[lead] + way * HAND_REACH + speed * duration;
            self.move_hand(lead, to, duration);
        }
    }

    fn move_hand(&mut self, i: usize, to: f32, duration: f32) {
        self.holds[i].shift = Some(HandShift {
            from: self.holds[i].along,
            to,
            elapsed: 0.0,
            duration,
        });
    }

    /// Midway between where the hands bear the body, along the edge from
    /// the grip: a hand on its way to a hold bears it where it left.
    fn weight_along(&self) -> f32 {
        let bears = |hold: &HandHold| hold.shift.map_or(hold.along, |shift| shift.from);
        0.5 * (bears(&self.holds[0]) + bears(&self.holds[1]))
    }

    /// Where hand `i` (0 left, 1 right) holds this frame: on the lip, or
    /// lifted over it on its way to the next hold.
    fn hand(&self, i: usize) -> Point3<f32> {
        let reading = &self.reading;
        let (right, out) = (reading.right(), reading.outward);
        let hold = &self.holds[i];
        let (along, lift) = match hold.shift {
            Some(shift) => {
                let t = shift.progress();
                let eased = t * t * (3.0 - 2.0 * t);
                (shift.from + (shift.to - shift.from) * eased, (PI * t).sin())
            }
            None => (hold.along, 0.0),
        };
        let on_lip = reading.grip() + right * along;
        Point3::new(on_lip.x, reading.lip_at(along), on_lip.z)
            + Vector3::y() * (HAND_LIFT * lift)
            + out * (HAND_CLEAR * lift)
    }

    fn climb_start(&self, figure: &FigureNow, rig: &CharacterRigConfig) -> ClimbStart {
        let grip = self.reading.grip();
        let posture = if self.mantle {
            Posture {
                pelvis_offset: Vector3::zeros(),
                torso_pitch: 0.0,
            }
        } else {
            Posture::hanging(self, figure.pelvis, figure.facing, rig)
        };
        let pelvis = figure.pelvis + posture.pelvis_offset;
        let feet = if self.mantle {
            figure.feet
        } else {
            hanging_feet(self, pelvis, figure.facing, rig)
        };
        let hands = if self.mantle {
            figure.hands
        } else {
            [self.hand(0), self.hand(1)]
        };
        ClimbStart {
            pelvis: pelvis - grip,
            torso_pitch: posture.torso_pitch,
            feet: feet.map(|f| f - grip),
            hands: hands.map(|h| h - grip),
            lead_right: self.swing.y <= 0.0,
        }
    }
}

/// The figure as it was last drawn, for a climb to start from.
pub struct FigureNow {
    pub pelvis: Point3<f32>,
    pub facing: Vector3<f32>,
    pub feet: [Point3<f32>; 2],
    pub hands: [Point3<f32>; 2],
}

/// The figure's pelvis offset and torso lean, placed under hands that hold
/// an edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Posture {
    pub pelvis_offset: Vector3<f32>,
    pub torso_pitch: f32,
}

/// How far the figure hangs in towards the wall from the capsule's axis: the
/// capsule is wider than the figure in it.
const HANG_IN: f32 = 0.08;
/// How much of the arm's length a hanger hangs on: near straight.
const HANG_ARM: f32 = 0.97;
/// How much of it is left when pulling up hard against a lean.
const PULL_ARM: f32 = 0.7;
/// How far the chest and head stay off the face, in metres.
const CHEST_CLEAR: f32 = 0.11;
/// The torso's lean in towards the wall while hanging, and how far back it
/// leans to drop and in to pull, in radians.
const HANG_PITCH: f32 = 0.22;
const DROP_PITCH: f32 = -0.45;
const PULL_PITCH: f32 = 0.12;

impl Posture {
    /// The figure's posture this frame: hanging, or along its climb.
    pub fn of(hang: &Hang, anim: &AnimationState, rig: &CharacterRigConfig) -> Self {
        match (hang.reading.phase, &hang.climb) {
            (HangPhase::Climb { progress }, Some(start)) => {
                let key = ClimbKeys::new(hang, start, anim.facing, rig);
                let pose = key.at(progress);
                Self {
                    pelvis_offset: pose.pelvis - anim.pelvis_position,
                    torso_pitch: pose.torso_pitch,
                }
            }
            _ => Self::hanging(hang, anim.pelvis_position, anim.facing, rig),
        }
    }

    /// Hanging: the shoulders where the body puts them, leaning in, drawn in
    /// to the hands if that is further than the arms reach. Pulling shortens
    /// the reach, which lifts the figure; dropping leans it back.
    fn hanging(
        hang: &Hang,
        pelvis: Point3<f32>,
        facing: Vector3<f32>,
        rig: &CharacterRigConfig,
    ) -> Self {
        let reading = &hang.reading;
        let (pull, drop) = (hang.lean.max(0.0), (-hang.lean).max(0.0));
        let torso_pitch = HANG_PITCH + DROP_PITCH * drop + PULL_PITCH * pull - 0.25 * hang.swing.x;
        let torso_up = Vector3::y() * torso_pitch.cos() + facing * torso_pitch.sin();
        let in_pelvis = pelvis + facing * HANG_IN;
        let shoulders = in_pelvis + torso_up * rig.torso_height;
        let grip = reading.grip();
        let reach = rig.arm_length() * (HANG_ARM + (PULL_ARM - HANG_ARM) * pull);
        let mut held = grip + clamp_length(shoulders - grip, reach);
        let off_face = (held - reading.face).dot(&reading.outward);
        if off_face < CHEST_CLEAR {
            held += reading.outward * (CHEST_CLEAR - off_face);
        }
        Self {
            pelvis_offset: held - shoulders + facing * HANG_IN + reading.right() * hang.sway,
            torso_pitch,
        }
    }
}

/// A vector no longer than `limit`.
fn clamp_length(v: Vector3<f32>, limit: f32) -> Vector3<f32> {
    let length = v.norm();
    if length > limit {
        v * (limit / length)
    } else {
        v
    }
}

/// Feet hanging from the hips: the legs swung by the pendulum, the knees
/// drawn up by the tuck, the toes resting on a wall below rather than in it.
fn hanging_feet(
    hang: &Hang,
    pelvis: Point3<f32>,
    facing: Vector3<f32>,
    rig: &CharacterRigConfig,
) -> [Point3<f32>; 2] {
    let reading = &hang.reading;
    let (out, right) = (reading.outward, reading.right());
    let leg = rig.leg_length();
    let hang_down =
        (-Vector3::y() + out * hang.swing.x.tan() + right * hang.swing.y.tan()).normalize();
    let length = leg * (0.95 - 0.4 * hang.tuck);
    [-1.0, 1.0].map(|side| {
        let hip = pelvis + right * (side * rig.hip_width);
        let mut foot = hip + hang_down * length + facing * (0.12 * hang.tuck);
        if reading.wall_below {
            let off = (foot - reading.face).dot(&out);
            if off < FOOT_CLEAR {
                foot += out * (FOOT_CLEAR - off);
            }
        }
        foot
    })
}

/// How far a foot resting on a face stays off it, in metres: the foot's
/// own thickness.
const FOOT_CLEAR: f32 = 0.05;

/// A climb's key poses, held to the edge: where it began, over the lip (hips
/// at the lip, chest out over the top, shoulders over the hands, the lead
/// knee up with its foot on the top), and standing where the body stands.
struct ClimbKeys {
    start: ClimbPose,
    over: ClimbPose,
    stand: ClimbPose,
    /// The feet on the top: the lead's over the lip, then each where it
    /// stands.
    lead_top: Point3<f32>,
    lead_stand: Point3<f32>,
    trail_top: Point3<f32>,
    lead_right: bool,
    hanging: [Point3<f32>; 2],
    hands_start: [Point3<f32>; 2],
    hands_held: [Point3<f32>; 2],
    hands_rest: [Point3<f32>; 2],
}

#[derive(Debug, Clone, Copy)]
struct ClimbPose {
    pelvis: Point3<f32>,
    torso_pitch: f32,
}

/// Where in a climb the figure is over the lip, as a fraction of it.
const OVER_AT: f32 = 0.5;
/// The lean over the lip, in radians.
const OVER_PITCH: f32 = 0.95;
/// When the lead foot swings up onto the top, and when the trailing one
/// follows it, as fractions of the climb.
const LEAD_STEP: (f32, f32) = (0.2, 0.45);
const TRAIL_STEP: (f32, f32) = (0.62, 0.85);
/// When the lead foot steps on from over the lip to stand under its hip.
const LEAD_SETTLE: (f32, f32) = (0.82, 0.97);
/// When a mantle's hands reach the lip, and when a climber's let go of it.
const HANDS_REACH: f32 = 0.2;
const HANDS_RELEASE: (f32, f32) = (0.75, 1.0);
/// How far in over the lip the lead foot lands, in metres.
const LEAD_IN: f32 = 0.1;
/// How high a foot stepping up lifts above the line it travels, in metres.
const STEP_LIFT: f32 = 0.12;

impl ClimbKeys {
    fn new(
        hang: &Hang,
        start: &ClimbStart,
        facing: Vector3<f32>,
        rig: &CharacterRigConfig,
    ) -> Self {
        let reading = &hang.reading;
        let grip = reading.grip();
        let (out, right) = (reading.outward, reading.right());
        let top = reading.top();
        let inward = -out;
        let reach = rig.arm_length();

        let over_pitch = OVER_PITCH;
        let over_up = Vector3::y() * over_pitch.cos() + facing * over_pitch.sin();
        let over_shoulders = grip + Vector3::y() * (reach * 0.85) + inward * 0.02;
        let over_pelvis = over_shoulders - over_up * rig.torso_height;

        let stand_centre = reading.stand.unwrap_or(grip + inward * 0.35);
        let stand_pelvis = Point3::new(
            stand_centre.x,
            top + rig.leg_length() * 0.94,
            stand_centre.z,
        );

        let lead_side = if start.lead_right { 1.0 } else { -1.0 };
        let on_top = |p: Point3<f32>| Point3::new(p.x, top, p.z);
        let lead_top = on_top(grip + inward * LEAD_IN + right * (lead_side * rig.hip_width));
        let trail_top = on_top(stand_pelvis - right * (lead_side * rig.hip_width));
        let lead_stand = on_top(stand_pelvis + right * (lead_side * rig.hip_width));

        let shoulder_rest = |side: f32| {
            stand_pelvis + Vector3::y() * rig.torso_height + right * (side * rig.shoulder_width)
                - Vector3::y() * (reach * 0.95)
        };
        Self {
            start: ClimbPose {
                pelvis: grip + start.pelvis,
                torso_pitch: start.torso_pitch,
            },
            over: ClimbPose {
                pelvis: over_pelvis,
                torso_pitch: over_pitch,
            },
            stand: ClimbPose {
                pelvis: stand_pelvis,
                torso_pitch: 0.0,
            },
            lead_top,
            lead_stand,
            trail_top,
            lead_right: start.lead_right,
            hanging: start.feet.map(|f| grip + f),
            hands_start: start.hands.map(|h| grip + h),
            hands_held: [hang.hand(0), hang.hand(1)],
            hands_rest: [shoulder_rest(-1.0), shoulder_rest(1.0)],
        }
    }

    /// The body `progress` of the way: up to the lip and over it, then up
    /// to stand.
    fn at(&self, progress: f32) -> ClimbPose {
        let (from, to, t) = if progress < OVER_AT {
            (self.start, self.over, ease(0.0, OVER_AT, progress))
        } else {
            (self.over, self.stand, ease(OVER_AT, 1.0, progress))
        };
        ClimbPose {
            pelvis: from.pelvis + (to.pelvis - from.pelvis) * t,
            torso_pitch: from.torso_pitch + (to.torso_pitch - from.torso_pitch) * t,
        }
    }

    /// Each foot `progress` of the way: hanging, then stepped up onto the
    /// top in an arc, the lead first over the lip; the trailing foot to where
    /// it stands, and the lead on from the lip to stand beside it.
    fn feet(&self, progress: f32) -> [Point3<f32>; 2] {
        let lead = if progress < LEAD_SETTLE.0 {
            step(
                self.hanging[self.lead()],
                self.lead_top,
                LEAD_STEP,
                progress,
            )
        } else {
            step(self.lead_top, self.lead_stand, LEAD_SETTLE, progress)
        };
        let trail = step(
            self.hanging[1 - self.lead()],
            self.trail_top,
            TRAIL_STEP,
            progress,
        );
        if self.lead_right {
            [trail, lead]
        } else {
            [lead, trail]
        }
    }

    fn lead(&self) -> usize {
        usize::from(self.lead_right)
    }

    /// Whether any foot is on the top yet.
    fn footed(&self, progress: f32) -> bool {
        progress >= LEAD_STEP.1
    }

    /// The hands `progress` of the way: a mantle's reaching the lip, held on
    /// it while the body goes over, then let go to hang at the sides.
    fn hands(&self, progress: f32) -> [Point3<f32>; 2] {
        let reach = ease(0.0, HANDS_REACH, progress);
        let release = ease(HANDS_RELEASE.0, HANDS_RELEASE.1, progress);
        [0, 1].map(|i| {
            let held = lerp(self.hands_start[i], self.hands_held[i], reach);
            lerp(held, self.hands_rest[i], release)
        })
    }
}

/// A foot moving from `from` to `to` over the `window` of the climb, in an
/// arc; before it, where it hung, and after, on the top.
fn step(from: Point3<f32>, to: Point3<f32>, window: (f32, f32), progress: f32) -> Point3<f32> {
    let t = ease(window.0, window.1, progress);
    let raw = ((progress - window.0) / (window.1 - window.0)).clamp(0.0, 1.0);
    lerp(from, to, t) + Vector3::y() * (STEP_LIFT * (PI * raw).sin())
}

fn lerp(a: Point3<f32>, b: Point3<f32>, t: f32) -> Point3<f32> {
    a + (b - a) * t
}

/// Smoothstep of `x` from `a` to `b`.
fn ease(a: f32, b: f32, x: f32) -> f32 {
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// How far the head moves along the facing to look up at the hands while
/// hanging (back), and over the top while climbing (forward), in metres.
const HEAD_UP: f32 = -0.03;
const HEAD_OVER: f32 = 0.05;

/// The lower body of a hanger or climber: the feet, the pelvis offset and
/// torso lean of [`Posture`], and the head looking up at the hands, then
/// over the top.
pub fn sample_hanging(
    hang: &Hang,
    anim: &AnimationState,
    rig: &CharacterRigConfig,
) -> PoseFragment {
    let posture = Posture::of(hang, anim, rig);
    let pelvis = anim.pelvis_position + posture.pelvis_offset;
    let (feet, anchor, head_down) = match (hang.reading.phase, &hang.climb) {
        (HangPhase::Climb { progress }, Some(start)) => {
            let keys = ClimbKeys::new(hang, start, anim.facing, rig);
            let anchor = if keys.footed(progress) {
                FootAnchor::World
            } else {
                FootAnchor::Hips
            };
            (keys.feet(progress), anchor, HEAD_OVER * (1.0 - progress))
        }
        // Feet resting on a wall stand on it: blended out of the hang (a
        // back-jump, a drop) they stay on it while the body leaves, the legs
        // stretching out to it, as a kick off it.
        _ => (
            hanging_feet(hang, pelvis, anim.facing, rig),
            if hang.reading.wall_below {
                FootAnchor::World
            } else {
                FootAnchor::Hips
            },
            HEAD_UP * (1.0 - (-hang.lean).max(0.0)),
        ),
    };
    PoseFragment {
        feet: Some(FeetPose {
            left: feet[0],
            right: feet[1],
            anchor,
        }),
        hands: None,
        pelvis_offset: Some(posture.pelvis_offset),
        shoulder_twist: None,
        head_tilt: Some(Vector2::new(head_down, 0.0)),
        head_bob: Some(0.0),
        torso_pitch: Some(posture.torso_pitch),
    }
}

/// The arms of a hanger or climber: the hands on their holds, hand over
/// hand along the edge, and on a climb held on the lip until the body is
/// over it, then let go. The shoulders turn a little after a hand that
/// moves.
pub fn sample_gripping(
    hang: &Hang,
    anim: &AnimationState,
    rig: &CharacterRigConfig,
) -> PoseFragment {
    let hands = match (hang.reading.phase, &hang.climb) {
        (HangPhase::Climb { progress }, Some(start)) => {
            ClimbKeys::new(hang, start, anim.facing, rig).hands(progress)
        }
        _ => [hang.hand(0), hang.hand(1)],
    };
    let reach = |i: usize| {
        hang.holds[i]
            .shift
            .map_or(0.0, |s| (PI * s.progress()).sin())
    };
    let twist = 0.12 * (reach(0) - reach(1));
    PoseFragment {
        feet: None,
        hands: Some(HandsPose {
            left: hands[0],
            right: hands[1],
        }),
        pelvis_offset: None,
        shoulder_twist: Some(twist),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}

/// The feet this frame's pose draws, for the legs to stand on when they
/// come back on: a climber's on the top.
pub fn drawn_feet(
    hang: &Hang,
    anim: &AnimationState,
    rig: &CharacterRigConfig,
) -> [Point3<f32>; 2] {
    let fragment = sample_hanging(hang, anim, rig);
    let feet = fragment.feet.expect("a hanger's pose draws its feet");
    [feet.left, feet.right]
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;
    const LIP: f32 = 2.0;
    /// Facing the wall, along +x; the face is the plane x = 0.
    const FACING: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);

    fn rig() -> CharacterRigConfig {
        CharacterRigConfig::default()
    }

    /// A lip at x = 0, the hands 4 cm in over it and `z` along it.
    fn reading(phase: HangPhase, z: f32, wall_below: bool) -> LedgeReading {
        LedgeReading {
            phase,
            hands: [
                Point3::new(0.04, LIP, z - 0.15),
                Point3::new(0.04, LIP, z + 0.15),
            ],
            outward: -Vector3::x(),
            face: Point3::new(0.0, LIP, z),
            stand: Some(Point3::new(0.39, LIP + 0.5, z)),
            edge_velocity: Vector3::zeros(),
            wall_below,
        }
    }

    fn hanging(lean: f32) -> LedgeReading {
        reading(HangPhase::Hang { lean }, 0.0, true)
    }

    /// The physics body hanging from the lip: its pelvis, at `z`.
    fn pelvis_at(z: f32) -> Point3<f32> {
        Point3::new(-0.26, LIP - 0.56, z)
    }

    fn anim(pelvis: Point3<f32>) -> AnimationState {
        let mut anim = AnimationState::new(pelvis, rig().leg_length());
        anim.facing = FACING;
        anim
    }

    fn figure(anim: &AnimationState) -> FigureNow {
        FigureNow {
            pelvis: anim.pelvis_position,
            facing: anim.facing,
            feet: [anim.left.position, anim.right.position],
            hands: [anim.left_hand.position, anim.right_hand.position],
        }
    }

    /// A hold settled at rest over `seconds`.
    fn settled(reading: LedgeReading, seconds: f32) -> (Hang, AnimationState) {
        let anim = anim(pelvis_at(0.0));
        let mut hang = Hang::begin(reading, Vector3::zeros());
        for _ in 0..(seconds / DT) as usize {
            hang.advance(reading, Vector3::zeros(), &figure(&anim), &rig(), DT);
        }
        (hang, anim)
    }

    /// The shoulders the skeleton puts on `posture`, left then right.
    fn shoulders(anim: &AnimationState, posture: &Posture) -> [Point3<f32>; 2] {
        let rig = rig();
        let (sin, cos) = posture.torso_pitch.sin_cos();
        let torso_up = Vector3::y() * cos + anim.facing * sin;
        let chest = anim.pelvis_position + posture.pelvis_offset + torso_up * rig.torso_height;
        let right = anim.facing.cross(&Vector3::y());
        [
            chest - right * rig.shoulder_width,
            chest + right * rig.shoulder_width,
        ]
    }

    #[test]
    fn a_hanger_hangs_on_near_straight_arms_with_its_chest_off_the_wall() {
        let (hang, anim) = settled(hanging(0.0), 1.0);
        let posture = Posture::of(&hang, &anim, &rig());
        let arm = rig().arm_length();
        for (i, shoulder) in shoulders(&anim, &posture).into_iter().enumerate() {
            let span = (hang.hand(i) - shoulder).norm();
            assert!(
                span <= arm + 1.0e-3,
                "hand {i} out of reach: {span} of {arm}"
            );
            assert!(span >= 0.85 * arm, "arm {i} bent at rest: {span} of {arm}");
        }
        let chest = nalgebra::center(
            &shoulders(&anim, &posture)[0],
            &shoulders(&anim, &posture)[1],
        );
        let off = (chest - hang.reading.face).dot(&hang.reading.outward);
        assert!(off >= CHEST_CLEAR - 1.0e-3, "chest {off} off the wall");
    }

    #[test]
    fn pulling_lifts_the_figure_and_dropping_leans_it_back() {
        let posture = |lean| {
            let (hang, anim) = settled(hanging(lean), 1.0);
            Posture::of(&hang, &anim, &rig())
        };
        let (rest, pull, drop) = (posture(0.0), posture(1.0), posture(-1.0));
        assert!(
            pull.pelvis_offset.y > rest.pelvis_offset.y + 0.05,
            "pulling lifts: {} against {}",
            pull.pelvis_offset.y,
            rest.pelvis_offset.y
        );
        assert!(
            drop.torso_pitch < rest.torso_pitch - 0.3,
            "dropping leans back"
        );
    }

    #[test]
    fn a_catch_swings_the_legs_and_gives_at_the_knees_then_settles() {
        let anim = anim(pelvis_at(0.0));
        let caught = reading(HangPhase::Catch { progress: 0.0 }, 0.0, false);
        let mut hang = Hang::begin(caught, Vector3::new(1.5, -3.0, 0.0));
        let mut most_tuck: f32 = 0.0;
        let mut most_swing: f32 = 0.0;
        for _ in 0..(3.0 / DT) as usize {
            hang.advance(caught, Vector3::zeros(), &figure(&anim), &rig(), DT);
            most_tuck = most_tuck.max(hang.tuck);
            most_swing = most_swing.max(hang.swing.x.abs());
        }
        assert!(most_tuck > TUCK_REST + 0.2, "the knees gave {most_tuck}");
        assert!(most_swing > 0.2, "the legs swung {most_swing}");
        assert!(
            (hang.tuck - TUCK_REST).abs() < 0.02,
            "settled tuck {}",
            hang.tuck
        );
        assert!(hang.swing.norm() < 0.02, "settled swing {}", hang.swing);
    }

    #[test]
    fn feet_rest_on_a_wall_below_and_never_in_it() {
        let anim = anim(pelvis_at(0.0));
        let held = hanging(0.0);
        let mut hang = Hang::begin(held, Vector3::new(-2.0, 0.0, 0.0));
        for _ in 0..(2.0 / DT) as usize {
            hang.advance(held, Vector3::zeros(), &figure(&anim), &rig(), DT);
            let posture = Posture::of(&hang, &anim, &rig());
            let pelvis = anim.pelvis_position + posture.pelvis_offset;
            for foot in hanging_feet(&hang, pelvis, FACING, &rig()) {
                let off = (foot - held.face).dot(&held.outward);
                assert!(off >= FOOT_CLEAR - 1.0e-4, "a foot {off} into the wall");
            }
        }
    }

    #[test]
    fn a_shimmy_leads_with_the_forward_hand_and_planted_hands_stay_put() {
        let speed = 1.0;
        let mut z = 0.0;
        let mut hang = Hang::begin(
            reading(HangPhase::Hang { lean: 0.0 }, z, true),
            Vector3::zeros(),
        );
        let velocity = Vector3::z() * speed;
        let mut was: [Option<Point3<f32>>; 2] = [None, None];
        let mut moves = Vec::new();
        let (mut furthest_lead, mut worst_trail) = (f32::MIN, f32::MAX);
        let (mut lead_near, mut trail_far) = (f32::MAX, f32::MIN);
        for _ in 0..(2.0 / DT) as usize {
            z += speed * DT;
            let held = reading(HangPhase::Hang { lean: 0.0 }, z, true);
            let anim = anim(pelvis_at(z));
            let before: Vec<bool> = hang.holds.iter().map(|h| h.shift.is_some()).collect();
            hang.advance(held, velocity, &figure(&anim), &rig(), DT);
            assert!(
                hang.holds.iter().any(|h| h.shift.is_none()),
                "both hands off the lip"
            );
            for i in 0..2 {
                let now = hang.hand(i);
                let planted = hang.holds[i].shift.is_none() && !before[i];
                if let (true, Some(then)) = (planted, was[i]) {
                    assert!(
                        (now - then).norm() < 1.0e-4,
                        "hand {i} slid {}",
                        (now - then).norm()
                    );
                }
                if hang.holds[i].shift.is_some() && !before[i] {
                    moves.push(i);
                }
                was[i] = Some(now);
                assert!(now.y >= LIP - 1.0e-4, "hand {i} below the lip");
                if hang.holds[i].shift.is_none() {
                    let along = hang.holds[i].along;
                    // The body starts at full speed, ahead of the first
                    // step; the trailing hand is judged from the second on.
                    if i == 1 {
                        furthest_lead = furthest_lead.max(along);
                        lead_near = lead_near.min(along - hang.sway);
                    } else if moves.len() >= 2 {
                        worst_trail = worst_trail.min(along - hang.sway);
                        trail_far = trail_far.max(along - hang.sway);
                    }
                }
            }
        }
        assert!(moves.len() >= 6, "{} hand moves in 2 m", moves.len());
        assert!(
            moves.windows(2).all(|pair| pair[0] != pair[1]),
            "hands alternate: {moves:?}"
        );
        assert_eq!(moves[0], 1, "the leading (right) hand moves first");
        let offset = hang.reading.hand_offset();
        assert!(
            furthest_lead > offset + 0.08,
            "the leading hand reaches {furthest_lead} m out, its place {offset} m"
        );
        assert!(
            lead_near > offset - 0.03,
            "the leading hand falls in to {lead_near} m off the figure, its place {offset} m"
        );
        // Stretched out a little behind the figure leaning on the leading hand,
        // before it follows; never across under the head.
        assert!(
            worst_trail > -offset - 0.07,
            "the trailing hand lags to {worst_trail} m off the figure, its place {} m",
            -offset
        );
        assert!(
            trail_far < 0.0,
            "the trailing hand crosses to {trail_far} m off the figure"
        );
    }

    /// A hold run up to a climb and through it, the figure's pose and hands
    /// at each step of `progress`.
    fn climb(mantle: bool) -> Vec<(f32, Hang, AnimationState)> {
        let mut anim = anim(pelvis_at(0.0));
        let mut frames = Vec::new();
        let mut hang = if mantle {
            anim.pelvis_position = Point3::new(-0.3, 0.47 + LIP - 0.72, 0.0);
            anim.left.position = Point3::new(-0.3, LIP - 0.72, -0.12);
            anim.right.position = Point3::new(-0.3, LIP - 0.72, 0.12);
            Hang::begin(
                reading(HangPhase::Climb { progress: 0.0 }, 0.0, true),
                Vector3::zeros(),
            )
        } else {
            settled(hanging(1.0), 1.0).0
        };
        for step in 0..=40 {
            let progress = step as f32 / 40.0;
            let held = reading(HangPhase::Climb { progress }, 0.0, true);
            hang.advance(held, Vector3::zeros(), &figure(&anim), &rig(), DT);
            frames.push((progress, hang.clone(), anim.clone()));
            anim.pelvis_position += Vector3::new(0.015, 0.025, 0.0);
        }
        frames
    }

    #[test]
    fn a_climb_starts_where_the_hang_was_and_ends_standing_on_the_top() {
        let (hang, anim) = settled(hanging(1.0), 1.0);
        let hung = anim.pelvis_position + Posture::of(&hang, &anim, &rig()).pelvis_offset;
        let frames = climb(false);
        let (_, first, at) = &frames[0];
        let started = at.pelvis_position + Posture::of(first, at, &rig()).pelvis_offset;
        assert!(
            (started - hung).norm() < 0.01,
            "the climb jumped {}",
            (started - hung).norm()
        );

        for (progress, hang, anim) in &frames {
            let hands = sample_gripping(hang, anim, &rig()).hands.unwrap();
            if *progress < HANDS_RELEASE.0 {
                for (i, hand) in [hands.left, hands.right].into_iter().enumerate() {
                    assert!(
                        (hand - hang.hand(i)).norm() < 1.0e-4,
                        "hand {i} off the lip at {progress}"
                    );
                }
            }
            let feet = drawn_feet(hang, anim, &rig());
            if *progress >= LEAD_STEP.1 {
                let lead = feet[usize::from(hang.climb.unwrap().lead_right)];
                assert!(
                    lead.y >= LIP - 1.0e-3 && lead.y <= LIP + STEP_LIFT + 1.0e-3,
                    "lead foot below the top or lifted past a step at {progress}: {lead}"
                );
            }
        }
        let (_, last, at) = frames.last().unwrap();
        let feet = drawn_feet(last, at, &rig());
        for foot in feet {
            assert!(
                (foot.y - LIP).abs() < 1.0e-3,
                "a foot off the top at the end: {foot}"
            );
            assert!(foot.x > 0.0, "a foot short of the lip at the end: {foot}");
        }
        let pelvis = at.pelvis_position + Posture::of(last, at, &rig()).pelvis_offset;
        assert!(pelvis.y > LIP + 0.4, "standing at the end: {pelvis}");
    }

    #[test]
    fn a_mantle_starts_from_how_the_figure_stood() {
        let frames = climb(true);
        let (_, first, at) = &frames[0];
        let posture = Posture::of(first, at, &rig());
        assert!(
            posture.pelvis_offset.norm() < 1.0e-3,
            "a mantle starts standing"
        );
        let feet = drawn_feet(first, at, &rig());
        assert!((feet[0] - at.left.position).norm() < 1.0e-3);
        assert!((feet[1] - at.right.position).norm() < 1.0e-3);
    }
}
