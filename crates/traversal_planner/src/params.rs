//! What the body and its legs can do: the limits every plan keeps within.

use std::ops::Range;

use serde::{Deserialize, Serialize};

use crate::path::PathVec;
use crate::plan::Limb;

/// The capsule that is the body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BodyDims {
    /// Capsule radius, in metres.
    pub radius: f32,
    /// Capsule height, end to end, in metres.
    pub height: f32,
    /// Steepest body floor, in degrees from level, the capsule can be driven
    /// over on its contacts. Steeper, as on a step's rounded lip or out of a
    /// wide gap, it has to fly. The drive climbs a long ramp far steeper
    /// (`character_viewer contract`), but meeting a slope's foot at speed is
    /// a collision, which stalls the capsule and pops it up, and rides do not
    /// model it: the limit keeps a voxel lip's 45° bevel a hop. Down a
    /// floor falling ahead the limit is the feet's, [`FootDims::max_slope_degrees`]:
    /// gravity carries the body down it.
    pub max_ground_slope_degrees: f32,
    /// Fastest change of speed along the path the drive gives on the ground,
    /// in m/s²: its mean from standing to speed (`character_viewer contract`).
    pub max_acceleration: f32,
    /// How far the floor may fall below the arc the body would fly riding
    /// off it and still be ridden, in metres: over a brow, a slat's dip or
    /// an arch's corners at speed the body lifts off by about this much,
    /// and is carried on as on the ground rather than by a flight.
    pub skim_height: f32,
    /// How long a body rolling off the floor while rising keeps rising, in
    /// seconds: off a brow it carries the climb it rode up with for this
    /// long, and then the drive cancels the rise. The game's roll-off rise
    /// held (`LocomotionConfig::roll_off_rise_held`, `character_viewer
    /// contract`).
    pub rise_held: f32,
}

/// How much steeper than `max_ground_slope_degrees` a floor may read and
/// still be ridden, in degrees: a voxel mesh facets a slope built at the limit
/// a little either side of it.
const SLOPE_FACETING_DEGREES: f32 = 1.0;

impl BodyDims {
    /// The steepest floor ridden as sampled, in radians: the limit, and the
    /// faceting a mesh gives a slope built at it.
    pub fn steepest_ride(&self) -> f32 {
        (self.max_ground_slope_degrees + SLOPE_FACETING_DEGREES).to_radians()
    }

    /// Speed across the ground the drive keeps on a floor rising `grade`
    /// metres per metre (falling, if negative), asked for `speed` on the
    /// level: it presses the asked speed along the floor's face, which the
    /// contact turns into `speed·cos θ` along the slope, `speed·cos²θ` across
    /// the ground. Measured by `character_viewer contract`.
    pub fn ground_speed(&self, speed: f32, grade: f32) -> f32 {
        speed / (1.0 + grade * grade)
    }

    /// Height of the top sphere's centre above the bottom sphere's.
    pub fn spine(&self) -> f32 {
        self.height - 2.0 * self.radius
    }
}

impl Default for BodyDims {
    /// The player's capsule: 0.5 m across and 1 m tall.
    fn default() -> Self {
        Self {
            radius: 0.25,
            height: 1.0,
            max_ground_slope_degrees: 40.0,
            max_acceleration: 20.0,
            skim_height: 0.05,
            rise_held: 0.08,
        }
    }
}

/// A foot, and how far it may land from where it naturally falls.
///
/// A foot's offset across the path: the left foot's natural offset is
/// `+stance_half_width`, the right's `-stance_half_width`; outward is away
/// from the path.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootDims {
    /// Radius of the foothold casts, in metres: a sole's size.
    pub radius: f32,
    /// Offset across the path where each foot lands on open ground, in metres.
    pub stance_half_width: f32,
    /// How far past its natural offset a foot may land outward, in metres.
    pub reach_out: f32,
    /// How far inward of its natural offset a foot may land, in metres; past
    /// `stance_half_width` it crosses over the other foot's side.
    pub reach_in: f32,
    /// Steepest ground, in degrees from level, a foot can stand on.
    pub max_slope_degrees: f32,
    /// Height of the hips above the capsule's bottom, in metres: where the
    /// legs hang from, standing.
    pub hip_height: f32,
    /// Farthest a foot reaches from its hip, in metres: the leg straightened.
    /// Down into a dip, back to the foot behind on a rise and on ahead to
    /// the next, all within it.
    pub leg_reach: f32,
    /// Highest a foot lands above the capsule's bottom, in metres: onto a lip
    /// the capsule is still rounding.
    pub above_body: f32,
    /// Least distance from a foothold to its surface's edge, in metres: less,
    /// and the sole hangs off.
    pub min_edge_margin: f32,
}

impl Default for FootDims {
    /// The player's feet as the animation placed them on 2026-10-26: across
    /// the path half its hips apart, reaching half the leg's level reach to
    /// either side, on legs hung at its standing height that straighten to
    /// its plant reach; the game passes its own.
    fn default() -> Self {
        Self {
            radius: 0.05,
            stance_half_width: 0.12,
            reach_out: 0.154,
            reach_in: 0.154,
            max_slope_degrees: 50.0,
            hip_height: 0.425,
            leg_reach: 0.525,
            above_body: 0.1,
            min_edge_margin: 0.04,
        }
    }
}

impl FootDims {
    /// Where `limb` lands across the path on open ground.
    pub fn natural_offset(&self, limb: Limb) -> f32 {
        match limb {
            Limb::LeftFoot => self.stance_half_width,
            Limb::RightFoot => -self.stance_half_width,
        }
    }

    /// The offsets across the path `limb` may land at.
    pub fn reach(&self, limb: Limb) -> Range<f32> {
        let natural = self.natural_offset(limb);
        match limb {
            Limb::LeftFoot => natural - self.reach_in..natural + self.reach_out,
            Limb::RightFoot => natural - self.reach_out..natural + self.reach_in,
        }
    }
}

/// How far one footfall may be from the last.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StrideLimits {
    /// Shortest step along the path, in metres.
    pub min_length: f32,
    /// Longest step along the path without a flight, in metres: as far as
    /// the legs reach. Longer is a hop, a running leap.
    pub max_length: f32,
    /// Highest a footfall may be above the last, in metres.
    pub max_rise: f32,
    /// Lowest a footfall may be below the last, in metres.
    pub max_drop: f32,
    /// The step the legs' gait takes at each speed.
    pub natural: NaturalSteps,
}

impl Default for StrideLimits {
    /// The player's legs as the animation planned them on 2026-10-26, from
    /// its smallest step to its longest without a flight; the game passes
    /// its own.
    fn default() -> Self {
        Self {
            min_length: 0.05,
            max_length: 0.81,
            max_rise: 0.45,
            max_drop: 8.0,
            natural: NaturalSteps::default(),
        }
    }
}

/// The step the legs' gait takes on level ground at one speed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NaturalStep {
    /// The speed, in m/s.
    pub speed: f32,
    /// Hip travel from footfall to footfall, in metres.
    pub length: f32,
    /// How far ahead of the hips a foot plants, in metres, and how far
    /// behind them it lifts: the stance is symmetric about the hips. Less
    /// than half the step, and the body flies between stances.
    pub plant_ahead: f32,
}

/// The step the legs' gait takes on level ground at each of a set of
/// speeds, speeds increasing: the game fills it from the animation, whose
/// footfalls the plan's must be. Read linearly between speeds and held past
/// the ends.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NaturalSteps(pub Vec<NaturalStep>);

impl NaturalSteps {
    /// The natural step at `speed`.
    pub fn at(&self, speed: f32) -> NaturalStep {
        let steps = &self.0;
        let Some(&first) = steps.first() else {
            return NaturalStep {
                speed,
                length: 0.0,
                plant_ahead: 0.0,
            };
        };
        if speed <= first.speed {
            return NaturalStep { speed, ..first };
        }
        steps.windows(2).find(|pair| speed <= pair[1].speed).map_or(
            NaturalStep {
                speed,
                ..steps[steps.len() - 1]
            },
            |pair| {
                let (slow, fast) = (pair[0], pair[1]);
                let u = (speed - slow.speed) / (fast.speed - slow.speed);
                NaturalStep {
                    speed,
                    length: slow.length + (fast.length - slow.length) * u,
                    plant_ahead: slow.plant_ahead + (fast.plant_ahead - slow.plant_ahead) * u,
                }
            },
        )
    }

    /// The shortest of the steps, in metres: the slowest walk's.
    pub fn shortest(&self) -> f32 {
        self.0
            .iter()
            .map(|step| step.length)
            .fold(f32::INFINITY, f32::min)
    }
}

impl Default for NaturalSteps {
    /// The player's legs, walking to 5 m/s and sprinting above, as the
    /// animation planned them on 2026-10-26; the game passes its own.
    fn default() -> Self {
        Self(vec![
            NaturalStep {
                speed: 0.5,
                length: 0.083,
                plant_ahead: 0.052,
            },
            NaturalStep {
                speed: 1.0,
                length: 0.167,
                plant_ahead: 0.103,
            },
            NaturalStep {
                speed: 1.5,
                length: 0.25,
                plant_ahead: 0.132,
            },
            NaturalStep {
                speed: 2.0,
                length: 0.333,
                plant_ahead: 0.127,
            },
            NaturalStep {
                speed: 2.5,
                length: 0.416,
                plant_ahead: 0.158,
            },
            NaturalStep {
                speed: 3.0,
                length: 0.5,
                plant_ahead: 0.190,
            },
            NaturalStep {
                speed: 3.5,
                length: 0.583,
                plant_ahead: 0.221,
            },
            NaturalStep {
                speed: 4.0,
                length: 0.666,
                plant_ahead: 0.253,
            },
            NaturalStep {
                speed: 4.5,
                length: 0.73,
                plant_ahead: 0.277,
            },
            NaturalStep {
                speed: 5.0,
                length: 0.73,
                plant_ahead: 0.277,
            },
            NaturalStep {
                speed: 5.5,
                length: 0.687,
                plant_ahead: 0.261,
            },
            NaturalStep {
                speed: 6.0,
                length: 0.73,
                plant_ahead: 0.277,
            },
        ])
    }
}

/// What a flight between footfalls may ask of the body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlightLimits {
    /// Least gap, in metres, between the capsule and the ground it flies over.
    pub clearance: f32,
    /// Fastest upward speed at takeoff, in m/s: about a 0.45 m hop.
    pub max_takeoff_climb: f32,
}

impl Default for FlightLimits {
    fn default() -> Self {
        Self {
            clearance: 0.03,
            max_takeoff_climb: 3.0,
        }
    }
}

/// Standard gravity, in m/s²: the default until the game passes its own.
pub const STANDARD_GRAVITY: f32 = 9.81;

/// The limits every plan keeps within. The game fills them in from the
/// capsule and the rig's legs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GaitParams {
    pub body: BodyDims,
    pub foot: FootDims,
    pub stride: StrideLimits,
    pub flight: FlightLimits,
    /// Downward acceleration, in m/s².
    pub gravity: f32,
}

impl GaitParams {
    /// How far along the path a hop at `speed` carries, landing as high as
    /// it left, pushed off with the most climb the gait gives, in metres:
    /// further than any hop that lands where it took off from would need.
    pub fn longest_hop(&self, speed: f32) -> f32 {
        speed * 2.0 * self.flight.max_takeoff_climb / self.gravity
    }

    /// How far ahead of the hips a foot plants, and how far behind them it
    /// lifts, taking steps `step` long at `speed`, in metres: the gait's
    /// share of the step planted, whatever the ground makes of the step.
    pub fn plant_ahead(&self, speed: f32, step: f32) -> f32 {
        let natural = self.stride.natural.at(speed);
        if natural.length <= 0.0 {
            return 0.0;
        }
        step * natural.plant_ahead / natural.length
    }

    /// The longest step at `speed` whose stance keeps the foot within reach
    /// of the body, on ground rising `grade` metres per metre (falling, if
    /// negative), in metres: the foot lifts its plant-ahead share of the
    /// step behind the hips, the lower foot runs out of reach first, and on
    /// a slope the capsule rests clear of the ground beneath it.
    pub fn reachable_step(&self, grade: f32, speed: f32) -> f32 {
        let grade = grade.abs();
        let slope = 1.0 + grade * grade;
        let rest_gap = self.body.radius * (slope.sqrt() - 1.0);
        let hip = self.foot.hip_height + rest_gap;
        let reach = self.foot.leg_reach;
        let room = (hip * grade).powi(2) - slope * (hip * hip - reach * reach);
        if room <= 0.0 {
            return 0.0;
        }
        let half_stance = ((room.sqrt() - hip * grade) / slope).max(0.0);
        let natural = self.stride.natural.at(speed);
        if natural.plant_ahead <= 0.0 || natural.length <= 0.0 {
            return 2.0 * half_stance;
        }
        half_stance * natural.length / natural.plant_ahead
    }

    /// Whether the legs step on ground rising `grade` metres per metre at
    /// `speed`: whether even the gait's shortest step keeps the foot behind
    /// within reach of the body resting there. Past it the stick and the
    /// drive carry the body.
    pub fn steps_up(&self, grade: f32, speed: f32) -> bool {
        self.reachable_step(grade, speed) >= self.stride.natural.shortest()
    }

    /// Whether a foot at `foot` is within reach of the body with its
    /// bottom-sphere centre at `body_s`, `body_z`: within a leg's reach of
    /// the nearer hip, along the path, across it and up, and no higher above
    /// the capsule's bottom than a foot lands.
    pub fn foot_reaches(&self, body_s: f32, body_z: f32, foot: &PathVec) -> bool {
        let bottom = body_z - self.body.radius;
        let hip = self.foot.stance_half_width;
        let along = foot.s - body_s;
        let across = (foot.d - hip).abs().min((foot.d + hip).abs());
        let down = bottom + self.foot.hip_height - foot.z;
        along * along + across * across + down * down <= self.foot.leg_reach * self.foot.leg_reach
            && foot.z <= bottom + self.foot.above_body
    }
}

impl Default for GaitParams {
    fn default() -> Self {
        Self {
            body: BodyDims::default(),
            foot: FootDims::default(),
            stride: StrideLimits::default(),
            flight: FlightLimits::default(),
            gravity: STANDARD_GRAVITY,
        }
    }
}
