//! What one run of a scenario records.
//!
//! A `Take` is the tool's single intermediate product: the metrics read it, the
//! CSV export writes it, and the filmstrip renders from it. Keeping one record
//! rather than three means a number printed in the report and a frame drawn in
//! the sheet cannot describe different runs.
//!
//! Meshes are the exception to "record everything". A character mesh is a few
//! thousand vertices and a take is a few hundred frames, so the driver captures
//! them only on the stride the caller asks for — none at all when only the
//! numbers are wanted, which is the common case and needs no GPU.

use nalgebra::{Point3, Vector3};

use crate::animation::{FootPhase, GaitTiming, PlacerFoot};
use crate::rendering::vertex::Vertex;

/// A character mesh captured at one frame, ready to draw.
pub struct CapturedMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

/// One foot at one frame.
///
/// Both the placer's intent and the animator's output are recorded, because the
/// interesting failures live in the gap between them: a foot whose anchor is
/// where the placer put it but whose rendered position is somewhere else is a
/// different bug from a foot whose anchor never moved.
#[derive(Clone, Debug)]
pub struct FootSample {
    pub planted: bool,
    /// Where the foot is drawn, from the skeleton.
    ///
    /// Not necessarily where animation asked for it: `Skeleton::apply_fragment`
    /// pulls a foot back on to the sphere its leg can reach, because a leg is
    /// as long as it is. The gap between this and `requested` is what that cost
    /// — see `overreach`.
    pub rendered: Point3<f32>,
    /// Where the pose layer asked the foot to be, before the leg's reach was
    /// applied. The two differ only when the gait asked for something the rig
    /// cannot do.
    pub requested: Point3<f32>,
    /// The placer's anchor: where this foot believes it is standing.
    pub anchor: Point3<f32>,
    /// Where the capture point says the foot ought to be right now.
    pub ideal: Point3<f32>,
    /// The hip this foot hangs from, after IK.
    pub hip: Point3<f32>,
    /// Ground the probe found under this foot's aim point, if any.
    pub probe: Option<Point3<f32>>,
    /// Seconds in the current stance (saturating; meaningless while stepping).
    pub since_plant: f32,
    /// Visual toe-off roll in `[0, 1]` while still planted.
    pub pre_lift: f32,
    /// True when this swing's landing was pulled in for want of ground.
    pub landing_shortened: bool,
}

impl FootSample {
    pub fn capture(
        foot: &PlacerFoot,
        rendered: Point3<f32>,
        requested: Point3<f32>,
        hip: Point3<f32>,
    ) -> Self {
        Self {
            planted: matches!(foot.phase, FootPhase::Planted),
            rendered,
            requested,
            anchor: foot.planted_position,
            ideal: foot.ideal_xz,
            hip,
            probe: None,
            since_plant: foot.since_plant,
            pre_lift: foot.pre_lift,
            landing_shortened: foot.landing_shortened,
        }
    }

    /// Straight-line hip-to-foot distance the gait asked for.
    ///
    /// Measured from `requested`, not from the drawn foot: the rig clamps at
    /// full leg length, so the drawn distance saturates at 100% and says only
    /// "the clamp engaged". The requested distance says by how much — at the
    /// leg's full length the knee is locked straight, and past it the foot is
    /// being dragged short of where the gait wanted it, which is what a foot
    /// glued to the floor looks like from outside.
    pub fn extension(&self) -> f32 {
        (self.requested - self.hip).magnitude()
    }

    /// How far past the leg's full reach the foot was *asked* to go.
    ///
    /// Measured against the requested position, not the drawn one: the rig
    /// clamps, so the drawn foot can never exceed the leg and measuring it
    /// would report a clean gait no matter what the gait asked for. This is
    /// the sticky-feet number — every millimetre of it is a millimetre the
    /// foot is being dragged short of where the gait wanted it.
    pub fn overreach(&self, leg_length: f32) -> f32 {
        (self.extension() - leg_length).max(0.0)
    }
}

/// The cadence the gait was planning to, at one frame.
#[derive(Clone, Copy, Debug)]
pub struct TimingSample {
    pub trigger_threshold: f32,
    pub cycle_distance: f32,
    pub duty_factor: f32,
    pub swing_duration: f32,
}

impl From<GaitTiming> for TimingSample {
    fn from(timing: GaitTiming) -> Self {
        Self {
            trigger_threshold: timing.trigger_threshold,
            cycle_distance: timing.cycle_distance,
            duty_factor: timing.duty_factor,
            swing_duration: timing.swing_duration,
        }
    }
}

/// One display frame of a scenario.
pub struct FrameSample {
    pub index: usize,
    pub time: f32,
    pub dt: f32,

    /// The beat of the script that was running — "walk", "crouch", "jump".
    pub beat: &'static str,
    /// Short label for the animator's lower-body pose variant.
    pub pose: &'static str,

    pub pelvis: Point3<f32>,
    /// World-space velocity of the body, platform motion included.
    pub velocity: Vector3<f32>,
    pub yaw: f32,
    pub grounded: bool,

    /// Velocity of the surface the character is standing on, and how far that
    /// surface has travelled since the scenario began.
    ///
    /// A gait means nothing in world coordinates when the floor is moving: a
    /// character riding a platform at 3 m/s is standing still, and a foot that
    /// holds a world position is sliding. Everything measured about the feet is
    /// therefore measured in the surface's frame — see `on_surface`.
    pub support_velocity: Vector3<f32>,
    pub support_offset: Vector3<f32>,

    pub gait_phase: f32,
    pub stride_phase: f32,
    pub stride_activity: f32,
    /// `None` while the placer is suspended and has planned no cadence.
    pub timing: Option<TimingSample>,

    pub left: FootSample,
    pub right: FootSample,

    /// Present only on frames the driver was asked to capture.
    pub mesh: Option<CapturedMesh>,
}

impl FrameSample {
    /// Speed across the ground — the body's, less the floor's. This is the
    /// speed the gait was planned for, and on static ground it is the world
    /// speed.
    pub fn horizontal_speed(&self) -> f32 {
        let relative = self.velocity - self.support_velocity;
        Vector3::new(relative.x, 0.0, relative.z).magnitude()
    }

    /// A world point expressed in the surface's own frame, on the plane.
    ///
    /// Two feet a metre apart on a moving platform are a metre apart in this
    /// frame at every instant, however far the platform has travelled; a foot
    /// that stays still in it is a foot planted on the plank.
    pub fn on_surface(&self, point: Point3<f32>) -> nalgebra::Vector2<f32> {
        nalgebra::Vector2::new(
            point.x - self.support_offset.x,
            point.z - self.support_offset.z,
        )
    }

    pub fn foot(&self, side: Side) -> &FootSample {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }
}

/// Which foot. A local mirror of the placer's own side enum, so metrics can
/// index a frame without importing placer internals.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub const BOTH: [Side; 2] = [Side::Left, Side::Right];

    pub fn label(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

/// A complete recorded run.
pub struct Take {
    /// The scenario that produced it.
    pub scenario: String,
    /// The ground it was run over, by name.
    pub ground: String,
    /// Leg length of the rig, for expressing extensions as a fraction.
    pub leg_length: f32,
    /// Standing pelvis height the rig was designed around — the LIP pendulum
    /// length behind the cadence.
    pub standing_height: f32,
    /// Pelvis height the body actually rode at. Equal to `standing_height`
    /// only if the physics capsule happens to agree with the rig.
    pub ride_height: f32,
    pub frames: Vec<FrameSample>,
}

impl Take {
    pub fn duration(&self) -> f32 {
        self.frames.last().map(|f| f.time).unwrap_or(0.0)
    }

    /// Frames of one beat, by label. Transitions are judged per beat: a walk
    /// that only misbehaves after the crouch is a different finding from one
    /// that misbehaves throughout.
    pub fn beat<'a>(&'a self, label: &'a str) -> impl Iterator<Item = &'a FrameSample> + 'a {
        self.frames.iter().filter(move |f| f.beat == label)
    }

    /// Distinct beat labels in the order they first ran.
    pub fn beats(&self) -> Vec<&'static str> {
        let mut out: Vec<&'static str> = Vec::new();
        for frame in &self.frames {
            if out.last() != Some(&frame.beat) {
                out.push(frame.beat);
            }
        }
        out
    }
}
