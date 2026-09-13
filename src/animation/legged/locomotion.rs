//! Rig-independent legged locomotion.
//!
//! Everything a two-legged body needs to walk, and nothing about what it
//! looks like while doing it:
//!
//! ```text
//!   body position ─┬─► pelvis_for ──► pelvis
//!                  │
//!   Grounding ─────┴─► observe_support ──► support frame
//!                                              │
//!   probes ──► process_contacts ──► FootGround │
//!                                              ▼
//!   intent, yaw, velocity ──────────────► FootPlacer ──► feet
//! ```
//!
//! A rig on top of this reads the feet and draws legs from them. It is the
//! rig's business how many bones that takes, whether there is a head above
//! it, and what the thing is made of; none of that reaches down here.

use nalgebra::{Point3, Vector3};

use crate::character::Grounding;
use crate::sensing::{ContactCandidate, Probe};

use super::super::foot_placer::{FootPlacer, FootSide, PlacerCtx, PlacerFoot, PlacerRecorder};
use super::super::FootPlacerConfig;
use super::dims::LegRigDims;
use super::ground::{self, FootGround};

/// How long locomotion keeps animating in a lost surface's frame, in
/// seconds.
///
/// Contact grounding chatters: a walking capsule leaves the floor between
/// footfalls, and that frame is not the frame the character stepped off the
/// platform. Without the memory the gait's frame of reference would flicker
/// between the platform's and the world's at footfall rate, which is a
/// perturbation at exactly the frequency of the gait it is perturbing. Short
/// enough that a real departure — a jump, a walk-off — is animated against the
/// world within a few frames.
const SUPPORT_MEMORY: f32 = 0.15;

/// Per-frame inputs to [`LeggedLocomotion::tick`].
///
/// `velocity` is expected support-relative — call
/// [`LeggedLocomotion::observe_support`] first and subtract what it returns.
/// The two are separate fields because the placer needs both: the gait is
/// planned in the support's frame, while planted anchors are carried along
/// by the support's world motion.
pub struct LocomotionCtx {
    pub dt: f32,
    pub pelvis: Point3<f32>,
    pub yaw: f32,
    /// Pelvis velocity relative to whatever is holding the body up.
    pub velocity: Vector3<f32>,
    /// World velocity of that surface.
    pub support_velocity: Vector3<f32>,
    /// Requested movement direction, before physics velocity has
    /// necessarily caught up. Drives anticipatory start steps.
    pub intent_direction: Vector3<f32>,
    /// Whether the body is off the ground, which suspends stepping.
    pub airborne: bool,
    /// Peak height of a swing arc, from the active gait.
    pub step_height: f32,
    /// Fraction of the capture point a step plants at, from the active
    /// gait. Sets the cadence as well as the foothold.
    pub stride_gain: f32,
    /// Short tag naming the caller's gait state. Recorder only.
    pub pose_tag: &'static str,
}

/// A two-legged body's contact with the ground, and the gait that keeps it
/// there.
///
/// Owns the foot placer, the probe readings that feed it, and the support
/// frame the whole thing is expressed in. Construct one per legged entity,
/// whatever rig is drawn around it.
pub struct LeggedLocomotion {
    dims: LegRigDims,
    config: FootPlacerConfig,
    placer: FootPlacer,

    left_ground: FootGround,
    right_ground: FootGround,

    /// Velocity of the surface last seen holding the body up, and how long
    /// ago that was. See [`SUPPORT_MEMORY`].
    support_velocity: Vector3<f32>,
    support_age: f32,

    /// How far the rig's pelvis hangs below the physics body's origin.
    ///
    /// The two are not the same point and never were: the body is a capsule
    /// whose centre rides half its height above the floor, while the rig's
    /// pelvis belongs one `standing_height` above the soles — which is less,
    /// because a standing body has bent knees. Feeding the body's origin
    /// straight in as the pelvis stretches every leg to its full length before
    /// a single step is taken, and a leg with no bend left in it can only
    /// answer a stride by dragging its foot. See [`Self::pelvis_for`].
    body_to_pelvis: f32,

    /// Previous-frame yaw, used to derive yaw rate for the placer.
    last_yaw: f32,

    /// Optional input/output recorder for the foot placer (enabled via
    /// the `PLACER_REC` env var). Feeds the offline replay harness.
    recorder: Option<PlacerRecorder>,
}

impl LeggedLocomotion {
    /// Start a body at `body_position` whose origin rides `ground_clearance`
    /// above the surface it rests on — for a capsule collider, its half
    /// height.
    pub fn new(
        dims: LegRigDims,
        config: FootPlacerConfig,
        body_position: Point3<f32>,
        ground_clearance: f32,
        yaw: f32,
    ) -> Self {
        // A rig taller than the body rides is not something an offset can
        // fix — raising the pelvis to suit would put the feet below the floor
        // — so that case keeps the origin and the old behaviour.
        let body_to_pelvis = (ground_clearance - dims.standing_height).max(0.0);
        let pelvis = body_position - Vector3::y() * body_to_pelvis;
        // The rest pose is built facing the way the body does. Placing the
        // first stance along a fixed axis leaves a body that spawns facing
        // anywhere else standing with its feet across its own path, and it
        // walks the first half-second out of that.
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
        let foot_centre_y = pelvis.y - dims.standing_height;

        Self {
            dims,
            config,
            placer: FootPlacer::new(pelvis, facing, dims.hip_width, foot_centre_y),
            left_ground: FootGround::default(),
            right_ground: FootGround::default(),
            support_velocity: Vector3::zeros(),
            support_age: SUPPORT_MEMORY,
            body_to_pelvis,
            last_yaw: yaw,
            recorder: PlacerRecorder::from_env(pelvis, yaw, dims.hip_width, foot_centre_y),
        }
    }

    /// Where the rig's pelvis belongs, given where the physics body is.
    ///
    /// The seam between the body the physics engine moves and the skeleton
    /// drawn around it. Everything that hands locomotion a position hands it
    /// a body position and goes through here.
    pub fn pelvis_for(&self, body_position: Point3<f32>) -> Point3<f32> {
        body_position - Vector3::y() * self.body_to_pelvis
    }

    /// The measurements this gait is planned against.
    pub fn dims(&self) -> LegRigDims {
        self.dims
    }

    /// The foot placer, for rigs reading foot poses and for debug overlays.
    pub fn placer(&self) -> &FootPlacer {
        &self.placer
    }

    /// One foot's placer state.
    pub fn foot(&self, side: FootSide) -> &PlacerFoot {
        match side {
            FootSide::Left => &self.placer.left,
            FootSide::Right => &self.placer.right,
        }
    }

    /// The latest probe reading under one foot.
    pub fn ground(&self, side: FootSide) -> FootGround {
        match side {
            FootSide::Left => self.left_ground,
            FootSide::Right => self.right_ground,
        }
    }

    /// The height of the ground under whichever foot has a reading,
    /// falling back to the rest-rig terrain level under `pelvis`.
    ///
    /// `standing_height` is measured pelvis-to-foot-centre and a planted
    /// foot's centre tracks the surface, so the fallback is where the foot
    /// would be standing if it were.
    pub fn ground_height(&self, pelvis: Point3<f32>) -> f32 {
        self.left_ground
            .contact
            .or(self.right_ground.contact)
            .map(|c| c.y)
            .unwrap_or(pelvis.y - self.dims.standing_height)
    }

    /// Aim a probe from each hip at what that foot cares about this frame.
    ///
    /// While stepping that is the committed landing target, so the swing
    /// reads the terrain height where it will plant rather than where the
    /// foot currently hangs; while planted it is the anchor, so the foot
    /// tracks the surface directly beneath it. A suspended placer has no
    /// opinion, so the caller supplies where its feet are being drawn —
    /// `[left, right]`.
    pub fn configure_probes(
        &self,
        pelvis: Point3<f32>,
        yaw: f32,
        suspended_aim: [Point3<f32>; 2],
    ) -> Vec<Probe> {
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
        let right = facing.cross(&Vector3::y());
        let suspended = self.placer.is_suspended();

        [FootSide::Left, FootSide::Right]
            .into_iter()
            .zip(suspended_aim)
            .map(|(side, drawn)| {
                let hip = pelvis + right * (side.side_sign() * self.dims.hip_width);
                let aim = if suspended {
                    drawn
                } else {
                    self.foot(side).probe_anchor()
                };
                self.foot_probe(ground::probe_tag(side), hip, aim)
            })
            .collect()
    }

    /// A ray from the hip through the aim point, falling back to straight
    /// down when the aim point is degenerate (at the hip itself).
    ///
    /// A probe must always outreach the point it is aimed at. The placer
    /// reads "no ground under the landing target" as "that target is
    /// illegal" and shortens the step toward the takeoff position, never
    /// re-extending it within the swing — so a target beyond the probe's
    /// range does not merely go unmeasured, it cancels the stride. At
    /// speed the target is projected a whole swing's worth of hip travel
    /// ahead, which outruns any fixed multiple of leg length:
    /// `standing_height` past the target is the margin for ground that
    /// falls away below it.
    fn foot_probe(&self, tag: u32, hip: Point3<f32>, aim: Point3<f32>) -> Probe {
        let to_aim = aim - hip;
        let direction = to_aim
            .try_normalize(1e-4)
            .unwrap_or_else(|| Vector3::new(0.0, -1.0, 0.0));

        Probe {
            tag,
            origin: hip,
            direction,
            length: self
                .dims
                .probe_length
                .max(to_aim.magnitude() + self.dims.standing_height),
        }
    }

    /// Take this frame's probe results.
    pub fn process_contacts(&mut self, contacts: &[ContactCandidate]) {
        let (left, right) = ground::resolve(contacts);
        self.left_ground = left;
        self.right_ground = right;
    }

    /// Track the surface the body is standing on, holding the last one
    /// through the chatter of a contact set. Returns the frame this
    /// update's animation is expressed in.
    pub fn observe_support(&mut self, grounding: &Grounding, dt: f32) -> Vector3<f32> {
        if grounding.is_grounded {
            self.support_velocity = grounding.surface_velocity;
            self.support_age = 0.0;
        } else {
            self.support_age += dt;
            if self.support_age >= SUPPORT_MEMORY {
                self.support_velocity = Vector3::zeros();
            }
        }
        self.support_velocity
    }

    /// Advance the gait by one frame.
    pub fn tick(&mut self, ctx: &LocomotionCtx) {
        let yaw_rate = if ctx.dt > 0.0 {
            shortest_angle_diff(ctx.yaw, self.last_yaw) / ctx.dt
        } else {
            0.0
        };
        self.last_yaw = ctx.yaw;

        self.placer.set_suspended(ctx.airborne);

        // Fallback foot-centre y used by the placer when no probe hit is
        // available: the rest-rig terrain level. It is also the rest
        // vertical the leg's reach budget is measured against, so a wrong
        // value here mis-sizes every stride the planner aims.
        let placer_ctx = PlacerCtx {
            dt: ctx.dt,
            pelvis: ctx.pelvis,
            velocity: ctx.velocity,
            support_velocity: ctx.support_velocity,
            intent_direction: ctx.intent_direction,
            yaw: ctx.yaw,
            yaw_rate,
            hip_width: self.dims.hip_width,
            leg_length: self.dims.leg_length,
            standing_height: self.dims.standing_height,
            foot_y_fallback: ctx.pelvis.y - self.dims.standing_height,
            step_height: ctx.step_height,
            stride_gain: ctx.stride_gain,
            left_ground_normal: self.left_ground.normal_or_up(),
            right_ground_normal: self.right_ground.normal_or_up(),
            left_ground: self.left_ground.contact,
            right_ground: self.right_ground.contact,
            config: &self.config,
        };
        self.placer.tick(&placer_ctx);

        if let Some(recorder) = &mut self.recorder {
            recorder.record(ctx.airborne, ctx.pose_tag, &placer_ctx, &self.placer);
        }
    }

    /// Start recording placer input, or stop and write what was kept.
    ///
    /// Nothing happens without `PLACER_REC` set. Recording keeps a ring of the
    /// last few seconds, so it can be started well before the thing worth
    /// looking at and stopped just after it.
    pub fn toggle_recording(&mut self) {
        if let Some(recorder) = &mut self.recorder {
            if recorder.is_armed() {
                recorder.dump_and_report();
                recorder.set_armed(false);
            } else {
                recorder.set_armed(true);
                eprintln!("PlacerRecorder: recording");
            }
        }
    }

    /// What the recorder is doing and what it costs, for the debug overlay.
    pub fn recording_status(&self) -> Option<String> {
        self.recorder.as_ref().map(|recorder| recorder.status())
    }

    /// Whether this rig is currently keeping ticks. Every rig owns a
    /// recorder, so a caller that shares one line of screen between them
    /// needs to know which one is the live one.
    pub fn is_recording(&self) -> bool {
        self.recorder
            .as_ref()
            .is_some_and(|recorder| recorder.is_armed())
    }
}

/// Shortest signed angle difference `b - a`, wrapped into `(-PI, PI]`.
#[inline]
fn shortest_angle_diff(b: f32, a: f32) -> f32 {
    let two_pi = std::f32::consts::TAU;
    let mut d = (b - a) % two_pi;
    if d > std::f32::consts::PI {
        d -= two_pi;
    } else if d < -std::f32::consts::PI {
        d += two_pi;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn dims() -> LegRigDims {
        LegRigDims {
            hip_width: 0.12,
            leg_length: 0.5,
            standing_height: 0.425,
            probe_length: 0.75,
        }
    }

    /// The surface a body stands on is the frame its gait is measured in,
    /// and a contact set that chatters must not flick that frame back and
    /// forth at footfall rate.
    #[test]
    fn a_lost_surface_is_remembered_for_a_moment_and_then_forgotten() {
        let mut legs = LeggedLocomotion::new(
            dims(),
            FootPlacerConfig::default(),
            Point3::origin(),
            0.5,
            0.0,
        );
        let platform = Grounding::on(Vector3::y()).carried_by(Vector3::new(3.0, 0.0, 0.0));
        let dt = 1.0 / 60.0;

        assert_eq!(
            legs.observe_support(&platform, dt),
            Vector3::new(3.0, 0.0, 0.0)
        );
        assert_eq!(
            legs.observe_support(&Grounding::airborne(), dt),
            Vector3::new(3.0, 0.0, 0.0),
            "one frame between footfalls is not stepping off the platform"
        );

        for _ in 0..(SUPPORT_MEMORY / dt) as usize + 1 {
            legs.observe_support(&Grounding::airborne(), dt);
        }
        assert_eq!(
            legs.observe_support(&Grounding::airborne(), dt),
            Vector3::zeros(),
            "nothing is carrying a body that has been in the air this long"
        );
    }

    /// A body whose origin rides lower than the rig is tall has no offset to
    /// give: lifting the pelvis to suit would put the feet under the floor.
    #[test]
    fn a_body_that_rides_low_keeps_its_own_origin() {
        let dims = dims();
        let legs = LeggedLocomotion::new(
            dims,
            FootPlacerConfig::default(),
            Point3::origin(),
            dims.standing_height * 0.5,
            0.0,
        );
        assert_eq!(legs.pelvis_for(Point3::origin()), Point3::origin());
    }

    /// A probe that stops short of the point it was aimed at reports "no
    /// ground there", which the placer reads as an illegal landing target
    /// and answers by cancelling the stride.
    #[test]
    fn a_probe_always_outreaches_what_it_was_aimed_at() {
        let dims = dims();
        let legs = LeggedLocomotion::new(
            dims,
            FootPlacerConfig::default(),
            Point3::origin(),
            0.5,
            0.0,
        );
        let hip = Point3::new(0.0, 1.0, 0.0);
        let far = Point3::new(0.0, 1.0, 4.0);

        let probe = legs.foot_probe(0, hip, far);
        assert!(
            probe.length > 4.0,
            "a probe aimed 4m out must reach past it, got {}",
            probe.length
        );
    }
}
