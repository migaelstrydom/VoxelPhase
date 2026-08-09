//! Which colliders need sweeping this substep, and where they moved.
//!
//! A candidate is one collider's motion over one substep, packaged so the
//! sweeps do not need the body arena. Selecting them is a policy question —
//! which motion is fast enough to outrun contact generation, and which pairs
//! the solver already owns — kept separate from the geometry of sweeping.

use nalgebra::{Point3, UnitQuaternion};
use smallvec::SmallVec;

use crate::physics::collider::{ColliderMaterial, ColliderShape};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};

use super::ownership::ContactPairKey;
use super::strategy::CcdContext;

/// One collider's motion across a substep, and what it takes to sweep it.
pub(super) struct CcdCandidate {
    pub body_handle: RigidBodyHandle,
    /// Identifies this candidate's cached sweep region.
    pub collider_handle: ColliderHandle,
    /// Bounding-sphere radius of the shape, the scale every CCD gate is
    /// measured against.
    pub radius: f32,
    /// Minimum travel into a surface for a hit to count as tunnelling rather
    /// than a graze along a surface the solver already handles.
    pub min_approach: f32,
    pub shape: ColliderShape,
    pub material: ColliderMaterial,
    pub pre_body_pos: Point3<f32>,
    pub post_body_pos: Point3<f32>,
    pub pre_rot: UnitQuaternion<f32>,
    pub post_rot: UnitQuaternion<f32>,
    pub pre_center: Point3<f32>,
    pub post_center: Point3<f32>,
}

impl CcdCandidate {
    /// Where this collider's body sits at time `t` through the substep.
    pub fn body_position_at(&self, t: f32) -> Point3<f32> {
        self.pre_body_pos + (self.post_body_pos - self.pre_body_pos) * t
    }

    /// Where this collider's centre sits at time `t` through the substep.
    pub fn center_at(&self, t: f32) -> Point3<f32> {
        self.pre_center + (self.post_center - self.pre_center) * t
    }

    /// The body's orientation at time `t` through the substep.
    pub fn rotation_at(&self, t: f32) -> UnitQuaternion<f32> {
        self.pre_rot.slerp(&self.post_rot, t)
    }
}

/// Collect every collider whose motion this substep warrants a sweep.
///
/// Skips static and sleeping bodies, which have no motion to sweep, and any
/// collider whose contact with the terrain the narrowphase still owns —
/// sweeping those would resolve the same contact twice.
pub(super) fn collect_candidates_into(ctx: &CcdContext<'_>, dt: f32, out: &mut Vec<CcdCandidate>) {
    out.clear();
    out.extend(
        ctx.bodies
            .iter()
            .filter(|(idx, body)| {
                if body.is_static() {
                    return false;
                }
                if let Some(sleeping) = ctx.sleeping {
                    return !sleeping.contains(&RigidBodyHandle(*idx));
                }
                true
            })
            .flat_map(|(idx, body)| {
                let handle = RigidBodyHandle(idx);
                let speed = body.linear_velocity().magnitude();
                let &(pre_pos, pre_rot) = match ctx.pre_states.get(&idx) {
                    Some(s) => s,
                    None => return SmallVec::new(),
                };
                let post_pos = body.position();
                let post_rot = body.rotation();

                let mut found = SmallVec::<[CcdCandidate; 4]>::new();
                for collider_handle in body.colliders() {
                    let Some(collider) = ctx.colliders.get(collider_handle.0) else {
                        continue;
                    };
                    let radius = collider.shape().bounding_radius();
                    let ccd_travel = radius * ctx.ccd_threshold;
                    // Two independent ways to outrun contact generation: a single
                    // substep long enough to skip past geometry, or a whole frame
                    // of travel that the once-per-frame narrowphase cannot bracket.
                    let frame_travel = speed * dt * ctx.substeps_per_frame as f32;
                    let needs_ccd =
                        speed * dt > ccd_travel || frame_travel > radius * ctx.ccd_frame_coverage;
                    if !needs_ccd {
                        continue;
                    }
                    // The solver owns this collider's contact with the terrain only
                    // while its frame-start manifold still describes where the
                    // collider is. Released once it has drifted a CCD travel window
                    // away from that anchor. Ownership of any *other* pair this
                    // collider is in says nothing about the terrain.
                    let terrain_pair = ContactPairKey::against_static(*collider_handle);
                    if ctx
                        .narrowphase_ownership
                        .owns(terrain_pair, post_pos.coords, ccd_travel)
                    {
                        continue;
                    }
                    let pre_center = Point3::from(
                        collider
                            .world_transform(pre_pos, pre_rot)
                            .translation
                            .vector,
                    );
                    let post_center = Point3::from(
                        collider
                            .world_transform(post_pos, post_rot)
                            .translation
                            .vector,
                    );
                    found.push(CcdCandidate {
                        body_handle: handle,
                        collider_handle: *collider_handle,
                        radius,
                        min_approach: ctx.contact_margin,
                        shape: collider.shape().clone(),
                        material: *collider.material(),
                        pre_body_pos: pre_pos,
                        post_body_pos: post_pos,
                        pre_rot,
                        post_rot,
                        pre_center,
                        post_center,
                    });
                }
                found
            }),
    );
}
