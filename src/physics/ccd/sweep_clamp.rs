/// Sweep-and-clamp CCD: sweep bounding sphere, clamp body to hit, solve contacts.
///
/// A body requires CCD when it outruns contact generation — either across one
/// substep (`ccd_threshold`) or across the whole frame (`ccd_frame_coverage`,
/// since the narrowphase samples only once per frame) — AND the narrowphase
/// does not currently own it. Bodies with fresh
/// narrowphase contacts are managed by the solver — CCD only catches bodies in
/// free flight that might skip past geometry entirely. That ownership expires
/// once the body outruns its frame-start manifold; see `NarrowphaseOwnership`.
use nalgebra::{Point3, UnitQuaternion, Vector3};
use smallvec::{smallvec, SmallVec};

use crate::collision::contact::FeatureId;
use crate::collision::continuous::gjk_raycast;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::{filter_patch, COPLANAR_DOT};
use crate::collision::obb::Obb;
use crate::collision::shape_view::ShapeView;
use crate::physics::collider::{ColliderMaterial, ColliderShape};
use crate::physics::contact_event::{ContactEvent, ContactSource};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::{PairHeader, SolverContact, SolverManifold};
use crate::physics::solver::ccd::solve_contacts;
use crate::physics::static_geometry::StaticGeometry;

use super::patch_cache::SweptPatchCache;
use super::strategy::{CcdContext, CcdStrategy};

pub struct SweepClampCcd {
    /// Static-geometry queries reused across the substeps of a frame.
    patch_cache: SweptPatchCache,
}

impl SweepClampCcd {
    pub fn new() -> Self {
        Self {
            patch_cache: SweptPatchCache::new(),
        }
    }
}

impl Default for SweepClampCcd {
    fn default() -> Self {
        Self::new()
    }
}

impl CcdStrategy for SweepClampCcd {
    fn begin_frame(&mut self, substeps: u32) {
        self.patch_cache.begin_frame(substeps);
    }

    fn run(
        &mut self,
        ctx: &mut CcdContext<'_>,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
    ) -> u32 {
        let mut corrections = 0u32;

        let candidates: Vec<CcdCandidate> = ctx
            .bodies
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

                let mut out = SmallVec::<[CcdCandidate; 4]>::new();
                for collider_handle in body.colliders() {
                    let Some(collider) = ctx.colliders.get(collider_handle.0) else {
                        continue;
                    };
                    let radius = collider.shape().bounding_radius();
                    let ccd_travel = radius * ctx.ccd_threshold;
                    // Two independent ways to outrun contact generation: a
                    // single substep long enough to skip past geometry, or a
                    // whole frame of travel that the once-per-frame
                    // narrowphase cannot bracket.
                    let frame_travel = speed * dt * ctx.substeps_per_frame as f32;
                    let needs_ccd =
                        speed * dt > ccd_travel || frame_travel > radius * ctx.ccd_frame_coverage;
                    if !needs_ccd {
                        continue;
                    }
                    // The solver owns this body only while its frame-start
                    // manifold still describes where the body is. Released once
                    // it has drifted a CCD travel window away from that anchor.
                    if ctx.narrowphase_ownership.owns(handle, post_pos, ccd_travel) {
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
                    out.push(CcdCandidate {
                        body_handle: handle,
                        collider_handle: *collider_handle,
                        radius,
                        min_approach: ctx.contact_margin,
                        shape: collider.shape().clone(),
                        material: *collider.material(),
                        pre_body_pos: pre_pos,
                        post_body_pos: post_pos,
                        pre_rot,
                        pre_center,
                        post_center,
                    });
                }
                out
            })
            .collect();

        for candidate in &candidates {
            let hit = match &candidate.shape {
                ColliderShape::Sphere { .. } => {
                    sweep_sphere_against_static(candidate, static_geometry, &mut self.patch_cache)
                }
                _ => {
                    let shape_hit = sweep_shape_against_static(
                        candidate,
                        static_geometry,
                        &mut self.patch_cache,
                    );
                    match shape_hit {
                        Some(h) => Some(h),
                        None => sweep_sphere_against_static(
                            candidate,
                            static_geometry,
                            &mut self.patch_cache,
                        ),
                    }
                }
            };
            let Some(hit) = hit else {
                continue;
            };

            let mut hit_pos =
                candidate.pre_body_pos + (candidate.post_body_pos - candidate.pre_body_pos) * hit.t;
            let mut hit_rot = candidate.pre_rot;
            if let Some(body) = ctx.bodies.get_mut(candidate.body_handle.0) {
                let post_rot = body.rotation();
                hit_rot = candidate.pre_rot.slerp(&post_rot, hit.t);
                if let ColliderShape::Box { half_extents } = &candidate.shape {
                    let hit_center = candidate.pre_center
                        + (candidate.post_center - candidate.pre_center) * hit.t;
                    let obb = Obb::new(hit_center, hit_rot, *half_extents);
                    let support = obb.project_half_extent(&hit.normal);
                    let extra = (candidate.radius - support).max(0.0);
                    hit_pos -= hit.normal * extra;
                }
                body.set_position(hit_pos);
                body.set_rotation(hit_rot);
            }

            let ccd_header = PairHeader {
                body_a: None,
                body_b: candidate.body_handle,
                collider_a: None,
                collider_b: None,
                restitution: candidate.material.restitution,
                friction: candidate.material.friction_at(&hit.normal, &hit_rot),
            };

            let ccd_contacts: SmallVec<[SolverContact; 4]> = match &candidate.shape {
                ColliderShape::Sphere { .. } => {
                    smallvec![cold_solver_contact(
                        hit.point,
                        hit.normal,
                        hit.normal,
                        0.0,
                        0.0,
                        FeatureId::SINGLE,
                    )]
                }
                ColliderShape::Box { half_extents } => box_ccd_solver_contacts(
                    candidate,
                    &mut self.patch_cache,
                    *half_extents,
                    hit_pos,
                    hit_rot,
                    static_geometry,
                    ctx.contact_margin,
                ),
                ColliderShape::Capsule { .. } | ColliderShape::ConvexHull { .. } => {
                    smallvec![cold_solver_contact(
                        hit.point,
                        hit.normal,
                        hit.normal,
                        0.0,
                        0.0,
                        FeatureId::SINGLE,
                    )]
                }
            };

            let mut ccd_manifold = SolverManifold {
                header: ccd_header,
                contacts: ccd_contacts,
            };

            for contact in &ccd_manifold.contacts {
                ctx.contact_events.push(ContactEvent::from_solver(
                    &ccd_manifold.header,
                    contact,
                    ContactSource::Ccd,
                ));
            }
            solve_contacts(
                ctx.bodies,
                std::slice::from_mut(&mut ccd_manifold),
                ctx.restitution_velocity_threshold,
            );
            ctx.impacts
                .record_solved(std::slice::from_ref(&ccd_manifold));

            corrections += 1;
        }

        corrections
    }
}

/// Data collected for a body that needs CCD sweeping.
struct CcdCandidate {
    body_handle: RigidBodyHandle,
    /// Identifies this candidate's cached sweep region.
    collider_handle: ColliderHandle,
    radius: f32,
    /// Minimum travel into a surface for a hit to count as tunnelling rather
    /// than a graze along a surface the solver already handles.
    min_approach: f32,
    shape: ColliderShape,
    material: ColliderMaterial,
    pre_body_pos: Point3<f32>,
    post_body_pos: Point3<f32>,
    pre_rot: UnitQuaternion<f32>,
    pre_center: Point3<f32>,
    post_center: Point3<f32>,
}

/// Generate precise box-terrain contacts at a CCD hit position.
///
/// Bounding-sphere sweep found the approximate hit. Now build an OBB at
/// the hit position (using the pre-integration rotation) and run the
/// mesh-aware manifold pipeline for accurate contact normals.
fn box_ccd_solver_contacts(
    candidate: &CcdCandidate,
    patch_cache: &mut SweptPatchCache,
    half_extents: Vector3<f32>,
    hit_pos: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> SmallVec<[SolverContact; 4]> {
    let obb = Obb::new(hit_pos, rotation, half_extents);
    let (aabb_min, aabb_max) = obb.enclosing_aabb();
    let margin = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = crate::collision::AABB::new(aabb_min - margin, aabb_max + margin);
    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, COPLANAR_DOT);
    let manifold = obb_patch_manifold(&obb, &filtered, contact_margin);

    let mut contacts: SmallVec<[SolverContact; 4]> = manifold
        .points
        .into_iter()
        .map(|cp| cold_solver_contact(cp.point, cp.normal, cp.raw_normal, 0.0, 0.0, cp.feature_id))
        .collect();

    if contacts.is_empty() {
        if let Some(hit) = sweep_sphere_against_static(candidate, static_geometry, patch_cache) {
            contacts.push(cold_solver_contact(
                hit.point,
                hit.normal,
                hit.normal,
                0.0,
                0.0,
                FeatureId::SINGLE,
            ));
        }
    }

    contacts
}

/// Build a cold (no warm-start) `SolverContact` for transient CCD contacts.
fn cold_solver_contact(
    point: Point3<f32>,
    normal: Vector3<f32>,
    raw_normal: Vector3<f32>,
    depth: f32,
    raw_depth: f32,
    feature_id: FeatureId,
) -> SolverContact {
    SolverContact {
        point,
        normal,
        raw_normal,
        depth,
        raw_depth,
        feature_id,
        warm_normal_impulse: 0.0,
        warm_friction_impulse_ws: Vector3::zeros(),
        accumulated_normal_impulse: 0.0,
        accumulated_friction_impulse_ws: Vector3::zeros(),
    }
}

/// Whether a swept hit is a genuine tunnelling threat rather than a graze.
///
/// A body travelling along a surface it is already touching reports a hit at
/// `t≈0` with a normal it is barely moving into. Clamping to such a hit would
/// teleport the body back to its substep-start position and re-solve friction
/// the solver already owns. Only hits the body drives into far enough to pass
/// through during this substep count — the same travel window that activates
/// CCD in the first place, measured along the hit normal.
fn is_tunnelling_hit(
    displacement: &Vector3<f32>,
    normal: &Vector3<f32>,
    min_approach: f32,
) -> bool {
    -displacement.dot(normal) > min_approach
}

/// Sweep a convex shape against static geometry using GJK raycast.
///
/// Uses the actual shape (not bounding sphere) for a tighter TOI estimate.
/// Returns a `SweptContact` compatible with the existing pipeline.
fn sweep_shape_against_static(
    candidate: &CcdCandidate,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &mut SweptPatchCache,
) -> Option<crate::collision::continuous::SweptContact> {
    let start = candidate.pre_center;
    let end = candidate.post_center;
    let radius = candidate.radius;
    let min_approach = candidate.min_approach;

    let query = crate::collision::AABB::new(
        Point3::new(
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
        ),
        Point3::new(
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        ),
    );
    let displacement = end - start;
    let triangles = patch_cache.triangles(
        candidate.collider_handle,
        &query,
        &displacement,
        static_geometry,
    );

    let shape_view = ShapeView {
        center: start,
        rotation: candidate.pre_rot,
        shape: &candidate.shape,
    };

    let mut earliest: Option<crate::collision::continuous::SweptContact> = None;

    for triangle in triangles {
        let hit = gjk_raycast(&shape_view, triangle, displacement, Vector3::zeros());
        if let Some(h) = hit {
            if !is_tunnelling_hit(&displacement, &h.normal, min_approach) {
                continue;
            }
            if earliest.is_none() || h.t < earliest.as_ref().unwrap().t {
                earliest = Some(crate::collision::continuous::SweptContact::new(
                    h.t, h.point, h.normal,
                ));
            }
        }
    }

    earliest
}

/// Sweep a sphere from `start` to `end` against static geometry.
///
/// Builds the enclosing AABB, queries the region, and returns the earliest
/// swept contact along the path.
fn sweep_sphere_against_static(
    candidate: &CcdCandidate,
    static_geometry: &dyn StaticGeometry,
    patch_cache: &mut SweptPatchCache,
) -> Option<crate::collision::continuous::SweptContact> {
    let start = candidate.pre_center;
    let end = candidate.post_center;
    let radius = candidate.radius;
    let min_approach = candidate.min_approach;
    let query = crate::collision::AABB::new(
        Point3::new(
            start.x.min(end.x) - radius,
            start.y.min(end.y) - radius,
            start.z.min(end.z) - radius,
        ),
        Point3::new(
            start.x.max(end.x) + radius,
            start.y.max(end.y) + radius,
            start.z.max(end.z) + radius,
        ),
    );
    let displacement = end - start;
    let triangles = patch_cache.triangles(
        candidate.collider_handle,
        &query,
        &displacement,
        static_geometry,
    );

    let mut earliest: Option<crate::collision::continuous::SweptContact> = None;
    for triangle in triangles {
        if let Some(contact) =
            crate::collision::continuous::swept_sphere_triangle(start, end, radius, triangle)
        {
            if !is_tunnelling_hit(&displacement, &contact.normal, min_approach) {
                continue;
            }
            if earliest.is_none() || contact.t < earliest.as_ref().unwrap().t {
                earliest = Some(contact);
            }
        }
    }
    earliest
}
