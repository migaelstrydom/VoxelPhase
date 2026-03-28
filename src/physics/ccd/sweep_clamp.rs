/// Sweep-and-clamp CCD: sweep bounding sphere, clamp body to hit, solve contacts.
///
/// A body requires CCD when `|linear_velocity| * dt > radius * ccd_threshold`
/// AND the narrowphase did not already generate static contacts for it.
/// Bodies with narrowphase contacts are managed by the solver — CCD only
/// catches bodies in free flight that might skip past geometry entirely.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use smallvec::{smallvec, SmallVec};

use crate::collision::contact::FeatureId;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::filter_patch;
use crate::collision::obb::Obb;
use crate::physics::collider::{ColliderMaterial, ColliderShape};
use crate::physics::contact_event::{ContactEvent, ContactSource};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::{PairHeader, SolverContact, SolverManifold};
use crate::physics::solver::ccd::solve_contacts;
use crate::physics::static_geometry::StaticGeometry;

use super::strategy::{CcdContext, CcdStrategy};

pub struct SweepClampCcd;

impl SweepClampCcd {
    pub fn new() -> Self {
        Self
    }
}

impl Default for SweepClampCcd {
    fn default() -> Self {
        Self::new()
    }
}

impl CcdStrategy for SweepClampCcd {
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
                if ctx.narrowphase_handled.contains(&handle) {
                    return SmallVec::<[CcdCandidate; 4]>::new();
                }
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
                    if speed * dt <= radius * ctx.ccd_threshold {
                        continue;
                    }
                    let pre_center =
                        Point3::from(collider.world_transform(pre_pos, pre_rot).translation.vector);
                    let post_center = Point3::from(
                        collider.world_transform(post_pos, post_rot).translation.vector,
                    );
                    out.push(CcdCandidate {
                        body_handle: handle,
                        radius,
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
            let Some(hit) = sweep_sphere_against_static(
                candidate.pre_center,
                candidate.post_center,
                candidate.radius,
                static_geometry,
            ) else {
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
                friction: candidate.material.friction,
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
                    *half_extents,
                    hit_pos,
                    hit_rot,
                    static_geometry,
                    ctx.contact_margin,
                ),
                ColliderShape::Capsule { .. } => {
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

            corrections += 1;
        }

        corrections
    }
}

/// Data collected for a body that needs CCD sweeping.
struct CcdCandidate {
    body_handle: RigidBodyHandle,
    radius: f32,
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
    let filtered = filter_patch(&patch, 0.98);
    let manifold = obb_patch_manifold(&obb, &filtered, contact_margin);

    let mut contacts: SmallVec<[SolverContact; 4]> = manifold
        .points
        .into_iter()
        .map(|cp| cold_solver_contact(cp.point, cp.normal, cp.raw_normal, 0.0, 0.0, cp.feature_id))
        .collect();

    if contacts.is_empty() {
        if let Some(hit) = sweep_sphere_against_static(
            candidate.pre_center,
            candidate.post_center,
            candidate.radius,
            static_geometry,
        ) {
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

/// Sweep a sphere from `start` to `end` against static geometry.
///
/// Builds the enclosing AABB, queries the region, and returns the earliest
/// swept contact along the path.
fn sweep_sphere_against_static(
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
) -> Option<crate::collision::continuous::SweptContact> {
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
    let patch = static_geometry.query_region(&query);

    let mut earliest: Option<crate::collision::continuous::SweptContact> = None;
    for pt in &patch.triangles {
        if let Some(contact) =
            crate::collision::continuous::swept_sphere_triangle(start, end, radius, &pt.triangle)
        {
            if earliest.is_none() || contact.t < earliest.as_ref().unwrap().t {
                earliest = Some(contact);
            }
        }
    }
    earliest
}
