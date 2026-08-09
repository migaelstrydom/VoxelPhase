//! Sweep-and-clamp CCD: sweep each fast collider, clamp it to the earliest
//! impact, and solve the resulting contact.
//!
//! A collider is swept when it outruns contact generation — either across one
//! substep (`ccd_threshold`) or across the whole frame (`ccd_frame_coverage`,
//! since the narrowphase samples only once per frame) — AND the narrowphase
//! does not currently own the pair being swept. A pair with fresh narrowphase
//! contacts is the solver's to resolve; CCD only catches motion that would skip
//! past geometry entirely. Ownership is per pair, so a body resting on the
//! floor is still swept against everything else, and expires once the pair
//! outruns its frame-start manifold; see `NarrowphaseOwnership`.
//!
//! ```text
//!   collect_candidates ──▶ sweep_against_static ──▶ clamp ──▶ contacts ──▶ solve
//!      (policy)               (geometry)          (this file)
//! ```

use nalgebra::{Point3, Vector3};
use smallvec::{smallvec, SmallVec};

use crate::collision::contact::FeatureId;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::{filter_patch, COPLANAR_DOT};
use crate::collision::obb::Obb;
use crate::collision::AABB;
use crate::physics::collider::ColliderShape;
use crate::physics::contact_event::{ContactEvent, ContactSource};
use crate::physics::pipeline::pair::{SolverContact, SolverManifold};
use crate::physics::solver::ccd::solve_contacts;
use crate::physics::static_geometry::StaticGeometry;

use super::candidate::{collect_candidates_into, CcdCandidate};
use super::dynamic_sweep::DynamicSweep;
use super::patch_cache::SweptPatchCache;
use super::static_sweep::{sweep_against_static, sweep_sphere_against_static};
use super::strategy::{CcdContext, CcdStrategy};
use super::swept_impact::{cold_solver_contact, SweptImpact};

pub struct SweepClampCcd {
    /// Static-geometry queries reused across the substeps of a frame.
    patch_cache: SweptPatchCache,
    /// Body-vs-body sweeping, holding its own broadphase scratch.
    dynamic: DynamicSweep,
    /// Impacts found this substep, as `(candidate index, impact)`.
    impacts: Vec<(usize, SweptImpact)>,
    /// Candidates collected this substep. Held across substeps for its
    /// capacity only; the contents are rebuilt every time.
    candidates: Vec<CcdCandidate>,
}

impl SweepClampCcd {
    pub fn new() -> Self {
        Self {
            patch_cache: SweptPatchCache::new(),
            dynamic: DynamicSweep::new(),
            impacts: Vec::new(),
            candidates: Vec::new(),
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

        let mut candidates = std::mem::take(&mut self.candidates);
        collect_candidates_into(ctx, dt, &mut candidates);
        self.impacts.clear();
        for (index, candidate) in candidates.iter().enumerate() {
            if let Some(impact) =
                sweep_against_static(candidate, static_geometry, &mut self.patch_cache)
            {
                self.impacts.push((index, impact));
            }
        }
        self.dynamic
            .impacts_into(ctx, &candidates, &mut self.impacts);
        retain_earliest_per_candidate(&mut self.impacts, candidates.len());

        // Moved out so the resolve loop can borrow `self` mutably; handed
        // back at the end so its capacity survives to the next substep.
        let mut impacts = std::mem::take(&mut self.impacts);
        for (index, impact) in impacts.drain(..) {
            let candidate = &candidates[index];

            let clamped_pos = self.clamp_to_impact(ctx, candidate, &impact);
            let contacts = self.impact_contacts(
                candidate,
                &impact,
                clamped_pos,
                static_geometry,
                ctx.contact_margin,
            );

            let mut manifold = SolverManifold {
                header: impact.header,
                contacts,
            };
            for contact in &manifold.contacts {
                ctx.contact_events.push(ContactEvent::from_solver(
                    &manifold.header,
                    contact,
                    ContactSource::Ccd,
                ));
            }
            solve_contacts(
                ctx.bodies,
                std::slice::from_mut(&mut manifold),
                ctx.restitution_velocity_threshold,
            );
            ctx.impacts.record_solved(std::slice::from_ref(&manifold));

            corrections += 1;
        }
        self.impacts = impacts;
        self.candidates = candidates;

        corrections
    }
}

impl SweepClampCcd {
    /// Rewind the impacted body to where it was at the time of impact.
    ///
    /// Returns the position it was placed at, which the contact-building pass
    /// needs to pose the shape.
    fn clamp_to_impact(
        &self,
        ctx: &mut CcdContext<'_>,
        candidate: &CcdCandidate,
        impact: &SweptImpact,
    ) -> Point3<f32> {
        let mut position = candidate.body_position_at(impact.toi);

        // The terrain sweep falls back to the bounding sphere, which for a box
        // stops the candidate a sphere's radius from the surface rather than a
        // face's. Push the surplus back out along the normal so the box lands
        // on its own face instead of hovering. Body sweeps use the real shape
        // throughout, so there is no surplus to undo.
        if let (ColliderShape::Box { half_extents }, None) =
            (&candidate.shape, impact.header.body_a)
        {
            let obb = Obb::new(
                candidate.center_at(impact.toi),
                impact.clamped_rotation,
                *half_extents,
            );
            let support = obb.project_half_extent(&impact.normal);
            position -= impact.normal * (candidate.radius - support).max(0.0);
        }

        if let Some(body) = ctx.bodies.get_mut(impact.clamped.0) {
            body.set_position(position);
            body.set_rotation(impact.clamped_rotation);
        }
        position
    }

    /// Turn an impact into the contacts the solver will resolve.
    ///
    /// A single swept point is enough to stop a sphere, but a box clamped
    /// against terrain needs a full manifold or it pivots about the one point
    /// it was given.
    fn impact_contacts(
        &mut self,
        candidate: &CcdCandidate,
        impact: &SweptImpact,
        clamped_pos: Point3<f32>,
        static_geometry: &dyn StaticGeometry,
        contact_margin: f32,
    ) -> SmallVec<[SolverContact; 4]> {
        match (&candidate.shape, impact.header.body_a) {
            // Terrain only: the refinement below asks the static geometry for a
            // patch, which a body impact has no equivalent of.
            (ColliderShape::Box { half_extents }, None) => self.box_impact_contacts(
                candidate,
                impact,
                *half_extents,
                clamped_pos,
                static_geometry,
                contact_margin,
            ),
            _ => smallvec![single_point_contact(impact)],
        }
    }

    /// Generate precise box-terrain contacts at a CCD hit position.
    ///
    /// The bounding-sphere sweep found the approximate hit; now build an OBB
    /// where the box was clamped and run the mesh-aware manifold pipeline for
    /// accurate normals across every face in contact.
    fn box_impact_contacts(
        &mut self,
        candidate: &CcdCandidate,
        impact: &SweptImpact,
        half_extents: Vector3<f32>,
        clamped_pos: Point3<f32>,
        static_geometry: &dyn StaticGeometry,
        contact_margin: f32,
    ) -> SmallVec<[SolverContact; 4]> {
        let obb = Obb::new(clamped_pos, impact.clamped_rotation, half_extents);
        let (aabb_min, aabb_max) = obb.enclosing_aabb();
        let margin = Vector3::repeat(contact_margin);
        let patch = static_geometry.query_region(&AABB::new(aabb_min - margin, aabb_max + margin));
        let filtered = filter_patch(&patch, COPLANAR_DOT);
        let manifold = obb_patch_manifold(&obb, &filtered, contact_margin);

        let mut contacts: SmallVec<[SolverContact; 4]> = manifold
            .points
            .into_iter()
            .map(|cp| {
                cold_solver_contact(cp.point, cp.normal, cp.raw_normal, 0.0, 0.0, cp.feature_id)
            })
            .collect();

        // The clamp can leave the box just clear of the surface, where the
        // discrete manifold finds nothing. Fall back to the swept point so the
        // impact is still resolved rather than silently dropped.
        if contacts.is_empty() {
            if let Some(hit) =
                sweep_sphere_against_static(candidate, static_geometry, &mut self.patch_cache)
            {
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
}

/// The swept hit itself, as a solver contact.
fn single_point_contact(impact: &SweptImpact) -> SolverContact {
    cold_solver_contact(
        impact.point,
        impact.normal,
        impact.normal,
        0.0,
        0.0,
        FeatureId::SINGLE,
    )
}

/// Keep only each candidate's earliest impact, in time-of-impact order.
///
/// A candidate can find several impacts in one substep — the terrain sweep and
/// the body sweep are independent, and one sweep can meet two obstacles. But a
/// body can only be rewound to one point in time. Clamping it more than once
/// leaves it wherever the last impact happened to put it, which for any impact
/// but the earliest is somewhere it never reached, and applies an impulse for
/// a collision that never happened.
///
/// This does mean a candidate that genuinely meets two obstacles at once — the
/// inside of a corner — resolves against one of them per substep, and needs the
/// next substep for the other. Sorting earliest-first is what makes that
/// harmless: the obstacle it would have reached first is the one it stops on.
///
/// Reaching the multi-impact case at all is harder than it sounds, and no
/// scenario in the bench harness manages it. Two obstacles in a line cannot
/// both be hit, since reaching the second means already being inside the first,
/// and at a substep of 1/240 a sweep is short enough that anything else has to
/// be arranged very deliberately. This is a guard on an invariant — a body can
/// be rewound to one time, not several — rather than a repair of an observed
/// fault.
fn retain_earliest_per_candidate(impacts: &mut Vec<(usize, SweptImpact)>, candidate_count: usize) {
    if impacts.len() < 2 {
        return;
    }
    impacts.sort_by(|(_, a), (_, b)| a.toi.total_cmp(&b.toi));

    let mut resolved = vec![false; candidate_count];
    impacts.retain(|(index, _)| !std::mem::replace(&mut resolved[*index], true));
}

#[cfg(test)]
mod tests {
    use nalgebra::UnitQuaternion;

    use crate::physics::handle::RigidBodyHandle;
    use crate::physics::pipeline::pair::PairHeader;

    use super::*;

    fn body(idx: usize) -> RigidBodyHandle {
        RigidBodyHandle(generational_arena::Index::from_raw_parts(idx, 0))
    }

    fn impact(candidate: usize, toi: f32) -> (usize, SweptImpact) {
        (
            candidate,
            SweptImpact {
                header: PairHeader {
                    body_a: None,
                    body_b: body(candidate),
                    collider_a: None,
                    collider_b: None,
                    restitution: 0.0,
                    friction: 0.0,
                },
                toi,
                point: Point3::origin(),
                normal: Vector3::y(),
                clamped: body(candidate),
                clamped_rotation: UnitQuaternion::identity(),
            },
        )
    }

    fn survivors(mut impacts: Vec<(usize, SweptImpact)>, count: usize) -> Vec<(usize, f32)> {
        retain_earliest_per_candidate(&mut impacts, count);
        impacts.into_iter().map(|(i, im)| (i, im.toi)).collect()
    }

    #[test]
    fn a_candidate_keeps_only_its_earliest_impact() {
        let impacts = vec![impact(0, 0.8), impact(0, 0.2), impact(0, 0.5)];
        assert_eq!(survivors(impacts, 1), vec![(0, 0.2)]);
    }

    #[test]
    fn distinct_candidates_all_survive_in_time_order() {
        let impacts = vec![impact(1, 0.9), impact(0, 0.1), impact(2, 0.5)];
        assert_eq!(survivors(impacts, 3), vec![(0, 0.1), (2, 0.5), (1, 0.9)]);
    }

    #[test]
    fn a_lone_impact_is_left_alone() {
        assert_eq!(survivors(vec![impact(0, 0.4)], 1), vec![(0, 0.4)]);
    }
}
