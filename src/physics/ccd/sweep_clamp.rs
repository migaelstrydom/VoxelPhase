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

use super::candidate::{collect_candidates, CcdCandidate};
use super::patch_cache::SweptPatchCache;
use super::static_sweep::{sweep_against_static, sweep_sphere_against_static};
use super::strategy::{CcdContext, CcdStrategy};
use super::swept_impact::{cold_solver_contact, SweptImpact};

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

        for candidate in &collect_candidates(ctx, dt) {
            let Some(impact) =
                sweep_against_static(candidate, static_geometry, &mut self.patch_cache)
            else {
                continue;
            };

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

        // The sweep used the bounding sphere, which for a box stops the
        // candidate a sphere's radius from the surface rather than a face's.
        // Push the surplus back out along the normal so the box lands on its
        // own face instead of hovering.
        if let ColliderShape::Box { half_extents } = &candidate.shape {
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
        match &candidate.shape {
            ColliderShape::Box { half_extents } => self.box_impact_contacts(
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
