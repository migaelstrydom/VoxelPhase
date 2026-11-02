//! Sweeping fast colliders against other bodies.
//!
//! The static sweep has no partner to move: terrain is a triangle soup and the
//! candidate's own displacement is the relative motion. Here both sides move,
//! so every question is asked in relative terms — the pair tunnels at its
//! closing speed, not at either side's ground speed.
//!
//! Only the swept side is clamped. Its partner keeps the position integration
//! gave it, and the solver takes over from there. Rewinding both would need a
//! global time-of-impact ordering across every pair at once; rewinding one is
//! enough to stop the pass-through, which is what CCD is for.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use rustc_hash::FxHashMap;

use crate::collision::continuous::gjk_raycast;
use crate::collision::shape_view::ShapeView;
use crate::collision::AABB;
use crate::physics::broadphase::SweepAndPrune;
use crate::physics::collider::{ColliderMaterial, ColliderShape};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::PairHeader;

use super::candidate::CcdCandidate;
use super::ownership::{pair_separation, ContactPairKey};
use super::static_sweep::is_tunnelling_hit;
use super::strategy::CcdContext;
use super::swept_impact::SweptImpact;

/// One collider's motion across the substep, as a sweep can see it.
///
/// Every collider in the world becomes an entry, whether or not it is fast
/// enough to need sweeping — a stationary wall is a perfectly good thing to
/// tunnel through. `candidate` marks the ones that are being swept.
struct SweepEntry {
    body: RigidBodyHandle,
    collider: ColliderHandle,
    shape: ColliderShape,
    material: ColliderMaterial,
    pre_center: Point3<f32>,
    post_center: Point3<f32>,
    pre_rot: UnitQuaternion<f32>,
    /// Index into the candidate list, when this collider is one of them.
    candidate: Option<usize>,
}

impl SweepEntry {
    fn displacement(&self) -> Vector3<f32> {
        self.post_center - self.pre_center
    }

    /// Bounds covering the whole path this collider takes over the substep.
    fn swept_bounds(&self, margin: f32) -> AABB {
        let at = |center| {
            ShapeView {
                center,
                rotation: self.pre_rot,
                shape: &self.shape,
            }
            .query_aabb(margin)
        };
        at(self.pre_center).merged(&at(self.post_center))
    }

    fn view_at(&self, center: Point3<f32>) -> ShapeView<'_> {
        ShapeView {
            center,
            rotation: self.pre_rot,
            shape: &self.shape,
        }
    }
}

/// Reusable scratch for the body-vs-body sweep.
#[derive(Default)]
pub(super) struct DynamicSweep {
    entries: Vec<SweepEntry>,
    bounds: Vec<AABB>,
    pairs: Vec<(usize, usize)>,
    broadphase: SweepAndPrune,
    /// Candidate index by collider, so marking entries is a lookup rather than
    /// a scan of the candidate list per collider in the world.
    candidate_index: FxHashMap<ColliderHandle, usize>,
}

impl DynamicSweep {
    pub fn new() -> Self {
        Self::default()
    }

    /// Find every impact between a swept candidate and another body.
    ///
    /// Appends `(candidate index, impact)` for each, leaving resolution to the
    /// caller. The candidate index identifies the side that will be clamped.
    pub fn impacts_into(
        &mut self,
        ctx: &CcdContext<'_>,
        candidates: &[CcdCandidate],
        out: &mut Vec<(usize, SweptImpact)>,
    ) {
        self.entries.clear();
        self.bounds.clear();
        self.pairs.clear();
        self.candidate_index.clear();
        if candidates.is_empty() {
            return;
        }
        for (index, candidate) in candidates.iter().enumerate() {
            self.candidate_index
                .insert(candidate.collider_handle, index);
        }

        self.collect_entries(ctx);
        if self.entries.len() < 2 {
            return;
        }

        let margin = ctx.contact_margin;
        self.bounds
            .extend(self.entries.iter().map(|e| e.swept_bounds(margin)));

        // "Mobile" here means "being swept": a pair of colliders where neither
        // is a candidate is nobody's tunnelling risk, however fast they move.
        let entries = &self.entries;
        self.broadphase.pairs_into(
            &self.bounds,
            |i| entries[i].candidate.is_some(),
            &mut self.pairs,
        );

        for &(i, j) in &self.pairs {
            let (a, b) = (&self.entries[i], &self.entries[j]);
            if a.body == b.body {
                continue;
            }
            // Sweep the faster side and clamp it. Time of impact is the same
            // whichever side is treated as moving, since only relative motion
            // enters the raycast, but the body that would have passed through
            // is the one worth rewinding.
            let (swept, target) = if a.candidate.is_some()
                && (b.candidate.is_none()
                    || a.displacement().magnitude_squared() >= b.displacement().magnitude_squared())
            {
                (a, b)
            } else {
                (b, a)
            };
            let Some(candidate_idx) = swept.candidate else {
                continue;
            };
            let candidate = &candidates[candidate_idx];

            if self.narrowphase_owns(ctx, swept, target, candidate.radius) {
                continue;
            }

            if let Some(impact) = sweep_pair(candidate, swept, target) {
                out.push((candidate_idx, impact));
            }
        }
    }

    /// Whether the solver already holds this pair's contact.
    fn narrowphase_owns(
        &self,
        ctx: &CcdContext<'_>,
        swept: &SweepEntry,
        target: &SweepEntry,
        radius: f32,
    ) -> bool {
        let key = ContactPairKey::new(swept.collider, target.collider);
        let Some(separation) = pair_separation(ctx.bodies, Some(target.body), swept.body) else {
            return false;
        };
        ctx.narrowphase_ownership
            .owns(key, separation, radius * ctx.ccd_threshold)
    }

    /// Snapshot every collider in the world, marking the swept ones.
    fn collect_entries(&mut self, ctx: &CcdContext<'_>) {
        for (idx, body) in ctx.bodies.iter() {
            let handle = RigidBodyHandle(idx);
            // A body absent from `pre_states` was not integrated this substep,
            // so where it is now is where it started.
            let (pre_pos, pre_rot) = ctx
                .pre_states
                .get(&idx)
                .copied()
                .unwrap_or((body.position(), body.rotation()));
            let post_pos = body.position();
            let post_rot = body.rotation();

            for collider_handle in body.colliders() {
                let Some(collider) = ctx.colliders.get(collider_handle.0) else {
                    continue;
                };
                let center_at =
                    |pos, rot| Point3::from(collider.world_transform(pos, rot).translation.vector);
                self.entries.push(SweepEntry {
                    body: handle,
                    collider: *collider_handle,
                    shape: collider.shape().clone(),
                    material: *collider.material(),
                    pre_center: center_at(pre_pos, pre_rot),
                    post_center: center_at(post_pos, post_rot),
                    pre_rot,
                    candidate: self.candidate_index.get(collider_handle).copied(),
                });
            }
        }
    }
}

/// Sweep one collider against another and report their impact, if any.
///
/// The raycast freezes the target at its substep-start pose and gives the swept
/// side the pair's *relative* displacement, so the time of impact is the one
/// the pair actually experiences.
///
/// # Rotation is not swept
///
/// Both shapes hold their substep-start orientation for the whole sweep. Only
/// translation is continuous; `gjk_raycast` marches a ray through configuration
/// space, and a rotating support function does not give it one. The swept
/// side's rotation is interpolated afterwards, to place it at the clamp, but it
/// took no part in finding the time of impact.
///
/// This is exact for the case CCD exists to catch — something small and fast
/// travelling in roughly a straight line — because over one substep a
/// projectile's orientation barely changes and its own spin cannot move its
/// surface far compared to its translation. It degrades for a long shape
/// whipping about its own axis: the tip of a swinging bat covers ground by
/// rotating, and a sweep that sees only the bat's centre of mass moving will
/// miss a ball the tip would have struck. Conservative advancement against a
/// motion bound that includes angular velocity is the standard remedy, and is
/// what to reach for if a fast rotating body starts passing through things.
fn sweep_pair(
    candidate: &CcdCandidate,
    swept: &SweepEntry,
    target: &SweepEntry,
) -> Option<SweptImpact> {
    let relative = swept.displacement() - target.displacement();
    let hit = gjk_raycast(
        &swept.view_at(swept.pre_center),
        &target.view_at(target.pre_center),
        relative,
        Vector3::zeros(),
    )?;

    if !is_tunnelling_hit(&relative, &hit.normal, candidate.min_approach) {
        return None;
    }

    let rotation = candidate.rotation_at(hit.t);
    // `gjk_raycast` reports its normal pointing from B toward A — here, from
    // the target toward the swept side. The solver wants A→B, so the target
    // takes the A slot. There is no appeal to `shape_type_rank`: that exists to
    // match the canonical shape ordering the *discrete* dispatch imposes on its
    // normals, and this normal did not come from the dispatch.
    let (restitution, friction) = ColliderMaterial::combine(&target.material, &swept.material);

    Some(SweptImpact {
        header: PairHeader {
            body_a: Some(target.body),
            body_b: swept.body,
            collider_a: None,
            // Left empty deliberately: it is what keeps this transient contact
            // out of the manifold cache. See `cold_solver_contact`.
            collider_b: None,
            restitution,
            friction,
        },
        toi: hit.t,
        point: hit.point,
        normal: hit.normal,
        clamped: swept.body,
        clamped_rotation: rotation,
    })
}
