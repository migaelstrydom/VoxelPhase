//! The Support Set: which contacts hold a body up, and along what axis.
//!
//! ```text
//!   solver manifolds ──┐
//!                      ├──► SupportResolver ──► SupportSets ──► SupportSet per body
//!   gravity direction ─┘                                         ├── contacts
//!                                                                └── mean_normal
//! ```
//!
//! One classification, computed once from the contacts that already exist, and
//! read by everything that needs to know what a body is standing on. Grounding
//! is the boolean projection of it (`super::super::grounding`); later stages
//! add the drive's own consumers.
//!
//! The axis is `PhysicsConfig::gravity_direction()`, never a literal world Y
//! and never a body-local axis. A world with no gravity has no floor, so
//! nothing is supported there — "which way is up" is not a question that
//! configuration can answer.

use nalgebra::{Point3, UnitVector3, Vector3};
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

/// Tuning for what counts as a floor.
#[derive(Clone, Copy, Debug)]
pub struct SupportConfig {
    /// Cosine of the widest angle from the anti-gravity axis at which a
    /// contact still counts as holding the body up. `0.5` is a 60° floor.
    pub min_support_cosine: f32,
    /// Added to a contact's depth before it is used as a weight, so that a
    /// body resting at zero depth still contributes a direction to the mean
    /// normal rather than dropping out of the average.
    pub depth_weight_bias: f32,
}

impl Default for SupportConfig {
    fn default() -> Self {
        Self {
            min_support_cosine: 0.5,
            depth_weight_bias: 0.001,
        }
    }
}

/// One contact that supports a body.
#[derive(Clone, Copy, Debug)]
pub struct SupportContact {
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Unit normal, oriented out of the support and into the supported body.
    ///
    /// The raw geometric normal, not the solver's smoothed one: smoothing
    /// exists to stabilise impulses across a faceted surface, and letting it
    /// feed the classification would allow a wall to average into a floor.
    pub normal: Vector3<f32>,
    /// The body on the other end, or `None` when the support is static
    /// geometry. This is where the reaction half of a drive impulse lands.
    pub partner: Option<RigidBodyHandle>,
}

/// The contacts holding one body up, and the axis they hold it along.
///
/// Carries no share, weight or fraction per contact deliberately: load
/// distribution across several supports is what the solver's per-contact
/// bound already does, and a second weighting term would double-count it.
#[derive(Clone, Debug, Default)]
pub struct SupportSet {
    contacts: SmallVec<[SupportContact; 4]>,
    mean_normal: Vector3<f32>,
}

impl SupportSet {
    /// The contacts holding this body up. Empty means airborne.
    pub fn contacts(&self) -> &[SupportContact] {
        &self.contacts
    }

    /// Depth-weighted mean of the support normals, normalised.
    ///
    /// The body's local up: the axis a jump leaves along, and the axis a
    /// tangent plane is built against. Zero when the set is empty.
    pub fn mean_normal(&self) -> Vector3<f32> {
        self.mean_normal
    }

    /// True when nothing holds this body up.
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }
}

/// Every body's Support Set for one step.
#[derive(Clone, Debug, Default)]
pub struct SupportSets {
    sets: FxHashMap<RigidBodyHandle, SupportSet>,
}

impl SupportSets {
    /// The Support Set for one body, or `None` if it had no contacts at all.
    pub fn get(&self, body: RigidBodyHandle) -> Option<&SupportSet> {
        self.sets.get(&body)
    }

    /// True when this body has at least one contact holding it up.
    pub fn is_supported(&self, body: RigidBodyHandle) -> bool {
        self.sets.get(&body).is_some_and(|set| !set.is_empty())
    }

    /// Every body that has at least one contact holding it up.
    pub fn supported_bodies(&self) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        self.sets
            .iter()
            .filter(|(_, set)| !set.is_empty())
            .map(|(handle, _)| *handle)
    }
}

/// Turns a step's manifolds into a Support Set per body.
#[derive(Clone, Copy, Debug, Default)]
pub struct SupportResolver {
    /// What this resolver calls a floor.
    config: SupportConfig,
}

impl SupportResolver {
    pub fn new(config: SupportConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &SupportConfig {
        &self.config
    }

    /// Classify every contact in `manifolds` against the world's up axis.
    ///
    /// `gravity_direction` is the world's *down*; `None` — a world with no
    /// gravity — yields no supports at all.
    pub fn resolve(
        &self,
        manifolds: &[SolverManifold],
        gravity_direction: Option<UnitVector3<f32>>,
    ) -> SupportSets {
        let Some(down) = gravity_direction else {
            return SupportSets::default();
        };
        let up = -down.into_inner();

        let mut builders: FxHashMap<RigidBodyHandle, SupportBuilder> = FxHashMap::default();
        for manifold in manifolds {
            let header = &manifold.header;
            for contact in &manifold.contacts {
                let normal = contact.raw_normal;
                if normal.magnitude_squared() <= 1e-8 {
                    continue;
                }
                let weight = (contact.depth + self.config.depth_weight_bias).max(0.0);

                // The normal points from A toward B, so B is pushed along it
                // and A along its opposite.
                builders.entry(header.body_b).or_default().add(
                    SupportContact {
                        point: contact.point,
                        normal,
                        partner: header.body_a,
                    },
                    weight,
                );
                if let Some(body_a) = header.body_a {
                    builders.entry(body_a).or_default().add(
                        SupportContact {
                            point: contact.point,
                            normal: -normal,
                            partner: Some(header.body_b),
                        },
                        weight,
                    );
                }
            }
        }

        SupportSets {
            sets: builders
                .into_iter()
                .map(|(handle, builder)| {
                    (handle, builder.finish(&up, self.config.min_support_cosine))
                })
                .collect(),
        }
    }
}

/// Per-body accumulation while the manifolds are being walked.
#[derive(Default)]
struct SupportBuilder {
    /// Every contact on this body, with the weight it carries into a mean.
    candidates: SmallVec<[(SupportContact, f32); 8]>,
    /// Weighted sum of every candidate normal, supporting or not.
    normal_sum: Vector3<f32>,
    /// Sum of the weights in `normal_sum`.
    weight_sum: f32,
}

impl SupportBuilder {
    fn add(&mut self, contact: SupportContact, weight: f32) {
        self.normal_sum += contact.normal * weight;
        self.weight_sum += weight;
        self.candidates.push((contact, weight));
    }

    /// Decide which candidates are supports.
    ///
    /// A contact whose normal stands within the floor cone supports on its own.
    /// Failing that the group is considered together, because a body wedged in
    /// a crevice is held up by two walls neither of which is a floor: if the
    /// mean of every normal stands within the cone, the whole group supports.
    fn finish(self, up: &Vector3<f32>, min_cosine: f32) -> SupportSet {
        let aligned = |contact: &SupportContact| contact.normal.dot(up) >= min_cosine;

        if self.candidates.iter().any(|(contact, _)| aligned(contact)) {
            let mut set = SupportSet::default();
            let mut sum = Vector3::zeros();
            let mut weight_sum = 0.0;
            for (contact, weight) in self.candidates.iter().filter(|(c, _)| aligned(c)) {
                sum += contact.normal * *weight;
                weight_sum += *weight;
                set.contacts.push(*contact);
            }
            // Every normal in this set already stands within the cone, so a
            // mean that cancels to nothing is degenerate rather than
            // meaningful; the axis that qualified them is the honest answer.
            set.mean_normal = normalised_mean(sum, weight_sum).unwrap_or(*up);
            return set;
        }

        match normalised_mean(self.normal_sum, self.weight_sum) {
            Some(wedge_normal) if wedge_normal.dot(up) >= min_cosine => SupportSet {
                contacts: self.candidates.iter().map(|(c, _)| *c).collect(),
                mean_normal: wedge_normal,
            },
            _ => SupportSet::default(),
        }
    }
}

/// Normalise a weighted sum of normals. `None` when the weights are empty or
/// the normals cancel out, neither of which names a direction.
fn normalised_mean(sum: Vector3<f32>, weight_sum: f32) -> Option<Vector3<f32>> {
    if weight_sum <= 1e-6 {
        return None;
    }
    Vector3::try_normalize(&(sum / weight_sum), 1e-6)
}

#[cfg(test)]
mod tests {
    use generational_arena::Index;
    use nalgebra::Point3;

    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::physics::pipeline::pair::{PairHeader, SolverContact};

    fn handle(raw: usize) -> RigidBodyHandle {
        RigidBodyHandle(Index::from_raw_parts(raw, 0))
    }

    fn down() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(Vector3::new(0.0, -1.0, 0.0)))
    }

    /// A manifold whose contacts all share one normal, pointing from `a` to `b`.
    fn manifold(
        a: Option<RigidBodyHandle>,
        b: RigidBodyHandle,
        normals: &[Vector3<f32>],
    ) -> SolverManifold {
        SolverManifold {
            header: PairHeader {
                body_a: a,
                body_b: b,
                collider_a: None,
                collider_b: None,
                restitution: 0.0,
                friction: 1.0,
            },
            contacts: normals
                .iter()
                .map(|normal| SolverContact {
                    point: Point3::origin(),
                    normal: *normal,
                    raw_normal: *normal,
                    depth: 0.01,
                    raw_depth: 0.01,
                    feature_id: FeatureId::from_face(0),
                    warm_normal_impulse: 0.0,
                    warm_friction_impulse_ws: Vector3::zeros(),
                    accumulated_normal_impulse: 0.0,
                    accumulated_friction_impulse_ws: Vector3::zeros(),
                })
                .collect(),
        }
    }

    fn resolve(manifolds: &[SolverManifold]) -> SupportSets {
        SupportResolver::default().resolve(manifolds, down())
    }

    #[test]
    fn a_floor_contact_supports_the_body_above_it() {
        let body = handle(1);
        let sets = resolve(&[manifold(None, body, &[Vector3::y()])]);
        let set = sets.get(body).expect("body had a contact");
        assert_eq!(set.contacts().len(), 1);
        assert_eq!(set.mean_normal(), Vector3::y());
        assert!(sets.is_supported(body));
    }

    #[test]
    fn a_wall_contact_supports_nothing() {
        let body = handle(1);
        let sets = resolve(&[manifold(None, body, &[Vector3::x()])]);
        assert!(sets.get(body).expect("body had a contact").is_empty());
        assert!(!sets.is_supported(body));
    }

    #[test]
    fn a_wall_does_not_dilute_the_floor_it_meets() {
        let body = handle(1);
        let sets = resolve(&[manifold(None, body, &[Vector3::y(), Vector3::x()])]);
        let set = sets.get(body).unwrap();
        assert_eq!(set.contacts().len(), 1);
        assert_eq!(set.mean_normal(), Vector3::y());
    }

    #[test]
    fn two_walls_forming_a_crevice_support_together() {
        let body = handle(1);
        let left = Vector3::new(0.8, 0.6, 0.0).normalize();
        let right = Vector3::new(-0.8, 0.6, 0.0).normalize();
        let sets = resolve(&[manifold(None, body, &[left, right])]);
        let set = sets.get(body).unwrap();
        assert_eq!(set.contacts().len(), 2, "neither wall is a floor alone");
        assert!((set.mean_normal() - Vector3::y()).magnitude() < 1e-5);
    }

    #[test]
    fn two_opposing_walls_are_a_squeeze_not_a_floor() {
        let body = handle(1);
        let sets = resolve(&[manifold(None, body, &[Vector3::x(), -Vector3::x()])]);
        assert!(sets.get(body).unwrap().is_empty());
    }

    #[test]
    fn the_upper_body_of_a_pair_is_supported_from_either_side_of_the_manifold() {
        let lower = handle(1);
        let upper = handle(2);

        // The normal points a → b, so the same stack read both ways must give
        // the same answer: whichever body the normal points at is held up.
        let a_is_lower = resolve(&[manifold(Some(lower), upper, &[Vector3::y()])]);
        assert!(a_is_lower.is_supported(upper));
        assert!(!a_is_lower.is_supported(lower));

        let a_is_upper = resolve(&[manifold(Some(upper), lower, &[-Vector3::y()])]);
        assert!(a_is_upper.is_supported(upper));
        assert!(!a_is_upper.is_supported(lower));
    }

    #[test]
    fn the_partner_is_what_the_reaction_would_push_against() {
        let lower = handle(1);
        let upper = handle(2);
        let sets = resolve(&[manifold(Some(lower), upper, &[Vector3::y()])]);
        assert_eq!(sets.get(upper).unwrap().contacts()[0].partner, Some(lower));

        let on_terrain = resolve(&[manifold(None, upper, &[Vector3::y()])]);
        assert_eq!(on_terrain.get(upper).unwrap().contacts()[0].partner, None);
    }

    #[test]
    fn support_is_measured_against_gravity_not_against_world_up() {
        let body = handle(1);
        let sideways = Some(UnitVector3::new_normalize(Vector3::new(-1.0, 0.0, 0.0)));
        let resolver = SupportResolver::default();
        let manifolds = [manifold(None, body, &[Vector3::x()])];

        assert!(resolver.resolve(&manifolds, sideways).is_supported(body));
        assert!(!resolver.resolve(&manifolds, down()).is_supported(body));
    }

    #[test]
    fn a_world_with_no_gravity_has_no_floor() {
        let body = handle(1);
        let sets =
            SupportResolver::default().resolve(&[manifold(None, body, &[Vector3::y()])], None);
        assert!(sets.get(body).is_none());
        assert!(!sets.is_supported(body));
    }

    #[test]
    fn a_degenerate_normal_is_not_a_contact() {
        let body = handle(1);
        let sets = resolve(&[manifold(None, body, &[Vector3::zeros()])]);
        assert!(sets.get(body).is_none());
    }
}
