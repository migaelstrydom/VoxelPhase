use generational_arena::Arena;
use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use super::event::Hit;
use super::patch::Patch;
use crate::physics::body::RigidBody;
use crate::physics::collider::Collider;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::{PairHeader, SolverContact, SolverManifold};

/// Default [`HitConfig::speed`], m/s.
pub const HIT_SPEED: f32 = 0.5;

/// When a contact is a hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitConfig {
    /// Closing speed, m/s, past which a contact that takes impulse is a hit.
    /// A resting contact closes at one substep of gravity (about 0.04 m/s),
    /// a waking body at about the same, a joint pushing a body into a
    /// contact at none.
    pub speed: f32,
}

impl Default for HitConfig {
    fn default() -> Self {
        Self { speed: HIT_SPEED }
    }
}

/// The bodies and colliders either side of a pair, as its header names them.
type PairKey = (
    Option<RigidBodyHandle>,
    Option<ColliderHandle>,
    RigidBodyHandle,
    Option<ColliderHandle>,
);

fn key_of(header: &PairHeader) -> PairKey {
    (
        header.body_a,
        header.collider_a,
        header.body_b,
        header.collider_b,
    )
}

/// The frame's hits, gathered from every solve: the main substeps' and CCD's.
#[derive(Debug, Default)]
pub(crate) struct HitRecorder {
    /// One per pair, the fastest of the frame's substeps.
    hits: Vec<Hit>,
    /// Where each pair's hit is in `hits`.
    index: FxHashMap<PairKey, usize>,
    /// This solve's fast pairs, kept for their capacity.
    pending: Vec<Pending>,
    /// Where each pending pair is in `pending`.
    pending_index: FxHashMap<PairKey, usize>,
}

/// A pair one solve found closing fast, while its patch is gathered.
#[derive(Debug)]
struct Pending {
    key: PairKey,
    approach: f32,
    normal: Vector3<f32>,
    restitution: f32,
    give: f32,
    /// Its contacts within its give of touching.
    points: Vec<Point3<f32>>,
}

impl HitRecorder {
    /// Drop the last frame's hits. Called once per frame, before any solve.
    pub(crate) fn clear(&mut self) {
        self.hits.clear();
        self.index.clear();
    }

    /// The frame's hits so far.
    pub(crate) fn hits(&self) -> &[Hit] {
        &self.hits
    }

    /// Fold one solve of `manifolds` into the frame's hits. Must run after
    /// the solver has written each contact's `approach_speed` and impulse;
    /// `substep_dt` is the solve's step, which turns a contact's closing
    /// allowance back into the gap it has left.
    pub(crate) fn record_solved(
        &mut self,
        config: &HitConfig,
        manifolds: &[SolverManifold],
        bodies: &Arena<RigidBody>,
        colliders: &Arena<Collider>,
        substep_dt: f32,
    ) {
        self.pending.clear();
        self.pending_index.clear();
        for manifold in manifolds {
            self.note_if_fast(config, manifold);
        }
        if self.pending.is_empty() {
            return;
        }
        // A pair's contacts can come in more than one manifold (a body over
        // several of the ground's triangles): its patch is all of them.
        for manifold in manifolds {
            let key = key_of(&manifold.header);
            let Some(&slot) = self.pending_index.get(&key) else {
                continue;
            };
            let pending = &mut self.pending[slot];
            let give = pending.give;
            pending.points.extend(
                manifold
                    .contacts
                    .iter()
                    .filter(|c| within_give(c, give, substep_dt))
                    .map(|c| c.point),
            );
        }
        let pending = std::mem::take(&mut self.pending);
        for pair in &pending {
            let hit = hit_of(pair, bodies, colliders);
            self.keep_fastest(pair.key, hit);
        }
        self.pending = pending;
    }

    /// Start gathering `manifold`'s pair if one of its contacts took impulse
    /// closing faster than `config.speed`.
    fn note_if_fast(&mut self, config: &HitConfig, manifold: &SolverManifold) {
        let Some(fastest) = manifold
            .contacts
            .iter()
            .filter(|c| c.accumulated_normal_impulse > 0.0)
            .max_by(|a, b| a.approach_speed.total_cmp(&b.approach_speed))
        else {
            return;
        };
        if fastest.approach_speed < config.speed {
            return;
        }
        let header = &manifold.header;
        let key = key_of(header);
        match self.pending_index.get(&key) {
            Some(&slot) => {
                let pending = &mut self.pending[slot];
                if fastest.approach_speed > pending.approach {
                    pending.approach = fastest.approach_speed;
                    pending.normal = fastest.normal;
                    pending.restitution = header.restitution;
                    pending.give = header.give;
                }
            }
            None => {
                self.pending_index.insert(key, self.pending.len());
                self.pending.push(Pending {
                    key,
                    approach: fastest.approach_speed,
                    normal: fastest.normal,
                    restitution: header.restitution,
                    give: header.give,
                    points: Vec::new(),
                });
            }
        }
    }

    /// Keep `hit` as its pair's, unless the pair already closed faster in an
    /// earlier solve this frame.
    fn keep_fastest(&mut self, key: PairKey, hit: Hit) {
        match self.index.get(&key) {
            Some(&slot) => {
                if hit.approach > self.hits[slot].approach {
                    self.hits[slot] = hit;
                }
            }
            None => {
                self.index.insert(key, self.hits.len());
                self.hits.push(hit);
            }
        }
    }
}

/// Whether `contact` is part of the patch: it took impulse, or the gap it has
/// left is within the pair's give.
fn within_give(contact: &SolverContact, give: f32, substep_dt: f32) -> bool {
    contact.accumulated_normal_impulse > 0.0 || contact.closing_allowance * substep_dt <= give
}

fn hit_of(pair: &Pending, bodies: &Arena<RigidBody>, colliders: &Arena<Collider>) -> Hit {
    let (body_a, collider_a, body_b, collider_b) = pair.key;
    let count = pair.points.len().max(1) as f32;
    let point = Point3::from(
        pair.points
            .iter()
            .fold(Vector3::zeros(), |sum, p| sum + p.coords)
            / count,
    );
    Hit {
        body_a,
        collider_a,
        body_b,
        collider_b,
        point,
        normal: pair.normal,
        approach: pair.approach,
        restitution: pair.restitution,
        give: pair.give,
        bluntness: side_bluntness(body_a, collider_a, bodies, colliders).min(side_bluntness(
            Some(body_b),
            collider_b,
            bodies,
            colliders,
        )),
        effective_mass: effective_mass(bodies, [body_a, Some(body_b)], &point, &pair.normal),
        patch: Patch::spanned(&pair.points, &pair.normal),
    }
}

/// The pair's mass along `normal` at `point`, kg: `1 / Σ (1/m + (r×n)·I⁻¹(r×n))`
/// over the sides that can move.
fn effective_mass(
    bodies: &Arena<RigidBody>,
    sides: [Option<RigidBodyHandle>; 2],
    point: &Point3<f32>,
    normal: &Vector3<f32>,
) -> f32 {
    let inverse: f32 = sides
        .into_iter()
        .flatten()
        .filter_map(|handle| bodies.get(handle.0))
        .filter(|body| body.is_dynamic())
        .map(|body| {
            let lever = (point - body.position()).cross(normal);
            body.inv_mass() + lever.dot(&(body.world_inv_inertia() * lever))
        })
        .sum();
    if inverse > 0.0 {
        1.0 / inverse
    } else {
        f32::INFINITY
    }
}

/// Bluntness of one side: its collider's, or where the contact kept no
/// collider, the sharpest of its body's. Static geometry has none.
fn side_bluntness(
    body: Option<RigidBodyHandle>,
    collider: Option<ColliderHandle>,
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
) -> f32 {
    if let Some(collider) = collider.and_then(|c| colliders.get(c.0)) {
        return collider.bluntness();
    }
    body.and_then(|b| bodies.get(b.0))
        .map_or(f32::INFINITY, |body| {
            body.colliders()
                .iter()
                .filter_map(|c| colliders.get(c.0))
                .map(Collider::bluntness)
                .fold(f32::INFINITY, f32::min)
        })
}
