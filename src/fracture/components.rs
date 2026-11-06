//! Fracture-related ECS components.

use nalgebra::Vector3;
use rustc_hash::FxHashMap;
use specs::{Component, VecStorage};

use super::contact_load::{ContactLoadTracker, Deadband};
use super::parts::{PartGraph, PartId};
use crate::physics::{BodyTraits, ColliderHandle, CollisionFilter, ConstraintHandle};

/// What body a piece split off a compound is, beyond where it is and how it
/// moves: the compound's policy for its pieces.
#[derive(Debug, Clone, Copy)]
pub struct PieceTraits {
    /// Linear velocity damping of a piece's body.
    pub linear_damping: f32,
    /// Angular velocity damping of a piece's body.
    pub angular_damping: f32,
    /// How much of gravity a piece feels.
    pub gravity_scale: f32,
    /// What a piece meets and which queries see it; `None` keeps the filter
    /// of the body it came off.
    pub filter: Option<CollisionFilter>,
}

impl Default for PieceTraits {
    /// A loose piece of an ordinary object: barely damped, falling as
    /// everything else does.
    fn default() -> Self {
        Self {
            linear_damping: 0.01,
            angular_damping: 0.005,
            gravity_scale: 1.0,
            filter: None,
        }
    }
}

impl PieceTraits {
    /// The body a piece is, coming off a body with `filter` that passes
    /// through static geometry if `ignores_static`.
    pub fn body(&self, filter: CollisionFilter, ignores_static: bool) -> BodyTraits {
        BodyTraits {
            linear_damping: self.linear_damping,
            angular_damping: self.angular_damping,
            gravity_scale: self.gravity_scale,
            filter: self.filter.unwrap_or(filter),
            ignores_static,
        }
    }
}

/// Marks a compound body as destructible: its parts, the bonds between them
/// and the ties holding it up, and the loads they are judged by.
///
/// A bond breaks when the load on what it would tear off (the side not
/// held by the world through a tie, else the lighter; for a bond in a
/// cycle, which tears nothing off on its own, its two parts where it
/// holds) exceeds what it can take,
/// where the load comes from two sources:
///
/// - a blast, read as its loading per area at the torn-off side's centre of
///   mass (a bond in a cycle: at its anchor) and judged against the bond's
///   `threshold`; and
/// - contact, read as the *spike* — the frame-over-frame increase in the
///   impulse the solver pushed through a collider of the torn-off side —
///   felt at the anchor over the bond's reach, and judged against the
///   bond's `toughness`, or this compound's default.
///
/// Contact has to be read as a spike rather than a level because a heavy object
/// standing still already pushes its own weight through its lowest parts
/// every frame, which for anything massive exceeds a sensible fracture
/// threshold outright. The spike is near zero for resting weight and for a
/// player leaning on the object, and enormous for a landing.
///
/// After breaking, the fracture system finds the groups of parts still
/// bonded, keeps one on this body and splits each other off whole.
///
/// Every record is keyed by part or collider handle, so a piece's share of
/// it is a filter.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct CompoundFracture {
    /// The parts, bonds and ties.
    pub graph: PartGraph,
    /// Per-collider contact load history, the baseline the impact spike is
    /// measured against.
    pub contact_load: ContactLoadTracker,
    /// Contact spike at a bond, in N·s, above which a bond that states no
    /// [`toughness`](super::Bond::toughness) of its own lets go. `None`: such
    /// a bond never breaks on contact.
    ///
    /// Never compared with a bond's blast `threshold`: the two loads are not
    /// the same quantity, and a bond authored to fail under an 8 N·s/m² blast
    /// may have to hold thousands of N·s of its own structure settling.
    pub default_toughness: Option<f32>,
    /// Impulse, in N·s, to hand each part as it comes away this frame. No
    /// entry for a part nobody has pushed.
    ///
    /// Set by whoever severs a part for a reason the solver did not see —
    /// a shard knocked out of a window carries the momentum of what hit it,
    /// and the hit itself was absorbed by the compound before the shard
    /// existed as a part. Consumed at the next split.
    pub kicks: FxHashMap<PartId, Vector3<f32>>,
    /// Constraints of ties given up by severed parts, removed at the next
    /// split: a part that comes away comes away from what held it too.
    pub untied: Vec<ConstraintHandle>,
    /// Whether bonds were removed by something other than this frame's load
    /// judgement, so the split step must run even though no bond broke under
    /// load. Cleared when the split runs.
    pub split_pending: bool,
    /// Whether split-off pieces are handed to the debris budget, which takes
    /// the small ones away once the break has been seen.
    ///
    /// Opt-in per compound: a window's shards are scenery the moment they
    /// land, while the planks of a bridge are still the bridge.
    pub sheds_debris: bool,
    /// Whether the compound is giving itself up: at the next split its ties'
    /// constraints are removed and every bond dropped, so every part becomes
    /// a piece and the heaviest keeps the body.
    pub released: bool,
    /// What body each piece split off this compound is.
    pub pieces: PieceTraits,
}

impl CompoundFracture {
    /// A destructible compound of `graph`'s parts.
    pub fn new(graph: PartGraph) -> Self {
        Self {
            graph,
            contact_load: ContactLoadTracker::default(),
            default_toughness: None,
            kicks: FxHashMap::default(),
            untied: Vec::new(),
            split_pending: false,
            sheds_debris: false,
            released: false,
            pieces: PieceTraits::default(),
        }
    }

    /// Split-off pieces count against the [`DebrisBudget`](super::DebrisBudget).
    pub fn shedding_debris(mut self) -> Self {
        self.sheds_debris = true;
        self
    }

    /// The contact spike, in N·s, that breaks one of this compound's bonds
    /// that states no toughness of its own.
    ///
    /// Measure it: run the object through the impacts it should and should not
    /// survive and read the spikes off, as the tests beside its spawnable do.
    /// Guessing gives either a structure that shatters when leaned on or one
    /// that cannot be broken by hand.
    pub fn breaking_on_impact_at(mut self, toughness: f32) -> Self {
        self.default_toughness = Some(toughness);
        self
    }

    /// Judge contact spikes against each collider's own weight rather than
    /// the whole body's.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.contact_load = std::mem::take(&mut self.contact_load).with_deadband(deadband);
        self
    }

    /// Split pieces off as `pieces` bodies.
    pub fn with_pieces(mut self, pieces: PieceTraits) -> Self {
        self.pieces = pieces;
        self
    }

    /// Cut every bond and tie that holds `part`, so it comes away at the
    /// next split, and give it `kick` on its way out.
    ///
    /// For a part that fails for a reason the bond judgement cannot see —
    /// it fatigued, or it sat under a hit that a finer structure absorbed —
    /// rather than for a load the solver put through it.
    pub fn sever(&mut self, part: PartId, kick: Vector3<f32>) {
        self.graph.cut_loose(part);
        self.untied.extend(self.graph.untie_part(part));
        *self.kicks.entry(part).or_insert_with(Vector3::zeros) += kick;
        self.split_pending = true;
    }

    /// Forget the tie of `constraint`, which something other than fracture
    /// has removed: the ground it held the compound to is gone.
    pub fn untie(&mut self, constraint: ConstraintHandle) {
        self.graph.untie(constraint);
    }

    /// Give up every tie and bond at the next split.
    pub fn release(&mut self) {
        self.released = true;
        self.split_pending = true;
    }

    /// The impulse a part leaves with.
    pub fn kick_of(&self, part: PartId) -> Vector3<f32> {
        self.kicks
            .get(&part)
            .copied()
            .unwrap_or_else(Vector3::zeros)
    }

    /// Keep only the parts whose colliders are all among `colliders`, and
    /// their records: what is left once pieces have gone.
    pub fn retain(&mut self, colliders: &[ColliderHandle]) {
        let parts: Vec<PartId> = self
            .graph
            .parts()
            .iter()
            .filter(|part| part.colliders.iter().all(|c| colliders.contains(c)))
            .map(|part| part.id)
            .collect();
        self.graph = self.graph.share(&parts);
        self.contact_load = self.contact_load.share(colliders);
        self.kicks.retain(|part, _| parts.contains(part));
    }

    /// The compound of a piece split off with `parts` (whose colliders are
    /// `colliders`): their graph, their contact baselines, the same rules.
    pub fn share(&self, parts: &[PartId], colliders: &[ColliderHandle]) -> Self {
        Self {
            graph: self.graph.share(parts),
            contact_load: self.contact_load.share(colliders),
            default_toughness: self.default_toughness,
            kicks: FxHashMap::default(),
            untied: Vec::new(),
            split_pending: false,
            sheds_debris: self.sheds_debris,
            released: false,
            pieces: self.pieces,
        }
    }
}
