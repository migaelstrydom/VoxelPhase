//! The component that makes a compound body a thing of brittle blocks.

use specs::{Component, VecStorage};

use super::plan::CleaveRule;
use crate::fracture::{BreakDepths, Cuttable};
use crate::physics::ColliderHandle;

/// A compound body whose children break into wedges where they are struck.
///
/// Sits beside a [`CompoundFracture`](crate::fracture::CompoundFracture) as
/// [`BrittleSheet`](crate::glass::BrittleSheet) does: this component decides
/// how a part *divides*, that one decides when the divisions *let go*. A
/// block that is hit hard enough is replaced by the two or three wedges it
/// cleaved into, bonded to each other, each of the block's bonds passed to
/// the wedge nearest it, and the fracture system takes it from there.
///
/// An object needs no special build to carry one: an igloo is already a
/// compound of ice blocks with a bond per touching pair, and a single block
/// is a compound of one.
#[derive(Component)]
#[storage(VecStorage)]
pub struct BrittleSolid {
    /// When a block cleaves and into what.
    pub cleaving: CleaveRule,
    /// Blast loading, in N·s/m², that breaks a bond between two wedges of
    /// the same block.
    ///
    /// Well below what it took to crack the block, because the block is
    /// already cracked: what holds two wedges together afterwards is the key
    /// of two fresh surfaces, not the substance. Held at the strength of the
    /// whole block — which is what this used to be — a struck block loses the
    /// one wedge the blow landed on and the rest stay rigid, so it stands
    /// there visibly cracked and behaving like one solid piece.
    pub joint_threshold: f32,
    /// Contact blow, in N·s, that breaks a bond between two wedges of the
    /// same block: the same fraction of the blow that cracked it.
    pub joint_toughness: f32,
    /// How many times over a block may be cleaved before a hit takes the
    /// piece out whole instead of dividing it further. One means the wedges
    /// of the first break are the pieces that fall.
    pub max_cleave_depth: u32,
    /// The breakable blocks and how a wedge of them is made: a collider it
    /// does not cover is something else — a stone footing under an ice wall
    /// — and is never cleaved.
    pub blocks: Cuttable,
    /// How deep in the cleaving each child was born.
    pub depths: BreakDepths,
}

/// Blocks cleave once, by default: a struck block breaks into wedges, and a
/// hit on one of those wedges takes it out rather than splintering it again.
const DEFAULT_MAX_CLEAVE_DEPTH: u32 = 1;

/// What a block's wedges hold each other at, as a fraction of the blow it
/// took to cleave the block in the first place.
///
/// Low enough that a cracked block comes apart under the next thing that
/// touches it rather than standing there in one rigid piece, high enough that
/// it does not fall apart on its own the moment it cracks. It has not been
/// play-tested; [`BrittleSolid::keyed_at`] overrides it per object.
const WEDGE_BOND: f32 = 0.25;

impl BrittleSolid {
    /// An object of whole blocks, none yet broken.
    pub fn new(cleaving: CleaveRule, blocks: Cuttable) -> Self {
        Self {
            cleaving,
            joint_threshold: cleaving.threshold * WEDGE_BOND,
            joint_toughness: cleaving.threshold * WEDGE_BOND,
            max_cleave_depth: DEFAULT_MAX_CLEAVE_DEPTH,
            blocks,
            depths: BreakDepths::default(),
        }
    }

    /// An object whose wedges hold each other against a blast loading of
    /// `threshold` N·s/m² and a contact spike of `toughness` N·s.
    pub fn keyed_at(mut self, threshold: f32, toughness: f32) -> Self {
        self.joint_threshold = threshold;
        self.joint_toughness = toughness;
        self
    }

    /// An object whose pieces may themselves cleave, `depth` levels deep.
    pub fn cleaving_at_most(mut self, depth: u32) -> Self {
        self.max_cleave_depth = depth.max(1);
        self
    }

    /// Whether `child` is one of the breakable blocks.
    pub fn is_brittle(&self, child: ColliderHandle) -> bool {
        self.blocks.covers(child)
    }

    /// Whether a hit on `child` cleaves it, or takes it out whole.
    pub fn may_cleave(&self, child: ColliderHandle) -> bool {
        self.depths.of(child) < self.max_cleave_depth
    }

    /// Whether a piece split off with `children` is still solid enough to
    /// need this: more than one block, or a block that may cleave again.
    pub fn worth_sharing(&self, children: &[ColliderHandle]) -> bool {
        children.len() > 1
            || children
                .iter()
                .any(|c| self.is_brittle(*c) && self.may_cleave(*c))
    }

    /// The solid of a piece split off with `children`: the same rules and
    /// the records of its own blocks.
    pub fn share(&self, children: &[ColliderHandle]) -> Self {
        Self {
            cleaving: self.cleaving,
            joint_threshold: self.joint_threshold,
            joint_toughness: self.joint_toughness,
            max_cleave_depth: self.max_cleave_depth,
            blocks: self.blocks.share(children),
            depths: self.depths.share(children),
        }
    }

    /// Forget the records of every block but `children`.
    pub fn retain(&mut self, children: &[ColliderHandle]) {
        self.blocks.retain(children);
        self.depths.retain(children);
    }
}
