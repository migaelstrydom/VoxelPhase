//! The component that makes a compound body a thing of brittle blocks.

use specs::{Component, VecStorage};

use super::plan::CleaveRule;
use crate::fracture::{BreakDepths, ContactLoadTracker, Deadband};
use crate::rendering::material::MaterialId;

/// A compound body whose children break into wedges where they are struck.
///
/// Sits beside a [`CompoundFracture`](crate::fracture::CompoundFracture) as
/// [`BrittleSheet`](crate::glass::BrittleSheet) does: this component decides
/// how a child *divides*, that one decides when the divisions *let go*. A
/// block that is hit hard enough is replaced by the two or three wedges it
/// cleaved into, joined to each other and to whatever its neighbours were,
/// and the fracture system takes it from there.
///
/// An object needs no special build to carry one: an igloo is already a
/// compound of ice blocks with a joint per touching pair, and a single block
/// is a compound of one.
#[derive(Component)]
#[storage(VecStorage)]
pub struct BrittleSolid {
    /// When a block cleaves and into what.
    pub cleaving: CleaveRule,
    /// Impulse, in N·s, that breaks a joint between two wedges of the same
    /// block. Fresh fracture surfaces key together, so this is usually
    /// higher than the joint that held the block to its neighbours.
    pub joint_threshold: f32,
    /// How many times over a block may be cleaved before a hit takes the
    /// piece out whole instead of dividing it further. One means the wedges
    /// of the first break are the pieces that fall.
    pub max_cleave_depth: u32,
    /// What marks a child as breakable: a child wearing any other material
    /// is something else — a stone footing under an ice wall — and is never
    /// cleaved.
    pub material: MaterialId,
    /// Contact spikes per child, read on this component's own baseline so
    /// cleaving can run before the fracture system judges the same frame.
    pub contact_load: ContactLoadTracker,
    /// How deep in the cleaving each child was born.
    pub depths: BreakDepths,
}

/// Blocks cleave once, by default: a struck block breaks into wedges, and a
/// hit on one of those wedges takes it out rather than splintering it again.
const DEFAULT_MAX_CLEAVE_DEPTH: u32 = 1;

impl BrittleSolid {
    /// An object of `child_count` whole blocks, none yet broken.
    pub fn new(cleaving: CleaveRule, material: MaterialId, child_count: usize) -> Self {
        Self {
            cleaving,
            joint_threshold: cleaving.threshold * 1.5,
            max_cleave_depth: DEFAULT_MAX_CLEAVE_DEPTH,
            material,
            contact_load: ContactLoadTracker::new(child_count),
            depths: BreakDepths::new(child_count),
        }
    }

    /// An object whose wedges hold each other at `threshold` N·s.
    pub fn keyed_at(mut self, threshold: f32) -> Self {
        self.joint_threshold = threshold;
        self
    }

    /// An object whose pieces may themselves cleave, `depth` levels deep.
    pub fn cleaving_at_most(mut self, depth: u32) -> Self {
        self.max_cleave_depth = depth.max(1);
        self
    }

    /// Judge contact spikes against each child's own weight, as an object
    /// whose blocks differ in size must.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.contact_load = std::mem::take(&mut self.contact_load).with_deadband(deadband);
        self
    }

    /// Whether the child wearing `material` is one of the breakable blocks.
    pub fn is_brittle(&self, material: MaterialId) -> bool {
        material == self.material
    }

    /// Whether a hit on `child` cleaves it, or takes it out whole.
    pub fn may_cleave(&self, child: usize) -> bool {
        self.depths.of(child) < self.max_cleave_depth
    }
}
