//! The component that makes a compound body a thing of brittle blocks.

use nalgebra::Vector3;
use specs::{Component, VecStorage};

use super::plan::CleaveRule;
use crate::fracture::{BreakDepths, ContactLoadTracker, Cuttable, Deadband};
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
    /// How many times over a block may be cleaved before a hit takes the
    /// piece out whole instead of dividing it further. One means the wedges
    /// of the first break are the pieces that fall.
    pub max_cleave_depth: u32,
    /// The breakable blocks and how a wedge of them is made: a collider it
    /// does not cover is something else — a stone footing under an ice wall
    /// — and is never cleaved.
    pub blocks: Cuttable,
    /// Contact spikes per child, read on this component's own baseline so
    /// cleaving can run before the fracture system judges the same frame.
    pub contact_load: ContactLoadTracker,
    /// How deep in the cleaving each child was born.
    pub depths: BreakDepths,
    /// What a blow is measured as.
    pub measure: BlowMeasure,
    /// The body's velocity at the end of the last frame, for
    /// [`BlowMeasure::Arrest`]. `None` until a frame has been seen.
    pub last_velocity: Option<Vector3<f32>>,
}

/// What a blow on a brittle block is measured as.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BlowMeasure {
    /// The jump in contact impulse a child takes from one frame to the next.
    ///
    /// Right for the blocks of a larger structure — an igloo's — where what
    /// matters is the blow one block takes, and the structure as a whole may
    /// not have moved at all.
    #[default]
    ContactSpike,
    /// How hard the whole body was stopped: its mass times the change in its
    /// velocity that gravity does not account for.
    ///
    /// Right for a free block that carries load through contacts on several
    /// sides. A voussoir is squeezed by the thrust of the whole arch, and when
    /// the arch shifts the load on it swings by many times its own weight —
    /// a contact spike far above anything that should break it, while the
    /// block itself barely moves. Squeezed from both sides, its velocity does
    /// not change; stopped by the ground, it does.
    Arrest,
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
            max_cleave_depth: DEFAULT_MAX_CLEAVE_DEPTH,
            blocks,
            contact_load: ContactLoadTracker::default(),
            depths: BreakDepths::default(),
            measure: BlowMeasure::default(),
            last_velocity: None,
        }
    }

    /// An object whose blows are measured as `measure`.
    pub fn measured_by(mut self, measure: BlowMeasure) -> Self {
        self.measure = measure;
        self
    }

    /// The blow the body took this frame by [`BlowMeasure::Arrest`], in N·s,
    /// given its velocity now. Advances the stored velocity, so it must be
    /// called every frame.
    pub fn arrest(&mut self, velocity: Vector3<f32>, gravity: Vector3<f32>, mass: f32) -> f32 {
        let blow = self
            .last_velocity
            .map_or(0.0, |last| (velocity - last - gravity).magnitude() * mass);
        self.last_velocity = Some(velocity);
        blow
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

    /// The solid of a piece split off with `children`: the same rules, the
    /// records of its own blocks, and the body's motion as it was.
    pub fn share(&self, children: &[ColliderHandle]) -> Self {
        Self {
            cleaving: self.cleaving,
            joint_threshold: self.joint_threshold,
            max_cleave_depth: self.max_cleave_depth,
            blocks: self.blocks.share(children),
            contact_load: self.contact_load.share(children),
            depths: self.depths.share(children),
            measure: self.measure,
            last_velocity: self.last_velocity,
        }
    }

    /// Forget the records of every block but `children`.
    pub fn retain(&mut self, children: &[ColliderHandle]) {
        self.blocks.retain(children);
        self.contact_load = self.contact_load.share(children);
        self.depths.retain(children);
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::cleave::CleaveRule;

    const DT: f32 = 1.0 / 60.0;

    fn solid() -> BrittleSolid {
        BrittleSolid::new(CleaveRule::default(), Cuttable::new(Arc::new(Nothing)))
            .measured_by(BlowMeasure::Arrest)
    }

    /// A mesher for a solid whose blocks are never cut here.
    struct Nothing;

    impl crate::fracture::FaceMesher for Nothing {
        fn collider(
            &self,
            hull: Arc<crate::collision::convex_hull::ConvexHull>,
        ) -> crate::physics::ColliderDesc {
            crate::physics::ColliderDesc::convex_hull(hull)
        }

        fn look(
            &self,
            _hull: &crate::collision::convex_hull::ConvexHull,
            _at: Vector3<f32>,
        ) -> crate::model::Look {
            crate::model::Look::default()
        }
    }

    /// Falling freely is not a blow, however fast.
    #[test]
    fn free_fall_is_no_blow() {
        let mut solid = solid();
        let gravity = Vector3::new(0.0, -9.81, 0.0) * DT;
        let mut velocity = Vector3::new(0.0, -8.0, 0.0);
        solid.arrest(velocity, gravity, 500.0);
        velocity += gravity;
        assert!(solid.arrest(velocity, gravity, 500.0) < 1e-2);
    }

    /// Being stopped dead is a blow of the whole momentum.
    #[test]
    fn being_stopped_is_a_blow_of_the_momentum() {
        let mut solid = solid();
        let gravity = Vector3::new(0.0, -9.81, 0.0) * DT;
        solid.arrest(Vector3::new(0.0, -8.0, 0.0), gravity, 500.0);
        let blow = solid.arrest(Vector3::zeros(), gravity, 500.0);
        assert!(
            (blow - 500.0 * (8.0 + 9.81 * DT)).abs() < 1.0,
            "blow {blow}"
        );
    }

    /// Nothing is a blow until a frame has been seen to compare against.
    #[test]
    fn the_first_frame_is_no_blow() {
        assert_eq!(
            solid().arrest(Vector3::new(5.0, 0.0, 0.0), Vector3::zeros(), 500.0),
            0.0
        );
    }
}
