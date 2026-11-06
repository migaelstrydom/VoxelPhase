//! Fracture-related ECS components.

use std::sync::Arc;

use nalgebra::Vector3;
use rustc_hash::FxHashMap;
use specs::{Component, VecStorage};

use super::contact_load::{ContactLoadTracker, Deadband};
use crate::app::spawnables::shared::models::{HullMesh, PieceMesh, PieceStyle, SurfaceUvs};
use crate::collision::convex_hull::ConvexHull;
use crate::physics::ColliderHandle;
use crate::rendering::material::MaterialId;

/// A structural joint between two children of a compound body.
///
/// The joint breaks when the solver impulse on *either* endpoint collider
/// exceeds `threshold`.
#[derive(Debug, Clone)]
pub struct FractureJoint {
    /// The first child's collider.
    pub child_a: ColliderHandle,
    /// The second child's collider.
    pub child_b: ColliderHandle,
    /// Impulse magnitude (N-s) above which this joint breaks.
    pub threshold: f32,
}

impl FractureJoint {
    /// Whether this joint holds `child`.
    pub fn holds(&self, child: ColliderHandle) -> bool {
        self.child_a == child || self.child_b == child
    }
}

/// Marks a compound body as destructible.
///
/// Joints define the structural connections between children. A joint breaks
/// when the load on either endpoint exceeds the joint's threshold, where the
/// load comes from two sources:
///
/// - an explicit impulse (an explosion), read at full magnitude; and
/// - contact, read as the *spike* — the frame-over-frame increase in the
///   impulse the solver pushed through that child.
///
/// Contact has to be read as a spike rather than a level because a heavy object
/// standing still already pushes its own weight through its lowest children
/// every frame, which for anything massive exceeds a sensible fracture
/// threshold outright. The spike is near zero for resting weight and for a
/// player leaning on the object, and enormous for a landing.
///
/// After breaking, the system computes connected components from the surviving
/// joints and splits disconnected groups into independent bodies.
///
/// Every per-child record is keyed by the child's collider handle, which a
/// collider keeps whatever happens to the body's collider list around it.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct CompoundFracture {
    /// Structural joints between children.
    pub joints: Vec<FractureJoint>,
    /// Per-child contact load history, the baseline the impact spike is
    /// measured against.
    pub contact_load: ContactLoadTracker,
    /// Contact spike at a joint, in N·s, above which that joint lets go.
    ///
    /// Separate from the joints' own `threshold` because the two loads are not
    /// the same scale: a joint authored to fail under an 8 N·s blast may have
    /// to hold thousands of N·s of its own structure settling. `None` judges
    /// contact on the joint's blast threshold, which is right only for a
    /// compound light enough that the two are comparable.
    pub contact_threshold: Option<f32>,
    /// What a child with no entry in `materials` is drawn with.
    pub material: MaterialId,
    /// Rendering material of each child that differs from `material`.
    ///
    /// Per child rather than per object because a compound need not be made of
    /// one substance: the plank bridge's beams and planks wear different
    /// materials, and a single material for the whole object re-textured every
    /// beam the moment anything broke off it.
    pub materials: FxHashMap<ColliderHandle, MaterialId>,
    /// How this compound's children are drawn, by shape.
    ///
    /// The system knows a box child only as a collider, which is where it is
    /// and not what it looks like. Without this, every object came apart into
    /// plain cuboids — and an object whose children are *not* plain cuboids
    /// changed shape at the moment it broke, which is the one moment the
    /// player is looking at it. A convex-hull child mostly needs no such
    /// help — the hull is its own drawing — which is why the default draws it
    /// exactly as it collides.
    pub style: PieceStyle,
    /// Impulse, in N·s, to hand each child if it comes free this frame. No
    /// entry for a child nobody has pushed.
    ///
    /// Set by whoever severs a child for a reason the solver did not see —
    /// a shard knocked out of a window carries the momentum of what hit it,
    /// and the hit itself was absorbed by the compound before the shard
    /// existed as a body. Consumed when the child is spawned free.
    pub kicks: FxHashMap<ColliderHandle, Vector3<f32>>,
    /// Whether joints were removed by something other than this frame's load
    /// judgement, so the split step must run even though no joint broke under
    /// load. Cleared when the split runs.
    pub split_pending: bool,
    /// Whether freed pieces are handed to the debris budget, which takes the
    /// small ones away once the break has been seen.
    ///
    /// Opt-in per compound: a window's shards are scenery the moment they
    /// land, while the planks of a bridge are still the bridge.
    pub sheds_debris: bool,
    /// Whether the compound is giving up its last piece: on the next split
    /// every component comes free, nothing stays on the original body, and
    /// the original entity is deleted.
    ///
    /// The split otherwise always keeps its largest component, which is right
    /// for a wall losing a brick and wrong for a pane chipped down to a
    /// crumb: a fixed body keeps whatever it is left with hanging in the air.
    pub released: bool,
    /// Where the object's texture origin sits in the body's *current* frame.
    ///
    /// The pattern a compound wears is projected from its body origin, and
    /// that origin moves: losing a piece leaves the survivors clustered to one
    /// side, so the body is re-centred on what is left of it. Nothing moves in
    /// the world when that happens, but every child's offset changes, and a
    /// projection read from the new origin slides the pattern across the
    /// object — the remnant's markings shift while a piece that broke off a
    /// moment earlier keeps the old ones, which is exactly the look of two
    /// halves of one block wearing different textures.
    ///
    /// Adding this to a child's offset puts it back where it was when the
    /// object was whole, so the markings stay with the material rather than
    /// with the body's bookkeeping.
    pub texture_anchor: Vector3<f32>,
    /// The object's shape before anything broke off it, in the frame child
    /// offsets are read in once `texture_anchor` is added. `None` unless the
    /// owner kept it.
    ///
    /// Meaningful for an object that is one solid — a block that cleaves —
    /// where it lets the piece mesh tell the block's old surfaces from the
    /// ones it broke along. See
    /// [`PieceHull::whole`](crate::app::spawnables::shared::models::PieceHull::whole).
    pub whole_shape: Option<Arc<ConvexHull>>,
    /// Whether the body's children have changed since its model was last
    /// built.
    ///
    /// Set by whatever reshapes the compound — a cleave, a craze, a split —
    /// and consumed by `FractureSystem`, which rebuilds the model once, after
    /// it has decided what comes free. A blast that cleaves every block of a
    /// dome and then frees most of the wedges would otherwise draw the whole
    /// dome twice in one frame, the first time for nothing.
    pub model_stale: bool,
}

impl CompoundFracture {
    /// A destructible compound whose children are plain boxes, all drawn
    /// with `material` unless [`Self::with_piece_materials`] says otherwise.
    pub fn boxes(joints: Vec<FractureJoint>, material: MaterialId) -> Self {
        Self {
            joints,
            contact_load: ContactLoadTracker::default(),
            contact_threshold: None,
            material,
            materials: FxHashMap::default(),
            style: PieceStyle::default(),
            kicks: FxHashMap::default(),
            split_pending: false,
            sheds_debris: false,
            released: false,
            texture_anchor: Vector3::zeros(),
            whole_shape: None,
            model_stale: false,
        }
    }

    /// Draw the object's pattern as though its origin sat at `anchor`, so
    /// that objects built alike do not all wear the same markings.
    pub fn with_texture_anchor(mut self, anchor: Vector3<f32>) -> Self {
        self.texture_anchor = anchor;
        self
    }

    /// Keep the object's whole shape, in its texture frame, for the piece
    /// meshes to read. See [`Self::whole_shape`].
    pub fn with_whole_shape(mut self, whole: Arc<ConvexHull>) -> Self {
        self.whole_shape = Some(whole);
        self
    }

    /// Freed pieces count against the [`DebrisBudget`](super::DebrisBudget).
    pub fn shedding_debris(mut self) -> Self {
        self.sheds_debris = true;
        self
    }

    /// The material each child is drawn with, for a compound that is not all
    /// one substance.
    pub fn with_piece_materials(
        mut self,
        materials: impl IntoIterator<Item = (ColliderHandle, MaterialId)>,
    ) -> Self {
        self.materials.extend(materials);
        self
    }

    /// Draw `child` with `material`.
    pub fn set_material(&mut self, child: ColliderHandle, material: MaterialId) {
        self.materials.insert(child, material);
    }

    /// The material of one child: its own, or the compound's if it has none.
    pub fn material_of(&self, child: ColliderHandle) -> MaterialId {
        self.materials.get(&child).copied().unwrap_or(self.material)
    }

    /// The contact spike, in N·s, that breaks one of this compound's joints.
    ///
    /// Measure it: run the object through the impacts it should and should not
    /// survive and read the spikes off, as the tests beside its spawnable do.
    /// Guessing gives either a structure that shatters when leaned on or one
    /// that cannot be broken by hand.
    pub fn breaking_on_impact_at(mut self, contact_threshold: f32) -> Self {
        self.contact_threshold = Some(contact_threshold);
        self
    }

    /// The same, for children with a shape of their own.
    pub fn with_piece_mesh(mut self, piece_mesh: PieceMesh) -> Self {
        self.style.boxes = piece_mesh;
        self
    }

    /// How this compound's convex-hull children are drawn, for an object whose
    /// look is not quite its collision shape.
    pub fn with_hull_mesh(mut self, hull_mesh: HullMesh) -> Self {
        self.style.hulls = hull_mesh;
        self
    }

    /// How this compound's pieces are laid out on the texture they share.
    ///
    /// Set it for any object whose texture is the substance it is made of.
    /// The default fits the texture to each face, which is a decoration and
    /// not a material: a fragment of an object textured that way changes size
    /// of pattern the instant it comes off.
    pub fn with_uvs(mut self, uvs: SurfaceUvs) -> Self {
        self.style.uvs = uvs;
        self
    }

    /// Cut every joint that holds `child`, so it comes free at the next split,
    /// and give it `kick` on its way out.
    ///
    /// For a child that fails for a reason the joint judgement cannot see —
    /// it fatigued, or it sat under a hit that a finer structure absorbed —
    /// rather than for a load the solver put through it.
    pub fn sever(&mut self, child: ColliderHandle, kick: Vector3<f32>) {
        self.joints.retain(|joint| !joint.holds(child));
        *self.kicks.entry(child).or_insert_with(Vector3::zeros) += kick;
        self.split_pending = true;
    }

    /// Drop everything recorded about `child`, which has left the body: its
    /// joints, its material and its kick.
    pub fn forget(&mut self, child: ColliderHandle) {
        self.joints.retain(|joint| !joint.holds(child));
        self.materials.remove(&child);
        self.kicks.remove(&child);
    }

    /// Judge contact spikes against each child's own weight rather than the
    /// whole body's.
    pub fn with_deadband(mut self, deadband: Deadband) -> Self {
        self.contact_load = std::mem::take(&mut self.contact_load).with_deadband(deadband);
        self
    }

    /// Give up every remaining child on the next split and retire the body.
    pub fn release(&mut self) {
        self.released = true;
        self.split_pending = true;
    }

    /// The impulse a freed child leaves with.
    pub fn kick_of(&self, child: ColliderHandle) -> Vector3<f32> {
        self.kicks
            .get(&child)
            .copied()
            .unwrap_or_else(Vector3::zeros)
    }

    /// The groups of `children` still joined to each other through surviving
    /// joints. Joints naming a collider not in `children` are ignored.
    ///
    /// Groups come in the order of the first child of each in `children`, and
    /// each lists its children in the order `children` gives them: pass the
    /// body's collider list, and the groups follow the body.
    pub fn connected_components(&self, children: &[ColliderHandle]) -> Vec<Vec<ColliderHandle>> {
        let place: FxHashMap<ColliderHandle, usize> = children
            .iter()
            .enumerate()
            .map(|(place, child)| (*child, place))
            .collect();
        let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); children.len()];
        for joint in &self.joints {
            if let (Some(&a), Some(&b)) = (place.get(&joint.child_a), place.get(&joint.child_b)) {
                adjacent[a].push(b);
                adjacent[b].push(a);
            }
        }

        let mut visited = vec![false; children.len()];
        let mut components = Vec::new();
        for start in 0..children.len() {
            if visited[start] {
                continue;
            }
            let mut component = Vec::new();
            let mut stack = vec![start];
            while let Some(node) = stack.pop() {
                if std::mem::replace(&mut visited[node], true) {
                    continue;
                }
                component.push(node);
                stack.extend(adjacent[node].iter().filter(|n| !visited[**n]));
            }
            component.sort_unstable();
            components.push(component.into_iter().map(|node| children[node]).collect());
        }
        components
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn child(slot: usize) -> ColliderHandle {
        ColliderHandle::for_test(slot)
    }

    fn joint(a: usize, b: usize) -> FractureJoint {
        FractureJoint {
            child_a: child(a),
            child_b: child(b),
            threshold: 1.0,
        }
    }

    /// A compound of more than one substance must not be repainted when it
    /// breaks. The plank bridge is two beams and eight planks wearing different
    /// materials; carrying one material for the whole object re-textured every
    /// beam the instant any plank came off, and a detaching beam came away
    /// looking like a plank.
    #[test]
    fn a_surviving_child_keeps_the_material_it_had() {
        let beam = MaterialId(7);
        let plank = MaterialId(9);
        let mut fracture = CompoundFracture::boxes(vec![joint(0, 2)], plank)
            .with_piece_materials([(child(0), beam), (child(1), beam)]);

        assert_eq!(fracture.material_of(child(0)), beam);
        assert_eq!(fracture.material_of(child(3)), plank);

        // Lose one beam and one plank; the others must keep their own material.
        fracture.forget(child(0));
        fracture.forget(child(2));

        assert_eq!(fracture.material_of(child(1)), beam);
        assert_eq!(fracture.material_of(child(3)), plank);
    }

    /// A compound built from one material still behaves as it always did.
    #[test]
    fn a_single_material_compound_paints_every_child_the_same() {
        let only = MaterialId(3);
        let fracture = CompoundFracture::boxes(Vec::new(), only);

        for slot in 0..3 {
            assert_eq!(fracture.material_of(child(slot)), only);
        }
        // And a child it has never heard of still answers with something sane.
        assert_eq!(fracture.material_of(child(99)), only);
    }

    /// Groups follow the list they are read against, whatever order the
    /// joints were made in.
    #[test]
    fn groups_follow_the_order_of_the_children() {
        let fracture = CompoundFracture::boxes(vec![joint(3, 0), joint(1, 2)], MaterialId(0));
        let children = [child(2), child(0), child(1), child(3), child(4)];
        assert_eq!(
            fracture.connected_components(&children),
            vec![
                vec![child(2), child(1)],
                vec![child(0), child(3)],
                vec![child(4)],
            ]
        );
    }
}
