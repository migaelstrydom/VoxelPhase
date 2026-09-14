//! Fracture-related ECS components.

use specs::{Component, VecStorage};

use super::contact_load::ContactLoadTracker;
use crate::app::spawnables::shared::models::{cuboid_mesh, PieceMesh};
use crate::rendering::material::MaterialId;

/// A structural joint between two children of a compound body.
///
/// The joint breaks when the solver impulse on *either* endpoint collider
/// exceeds `threshold`.
#[derive(Debug, Clone)]
pub struct FractureJoint {
    /// Index of the first child in the body's collider list.
    pub child_a: usize,
    /// Index of the second child in the body's collider list.
    pub child_b: usize,
    /// Impulse magnitude (N-s) above which this joint breaks.
    pub threshold: f32,
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
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct CompoundFracture {
    /// Structural joints between children.
    pub joints: Vec<FractureJoint>,
    /// Number of children (colliders) in the compound body.
    /// Tracked separately because joint edges alone don't tell us
    /// about isolated nodes (children with no joints).
    pub child_count: usize,
    /// Per-child contact load history, the baseline the impact spike is
    /// measured against. Kept in step with `child_count` through
    /// [`CompoundFracture::remap_children`].
    pub contact_load: ContactLoadTracker,
    /// Contact spike at a joint, in N·s, above which that joint lets go.
    ///
    /// Separate from the joints' own `threshold` because the two loads are not
    /// the same scale: a joint authored to fail under an 8 N·s blast may have
    /// to hold thousands of N·s of its own structure settling. `None` judges
    /// contact on the joint's blast threshold, which is right only for a
    /// compound light enough that the two are comparable.
    pub contact_threshold: Option<f32>,
    /// Rendering material of each child, indexed by child.
    ///
    /// Per child rather than per object because a compound need not be made of
    /// one substance: the plank bridge's beams and planks wear different
    /// materials, and a single material for the whole object re-textured every
    /// beam the moment anything broke off it. Kept in step with `child_count`
    /// through [`CompoundFracture::remap_children`].
    pub materials: Vec<MaterialId>,
    /// How one child is drawn.
    ///
    /// The system knows a child only as a box collider, which is where it is
    /// and not what it looks like. Without this, every object came apart into
    /// plain cuboids — and an object whose children are *not* plain cuboids
    /// changed shape at the moment it broke, which is the one moment the
    /// player is looking at it.
    pub piece_mesh: PieceMesh,
}

impl CompoundFracture {
    /// A destructible compound whose children are plain boxes.
    pub fn boxes(joints: Vec<FractureJoint>, child_count: usize, material: MaterialId) -> Self {
        Self {
            joints,
            child_count,
            contact_load: ContactLoadTracker::new(child_count),
            contact_threshold: None,
            materials: vec![material; child_count],
            piece_mesh: cuboid_mesh,
        }
    }

    /// The material each child is drawn with, for a compound that is not all
    /// one substance. Indexed in step with the body's collider list.
    pub fn with_piece_materials(mut self, materials: Vec<MaterialId>) -> Self {
        self.materials = materials;
        self
    }

    /// The material of one child, or the last one given if the list is short.
    pub fn material_of(&self, child: usize) -> MaterialId {
        self.materials
            .get(child)
            .or_else(|| self.materials.last())
            .copied()
            .unwrap_or(MaterialId(0))
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
        self.piece_mesh = piece_mesh;
        self
    }

    /// Compute connected components from the surviving joints.
    /// Returns a list of components, each a sorted list of child indices.
    pub fn connected_components(&self) -> Vec<Vec<usize>> {
        let n = self.child_count;
        let mut visited = vec![false; n];
        let mut components = Vec::new();

        // Build adjacency from surviving joints.
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for joint in &self.joints {
            if joint.child_a < n && joint.child_b < n {
                adj[joint.child_a].push(joint.child_b);
                adj[joint.child_b].push(joint.child_a);
            }
        }

        for start in 0..n {
            if visited[start] {
                continue;
            }
            let mut component = Vec::new();
            let mut stack = vec![start];
            while let Some(node) = stack.pop() {
                if visited[node] {
                    continue;
                }
                visited[node] = true;
                component.push(node);
                for &neighbor in &adj[node] {
                    if !visited[neighbor] {
                        stack.push(neighbor);
                    }
                }
            }
            component.sort_unstable();
            components.push(component);
        }

        components
    }

    /// Remap child indices after some children have been removed.
    /// `kept` lists the old indices that survive, in new-index order.
    pub fn remap_children(&mut self, kept: &[usize], new_count: usize) {
        let mut old_to_new = vec![None; self.child_count];
        for (new_idx, &old_idx) in kept.iter().enumerate() {
            if old_idx < old_to_new.len() {
                old_to_new[old_idx] = Some(new_idx);
            }
        }

        self.joints.retain(|j| {
            old_to_new.get(j.child_a).copied().flatten().is_some()
                && old_to_new.get(j.child_b).copied().flatten().is_some()
        });

        for joint in &mut self.joints {
            joint.child_a = old_to_new[joint.child_a].unwrap();
            joint.child_b = old_to_new[joint.child_b].unwrap();
        }

        self.contact_load.remap(kept);
        self.materials = kept.iter().map(|&old| self.material_of(old)).collect();

        self.child_count = new_count;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn joint(a: usize, b: usize) -> FractureJoint {
        FractureJoint {
            child_a: a,
            child_b: b,
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
        let mut fracture = CompoundFracture::boxes(vec![joint(0, 2)], 4, plank)
            .with_piece_materials(vec![beam, beam, plank, plank]);

        assert_eq!(fracture.material_of(0), beam);
        assert_eq!(fracture.material_of(3), plank);

        // Keep one beam and one plank; both must keep their own material.
        fracture.remap_children(&[1, 3], 2);

        assert_eq!(fracture.material_of(0), beam);
        assert_eq!(fracture.material_of(1), plank);
    }

    /// A compound built from one material still behaves as it always did.
    #[test]
    fn a_single_material_compound_paints_every_child_the_same() {
        let only = MaterialId(3);
        let fracture = CompoundFracture::boxes(Vec::new(), 3, only);

        for child in 0..3 {
            assert_eq!(fracture.material_of(child), only);
        }
        // And a child index past the end still answers with something sane.
        assert_eq!(fracture.material_of(99), only);
    }
}
