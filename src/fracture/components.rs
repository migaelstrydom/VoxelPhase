//! Fracture-related ECS components.

use specs::{Component, VecStorage};

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
/// when the *spike* (frame-over-frame increase) in impulse on either endpoint
/// exceeds the joint's threshold. This distinguishes sharp impacts from
/// sustained pushing forces.
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
    /// Rendering material for building models of detached pieces
    /// and rebuilding the remaining compound model.
    pub material: MaterialId,
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
            material,
            piece_mesh: cuboid_mesh,
        }
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

        self.child_count = new_count;
    }
}
