//! Shock propagation conditioner: contact graph ordering + mass scaling.
//!
//! Builds a BFS depth graph from static geometry outward through the contact
//! graph, then sorts manifolds top-down and computes per-manifold mass-scaling
//! factors so that lower bodies appear heavier during the solve.

use rustc_hash::FxHashMap;
use std::collections::VecDeque;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::conditioning::{ManifoldConditioner, ManifoldConditions};

/// Configuration for shock propagation.
#[derive(Debug, Clone, Copy)]
pub struct ShockPropagationConfig {
    /// Decay factor controlling how much lower bodies are stabilised.
    /// 0.0 = full propagation (lower body immovable during upper solve).
    /// 1.0 = disabled (no mass scaling).
    pub shock_alpha: f32,
    /// Contacts with `|dot(normal, gravity_dir)| < horizontal_threshold` are
    /// considered horizontal (wall contacts) and do not create depth edges.
    pub horizontal_threshold: f32,
}

impl Default for ShockPropagationConfig {
    fn default() -> Self {
        Self {
            shock_alpha: 0.3,
            horizontal_threshold: 0.3,
        }
    }
}

/// Shock propagation conditioner.
///
/// Owns a reusable `ContactGraph` whose internal allocations persist across
/// frames.
pub struct ShockPropagationConditioner {
    config: ShockPropagationConfig,
    graph: ContactGraph,
}

impl ShockPropagationConditioner {
    pub fn new(config: ShockPropagationConfig) -> Self {
        Self {
            config,
            graph: ContactGraph::new(),
        }
    }
}

impl Default for ShockPropagationConditioner {
    fn default() -> Self {
        Self::new(ShockPropagationConfig::default())
    }
}

impl ManifoldConditioner for ShockPropagationConditioner {
    fn condition(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
        manifolds: &mut [SolverManifold],
        gravity_dir: Vector3<f32>,
        conditions: &mut ManifoldConditions,
    ) {
        self.graph.rebuild(
            bodies,
            constraints,
            manifolds,
            gravity_dir,
            self.config.horizontal_threshold,
        );
        self.graph
            .sort_manifolds(manifolds, self.config.shock_alpha, conditions);
    }
}

// ---------------------------------------------------------------------------
// Contact graph (internal)
// ---------------------------------------------------------------------------

/// BFS depth graph from static geometry through the contact network.
///
/// All internal allocations are reused across frames (clear + repopulate).
struct ContactGraph {
    /// Depth for each body. Bodies not in the map have no contacts.
    depth: FxHashMap<RigidBodyHandle, u32>,
    /// BFS work queue, kept allocated between frames.
    bfs_queue: VecDeque<RigidBodyHandle>,
    /// Adjacency list: for each body, which bodies does it support
    /// (i.e. which bodies have this body as their `lower`)?
    adjacency: FxHashMap<RigidBodyHandle, Vec<RigidBodyHandle>>,
}

impl ContactGraph {
    fn new() -> Self {
        Self {
            depth: FxHashMap::default(),
            bfs_queue: VecDeque::new(),
            adjacency: FxHashMap::default(),
        }
    }

    /// Rebuild the graph from the current manifold set and active constraints.
    fn rebuild(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
        manifolds: &[SolverManifold],
        gravity_dir: Vector3<f32>,
        horizontal_threshold: f32,
    ) {
        self.depth.clear();
        self.bfs_queue.clear();
        self.adjacency.clear();

        // Pass 0: include active constraints in the graph.
        // World-anchored (single-body) constraints make the body depth 0.
        // Two-body constraints add bidirectional edges.
        for (_, constraint) in constraints.iter() {
            if !constraint.active {
                continue;
            }
            let refs = constraint.kind.referenced_bodies();
            if refs.len() == 1 {
                let handle = refs[0];
                if bodies.get(handle.0).map_or(false, |b| b.is_dynamic()) {
                    if !self.depth.contains_key(&handle) {
                        self.depth.insert(handle, 0);
                        self.bfs_queue.push_back(handle);
                    }
                }
            } else if refs.len() == 2 {
                let a = refs[0];
                let b = refs[1];
                let a_dynamic = bodies.get(a.0).map_or(false, |b| b.is_dynamic());
                let b_dynamic = bodies.get(b.0).map_or(false, |b| b.is_dynamic());
                if a_dynamic && b_dynamic {
                    self.adjacency.entry(a).or_default().push(b);
                    self.adjacency.entry(b).or_default().push(a);
                } else if a_dynamic && !b_dynamic {
                    if !self.depth.contains_key(&a) {
                        self.depth.insert(a, 0);
                        self.bfs_queue.push_back(a);
                    }
                } else if b_dynamic && !a_dynamic {
                    if !self.depth.contains_key(&b) {
                        self.depth.insert(b, 0);
                        self.bfs_queue.push_back(b);
                    }
                }
            }
        }

        // Pass 1: identify depth-0 bodies (touching static geometry) and
        // build directed support edges for dynamic-dynamic pairs.
        for manifold in manifolds {
            let header = &manifold.header;

            if header.body_a.is_none() {
                // body_b touches static geometry → depth 0
                if !self.depth.contains_key(&header.body_b) {
                    self.depth.insert(header.body_b, 0);
                    self.bfs_queue.push_back(header.body_b);
                }
                continue;
            }

            let handle_a = header.body_a.unwrap();
            let handle_b = header.body_b;

            // Compute average contact normal for this manifold
            let avg_normal = {
                let mut sum = Vector3::zeros();
                for c in &manifold.contacts {
                    sum += c.normal;
                }
                let len = sum.norm();
                if len < 1e-6 {
                    continue;
                }
                sum / len
            };

            let alignment = avg_normal.dot(&gravity_dir);

            // Skip near-horizontal contacts (wall contacts)
            if alignment.abs() < horizontal_threshold {
                continue;
            }

            // Normal convention: normal points from A to B.
            // If alignment < 0 (normal opposes gravity → points upward),
            // then A supports B (A is lower, B is upper).
            let (upper, lower) = if alignment < 0.0 {
                (handle_b, handle_a)
            } else {
                (handle_a, handle_b)
            };

            // Only create edges for dynamic bodies
            let upper_dynamic = bodies.get(upper.0).map_or(false, |b| b.is_dynamic());
            let lower_dynamic = bodies.get(lower.0).map_or(false, |b| b.is_dynamic());

            if upper_dynamic && lower_dynamic {
                self.adjacency.entry(lower).or_default().push(upper);
            } else if upper_dynamic && !lower_dynamic {
                // Lower is kinematic/static-like — upper is at depth 0
                if !self.depth.contains_key(&upper) {
                    self.depth.insert(upper, 0);
                    self.bfs_queue.push_back(upper);
                }
            }
        }

        // Pass 2: BFS from depth-0 bodies outward
        while let Some(current) = self.bfs_queue.pop_front() {
            let current_depth = self.depth[&current];

            if let Some(supported) = self.adjacency.get(&current) {
                for &upper in supported {
                    let new_depth = current_depth + 1;
                    let entry = self.depth.entry(upper).or_insert(u32::MAX);
                    if new_depth < *entry {
                        *entry = new_depth;
                        self.bfs_queue.push_back(upper);
                    }
                }
            }
        }
    }

    /// Sort manifolds top-down (highest depth first) and populate conditions.
    fn sort_manifolds(
        &self,
        manifolds: &mut [SolverManifold],
        shock_alpha: f32,
        conditions: &mut ManifoldConditions,
    ) {
        // Sort by max depth of the pair (descending) so top-of-stack solves first
        manifolds.sort_by(|a, b| {
            let depth_a = self.manifold_sort_key(a);
            let depth_b = self.manifold_sort_key(b);
            depth_b.cmp(&depth_a)
        });

        // Compute shock scales
        conditions.shock_scales.clear();
        conditions.shock_scales.reserve(manifolds.len());

        for manifold in manifolds.iter() {
            let header = &manifold.header;

            let depth_a = header.body_a.and_then(|h| self.depth.get(&h).copied());
            let depth_b = self.depth.get(&header.body_b).copied();

            let scales = match (depth_a, depth_b) {
                (Some(da), Some(db)) if da != db => {
                    let diff = da.abs_diff(db);
                    let factor = shock_alpha.powi(diff as i32);
                    if da < db {
                        // A is lower (closer to ground) → scale A down
                        (factor, 1.0)
                    } else {
                        // B is lower → scale B down
                        (1.0, factor)
                    }
                }
                _ => (1.0, 1.0),
            };

            conditions.shock_scales.push(scales);
        }
    }

    /// Sort key for a manifold: max depth of either body (higher = solved first).
    fn manifold_sort_key(&self, manifold: &SolverManifold) -> u32 {
        let depth_a = manifold
            .header
            .body_a
            .and_then(|h| self.depth.get(&h).copied())
            .unwrap_or(0);
        let depth_b = self
            .depth
            .get(&manifold.header.body_b)
            .copied()
            .unwrap_or(0);
        depth_a.max(depth_b)
    }
}
