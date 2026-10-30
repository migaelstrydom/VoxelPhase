//! Warm starts solved under last frame's scales, carried over to this frame's.

use rustc_hash::FxHashMap;
use smallvec::SmallVec;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::super::conditioning::ManifoldConditions;
use super::contact_graph::ContactGraph;

/// The most a body's supports' warm starts are scaled by in one frame.
const MAX_RATIO: f32 = 4.0;

/// Frames a pair's scales are remembered after it last came up: a pair
/// asleep that long has nothing left warm-started from them.
const FORGET_AFTER_FRAMES: u64 = 600;

/// Rescales each body's supports' warm starts to the scales the pairs above
/// it are solved with this frame.
///
/// Under scaling a resting body is held from below by its own load plus the
/// scaled share of what rests on it: `support = own + s·load_above`. Warm
/// starts solved under one set of scales are not a fixed point under another,
/// and the difference pops the stack up the frame the scales change.
/// Working top-down, each body's supports are scaled together so they carry
/// its own load, which a change of scale leaves alone, plus what this frame's
/// scales pass down to it.
#[derive(Default)]
pub(super) struct WarmStartRemap {
    /// Each pair's scales when its warm starts were last solved, and that frame.
    solved_under: FxHashMap<(Option<RigidBodyHandle>, RigidBodyHandle), ((f32, f32), u64)>,
    /// This frame's vertical supports, by manifold.
    supports: Vec<Option<Support>>,
    /// Each body's supports, and the supports it gives.
    below: FxHashMap<RigidBodyHandle, SmallVec<[usize; 4]>>,
    above: FxHashMap<RigidBodyHandle, SmallVec<[usize; 4]>>,
    /// How much each manifold's warm starts are scaled by.
    ratios: Vec<f32>,
    /// Frames remapped.
    frame: u64,
}

/// A manifold holding a body up.
struct Support {
    upper: RigidBodyHandle,
    /// The body under it, `None` for the ground.
    lower: Option<RigidBodyHandle>,
    /// The vertical impulse its warm start gives the upper body.
    lift: f32,
    /// The lower body's scale when the warm start was solved, and now.
    solved_scale: f32,
    scale: f32,
}

impl WarmStartRemap {
    pub(super) fn apply(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        graph: &ContactGraph,
        conditions: &ManifoldConditions,
        up: Vector3<f32>,
        vertical: f32,
    ) {
        self.frame += 1;
        self.collect_supports(bodies, manifolds, graph, conditions, up, vertical);
        self.solve_ratios(graph);
        for (manifold, &ratio) in manifolds.iter_mut().zip(&self.ratios) {
            if ratio != 1.0 {
                for contact in &mut manifold.contacts {
                    contact.warm_normal_impulse *= ratio;
                    contact.warm_friction_impulse_ws *= ratio;
                }
            }
        }

        let frame = self.frame;
        for (index, manifold) in manifolds.iter().enumerate() {
            let key = (manifold.header.body_a, manifold.header.body_b);
            self.solved_under
                .insert(key, (conditions.shock_scales_for(index), frame));
        }
        if frame % 120 == 0 {
            self.solved_under
                .retain(|_, (_, seen)| frame - *seen < FORGET_AFTER_FRAMES);
        }
    }

    /// Each manifold within `vertical` (|normal·up|) that holds a body up,
    /// whether from another body or from the ground.
    fn collect_supports(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        graph: &ContactGraph,
        conditions: &ManifoldConditions,
        up: Vector3<f32>,
        vertical: f32,
    ) {
        let is_dynamic = |h: RigidBodyHandle| bodies.get(h.0).is_some_and(|b| b.is_dynamic());
        self.supports.clear();
        self.below.clear();
        self.above.clear();
        for (index, manifold) in manifolds.iter().enumerate() {
            let header = &manifold.header;
            let depth_a = header.body_a.and_then(|h| graph.depth.get(&h).copied());
            let depth_b = graph.depth.get(&header.body_b).copied();
            // The upper body, and the lower one with its side (0 for a).
            let roles = match (header.body_a, depth_a, depth_b) {
                (Some(a), Some(da), Some(db)) if da < db => Some((header.body_b, Some((a, 0)))),
                (Some(a), Some(da), Some(db)) if da > db => Some((a, Some((header.body_b, 1)))),
                (a, None, Some(_)) if a.is_none_or(|a| !is_dynamic(a)) => {
                    Some((header.body_b, None))
                }
                (Some(a), Some(_), None) if !is_dynamic(header.body_b) => Some((a, None)),
                _ => None,
            };
            let support = roles.and_then(|(upper, lower)| {
                let normal: Vector3<f32> = manifold.contacts.iter().map(|c| c.normal).sum();
                if !(normal.normalize().dot(&up).abs() >= vertical) {
                    return None;
                }
                let toward_upper = if upper == header.body_b { 1.0 } else { -1.0 };
                let lift: f32 = manifold
                    .contacts
                    .iter()
                    .map(|c| c.warm_normal_impulse * c.normal.dot(&up) * toward_upper)
                    .sum();
                let side =
                    |scales: (f32, f32), side: usize| if side == 0 { scales.0 } else { scales.1 };
                let (solved_scale, scale) = match lower {
                    Some((_, s)) => (
                        self.solved_under
                            .get(&(header.body_a, header.body_b))
                            .map_or(1.0, |(scales, _)| side(*scales, s)),
                        side(conditions.shock_scales_for(index), s),
                    ),
                    None => (1.0, 1.0),
                };
                Some(Support {
                    upper,
                    lower: lower.map(|(body, _)| body),
                    lift: lift.max(0.0),
                    solved_scale,
                    scale,
                })
            });
            if let Some(support) = &support {
                self.below.entry(support.upper).or_default().push(index);
                if let Some(lower) = support.lower {
                    self.above.entry(lower).or_default().push(index);
                }
            }
            self.supports.push(support);
        }
    }

    /// Each body's supports' ratio, top-down so a body's loads from above
    /// are already remapped when it is reached.
    fn solve_ratios(&mut self, graph: &ContactGraph) {
        self.ratios.clear();
        self.ratios.resize(self.supports.len(), 1.0);
        let mut order: Vec<RigidBodyHandle> = self.below.keys().copied().collect();
        order.sort_by_key(|body| std::cmp::Reverse(graph.depth.get(body).copied().unwrap_or(0)));
        for body in order {
            let support = |i: usize| self.supports[i].as_ref().unwrap();
            let held: f32 = self.below[&body].iter().map(|&i| support(i).lift).sum();
            if held < 1e-6 {
                continue;
            }
            let (mut solved_load, mut load) = (0.0, 0.0);
            for &i in self.above.get(&body).map_or(&[][..], |v| v.as_slice()) {
                solved_load += support(i).solved_scale * support(i).lift;
                load += support(i).scale * support(i).lift * self.ratios[i];
            }
            let ratio = ((held + load - solved_load) / held).clamp(0.0, MAX_RATIO);
            for &i in &self.below[&body] {
                self.ratios[i] = ratio;
            }
        }
    }
}
