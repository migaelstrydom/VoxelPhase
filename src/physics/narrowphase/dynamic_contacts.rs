//! Unified narrowphase contact generation for all dynamic-vs-dynamic collider pairs.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::collision::contact::ContactManifold;
use crate::collision::discrete::obb_obb::obb_obb_manifold;
use crate::collision::discrete::sphere_obb::sphere_obb_manifold;
use crate::collision::discrete::sphere_sphere::sphere_sphere_manifold;
use crate::collision::obb::Obb;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::collision::continuous::swept_sphere_sphere;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::solver::ContactConstraint;

/// Shape-agnostic snapshot of a collider's world-space state for pair dispatch.
struct ColliderState {
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: ColliderShape,
    bounding_radius: f32,
    velocity: Vector3<f32>,
    material: ColliderMaterial,
    is_sleeping: bool,
}

/// Generate contacts between all pairs of non-static colliders.
///
/// Uses sort-and-sweep broadphase on the axis of greatest positional spread
/// to prune pairs before narrowphase dispatch:
/// - (Sphere, Sphere) → sphere-sphere overlap + speculative sweep
/// - (Sphere, Box) | (Box, Sphere) → OBB closest-point
/// - (Box, Box) → OBB SAT + face clipping
pub fn generate_dynamic_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<ContactConstraint> {
    let states = collect_collider_states(bodies, colliders, sleeping);
    if states.len() < 2 {
        return Vec::new();
    }

    let pairs = sweep_and_prune(&states, contact_margin);
    let mut contacts = Vec::new();

    for (i, j) in pairs {
        let si = &states[i];
        let sj = &states[j];

        let pair_contacts = match (&si.shape, &sj.shape) {
            (ColliderShape::Sphere { radius: ra }, ColliderShape::Sphere { radius: rb }) => {
                sphere_sphere_pair(
                    si,
                    *ra,
                    sj,
                    *rb,
                    contact_margin,
                    dt,
                    ccd_threshold,
                    enable_speculative_contacts,
                    speculative_min_speed,
                    speculative_margin_multiplier,
                )
            }
            (ColliderShape::Sphere { radius }, ColliderShape::Box { half_extents }) => {
                sphere_box_pair(si, *radius, sj, *half_extents, contact_margin)
            }
            (ColliderShape::Box { half_extents }, ColliderShape::Sphere { radius }) => {
                box_sphere_pair(si, *half_extents, sj, *radius, contact_margin)
            }
            (ColliderShape::Box { half_extents: he_a }, ColliderShape::Box { half_extents: he_b }) => {
                box_box_pair(si, *he_a, sj, *he_b, contact_margin)
            }
        };

        contacts.extend(pair_contacts);
    }

    contacts
}

/// Sort-and-sweep broadphase returning candidate pairs.
///
/// Picks the axis with greatest positional spread, sorts colliders by their
/// AABB min on that axis, then sweeps to find overlapping intervals. Pairs
/// that overlap on the sweep axis are checked for full 3-axis AABB overlap
/// before being emitted.
fn sweep_and_prune(states: &[ColliderState], margin: f32) -> Vec<(usize, usize)> {
    let sweep_axis = pick_sweep_axis(states);

    // Build (aabb_min, aabb_max) per collider on each axis.
    let bounds: Vec<([f32; 3], [f32; 3])> = states
        .iter()
        .map(|s| {
            let r = s.bounding_radius + margin;
            (
                [s.center.x - r, s.center.y - r, s.center.z - r],
                [s.center.x + r, s.center.y + r, s.center.z + r],
            )
        })
        .collect();

    // Sort indices by AABB min on the sweep axis.
    let mut sorted: Vec<usize> = (0..states.len()).collect();
    sorted.sort_unstable_by(|&a, &b| {
        bounds[a].0[sweep_axis]
            .partial_cmp(&bounds[b].0[sweep_axis])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut pairs = Vec::new();

    for ii in 0..sorted.len() {
        let i = sorted[ii];
        let i_max = bounds[i].1[sweep_axis];

        for jj in (ii + 1)..sorted.len() {
            let j = sorted[jj];

            // Past the end of i's interval on the sweep axis — no further overlaps.
            if bounds[j].0[sweep_axis] > i_max {
                break;
            }

            // Skip pairs where both are sleeping.
            if states[i].is_sleeping && states[j].is_sleeping {
                continue;
            }

            // Check overlap on the remaining two axes.
            if aabb_overlap_3d(&bounds[i], &bounds[j]) {
                pairs.push((i, j));
            }
        }
    }

    pairs
}

/// Pick the axis (0=x, 1=y, 2=z) with the greatest positional spread.
fn pick_sweep_axis(states: &[ColliderState]) -> usize {
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for s in states {
        let c = [s.center.x, s.center.y, s.center.z];
        for axis in 0..3 {
            min[axis] = min[axis].min(c[axis]);
            max[axis] = max[axis].max(c[axis]);
        }
    }
    let spread = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    if spread[0] >= spread[1] && spread[0] >= spread[2] {
        0
    } else if spread[1] >= spread[2] {
        1
    } else {
        2
    }
}

/// Full 3-axis AABB overlap test.
fn aabb_overlap_3d(a: &([f32; 3], [f32; 3]), b: &([f32; 3], [f32; 3])) -> bool {
    a.0[0] <= b.1[0]
        && a.1[0] >= b.0[0]
        && a.0[1] <= b.1[1]
        && a.1[1] >= b.0[1]
        && a.0[2] <= b.1[2]
        && a.1[2] >= b.0[2]
}

fn collect_collider_states(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<ColliderState> {
    let mut states = Vec::new();

    for (idx, body) in bodies.iter() {
        if body.is_static() {
            continue;
        }
        let body_handle = RigidBodyHandle(idx);
        let is_sleeping = sleeping
            .map(|s| s.contains(&body_handle))
            .unwrap_or(false);

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };
            states.push(ColliderState {
                body_handle,
                collider_handle: *collider_handle,
                center: collider.world_center(body.position(), body.rotation()),
                rotation: body.rotation(),
                shape: collider.shape().clone(),
                bounding_radius: collider.shape().bounding_radius(),
                velocity: body.linear_velocity(),
                material: *collider.material(),
                is_sleeping,
            });
        }
    }

    states
}

fn sphere_sphere_pair(
    a: &ColliderState,
    radius_a: f32,
    b: &ColliderState,
    radius_b: f32,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> Vec<ContactConstraint> {
    let manifold = sphere_sphere_manifold(
        a.center, radius_a, b.center, radius_b, contact_margin,
    );

    if !manifold.is_empty() {
        return manifold_to_constraints(
            &manifold,
            Some(a.body_handle),
            b.body_handle,
            Some(a.collider_handle),
            Some(b.collider_handle),
            &a.material,
            &b.material,
        );
    }

    // Speculative CCD.
    if should_add_speculative(
        a.velocity.magnitude() * dt,
        radius_a,
        b.velocity.magnitude() * dt,
        radius_b,
        ccd_threshold,
        contact_margin,
        enable_speculative_contacts,
        speculative_min_speed,
        speculative_margin_multiplier,
    ) {
        let end_a = a.center + a.velocity * dt;
        let end_b = b.center + b.velocity * dt;
        if let Some(t) = swept_sphere_sphere(
            a.center,
            end_a,
            radius_a + contact_margin,
            b.center,
            end_b,
            radius_b + contact_margin,
        ) {
            let pos_a = a.center + (end_a - a.center) * t;
            let pos_b = b.center + (end_b - b.center) * t;
            let delta = pos_b - pos_a;
            let dist = delta.magnitude();
            let normal = if dist < 1e-6 {
                Vector3::y()
            } else {
                delta / dist
            };
            let actual_depth = (radius_a + radius_b) - dist;
            let solver_depth = actual_depth.max(0.0);
            let point = pos_a + normal * (radius_a - actual_depth * 0.5);
            let (restitution, friction) = ColliderMaterial::combine(&a.material, &b.material);
            return vec![ContactConstraint {
                body_a: Some(a.body_handle),
                body_b: b.body_handle,
                collider_a: Some(a.collider_handle),
                collider_b: Some(b.collider_handle),
                point,
                normal,
                raw_normal: normal,
                depth: solver_depth,
                raw_depth: actual_depth,
                restitution,
                friction,
                warm_normal_impulse: 0.0,
                warm_tangent_impulse: [0.0, 0.0],
            }];
        }
    }

    Vec::new()
}

fn sphere_box_pair(
    sphere: &ColliderState,
    radius: f32,
    box_state: &ColliderState,
    half_extents: Vector3<f32>,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb = Obb::new(box_state.center, box_state.rotation, half_extents);
    let manifold = sphere_obb_manifold(&obb, sphere.center, radius, contact_margin);
    // sphere_obb_manifold produces normal OBB→sphere. For body_a=box, body_b=sphere
    // the solver expects normal from A→B, which is OBB→sphere. Correct as-is.
    manifold_to_constraints(
        &manifold,
        Some(box_state.body_handle),
        sphere.body_handle,
        Some(box_state.collider_handle),
        Some(sphere.collider_handle),
        &box_state.material,
        &sphere.material,
    )
}

fn box_sphere_pair(
    box_state: &ColliderState,
    half_extents: Vector3<f32>,
    sphere: &ColliderState,
    radius: f32,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    sphere_box_pair(sphere, radius, box_state, half_extents, contact_margin)
}

fn box_box_pair(
    a: &ColliderState,
    he_a: Vector3<f32>,
    b: &ColliderState,
    he_b: Vector3<f32>,
    contact_margin: f32,
) -> Vec<ContactConstraint> {
    let obb_a = Obb::new(a.center, a.rotation, he_a);
    let obb_b = Obb::new(b.center, b.rotation, he_b);
    let manifold = obb_obb_manifold(&obb_a, &obb_b, contact_margin);
    manifold_to_constraints(
        &manifold,
        Some(a.body_handle),
        b.body_handle,
        Some(a.collider_handle),
        Some(b.collider_handle),
        &a.material,
        &b.material,
    )
}

/// Convert a `ContactManifold` from the collision library into solver `ContactConstraint`s.
fn manifold_to_constraints(
    manifold: &ContactManifold,
    body_a: Option<RigidBodyHandle>,
    body_b: RigidBodyHandle,
    collider_a: Option<ColliderHandle>,
    collider_b: Option<ColliderHandle>,
    material_a: &ColliderMaterial,
    material_b: &ColliderMaterial,
) -> Vec<ContactConstraint> {
    let (restitution, friction) = ColliderMaterial::combine(material_a, material_b);
    manifold
        .points
        .iter()
        .map(|cp| ContactConstraint {
            body_a,
            body_b,
            collider_a,
            collider_b,
            point: cp.point,
            normal: cp.normal,
            raw_normal: cp.raw_normal,
            depth: cp.depth,
            raw_depth: cp.raw_depth,
            restitution,
            friction,
            warm_normal_impulse: 0.0,
            warm_tangent_impulse: [0.0, 0.0],
        })
        .collect()
}

fn should_add_speculative(
    travel_a: f32,
    radius_a: f32,
    travel_b: f32,
    radius_b: f32,
    ccd_threshold: f32,
    contact_margin: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> bool {
    if !enable_speculative_contacts {
        return false;
    }
    let margin_gate = contact_margin * speculative_margin_multiplier;
    let fast_a = travel_a >= speculative_min_speed
        && travel_a > margin_gate
        && travel_a <= radius_a * ccd_threshold;
    let fast_b = travel_b >= speculative_min_speed
        && travel_b > margin_gate
        && travel_b <= radius_b * ccd_threshold;
    fast_a || fast_b
}
