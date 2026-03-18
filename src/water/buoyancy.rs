//! Pure buoyancy force calculation.
//!
//! Computes buoyancy, linear drag, and angular drag forces for rigid bodies
//! partially submerged in water. No ECS dependency — the ECS system in
//! `src/systems/buoyancy.rs` bridges between the physics world and these
//! functions.
//!
//! Returns forces (N) and torques (N·m), not impulses. The caller sets these
//! as persistent forces on the rigid body so they integrate per substep
//! alongside gravity.

use std::f32::consts::PI;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::{WaterGrid, WaveGrid};
use crate::physics::{
    ColliderShape, ForceContext, ForceOutput, RigidBodyHandle, SubstepForceProvider,
};

/// Water surface and floor level at a single (x, z) probe point.
pub struct WaterSample {
    /// Effective surface level (bulk level + wave displacement).
    pub surface_level: f32,
    /// Terrain floor that the water rests on.
    pub floor_level: f32,
}

/// Computed buoyancy and drag for a single body.
pub struct BuoyancyForces {
    /// Buoyancy force (N), opposite gravity. Applied at `buoyancy_center`.
    pub buoyancy_force: Vector3<f32>,
    /// World-space point at which buoyancy acts (submerged centroid).
    pub buoyancy_center: Point3<f32>,
    /// Linearized heave stiffness k = ρ g A_wp (N/m).
    pub heave_stiffness: f32,
    /// Quadratic linear drag base coefficient scaled by submerged fraction.
    pub quadratic_linear_drag_coeff: f32,
    /// Quadratic angular drag base coefficient scaled by submerged fraction.
    pub quadratic_angular_drag_coeff: f32,
    /// Fraction of the body volume that is submerged [0, 1].
    pub submerged_fraction: f32,
}

/// Target damping ratio for small vertical heave oscillations.
const HEAVE_DAMPING_RATIO: f32 = 0.1;
/// Drag coefficient used in Fd = 0.5 * rho * Cd * A * |v| * v.
const DRAG_COEFF_SPHERE: f32 = 0.47;
const DRAG_COEFF_BOX: f32 = 1.05;
const DRAG_COEFF_CAPSULE: f32 = 0.82;
/// Rotational drag multiplier in τd = 0.5 * rho * Cω * A * r² * |ω| * ω.
const ANGULAR_DRAG_COEFF: f32 = 2.0;
/// Vertical offset used for numerical dV/dy waterplane area estimation.
const WATERPLANE_EPSILON: f32 = 0.05;

/// Per-substep buoyancy force provider.
///
/// Borrows the water grids for the duration of the physics step and
/// recomputes buoyancy from each body's current position every substep,
/// eliminating the stale-force energy gain that occurs when forces are
/// frozen for the entire frame.
pub struct BuoyancyForceProvider<'a> {
    flow_grid: &'a WaterGrid,
    wave_grid: Option<&'a WaveGrid>,
    fluid_density: f32,
    affected: Vec<RigidBodyHandle>,
}

impl<'a> BuoyancyForceProvider<'a> {
    pub fn new(
        flow_grid: &'a WaterGrid,
        wave_grid: Option<&'a WaveGrid>,
        affected: Vec<RigidBodyHandle>,
    ) -> Self {
        Self {
            fluid_density: flow_grid.fluid_density(),
            flow_grid,
            wave_grid,
            affected,
        }
    }
}

impl SubstepForceProvider for BuoyancyForceProvider<'_> {
    fn affected_bodies(&self) -> &[RigidBodyHandle] {
        &self.affected
    }

    fn compute_force(&self, handle: RigidBodyHandle, ctx: &ForceContext) -> ForceOutput {
        let Some(body) = ctx.bodies.get(handle.0) else {
            return ForceOutput::zero();
        };
        if !body.is_dynamic() {
            return ForceOutput::zero();
        }

        let collider_handle = match body.colliders().first() {
            Some(h) => *h,
            None => return ForceOutput::zero(),
        };
        let Some(collider) = ctx.colliders.get(collider_handle.0) else {
            return ForceOutput::zero();
        };

        let collider_center = collider.world_center(body.position(), body.rotation());

        let result = match compute_buoyancy(
            collider_center,
            body.rotation(),
            collider.shape(),
            self.fluid_density,
            ctx.gravity,
            ctx.gravity_magnitude,
            self.flow_grid,
            self.wave_grid,
        ) {
            Some(f) => f,
            None => return ForceOutput::zero(),
        };

        // Decompose off-center buoyancy into central force + torque.
        let r = result.buoyancy_center - body.position();
        let buoyancy_torque = r.cross(&result.buoyancy_force);

        // Linearized heave damping target: c = 2 ζ sqrt(m k).
        let mass = body.mass().max(1e-4);
        let linear_drag_floor =
            2.0 * HEAVE_DAMPING_RATIO * (mass * result.heave_stiffness.max(0.0)).sqrt();

        // Quadratic drag from shape area: Fd = -k |v| v.
        let linear_speed = body.linear_velocity().magnitude();
        let angular_speed = body.angular_velocity().magnitude();

        ForceOutput {
            force: result.buoyancy_force,
            torque: buoyancy_torque,
            linear_drag_coeff: linear_drag_floor
                + result.quadratic_linear_drag_coeff * linear_speed,
            angular_drag_coeff: result.quadratic_angular_drag_coeff * angular_speed,
        }
    }
}

/// Sample the effective water surface at (x, z), combining the coarse flow
/// grid bulk level with fine wave displacement.
///
/// Returns `None` if the position is outside the grid or the cell is dry.
pub fn sample_water(
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
    x: f32,
    z: f32,
) -> Option<WaterSample> {
    let (i, j) = flow_grid.world_to_grid(x, z)?;
    let cell = flow_grid.cell(i, j);

    let bulk_level = if cell.volume > 0.0 {
        cell.surface_level(flow_grid.cell_area())
    } else {
        // Delegate to surface_level_at for ocean-coupled boundary cells.
        flow_grid.surface_level_at(x, z)?
    };

    let displacement = wave_grid
        .and_then(|wg| {
            let (wi, wj) = wg.world_to_wave(x, z)?;
            Some(wg.cell(wi, wj).displacement)
        })
        .unwrap_or(0.0);

    Some(WaterSample {
        surface_level: bulk_level + displacement,
        floor_level: cell.floor_level,
    })
}

/// Compute buoyancy force and drag coefficients for a body with the given shape.
///
/// Returns the buoyancy force (N) at the submerged centroid and drag
/// coefficients scaled by submersion fraction. The caller decomposes
/// the off-center buoyancy into a central force + torque, and sets
/// drag coefficients on the body for per-substep application.
///
/// Returns `None` if no part of the body is submerged.
pub fn compute_buoyancy(
    collider_center: Point3<f32>,
    body_rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    fluid_density: f32,
    gravity: Vector3<f32>,
    gravity_magnitude: f32,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
) -> Option<BuoyancyForces> {
    let (submerged_volume, submerged_fraction, buoyancy_center) = match shape {
        ColliderShape::Sphere { radius } => {
            compute_sphere_submersion(collider_center, *radius, flow_grid, wave_grid)?
        }
        ColliderShape::Box { half_extents } => compute_box_submersion(
            collider_center,
            body_rotation,
            *half_extents,
            flow_grid,
            wave_grid,
        )?,
        ColliderShape::Capsule {
            half_height,
            radius,
        } => compute_capsule_submersion(
            collider_center,
            body_rotation,
            *half_height,
            *radius,
            flow_grid,
            wave_grid,
        )?,
    };

    if submerged_volume <= 0.0 {
        return None;
    }

    // Buoyancy force: ρ_fluid × g × V_submerged opposite gravity.
    let up_dir = if gravity_magnitude > 1e-6 {
        -gravity / gravity_magnitude
    } else {
        Vector3::y()
    };
    let buoyancy_magnitude = fluid_density * gravity_magnitude * submerged_volume;
    let buoyancy_force = up_dir * buoyancy_magnitude;

    // Heave stiffness from numerical waterplane area estimate: k = rho * g * A_wp.
    let waterplane_area = estimate_waterplane_area(
        collider_center,
        body_rotation,
        shape,
        flow_grid,
        wave_grid,
        WATERPLANE_EPSILON,
    );
    let heave_stiffness = fluid_density * gravity_magnitude * waterplane_area;

    // Shape-based quadratic drag coefficient.
    let (shape_cd, mean_projected_area, angular_radius_sq) = shape_drag_properties(shape);
    let quadratic_linear_drag_coeff =
        0.5 * fluid_density * shape_cd * mean_projected_area * submerged_fraction;
    let quadratic_angular_drag_coeff = 0.5
        * fluid_density
        * ANGULAR_DRAG_COEFF
        * mean_projected_area
        * angular_radius_sq
        * submerged_fraction;

    Some(BuoyancyForces {
        buoyancy_force,
        buoyancy_center,
        heave_stiffness,
        quadratic_linear_drag_coeff,
        quadratic_angular_drag_coeff,
        submerged_fraction,
    })
}

/// Compute submerged volume and centroid for a sphere.
///
/// Uses the spherical cap formula for partial submersion.
fn compute_sphere_submersion(
    center: Point3<f32>,
    radius: f32,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = sample_water(flow_grid, wave_grid, center.x, center.z)?;

    // Floor check: sphere center must be above the floor to be affected.
    let bottom = center.y - radius;
    if bottom <= sample.floor_level {
        return None;
    }

    // Submerged depth: how far the sphere bottom is below the water surface.
    let depth = (sample.surface_level - bottom).clamp(0.0, 2.0 * radius);
    if depth <= 0.0 {
        return None;
    }

    // Spherical cap volume: V = π × d² × (3R - d) / 3
    let submerged_volume = PI * depth * depth * (3.0 * radius - depth) / 3.0;
    let full_volume = (4.0 / 3.0) * PI * radius * radius * radius;
    let submerged_fraction = (submerged_volume / full_volume).min(1.0);

    // Centroid of the submerged spherical cap, measured from sphere bottom:
    //   y_local = d × (4R - d) / (4 × (3R - d))
    let denom = 4.0 * (3.0 * radius - depth);
    let centroid_y = if denom.abs() > 1e-6 {
        bottom + depth * (4.0 * radius - depth) / denom
    } else {
        center.y
    };

    let buoyancy_center = Point3::new(center.x, centroid_y, center.z);
    Some((submerged_volume, submerged_fraction, buoyancy_center))
}

/// Compute submerged volume and centroid for a box using 4 bottom-face probes.
///
/// Each probe represents one quarter of the box volume. The submerged depth
/// at each corner determines the local buoyancy contribution, naturally
/// producing torque on tilted boxes.
fn compute_box_submersion(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = sample_water(flow_grid, wave_grid, center.x, center.z)?;
    let hx = half_extents.x;
    let hy = half_extents.y;
    let hz = half_extents.z;
    let full_volume = 8.0 * hx * hy * hz;

    let mut poly = build_obb_polyhedron(center, rotation, half_extents);
    // Water occupies the slab floor <= y <= surface.
    clip_polyhedron_with_plane(&mut poly, Vector3::new(0.0, 1.0, 0.0), sample.surface_level);
    clip_polyhedron_with_plane(&mut poly, Vector3::new(0.0, -1.0, 0.0), -sample.floor_level);

    let (submerged_volume, buoyancy_center) = polyhedron_volume_centroid(&poly)?;
    if submerged_volume <= 0.0 {
        return None;
    }
    let submerged_fraction = (submerged_volume / full_volume).min(1.0);
    Some((submerged_volume, submerged_fraction, buoyancy_center))
}

#[derive(Clone)]
struct Polyhedron {
    vertices: Vec<Point3<f32>>,
    faces: Vec<Vec<usize>>,
}

fn build_obb_polyhedron(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
) -> Polyhedron {
    let hx = half_extents.x;
    let hy = half_extents.y;
    let hz = half_extents.z;
    let local = [
        Vector3::new(-hx, -hy, -hz), // 0
        Vector3::new(hx, -hy, -hz),  // 1
        Vector3::new(hx, hy, -hz),   // 2
        Vector3::new(-hx, hy, -hz),  // 3
        Vector3::new(-hx, -hy, hz),  // 4
        Vector3::new(hx, -hy, hz),   // 5
        Vector3::new(hx, hy, hz),    // 6
        Vector3::new(-hx, hy, hz),   // 7
    ];
    let vertices = local
        .into_iter()
        .map(|v| center + rotation * v)
        .collect::<Vec<_>>();
    // Outward-oriented face loops.
    let faces = vec![
        vec![0, 1, 5, 4], // -Y
        vec![3, 7, 6, 2], // +Y
        vec![0, 3, 2, 1], // -Z
        vec![4, 5, 6, 7], // +Z
        vec![0, 4, 7, 3], // -X
        vec![1, 2, 6, 5], // +X
    ];
    Polyhedron { vertices, faces }
}

fn clip_polyhedron_with_plane(poly: &mut Polyhedron, normal: Vector3<f32>, h: f32) {
    const EPS: f32 = 1e-5;
    if poly.faces.is_empty() {
        return;
    }

    let mut new_vertices: Vec<Point3<f32>> = Vec::new();
    let mut new_faces: Vec<Vec<usize>> = Vec::new();
    let mut cap_points: Vec<Point3<f32>> = Vec::new();

    let add_vertex = |v: Point3<f32>, verts: &mut Vec<Point3<f32>>| -> usize {
        for (idx, existing) in verts.iter().enumerate() {
            if (existing - v).magnitude_squared() <= 1e-10 {
                return idx;
            }
        }
        verts.push(v);
        verts.len() - 1
    };

    for face in &poly.faces {
        if face.len() < 3 {
            continue;
        }
        let mut out_face: Vec<usize> = Vec::new();
        for edge_idx in 0..face.len() {
            let ia = face[edge_idx];
            let ib = face[(edge_idx + 1) % face.len()];
            let a = poly.vertices[ia];
            let b = poly.vertices[ib];
            let da = normal.dot(&a.coords) - h;
            let db = normal.dot(&b.coords) - h;
            let a_inside = da <= EPS;
            let b_inside = db <= EPS;

            if a_inside {
                let idx = add_vertex(a, &mut new_vertices);
                if out_face.last().copied() != Some(idx) {
                    out_face.push(idx);
                }
            }

            if a_inside ^ b_inside {
                let t = da / (da - db);
                let p = Point3::from(a.coords + (b.coords - a.coords) * t);
                let idx = add_vertex(p, &mut new_vertices);
                if out_face.last().copied() != Some(idx) {
                    out_face.push(idx);
                }
                cap_points.push(p);
            }
        }
        if out_face.len() >= 3 {
            // Remove duplicated closing vertex if present.
            if out_face.first() == out_face.last() {
                out_face.pop();
            }
            if out_face.len() >= 3 {
                new_faces.push(out_face);
            }
        }
    }

    // Add clipping cap face (outward normal points toward removed half-space).
    let mut unique_cap: Vec<Point3<f32>> = Vec::new();
    for p in cap_points {
        if !unique_cap
            .iter()
            .any(|q| (*q - p).magnitude_squared() <= 1e-10)
        {
            unique_cap.push(p);
        }
    }
    if unique_cap.len() >= 3 {
        let cap_center = unique_cap
            .iter()
            .fold(Vector3::zeros(), |acc, p| acc + p.coords)
            / unique_cap.len() as f32;
        let n = normal.normalize();
        let tangent_seed = if n.x.abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        let u = (tangent_seed - n * tangent_seed.dot(&n)).normalize();
        let v = n.cross(&u);
        unique_cap.sort_by(|a, b| {
            let da = a.coords - cap_center;
            let db = b.coords - cap_center;
            let aa = da.dot(&v).atan2(da.dot(&u));
            let bb = db.dot(&v).atan2(db.dot(&u));
            aa.total_cmp(&bb)
        });
        let mut cap_face: Vec<usize> = Vec::with_capacity(unique_cap.len());
        for p in unique_cap {
            let idx = add_vertex(p, &mut new_vertices);
            cap_face.push(idx);
        }
        if cap_face.len() >= 3 {
            new_faces.push(cap_face);
        }
    }

    poly.vertices = new_vertices;
    poly.faces = new_faces;
}

fn polyhedron_volume_centroid(poly: &Polyhedron) -> Option<(f32, Point3<f32>)> {
    if poly.faces.is_empty() || poly.vertices.is_empty() {
        return None;
    }
    let mut signed_volume = 0.0f32;
    let mut centroid_acc = Vector3::zeros();

    for face in &poly.faces {
        if face.len() < 3 {
            continue;
        }
        let a = poly.vertices[face[0]].coords;
        for i in 1..(face.len() - 1) {
            let b = poly.vertices[face[i]].coords;
            let c = poly.vertices[face[i + 1]].coords;
            let tet_volume = a.dot(&b.cross(&c)) / 6.0;
            let tet_centroid = (a + b + c) * 0.25;
            signed_volume += tet_volume;
            centroid_acc += tet_centroid * tet_volume;
        }
    }

    if signed_volume.abs() <= 1e-8 {
        return None;
    }
    let center = centroid_acc / signed_volume;
    Some((signed_volume.abs(), Point3::from(center)))
}

/// Compute submerged volume and centroid for a capsule using 2 hemisphere probes.
///
/// Probes are placed at the bottom and top hemisphere centers (along local Y).
fn compute_capsule_submersion(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_height: f32,
    radius: f32,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
) -> Option<(f32, f32, Point3<f32>)> {
    let cylinder_half = half_height - radius;
    let full_height = 2.0 * half_height;

    // Full capsule volume = cylinder + two hemispheres (= one sphere).
    let cyl_volume = PI * radius * radius * (2.0 * cylinder_half);
    let sphere_volume = (4.0 / 3.0) * PI * radius * radius * radius;
    let full_volume = cyl_volume + sphere_volume;

    // Probe positions: bottom and top hemisphere centers in local space.
    let probes_local = [
        Vector3::new(0.0, -cylinder_half, 0.0),
        Vector3::new(0.0, cylinder_half, 0.0),
    ];

    let mut total_submerged_volume = 0.0f32;
    let mut weighted_pos = Vector3::zeros();
    let mut any_submerged = false;

    for local in &probes_local {
        let world = center + rotation * local;
        let sample = match sample_water(flow_grid, wave_grid, world.x, world.z) {
            Some(s) => s,
            None => continue,
        };

        // The probe's effective bottom is one radius below the hemisphere center.
        let probe_bottom = world.y - radius;
        if probe_bottom <= sample.floor_level {
            continue;
        }

        let depth = (sample.surface_level - probe_bottom).clamp(0.0, full_height);
        if depth <= 0.0 {
            continue;
        }

        any_submerged = true;
        let column_volume = (depth / full_height) * (full_volume / 2.0);
        total_submerged_volume += column_volume;

        let centroid_y = probe_bottom + depth * 0.5;
        weighted_pos += Vector3::new(world.x, centroid_y, world.z) * column_volume;
    }

    if !any_submerged || total_submerged_volume <= 0.0 {
        return None;
    }

    let submerged_fraction = (total_submerged_volume / full_volume).min(1.0);
    let buoyancy_center = Point3::from(weighted_pos / total_submerged_volume);
    Some((total_submerged_volume, submerged_fraction, buoyancy_center))
}

fn submerged_volume_for_pose(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
) -> f32 {
    match shape {
        ColliderShape::Sphere { radius } => {
            compute_sphere_submersion(center, *radius, flow_grid, wave_grid)
                .map(|(v, _, _)| v)
                .unwrap_or(0.0)
        }
        ColliderShape::Box { half_extents } => {
            compute_box_submersion(center, rotation, *half_extents, flow_grid, wave_grid)
                .map(|(v, _, _)| v)
                .unwrap_or(0.0)
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => compute_capsule_submersion(
            center,
            rotation,
            *half_height,
            *radius,
            flow_grid,
            wave_grid,
        )
        .map(|(v, _, _)| v)
        .unwrap_or(0.0),
    }
}

fn estimate_waterplane_area(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    flow_grid: &WaterGrid,
    wave_grid: Option<&WaveGrid>,
    epsilon: f32,
) -> f32 {
    if epsilon <= 1e-6 {
        return 0.0;
    }
    let center_up = Point3::new(center.x, center.y + epsilon, center.z);
    let center_down = Point3::new(center.x, center.y - epsilon, center.z);
    let volume_up = submerged_volume_for_pose(center_up, rotation, shape, flow_grid, wave_grid);
    let volume_down = submerged_volume_for_pose(center_down, rotation, shape, flow_grid, wave_grid);
    ((volume_down - volume_up) / (2.0 * epsilon)).max(0.0)
}

fn shape_drag_properties(shape: &ColliderShape) -> (f32, f32, f32) {
    match shape {
        ColliderShape::Sphere { radius } => {
            let r = *radius;
            let projected_area = PI * r * r;
            (DRAG_COEFF_SPHERE, projected_area, r * r)
        }
        ColliderShape::Box { half_extents } => {
            let hx = half_extents.x;
            let hy = half_extents.y;
            let hz = half_extents.z;
            let projected_area = 2.0 * (hx * hy + hy * hz + hx * hz);
            let radius_sq = hx * hx + hy * hy + hz * hz;
            (DRAG_COEFF_BOX, projected_area, radius_sq)
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let r = *radius;
            let cyl_h = (2.0 * (half_height - r)).max(0.0);
            let surface_area = 2.0 * PI * r * cyl_h + 4.0 * PI * r * r;
            let projected_area = surface_area * 0.25;
            let radius_sq = half_height * half_height;
            (DRAG_COEFF_CAPSULE, projected_area, radius_sq)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::{WaterGridConfig, WaterProperties};

    fn make_test_grid(surface_level: f32) -> WaterGrid {
        let config = WaterGridConfig {
            cell_size: 10.0,
            dims: (3, 3),
            origin: Vector3::new(-15.0, 0.0, -15.0),
            ocean_level: None,
        };
        let mut grid = WaterGrid::new(config, &WaterProperties::default());
        // Fill all cells with water up to surface_level on a flat floor at y=0.
        for j in 0..3 {
            for i in 0..3 {
                let volume = surface_level * grid.cell_area();
                grid.add_water(i, j, volume, 0.0);
            }
        }
        grid
    }

    #[test]
    fn sphere_fully_submerged() {
        let grid = make_test_grid(10.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let radius = 1.0;

        let (vol, frac, _centroid) =
            compute_sphere_submersion(center, radius, &grid, None).unwrap();

        let full_vol = (4.0 / 3.0) * PI * radius.powi(3);
        assert!(
            (frac - 1.0).abs() < 0.01,
            "Should be fully submerged, got fraction={frac}"
        );
        assert!(
            (vol - full_vol).abs() < 0.01,
            "Submerged volume should match full sphere, got {vol} expected {full_vol}"
        );
    }

    #[test]
    fn sphere_half_submerged() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let radius = 1.0;

        let (_vol, frac, _centroid) =
            compute_sphere_submersion(center, radius, &grid, None).unwrap();

        // Bottom of sphere at y=4, water at y=5 → depth=1 out of diameter=2 → ~half
        assert!(
            (frac - 0.5).abs() < 0.1,
            "Should be ~half submerged, got fraction={frac}"
        );
    }

    #[test]
    fn sphere_above_water() {
        let grid = make_test_grid(2.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let radius = 1.0;

        let result = compute_sphere_submersion(center, radius, &grid, None);
        assert!(result.is_none(), "Sphere above water should return None");
    }

    #[test]
    fn box_partially_submerged() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_extents = Vector3::new(0.5, 0.5, 0.5);

        let (vol, frac, _centroid) = compute_box_submersion(
            center,
            UnitQuaternion::identity(),
            half_extents,
            &grid,
            None,
        )
        .unwrap();

        // Bottom face at y=4.5, water at y=5 → depth=0.5 out of height=1.0
        let expected_frac = 0.5;
        assert!(
            (frac - expected_frac).abs() < 0.1,
            "Should be ~half submerged, got fraction={frac}"
        );
        assert!(vol > 0.0, "Submerged volume should be positive");
    }

    #[test]
    fn cube_submersion_is_orientation_invariant() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_extents = Vector3::new(0.5, 0.5, 0.5);

        let (vol_identity, frac_identity, _) = compute_box_submersion(
            center,
            UnitQuaternion::identity(),
            half_extents,
            &grid,
            None,
        )
        .unwrap();
        let (vol_turned, frac_turned, _) = compute_box_submersion(
            center,
            UnitQuaternion::from_euler_angles(PI * 0.5, 0.0, 0.0),
            half_extents,
            &grid,
            None,
        )
        .unwrap();

        assert!(
            (vol_identity - vol_turned).abs() < 1e-3,
            "Cube submerged volume should be orientation invariant, got {vol_identity} vs {vol_turned}"
        );
        assert!(
            (frac_identity - frac_turned).abs() < 1e-3,
            "Cube submerged fraction should be orientation invariant, got {frac_identity} vs {frac_turned}"
        );
    }

    #[test]
    fn buoyancy_produces_upward_force() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);

        let forces = compute_buoyancy(
            center,
            UnitQuaternion::identity(),
            &ColliderShape::Sphere { radius: 1.0 },
            1000.0,
            Vector3::new(0.0, -9.81, 0.0),
            9.81,
            &grid,
            None,
        )
        .unwrap();

        assert!(
            forces.buoyancy_force.y > 0.0,
            "Buoyancy should push upward, got y={}",
            forces.buoyancy_force.y
        );
    }

    #[test]
    fn submerged_body_has_positive_drag_coefficients() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);

        let forces = compute_buoyancy(
            center,
            UnitQuaternion::identity(),
            &ColliderShape::Sphere { radius: 1.0 },
            1000.0,
            Vector3::new(0.0, -9.81, 0.0),
            9.81,
            &grid,
            None,
        )
        .unwrap();

        // Heave stiffness is non-zero only when the waterplane cuts the body.
        assert!(
            forces.heave_stiffness > 0.0,
            "Submerged body should have positive heave stiffness, got {}",
            forces.heave_stiffness
        );
        assert!(
            forces.quadratic_linear_drag_coeff > 0.0,
            "Submerged body should have positive quadratic linear drag, got {}",
            forces.quadratic_linear_drag_coeff
        );
        assert!(
            forces.quadratic_angular_drag_coeff > 0.0,
            "Submerged body should have positive quadratic angular drag, got {}",
            forces.quadratic_angular_drag_coeff
        );
    }

    #[test]
    fn floor_level_prevents_false_underwater() {
        // Sky island scenario: water floor is at y=10, body is at y=5 (below the island).
        let config = WaterGridConfig {
            cell_size: 10.0,
            dims: (3, 3),
            origin: Vector3::new(-15.0, 0.0, -15.0),
            ocean_level: None,
        };
        let mut grid = WaterGrid::new(config, &WaterProperties::default());
        for j in 0..3 {
            for i in 0..3 {
                // Water sitting on a sky island floor at y=10, surface at y=12.
                grid.add_water(i, j, 2.0 * grid.cell_area(), 10.0);
            }
        }

        let center = Point3::new(0.0, 5.0, 0.0);
        let result = compute_sphere_submersion(center, 1.0, &grid, None);
        assert!(
            result.is_none(),
            "Body below sky island floor should not be submerged"
        );
    }

    #[test]
    fn persistent_force_integrates_per_substep() {
        // Verify that a persistent buoyancy force, when integrated over
        // multiple substeps, produces the same total velocity change as
        // gravity does (for a neutrally buoyant body).
        use crate::physics::{RigidBody, RigidBodyDesc};

        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        body.set_mass_properties(1.0, nalgebra::Matrix3::identity());

        // Buoyancy force that exactly cancels gravity (neutrally buoyant).
        let gravity = Vector3::new(0.0, -9.81, 0.0);
        body.set_force(Vector3::new(0.0, 9.81, 0.0));

        // Integrate 4 substeps at 1/240s each.
        for _ in 0..4 {
            body.integrate_forces(1.0 / 240.0, gravity);
        }

        // Vertical velocity should be near zero (gravity and buoyancy cancel).
        assert!(
            body.linear_velocity().y.abs() < 0.001,
            "Neutrally buoyant body should have ~zero vertical velocity, got {}",
            body.linear_velocity().y
        );
    }
}
