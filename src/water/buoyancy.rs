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

use crate::physics::{
    ColliderShape, ForceContext, ForceOutput, PhysicsWorld, RigidBodyHandle, SubstepForceProvider,
};

/// Water surface and floor level at a single probe point.
pub struct WaterSample {
    /// Effective surface level (bulk level + swell + ripples).
    pub surface_level: f32,
    /// Terrain floor that the water rests on.
    pub floor_level: f32,
    /// Velocity of the water: a river's flow, or a lake's current towards
    /// its outlet. Drag acts on a body relative to it.
    pub velocity: Vector3<f32>,
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
/// 0.3 gives ~2-4 bobs before settling — enough to look natural without
/// sustaining perpetual oscillation via wave coupling feedback.
const HEAVE_DAMPING_RATIO: f32 = 0.3;
/// Drag coefficient used in Fd = 0.5 * rho * Cd * A * |v| * v.
const DRAG_COEFF_SPHERE: f32 = 0.47;
const DRAG_COEFF_BOX: f32 = 1.05;
const DRAG_COEFF_CAPSULE: f32 = 0.82;
/// Rotational drag multiplier in τd = 0.5 * rho * Cω * A * r² * |ω| * ω.
const ANGULAR_DRAG_COEFF: f32 = 2.0;
/// Angular damping floor (N·m·s/rad) per unit submerged fraction.
/// Provides linear angular drag that settles low-amplitude rocking where
/// the quadratic term (∝ |ω|·ω) is too weak to converge.
const ANGULAR_DRAG_FLOOR: f32 = 0.5;
/// Vertical offset used for numerical dV/dy waterplane area estimation.
const WATERPLANE_EPSILON: f32 = 0.05;
/// Speed through the water, m/s, below which a body has no flow direction and
/// its drag is taken over its mean projected area. The quadratic drag scales
/// with the speed, so which area stands in there makes no difference.
const FLOW_SPEED_EPSILON: f32 = 1e-3;

/// Per-substep buoyancy force provider.
///
/// Borrows the water for the duration of the physics step and recomputes
/// buoyancy from each body's current position every substep, eliminating the
/// stale-force energy gain that occurs when forces are frozen for the entire
/// frame.
pub struct BuoyancyForceProvider<'a> {
    water: &'a (dyn WaterSurface + Sync),
    fluid_density: f32,
    affected: Vec<RigidBodyHandle>,
}

impl<'a> BuoyancyForceProvider<'a> {
    pub fn new(water: &'a (dyn WaterSurface + Sync), affected: Vec<RigidBodyHandle>) -> Self {
        Self {
            water,
            fluid_density: FLUID_DENSITY,
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

        let body_pos = body.position();
        let body_rot = body.rotation();

        // Drag acts relative to the water, so a current carries a floating
        // body with it: −c·(v − u) is −c·v, which the engine applies, plus
        // c·u, added below.
        let water_velocity = self
            .water
            .sample(body_pos)
            .map_or(Vector3::zeros(), |s| s.velocity);
        let relative_velocity = body.linear_velocity() - water_velocity;
        let linear_speed = relative_velocity.magnitude();
        let flow = (linear_speed > FLOW_SPEED_EPSILON).then(|| relative_velocity / linear_speed);

        // Accumulate contributions from every part of the body's envelope, so
        // compound bodies (bridges, fracturable structures) get their whole
        // buoyancy, and a body whose bulk is not its colliders floats as its
        // bulk does.
        let mut total_force = Vector3::zeros();
        let mut total_torque = Vector3::zeros();
        let mut total_heave_stiffness = 0.0f32;
        let mut total_linear_drag_coeff = 0.0f32;
        let mut total_angular_drag_coeff = 0.0f32;
        let mut total_submerged_fraction = 0.0f32;
        let mut part_count = 0u32;

        for part in body.envelope(ctx.colliders) {
            let part_xform = part.world_transform(body_pos, body_rot);
            let part_center = Point3::from(part_xform.translation.vector);
            let part_rot = part_xform.rotation;

            let result = match compute_buoyancy(
                part_center,
                part_rot,
                part.shape,
                self.fluid_density,
                ctx.gravity,
                ctx.gravity_magnitude,
                flow,
                self.water,
            ) {
                Some(f) => f,
                None => continue,
            };

            let r = result.buoyancy_center - body_pos;
            total_force += result.buoyancy_force;
            total_torque += r.cross(&result.buoyancy_force);
            total_heave_stiffness += result.heave_stiffness;
            total_linear_drag_coeff += result.quadratic_linear_drag_coeff;
            total_angular_drag_coeff += result.quadratic_angular_drag_coeff;
            total_submerged_fraction += result.submerged_fraction;
            part_count += 1;
        }

        if part_count == 0 {
            return ForceOutput::zero();
        }

        let avg_submerged_fraction = total_submerged_fraction / part_count as f32;

        // Linearized heave damping target: c = 2 ζ sqrt(m k).
        let mass = body.mass().max(1e-4);
        let linear_drag_floor =
            2.0 * HEAVE_DAMPING_RATIO * (mass * total_heave_stiffness.max(0.0)).sqrt();

        // Quadratic drag from shape area: Fd = -k |v - u| (v - u).
        let angular_speed = body.angular_velocity().magnitude();

        // Angular drag floor: linear term that guarantees rocking settles
        // even at low angular speeds where the quadratic term vanishes.
        let angular_drag_floor = ANGULAR_DRAG_FLOOR * avg_submerged_fraction;

        let linear_drag_coeff = linear_drag_floor + total_linear_drag_coeff * linear_speed;
        ForceOutput {
            force: total_force + water_velocity * linear_drag_coeff,
            torque: total_torque,
            linear_drag_coeff,
            angular_drag_coeff: angular_drag_floor + total_angular_drag_coeff * angular_speed,
        }
    }
}

/// Water as buoyancy sees it: the surface and floor over a point.
pub trait WaterSurface {
    /// The water at a point, or `None` where there is none.
    fn sample(&self, point: Point3<f32>) -> Option<WaterSample>;
}

/// Still water standing at `surface` over a flat floor at `floor`,
/// everywhere: a pool to float something in and see how it lies.
pub struct StillWater {
    pub surface: f32,
    pub floor: f32,
}

impl WaterSurface for StillWater {
    fn sample(&self, _point: Point3<f32>) -> Option<WaterSample> {
        Some(WaterSample {
            surface_level: self.surface,
            floor_level: self.floor,
            velocity: Vector3::zeros(),
        })
    }
}

/// Upward buoyancy on a body's whole envelope, in newtons, as it stands in
/// `water`, with Earth's gravity.
pub fn lift(world: &PhysicsWorld, body: RigidBodyHandle, water: &dyn WaterSurface) -> f32 {
    const GRAVITY: f32 = 9.81;
    let Some(state) = world.body(body) else {
        return 0.0;
    };
    world
        .envelope(body)
        .filter_map(|part| {
            let pose = part.world_transform(state.position(), state.rotation());
            compute_buoyancy(
                Point3::from(pose.translation.vector),
                pose.rotation,
                part.shape,
                FLUID_DENSITY,
                Vector3::new(0.0, -GRAVITY, 0.0),
                GRAVITY,
                None,
                water,
            )
        })
        .map(|forces| forces.buoyancy_force.y)
        .sum()
}

/// Density of water, kg/m³.
pub const FLUID_DENSITY: f32 = 1000.0;

/// Compute buoyancy force and drag coefficients for a body with the given shape.
///
/// Returns the buoyancy force (N) at the submerged centroid and drag
/// coefficients scaled by submersion fraction. The caller decomposes
/// the off-center buoyancy into a central force + torque, and sets
/// drag coefficients on the body for per-substep application.
///
/// `flow` is the world-space direction the body moves through the water, a
/// unit vector; the linear drag is taken over the area the shape presents
/// along it. `None` takes it over the mean projected area.
///
/// Returns `None` if no part of the body is submerged.
#[allow(clippy::too_many_arguments)]
pub fn compute_buoyancy(
    collider_center: Point3<f32>,
    body_rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    fluid_density: f32,
    gravity: Vector3<f32>,
    gravity_magnitude: f32,
    flow: Option<Vector3<f32>>,
    water: &dyn WaterSurface,
) -> Option<BuoyancyForces> {
    let (submerged_volume, submerged_fraction, buoyancy_center) = match shape {
        ColliderShape::Sphere { radius } => {
            compute_sphere_submersion(collider_center, *radius, water)?
        }
        ColliderShape::Box { half_extents } => {
            compute_box_submersion(collider_center, body_rotation, *half_extents, water)?
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => compute_capsule_submersion(
            collider_center,
            body_rotation,
            *half_height,
            *radius,
            water,
        )?,
        ColliderShape::ConvexHull { hull } => {
            compute_hull_submersion(collider_center, body_rotation, hull, water)?
        }
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
        water,
        WATERPLANE_EPSILON,
    );
    let heave_stiffness = fluid_density * gravity_magnitude * waterplane_area;

    // Shape-based quadratic drag coefficient.
    let (shape_cd, mean_projected_area, angular_radius_sq) = shape_drag_properties(shape);
    let frontal_area = flow.map_or(mean_projected_area, |direction| {
        shape.projected_area(body_rotation.inverse() * direction)
    });
    let quadratic_linear_drag_coeff =
        0.5 * fluid_density * shape_cd * frontal_area * submerged_fraction;
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
    water: &dyn WaterSurface,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = water.sample(center)?;

    // Floor check: reject bodies whose center is at or below the terrain
    // floor. This prevents false buoyancy for bodies underneath floating
    // water (sky islands). The bottom of the shape may touch the floor
    // while the body is legitimately in the water column.
    let bottom = center.y - radius;
    if center.y <= sample.floor_level {
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
    water: &dyn WaterSurface,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = water.sample(center)?;
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

/// Compute submerged volume and centroid for a convex hull.
///
/// Transforms hull vertices into world space, builds a closed polyhedron, and
/// clips against the water surface and floor planes — same approach as OBB
/// submersion but with arbitrary face topology.
fn compute_hull_submersion(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    hull: &crate::collision::convex_hull::ConvexHull,
    water: &dyn WaterSurface,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = water.sample(center)?;

    if center.y <= sample.floor_level {
        return None;
    }

    let full_volume = hull.compute_volume();
    if full_volume <= 1e-10 {
        return None;
    }

    let vertices: Vec<Point3<f32>> = hull
        .vertices
        .iter()
        .map(|v| center + rotation * v)
        .collect();

    let faces: Vec<Vec<usize>> = hull
        .faces
        .iter()
        .map(|f| f.vertex_indices.iter().map(|&i| i as usize).collect())
        .collect();

    let mut poly = Polyhedron { vertices, faces };
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

/// Compute submerged volume and centroid for a capsule analytically.
///
/// Uses a single water sample at the capsule's XZ center and closed-form
/// volume/centroid integrals over three zones: bottom hemisphere cap,
/// cylinder, and top hemisphere cap. Exact for vertical capsules and a
/// close approximation for tilted ones.
fn compute_capsule_submersion(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    half_height: f32,
    radius: f32,
    water: &dyn WaterSurface,
) -> Option<(f32, f32, Point3<f32>)> {
    let sample = water.sample(center)?;

    let r = radius;
    let cylinder_half = half_height - r;
    let cyl_len = 2.0 * cylinder_half;
    let total_height = 2.0 * half_height;

    // Full capsule volume = cylinder + sphere (two hemispheres).
    let full_volume = PI * r * r * cyl_len + (4.0 / 3.0) * PI * r * r * r;

    // Capsule axis in world space. Hemisphere centers along this axis.
    let axis = rotation * Vector3::y();
    let p_low_y = (center.y - axis.y * cylinder_half).min(center.y + axis.y * cylinder_half);
    let y_min = p_low_y - r;
    let vertical_span = 2.0 * (r + axis.y.abs() * cylinder_half);
    // Floor check: reject capsules whose center is at or below the terrain
    // floor. A tall capsule's bottom may reach the floor while the body is
    // legitimately in the water column.
    if center.y <= sample.floor_level {
        return None;
    }

    let depth_vertical = (sample.surface_level - y_min).clamp(0.0, vertical_span);
    if depth_vertical <= 0.0 || vertical_span <= 1e-6 {
        return None;
    }
    // The closed-form phase integrals are defined along the capsule's local
    // axis depth [0, 2*half_height]. Convert vertical immersion depth into
    // equivalent axis depth so tilted capsules preserve displaced volume.
    let depth = (depth_vertical * total_height / vertical_span).clamp(0.0, total_height);

    // Three-phase volume and centroid (measured from capsule bottom = y_min).
    //   Phase 1: bottom hemisphere [0, r]
    //   Phase 2: cylinder          [r, r + cyl_len]
    //   Phase 3: top hemisphere    [r + cyl_len, total_height]
    let (submerged_volume, centroid_from_bottom) = if depth <= r {
        // Phase 1: spherical cap only.
        let v = PI * depth * depth * (3.0 * r - depth) / 3.0;
        let denom = 4.0 * (3.0 * r - depth);
        let cy = if denom.abs() > 1e-6 {
            depth * (4.0 * r - depth) / denom
        } else {
            depth * 0.5
        };
        (v, cy)
    } else if depth <= r + cyl_len {
        // Phase 2: full bottom hemisphere + partial cylinder.
        let hemi_vol = (2.0 / 3.0) * PI * r * r * r;
        let hemi_centroid = 3.0 * r / 8.0;
        let cyl_h = depth - r;
        let cyl_vol = PI * r * r * cyl_h;
        let cyl_centroid = r + cyl_h * 0.5;
        let v = hemi_vol + cyl_vol;
        let cy = (hemi_vol * hemi_centroid + cyl_vol * cyl_centroid) / v;
        (v, cy)
    } else {
        // Phase 3: full volume minus dry top cap.
        let cap_remaining = total_height - depth;
        let dry_vol = PI * cap_remaining * cap_remaining * (3.0 * r - cap_remaining) / 3.0;
        let v = full_volume - dry_vol;

        // Centroid via complement: V*cy = V_full*cy_full - V_dry*cy_dry.
        let full_centroid = total_height * 0.5;
        let dry_denom = 4.0 * (3.0 * r - cap_remaining);
        let dry_centroid_from_top = if dry_denom.abs() > 1e-6 {
            cap_remaining * (4.0 * r - cap_remaining) / dry_denom
        } else {
            cap_remaining * 0.5
        };
        let dry_centroid = total_height - dry_centroid_from_top;
        let cy = (full_volume * full_centroid - dry_vol * dry_centroid) / v;
        (v, cy)
    };

    if submerged_volume <= 0.0 {
        return None;
    }

    let submerged_fraction = (submerged_volume / full_volume).min(1.0);
    let centroid_y = y_min + centroid_from_bottom * (vertical_span / total_height);

    // For tilted capsules, offset the buoyancy center along the axis so it
    // sits at the XZ position corresponding to centroid_y on the capsule axis.
    // This produces a natural righting torque.
    let buoyancy_center = if axis.y.abs() > 1e-3 {
        let t = ((centroid_y - center.y) / axis.y).clamp(-half_height, half_height);
        let axis_point = center + axis * t;
        Point3::new(axis_point.x, centroid_y, axis_point.z)
    } else {
        Point3::new(center.x, centroid_y, center.z)
    };

    Some((submerged_volume, submerged_fraction, buoyancy_center))
}

fn submerged_volume_for_pose(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    water: &dyn WaterSurface,
) -> f32 {
    match shape {
        ColliderShape::Sphere { radius } => compute_sphere_submersion(center, *radius, water)
            .map(|(v, _, _)| v)
            .unwrap_or(0.0),
        ColliderShape::Box { half_extents } => {
            compute_box_submersion(center, rotation, *half_extents, water)
                .map(|(v, _, _)| v)
                .unwrap_or(0.0)
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => compute_capsule_submersion(center, rotation, *half_height, *radius, water)
            .map(|(v, _, _)| v)
            .unwrap_or(0.0),
        ColliderShape::ConvexHull { hull } => {
            compute_hull_submersion(center, rotation, hull, water)
                .map(|(v, _, _)| v)
                .unwrap_or(0.0)
        }
    }
}

fn estimate_waterplane_area(
    center: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    shape: &ColliderShape,
    water: &dyn WaterSurface,
    epsilon: f32,
) -> f32 {
    if epsilon <= 1e-6 {
        return 0.0;
    }
    let center_up = Point3::new(center.x, center.y + epsilon, center.z);
    let center_down = Point3::new(center.x, center.y - epsilon, center.z);
    let volume_up = submerged_volume_for_pose(center_up, rotation, shape, water);
    let volume_down = submerged_volume_for_pose(center_down, rotation, shape, water);
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
        ColliderShape::ConvexHull { hull } => {
            let (mut min_v, mut max_v) = (
                Vector3::from_element(f32::MAX),
                Vector3::from_element(f32::MIN),
            );
            for v in &hull.vertices {
                min_v = min_v.zip_map(v, f32::min);
                max_v = max_v.zip_map(v, f32::max);
            }
            let ext = max_v - min_v;
            // Mean projected area from three AABB face areas.
            let projected_area = (ext.x * ext.y + ext.y * ext.z + ext.x * ext.z) / 3.0;
            let radius_sq = hull.bounding_radius * hull.bounding_radius;
            (DRAG_COEFF_BOX, projected_area, radius_sq)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_grid(surface_level: f32) -> StillWater {
        StillWater {
            surface: surface_level,
            floor: 0.0,
        }
    }

    #[test]
    fn sphere_fully_submerged() {
        let grid = make_test_grid(10.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let radius = 1.0;

        let (vol, frac, _centroid) = compute_sphere_submersion(center, radius, &grid).unwrap();

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

        let (_vol, frac, _centroid) = compute_sphere_submersion(center, radius, &grid).unwrap();

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

        let result = compute_sphere_submersion(center, radius, &grid);
        assert!(result.is_none(), "Sphere above water should return None");
    }

    #[test]
    fn box_partially_submerged() {
        let grid = make_test_grid(5.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_extents = Vector3::new(0.5, 0.5, 0.5);

        let (vol, frac, _centroid) =
            compute_box_submersion(center, UnitQuaternion::identity(), half_extents, &grid)
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

        let (vol_identity, frac_identity, _) =
            compute_box_submersion(center, UnitQuaternion::identity(), half_extents, &grid)
                .unwrap();
        let (vol_turned, frac_turned, _) = compute_box_submersion(
            center,
            UnitQuaternion::from_euler_angles(PI * 0.5, 0.0, 0.0),
            half_extents,
            &grid,
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
            None,
            &grid,
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
            None,
            &grid,
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
        // Sky island scenario: water on a floor at y=10, surface at y=12, and
        // a body at y=5, below the island. A surface that reads this column's
        // water for the point is what a single-layer water would give.
        let grid = StillWater {
            surface: 12.0,
            floor: 10.0,
        };

        let center = Point3::new(0.0, 5.0, 0.0);
        let result = compute_sphere_submersion(center, 1.0, &grid);
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

    #[test]
    fn capsule_fully_submerged() {
        let grid = make_test_grid(10.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_height = 1.0;
        let radius = 0.5;

        let (vol, frac, centroid) = compute_capsule_submersion(
            center,
            UnitQuaternion::identity(),
            half_height,
            radius,
            &grid,
        )
        .unwrap();

        let cyl_len = 2.0 * (half_height - radius);
        let expected_vol = PI * radius * radius * cyl_len + (4.0 / 3.0) * PI * radius.powi(3);
        assert!(
            (frac - 1.0).abs() < 0.01,
            "Should be fully submerged, got fraction={frac}"
        );
        assert!(
            (vol - expected_vol).abs() < 0.01,
            "Volume should match full capsule, got {vol} expected {expected_vol}"
        );
        assert!(
            (centroid.y - center.y).abs() < 0.01,
            "Fully submerged centroid should be at center, got y={} expected y={}",
            centroid.y,
            center.y
        );
    }

    #[test]
    fn capsule_half_submerged() {
        let grid = make_test_grid(5.0);
        // Center at water surface → bottom half submerged.
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_height = 1.0;
        let radius = 0.5;

        let (_vol, frac, centroid) = compute_capsule_submersion(
            center,
            UnitQuaternion::identity(),
            half_height,
            radius,
            &grid,
        )
        .unwrap();

        assert!(
            (frac - 0.5).abs() < 0.05,
            "Should be ~half submerged, got fraction={frac}"
        );
        assert!(
            centroid.y < center.y,
            "Centroid should be below center, got y={} vs center y={}",
            centroid.y,
            center.y
        );
    }

    #[test]
    fn capsule_above_water() {
        let grid = make_test_grid(2.0);
        let center = Point3::new(0.0, 5.0, 0.0);

        let result =
            compute_capsule_submersion(center, UnitQuaternion::identity(), 1.0, 0.5, &grid);
        assert!(result.is_none(), "Capsule above water should return None");
    }

    #[test]
    fn capsule_tilted_produces_righting_offset() {
        let grid = make_test_grid(5.0);
        // Center at water surface, tilted 45° around Z axis.
        let center = Point3::new(0.0, 5.0, 0.0);
        let tilt = UnitQuaternion::from_euler_angles(0.0, 0.0, PI * 0.25);

        let (_vol, _frac, centroid) =
            compute_capsule_submersion(center, tilt, 1.0, 0.5, &grid).unwrap();

        // Buoyancy center should be offset horizontally toward the lower
        // side of the tilt (negative x for a positive z-rotation).
        assert!(
            (centroid.x - center.x).abs() > 0.01,
            "Tilted capsule should have horizontal buoyancy offset, got x={}",
            centroid.x
        );
    }

    #[test]
    fn capsule_horizontal_fully_submerged_matches_full_volume() {
        let grid = make_test_grid(10.0);
        let center = Point3::new(0.0, 5.0, 0.0);
        let half_height = 3.0;
        let radius = 0.5;
        // Rotate 90° around Z so capsule axis is horizontal (X axis).
        let horizontal = UnitQuaternion::from_euler_angles(0.0, 0.0, PI * 0.5);

        let (vol, frac, centroid) =
            compute_capsule_submersion(center, horizontal, half_height, radius, &grid).unwrap();

        let cyl_len = 2.0 * (half_height - radius);
        let expected_vol = PI * radius * radius * cyl_len + (4.0 / 3.0) * PI * radius.powi(3);
        assert!(
            (frac - 1.0).abs() < 0.01,
            "Horizontal capsule should be fully submerged, got fraction={frac}"
        );
        assert!(
            (vol - expected_vol).abs() < 0.01,
            "Volume should match full capsule, got {vol} expected {expected_vol}"
        );
        assert!(
            (centroid.y - center.y).abs() < 0.01,
            "Fully submerged centroid should be at center, got y={} expected y={}",
            centroid.y,
            center.y
        );
    }

    #[test]
    fn a_capsule_moving_end_on_meets_less_drag_than_side_on() {
        let shape = ColliderShape::Capsule {
            half_height: 0.5,
            radius: 0.17,
        };
        let water = make_test_grid(10.0);
        let drag = |flow: Option<Vector3<f32>>| {
            compute_buoyancy(
                Point3::new(0.0, 5.0, 0.0),
                UnitQuaternion::identity(),
                &shape,
                FLUID_DENSITY,
                Vector3::new(0.0, -9.81, 0.0),
                9.81,
                flow,
                &water,
            )
            .unwrap()
            .quadratic_linear_drag_coeff
        };
        let end_on = drag(Some(Vector3::y()));
        let side_on = drag(Some(Vector3::x()));
        let mean = drag(None);
        assert!(end_on < mean && mean < side_on, "{end_on} {mean} {side_on}");
        assert!((side_on / end_on - (1.0 + 0.34 * 0.66 / (PI * 0.0289))).abs() < 1e-3);
    }
}
