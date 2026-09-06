use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3};
use smallvec::SmallVec;

use super::framework::PhysicsBenchScenario;
use super::geometry::*;
use crate::collision::convex_hull::{ConvexHull, HullFace};
use crate::physics::constraint::ConstraintKind;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry};

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: mesh pipeline (sphere/box vs static terrain)
// ═══════════════════════════════════════════════════════════════════════════

/// Sphere dropped onto a flat surface. Exercises sphere-patch manifold
/// generation and the mesh-aware pipeline (seam filter -> sphere_patch).
#[derive(Debug, Clone)]
pub struct FlatSphereRestScenario {
    pub restitution: f32,
    pub friction: f32,
    pub radius: f32,
    pub spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatSphereRestScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            friction: 0.5,
            radius: 0.5,
            spawn_height: 3.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for FlatSphereRestScenario {
    fn name(&self) -> &'static str {
        "flat_sphere_rest"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            self.spawn_height,
            0.0,
        )));
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(self.friction);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere sliding laterally across flat terrain. The sphere crosses the
/// internal mesh edge (diagonal seam between the two triangles in
/// FlatQuadGeometry). This must not cause jitter or speed anomalies —
/// the seam filter suppresses the internal edge, and FeatureId-based
/// manifold caching ensures smooth contact transitions.
#[derive(Debug, Clone)]
pub struct SphereSlideScenario {
    pub radius: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSlideScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.5,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for SphereSlideScenario {
    fn name(&self) -> &'static str {
        "sphere_slide"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Sphere placed at rest on the surface with lateral velocity.
        // Low friction so it slides rather than rolls to a stop immediately.
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, self.radius + 0.01, 0.0))
                .linear_velocity(Vector3::new(3.0, 0.0, 0.0)),
        );
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.1);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped onto a flat surface. Exercises OBB-patch manifold generation
/// (seam filter -> obb_patch clipping).
#[derive(Debug, Clone)]
pub struct FlatBoxRestScenario {
    pub restitution: f32,
    pub friction: f32,
    pub half_extents: Vector3<f32>,
    pub spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatBoxRestScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            friction: 0.6,
            half_extents: Vector3::new(0.5, 0.5, 2.5),
            spawn_height: 2.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for FlatBoxRestScenario {
    fn name(&self) -> &'static str {
        "flat_box_rest"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            self.spawn_height,
            0.0,
        )));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(10.0)
            .restitution(self.restitution)
            .friction(self.friction);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere rolling down a ramp. Exercises sphere-patch on non-flat terrain
/// where the seam filter encounters a crease edge between the flat and sloped
/// sections. Verifies lateral movement (x stays centered, z advances).
#[derive(Debug, Clone)]
pub struct SphereOnRampScenario {
    pub restitution: f32,
    pub radius: f32,
    geometry: RampGeometry,
}

impl SphereOnRampScenario {
    pub fn new() -> Self {
        Self {
            restitution: 0.1,
            radius: 0.4,
            geometry: RampGeometry::new(4.0, 6.0, 3.0),
        }
    }
}

impl PhysicsBenchScenario for SphereOnRampScenario {
    fn name(&self) -> &'static str {
        "sphere_on_ramp"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop sphere onto the ramp midway up.
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 3.0, 3.0))
                .linear_damping(0.3)
                .angular_damping(0.3),
        );
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.6);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped onto a ramp. Exercises OBB-patch manifold generation against
/// a sloped surface (non-axis-aligned face normal).
#[derive(Debug, Clone)]
pub struct BoxOnRampScenario {
    pub restitution: f32,
    pub half_extents: Vector3<f32>,
    geometry: RampGeometry,
}

impl BoxOnRampScenario {
    pub fn new() -> Self {
        Self {
            restitution: 0.1,
            half_extents: Vector3::new(0.4, 0.4, 0.4),
            geometry: RampGeometry::new(4.0, 6.0, 3.0),
        }
    }
}

impl PhysicsBenchScenario for BoxOnRampScenario {
    fn name(&self) -> &'static str {
        "box_on_ramp"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 3.0, 3.0)));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.6);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped near a step edge. Exercises OBB-patch manifold generation
/// against multi-level terrain with a vertical wall face.
#[derive(Debug, Clone)]
pub struct BoxOnStepScenario {
    pub restitution: f32,
    pub half_extents: Vector3<f32>,
    geometry: StepGeometry,
}

impl BoxOnStepScenario {
    pub fn new() -> Self {
        Self {
            restitution: 0.1,
            half_extents: Vector3::new(0.3, 0.3, 0.3),
            geometry: StepGeometry::new(4.0, 0.5),
        }
    }
}

impl PhysicsBenchScenario for BoxOnStepScenario {
    fn name(&self) -> &'static str {
        "box_on_step"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop box onto the lower level near the step edge.
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(-0.5, 2.0, 0.0)));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.5);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Heavy sphere resting near the edge of a thin platform box on flat terrain.
///
/// This reproduces a dynamic-on-dynamic resting-contact case where the lower
/// platform can develop rotational jitter under asymmetric load.
#[derive(Debug, Clone)]
pub struct HeavySphereOnPlatformScenario {
    pub platform_half_extents: Vector3<f32>,
    pub platform_density: f32,
    pub sphere_radius: f32,
    pub sphere_density: f32,
    geometry: FlatQuadGeometry,
}

impl HeavySphereOnPlatformScenario {
    pub fn new() -> Self {
        Self {
            platform_half_extents: Vector3::new(3.5, 0.08, 3.5),
            platform_density: 350.0,
            sphere_radius: 0.7,
            sphere_density: 30_000.0,
            geometry: FlatQuadGeometry::new(30.0),
        }
    }
}

impl PhysicsBenchScenario for HeavySphereOnPlatformScenario {
    fn name(&self) -> &'static str {
        "heavy_sphere_on_platform"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = self.platform_half_extents;
        let platform = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            he.y + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            platform,
            ColliderDesc::box_shape(he)
                .density(self.platform_density)
                .restitution(0.0)
                .friction(0.8),
        );

        let sphere_x = he.x - self.sphere_radius * 0.35;
        let sphere_z = he.z - self.sphere_radius * 0.55;
        let sphere_y = he.y * 2.0 + self.sphere_radius + 0.2;
        let sphere = world.create_body(
            RigidBodyDesc::dynamic().position(Point3::new(sphere_x, sphere_y, sphere_z)),
        );
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(self.sphere_radius)
                .density(self.sphere_density)
                .restitution(0.0)
                .friction(0.8),
        );

        platform
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere dropped into an inverted square-based pyramid (bowl). The sphere
/// should come to rest touching all four sloped faces simultaneously.
///
/// This scenario exercises multi-face contact stability: the contact
/// pipeline must generate contacts on all four non-coplanar faces at once,
/// not oscillate a single contact between opposite sides.
#[derive(Debug, Clone)]
pub struct SphereInBowlScenario {
    pub radius: f32,
    geometry: BowlGeometry,
}

impl SphereInBowlScenario {
    pub fn new() -> Self {
        // Bowl: half_size=2, depth=2 → 45° slope faces.
        // For a sphere of radius 0.5, the equilibrium center is at
        // y ≈ 0.5*√2 - 2 ≈ -1.293 (each face distance equals radius).
        Self {
            radius: 0.5,
            geometry: BowlGeometry::new(2.0, 2.0),
        }
    }
}

impl PhysicsBenchScenario for SphereInBowlScenario {
    fn name(&self) -> &'static str {
        "sphere_in_bowl"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop the sphere from just above the rim, centered over the apex.
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 0.5, 0.0)));
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.8);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box leaning against a vertical wall at x=0 with its bottom corner on the
/// floor at y=0 — the classic "ladder sliding down a wall" problem.
///
/// Both contact corners lie on the box's −X face:
///   - Wall contact at local (−a, +L) → world (0, 2L cos θ)
///   - Floor contact at local (−a, −L) → world (2L sin θ, 0)
///
/// The constrained center of mass follows:
///   x_c = L sin θ + a cos θ
///   y_c = L cos θ + a sin θ
///
/// where a = half_extents.x (thin), L = half_extents.y (long), and θ is the
/// CCW tilt angle from vertical. This is a 1-DOF system with an analytical ODE
/// solution (see `LadderOde`), so the simulation can be compared against the
/// reference trajectory.
///
/// The scenario uses zero friction and zero restitution for a clean comparison
/// against the frictionless analytical solution.
#[derive(Debug, Clone)]
pub struct BoxSlidesDownWallScenario {
    /// Half-extents: x = thin (perpendicular to wall), y = long (parallel).
    pub half_extents: Vector3<f32>,
    /// Initial tilt angle from vertical (radians).
    pub theta0: f32,
    /// Friction coefficient for both wall and floor contacts.
    pub friction: f32,
    geometry: WallAndFloorGeometry,
}

impl BoxSlidesDownWallScenario {
    pub fn new(friction: f32) -> Self {
        let theta0 = std::f32::consts::FRAC_PI_6; // 30°
        Self {
            half_extents: Vector3::new(0.15, 1.2, 0.4),
            theta0,
            friction,
            geometry: WallAndFloorGeometry::new(8.0, 8.0),
        }
    }
}

impl PhysicsBenchScenario for BoxSlidesDownWallScenario {
    fn name(&self) -> &'static str {
        "box_slides_down_wall"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let a = self.half_extents.x;
        let big_l = self.half_extents.y;
        let theta = self.theta0;

        // Place the box exactly in the constrained configuration: wall corner
        // at x=0, floor corner at y=0, both on the −X face.
        let x_c = big_l * theta.sin() + a * theta.cos();
        let y_c = big_l * theta.cos() + a * theta.sin();

        let mut desc = RigidBodyDesc::dynamic().position(Point3::new(x_c, y_c, 0.0));
        desc.rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta);
        let body = world.create_body(desc);
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(500.0)
            .restitution(0.0)
            .friction(self.friction);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: discrete dynamic-dynamic contacts
// ═══════════════════════════════════════════════════════════════════════════

/// Two spheres on a collision course. A radial impulse kicks one sphere
/// toward the other. Exercises discrete sphere-sphere contacts and the
/// dynamic pair pipeline.
#[derive(Debug, Clone)]
pub struct SphereSphereCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSphereCollisionScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for SphereSphereCollisionScenario {
    fn name(&self) -> &'static str {
        "sphere_sphere_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        let y = radius + 0.01;

        // Target sphere (tracked) — sitting at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile sphere — offset in -X, launched directly toward target.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, y, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere launched toward a stationary box. Exercises discrete sphere-OBB
/// contacts in the dynamic pair pipeline.
#[derive(Debug, Clone)]
pub struct SphereObbCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereObbCollisionScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for SphereObbCollisionScenario {
    fn name(&self) -> &'static str {
        "sphere_obb_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let sphere_radius = 0.4;
        let box_half = Vector3::new(0.5, 0.5, 0.5);
        let y = box_half.y + 0.01;

        // Target box (tracked) at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::box_shape(box_half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile sphere — launched directly toward target.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, sphere_radius + 0.01, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::sphere(sphere_radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Two boxes on a collision course. Exercises discrete OBB-OBB contact
/// generation (15-axis SAT + Sutherland-Hodgman clipping).
#[derive(Debug, Clone)]
pub struct ObbObbCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl ObbObbCollisionScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for ObbObbCollisionScenario {
    fn name(&self) -> &'static str {
        "obb_obb_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let half = Vector3::new(0.5, 0.5, 0.5);
        let y = half.y + 0.01;

        // Target box (tracked) at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::box_shape(half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile box — launched directly toward target via initial velocity.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, y, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::box_shape(half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: many-body dynamic contacts (narrowphase throughput)
// ═══════════════════════════════════════════════════════════════════════════

/// Grid of boxes dropped onto flat terrain. Exercises the dynamic-dynamic
/// narrowphase pipeline with many simultaneous broadphase pairs. Used to
/// benchmark the work buffer's allocation reuse and cache performance.
#[derive(Debug, Clone)]
pub struct BoxGridScenario {
    pub grid_size: usize,
    pub half_extent: f32,
    pub spacing: f32,
    geometry: FlatQuadGeometry,
}

impl BoxGridScenario {
    pub fn new(grid_size: usize) -> Self {
        Self {
            grid_size,
            half_extent: 0.5,
            spacing: 0.9,
            geometry: FlatQuadGeometry::new(5.0),
        }
    }
}

impl PhysicsBenchScenario for BoxGridScenario {
    fn name(&self) -> &'static str {
        "box_grid"
    }

    fn restitution(&self) -> f32 {
        0.1
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = Vector3::new(self.half_extent, self.half_extent, self.half_extent);
        let n = self.grid_size;
        let offset = 0.0; // (n as f32 - 1.0) * self.spacing * 0.5;
        let mut first_handle = None;

        for ix in 0..n {
            for iz in 0..n {
                let x = ix as f32 * self.spacing - offset;
                let z = 0.0;
                let y = 0.3 + iz as f32 * self.spacing - offset;
                let body =
                    world.create_body(RigidBodyDesc::dynamic().position(Point3::new(x, y, z)));
                let _ = world.attach_collider(
                    body,
                    ColliderDesc::box_shape(he)
                        .density(1000.0)
                        .restitution(0.1)
                        .friction(0.5),
                );
                if first_handle.is_none() {
                    first_handle = Some(body);
                }
            }
        }

        first_handle.expect("grid should contain at least one body")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: continuous collision detection
// ═══════════════════════════════════════════════════════════════════════════

/// High-speed sphere dropped from height. Without CCD, the sphere would
/// tunnel through the ground. Exercises `swept_sphere_triangle` from the
/// continuous collision module.
#[derive(Debug, Clone)]
pub struct HighSpeedSphereCcdScenario {
    geometry: FlatQuadGeometry,
}

impl HighSpeedSphereCcdScenario {
    pub fn new() -> Self {
        Self {
            geometry: FlatQuadGeometry::new(10.0),
        }
    }
}

impl PhysicsBenchScenario for HighSpeedSphereCcdScenario {
    fn name(&self) -> &'static str {
        "high_speed_sphere_ccd"
    }

    fn restitution(&self) -> f32 {
        0.3
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.ccd_threshold = 0.3;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.3;
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 50.0, 0.0)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.3)
                .friction(0.5),
        );
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Solver stability validation scenarios
// ═══════════════════════════════════════════════════════════════════════════

/// Heavy box resting on a plank on flat terrain.
///
/// Reproduces stacking jitter where the bottom body (plank) develops vertical
/// oscillation from accumulated solver forces. The plank sits at an offset
/// from the terrain mesh center so it straddles the internal diagonal seam,
/// matching typical in-game placement. The box sits on top, creating
/// asymmetric load that stresses the solver.
///
/// The plank is the tracked body — its Y position and velocity should be
/// rock-steady once the system settles.
#[derive(Debug, Clone)]
pub struct BoxOnPlankScenario {
    /// Half-extents of the upper box.
    pub box_half_extents: Vector3<f32>,
    /// Density of the upper box.
    pub box_density: f32,
    /// Half-extents of the lower plank (height = 0.5 → full height 1.0).
    pub plank_half_extents: Vector3<f32>,
    /// Density of the plank.
    pub plank_density: f32,
    /// XZ offset from terrain center (places bodies over the mesh seam).
    pub offset_xz: f32,
    geometry: FlatGridGeometry,
}

impl BoxOnPlankScenario {
    pub fn new() -> Self {
        Self {
            box_half_extents: Vector3::new(0.5, 0.5, 0.5),
            box_density: 1000.0,
            plank_half_extents: Vector3::new(1.5, 0.5, 1.5),
            plank_density: 100.0,
            offset_xz: 3.0,
            geometry: FlatGridGeometry::new(10.0, 1.0),
        }
    }
}

impl PhysicsBenchScenario for BoxOnPlankScenario {
    fn name(&self) -> &'static str {
        "box_on_plank"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let xz = self.offset_xz;
        let plank_he = self.plank_half_extents;
        let plank_y = plank_he.y + 0.01;
        let plank =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(xz, plank_y, xz)));
        let _ = world.attach_collider(
            plank,
            ColliderDesc::box_shape(plank_he)
                .density(self.plank_density)
                .restitution(0.0)
                .friction(0.6),
        );

        let box_he = self.box_half_extents;
        let box_y = plank_he.y * 2.0 + box_he.y + 0.02;
        let box_x = xz + plank_he.x - box_he.x;
        let box_z = xz + plank_he.z - box_he.z;
        let upper_box =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(box_x, box_y, box_z)));
        let _ = world.attach_collider(
            upper_box,
            ColliderDesc::box_shape(box_he)
                .density(self.box_density)
                .restitution(0.0)
                .friction(0.6),
        );

        plank
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere launched horizontally across a flat surface with moderate friction.
///
/// Tests that friction warm-start does not inject angular torque spikes as
/// the sphere decelerates. The sphere should slow smoothly and come to rest.
#[derive(Debug, Clone)]
pub struct SlidingSphereScenario {
    geometry: FlatQuadGeometry,
}

impl SlidingSphereScenario {
    pub fn new() -> Self {
        Self {
            geometry: FlatQuadGeometry::new(50.0),
        }
    }
}

impl PhysicsBenchScenario for SlidingSphereScenario {
    fn name(&self) -> &'static str {
        "sliding_sphere"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, radius + 0.01, 0.0))
                .linear_damping(0.4)
                .angular_damping(0.5),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.5),
        );
        // Give it a horizontal push
        world.set_body_velocity(body, Vector3::new(5.0, 0.0, 0.0), Vector3::zeros());
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere on a low-friction 30° ramp. With friction = 0.1 (below tan(30°) ≈ 0.577),
/// the sphere should slide down and accelerate rather than sticking.
#[derive(Debug, Clone)]
pub struct LowFrictionRampScenario {
    geometry: RampGeometry,
}

impl LowFrictionRampScenario {
    pub fn new() -> Self {
        // 30° slope: rise/run = tan(30°) ≈ 0.577
        let run = 20.0;
        let rise = run * 0.577;
        Self {
            geometry: RampGeometry::new(5.0, run, rise),
        }
    }
}

impl PhysicsBenchScenario for LowFrictionRampScenario {
    fn name(&self) -> &'static str {
        "low_friction_ramp"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        // Start on the ramp (z > 0 is the ramp section, partway up)
        let z = 10.0;
        let y_on_ramp = 0.577 * z + radius + 0.01;
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y_on_ramp, z)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.1),
        );
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: constraint system
// ═══════════════════════════════════════════════════════════════════════════

/// Capsule on flat ground with a KeepUpright constraint, given an initial
/// angular velocity to try to topple it. Validates that the constraint keeps
/// the capsule upright and that it settles to rest.
#[derive(Debug, Clone)]
pub struct KeepUprightScenario {
    /// Half-height of the capsule cylinder section.
    pub half_height: f32,
    /// Capsule radius.
    pub radius: f32,
    /// Initial angular velocity applied to try to topple the capsule.
    pub initial_angular_velocity: Vector3<f32>,
    /// Constraint compliance (0 = rigid).
    pub compliance: f32,
    geometry: FlatQuadGeometry,
}

impl KeepUprightScenario {
    pub fn new() -> Self {
        Self {
            half_height: 0.5,
            radius: 0.3,
            initial_angular_velocity: Vector3::new(5.0, 0.0, 3.0),
            compliance: 0.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for KeepUprightScenario {
    fn name(&self) -> &'static str {
        "keep_upright"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let spawn_y = self.half_height + self.radius + 0.01;
        let mut desc = RigidBodyDesc::dynamic().position(Point3::new(0.0, spawn_y, 0.0));
        desc.angular_velocity = self.initial_angular_velocity;
        let body = world.create_body(desc);
        let collider = ColliderDesc::capsule(self.half_height, self.radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.5);
        let _ = world.attach_collider(body, collider);

        let _ = world.create_constraint(ConstraintKind::KeepUpright {
            body,
            target_up: UnitVector3::new_normalize(Vector3::y()),
            compliance: self.compliance,
            max_impulse: f32::INFINITY,
        });

        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: thin-block wobble
// ═══════════════════════════════════════════════════════════════════════════

/// Two Jenga-proportioned blocks in a cross configuration: bottom block flat on
/// terrain, top block perpendicular and resting roughly halfway across the bottom
/// block's top edge.
///
/// Reproduces a persistent angular oscillation (wobble) that thin OBB-on-OBB
/// contacts are susceptible to. The top block is given a small initial tilt and
/// lateral offset to break symmetry, matching the configurations that arise
/// naturally when a Jenga tower collapses.
#[derive(Debug, Clone)]
pub struct JengaCrossWobbleScenario {
    /// Half-extents of each Jenga block: (half_length, half_height, half_width).
    pub block_half_extents: Vector3<f32>,
    /// Density of both blocks.
    pub density: f32,
    /// Friction coefficient for both blocks and terrain contact.
    pub friction: f32,
    /// Small tilt angle (radians) applied to the top block to break symmetry.
    pub tilt_rad: f32,
    geometry: FlatGridGeometry,
}

impl JengaCrossWobbleScenario {
    pub fn new() -> Self {
        let half_length = 0.75;
        Self {
            block_half_extents: Vector3::new(half_length, half_length / 5.0, half_length / 3.0),
            density: 500.0,
            friction: 0.6,
            tilt_rad: 0.012,
            geometry: FlatGridGeometry::new(10.0, 1.0),
        }
    }
}

impl PhysicsBenchScenario for JengaCrossWobbleScenario {
    fn name(&self) -> &'static str {
        "jenga_cross_wobble"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = self.block_half_extents;

        // Bottom block: a narrow ridge (thin in Z) on terrain. The narrow top
        // surface creates an edge-like contact strip with the perpendicular top
        // block, mimicking the cross-block configuration from a Jenga collapse.
        let ridge_he = Vector3::new(he.x, he.y, 0.05);
        let bottom_y = ridge_he.y + 0.001;
        let bottom = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, bottom_y, 0.0))
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let _ = world.attach_collider(
            bottom,
            ColliderDesc::box_shape(ridge_he)
                .density(self.density * 3.0)
                .restitution(0.0)
                .friction(self.friction),
        );

        // Top block: perpendicular (90° around Y), resting on the narrow ridge.
        // The contact strip is only 0.10m wide, creating a near-edge contact
        // that provides minimal restoring torque around the long axis.
        let bottom_top = bottom_y + ridge_he.y;
        let top_y = bottom_top + he.y + 0.003;
        let top_x = 0.0;
        let top_z = 0.013;

        let yaw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.tilt_rad);
        let rotation = tilt * yaw;

        let top = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(top_x, top_y, top_z))
                .rotation(rotation)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let _ = world.attach_collider(
            top,
            ColliderDesc::box_shape(he)
                .density(self.density)
                .restitution(0.0)
                .friction(self.friction),
        );

        top
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: compound colliders
// ═══════════════════════════════════════════════════════════════════════════

/// A table built from 5 box colliders (top + 4 legs) on a single body,
/// dropped onto a flat surface. Exercises compound collider offset rotation,
/// parallel axis theorem mass aggregation, and multi-collider narrowphase.
#[derive(Debug, Clone)]
pub struct CompoundTableScenario {
    /// Half-extents of the table top.
    pub top_half_extents: Vector3<f32>,
    /// Half-extents of each leg.
    pub leg_half_extents: Vector3<f32>,
    /// Height to drop the table from (center of mass).
    pub spawn_height: f32,
    geometry: FlatGridGeometry,
}

impl CompoundTableScenario {
    pub fn new() -> Self {
        Self {
            top_half_extents: Vector3::new(0.5, 0.025, 0.3),
            leg_half_extents: Vector3::new(0.03, 0.175, 0.03),
            spawn_height: 1.5,
            geometry: FlatGridGeometry::new(8.0, 1.0),
        }
    }
}

impl PhysicsBenchScenario for CompoundTableScenario {
    fn name(&self) -> &'static str {
        "compound_table"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let top_he = self.top_half_extents;
        let leg_he = self.leg_half_extents;

        // Table top sits at the top; legs hang below it.
        // Body origin is at the geometric center of the table.
        let table_height = leg_he.y * 2.0 + top_he.y * 2.0;
        let top_y = table_height * 0.5 - top_he.y;
        let leg_y = -top_he.y;

        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            self.spawn_height,
            0.0,
        )));

        // Table top
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(top_he)
                .offset_translation(Vector3::new(0.0, top_y, 0.0))
                .density(600.0)
                .restitution(0.0)
                .friction(0.5),
        );

        // Four legs at the corners of the table top
        let leg_x = top_he.x - leg_he.x;
        let leg_z = top_he.z - leg_he.z;
        let leg_positions = [
            Vector3::new(-leg_x, leg_y, -leg_z),
            Vector3::new(leg_x, leg_y, -leg_z),
            Vector3::new(-leg_x, leg_y, leg_z),
            Vector3::new(leg_x, leg_y, leg_z),
        ];
        for &pos in &leg_positions {
            let _ = world.attach_collider(
                body,
                ColliderDesc::box_shape(leg_he)
                    .offset_translation(pos)
                    .density(600.0)
                    .restitution(0.0)
                    .friction(0.5),
            );
        }

        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Convex hull helper
// ═══════════════════════════════════════════════════════════════════════════

/// Face definition for building convex hulls in bench scenarios.
///
/// Mirrors `SolidFace` from the app layer but lives in the physics bench
/// harness to avoid a reverse dependency.
struct FaceDef {
    /// Vertex indices defining the face polygon (CCW from outside).
    indices: Vec<usize>,
    /// Index of a vertex on the opposite side of the hull, used to determine
    /// the outward normal direction.
    opposite: usize,
}

/// Build a `ConvexHull` from vertices and face definitions.
///
/// Computes outward normals using the opposite-vertex trick: the cross product
/// of the first two edges gives a candidate normal; if it points toward the
/// opposite vertex, it's flipped.
fn build_hull(vertices: &[Vector3<f32>], faces: &[FaceDef]) -> ConvexHull {
    let hull_faces: Vec<HullFace> = faces
        .iter()
        .map(|face| {
            let a = vertices[face.indices[0]];
            let b = vertices[face.indices[1]];
            let c = vertices[face.indices[2]];
            let opp = vertices[face.opposite];

            let raw_normal = (b - a).cross(&(c - a));
            let flip = raw_normal.dot(&(a - opp)) < 0.0;
            let normal = if flip {
                -raw_normal.normalize()
            } else {
                raw_normal.normalize()
            };

            let mut indices: SmallVec<[u16; 6]> = face.indices.iter().map(|&i| i as u16).collect();
            if flip {
                indices[1..].reverse();
            }

            HullFace {
                vertex_indices: indices,
                normal,
            }
        })
        .collect();

    ConvexHull::new(vertices.to_vec(), hull_faces)
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: structural stability regression
// ═══════════════════════════════════════════════════════════════════════════

/// Honeycomb wall: a tiled grid of hexagonal prisms stacked in a honeycomb
/// pattern. Each cell is an independent dynamic body with a convex hull
/// collider. Tests multi-body convex-hull-on-ground and hull-on-hull contact
/// stability.
#[derive(Debug, Clone)]
pub struct HoneycombWallScenario {
    pub columns: u32,
    pub rows: u32,
    pub radius: f32,
    pub half_height: f32,
    pub density: f32,
    geometry: FlatQuadGeometry,
}

impl HoneycombWallScenario {
    pub fn new() -> Self {
        Self {
            columns: 5,
            rows: 4,
            radius: 0.4,
            half_height: 0.25,
            density: 500.0,
            geometry: FlatQuadGeometry::new(10.0),
        }
    }
}

impl PhysicsBenchScenario for HoneycombWallScenario {
    fn name(&self) -> &'static str {
        "honeycomb_wall"
    }

    fn restitution(&self) -> f32 {
        0.15
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let (vertices, faces) = hex_prism_geometry_z(self.radius, self.half_height);
        let hull = Arc::new(build_hull(&vertices, &faces));

        let col_spacing = self.radius * 3.0f32.sqrt();
        let row_spacing = self.radius * 1.5;
        let total_width = (self.columns - 1) as f32 * col_spacing;
        let x_start = -total_width * 0.5;

        let mut tracked = None;

        for row in 0..self.rows {
            let y = self.radius + row as f32 * row_spacing;
            let x_offset = if row % 2 == 1 { col_spacing * 0.5 } else { 0.0 };

            for col in 0..self.columns {
                let x = x_start + col as f32 * col_spacing + x_offset;
                let pos = Point3::new(x, y, 0.0);

                let body = world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .linear_damping(0.01)
                        .angular_damping(0.005),
                );
                world.attach_collider(
                    body,
                    ColliderDesc::convex_hull(hull.clone())
                        .density(self.density)
                        .restitution(0.15)
                        .friction(0.7),
                );

                // Track a top-row center cell.
                if row == self.rows - 1 && col == self.columns / 2 {
                    tracked = Some(body);
                }
            }
        }

        tracked.expect("should have created at least one body")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Hex prism with the prism axis along Z (hex faces point ±Z).
///
/// Pointy-topped in XY: vertex 0 at +Y, flat edges horizontal. This is the
/// classic honeycomb orientation when viewed from the front (+Z).
fn hex_prism_geometry_z(radius: f32, half_depth: f32) -> (Vec<Vector3<f32>>, Vec<FaceDef>) {
    let mut vertices = Vec::with_capacity(12);

    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32 + std::f32::consts::FRAC_PI_6;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Vector3::new(x, y, half_depth));
    }
    for i in 0..6 {
        let angle = std::f32::consts::FRAC_PI_3 * i as f32 + std::f32::consts::FRAC_PI_6;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        vertices.push(Vector3::new(x, y, -half_depth));
    }

    let mut faces = Vec::with_capacity(8);

    faces.push(FaceDef {
        indices: vec![0, 1, 2, 3, 4, 5],
        opposite: 6,
    });
    faces.push(FaceDef {
        indices: vec![11, 10, 9, 8, 7, 6],
        opposite: 0,
    });
    for i in 0..6usize {
        let next = (i + 1) % 6;
        let opposite = (i + 3) % 6;
        faces.push(FaceDef {
            indices: vec![i, i + 6, next + 6, next],
            opposite,
        });
    }

    (vertices, faces)
}

/// Voussoir arch: a semicircular masonry arch from wedge-shaped stones held
/// together by compression and friction, supported by two heavy abutment
/// pillars. Tests convex-hull-on-hull stability under gravitational load.
#[derive(Debug, Clone)]
pub struct VoussoirArchScenario {
    pub inner_radius: f32,
    pub thickness: f32,
    pub depth: f32,
    pub num_voussoirs: u32,
    pub abutment_height: f32,
    pub density: f32,
    pub friction: f32,
    geometry: FlatQuadGeometry,
}

impl VoussoirArchScenario {
    pub fn new() -> Self {
        Self {
            inner_radius: 10.0,
            thickness: 5.0,
            depth: 3.2,
            num_voussoirs: 15,
            abutment_height: 0.5,
            density: 2000.0,
            friction: 0.9,
            geometry: FlatQuadGeometry::new(25.0),
        }
    }
}

impl PhysicsBenchScenario for VoussoirArchScenario {
    fn name(&self) -> &'static str {
        "voussoir_arch"
    }

    fn restitution(&self) -> f32 {
        0.05
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let n = self.num_voussoirs;
        let inner_r = self.inner_radius;
        let outer_r = inner_r + self.thickness;
        let half_depth = self.depth / 2.0;
        let angle_step = std::f32::consts::PI / n as f32;
        let center_y = self.abutment_height;

        let mut keystone_handle = None;
        let keystone_idx = n / 2;

        // Voussoirs
        for i in 0..n {
            let angle_start = i as f32 * angle_step;
            let angle_end = (i + 1) as f32 * angle_step;

            let (arch_verts, faces) =
                voussoir_geometry(inner_r, outer_r, half_depth, angle_start, angle_end);

            let centroid =
                arch_verts.iter().copied().sum::<Vector3<f32>>() / arch_verts.len() as f32;
            let local_verts: Vec<_> = arch_verts.iter().map(|v| v - centroid).collect();

            let pos = Point3::new(centroid.x, center_y + centroid.y, centroid.z);
            let hull = Arc::new(build_hull(&local_verts, &faces));

            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::convex_hull(hull)
                    .density(self.density)
                    .restitution(0.05)
                    .friction(self.friction),
            );

            if i == keystone_idx {
                keystone_handle = Some(body);
            }
        }

        // Abutment pillars
        let abutment_he =
            Vector3::new(self.thickness / 2.0, self.abutment_height / 2.0, half_depth);
        let abutment_density = self.density * 2.0;

        for side in [1.0f32, -1.0] {
            let x = side * (inner_r + self.thickness / 2.0);
            let y = self.abutment_height / 2.0;
            let pos = Point3::new(x, y, 0.0);

            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::box_shape(abutment_he)
                    .density(abutment_density)
                    .restitution(0.05)
                    .friction(self.friction),
            );
        }

        keystone_handle.expect("should have created the keystone voussoir")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Vertices and faces of a single voussoir (truncated wedge).
///
/// Computed in arch-local space where the arch center of curvature is at the
/// origin. The voussoir spans from `angle_start` to `angle_end` (radians,
/// measured counter-clockwise from the +X axis in the XY plane).
fn voussoir_geometry(
    inner_radius: f32,
    outer_radius: f32,
    half_depth: f32,
    angle_start: f32,
    angle_end: f32,
) -> (Vec<Vector3<f32>>, Vec<FaceDef>) {
    let cos_s = angle_start.cos();
    let sin_s = angle_start.sin();
    let cos_e = angle_end.cos();
    let sin_e = angle_end.sin();

    let vertices = vec![
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, half_depth),
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, half_depth),
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, half_depth),
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, half_depth),
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, -half_depth),
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, -half_depth),
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, -half_depth),
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, -half_depth),
    ];

    let faces = vec![
        FaceDef {
            indices: vec![0, 1, 2, 3],
            opposite: 4,
        },
        FaceDef {
            indices: vec![7, 6, 5, 4],
            opposite: 0,
        },
        FaceDef {
            indices: vec![0, 4, 5, 1],
            opposite: 3,
        },
        FaceDef {
            indices: vec![3, 2, 6, 7],
            opposite: 0,
        },
        FaceDef {
            indices: vec![1, 5, 6, 2],
            opposite: 0,
        },
        FaceDef {
            indices: vec![0, 3, 7, 4],
            opposite: 1,
        },
    ];

    (vertices, faces)
}

/// Jenga tower: alternating layers of three planks rotated 90°, using real
/// Jenga proportions. Tests OBB-on-OBB stacking stability with many thin
/// contacts.
#[derive(Debug, Clone)]
pub struct JengaTowerScenario {
    pub layers: u32,
    pub block_half_length: f32,
    pub density: f32,
    pub friction: f32,
    geometry: FlatQuadGeometry,
}

impl JengaTowerScenario {
    pub fn new(layers: u32) -> Self {
        Self {
            layers,
            block_half_length: 0.75,
            density: 500.0,
            friction: 0.6,
            geometry: FlatQuadGeometry::new(10.0),
        }
    }

    fn block_half_extents(&self) -> Vector3<f32> {
        let hl = self.block_half_length;
        Vector3::new(hl, hl / 5.0, hl / 3.0)
    }
}

impl PhysicsBenchScenario for JengaTowerScenario {
    fn name(&self) -> &'static str {
        "jenga_tower"
    }

    fn restitution(&self) -> f32 {
        0.05
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = self.block_half_extents();
        let block_height = he.y * 2.0;
        let block_width = he.z * 2.0;
        let blocks_per_layer = 3u32;

        let yaw_90 =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);

        let mut tracked = None;

        for layer in 0..self.layers {
            let y = he.y + layer as f32 * block_height;
            let rotated = layer % 2 == 1;

            for slot in 0..blocks_per_layer {
                let lateral_offset =
                    (slot as f32 - (blocks_per_layer - 1) as f32 / 2.0) * block_width;

                let (x, z) = if rotated {
                    (lateral_offset, 0.0)
                } else {
                    (0.0, lateral_offset)
                };

                let pos = Point3::new(x, y, z);
                let rotation = if rotated {
                    yaw_90
                } else {
                    UnitQuaternion::identity()
                };

                let body = world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(pos)
                        .rotation(rotation)
                        .linear_damping(0.01)
                        .angular_damping(0.05),
                );
                world.attach_collider(
                    body,
                    ColliderDesc::box_shape(he)
                        .density(self.density)
                        .restitution(0.05)
                        .friction(self.friction),
                );

                // Track the center block of the top layer.
                if layer == self.layers - 1 && slot == 1 {
                    tracked = Some(body);
                }
            }
        }

        tracked.expect("should have created at least one block")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

const PHI: f32 = 1.618034;
const COLUMN_SIDES: u32 = 20;

/// Greek temple: stepped stylobate, tapered Doric columns, entablature beams,
/// triangular pediments, and pitched roof panels — all independent dynamic
/// bodies. Tests large-scale multi-body structural stability with mixed
/// collider types (boxes + convex hulls).
#[derive(Debug, Clone)]
pub struct TempleScenario {
    pub column_height: f32,
    pub front_columns: u32,
    pub side_columns: u32,
    geometry: FlatQuadGeometry,
}

impl TempleScenario {
    pub fn new() -> Self {
        Self {
            column_height: 8.0,
            front_columns: 6,
            side_columns: 9,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

struct TempleLayout {
    col_base_r: f32,
    col_top_r: f32,
    col_height: f32,
    spacing: f32,
    half_w: f32,
    half_l: f32,
    num_steps: u32,
    step_h: f32,
    step_margin: f32,
    stylobate_top: f32,
    entab_h: f32,
    entab_overhang: f32,
    entab_base_y: f32,
    pediment_h: f32,
    pediment_base_y: f32,
    roof_t: f32,
}

impl TempleLayout {
    fn from_scenario(s: &TempleScenario) -> Self {
        let ch = s.column_height;
        let col_base_r = ch / 12.0;
        let col_top_r = col_base_r * 0.82;
        let spacing = ch / PHI.powi(2);
        let half_w = (s.front_columns - 1) as f32 * spacing / 2.0;
        let half_l = (s.side_columns - 1) as f32 * spacing / 2.0;
        let num_steps = 3u32;
        let step_h = ch / 24.0;
        let step_margin = spacing * 0.12;
        let stylobate_top = num_steps as f32 * step_h;
        let entab_h = ch / PHI.powi(2);
        let entab_overhang = col_base_r * 0.6;
        let entab_base_y = stylobate_top + ch;
        let pediment_h = half_w / PHI;
        let pediment_base_y = entab_base_y + entab_h;
        let roof_t = ch / 40.0;

        Self {
            col_base_r,
            col_top_r,
            col_height: ch,
            spacing,
            half_w,
            half_l,
            num_steps,
            step_h,
            step_margin,
            stylobate_top,
            entab_h,
            entab_overhang,
            entab_base_y,
            pediment_h,
            pediment_base_y,
            roof_t,
        }
    }
}

impl PhysicsBenchScenario for TempleScenario {
    fn name(&self) -> &'static str {
        "temple"
    }

    fn restitution(&self) -> f32 {
        0.05
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let lay = TempleLayout::from_scenario(self);
        let density = 2400.0;
        let friction = 1.5;

        let mut tracked = None;

        // ── Stylobate (compound body with 3 stepped box colliders) ───
        let top_step_hw = lay.half_w + lay.col_base_r + lay.step_margin;
        let top_step_hl = lay.half_l + lay.col_base_r + lay.step_margin;
        let stylobate_center_y = lay.num_steps as f32 * lay.step_h / 2.0;

        let stylobate = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, stylobate_center_y, 0.0))
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        for i in 0..lay.num_steps {
            let grow = (lay.num_steps - 1 - i) as f32 * lay.step_margin;
            let hw = top_step_hw + grow;
            let hl = top_step_hl + grow;
            let hh = lay.step_h / 2.0;
            let offset_y = i as f32 * lay.step_h + hh - lay.num_steps as f32 * lay.step_h / 2.0;
            world.attach_collider(
                stylobate,
                ColliderDesc::box_shape(Vector3::new(hw, hh, hl))
                    .offset_translation(Vector3::new(0.0, offset_y, 0.0))
                    .density(density)
                    .restitution(0.05)
                    .friction(friction),
            );
        }

        // ── Columns ──────────────────────────────────────────────────
        let col_y_base = lay.stylobate_top;
        let mut col_positions = Vec::new();

        for i in 0..self.front_columns {
            let x = -lay.half_w + i as f32 * lay.spacing;
            col_positions.push((x, lay.half_l));
            col_positions.push((x, -lay.half_l));
        }
        for j in 1..(self.side_columns - 1) {
            let z = -lay.half_l + j as f32 * lay.spacing;
            col_positions.push((-lay.half_w, z));
            col_positions.push((lay.half_w, z));
        }

        let (hull_verts, hull_faces) =
            column_hull_geometry(lay.col_base_r, lay.col_top_r, lay.col_height, COLUMN_SIDES);

        for (idx, &(cx, cz)) in col_positions.iter().enumerate() {
            let world_hull: Vec<_> = hull_verts
                .iter()
                .map(|v| Vector3::new(v.x + cx, v.y + col_y_base, v.z + cz))
                .collect();
            let centroid =
                world_hull.iter().copied().sum::<Vector3<f32>>() / world_hull.len() as f32;
            let local_hull: Vec<_> = world_hull.iter().map(|v| v - centroid).collect();
            let pos = Point3::new(centroid.x, centroid.y, centroid.z);
            let hull = Arc::new(build_hull(&local_hull, &hull_faces));

            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::convex_hull(hull)
                    .density(density)
                    .restitution(0.05)
                    .friction(friction),
            );

            // Track a front-center column.
            if idx == 0 {
                tracked = Some(body);
            }
        }

        // ── Entablature (4 beams) ────────────────────────────────────
        let entab_hh = lay.entab_h / 2.0;
        let entab_cy = lay.entab_base_y + entab_hh;
        let beam_depth = lay.col_base_r + lay.entab_overhang;
        let fb_hw = lay.half_w + beam_depth;

        let spawn_box = |world: &mut PhysicsWorld, pos: Point3<f32>, he: Vector3<f32>| {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::box_shape(he)
                    .density(density)
                    .restitution(0.05)
                    .friction(friction),
            );
            body
        };

        // Front and back beams
        for &z_sign in &[1.0f32, -1.0] {
            spawn_box(
                world,
                Point3::new(0.0, entab_cy, z_sign * lay.half_l),
                Vector3::new(fb_hw, entab_hh, beam_depth),
            );
        }

        // Side beams
        let side_hl = lay.half_l - beam_depth;
        for &x_sign in &[1.0f32, -1.0] {
            spawn_box(
                world,
                Point3::new(x_sign * lay.half_w, entab_cy, 0.0),
                Vector3::new(beam_depth, entab_hh, side_hl),
            );
        }

        // ── Pediments (triangular gables) ────────────────────────────
        let ped_base = lay.pediment_base_y;
        let ped_peak = ped_base + lay.pediment_h;
        let ped_hw = lay.half_w + beam_depth;

        for &z_sign in &[1.0f32, -1.0] {
            let z_outer = z_sign * (lay.half_l + beam_depth);
            let z_inner = z_sign * (lay.half_l - beam_depth);
            let (verts, faces) =
                gable_hull_geometry(0.0, ped_base, ped_peak, ped_hw, z_outer, z_inner);
            let centroid = verts.iter().copied().sum::<Vector3<f32>>() / verts.len() as f32;
            let local: Vec<_> = verts.iter().map(|v| v - centroid).collect();
            let pos = Point3::new(centroid.x, centroid.y, centroid.z);
            let hull = Arc::new(build_hull(&local, &faces));

            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(pos)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::convex_hull(hull)
                    .density(density)
                    .restitution(0.05)
                    .friction(friction),
            );
        }

        // ── Roof (two sloped panels) ─────────────────────────────────
        let eave_dist = ped_hw;
        let peak_h = lay.pediment_h;
        let slope_len = (eave_dist * eave_dist + peak_h * peak_h).sqrt();
        let roof_angle = peak_h.atan2(eave_dist);
        let roof_overhang = lay.step_margin * 2.0;
        let roof_half_depth = lay.half_l + beam_depth + roof_overhang;
        let roof_he = Vector3::new(slope_len / 2.0, lay.roof_t / 2.0, roof_half_depth);

        for &side in &[-1.0f32, 1.0] {
            let mid_x = side * eave_dist / 2.0;
            let mid_y = ped_base + peak_h / 2.0;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -side * roof_angle);

            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(mid_x, mid_y, 0.0))
                    .rotation(rot)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            world.attach_collider(
                body,
                ColliderDesc::box_shape(roof_he)
                    .density(density)
                    .restitution(0.05)
                    .friction(friction),
            );
        }

        tracked.expect("should have created columns")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Tapered polygonal prism for a Doric column (physics hull only).
fn column_hull_geometry(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    num_sides: u32,
) -> (Vec<Vector3<f32>>, Vec<FaceDef>) {
    let n = num_sides as usize;
    let angle_step = std::f32::consts::TAU / num_sides as f32;
    let mut vertices = Vec::with_capacity(2 * n);

    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            base_radius * angle.cos(),
            0.0,
            base_radius * angle.sin(),
        ));
    }
    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            top_radius * angle.cos(),
            height,
            top_radius * angle.sin(),
        ));
    }

    let mut faces = Vec::with_capacity(n + 2);

    // Bottom face
    faces.push(FaceDef {
        indices: (0..n).rev().collect(),
        opposite: n,
    });
    // Top face
    faces.push(FaceDef {
        indices: (n..2 * n).collect(),
        opposite: 0,
    });
    // Side faces
    for i in 0..n {
        let i_next = (i + 1) % n;
        faces.push(FaceDef {
            indices: vec![i, i_next, n + i_next, n + i],
            opposite: (i + n / 2) % n,
        });
    }

    (vertices, faces)
}

/// Triangular gable prism geometry.
fn gable_hull_geometry(
    cx: f32,
    base_y: f32,
    peak_y: f32,
    half_w: f32,
    z_a: f32,
    z_b: f32,
) -> (Vec<Vector3<f32>>, Vec<FaceDef>) {
    let vertices = vec![
        Vector3::new(cx - half_w, base_y, z_a),
        Vector3::new(cx + half_w, base_y, z_a),
        Vector3::new(cx, peak_y, z_a),
        Vector3::new(cx - half_w, base_y, z_b),
        Vector3::new(cx + half_w, base_y, z_b),
        Vector3::new(cx, peak_y, z_b),
    ];

    let faces = vec![
        FaceDef {
            indices: vec![0, 1, 2],
            opposite: 3,
        },
        FaceDef {
            indices: vec![5, 4, 3],
            opposite: 0,
        },
        FaceDef {
            indices: vec![0, 3, 4, 1],
            opposite: 2,
        },
        FaceDef {
            indices: vec![0, 2, 5, 3],
            opposite: 1,
        },
        FaceDef {
            indices: vec![1, 4, 5, 2],
            opposite: 0,
        },
    ];

    (vertices, faces)
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: hinge constraint
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: BallJoint constraints
// ═══════════════════════════════════════════════════════════════════════════

/// World-anchored BallJoint with a 5 kg weight hanging 1 m below.
/// Exercises BallJoint under gravity with no angular constraints.
#[derive(Debug, Clone)]
pub struct PendulumBallJointSettlesScenario {
    /// Mass of the pendulum weight.
    pub weight_mass: f32,
    /// Distance from anchor to weight center.
    pub pendulum_length: f32,
    /// Initial horizontal velocity applied to the weight.
    pub initial_horizontal_speed: f32,
    geometry: EmptyGeometry,
}

impl PendulumBallJointSettlesScenario {
    pub fn new() -> Self {
        Self {
            weight_mass: 5.0,
            pendulum_length: 1.0,
            initial_horizontal_speed: 5.0,
            geometry: EmptyGeometry,
        }
    }
}

impl PhysicsBenchScenario for PendulumBallJointSettlesScenario {
    fn name(&self) -> &'static str {
        "pendulum_ball_joint_settles"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let anchor_y = 3.0;
        let anchor_pos = Point3::new(0.0, anchor_y, 0.0);
        let weight_pos = Point3::new(0.0, anchor_y - self.pendulum_length, 0.0);

        let sphere_radius: f32 = 0.15;
        let sphere_volume = (4.0 / 3.0) * std::f32::consts::PI * sphere_radius.powi(3);
        let sphere_density = self.weight_mass / sphere_volume;

        let weight = world.create_body(
            RigidBodyDesc::dynamic()
                .position(weight_pos)
                .linear_damping(0.5)
                .angular_damping(0.05),
        );
        let _ = world.attach_collider(
            weight,
            ColliderDesc::sphere(sphere_radius)
                .density(sphere_density)
                .restitution(0.0)
                .friction(0.5),
        );

        let _ = world.create_constraint(ConstraintKind::world_ball_joint(
            weight,
            anchor_pos,
            Vector3::new(0.0, self.pendulum_length, 0.0),
            0.0,
            f32::MAX,
        ));

        world.set_body_velocity(
            weight,
            Vector3::new(self.initial_horizontal_speed, 0.0, 0.0),
            Vector3::zeros(),
        );

        weight
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: Hinge constraints
// ═══════════════════════════════════════════════════════════════════════════

/// World-anchored hinge with a plank and a box on one end. The plank tilts
/// under the box's weight and should settle to a steady angle once
/// angular damping dissipates energy.
#[derive(Debug, Clone)]
pub struct HingeSettlesUnderLoadScenario {
    /// Half-extents of the plank.
    pub plank_half_extents: Vector3<f32>,
    /// Density of the plank.
    pub plank_density: f32,
    /// Half-extents of the load box.
    pub box_half_extents: Vector3<f32>,
    /// Mass of the load box.
    pub box_mass: f32,
    geometry: FlatQuadGeometry,
}

impl HingeSettlesUnderLoadScenario {
    pub fn new() -> Self {
        Self {
            plank_half_extents: Vector3::new(2.0, 0.06, 0.3),
            plank_density: 500.0,
            box_half_extents: Vector3::new(0.25, 0.25, 0.25),
            box_mass: 5.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for HingeSettlesUnderLoadScenario {
    fn name(&self) -> &'static str {
        "hinge_settles_under_load"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let plank_he = self.plank_half_extents;
        let box_he = self.box_half_extents;
        let pivot_y = 0.3;

        let plank_pos = Point3::new(0.0, pivot_y, 0.0);
        let plank = world.create_body(
            RigidBodyDesc::dynamic()
                .position(plank_pos)
                .angular_damping(0.2)
                .linear_damping(0.01),
        );
        let _ = world.attach_collider(
            plank,
            ColliderDesc::box_shape(plank_he)
                .density(self.plank_density)
                .restitution(0.0)
                .friction(0.6),
        );

        let _ = world.create_constraint(ConstraintKind::world_hinge(
            plank,
            plank_pos,
            Vector3::zeros(),
            UnitVector3::new_normalize(Vector3::z()),
            &UnitQuaternion::identity(),
            0.0,
            f32::MAX,
        ));

        let box_x = plank_he.x - box_he.x;
        let box_y = pivot_y + plank_he.y + box_he.y + 0.01;
        let box_body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(box_x, box_y, 0.0)));
        let box_volume = box_he.x * box_he.y * box_he.z * 8.0;
        let box_density = self.box_mass / box_volume;
        let _ = world.attach_collider(
            box_body,
            ColliderDesc::box_shape(box_he)
                .density(box_density)
                .restitution(0.0)
                .friction(0.6),
        );

        plank
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// World-anchored hinge with a heavy (10 kg) box. Tests that the hinge holds
/// under sustained gravitational torque.
#[derive(Debug, Clone)]
pub struct HingeHoldsUnderSustainedForceScenario {
    /// Half-extents of the plank.
    pub plank_half_extents: Vector3<f32>,
    /// Density of the plank.
    pub plank_density: f32,
    /// Half-extents of the load box.
    pub box_half_extents: Vector3<f32>,
    /// Mass of the load box.
    pub box_mass: f32,
    geometry: FlatQuadGeometry,
}

impl HingeHoldsUnderSustainedForceScenario {
    pub fn new() -> Self {
        Self {
            plank_half_extents: Vector3::new(2.0, 0.06, 0.3),
            plank_density: 500.0,
            box_half_extents: Vector3::new(0.3, 0.3, 0.3),
            box_mass: 10.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for HingeHoldsUnderSustainedForceScenario {
    fn name(&self) -> &'static str {
        "hinge_holds_under_sustained_force"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let plank_he = self.plank_half_extents;
        let box_he = self.box_half_extents;
        let pivot_y = 0.3;

        let plank_pos = Point3::new(0.0, pivot_y, 0.0);
        let plank = world.create_body(
            RigidBodyDesc::dynamic()
                .position(plank_pos)
                .angular_damping(0.2)
                .linear_damping(0.01),
        );
        let _ = world.attach_collider(
            plank,
            ColliderDesc::box_shape(plank_he)
                .density(self.plank_density)
                .restitution(0.0)
                .friction(0.6),
        );

        let _ = world.create_constraint(ConstraintKind::world_hinge(
            plank,
            plank_pos,
            Vector3::zeros(),
            UnitVector3::new_normalize(Vector3::z()),
            &UnitQuaternion::identity(),
            0.0,
            f32::MAX,
        ));

        let box_x = plank_he.x - box_he.x;
        let box_y = pivot_y + plank_he.y + box_he.y + 0.01;
        let box_body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(box_x, box_y, 0.0)));
        let box_volume = box_he.x * box_he.y * box_he.z * 8.0;
        let box_density = self.box_mass / box_volume;
        let _ = world.attach_collider(
            box_body,
            ColliderDesc::box_shape(box_he)
                .density(box_density)
                .restitution(0.0)
                .friction(0.6),
        );

        plank
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Zero-gravity hinge spinning around the free axis (Z). The locked axes
/// (X, Y) should accumulate no drift; the free axis should maintain its
/// initial angular velocity.
#[derive(Debug, Clone)]
pub struct HingeAxisNoDriftZeroGravityScenario {
    /// Half-extents of the body.
    pub half_extents: Vector3<f32>,
    /// Density of the body.
    pub density: f32,
    /// Initial angular velocity around the free axis (Z).
    pub initial_angular_velocity: f32,
    geometry: EmptyGeometry,
}

impl HingeAxisNoDriftZeroGravityScenario {
    pub fn new() -> Self {
        Self {
            half_extents: Vector3::new(1.0, 0.2, 0.3),
            density: 500.0,
            initial_angular_velocity: 3.0,
            geometry: EmptyGeometry,
        }
    }
}

impl PhysicsBenchScenario for HingeAxisNoDriftZeroGravityScenario {
    fn name(&self) -> &'static str {
        "hinge_axis_no_drift_zero_gravity"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let pos = Point3::new(0.0, 2.0, 0.0);
        let body = world.create_body(RigidBodyDesc::dynamic().position(pos));
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(self.half_extents)
                .density(self.density)
                .restitution(0.0)
                .friction(0.5),
        );

        world.set_body_velocity(
            body,
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, self.initial_angular_velocity),
        );

        let _ = world.create_constraint(ConstraintKind::world_hinge(
            body,
            pos,
            Vector3::zeros(),
            UnitVector3::new_normalize(Vector3::z()),
            &UnitQuaternion::identity(),
            0.0,
            f32::MAX,
        ));

        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Fast sphere skimming the floor into a wall.
///
/// The sphere starts resting on the floor, so the narrowphase generates static
/// floor contacts for it at frame start. It then travels fast enough to need
/// CCD, horizontally into the wall at x=0.
///
/// This is the grenade "bounce off the ground, then pass through the next
/// surface" case: narrowphase ownership established at frame start must not
/// suppress CCD for the rest of the frame, or the sphere tunnels through the
/// wall. The floor contact must also not be treated as a CCD hit, or the
/// sphere is clamped back to its substep-start position and frozen.
#[derive(Debug, Clone)]
pub struct GrazingSphereWallCcdScenario {
    /// Sphere radius. Grenade-sized.
    pub radius: f32,
    /// Horizontal speed toward the wall, well above the CCD activation gate.
    pub speed: f32,
    /// Distance from the wall the sphere starts at.
    pub start_x: f32,
    geometry: WallAndFloorGeometry,
}

impl GrazingSphereWallCcdScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.2,
            speed: 60.0,
            start_x: 4.5,
            geometry: WallAndFloorGeometry::new(10.0, 4.0),
        }
    }
}

impl Default for GrazingSphereWallCcdScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBenchScenario for GrazingSphereWallCcdScenario {
    fn name(&self) -> &'static str {
        "grazing_sphere_wall_ccd"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(self.start_x, self.radius, 0.0))
                .linear_velocity(Vector3::new(-self.speed, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(self.radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.4),
        );
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Grenade-speed sphere against a wall, at a frame long enough to outrun the
/// narrowphase.
///
/// Uses the real grenade parameters: radius 0.2, 20 m/s. At a 1/240 substep
/// that is 0.083 m of travel per substep, below the per-substep CCD gate of
/// `radius * ccd_threshold` = 0.1 m — so this body never activates CCD on the
/// per-substep test. Over 8 substeps it covers 0.67 m per frame while the
/// narrowphase, which samples once per frame, reaches only ~0.3 m ahead of the
/// frame-start centre. Only the frame-level gate catches it.
#[derive(Debug, Clone)]
pub struct GrenadeSpeedWallCcdScenario {
    /// Sphere radius, matching `GrenadeConfig::radius`.
    pub radius: f32,
    /// Throw speed, matching `GrenadeConfig::throw_speed`.
    pub speed: f32,
    /// Distance from the wall the sphere starts at.
    pub start_x: f32,
    geometry: WallAndFloorGeometry,
}

impl GrenadeSpeedWallCcdScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.2,
            speed: 20.0,
            start_x: 5.0,
            geometry: WallAndFloorGeometry::new(10.0, 4.0),
        }
    }
}

impl Default for GrenadeSpeedWallCcdScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBenchScenario for GrenadeSpeedWallCcdScenario {
    fn name(&self) -> &'static str {
        "grenade_speed_wall_ccd"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(self.start_x, 2.0, 0.0))
                .linear_velocity(Vector3::new(-self.speed, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(self.radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.4),
        );
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        // 8 substeps of 1/240 per frame: a 30 fps frame, or a hitch at 60.
        8.0 / 240.0
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: static rigid bodies
// ═══════════════════════════════════════════════════════════════════════════

/// Dynamic sphere dropped onto a static rigid body, with no terrain at all.
///
/// A static body is not level geometry — it is an ordinary collider of infinite
/// mass, and must therefore be reachable through the body-vs-body narrowphase.
/// `EmptyGeometry` removes every other means of support, so the sphere comes to
/// rest only if the static body is genuinely solid.
#[derive(Debug, Clone)]
pub struct SphereOnStaticBodyScenario {
    pub restitution: f32,
    /// Top surface of the static platform.
    pub platform_top: f32,
    geometry: EmptyGeometry,
}

impl SphereOnStaticBodyScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            platform_top: 1.0,
            geometry: EmptyGeometry,
        }
    }
}

impl PhysicsBenchScenario for SphereOnStaticBodyScenario {
    fn name(&self) -> &'static str {
        "sphere_on_static_body"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let platform_half = Vector3::new(2.0, 0.5, 2.0);
        let platform = world.create_body(RigidBodyDesc::static_body().position(Point3::new(
            0.0,
            self.platform_top - platform_half.y,
            0.0,
        )));
        let _ = world.attach_collider(
            platform,
            ColliderDesc::box_shape(platform_half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        let radius = 0.4;
        let sphere = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            self.platform_top + 1.5,
            0.0,
        )));
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        sphere
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Dynamic box resting on a static body that is itself resting on terrain.
///
/// Shock propagation must treat the static body as ground rather than as a
/// stackable node, or the box is ordered as if its support were itself
/// supported and the stack solves in the wrong sequence.
#[derive(Debug, Clone)]
pub struct BoxOnStaticPlatformScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl BoxOnStaticPlatformScenario {
    pub fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for BoxOnStaticPlatformScenario {
    fn name(&self) -> &'static str {
        "box_on_static_platform"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let platform_half = Vector3::new(2.0, 0.5, 2.0);
        let platform =
            world.create_body(RigidBodyDesc::static_body().position(Point3::new(0.0, 0.5, 0.0)));
        let _ = world.attach_collider(
            platform,
            ColliderDesc::box_shape(platform_half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.6),
        );

        let box_half = Vector3::new(0.4, 0.4, 0.4);
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            1.0 + box_half.y + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(box_half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.6),
        );

        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: CCD against dynamic bodies
// ═══════════════════════════════════════════════════════════════════════════

/// A grenade-speed sphere fired at a thin dynamic slab, in an empty world.
///
/// This is the pendulum arm reduced to its essentials. The slab is dynamic, so
/// it is invisible to a CCD stage that only sweeps against `StaticGeometry`,
/// and the once-per-frame narrowphase is the sphere's only other chance to see
/// it. At 20 m/s the sphere covers 0.67 m per 30 Hz frame while the detection
/// window is barely 0.28 m wide, so discrete detection cannot close the gap:
/// the sphere passes clean through.
///
/// The slab is made heavy rather than static because a static slab would be
/// caught by a fix that only widened CCD to static *bodies*. It must be an
/// ordinary dynamic body for this to test what it claims to.
#[derive(Debug, Clone)]
pub struct SphereThroughDynamicSlabScenario {
    /// Sphere radius, matching `GrenadeConfig::radius`.
    pub radius: f32,
    /// Throw speed, matching `GrenadeConfig::throw_speed`.
    pub speed: f32,
    /// Distance from the slab the sphere starts at.
    pub start_x: f32,
    /// Half-extents of the slab. Thin along the sphere's line of travel.
    pub slab_half_extents: Vector3<f32>,
    geometry: EmptyGeometry,
}

impl SphereThroughDynamicSlabScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.2,
            speed: 20.0,
            start_x: 3.0,
            slab_half_extents: Vector3::new(0.06, 1.0, 0.06),
            geometry: EmptyGeometry,
        }
    }

    /// Far face of the slab: the plane the sphere must never cross.
    pub fn far_face_x(&self) -> f32 {
        -self.slab_half_extents.x
    }
}

impl Default for SphereThroughDynamicSlabScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBenchScenario for SphereThroughDynamicSlabScenario {
    fn name(&self) -> &'static str {
        "sphere_through_dynamic_slab"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let slab = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        let _ = world.attach_collider(
            slab,
            ColliderDesc::box_shape(self.slab_half_extents)
                .density(50000.0)
                .restitution(0.2)
                .friction(0.4),
        );

        let sphere = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(self.start_x, 0.0, 0.0))
                .linear_velocity(Vector3::new(-self.speed, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(self.radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.4),
        );
        sphere
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        // 8 substeps of 1/240 per frame: a 30 fps frame, or a hitch at 60.
        8.0 / 240.0
    }
}

/// A sphere fired at two thin dynamic slabs standing one behind the other,
/// close enough that a single frame's travel spans both.
///
/// The sphere can only be stopped once. If the impacts are resolved in the
/// order the broadphase happened to emit them, the far slab's later time of
/// impact wins and the sphere is placed *past* the near slab it should have
/// bounced off, with both impulses applied.
#[derive(Debug, Clone)]
pub struct SphereThroughTwoSlabsScenario {
    pub radius: f32,
    pub speed: f32,
    pub start_x: f32,
    pub slab_half_extents: Vector3<f32>,
    /// X of the slab the sphere reaches first.
    pub near_slab_x: f32,
    /// X of the slab behind it.
    pub far_slab_x: f32,
    geometry: EmptyGeometry,
}

impl SphereThroughTwoSlabsScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.2,
            speed: 20.0,
            start_x: 3.0,
            slab_half_extents: Vector3::new(0.06, 1.0, 0.06),
            near_slab_x: 0.0,
            far_slab_x: -0.18,
            geometry: EmptyGeometry,
        }
    }

    /// Far face of the near slab: the plane the sphere must never cross.
    pub fn near_slab_far_face_x(&self) -> f32 {
        self.near_slab_x - self.slab_half_extents.x
    }
}

impl Default for SphereThroughTwoSlabsScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBenchScenario for SphereThroughTwoSlabsScenario {
    fn name(&self) -> &'static str {
        "sphere_through_two_slabs"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        for x in [self.near_slab_x, self.far_slab_x] {
            let slab =
                world.create_body(RigidBodyDesc::dynamic().position(Point3::new(x, 0.0, 0.0)));
            let _ = world.attach_collider(
                slab,
                ColliderDesc::box_shape(self.slab_half_extents)
                    .density(50000.0)
                    .restitution(0.2)
                    .friction(0.4),
            );
        }

        let sphere = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(self.start_x, 0.0, 0.0))
                .linear_velocity(Vector3::new(-self.speed, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(self.radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.4),
        );
        sphere
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        8.0 / 240.0
    }
}

/// A sphere fired diagonally into two overlapping dynamic slabs.
///
/// The slabs form a wall and a floor crossing at the origin, so the sphere
/// arrives at an angle to both and to neither's face normal. It covers the
/// oblique case the head-on scenarios do not: the swept normal is not aligned
/// with the motion, and the clamp has to place the sphere against a face it is
/// sliding along as much as driving into.
#[derive(Debug, Clone)]
pub struct SphereIntoDynamicCornerScenario {
    pub radius: f32,
    pub speed: f32,
    pub slab_half_thickness: f32,
    geometry: EmptyGeometry,
}

impl SphereIntoDynamicCornerScenario {
    pub fn new() -> Self {
        Self {
            radius: 0.2,
            speed: 20.0,
            slab_half_thickness: 0.06,
            geometry: EmptyGeometry,
        }
    }

    /// The far face of either slab: neither may be crossed.
    pub fn far_face(&self) -> f32 {
        -self.slab_half_thickness
    }
}

impl Default for SphereIntoDynamicCornerScenario {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsBenchScenario for SphereIntoDynamicCornerScenario {
    fn name(&self) -> &'static str {
        "sphere_into_dynamic_corner"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let t = self.slab_half_thickness;
        // A wall in the YZ plane and a floor in the XZ plane, meeting at the
        // origin. The sphere arrives along their bisector.
        //
        // Both are colliders on one body. They cross, so as two bodies they
        // would start deeply interpenetrating and the solver's first job would
        // be to shove them out of each other -- and out of the sphere's path,
        // leaving nothing at the origin for it to hit.
        let corner = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        for half in [Vector3::new(t, 1.0, 1.0), Vector3::new(1.0, t, 1.0)] {
            let _ = world.attach_collider(
                corner,
                ColliderDesc::box_shape(half)
                    .density(50000.0)
                    .restitution(0.2)
                    .friction(0.4),
            );
        }

        let sphere = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(2.0, 2.0, 0.0))
                .linear_velocity(Vector3::new(-self.speed, -self.speed, 0.0)),
        );
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(self.radius)
                .density(1000.0)
                .restitution(0.2)
                .friction(0.4),
        );
        sphere
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        8.0 / 240.0
    }
}

/// Two shapes closing inside the speculative band: fast enough to step past the
/// contact margin in a substep, too slow for either CCD gate to fire.
///
/// The band is narrow and frame-rate dependent. At 60 Hz a 0.2 m sphere at
/// 12 m/s travels 0.05 m per substep — past the 0.04 m margin gate — while its
/// 0.2 m frame travel stays under the 0.3 m frame-coverage gate. Speculative
/// contacts are the only mechanism covering it.
#[derive(Debug, Clone)]
pub struct SpeculativeBandApproachScenario {
    pub radius: f32,
    pub speed: f32,
    pub start_x: f32,
    /// When set, the pair is boxes rather than spheres.
    pub boxes: bool,
    geometry: EmptyGeometry,
}

impl SpeculativeBandApproachScenario {
    pub fn spheres() -> Self {
        Self {
            radius: 0.2,
            speed: 12.0,
            start_x: 1.5,
            boxes: false,
            geometry: EmptyGeometry,
        }
    }

    pub fn boxes() -> Self {
        Self {
            boxes: true,
            ..Self::spheres()
        }
    }

    /// Closest the two centres may come: the shapes must not interpenetrate.
    pub fn min_separation(&self) -> f32 {
        2.0 * self.radius
    }
}

impl PhysicsBenchScenario for SpeculativeBandApproachScenario {
    fn name(&self) -> &'static str {
        "speculative_band_approach"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.gravity = Vector3::zeros();
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let mut spawn = |x: f32, vx: f32| {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(x, 0.0, 0.0))
                    .linear_velocity(Vector3::new(vx, 0.0, 0.0)),
            );
            let shape = if self.boxes {
                ColliderDesc::box_shape(Vector3::repeat(self.radius))
            } else {
                ColliderDesc::sphere(self.radius)
            };
            let _ =
                world.attach_collider(body, shape.density(1000.0).restitution(0.0).friction(0.4));
            body
        };
        spawn(-self.start_x, self.speed);
        spawn(self.start_x, -self.speed)
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        // 4 substeps of 1/240: a 60 fps frame.
        4.0 / 240.0
    }
}

/// A grid of resting boxes with fast projectiles crossing it.
///
/// The shape of world CCD costs the most in: many colliders that are not
/// candidates, and a handful that are. Every collider becomes a sweep entry
/// each substep a candidate exists, so this is where that price shows up.
#[derive(Debug, Clone)]
pub struct CcdStressScenario {
    pub grid_size: usize,
    pub projectiles: usize,
    pub speed: f32,
    geometry: FlatQuadGeometry,
}

impl CcdStressScenario {
    pub fn new(grid_size: usize, projectiles: usize) -> Self {
        Self {
            grid_size,
            projectiles,
            speed: 25.0,
            geometry: FlatQuadGeometry::new(40.0),
        }
    }
}

impl PhysicsBenchScenario for CcdStressScenario {
    fn name(&self) -> &'static str {
        "ccd_stress"
    }

    fn restitution(&self) -> f32 {
        0.2
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let half = 0.4;
        let spacing = 1.2;
        let offset = (self.grid_size as f32 - 1.0) * spacing * 0.5;
        for ix in 0..self.grid_size {
            for iz in 0..self.grid_size {
                let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
                    ix as f32 * spacing - offset,
                    half,
                    iz as f32 * spacing - offset,
                )));
                let _ = world.attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::repeat(half))
                        .density(500.0)
                        .restitution(0.2)
                        .friction(0.5),
                );
            }
        }

        let mut tracked = None;
        for i in 0..self.projectiles {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(offset + 6.0, 0.6 + i as f32 * 0.35, 0.0))
                    .linear_velocity(Vector3::new(-self.speed, 0.0, 0.0)),
            );
            let _ = world.attach_collider(
                body,
                ColliderDesc::sphere(0.2)
                    .density(1000.0)
                    .restitution(0.2)
                    .friction(0.4),
            );
            tracked.get_or_insert(body);
        }
        tracked.expect("stress scenario needs at least one projectile")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        8.0 / 240.0
    }
}
