use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3};

use super::framework::PhysicsBenchScenario;
use super::geometry::*;
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
