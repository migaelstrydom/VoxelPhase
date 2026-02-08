//! Physics synchronization system.
//!
//! This system is the single point of integration between the physics engine
//! and the ECS world. It:
//! 1. Steps the physics simulation
//! 2. Syncs physics state back to ECS components

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};
use std::collections::HashSet;

use crate::biped::BipedController;
use crate::components::{Orientation, Position, RigidBodyComponent, Velocity};
use crate::debug::{DebugLines, DebugOverlays};
use crate::physics::{ContactSource, PhysicsImpulseQueue, PhysicsWorld, RigidBodyHandle};
use crate::rendering::Colour;
use crate::terrain::TerrainManager;
use crate::time::Time;

/// Wrapper for PhysicsWorld to use as a specs resource.
#[derive(Default)]
pub struct PhysicsResource(pub PhysicsWorld);

/// Steps physics simulation and syncs state to ECS.
pub struct PhysicsSyncSystem;

impl PhysicsSyncSystem {
    fn sync_kinematics_from_ecs(
        physics: &mut PhysicsWorld,
        positions: &WriteStorage<Position>,
        velocities: &WriteStorage<Velocity>,
        orientations: &WriteStorage<Orientation>,
        bodies: &ReadStorage<RigidBodyComponent>,
    ) {
        let mut kinematic_updates = Vec::new();
        {
            let physics_ref = &*physics;
            for (pos, vel, orient, body) in
                ((&*positions), (&*velocities), (&*orientations), bodies).join()
            {
                if let Some(rb) = physics_ref.body(body.0) {
                    if rb.is_kinematic() {
                        kinematic_updates.push((body.0, pos.0, vel.0, orient.0));
                    }
                }
            }
        }
        for (handle, pos, vel, orient) in kinematic_updates {
            let position = Point3::new(pos.x, pos.y, pos.z);
            let _ = physics.set_kinematic_transform(handle, position, orient);
            let _ = physics.set_kinematic_velocity(handle, vel, Vector3::zeros());
        }
    }

    fn sync_physics_to_ecs(
        physics: &PhysicsWorld,
        positions: &mut WriteStorage<Position>,
        velocities: &mut WriteStorage<Velocity>,
        orientations: &mut WriteStorage<Orientation>,
        bodies: &ReadStorage<RigidBodyComponent>,
    ) {
        for (pos, vel, orient, body) in (positions, velocities, orientations, bodies).join() {
            if let Some(rb) = physics.body(body.0) {
                pos.0 = Vector3::new(rb.position().x, rb.position().y, rb.position().z);
                vel.0 = rb.linear_velocity();
                orient.0 = rb.rotation();
            }
        }
    }

    fn apply_grounded_state(
        grounded_handles: &HashSet<RigidBodyHandle>,
        bodies: &ReadStorage<RigidBodyComponent>,
        controllers: &mut WriteStorage<BipedController>,
    ) {
        for (body, controller) in (bodies, controllers).join() {
            controller.state.is_grounded = grounded_handles.contains(&body.0);
        }
    }

    fn add_contact_overlays(
        physics: &PhysicsWorld,
        overlays: &mut DebugOverlays,
        debug_lines: &mut DebugLines,
    ) {
        if !physics.config().debug_draw_contacts {
            return;
        }
        let normal_scale = 0.3;
        let mut smoothed_count = 0usize;
        let mut max_angle_deg = 0.0f32;

        for contact in physics.contact_events() {
            let colour = match contact.source {
                ContactSource::Narrowphase => Colour::RED,
                ContactSource::Ccd => Colour::BLUE,
            };
            overlays.add_sphere(contact.point, 0.06, colour);
            if contact.normal.magnitude_squared() > 1e-8 {
                overlays.add_line(
                    contact.point,
                    contact.point + contact.normal * normal_scale,
                    colour,
                );
            }
            if physics.config().debug_draw_contact_raw_normals
                && contact.source == ContactSource::Narrowphase
                && contact.raw_normal.magnitude_squared() > 1e-8
            {
                let dot = contact.normal.dot(&contact.raw_normal).clamp(-1.0, 1.0);
                let angle_deg = dot.acos() * 180.0 / std::f32::consts::PI;
                if angle_deg > 0.01 {
                    smoothed_count += 1;
                    max_angle_deg = max_angle_deg.max(angle_deg);
                    overlays.add_line(
                        contact.point,
                        contact.point + contact.raw_normal * normal_scale,
                        Colour::YELLOW,
                    );
                }
            }
        }

        if physics.config().debug_draw_contact_raw_normals {
            debug_lines.add("Contacts/Smoothed", smoothed_count.to_string());
            debug_lines.add(
                "Contacts/MaxNormalDeltaDeg",
                format!("{:.3}", max_angle_deg),
            );
        }
    }

    fn add_sleep_overlays(physics: &PhysicsWorld, overlays: &mut DebugOverlays) {
        if !physics.config().debug_draw_sleeping {
            return;
        }
        let colour = Colour::new(0.6, 0.65, 1.0, 1.0);
        for handle in physics.sleeping_bodies() {
            if let Some(body) = physics.body(handle) {
                let pos = body.position();
                let marker_pos = Point3::new(pos.x, pos.y + 0.6, pos.z);
                overlays.add_sphere(marker_pos, 0.08, colour);
            }
        }
    }
}

impl<'a> System<'a> for PhysicsSyncSystem {
    type SystemData = (
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainManager>>,
        Read<'a, Time>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, BipedController>,
        Write<'a, DebugLines>,
        Write<'a, DebugOverlays>,
        Write<'a, PhysicsImpulseQueue>,
    );

    fn run(
        &mut self,
        (
            mut physics,
            terrain_opt,
            time,
            mut positions,
            mut velocities,
            mut orientations,
            bodies,
            mut controllers,
            mut debug_lines,
            mut debug_overlays,
            mut impulse_queue,
        ): Self::SystemData,
    ) {
        let dt = time.delta_seconds();

        // Sync kinematic bodies from ECS into physics before stepping.
        Self::sync_kinematics_from_ecs(
            &mut physics.0,
            &positions,
            &velocities,
            &orientations,
            &bodies,
        );

        for impulse in impulse_queue.drain() {
            physics.0.apply_radial_impulse(
                impulse.center,
                impulse.radius,
                impulse.strength,
                impulse.upward_boost,
            );
        }

        // Step physics with terrain as static geometry
        if let Some(ref terrain) = terrain_opt {
            physics.0.step(dt, &**terrain, &mut debug_lines);
        }

        Self::add_contact_overlays(&physics.0, &mut debug_overlays, &mut debug_lines);
        Self::add_sleep_overlays(&physics.0, &mut debug_overlays);

        let grounded_handles = physics.0.grounded_handles();

        // Sync physics state back to ECS components
        Self::sync_physics_to_ecs(
            &physics.0,
            &mut positions,
            &mut velocities,
            &mut orientations,
            &bodies,
        );

        Self::apply_grounded_state(&grounded_handles, &bodies, &mut controllers);
    }
}
