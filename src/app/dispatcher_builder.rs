use specs::{Dispatcher, DispatcherBuilder};

use crate::biped::{BipedAnimationSystem, BipedProbeConfigSystem};
use crate::explosion::ExplosionSystem;
use crate::input::InputActionSystem;
use crate::particles::{ParticleSpawnSystem, ParticleUpdateSystem};
use crate::projectile::{GrenadeSpawnSystem, LifetimeSystem, ProjectileImpactDetectionSystem};
use crate::sensing::SensorProbeSystem;
use crate::systems::{
    CameraControlSystem, GravitySystem, MotionPredictionSystem, PhysicsSyncSystem,
    PlayerInputSystem, PlayerMotionSystem, RenderSystem, TerrainUpdateSystem,
    VelocityIntegrationSystem,
};

/// Builds the system dispatcher with proper dependency ordering.
pub fn build_dispatcher<'a, 'b>() -> Dispatcher<'a, 'b> {
    DispatcherBuilder::new()
        // === Phase 1: Input processing ===
        .with(InputActionSystem, "input_actions", &[])
        .with(PlayerInputSystem, "player_input", &["input_actions"])
        .with(PlayerMotionSystem, "player_motion", &["player_input"])
        // Player motion integration (kinematic body synced into physics)
        .with(GravitySystem, "gravity", &["player_motion"])
        .with(
            VelocityIntegrationSystem,
            "velocity_integration",
            &["gravity"],
        )
        .with(
            MotionPredictionSystem,
            "motion_prediction",
            &["velocity_integration"],
        )
        // Physics engine for dynamic bodies (beach balls, etc.)
        .with(
            PhysicsSyncSystem::default(),
            "physics_sync",
            &["motion_prediction"],
        )
        // Animation and sensing
        .with(BipedAnimationSystem, "biped_animation", &["physics_sync"])
        .with(
            BipedProbeConfigSystem,
            "biped_probe_config",
            &["biped_animation"],
        )
        .with(SensorProbeSystem, "sensor_probe", &["biped_probe_config"])
        .with(CameraControlSystem, "camera_control", &["physics_sync"])
        // Projectiles and explosions
        .with(GrenadeSpawnSystem, "grenade_spawn", &["camera_control"])
        .with(LifetimeSystem, "lifetime", &["grenade_spawn"])
        .with(
            ProjectileImpactDetectionSystem,
            "projectile_impact_detection",
            &["lifetime"],
        )
        .with(
            ExplosionSystem,
            "explosion",
            &["projectile_impact_detection"],
        )
        .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
        // Particles
        .with(ParticleSpawnSystem, "particle_spawn", &["explosion"])
        .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
        // Rendering (thread-local)
        .with_thread_local(RenderSystem)
        .build()
}
