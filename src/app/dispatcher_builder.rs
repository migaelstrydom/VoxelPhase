use specs::{Dispatcher, DispatcherBuilder};

use crate::biped::{BipedAnimationSystem, BipedProbeConfigSystem};
use crate::explosion::ExplosionSystem;
use crate::input::InputActionSystem;
use crate::particles::{ParticleSpawnSystem, ParticleUpdateSystem};
use crate::projectile::{GrenadeSpawnSystem, LifetimeSystem, ProjectileCollisionSystem};
use crate::sensing::SensorProbeSystem;
use crate::systems::{
    CameraControlSystem, DynamicTerrainCollisionSystem, GravitySystem, MotionPredictionSystem,
    PenetrationResolutionSystem, PlayerInputSystem, PlayerMotionSystem, RenderSystem,
    SpringBipedCollisionSystem, TerrainCollisionSystem, TerrainUpdateSystem,
    VelocityIntegrationSystem,
};

/// Builds the system dispatcher with proper dependency ordering.
pub fn build_dispatcher<'a, 'b>() -> Dispatcher<'a, 'b> {
    DispatcherBuilder::new()
        // === Phase 1: Input processing ===
        .with(InputActionSystem, "input_actions", &[])
        .with(PlayerInputSystem, "player_input", &["input_actions"])
        .with(PlayerMotionSystem, "player_motion", &["player_input"])
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
        // === Phase 7: Collision resolution ===
        .with(
            SpringBipedCollisionSystem,
            "spring_biped_collision",
            &["motion_prediction"],
        )
        .with(
            DynamicTerrainCollisionSystem,
            "dynamic_terrain_collision",
            &["motion_prediction"],
        )
        .with(
            TerrainCollisionSystem,
            "terrain_collision",
            &["spring_biped_collision", "dynamic_terrain_collision"],
        )
        .with(
            PenetrationResolutionSystem,
            "penetration_resolution",
            &["terrain_collision"],
        )
        .with(
            BipedAnimationSystem,
            "biped_animation",
            &["penetration_resolution"],
        )
        .with(
            BipedProbeConfigSystem,
            "biped_probe_config",
            &["biped_animation"],
        )
        .with(SensorProbeSystem, "sensor_probe", &["biped_probe_config"])
        // === Phase 8: Camera ===
        .with(
            CameraControlSystem,
            "camera_control",
            &["penetration_resolution"],
        )
        // === Phase 9: Projectiles and effects ===
        .with(GrenadeSpawnSystem, "grenade_spawn", &["camera_control"])
        .with(LifetimeSystem, "lifetime", &["grenade_spawn"])
        .with(
            ProjectileCollisionSystem,
            "projectile_collision",
            &["lifetime"],
        )
        .with(ExplosionSystem, "explosion", &["projectile_collision"])
        .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
        .with(ParticleSpawnSystem, "particle_spawn", &["explosion"])
        .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
        // === Phase 10: Rendering (thread-local) ===
        .with_thread_local(RenderSystem)
        .build()
}
