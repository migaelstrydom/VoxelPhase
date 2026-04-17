use specs::{Dispatcher, DispatcherBuilder};

use crate::biped::{BipedAnimationSystem, BipedProbeConfigSystem};
use crate::explosion::ExplosionSystem;
use crate::fire::systems::{FireCleanupSystem, FireIgnitionSystem};
use crate::fracture::FractureSystem;
use crate::input::InputActionSystem;
use crate::particles::{ParticleSpawnSystem, ParticleUpdateSystem};
use crate::projectile::{GrenadeSpawnSystem, LifetimeSystem, ProjectileImpactDetectionSystem};
use crate::sensing::SensorProbeSystem;
use crate::systems::{
    CameraControlSystem, PhysicsSyncSystem, PlayerControlSystem, PlayerInputSystem, RenderSystem,
    TerrainAnchorSystem, TerrainUpdateSystem, WaterSystem,
};

/// Builds the system dispatcher with proper dependency ordering.
pub fn build_dispatcher<'a, 'b>() -> Dispatcher<'a, 'b> {
    DispatcherBuilder::new()
        // === Phase 1: Input processing ===
        .with(InputActionSystem, "input_actions", &[])
        .with(PlayerInputSystem, "player_input", &["input_actions"])
        .with(PlayerControlSystem, "player_control", &["player_input"])
        // Physics engine (buoyancy forces computed per-substep internally)
        .with(
            PhysicsSyncSystem::default(),
            "physics_sync",
            &["player_control"],
        )
        // Compound body fracture (uses solver impulses from this frame)
        .with(FractureSystem, "fracture", &["physics_sync"])
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
        .with(FireIgnitionSystem, "fire_ignition", &["explosion"])
        .with(FireCleanupSystem, "fire_cleanup", &["fire_ignition"])
        .with(TerrainAnchorSystem, "terrain_anchor", &["explosion"])
        .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
        // Water simulation (after terrain update so dirty_regions are visible)
        .with(WaterSystem, "water", &["terrain_update"])
        // Particles
        .with(ParticleSpawnSystem, "particle_spawn", &["explosion"])
        .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
        // Rendering (thread-local)
        .with_thread_local(RenderSystem::default())
        .build()
}
