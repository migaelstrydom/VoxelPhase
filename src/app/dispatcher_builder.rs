use specs::{Dispatcher, DispatcherBuilder};

use crate::animation::{AnimationProbeConfigSystem, CharacterAnimationSystem};
use crate::explosion::ExplosionSystem;
use crate::fire::light::FireLightSystem;
use crate::fire::systems::{FireCleanupSystem, FireIgnitionSystem};
use crate::fracture::FractureSystem;
use crate::input::InputActionSystem;
use crate::lighting::LightCollectionSystem;
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
        // Animation and sensing. Probes are configured from last-frame
        // animator state, fired against this-frame physics, and consumed
        // by the animator in the same frame — no one-frame lag on ground
        // contacts.
        .with(
            AnimationProbeConfigSystem,
            "animation_probe_config",
            &["physics_sync"],
        )
        .with(
            SensorProbeSystem,
            "sensor_probe",
            &["animation_probe_config"],
        )
        .with(
            CharacterAnimationSystem,
            "character_animation",
            &["sensor_probe"],
        )
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
        // Firelight tracks OnFire, so it must settle before lights are collected.
        .with(FireLightSystem, "fire_light", &["fire_cleanup"])
        .with(TerrainAnchorSystem, "terrain_anchor", &["explosion"])
        .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
        // Water simulation (after terrain update so dirty_regions are visible)
        .with(WaterSystem, "water", &["terrain_update"])
        // Particles
        .with(ParticleSpawnSystem, "particle_spawn", &["explosion"])
        .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
        // Per-frame point light selection. Must see the final camera position
        // and every light attached this frame, so it runs after both the camera
        // and the systems that create or remove lights. RenderSystem is
        // thread-local, so it always runs after this.
        .with(
            LightCollectionSystem::default(),
            "light_collection",
            &["camera_control", "fire_light"],
        )
        // Rendering (thread-local)
        .with_thread_local(RenderSystem::default())
        .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dependency_graph_is_valid() {
        // `DispatcherBuilder::build` panics on a dependency naming a system that
        // does not exist, which is otherwise only discovered by launching the
        // game. Cheap insurance whenever the graph is edited.
        let _ = build_dispatcher();
    }
}
