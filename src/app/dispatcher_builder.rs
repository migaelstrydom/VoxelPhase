use specs::{Dispatcher, DispatcherBuilder};

use crate::aim::AimPredictionSystem;
use crate::animation::critter::{CritterAnimationSystem, CritterProbeConfigSystem};
use crate::animation::peeper::{PeeperAnimationSystem, PeeperProbeConfigSystem};
use crate::animation::{AnimationProbeConfigSystem, CharacterAnimationSystem};
use crate::character::ContactGroundingSystem;
use crate::creature::{
    AlertTelegraphSystem, BrainSystem, CollectionSystem, MeleeAttackSystem, PerceptionSystem,
    RollerLocomotionSystem,
};
use crate::damage::{
    BlastDamageSystem, BurnDamageSystem, DamageApplySystem, DeathSystem, ImpactDamageSystem,
};
use crate::explosion::{BlastLightSystem, ExplosionSystem};
use crate::fire::light::FireLightSystem;
use crate::fire::systems::{FireCleanupSystem, FireIgnitionSystem};
use crate::fracture::{DebrisCullSystem, FractureSystem};
use crate::glass::GlassCrackSystem;
use crate::input::InputActionSystem;
use crate::lighting::LightCollectionSystem;
use crate::objective::{GemMotionSystem, GoalSystem, ObjectiveHudSystem, ProgressSystem};
use crate::particles::{ParticleSpawnSystem, ParticleUpdateSystem};
use crate::platform::MovingPlatformSystem;
use crate::projectile::{
    GrenadeSpawnSystem, GrenadeVisualSystem, LifetimeSystem, ProjectileDetonationSystem,
};
use crate::sensing::SensorProbeSystem;
use crate::systems::{
    CameraControlSystem, CharacterControlSystem, PhysicsSyncSystem, PlayerInputSystem,
    RenderSystem, TerrainAnchorSystem, TerrainUpdateSystem, WaterSystem,
};

/// Builds the system dispatcher with proper dependency ordering.
pub fn build_dispatcher<'a, 'b>() -> Dispatcher<'a, 'b> {
    DispatcherBuilder::new()
        // === Phase 1: Input processing ===
        .with(InputActionSystem, "input_actions", &[])
        .with(PlayerInputSystem, "player_input", &["input_actions"])
        // Creature AI fills the same intent the keyboard does, so it belongs in
        // the input phase alongside PlayerInputSystem — both must land before
        // character_control consumes intent. Perception reads last frame's
        // synced positions, which is the same one-frame-old world the player
        // is reacting to on screen.
        .with(PerceptionSystem, "perception", &["input_actions"])
        .with(BrainSystem, "brain", &["perception"])
        // Rolling creatures consume the same intent the walk FSM does, just
        // as torque instead of velocity. Must land before physics_sync.
        // A swing is decided by the brain and timed here, so the blow it
        // throws is in the queue before `damage_apply` drains it.
        .with(MeleeAttackSystem, "melee_attack", &["brain"])
        .with(AlertTelegraphSystem, "alert_telegraph", &["brain"])
        .with(RollerLocomotionSystem, "roller_locomotion", &["brain"])
        .with(
            CharacterControlSystem,
            "character_control",
            &["player_input", "brain"],
        )
        // Platform motors state their intent as a velocity, exactly as the
        // character systems do, and must land before physics_sync turns it
        // into a drive target.
        .with(MovingPlatformSystem, "moving_platform", &["input_actions"])
        // Physics engine (buoyancy forces computed per-substep internally)
        .with(
            PhysicsSyncSystem::default(),
            "physics_sync",
            &[
                "character_control",
                "roller_locomotion",
                "alert_telegraph",
                "moving_platform",
            ],
        )
        // Brittle sheets craze under this frame's impacts first, so that a
        // hit on a pane is judged against a web of shards and not one slab.
        .with(GlassCrackSystem, "glass_crack", &["physics_sync"])
        // Compound body fracture (uses solver impulses from this frame)
        .with(FractureSystem, "fracture", &["physics_sync", "glass_crack"])
        // Pieces the fracture freed and the level's budget will not keep.
        .with(DebrisCullSystem, "debris_cull", &["fracture"])
        // Animation and sensing. Probes are configured from last-frame
        // animator state, fired against this-frame physics, and consumed
        // by the animator in the same frame — no one-frame lag on ground
        // contacts.
        .with(
            AnimationProbeConfigSystem,
            "animation_probe_config",
            &["physics_sync"],
        )
        // Critters aim their own foot probes from the same seam, into the
        // same sensor set the humanoid uses. Both must land before the
        // probes are fired.
        .with(
            CritterProbeConfigSystem,
            "critter_probe_config",
            &["physics_sync"],
        )
        .with(
            PeeperProbeConfigSystem,
            "peeper_probe_config",
            &["physics_sync"],
        )
        .with(
            SensorProbeSystem,
            "sensor_probe",
            &[
                "animation_probe_config",
                "critter_probe_config",
                "peeper_probe_config",
            ],
        )
        // The one writer of `Grounding`, for every character. Runs after
        // physics, on this frame's contacts; the animator reads it in the same
        // frame and `character_control` at the top of the next.
        .with(
            ContactGroundingSystem,
            "contact_grounding",
            &["physics_sync"],
        )
        .with(
            CharacterAnimationSystem,
            "character_animation",
            &["sensor_probe", "contact_grounding"],
        )
        .with(
            CritterAnimationSystem,
            "critter_animation",
            &["sensor_probe", "contact_grounding"],
        )
        // A peeper's eye and neck are posed from what the brain and the
        // swing decided earlier in the frame, so this also runs after both.
        .with(
            PeeperAnimationSystem,
            "peeper_animation",
            &["sensor_probe", "contact_grounding", "melee_attack"],
        )
        // Catching happens on this frame's synced positions, so a critter
        // caught at a sprint is caught where the player saw it.
        .with(CollectionSystem, "collection", &["physics_sync"])
        // The objective, in the order it is decided: the clock runs, gems bob
        // into reach and are caught, the goal reads the resulting count, and
        // the HUD draws whatever all three left behind.
        .with(ProgressSystem, "objective_progress", &["physics_sync"])
        .with(GemMotionSystem, "gem_motion", &["physics_sync"])
        .with(
            GoalSystem,
            "goal",
            &["collection", "objective_progress", "physics_sync"],
        )
        .with(
            ObjectiveHudSystem,
            "objective_hud",
            &["goal", "objective_progress"],
        )
        .with(CameraControlSystem, "camera_control", &["physics_sync"])
        // Where a throw would land, predicted from this frame's camera and
        // this frame's arm state — the same two things the throw itself reads,
        // so the cursor and the grenade cannot disagree.
        .with(
            AimPredictionSystem::default(),
            "aim_prediction",
            &["camera_control", "character_control"],
        )
        // Projectiles and explosions
        .with(GrenadeSpawnSystem, "grenade_spawn", &["camera_control"])
        .with(LifetimeSystem, "lifetime", &["grenade_spawn"])
        // Grenade glow tracks velocity, so it runs after physics has synced it
        // and before lights are collected for the frame.
        .with(
            GrenadeVisualSystem::default(),
            "grenade_visuals",
            &["lifetime"],
        )
        .with(
            ProjectileDetonationSystem,
            "projectile_detonation",
            &["lifetime"],
        )
        .with(
            ExplosionSystem::default(),
            "explosion",
            &["projectile_detonation"],
        )
        // The blast flash is the brightest light in the game and the shortest
        // lived, so it must be updated after explosions create it and before
        // the frame's lights are chosen.
        .with(BlastLightSystem, "blast_light", &["explosion"])
        .with(FireIgnitionSystem, "fire_ignition", &["explosion"])
        .with(FireCleanupSystem, "fire_cleanup", &["fire_ignition"])
        // Firelight tracks OnFire, so it must settle before lights are collected.
        .with(FireLightSystem, "fire_light", &["fire_cleanup"])
        // Damage. Each source runs after whatever produces the thing that
        // hurts: the blast reader must see explosions before
        // `world.maintain()` deletes them, and the burn reader must see this
        // frame's ignitions and burn-outs. Apply then drains all three at
        // once, so a frame's blast, burn and impact land together and only one
        // can be the killing blow.
        .with(BlastDamageSystem, "blast_damage", &["explosion"])
        .with(BurnDamageSystem, "burn_damage", &["fire_cleanup"])
        .with(ImpactDamageSystem, "impact_damage", &["physics_sync"])
        .with(
            DamageApplySystem,
            "damage_apply",
            &[
                "blast_damage",
                "burn_damage",
                "impact_damage",
                "melee_attack",
            ],
        )
        .with(DeathSystem, "death", &["damage_apply"])
        .with(TerrainAnchorSystem, "terrain_anchor", &["explosion"])
        .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
        // Water simulation (after terrain update so dirty_regions are visible)
        .with(WaterSystem, "water", &["terrain_update"])
        // Particles
        .with(
            ParticleSpawnSystem,
            "particle_spawn",
            // Grenade visuals set their trail's spawn rate for the frame.
            &["explosion", "grenade_visuals"],
        )
        .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
        // Per-frame point light selection. Must see the final camera position
        // and every light attached this frame, so it runs after both the camera
        // and the systems that create or remove lights. RenderSystem is
        // thread-local, so it always runs after this.
        .with(
            LightCollectionSystem::default(),
            "light_collection",
            &[
                "camera_control",
                "fire_light",
                "grenade_visuals",
                "blast_light",
            ],
        )
        // Rendering (thread-local)
        .with_thread_local(RenderSystem::default())
        .build()
}

#[cfg(test)]
mod tests {
    use specs::WorldExt;

    use super::*;
    use crate::app::world_builder::WorldBuilder;
    use crate::character::CharacterIntent;
    use crate::creature::{Brain, Perception, Roller};
    use crate::damage::Health;
    use crate::projectile::GrenadeCooldown;

    #[test]
    fn dependency_graph_is_valid() {
        // `DispatcherBuilder::build` panics on a dependency naming a system that
        // does not exist, which is otherwise only discovered by launching the
        // game. Cheap insurance whenever the graph is edited.
        let _ = build_dispatcher();
    }

    /// Every component a system reads must have storage in the world before
    /// anything is spawned, or the first entity carrying it panics on insert.
    ///
    /// `App` gets that by calling `Dispatcher::setup` before spawning the
    /// level; this asserts the same call covers the world `WorldBuilder`
    /// produces. Without it, a component reaching the world only through a
    /// system is a launch-time panic that no test sees — which is exactly how
    /// `Roller` shipped unregistered.
    #[test]
    fn dispatcher_setup_registers_every_system_component() {
        let mut world = WorldBuilder::new()
            .with_default_resources()
            .build()
            .unwrap();
        let mut dispatcher = build_dispatcher();

        dispatcher.setup(&mut world);

        // Spot-check the creature stack: these reach the world only via
        // `app::creatures`, so they are the ones most likely to be forgotten.
        world.write_storage::<Roller>();
        world.write_storage::<Brain>();
        world.write_storage::<Perception>();
        world.write_storage::<Health>();
        world.write_storage::<CharacterIntent>();
    }

    /// The world must survive being built and set up twice over — a cheap
    /// guard against `setup` clobbering resources `WorldBuilder` inserted.
    #[test]
    fn setup_preserves_resources_inserted_by_the_world_builder() {
        let mut world = WorldBuilder::new()
            .with_default_resources()
            .build()
            .unwrap();
        world.write_resource::<GrenadeCooldown>().remaining = 42.0;

        build_dispatcher().setup(&mut world);

        assert_eq!(
            world.read_resource::<GrenadeCooldown>().remaining,
            42.0,
            "setup must fill gaps, not overwrite what init already configured"
        );
    }
}
