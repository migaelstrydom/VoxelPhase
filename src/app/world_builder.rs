use specs::{World, WorldExt};

use crate::animation::{AnimationDebugConfig, CharacterAnimator};
use crate::camera::{CameraConfig, FollowTarget};
use crate::character::grab::GrabConfig;
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    CameraComponent, MaterialModulation, ModelInstance, Orientation, Position, Renderable,
    RigidBodyComponent, Rotation, TerrainAnchored, Velocity, VelocityDriven,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Perception, Roller};
use crate::damage::{DamageQueue, Dead, Health, LastVelocity, Ragdoll};
use crate::debug::{DebugConfig, DebugLines, DebugLog, DebugOverlays};
use crate::explosion::Explosion;
use crate::fire::components::{Flammable, OnFire};
use crate::fire::light::FireLight;
use crate::fracture::CompoundFracture;
use crate::input::{GameplayActions, InputState};
use crate::lighting::{ActiveLights, PointLight};
use crate::particles::{ParticleConfig, ParticleEmitter, ParticlePool};
use crate::physics::PhysicsImpulseQueue;
use crate::player::Player;
use crate::projectile::{
    Grenade, GrenadeConfig, GrenadeCooldown, GrenadeModelResource, Lifetime, Projectile,
};
use crate::rendering::material::MaterialManager;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::{FrameStart, PhysicsResource};
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// Builds and configures the ECS World with all components and resources
pub struct WorldBuilder {
    world: World,
}

impl WorldBuilder {
    pub fn new() -> Self {
        let mut world = World::new();

        Self::register_components(&mut world);

        Self { world }
    }

    fn register_components(world: &mut World) {
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Rotation>();
        world.register::<Orientation>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Player>();
        world.register::<CharacterIntent>();
        world.register::<CharacterState>();
        world.register::<LocomotionConfig>();
        world.register::<Grounding>();
        world.register::<Health>();
        world.register::<Dead>();
        world.register::<Ragdoll>();
        world.register::<LastVelocity>();
        world.register::<Brain>();
        world.register::<Perception>();
        world.register::<Roller>();
        world.register::<CharacterAnimator>();
        world.register::<FollowTarget>();
        world.register::<SensorSet>();
        world.register::<ContactCandidates>();
        world.register::<RigidBodyComponent>();
        world.register::<VelocityDriven>();
        world.register::<Grenade>();
        world.register::<Lifetime>();
        world.register::<Projectile>();
        world.register::<Explosion>();
        world.register::<ParticleEmitter>();
        world.register::<Flammable>();
        world.register::<OnFire>();
        world.register::<CompoundFracture>();
        world.register::<TerrainAnchored>();
        world.register::<PointLight>();
        world.register::<MaterialModulation>();
        world.register::<FireLight>();
    }

    pub fn with_renderer(mut self, renderer: Renderer) -> Self {
        self.world.insert(renderer);
        self
    }

    pub fn with_resource_manager(mut self, resource_manager: ResourceManager) -> Self {
        self.world.insert(resource_manager);
        self
    }

    pub fn with_texture_manager(mut self, texture_manager: TextureManager) -> Self {
        self.world.insert(texture_manager);
        self
    }

    pub fn with_material_manager(mut self, material_manager: MaterialManager) -> Self {
        self.world.insert(material_manager);
        self
    }

    pub fn with_terrain(mut self, terrain_manager: TerrainWorld) -> Self {
        self.world.insert(terrain_manager);
        self
    }

    pub fn with_grenade_model(mut self, grenade_model: GrenadeModelResource) -> Self {
        self.world.insert(grenade_model);
        self
    }

    pub fn with_default_resources(mut self) -> Self {
        self.world.insert(Time::new());
        self.world.insert(InputState::new());
        self.world.insert(GameplayActions::default());
        self.world.insert(GrabConfig::default());
        self.world.insert(CameraConfig::default());
        self.world.insert(DebugConfig::default());
        self.world.insert(AnimationDebugConfig::default());
        self.world.insert(DebugLines::default());
        self.world.insert(DebugLog::default());
        self.world.insert(DebugOverlays::default());
        self.world.insert(GrenadeConfig::default());
        self.world.insert(GrenadeCooldown::default());
        self.world.insert(ParticleConfig::new());
        self.world.insert(ParticlePool::default());
        self.world.insert(PhysicsResource::default());
        self.world.insert(FrameStart::default());
        self.world.insert(PhysicsImpulseQueue::default());
        self.world.insert(ActiveLights::default());
        self.world.insert(DamageQueue::default());
        self
    }

    pub fn build(self) -> EngineResult<World> {
        Ok(self.world)
    }
}

impl Default for WorldBuilder {
    fn default() -> Self {
        Self::new()
    }
}
