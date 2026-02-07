use specs::{World, WorldExt};

use crate::biped::BipedController;
use crate::camera::{CameraConfig, FollowTarget};
use crate::components::{
    Acceleration, CameraComponent, Collider, Gravity, ModelInstance, MotionState, Orientation,
    PhysicsBody, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::core::error::EngineResult;
use crate::debug::{DebugLines, DebugOverlays};
use crate::explosion::Explosion;
use crate::input::{GameplayActions, InputState};
use crate::particles::{ParticleConfig, ParticleEmitter, ParticlePool};
use crate::player::{Player, PlayerConfig, PlayerTargetState};
use crate::projectile::{
    Grenade, GrenadeConfig, GrenadeCooldown, GrenadeModelResource, Lifetime, Projectile,
};
use crate::physics::PhysicsImpulseQueue;
use crate::rendering::material::MaterialManager;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainManager;
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
        world.register::<Acceleration>();
        world.register::<Gravity>();
        world.register::<Rotation>();
        world.register::<Orientation>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Player>();
        world.register::<PlayerTargetState>();
        world.register::<BipedController>();
        world.register::<FollowTarget>();
        world.register::<Collider>();
        world.register::<MotionState>();
        world.register::<SensorSet>();
        world.register::<ContactCandidates>();
        world.register::<PhysicsBody>();
        world.register::<RigidBodyComponent>();
        world.register::<Grenade>();
        world.register::<Lifetime>();
        world.register::<Projectile>();
        world.register::<Explosion>();
        world.register::<ParticleEmitter>();
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

    pub fn with_terrain(mut self, terrain_manager: TerrainManager) -> Self {
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
        self.world.insert(PlayerConfig::default());
        self.world.insert(CameraConfig::default());
        self.world.insert(DebugLines::default());
        self.world.insert(DebugOverlays::default());
        self.world.insert(GrenadeConfig::default());
        self.world.insert(GrenadeCooldown::default());
        self.world.insert(ParticleConfig::new());
        self.world.insert(ParticlePool::default());
        self.world.insert(PhysicsResource::default());
        self.world.insert(PhysicsImpulseQueue::default());
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
