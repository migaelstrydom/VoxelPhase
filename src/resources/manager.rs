use std::sync::Arc;

use crate::core::error::EngineResult;
use crate::core::{device::ManagedDevice, vulkan_context::VulkanContext};
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::grain;

use super::{textures::TextureManager, transfer_service::TransferService};

/// Central manager for all engine resources
pub struct ResourceManager {
    device: Arc<ManagedDevice>,
    transfer_service: Arc<TransferService>,
}

impl ResourceManager {
    /// Create a new resource manager
    pub fn new(vulkan_context: Arc<VulkanContext>) -> EngineResult<Self> {
        // Create shared services
        let transfer_service = Arc::new(TransferService::new(Arc::clone(&vulkan_context))?);

        Ok(Self {
            device: Arc::clone(&vulkan_context.device),
            transfer_service,
        })
    }

    /// Create a texture manager with the given descriptor manager.
    ///
    /// Also generates the shared grain texture and binds it to the scene
    /// descriptor set. That happens here, rather than at each of the four
    /// places that build a texture manager, because the binding is not
    /// optional: the fragment shader statically samples it, so a renderer whose
    /// binding was never written is invalid whether or not any material asks
    /// for grain. Doing it in the one place every caller funnels through is
    /// what makes forgetting it impossible.
    pub fn create_texture_manager(
        &self,
        descriptor_manager: Arc<DescriptorManager>,
    ) -> EngineResult<TextureManager> {
        let textures = TextureManager::new(
            self.device.clone(),
            self.transfer_service.clone(),
            Arc::clone(&descriptor_manager),
        )?;

        let grain = grain::create_grain_texture(&textures)?;
        descriptor_manager
            .update_grain_texture(grain.texture().image_view, grain.texture().sampler);
        textures.keep_resident(grain);

        Ok(textures)
    }

    // Future managers:
    // pub fn create_mesh_manager(&self) -> MeshManager {...}
    // pub fn create_shader_manager(&self) -> ShaderManager {...}
    // pub fn create_material_manager(&self) -> MaterialManager {...}
    // etc.
}
