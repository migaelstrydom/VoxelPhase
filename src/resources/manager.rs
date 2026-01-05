use std::sync::Arc;

use crate::core::error::EngineResult;
use crate::core::{device::ManagedDevice, vulkan_context::VulkanContext};
use crate::rendering::descriptors::DescriptorManager;

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

    /// Create a texture manager with the given descriptor manager
    pub fn create_texture_manager(
        &self,
        descriptor_manager: Arc<DescriptorManager>,
    ) -> EngineResult<TextureManager> {
        TextureManager::new(
            self.device.clone(),
            self.transfer_service.clone(),
            descriptor_manager,
        )
    }

    /// Access the transfer service
    pub fn transfer_service(&self) -> Arc<TransferService> {
        self.transfer_service.clone()
    }

    /// Access the device
    pub fn device(&self) -> Arc<ManagedDevice> {
        self.device.clone()
    }

    // Future managers:
    // pub fn create_mesh_manager(&self) -> MeshManager {...}
    // pub fn create_shader_manager(&self) -> ShaderManager {...}
    // pub fn create_material_manager(&self) -> MaterialManager {...}
    // etc.
}
