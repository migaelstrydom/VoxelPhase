use std::sync::Arc;

use crate::core::{device::ManagedDevice, vulkan_context::VulkanContext};

use super::{textures::TextureManager, transfer_service::TransferService};

/// Central manager for all engine resources
pub struct ResourceManager {
    device: Arc<ManagedDevice>,
    transfer_service: Arc<TransferService>,
}

impl ResourceManager {
    /// Create a new resource manager
    pub fn new(vulkan_context: Arc<VulkanContext>) -> Result<Self, Box<dyn std::error::Error>> {
        // Create shared services
        let transfer_service = Arc::new(TransferService::new(Arc::clone(&vulkan_context))?);

        Ok(Self {
            device: Arc::clone(&vulkan_context.device),
            transfer_service,
        })
    }

    /// Create a texture manager
    pub fn create_texture_manager(&self) -> TextureManager {
        TextureManager::new(self.device.clone(), self.transfer_service.clone())
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
