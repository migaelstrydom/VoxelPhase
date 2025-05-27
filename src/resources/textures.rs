use crate::core::device::ManagedDevice;
use crate::rendering::texture::ManagedTexture;
use specs::{Component, DenseVecStorage};
use std::{
    collections::HashMap,
    error::Error,
    path::Path,
    sync::{Arc, Weak},
};

use ash::vk;

use crate::rendering::renderer::ManagedBuffer;

use super::transfer_service::TransferService;

/// Factory for creating textures
pub struct TextureFactory {
    device: Arc<ManagedDevice>,
    transfer_service: Arc<TransferService>,
}

impl TextureFactory {
    /// Create a new texture factory
    pub fn new(device: Arc<ManagedDevice>, transfer_service: Arc<TransferService>) -> Self {
        Self {
            device,
            transfer_service,
        }
    }

    /// Create a texture from an image file
    pub fn create_from_file<P: AsRef<Path>>(
        &self,
        file_path: P,
        generate_mipmaps: Option<bool>,
    ) -> Result<ManagedTexture, Box<dyn Error>> {
        let generate_mipmaps = generate_mipmaps.unwrap_or(true);

        // Load image using the 'image' crate
        let img = image::open(file_path)?;
        let img_rgba = img.to_rgba8();
        let width = img.width();
        let height = img.height();
        let image_data = img_rgba.into_raw();
        let image_size = image_data.len() as u64;

        // Calculate mip levels if mipmaps are requested
        let mip_levels = if generate_mipmaps {
            ((width.max(height) as f32).log2().floor() as u32) + 1
        } else {
            1
        };

        // Create a staging buffer to hold the image data
        let staging_buffer = ManagedBuffer::new(
            self.device.clone(),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        // Copy image data to staging buffer
        unsafe {
            let data_ptr = self.device.device.map_memory(
                staging_buffer.memory,
                0,
                image_size,
                vk::MemoryMapFlags::empty(),
            )? as *mut u8;

            std::ptr::copy_nonoverlapping(image_data.as_ptr(), data_ptr, image_data.len());
            self.device.device.unmap_memory(staging_buffer.memory);
        }

        // Create texture with optimal settings for GPU
        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            vk::Format::R8G8B8A8_SRGB, // Common format for RGBA images
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )?;

        // Setup image for copy operation
        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            mip_levels,
        )?;

        // Copy data from staging buffer to image
        self.transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

        // Generate mipmaps or transition to shader read
        if generate_mipmaps {
            self.transfer_service.generate_mipmaps(
                texture.image,
                vk::Format::R8G8B8A8_SRGB,
                width,
                height,
                mip_levels,
            )?;
        } else {
            self.transfer_service.transition_image_layout(
                texture.image,
                vk::Format::R8G8B8A8_SRGB,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                mip_levels,
            )?;
        }

        Ok(texture)
    }
}

// Allow cloning of the factory
impl Clone for TextureFactory {
    fn clone(&self) -> Self {
        Self {
            device: self.device.clone(),
            transfer_service: self.transfer_service.clone(),
        }
    }
}

/// Manager for handling texture resources
#[derive(Clone)]
pub struct TextureManager {
    texture_factory: TextureFactory,
    textures: HashMap<String, Weak<ManagedTexture>>,
}

impl TextureManager {
    /// Create a new texture manager
    pub fn new(device: Arc<ManagedDevice>, transfer_service: Arc<TransferService>) -> Self {
        let texture_factory = TextureFactory::new(device, transfer_service);

        Self {
            texture_factory,
            textures: HashMap::new(),
        }
    }

    /// Get or load a texture (with caching)
    pub fn get_or_load<P: AsRef<Path>>(
        &mut self,
        path: P,
    ) -> Result<Arc<ManagedTexture>, Box<dyn Error>> {
        let path_str = path.as_ref().to_string_lossy().to_string();

        // Check cache first
        if let Some(weak_texture) = self.textures.get(&path_str) {
            if let Some(strong_texture) = weak_texture.upgrade() {
                return Ok(strong_texture); // Return cached texture if still alive
            }
        }

        // Load new texture
        let texture = Arc::new(self.texture_factory.create_from_file(
            &path, None, // Use default (true) for mipmaps
        )?);

        // Store weak reference for caching
        self.textures.insert(path_str, Arc::downgrade(&texture));

        Ok(texture)
    }

    /// Load texture with explicit settings
    pub fn load_with_options<P: AsRef<Path>>(
        &mut self,
        path: P,
        generate_mipmaps: bool,
    ) -> Result<Arc<ManagedTexture>, Box<dyn Error>> {
        let texture = Arc::new(
            self.texture_factory
                .create_from_file(&path, Some(generate_mipmaps))?,
        );

        // We don't cache textures loaded with specific options

        Ok(texture)
    }

    /// Clean dead references from the cache (optional optimization)
    pub fn clean_cache(&mut self) {
        self.textures
            .retain(|_, weak_tex| weak_tex.upgrade().is_some());
    }
}

/// Component for entities with textures
#[derive(Component, Debug)]
pub struct TextureComponent {
    pub texture: Arc<ManagedTexture>,
}
