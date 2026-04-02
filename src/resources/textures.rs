// textures.rs - Enhanced texture manager with handle-based lifecycle management

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::rendering::colour::Colour;
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::texture::ManagedTexture;
use crate::utils::noise::fbm_2d_periodic;
use std::{
    collections::HashMap,
    path::Path,
    sync::{Arc, Mutex},
};

use ash::vk;

use super::transfer_service::TransferService;

/// Handle to a texture that automatically manages its lifecycle.
///
/// Uses strong references to ensure proper cleanup order - when the last handle
/// is dropped, the texture and its descriptor set are automatically cleaned up.
#[derive(Clone)]
pub struct TextureHandle {
    texture: Arc<ManagedTexture>,
    id: u64,
    manager: Arc<Mutex<TextureManagerInner>>,
}

impl TextureHandle {
    /// Get the texture ID for tracking purposes
    pub fn id(&self) -> u64 {
        self.id
    }

    /// Get access to the underlying texture
    pub fn texture(&self) -> &ManagedTexture {
        &self.texture
    }
}

impl Drop for TextureHandle {
    fn drop(&mut self) {
        // Automatically clean up when the last handle is dropped
        if let Ok(mut manager) = self.manager.lock() {
            manager.release_texture(self.id);
        }
    }
}

impl std::fmt::Debug for TextureHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TextureHandle")
            .field("id", &self.id)
            .field("texture", &self.texture)
            .finish()
    }
}

struct TextureManagerInner {
    textures: HashMap<u64, Arc<ManagedTexture>>,
    descriptor_sets: HashMap<u64, vk::DescriptorSet>, // texture_id -> descriptor_set
    descriptor_manager: Arc<DescriptorManager>,
    next_id: u64,
}

impl TextureManagerInner {
    fn release_texture(&mut self, id: u64) {
        // Clean up descriptor set immediately
        if let Some(descriptor_set) = self.descriptor_sets.remove(&id) {
            let _ = self.descriptor_manager.free_texture_set(descriptor_set);
            log::debug!("Freed descriptor set for texture {}", id);
        }

        // Remove texture - Arc will handle cleanup when ref count reaches 0
        if self.textures.remove(&id).is_some() {
            log::debug!("Released texture {}", id);
        }
    }
}

/// Factory for creating textures
pub struct TextureFactory {
    device: Arc<ManagedDevice>,
    transfer_service: Arc<TransferService>,
}

impl TextureFactory {
    pub fn new(device: Arc<ManagedDevice>, transfer_service: Arc<TransferService>) -> Self {
        Self {
            device,
            transfer_service,
        }
    }

    pub fn create_from_file<P: AsRef<Path>>(
        &self,
        file_path: P,
        generate_mipmaps: Option<bool>,
    ) -> EngineResult<ManagedTexture> {
        // ... existing implementation unchanged ...
        let generate_mipmaps = generate_mipmaps.unwrap_or(true);

        let img = image::open(file_path.as_ref())?;
        let img_rgba = img.to_rgba8();
        let width = img.width();
        let height = img.height();
        let image_data = img_rgba.into_raw();
        let image_size = image_data.len() as u64;

        let mip_levels = if generate_mipmaps {
            ((width.max(height) as f32).log2().floor() as u32) + 1
        } else {
            1
        };

        let staging_buffer = crate::rendering::frame::ManagedBuffer::new(
            self.device.clone(),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )
        .map_err(|e| EngineError::Buffer {
            operation: crate::core::error::BufferOperation::Create,
            size: image_size,
            reason: e.to_string(),
        })?;

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

        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )
        .map_err(|e| EngineError::Image {
            operation: ImageOperation::Create,
            width,
            height,
            reason: e.to_string(),
        })?;

        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            mip_levels,
        )?;

        self.transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

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

    /// Create a procedural noise texture.
    ///
    /// - `width, height`: Texture dimensions (e.g., 512x512)
    /// - `octaves`: Layers of noise (3-5 recommended)
    /// - `scale`: Noise frequency (higher = finer patterns, e.g., 16.0-24.0)
    /// - `seed`: Random seed for variation
    pub fn create_noise_texture(
        &self,
        width: u32,
        height: u32,
        octaves: u32,
        scale: f32,
        seed: u32,
    ) -> EngineResult<ManagedTexture> {
        let mut image_data = Vec::with_capacity((width * height * 4) as usize);

        // Use periodic noise for seamless tiling
        let period = scale as i32;

        for y in 0..height {
            for x in 0..width {
                let nx = x as f32 / width as f32 * scale;
                let ny = y as f32 / height as f32 * scale;

                let noise_value = fbm_2d_periodic(nx, ny, octaves, 0.5, 2.0, seed, Some(period));

                // Map noise from [0, 1] to [0.85, 1.0] for subtle, light variation
                let remapped = 0.85 + noise_value * 0.15;
                let intensity = (remapped * 255.0) as u8;

                image_data.push(intensity);
                image_data.push(intensity);
                image_data.push(intensity);
                image_data.push(255);
            }
        }

        let image_size = image_data.len() as u64;
        let mip_levels = ((width.max(height) as f32).log2().floor() as u32) + 1;

        let staging_buffer = crate::rendering::frame::ManagedBuffer::new(
            self.device.clone(),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )
        .map_err(|e| EngineError::Buffer {
            operation: crate::core::error::BufferOperation::Create,
            size: image_size,
            reason: e.to_string(),
        })?;

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

        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )
        .map_err(|e| EngineError::Image {
            operation: ImageOperation::Create,
            width,
            height,
            reason: e.to_string(),
        })?;

        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            mip_levels,
        )?;

        self.transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

        self.transfer_service.generate_mipmaps(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            width,
            height,
            mip_levels,
        )?;

        Ok(texture)
    }

    /// Create a texture from raw RGBA pixel data.
    ///
    /// - `width, height`: Texture dimensions
    /// - `rgba_data`: Raw pixel data in RGBA8 format (length must be `width * height * 4`)
    /// - `generate_mipmaps`: Whether to generate a mip chain
    pub fn create_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
    ) -> EngineResult<ManagedTexture> {
        assert_eq!(
            rgba_data.len(),
            (width * height * 4) as usize,
            "RGBA data length mismatch"
        );

        let image_size = rgba_data.len() as u64;
        let mip_levels = if generate_mipmaps {
            ((width.max(height) as f32).log2().floor() as u32) + 1
        } else {
            1
        };

        let staging_buffer = crate::rendering::frame::ManagedBuffer::new(
            self.device.clone(),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )
        .map_err(|e| EngineError::Buffer {
            operation: crate::core::error::BufferOperation::Create,
            size: image_size,
            reason: e.to_string(),
        })?;

        unsafe {
            let data_ptr = self.device.device.map_memory(
                staging_buffer.memory,
                0,
                image_size,
                vk::MemoryMapFlags::empty(),
            )? as *mut u8;

            std::ptr::copy_nonoverlapping(rgba_data.as_ptr(), data_ptr, rgba_data.len());
            self.device.device.unmap_memory(staging_buffer.memory);
        }

        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )
        .map_err(|e| EngineError::Image {
            operation: ImageOperation::Create,
            width,
            height,
            reason: e.to_string(),
        })?;

        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            mip_levels,
        )?;

        self.transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

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

    /// Create a 1x1 solid colour texture.
    pub fn create_solid_colour(&self, colour: Colour) -> EngineResult<ManagedTexture> {
        let width = 1u32;
        let height = 1u32;
        let mip_levels = 1u32;

        // Create RGBA pixel data
        let r = (colour.r.clamp(0.0, 1.0) * 255.0) as u8;
        let g = (colour.g.clamp(0.0, 1.0) * 255.0) as u8;
        let b = (colour.b.clamp(0.0, 1.0) * 255.0) as u8;
        let a = (colour.a.clamp(0.0, 1.0) * 255.0) as u8;
        let image_data: [u8; 4] = [r, g, b, a];
        let image_size = image_data.len() as u64;

        let staging_buffer = crate::rendering::frame::ManagedBuffer::new(
            self.device.clone(),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )
        .map_err(|e| EngineError::Buffer {
            operation: crate::core::error::BufferOperation::Create,
            size: image_size,
            reason: e.to_string(),
        })?;

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

        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )
        .map_err(|e| EngineError::Image {
            operation: ImageOperation::Create,
            width,
            height,
            reason: e.to_string(),
        })?;

        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            mip_levels,
        )?;

        self.transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

        self.transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
            mip_levels,
        )?;

        Ok(texture)
    }
}

impl Clone for TextureFactory {
    fn clone(&self) -> Self {
        Self {
            device: self.device.clone(),
            transfer_service: self.transfer_service.clone(),
        }
    }
}

/// Manager for handling texture resources with handle-based lifecycle.
///
/// Textures are automatically cleaned up when their last handle is dropped.
/// No manual cleanup calls required!
#[derive(Clone)]
pub struct TextureManager {
    inner: Arc<Mutex<TextureManagerInner>>,
    texture_factory: TextureFactory,
}

impl TextureManager {
    pub fn new(
        device: Arc<ManagedDevice>,
        transfer_service: Arc<TransferService>,
        descriptor_manager: Arc<DescriptorManager>,
    ) -> EngineResult<Self> {
        let texture_factory = TextureFactory::new(device.clone(), transfer_service);

        Ok(Self {
            inner: Arc::new(Mutex::new(TextureManagerInner {
                textures: HashMap::new(),
                descriptor_sets: HashMap::new(),
                descriptor_manager,
                next_id: 0,
            })),
            texture_factory,
        })
    }

    /// Get or create a descriptor set for a texture
    pub fn get_or_create_descriptor_set(
        &self,
        texture_handle: &TextureHandle,
    ) -> EngineResult<vk::DescriptorSet> {
        let mut inner = self.inner.lock().unwrap();
        let texture_id = texture_handle.id();

        // Check if we already have a descriptor set for this texture
        if let Some(&descriptor_set) = inner.descriptor_sets.get(&texture_id) {
            return Ok(descriptor_set);
        }

        // Allocate a new descriptor set from the centralized manager
        let descriptor_set = inner.descriptor_manager.allocate_texture_set()?;

        // Update the descriptor set with texture info
        let texture = texture_handle.texture();
        inner.descriptor_manager.update_texture_set(
            descriptor_set,
            texture.image_view,
            texture.sampler,
        );

        // Track the descriptor set
        inner.descriptor_sets.insert(texture_id, descriptor_set);

        log::debug!(
            "Allocated and updated descriptor set for texture {}",
            texture_id
        );

        Ok(descriptor_set)
    }

    /// Load a texture and return a handle to it.
    ///
    /// The texture and its descriptor set will be automatically cleaned up
    /// when the last handle is dropped - no manual cleanup required!
    pub fn load_texture<P: AsRef<Path>>(&self, path: P) -> EngineResult<TextureHandle> {
        let texture = Arc::new(self.texture_factory.create_from_file(&path, None)?);

        let mut inner = self.inner.lock().unwrap();
        let id = inner.next_id;
        inner.next_id += 1;

        inner.textures.insert(id, texture.clone());

        Ok(TextureHandle {
            texture,
            id,
            manager: Arc::clone(&self.inner),
        })
    }

    /// Create a texture from raw RGBA pixel data.
    pub fn create_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
    ) -> EngineResult<TextureHandle> {
        let texture = Arc::new(self.texture_factory.create_from_rgba(
            width,
            height,
            rgba_data,
            generate_mipmaps,
        )?);

        let mut inner = self.inner.lock().unwrap();
        let id = inner.next_id;
        inner.next_id += 1;

        inner.textures.insert(id, texture.clone());

        log::debug!("Created RGBA texture {}x{} (id={})", width, height, id);

        Ok(TextureHandle {
            texture,
            id,
            manager: Arc::clone(&self.inner),
        })
    }

    /// Create a procedural noise texture.
    ///
    /// Generates a tileable noise pattern useful for terrain variation.
    pub fn create_noise_texture(
        &self,
        width: u32,
        height: u32,
        octaves: u32,
        scale: f32,
        seed: u32,
    ) -> EngineResult<TextureHandle> {
        let texture = Arc::new(
            self.texture_factory
                .create_noise_texture(width, height, octaves, scale, seed)?,
        );

        let mut inner = self.inner.lock().unwrap();
        let id = inner.next_id;
        inner.next_id += 1;

        inner.textures.insert(id, texture.clone());

        log::debug!("Created noise texture {}x{} (id={})", width, height, id);

        Ok(TextureHandle {
            texture,
            id,
            manager: Arc::clone(&self.inner),
        })
    }

    /// Create a 1x1 solid colour texture.
    ///
    /// Useful for fallback textures or when materials don't need textures.
    pub fn create_solid_colour(&self, colour: Colour) -> EngineResult<TextureHandle> {
        let texture = Arc::new(self.texture_factory.create_solid_colour(colour)?);

        let mut inner = self.inner.lock().unwrap();
        let id = inner.next_id;
        inner.next_id += 1;

        inner.textures.insert(id, texture.clone());

        log::debug!("Created solid colour texture (id={})", id);

        Ok(TextureHandle {
            texture,
            id,
            manager: Arc::clone(&self.inner),
        })
    }
}

impl Drop for TextureManager {
    fn drop(&mut self) {
        // Descriptor pools are now managed by DescriptorManager, which will clean them up
        // We only need to ensure textures are properly cleaned up (already handled by ManagedTexture Drop)
    }
}
