// textures.rs - Enhanced texture manager with handle-based lifecycle management

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::rendering::colour::Colour;
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::texture::ManagedTexture;
use crate::resources::texture_encoding::TextureEncoding;
use crate::resources::texture_registry::{ReleaseOutcome, TextureRegistry};
use crate::utils::noise::fbm_2d_periodic;
use std::{
    path::Path,
    sync::{Arc, Mutex},
};

use ash::vk;

use super::transfer_service::TransferService;

/// Handle to a texture that automatically manages its lifecycle.
///
/// Cloning a handle takes another reference to the same texture. When the last
/// handle is dropped, the texture and its descriptor set are cleaned up.
pub struct TextureHandle {
    /// The texture, kept alive for as long as this handle exists.
    texture: Arc<ManagedTexture>,
    /// Registry id of the texture, used for retain and release bookkeeping.
    id: u64,
    /// Shared manager state that counts the live handles to this texture.
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

impl Clone for TextureHandle {
    fn clone(&self) -> Self {
        // The retain must land before the new handle exists. A count that lags
        // behind the live handles lets a later release free a texture another
        // handle still uses, so a poisoned lock panics here rather than hand
        // back an untracked handle.
        self.manager
            .lock()
            .expect("texture manager mutex poisoned")
            .retain_texture(self.id);

        Self {
            texture: Arc::clone(&self.texture),
            id: self.id,
            manager: Arc::clone(&self.manager),
        }
    }
}

impl Drop for TextureHandle {
    fn drop(&mut self) {
        // A skipped release only leaks a texture, so this stays silent where
        // the retain in `clone` panics. Unwinding out of a drop would turn a
        // poisoned lock into an abort.
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
    /// Live textures and their handle counts.
    registry: TextureRegistry,
    /// Source of the descriptor sets bound to those textures.
    descriptor_manager: Arc<DescriptorManager>,
}

impl TextureManagerInner {
    fn retain_texture(&mut self, id: u64) {
        self.registry.retain(id);
    }

    /// Drop one handle to a texture, freeing its GPU resources if it was the last.
    fn release_texture(&mut self, id: u64) {
        let ReleaseOutcome::Released { descriptor_set } = self.registry.release(id) else {
            return;
        };

        if let Some(descriptor_set) = descriptor_set {
            let _ = self.descriptor_manager.free_texture_set(descriptor_set);
            log::debug!("Freed descriptor set for texture {}", id);
        }

        log::debug!("Released texture {}", id);
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
    /// - `encoding`: Whether the bytes are colour or measurements. Getting this
    ///   wrong is silent; see [`TextureEncoding`].
    pub fn create_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
        encoding: TextureEncoding,
    ) -> EngineResult<ManagedTexture> {
        let format = encoding.rgba8_format();

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
            format,
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
            format,
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
                format,
                width,
                height,
                mip_levels,
            )?;
        } else {
            self.transfer_service.transition_image_layout(
                texture.image,
                format,
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
                registry: TextureRegistry::new(),
                descriptor_manager,
            })),
            texture_factory,
        })
    }

    /// Number of textures with at least one live handle.
    pub fn live_texture_count(&self) -> usize {
        self.inner.lock().unwrap().registry.len()
    }

    /// Take ownership of a freshly created texture and return the first handle to it.
    fn register(&self, texture: ManagedTexture) -> TextureHandle {
        let texture = Arc::new(texture);

        let mut inner = self.inner.lock().unwrap();
        let id = inner.registry.insert(Arc::clone(&texture));

        TextureHandle {
            texture,
            id,
            manager: Arc::clone(&self.inner),
        }
    }

    /// Get or create a descriptor set for a texture
    pub fn get_or_create_descriptor_set(
        &self,
        texture_handle: &TextureHandle,
    ) -> EngineResult<vk::DescriptorSet> {
        let mut inner = self.inner.lock().unwrap();
        let texture_id = texture_handle.id();

        // Check if we already have a descriptor set for this texture
        if let Some(descriptor_set) = inner.registry.descriptor_set(texture_id) {
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
        inner
            .registry
            .set_descriptor_set(texture_id, descriptor_set);

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
        let texture = self.texture_factory.create_from_file(&path, None)?;

        Ok(self.register(texture))
    }

    /// Create a colour texture from raw sRGB-encoded RGBA pixel data.
    ///
    /// The overwhelmingly common case, and the one every hand-authored albedo
    /// wants. A texture whose channels carry measurements rather than light —
    /// normals, masks, packed fields — must use
    /// [`create_data_from_rgba`](Self::create_data_from_rgba) instead.
    pub fn create_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
    ) -> EngineResult<TextureHandle> {
        self.create_encoded_from_rgba(
            width,
            height,
            rgba_data,
            generate_mipmaps,
            TextureEncoding::Srgb,
        )
    }

    /// Create a data texture from raw RGBA pixel data, sampled without any
    /// transfer function.
    ///
    /// For tangent-space normals, masks, and packed fields. What the CPU writes
    /// is what the shader reads.
    pub fn create_data_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
    ) -> EngineResult<TextureHandle> {
        self.create_encoded_from_rgba(
            width,
            height,
            rgba_data,
            generate_mipmaps,
            TextureEncoding::Linear,
        )
    }

    /// Create a texture from raw RGBA pixel data under an explicit encoding.
    pub fn create_encoded_from_rgba(
        &self,
        width: u32,
        height: u32,
        rgba_data: &[u8],
        generate_mipmaps: bool,
        encoding: TextureEncoding,
    ) -> EngineResult<TextureHandle> {
        let texture = self.texture_factory.create_from_rgba(
            width,
            height,
            rgba_data,
            generate_mipmaps,
            encoding,
        )?;
        let handle = self.register(texture);

        log::debug!(
            "Created {:?} RGBA texture {}x{} (id={})",
            encoding,
            width,
            height,
            handle.id()
        );

        Ok(handle)
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
        let texture = self
            .texture_factory
            .create_noise_texture(width, height, octaves, scale, seed)?;
        let handle = self.register(texture);

        log::debug!(
            "Created noise texture {}x{} (id={})",
            width,
            height,
            handle.id()
        );

        Ok(handle)
    }

    /// Create a 1x1 solid colour texture.
    ///
    /// Useful for fallback textures or when materials don't need textures.
    pub fn create_solid_colour(&self, colour: Colour) -> EngineResult<TextureHandle> {
        let texture = self.texture_factory.create_solid_colour(colour)?;
        let handle = self.register(texture);

        log::debug!("Created solid colour texture (id={})", handle.id());

        Ok(handle)
    }
}

impl Drop for TextureManager {
    fn drop(&mut self) {
        // Descriptor pools are now managed by DescriptorManager, which will clean them up
        // We only need to ensure textures are properly cleaned up (already handled by ManagedTexture Drop)
    }
}
