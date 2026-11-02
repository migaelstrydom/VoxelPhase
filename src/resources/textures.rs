//! Textures behind reference-counted handles: created from pixels, released
//! when the last handle drops.

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::rendering::colour::Colour;
use crate::rendering::descriptors::DescriptorManager;
use crate::rendering::texture::ManagedTexture;
use crate::resources::texture_encoding::TextureEncoding;
use crate::resources::texture_registry::{ReleaseOutcome, TextureRegistry};
use std::sync::{Arc, Mutex};

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
            manager.release_texture(self.id, &self.texture);
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

    /// Drop one handle to a texture. If it was the last, the texture and its
    /// descriptor set are retired: frames still in flight may be drawing with
    /// them, so they are freed once those frames finish, not here.
    fn release_texture(&mut self, id: u64, texture: &Arc<ManagedTexture>) {
        let ReleaseOutcome::Released { descriptor_set } = self.registry.release(id) else {
            return;
        };

        self.descriptor_manager
            .retire_texture(descriptor_set, Arc::clone(texture));
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

        let mip_levels = if generate_mipmaps {
            ((width.max(height) as f32).log2().floor() as u32) + 1
        } else {
            1
        };
        let texture = ManagedTexture::new(
            self.device.clone(),
            width,
            height,
            mip_levels,
            format,
            vk::ImageUsageFlags::TRANSFER_DST
                | vk::ImageUsageFlags::TRANSFER_SRC
                | vk::ImageUsageFlags::SAMPLED,
        )?;
        self.transfer_service.upload_texture(&texture, rgba_data)?;
        Ok(texture)
    }

    /// Create a 1x1 solid colour texture.
    pub fn create_solid_colour(&self, colour: Colour) -> EngineResult<ManagedTexture> {
        let byte = |channel: f32| (channel.clamp(0.0, 1.0) * 255.0) as u8;
        let pixel = [
            byte(colour.r),
            byte(colour.g),
            byte(colour.b),
            byte(colour.a),
        ];
        self.create_from_rgba(1, 1, &pixel, false, TextureEncoding::Srgb)
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
    /// Textures the engine binds to a global descriptor slot and must keep
    /// alive for as long as it can draw.
    ///
    /// A `TextureHandle` frees its descriptor set when the last one drops, so a
    /// texture bound into the scene descriptor set and then dropped leaves that
    /// binding pointing at freed resources. Holding the handle here is what
    /// says "this one outlives every material".
    ///
    /// Shared across clones of the manager, so a resident texture lives as long
    /// as any clone can still draw with it.
    resident: Arc<Mutex<Vec<TextureHandle>>>,
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
            resident: Arc::new(Mutex::new(Vec::new())),
        })
    }

    /// Keep a texture alive for the lifetime of this manager.
    ///
    /// For textures bound to a global descriptor slot rather than owned by a
    /// material — see [`Self::resident`].
    pub fn keep_resident(&self, texture: TextureHandle) {
        self.resident.lock().unwrap().push(texture);
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
            texture.image_view.handle(),
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
