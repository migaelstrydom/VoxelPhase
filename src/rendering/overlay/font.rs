//! Font atlas management for text rendering.
//!
//! Provides glyph rasterization and texture atlas generation using fontdue.

use std::collections::HashMap;
use std::sync::Arc;

use ash::vk;
use fontdue::{Font, FontSettings};
use nalgebra::{Vector2, Vector4};

use crate::core::error::EngineResult;
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::frame::ManagedBuffer;
use crate::rendering::texture::ManagedTexture;
use crate::resources::transfer_service::TransferService;

use super::vertex::OverlayVertex;

/// Font data embedded at compile time
const FONT_DATA: &[u8] = include_bytes!("../../../data/JetBrainsMono-Regular.ttf");

/// Glyph metrics for text layout
#[derive(Copy, Clone, Debug)]
pub struct GlyphMetrics {
    pub uv_min: Vector2<f32>,
    pub uv_max: Vector2<f32>,
    pub size: Vector2<f32>,    // Width and height in pixels
    pub bearing: Vector2<f32>, // Offset from baseline
    pub advance: f32,          // Horizontal advance to next glyph
}

/// Font atlas containing pre-rasterized glyphs.
///
/// Rasterizes ASCII characters 32-126 at 24px and packs them into a texture.
pub struct FontAtlas {
    _texture: Arc<ManagedTexture>,
    descriptor_set: vk::DescriptorSet,
    descriptor_set_layout: vk::DescriptorSetLayout,
    descriptor_pool: vk::DescriptorPool,
    glyphs: HashMap<char, GlyphMetrics>,
    line_height: f32,
    device: Arc<crate::core::device::ManagedDevice>,
}

impl FontAtlas {
    /// Create a new font atlas.
    ///
    /// Loads JetBrains Mono, rasterizes glyphs, and creates GPU texture.
    pub fn new(vulkan_context: Arc<VulkanContext>) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);
        const FONT_SIZE: f32 = 24.0;
        const ATLAS_WIDTH: usize = 512;
        const ATLAS_HEIGHT: usize = 512;
        const PADDING: usize = 2;

        // Load font with fontdue
        let font = Font::from_bytes(FONT_DATA, FontSettings::default()).map_err(|e| {
            crate::core::error::EngineError::InvalidState(format!("Failed to load font: {}", e))
        })?;

        // Get line metrics for proper vertical positioning
        let line_metrics = font.horizontal_line_metrics(FONT_SIZE).ok_or_else(|| {
            crate::core::error::EngineError::InvalidState(
                "Font has no horizontal line metrics".to_string(),
            )
        })?;
        let ascent = line_metrics.ascent;
        let line_height = line_metrics.new_line_size;

        // Create atlas image buffer (R8 grayscale)
        let mut atlas_data = vec![0u8; ATLAS_WIDTH * ATLAS_HEIGHT];
        let mut glyphs = HashMap::new();

        // Pack glyphs into atlas
        let mut atlas_x = PADDING;
        let mut atlas_y = PADDING;
        let mut row_height = 0;

        for c in 32u8..=126 {
            let ch = c as char;
            let (metrics, bitmap) = font.rasterize(ch, FONT_SIZE);

            // Check if we need to move to next row
            if atlas_x + metrics.width + PADDING > ATLAS_WIDTH {
                atlas_x = PADDING;
                atlas_y += row_height + PADDING;
                row_height = 0;
            }

            // Check if we have vertical space
            if atlas_y + metrics.height > ATLAS_HEIGHT {
                return Err(crate::core::error::EngineError::InvalidState(
                    "Font atlas too small".to_string(),
                ));
            }

            // Copy bitmap to atlas
            for by in 0..metrics.height {
                for bx in 0..metrics.width {
                    let dst_x = atlas_x + bx;
                    let dst_y = atlas_y + by;
                    let dst_idx = dst_y * ATLAS_WIDTH + dst_x;
                    let src_idx = by * metrics.width + bx;
                    atlas_data[dst_idx] = bitmap[src_idx];
                }
            }

            // Calculate UV coordinates
            let uv_min = Vector2::new(
                atlas_x as f32 / ATLAS_WIDTH as f32,
                atlas_y as f32 / ATLAS_HEIGHT as f32,
            );
            let uv_max = Vector2::new(
                (atlas_x + metrics.width) as f32 / ATLAS_WIDTH as f32,
                (atlas_y + metrics.height) as f32 / ATLAS_HEIGHT as f32,
            );

            // Calculate bearing for proper baseline alignment.
            // In screen coords (y down), bearing.y is the offset from line-top to glyph-top.
            // Formula: ascent - ymin - height positions glyph correctly relative to baseline.
            let bearing_x = metrics.xmin as f32;
            let bearing_y = ascent - metrics.ymin as f32 - metrics.height as f32;

            glyphs.insert(
                ch,
                GlyphMetrics {
                    uv_min,
                    uv_max,
                    size: Vector2::new(metrics.width as f32, metrics.height as f32),
                    bearing: Vector2::new(bearing_x, bearing_y),
                    advance: metrics.advance_width,
                },
            );

            // Update position
            atlas_x += metrics.width + PADDING;
            row_height = row_height.max(metrics.height);
        }

        // Create texture from atlas data
        let texture = Self::create_atlas_texture(
            Arc::clone(&vulkan_context),
            &atlas_data,
            ATLAS_WIDTH as u32,
            ATLAS_HEIGHT as u32,
        )?;

        // Create descriptor set layout
        let sampler_binding = vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::FRAGMENT);

        let layout_create_info = vk::DescriptorSetLayoutCreateInfo::default()
            .bindings(std::slice::from_ref(&sampler_binding));

        let descriptor_set_layout = unsafe {
            device
                .device
                .create_descriptor_set_layout(&layout_create_info, None)
        }?;

        // Create descriptor pool
        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
            descriptor_count: 1,
        };

        let pool_create_info = vk::DescriptorPoolCreateInfo::default()
            .max_sets(1)
            .pool_sizes(std::slice::from_ref(&pool_size));

        let descriptor_pool = unsafe {
            device
                .device
                .create_descriptor_pool(&pool_create_info, None)
        }?;

        // Allocate descriptor set
        let alloc_info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(descriptor_pool)
            .set_layouts(std::slice::from_ref(&descriptor_set_layout));

        let descriptor_sets = unsafe { device.device.allocate_descriptor_sets(&alloc_info) }?;
        let descriptor_set = descriptor_sets[0];

        // Update descriptor set with texture
        let image_info = vk::DescriptorImageInfo {
            sampler: texture.sampler,
            image_view: texture.image_view,
            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        };

        let write_descriptor = vk::WriteDescriptorSet::default()
            .dst_set(descriptor_set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .image_info(std::slice::from_ref(&image_info));

        unsafe {
            device
                .device
                .update_descriptor_sets(std::slice::from_ref(&write_descriptor), &[]);
        }

        Ok(Self {
            _texture: texture,
            descriptor_set,
            descriptor_set_layout,
            descriptor_pool,
            glyphs,
            line_height,
            device,
        })
    }

    /// Create GPU texture from atlas data.
    fn create_atlas_texture(
        vulkan_context: Arc<VulkanContext>,
        data: &[u8],
        width: u32,
        height: u32,
    ) -> EngineResult<Arc<ManagedTexture>> {
        let transfer_service = TransferService::new(Arc::clone(&vulkan_context))?;
        let image_size = data.len() as u64;

        // Create staging buffer
        let staging_buffer = ManagedBuffer::new(
            Arc::clone(&vulkan_context.device),
            image_size,
            vk::BufferUsageFlags::TRANSFER_SRC,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;

        // Upload data to staging buffer
        unsafe {
            let ptr = staging_buffer.map_memory(0, vk::MemoryMapFlags::empty())?;
            std::ptr::copy_nonoverlapping(data.as_ptr(), ptr as *mut u8, data.len());
            staging_buffer.unmap_memory();
        }

        // Create texture (R8 format for grayscale)
        let texture = ManagedTexture::new(
            Arc::clone(&vulkan_context.device),
            width,
            height,
            1, // No mipmaps
            vk::Format::R8_UNORM,
            vk::ImageTiling::OPTIMAL,
            vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
            vk::ImageAspectFlags::COLOR,
        )?;

        // Transition layout and copy from staging
        transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8_UNORM,
            vk::ImageLayout::UNDEFINED,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            1,
        )?;

        transfer_service.copy_buffer_to_image(
            staging_buffer.buffer,
            texture.image,
            width,
            height,
        )?;

        transfer_service.transition_image_layout(
            texture.image,
            vk::Format::R8_UNORM,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
            1,
        )?;

        Ok(Arc::new(texture))
    }

    pub fn descriptor_set(&self) -> vk::DescriptorSet {
        self.descriptor_set
    }

    pub fn descriptor_set_layout(&self) -> vk::DescriptorSetLayout {
        self.descriptor_set_layout
    }

    pub fn get_glyph(&self, c: char) -> Option<&GlyphMetrics> {
        self.glyphs.get(&c)
    }

    pub fn line_height(&self) -> f32 {
        self.line_height
    }
}

impl Drop for FontAtlas {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_descriptor_pool(self.descriptor_pool, None);
            self.device
                .device
                .destroy_descriptor_set_layout(self.descriptor_set_layout, None);
        }
    }
}

/// Text layout utility for generating quad vertices from strings.
pub struct TextLayout;

impl TextLayout {
    /// Layout text into quads for rendering.
    ///
    /// Returns (vertices, indices) for indexed drawing.
    pub fn layout_text(
        atlas: &FontAtlas,
        text: &str,
        x: f32,
        y: f32,
        color: Vector4<f32>,
    ) -> (Vec<OverlayVertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut cursor_x = x;

        for ch in text.chars() {
            if let Some(glyph) = atlas.get_glyph(ch) {
                // Calculate quad position
                let x0 = cursor_x + glyph.bearing.x;
                let y0 = y + glyph.bearing.y;
                let x1 = x0 + glyph.size.x;
                let y1 = y0 + glyph.size.y;

                let base_idx = vertices.len() as u32;

                // Create quad vertices (2 triangles = 6 vertices)
                vertices.push(OverlayVertex {
                    pos: Vector2::new(x0, y0),
                    uv: glyph.uv_min,
                    color,
                });
                vertices.push(OverlayVertex {
                    pos: Vector2::new(x1, y0),
                    uv: Vector2::new(glyph.uv_max.x, glyph.uv_min.y),
                    color,
                });
                vertices.push(OverlayVertex {
                    pos: Vector2::new(x1, y1),
                    uv: glyph.uv_max,
                    color,
                });
                vertices.push(OverlayVertex {
                    pos: Vector2::new(x0, y1),
                    uv: Vector2::new(glyph.uv_min.x, glyph.uv_max.y),
                    color,
                });

                // Create indices for 2 triangles
                indices.extend_from_slice(&[
                    base_idx,
                    base_idx + 1,
                    base_idx + 2,
                    base_idx,
                    base_idx + 2,
                    base_idx + 3,
                ]);

                cursor_x += glyph.advance;
            } else {
                // Fallback for missing glyphs: advance by half line height
                cursor_x += atlas.line_height() * 0.5;
            }
        }

        (vertices, indices)
    }
}
