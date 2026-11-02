//! Sampled textures: a device image, a view of all of it and a sampler.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation};
use crate::rendering::device_image::{DeviceImage, ImageSpec, ImageView};

pub struct ManagedTexture {
    pub sampler: vk::Sampler,
    /// Every mip level of the image; declared before it, so dropped first.
    pub image_view: ImageView,
    pub image: DeviceImage,
    pub width: u32,
    pub height: u32,
    /// Depth extent. 1 for 2D textures, >1 for 3D volumes.
    pub depth: u32,
    pub mip_levels: u32,
    pub format: vk::Format,
    device: Arc<ManagedDevice>,
}

impl std::fmt::Debug for ManagedTexture {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManagedTexture")
            .field("image", &self.image.handle())
            .field("image_view", &self.image_view.handle())
            .field("sampler", &self.sampler)
            .field("width", &self.width)
            .field("height", &self.height)
            .field("depth", &self.depth)
            .field("mip_levels", &self.mip_levels)
            .field("format", &self.format)
            .finish()
    }
}

impl ManagedTexture {
    /// A 2D colour texture of `mip_levels` levels, sampled trilinearly with
    /// repeat addressing.
    pub fn new(
        device: Arc<ManagedDevice>,
        width: u32,
        height: u32,
        mip_levels: u32,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
    ) -> EngineResult<Self> {
        let extent = vk::Extent2D { width, height };
        let image = DeviceImage::new(
            &device,
            ImageSpec::new_2d(format, extent, usage).mip_levels(mip_levels),
        )?;
        let image_view =
            image.full_view(vk::ImageViewType::TYPE_2D, vk::ImageAspectFlags::COLOR)?;
        let sampler_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::LINEAR)
            .min_filter(vk::Filter::LINEAR)
            .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::REPEAT)
            .address_mode_v(vk::SamplerAddressMode::REPEAT)
            .address_mode_w(vk::SamplerAddressMode::REPEAT)
            .mip_lod_bias(0.0)
            .anisotropy_enable(false)
            .max_anisotropy(1.0)
            .compare_enable(false)
            .compare_op(vk::CompareOp::ALWAYS)
            .min_lod(0.0)
            .max_lod(mip_levels as f32)
            .border_color(vk::BorderColor::INT_OPAQUE_BLACK)
            .unnormalized_coordinates(false);
        let sampler = create_sampler(&device, &sampler_info, width, height)?;
        Ok(Self {
            sampler,
            image_view,
            image,
            width,
            height,
            depth: 1,
            mip_levels,
            format,
            device,
        })
    }

    /// Create a 3D volume texture.
    ///
    /// Used for volumetric data such as fire simulation grids, with a
    /// trilinear sampler and clamp-to-border addressing.
    pub fn new_3d(
        device: Arc<ManagedDevice>,
        width: u32,
        height: u32,
        depth: u32,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
    ) -> EngineResult<Self> {
        let extent = vk::Extent3D {
            width,
            height,
            depth,
        };
        let image = DeviceImage::new(&device, ImageSpec::new_3d(format, extent, usage))?;
        let image_view =
            image.full_view(vk::ImageViewType::TYPE_3D, vk::ImageAspectFlags::COLOR)?;
        let sampler_info = vk::SamplerCreateInfo::default()
            .mag_filter(vk::Filter::LINEAR)
            .min_filter(vk::Filter::LINEAR)
            .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_BORDER)
            .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_BORDER)
            .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_BORDER)
            .border_color(vk::BorderColor::FLOAT_TRANSPARENT_BLACK)
            .unnormalized_coordinates(false);
        let sampler = create_sampler(&device, &sampler_info, width, height)?;
        Ok(Self {
            sampler,
            image_view,
            image,
            width,
            height,
            depth,
            mip_levels: 1,
            format,
            device,
        })
    }
}

fn create_sampler(
    device: &ManagedDevice,
    info: &vk::SamplerCreateInfo,
    width: u32,
    height: u32,
) -> EngineResult<vk::Sampler> {
    unsafe { device.device.create_sampler(info, None) }.map_err(|e| EngineError::Image {
        operation: ImageOperation::Create,
        width,
        height,
        reason: format!("Failed to create sampler: {:?}", e),
    })
}

impl Drop for ManagedTexture {
    fn drop(&mut self) {
        unsafe { self.device.device.destroy_sampler(self.sampler, None) };
    }
}
