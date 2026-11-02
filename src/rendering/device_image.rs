//! GPU images in device-local memory, and views of them, each released
//! when dropped — including when a later step of making it fails.
//!
//! ```text
//!   ImageSpec ──▶ DeviceImage (image + bound memory) ──▶ ImageView ...
//! ```
//!
//! An owner holds its views before the image they look into, so that field
//! drop order destroys the views first.

use std::ops::Range;
use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ImageOperation, VkResultExt};
use crate::core::vulkan_context::find_memorytype_index;

/// What image to make: optimal tiling, one sample, exclusive to one queue.
#[derive(Debug, Clone, Copy)]
pub struct ImageSpec {
    pub image_type: vk::ImageType,
    pub format: vk::Format,
    pub extent: vk::Extent3D,
    pub mip_levels: u32,
    pub array_layers: u32,
    pub usage: vk::ImageUsageFlags,
    pub flags: vk::ImageCreateFlags,
}

impl ImageSpec {
    /// A 2D image with one mip level and one layer.
    pub fn new_2d(format: vk::Format, extent: vk::Extent2D, usage: vk::ImageUsageFlags) -> Self {
        Self {
            image_type: vk::ImageType::TYPE_2D,
            format,
            extent: extent.into(),
            mip_levels: 1,
            array_layers: 1,
            usage,
            flags: vk::ImageCreateFlags::empty(),
        }
    }

    /// A 3D volume with one mip level.
    pub fn new_3d(format: vk::Format, extent: vk::Extent3D, usage: vk::ImageUsageFlags) -> Self {
        Self {
            image_type: vk::ImageType::TYPE_3D,
            extent,
            ..Self::new_2d(format, vk::Extent2D::default(), usage)
        }
    }

    pub fn mip_levels(self, mip_levels: u32) -> Self {
        Self { mip_levels, ..self }
    }

    /// `layers` array layers, made with `flags` (e.g. `CUBE_COMPATIBLE`).
    pub fn array_layers(self, layers: u32, flags: vk::ImageCreateFlags) -> Self {
        Self {
            array_layers: layers,
            flags,
            ..self
        }
    }
}

/// An image and the device-local memory bound to it.
pub struct DeviceImage {
    image: vk::Image,
    memory: vk::DeviceMemory,
    spec: ImageSpec,
    device: Arc<ManagedDevice>,
}

impl DeviceImage {
    /// Make the image, allocate device-local memory for it and bind the two.
    /// Whatever was made before a failing step is released.
    pub fn new(device: &Arc<ManagedDevice>, spec: ImageSpec) -> EngineResult<Self> {
        let (width, height) = (spec.extent.width, spec.extent.height);
        let info = vk::ImageCreateInfo::default()
            .flags(spec.flags)
            .image_type(spec.image_type)
            .format(spec.format)
            .extent(spec.extent)
            .mip_levels(spec.mip_levels)
            .array_layers(spec.array_layers)
            .samples(vk::SampleCountFlags::TYPE_1)
            .tiling(vk::ImageTiling::OPTIMAL)
            .usage(spec.usage)
            .sharing_mode(vk::SharingMode::EXCLUSIVE)
            .initial_layout(vk::ImageLayout::UNDEFINED);
        let image = unsafe { device.device.create_image(&info, None) }.image_context(
            ImageOperation::Create,
            width,
            height,
        )?;
        let mut this = Self {
            image,
            memory: vk::DeviceMemory::null(),
            spec,
            device: Arc::clone(device),
        };

        let requirements = unsafe { device.device.get_image_memory_requirements(image) };
        let memory_type_index = find_memorytype_index(
            &requirements,
            &device.device_memory_properties,
            vk::MemoryPropertyFlags::DEVICE_LOCAL,
        )
        .ok_or_else(|| EngineError::Image {
            operation: ImageOperation::AllocateMemory,
            width,
            height,
            reason: "no device-local memory type".to_string(),
        })?;
        let alloc_info = vk::MemoryAllocateInfo::default()
            .allocation_size(requirements.size)
            .memory_type_index(memory_type_index);
        this.memory = unsafe { device.device.allocate_memory(&alloc_info, None) }.image_context(
            ImageOperation::AllocateMemory,
            width,
            height,
        )?;
        unsafe { device.device.bind_image_memory(image, this.memory, 0) }.image_context(
            ImageOperation::Bind,
            width,
            height,
        )?;
        Ok(this)
    }

    pub fn handle(&self) -> vk::Image {
        self.image
    }

    pub fn spec(&self) -> &ImageSpec {
        &self.spec
    }

    /// A view of `levels` mip levels and `layers` array layers of the image,
    /// in its own format.
    pub fn view(
        &self,
        view_type: vk::ImageViewType,
        aspect_mask: vk::ImageAspectFlags,
        levels: Range<u32>,
        layers: Range<u32>,
    ) -> EngineResult<ImageView> {
        let info = vk::ImageViewCreateInfo::default()
            .image(self.image)
            .view_type(view_type)
            .format(self.spec.format)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask,
                base_mip_level: levels.start,
                level_count: levels.len() as u32,
                base_array_layer: layers.start,
                layer_count: layers.len() as u32,
            });
        let view = unsafe { self.device.device.create_image_view(&info, None) }.image_context(
            ImageOperation::CreateView,
            self.spec.extent.width,
            self.spec.extent.height,
        )?;
        Ok(ImageView {
            view,
            device: Arc::clone(&self.device),
        })
    }

    /// A view of the whole image: every level and layer.
    pub fn full_view(
        &self,
        view_type: vk::ImageViewType,
        aspect_mask: vk::ImageAspectFlags,
    ) -> EngineResult<ImageView> {
        self.view(
            view_type,
            aspect_mask,
            0..self.spec.mip_levels,
            0..self.spec.array_layers,
        )
    }
}

impl Drop for DeviceImage {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}

/// A view into a `DeviceImage`. It must be dropped before the image.
pub struct ImageView {
    view: vk::ImageView,
    device: Arc<ManagedDevice>,
}

impl ImageView {
    pub fn handle(&self) -> vk::ImageView {
        self.view
    }
}

impl Drop for ImageView {
    fn drop(&mut self) {
        unsafe { self.device.device.destroy_image_view(self.view, None) };
    }
}
