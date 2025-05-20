use std::{error::Error, sync::Arc};

use ash::vk;

use crate::core::vulkan_context::{find_memorytype_index, ManagedDevice};

pub struct ManagedTexture {
    pub image: vk::Image,
    pub image_view: vk::ImageView,
    pub memory: vk::DeviceMemory,
    pub sampler: vk::Sampler,
    pub width: u32,
    pub height: u32,
    pub mip_levels: u32,
    pub format: vk::Format,
    device: Arc<ManagedDevice>,
}

impl std::fmt::Debug for ManagedTexture {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManagedTexture")
            .field("image", &self.image)
            .field("image_view", &self.image_view)
            .field("memory", &self.memory)
            .field("sampler", &self.sampler)
            .field("width", &self.width)
            .field("height", &self.height)
            .field("mip_levels", &self.mip_levels)
            .field("format", &self.format)
            // Skipping self.device as ash::Device is not Debug
            .finish()
    }
}

impl ManagedTexture {
    pub fn new(
        managed_device: Arc<ManagedDevice>,
        width: u32,
        height: u32,
        mip_levels: u32,
        format: vk::Format,
        tiling: vk::ImageTiling,
        usage: vk::ImageUsageFlags,
        memory_properties: vk::MemoryPropertyFlags,
        aspect_mask: vk::ImageAspectFlags, // e.g., vk::ImageAspectFlags::COLOR for color textures
    ) -> Result<Self, Box<dyn Error>> {
        unsafe {
            // 1. Create vk::Image
            let image_create_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(format)
                .extent(vk::Extent3D {
                    width,
                    height,
                    depth: 1,
                })
                .mip_levels(mip_levels)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(tiling)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE)
                .initial_layout(vk::ImageLayout::UNDEFINED); // Image layout will need to be transitioned

            let image = managed_device
                .device
                .create_image(&image_create_info, None)?;

            // 2. Allocate vk::DeviceMemory for the image
            let memory_req = managed_device.device.get_image_memory_requirements(image);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &managed_device.device_memory_properties,
                memory_properties,
            )
            .ok_or("Unable to find suitable memory type for image.")?;

            let allocate_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);
            let memory = managed_device
                .device
                .allocate_memory(&allocate_info, None)?;

            // 3. Bind image memory
            managed_device.device.bind_image_memory(image, memory, 0)?;

            // 4. Create vk::ImageView
            let image_view_create_info = vk::ImageViewCreateInfo::default()
                .image(image)
                .view_type(vk::ImageViewType::TYPE_2D)
                .format(format)
                .components(vk::ComponentMapping {
                    r: vk::ComponentSwizzle::IDENTITY,
                    g: vk::ComponentSwizzle::IDENTITY,
                    b: vk::ComponentSwizzle::IDENTITY,
                    a: vk::ComponentSwizzle::IDENTITY,
                })
                .subresource_range(vk::ImageSubresourceRange {
                    aspect_mask,
                    base_mip_level: 0,
                    level_count: mip_levels,
                    base_array_layer: 0,
                    layer_count: 1,
                });
            let image_view = managed_device
                .device
                .create_image_view(&image_view_create_info, None)?;

            // 5. Create vk::Sampler (with default parameters for now)
            let sampler_create_info = vk::SamplerCreateInfo::default()
                .mag_filter(vk::Filter::LINEAR)
                .min_filter(vk::Filter::LINEAR)
                .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
                .address_mode_u(vk::SamplerAddressMode::REPEAT)
                .address_mode_v(vk::SamplerAddressMode::REPEAT)
                .address_mode_w(vk::SamplerAddressMode::REPEAT)
                .mip_lod_bias(0.0)
                .anisotropy_enable(false) // Can be enabled if supported and desired
                .max_anisotropy(1.0) //
                .compare_enable(false)
                .compare_op(vk::CompareOp::ALWAYS)
                .min_lod(0.0)
                .max_lod(mip_levels as f32)
                .border_color(vk::BorderColor::INT_OPAQUE_BLACK)
                .unnormalized_coordinates(false);

            let sampler = managed_device
                .device
                .create_sampler(&sampler_create_info, None)?;

            Ok(Self {
                image,
                image_view,
                memory,
                sampler,
                width,
                height,
                mip_levels,
                format,
                device: managed_device,
            })
        }
    }
}

impl Drop for ManagedTexture {
    fn drop(&mut self) {
        unsafe {
            // Order of destruction: sampler, image view, image, then memory
            if self.sampler != vk::Sampler::null() {
                self.device.device.destroy_sampler(self.sampler, None);
            }
            if self.image_view != vk::ImageView::null() {
                self.device.device.destroy_image_view(self.image_view, None);
            }
            if self.image != vk::Image::null() {
                self.device.device.destroy_image(self.image, None);
            }
            if self.memory != vk::DeviceMemory::null() {
                self.device.device.free_memory(self.memory, None);
            }
        }
    }
}
