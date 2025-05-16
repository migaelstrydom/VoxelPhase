use std::io::Cursor;
use std::mem;
use std::{error::Error, sync::Arc}; // For offset_of, size_of etc. if needed here later // For shader loading

use ash::{
    khr::{surface, swapchain},
    util::*, // For read_spv, Align etc. if shader/buffer logic comes here
    vk,
};
use winit::{
    raw_window_handle::{HasDisplayHandle, HasWindowHandle},
    window::Window,
};

use crate::core::vulkan_context::{
    find_memorytype_index, record_submit_commandbuffer, ManagedDevice, VulkanContext,
};

use crate::rendering::vertex::Vertex;

// ECS and Math imports
use nalgebra::Matrix4;

// Define offset_of! macro locally
#[allow(unused_macros)] // Add this in case it's not immediately used after this edit
macro_rules! offset_of {
    ($base:path, $field:ident) => {{
        #[allow(unused_unsafe)]
        unsafe {
            let b: $base = mem::zeroed();
            std::ptr::addr_of!(b.$field) as isize - std::ptr::addr_of!(b) as isize
        }
    }};
}

#[derive(Clone, Debug, Copy)]
#[repr(C)]
pub struct SceneUbo {
    pub model: Matrix4<f32>,
    pub view: Matrix4<f32>,
    pub proj: Matrix4<f32>,
}

pub struct ManagedBuffer {
    pub buffer: vk::Buffer,
    pub memory: vk::DeviceMemory,
    pub size: vk::DeviceSize,   // Store buffer size for potential use
    device: Arc<ManagedDevice>, // To call destroy/free in Drop
}

// Manual Debug implementation for ManagedBuffer
impl std::fmt::Debug for ManagedBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManagedBuffer")
            .field("buffer", &self.buffer)
            .field("memory", &self.memory)
            .field("size", &self.size)
            // Skipping self.device as ash::Device is not Debug
            .finish()
    }
}

impl ManagedBuffer {
    pub fn new(
        vulkan_context: &VulkanContext,
        size: vk::DeviceSize,
        usage: vk::BufferUsageFlags,
        memory_properties: vk::MemoryPropertyFlags,
    ) -> Result<Self, Box<dyn Error>> {
        unsafe {
            let buffer_info = vk::BufferCreateInfo::default()
                .size(size)
                .usage(usage)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);
            let buffer = vulkan_context.device().create_buffer(&buffer_info, None)?;

            let memory_req = vulkan_context
                .device()
                .get_buffer_memory_requirements(buffer);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &vulkan_context.device.device_memory_properties,
                memory_properties,
            )
            .ok_or("Unable to find suitable memory type for buffer.")?;

            let allocate_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);
            let memory = vulkan_context
                .device()
                .allocate_memory(&allocate_info, None)?;

            vulkan_context
                .device()
                .bind_buffer_memory(buffer, memory, 0)?;

            Ok(Self {
                buffer,
                memory,
                size,
                device: Arc::clone(&vulkan_context.device),
            })
        }
    }

    // Helper to map memory, returning a raw pointer
    // The caller is responsible for correct usage (size, alignment, unmapping)
    pub unsafe fn map_memory(
        &self,
        offset: vk::DeviceSize,
        flags: vk::MemoryMapFlags,
    ) -> Result<*mut std::ffi::c_void, vk::Result> {
        self.device
            .device
            .map_memory(self.memory, offset, self.size, flags)
    }

    // Helper to unmap memory
    pub unsafe fn unmap_memory(&self) {
        self.device.device.unmap_memory(self.memory);
    }
}

impl Drop for ManagedBuffer {
    fn drop(&mut self) {
        unsafe {
            if self.buffer != vk::Buffer::null() {
                self.device.device.destroy_buffer(self.buffer, None);
            }
            if self.memory != vk::DeviceMemory::null() {
                self.device.device.free_memory(self.memory, None);
            }
        }
    }
}

pub struct Renderer {
    pub surface_loader: surface::Instance,
    pub surface: vk::SurfaceKHR,
    pub swapchain_loader: swapchain::Device,

    pub present_queue: vk::Queue,
    pub _surface_format: vk::SurfaceFormatKHR,
    pub surface_resolution: vk::Extent2D,

    pub swapchain: vk::SwapchainKHR,
    pub _present_images: Vec<vk::Image>,
    pub present_image_views: Vec<vk::ImageView>,

    pub pool: vk::CommandPool,
    pub draw_command_buffer: vk::CommandBuffer,
    pub setup_command_buffer: vk::CommandBuffer,

    pub depth_image: vk::Image,
    pub depth_image_view: vk::ImageView,
    pub depth_image_memory: vk::DeviceMemory,

    pub present_complete_semaphore: vk::Semaphore,
    pub rendering_complete_semaphore: vk::Semaphore,
    pub draw_commands_reuse_fence: vk::Fence,
    pub setup_commands_reuse_fence: vk::Fence,

    // Resources for the graphics pipeline and drawing, to be initialized in new()
    pub renderpass: vk::RenderPass,
    pub framebuffers: Vec<vk::Framebuffer>,
    pub pipeline_layout: vk::PipelineLayout,
    pub graphics_pipeline: vk::Pipeline,
    pub vertex_buffer: ManagedBuffer,
    pub index_buffer: ManagedBuffer,
    pub index_count: u32,
    pub vertex_shader_module: vk::ShaderModule,
    pub fragment_shader_module: vk::ShaderModule,

    pub scene_ubo_buffer: ManagedBuffer,
    pub scene_ubo_descriptor_set_layout: vk::DescriptorSetLayout,
    pub scene_ubo_descriptor_pool: vk::DescriptorPool,
    pub scene_ubo_descriptor_set: vk::DescriptorSet,

    // VulkanContext should be the last field to ensure it's dropped last.
    pub vulkan_context: VulkanContext,
}

impl Renderer {
    pub fn new(
        vulkan_context: VulkanContext,
        window: &Window,
        window_width: u32,
        window_height: u32,
    ) -> Result<Self, Box<dyn Error>> {
        unsafe {
            let surface = ash_window::create_surface(
                &vulkan_context.entry,
                &vulkan_context.instance.instance,
                window.display_handle()?.as_raw(),
                window.window_handle()?.as_raw(),
                None,
            )?;
            let surface_loader =
                surface::Instance::new(&vulkan_context.entry, &vulkan_context.instance.instance);

            let surface_formats = surface_loader
                .get_physical_device_surface_formats(vulkan_context.physical_device(), surface)?;
            let surface_format = surface_formats[0];

            let surface_capabilities = surface_loader.get_physical_device_surface_capabilities(
                vulkan_context.physical_device(),
                surface,
            )?;
            let mut desired_image_count = surface_capabilities.min_image_count + 1;
            if surface_capabilities.max_image_count > 0
                && desired_image_count > surface_capabilities.max_image_count
            {
                desired_image_count = surface_capabilities.max_image_count;
            }

            let surface_resolution = match surface_capabilities.current_extent.width {
                u32::MAX => vk::Extent2D {
                    width: window_width,
                    height: window_height,
                },
                _ => surface_capabilities.current_extent,
            };

            let pre_transform = if surface_capabilities
                .supported_transforms
                .contains(vk::SurfaceTransformFlagsKHR::IDENTITY)
            {
                vk::SurfaceTransformFlagsKHR::IDENTITY
            } else {
                surface_capabilities.current_transform
            };

            let present_modes = surface_loader.get_physical_device_surface_present_modes(
                vulkan_context.physical_device(),
                surface,
            )?;
            let present_mode = present_modes
                .iter()
                .cloned()
                .find(|&mode| mode == vk::PresentModeKHR::MAILBOX)
                .unwrap_or(vk::PresentModeKHR::FIFO);

            let swapchain_loader =
                swapchain::Device::new(&vulkan_context.instance.instance, &vulkan_context.device());
            let swapchain_create_info = vk::SwapchainCreateInfoKHR::default()
                .surface(surface)
                .min_image_count(desired_image_count)
                .image_color_space(surface_format.color_space)
                .image_format(surface_format.format)
                .image_extent(surface_resolution)
                .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT)
                .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
                .pre_transform(pre_transform)
                .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
                .present_mode(present_mode)
                .clipped(true)
                .image_array_layers(1);
            let swapchain = swapchain_loader.create_swapchain(&swapchain_create_info, None)?;

            let pool_create_info = vk::CommandPoolCreateInfo::default()
                .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER)
                .queue_family_index(vulkan_context.device.queue_family_index);
            let pool = vulkan_context
                .device()
                .create_command_pool(&pool_create_info, None)?;

            let command_buffer_allocate_info = vk::CommandBufferAllocateInfo::default()
                .command_buffer_count(2)
                .command_pool(pool)
                .level(vk::CommandBufferLevel::PRIMARY);
            let command_buffers = vulkan_context
                .device()
                .allocate_command_buffers(&command_buffer_allocate_info)?;
            let setup_command_buffer = command_buffers[0];
            let draw_command_buffer = command_buffers[1];

            let present_images = swapchain_loader.get_swapchain_images(swapchain)?;
            let present_image_views: Vec<vk::ImageView> = present_images
                .iter()
                .map(|&image| {
                    let create_view_info = vk::ImageViewCreateInfo::default()
                        .view_type(vk::ImageViewType::TYPE_2D)
                        .format(surface_format.format)
                        .components(vk::ComponentMapping {
                            r: vk::ComponentSwizzle::R,
                            g: vk::ComponentSwizzle::G,
                            b: vk::ComponentSwizzle::B,
                            a: vk::ComponentSwizzle::A,
                        })
                        .subresource_range(vk::ImageSubresourceRange {
                            aspect_mask: vk::ImageAspectFlags::COLOR,
                            base_mip_level: 0,
                            level_count: 1,
                            base_array_layer: 0,
                            layer_count: 1,
                        })
                        .image(image);
                    vulkan_context
                        .device()
                        .create_image_view(&create_view_info, None)
                })
                .collect::<Result<Vec<_>, _>>()?;

            let depth_image_create_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(vk::Format::D16_UNORM)
                .extent(surface_resolution.into())
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(vk::ImageTiling::OPTIMAL)
                .usage(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let depth_image = vulkan_context
                .device()
                .create_image(&depth_image_create_info, None)?;

            let depth_image_memory_req = vulkan_context
                .device()
                .get_image_memory_requirements(depth_image);
            let depth_image_memory_index = find_memorytype_index(
                &depth_image_memory_req,
                &vulkan_context.device.device_memory_properties,
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            )
            .expect("Unable to find suitable memory index for depth image.");

            let depth_image_allocate_info = vk::MemoryAllocateInfo::default()
                .allocation_size(depth_image_memory_req.size)
                .memory_type_index(depth_image_memory_index);
            let depth_image_memory = vulkan_context
                .device()
                .allocate_memory(&depth_image_allocate_info, None)?;

            vulkan_context
                .device()
                .bind_image_memory(depth_image, depth_image_memory, 0)?;

            let fence_create_info =
                vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);
            let draw_commands_reuse_fence = vulkan_context
                .device()
                .create_fence(&fence_create_info, None)?;
            let setup_commands_reuse_fence = vulkan_context
                .device()
                .create_fence(&fence_create_info, None)?;

            let present_queue = vulkan_context
                .device()
                .get_device_queue(vulkan_context.device.queue_family_index, 0);

            vulkan_context.device().wait_for_fences(
                &[setup_commands_reuse_fence],
                true,
                u64::MAX,
            )?;
            vulkan_context
                .device()
                .reset_fences(&[setup_commands_reuse_fence])?;
            record_submit_commandbuffer(
                &vulkan_context.device(),
                setup_command_buffer,
                setup_commands_reuse_fence,
                present_queue,
                &[],
                &[],
                &[],
                |device, setup_cb| {
                    let layout_transition_barriers = vk::ImageMemoryBarrier::default()
                        .image(depth_image)
                        .dst_access_mask(
                            vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_READ
                                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
                        )
                        .new_layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
                        .old_layout(vk::ImageLayout::UNDEFINED)
                        .subresource_range(
                            vk::ImageSubresourceRange::default()
                                .aspect_mask(vk::ImageAspectFlags::DEPTH)
                                .layer_count(1)
                                .level_count(1),
                        );
                    device.cmd_pipeline_barrier(
                        setup_cb,
                        vk::PipelineStageFlags::BOTTOM_OF_PIPE,
                        vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[layout_transition_barriers],
                    );
                },
            );

            let depth_image_view_info = vk::ImageViewCreateInfo::default()
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::DEPTH)
                        .level_count(1)
                        .layer_count(1),
                )
                .image(depth_image)
                .format(depth_image_create_info.format)
                .view_type(vk::ImageViewType::TYPE_2D);
            let depth_image_view = vulkan_context
                .device()
                .create_image_view(&depth_image_view_info, None)?;

            let semaphore_create_info = vk::SemaphoreCreateInfo::default();
            let present_complete_semaphore = vulkan_context
                .device()
                .create_semaphore(&semaphore_create_info, None)?;
            let rendering_complete_semaphore = vulkan_context
                .device()
                .create_semaphore(&semaphore_create_info, None)?;

            // 1. Render Pass
            let renderpass_attachments = [
                vk::AttachmentDescription {
                    format: surface_format.format,
                    samples: vk::SampleCountFlags::TYPE_1,
                    load_op: vk::AttachmentLoadOp::CLEAR,
                    store_op: vk::AttachmentStoreOp::STORE,
                    final_layout: vk::ImageLayout::PRESENT_SRC_KHR,
                    ..Default::default()
                },
                vk::AttachmentDescription {
                    format: vk::Format::D16_UNORM, // Depth format
                    samples: vk::SampleCountFlags::TYPE_1,
                    load_op: vk::AttachmentLoadOp::CLEAR,
                    initial_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    ..Default::default()
                },
            ];
            let color_attachment_refs = [vk::AttachmentReference {
                attachment: 0,
                layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
            }];
            let depth_attachment_ref = vk::AttachmentReference {
                attachment: 1,
                layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
            };
            let dependencies = [vk::SubpassDependency {
                src_subpass: vk::SUBPASS_EXTERNAL,
                src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                    | vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                ..Default::default()
            }];
            let subpass = vk::SubpassDescription::default()
                .color_attachments(&color_attachment_refs)
                .depth_stencil_attachment(&depth_attachment_ref)
                .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS);
            let renderpass_create_info = vk::RenderPassCreateInfo::default()
                .attachments(&renderpass_attachments)
                .subpasses(std::slice::from_ref(&subpass))
                .dependencies(&dependencies);
            let renderpass = vulkan_context
                .device()
                .create_render_pass(&renderpass_create_info, None)?;

            // 2. Framebuffers
            let framebuffers: Vec<vk::Framebuffer> = present_image_views
                .iter()
                .map(|&present_image_view| {
                    let framebuffer_attachments = [present_image_view, depth_image_view];
                    let frame_buffer_create_info = vk::FramebufferCreateInfo::default()
                        .render_pass(renderpass)
                        .attachments(&framebuffer_attachments)
                        .width(surface_resolution.width)
                        .height(surface_resolution.height)
                        .layers(1);
                    vulkan_context
                        .device()
                        .create_framebuffer(&frame_buffer_create_info, None)
                })
                .collect::<Result<Vec<_>, _>>()?;

            // 3. Index Buffer (using ManagedBuffer) - Placeholder, will be dynamic
            // Initialize with a small size or leave for first draw_mesh_data call
            let initial_index_buffer_size = mem::size_of::<u32>() as u64 * 3; // e.g., for one triangle
            let index_buffer = ManagedBuffer::new(
                &vulkan_context,
                initial_index_buffer_size, // Placeholder size
                vk::BufferUsageFlags::INDEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            let index_count = 0; // No static index count anymore

            // 4. Vertex Buffer (using ManagedBuffer) - Placeholder, will be dynamic
            let initial_vertex_buffer_size = mem::size_of::<Vertex>() as u64 * 3; // e.g., for one triangle
            let vertex_buffer = ManagedBuffer::new(
                &vulkan_context,
                initial_vertex_buffer_size, // Placeholder size
                vk::BufferUsageFlags::VERTEX_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;

            // 5. Shader Modules
            let mut vertex_spv_file = Cursor::new(&include_bytes!("../../shader/vert.spv")[..]);
            let mut frag_spv_file = Cursor::new(&include_bytes!("../../shader/frag.spv")[..]);
            let vertex_code =
                read_spv(&mut vertex_spv_file).expect("Failed to read vertex shader spv file");
            let vertex_shader_info = vk::ShaderModuleCreateInfo::default().code(&vertex_code);
            let vertex_shader_module = vulkan_context
                .device()
                .create_shader_module(&vertex_shader_info, None)
                .expect("Vertex shader module error");
            let frag_code =
                read_spv(&mut frag_spv_file).expect("Failed to read fragment shader spv file");
            let frag_shader_info = vk::ShaderModuleCreateInfo::default().code(&frag_code);
            let fragment_shader_module = vulkan_context
                .device()
                .create_shader_module(&frag_shader_info, None)
                .expect("Fragment shader module error");

            // Create Uniform Buffer for Scene Data (Model, View, Projection)
            let scene_ubo_size = std::mem::size_of::<SceneUbo>() as vk::DeviceSize;
            let scene_ubo_buffer = ManagedBuffer::new(
                &vulkan_context,
                scene_ubo_size,
                vk::BufferUsageFlags::UNIFORM_BUFFER,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;

            // Create DescriptorSetLayout for SceneUBO
            let ubo_descriptor_set_layout_bindings = [vk::DescriptorSetLayoutBinding::default()
                .binding(0) // Corresponds to layout(binding=0) in shader
                .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                .descriptor_count(1)
                .stage_flags(vk::ShaderStageFlags::VERTEX)]; // UBO used in vertex shader
            let ubo_descriptor_set_layout_create_info =
                vk::DescriptorSetLayoutCreateInfo::default()
                    .bindings(&ubo_descriptor_set_layout_bindings);
            let scene_ubo_descriptor_set_layout = vulkan_context
                .device()
                .create_descriptor_set_layout(&ubo_descriptor_set_layout_create_info, None)?;

            // Create Pipeline Layout using the descriptor set layout
            let set_layouts = [scene_ubo_descriptor_set_layout];
            let pipeline_layout_create_info =
                vk::PipelineLayoutCreateInfo::default().set_layouts(&set_layouts);
            let pipeline_layout = vulkan_context
                .device()
                .create_pipeline_layout(&pipeline_layout_create_info, None)?;

            // Create Descriptor Pool for SceneUBO
            let descriptor_pool_sizes = [vk::DescriptorPoolSize::default()
                .ty(vk::DescriptorType::UNIFORM_BUFFER)
                .descriptor_count(1)];
            let descriptor_pool_create_info = vk::DescriptorPoolCreateInfo::default()
                .max_sets(1)
                .pool_sizes(&descriptor_pool_sizes);
            let scene_ubo_descriptor_pool = vulkan_context
                .device()
                .create_descriptor_pool(&descriptor_pool_create_info, None)?;

            // Allocate Descriptor Set
            let descriptor_set_allocate_info = vk::DescriptorSetAllocateInfo::default()
                .descriptor_pool(scene_ubo_descriptor_pool)
                .set_layouts(std::slice::from_ref(&scene_ubo_descriptor_set_layout));
            let scene_ubo_descriptor_sets = vulkan_context
                .device()
                .allocate_descriptor_sets(&descriptor_set_allocate_info)?;
            let scene_ubo_descriptor_set = scene_ubo_descriptor_sets[0];

            // Update Descriptor Set to point to the UBO buffer
            let ubo_buffer_info_for_descriptor = [vk::DescriptorBufferInfo::default()
                .buffer(scene_ubo_buffer.buffer)
                .offset(0)
                .range(std::mem::size_of::<SceneUbo>() as u64)];
            let write_descriptor_sets = [vk::WriteDescriptorSet::default()
                .dst_set(scene_ubo_descriptor_set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
                .buffer_info(&ubo_buffer_info_for_descriptor)];
            vulkan_context
                .device()
                .update_descriptor_sets(&write_descriptor_sets, &[]);

            // Graphics Pipeline (uses the pipeline_layout)
            let shader_entry_name = std::ffi::CStr::from_bytes_with_nul(b"main\0").unwrap();
            let shader_stage_create_infos = [
                vk::PipelineShaderStageCreateInfo {
                    module: vertex_shader_module,
                    p_name: shader_entry_name.as_ptr(),
                    stage: vk::ShaderStageFlags::VERTEX,
                    ..Default::default()
                },
                vk::PipelineShaderStageCreateInfo {
                    module: fragment_shader_module,
                    p_name: shader_entry_name.as_ptr(),
                    stage: vk::ShaderStageFlags::FRAGMENT,
                    ..Default::default()
                },
            ];
            let vertex_input_binding_descriptions = [vk::VertexInputBindingDescription {
                binding: 0,
                stride: mem::size_of::<Vertex>() as u32,
                input_rate: vk::VertexInputRate::VERTEX,
            }];
            let vertex_input_attribute_descriptions = [
                vk::VertexInputAttributeDescription {
                    location: 0,
                    binding: 0,
                    format: vk::Format::R32G32B32A32_SFLOAT,
                    offset: offset_of!(Vertex, pos) as u32,
                },
                vk::VertexInputAttributeDescription {
                    location: 1,
                    binding: 0,
                    format: vk::Format::R32G32B32A32_SFLOAT,
                    offset: offset_of!(Vertex, color) as u32,
                },
            ];
            let vertex_input_state_info = vk::PipelineVertexInputStateCreateInfo::default()
                .vertex_attribute_descriptions(&vertex_input_attribute_descriptions)
                .vertex_binding_descriptions(&vertex_input_binding_descriptions);
            let vertex_input_assembly_state_info = vk::PipelineInputAssemblyStateCreateInfo {
                topology: vk::PrimitiveTopology::TRIANGLE_LIST,
                ..Default::default()
            };
            let viewports = [vk::Viewport {
                x: 0.0,
                y: 0.0,
                width: surface_resolution.width as f32,
                height: surface_resolution.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            }];
            let scissors = [surface_resolution.into()];
            let viewport_state_info = vk::PipelineViewportStateCreateInfo::default()
                .scissors(&scissors)
                .viewports(&viewports);
            let rasterization_info = vk::PipelineRasterizationStateCreateInfo {
                front_face: vk::FrontFace::COUNTER_CLOCKWISE,
                line_width: 1.0,
                polygon_mode: vk::PolygonMode::FILL,
                ..Default::default()
            };
            let multisample_state_info = vk::PipelineMultisampleStateCreateInfo {
                rasterization_samples: vk::SampleCountFlags::TYPE_1,
                ..Default::default()
            };
            let noop_stencil_state = vk::StencilOpState {
                fail_op: vk::StencilOp::KEEP,
                pass_op: vk::StencilOp::KEEP,
                depth_fail_op: vk::StencilOp::KEEP,
                compare_op: vk::CompareOp::ALWAYS,
                ..Default::default()
            };
            let depth_state_info = vk::PipelineDepthStencilStateCreateInfo {
                depth_test_enable: 1,
                depth_write_enable: 1,
                depth_compare_op: vk::CompareOp::LESS_OR_EQUAL,
                front: noop_stencil_state,
                back: noop_stencil_state,
                max_depth_bounds: 1.0,
                ..Default::default()
            };
            let color_blend_attachment_states = [vk::PipelineColorBlendAttachmentState {
                blend_enable: 0,
                src_color_blend_factor: vk::BlendFactor::SRC_COLOR,
                dst_color_blend_factor: vk::BlendFactor::ONE_MINUS_DST_COLOR,
                color_blend_op: vk::BlendOp::ADD,
                src_alpha_blend_factor: vk::BlendFactor::ZERO,
                dst_alpha_blend_factor: vk::BlendFactor::ZERO,
                alpha_blend_op: vk::BlendOp::ADD,
                color_write_mask: vk::ColorComponentFlags::RGBA,
            }];
            let color_blend_state = vk::PipelineColorBlendStateCreateInfo::default()
                .logic_op(vk::LogicOp::CLEAR)
                .attachments(&color_blend_attachment_states);
            let dynamic_state = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
            let dynamic_state_info =
                vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_state);
            let graphic_pipeline_info = vk::GraphicsPipelineCreateInfo::default()
                .stages(&shader_stage_create_infos)
                .vertex_input_state(&vertex_input_state_info)
                .input_assembly_state(&vertex_input_assembly_state_info)
                .viewport_state(&viewport_state_info)
                .rasterization_state(&rasterization_info)
                .multisample_state(&multisample_state_info)
                .depth_stencil_state(&depth_state_info)
                .color_blend_state(&color_blend_state)
                .dynamic_state(&dynamic_state_info)
                .layout(pipeline_layout)
                .render_pass(renderpass);
            let graphics_pipelines = vulkan_context
                .device()
                .create_graphics_pipelines(
                    vk::PipelineCache::null(),
                    &[graphic_pipeline_info],
                    None,
                )
                .expect("Unable to create graphics pipeline");
            let graphics_pipeline = graphics_pipelines[0];

            Ok(Self {
                surface_loader,
                surface,
                swapchain_loader,
                present_queue,
                _surface_format: surface_format,
                surface_resolution,
                swapchain,
                _present_images: present_images,
                present_image_views,
                pool,
                draw_command_buffer,
                setup_command_buffer,
                depth_image,
                depth_image_view,
                depth_image_memory,
                present_complete_semaphore,
                rendering_complete_semaphore,
                draw_commands_reuse_fence,
                setup_commands_reuse_fence,
                renderpass,
                framebuffers,
                pipeline_layout,
                graphics_pipeline,
                vertex_buffer,
                index_buffer,
                index_count,
                vertex_shader_module,
                fragment_shader_module,
                scene_ubo_buffer,
                scene_ubo_descriptor_set_layout,
                scene_ubo_descriptor_pool,
                scene_ubo_descriptor_set,
                vulkan_context,
            })
        }
    }

    // Helper to re-create managed buffer if needed (e.g. if too small)
    // This is a simplified version. A more robust implementation might try to reuse memory
    // or use staging buffers for DEVICE_LOCAL buffers.
    unsafe fn ensure_buffer_size(
        vulkan_context: &VulkanContext,
        existing_buffer: &mut ManagedBuffer,
        required_size: vk::DeviceSize,
        usage: vk::BufferUsageFlags,
        memory_properties: vk::MemoryPropertyFlags,
    ) -> Result<(), Box<dyn Error>> {
        if existing_buffer.size < required_size {
            // existing_buffer will be dropped, freeing its old resources
            *existing_buffer =
                ManagedBuffer::new(vulkan_context, required_size, usage, memory_properties)?;
        }
        Ok(())
    }

    pub fn prepare_frame_for_rendering(
        &self,
    ) -> Result<(vk::CommandBuffer, usize /*present_index*/), Box<dyn Error>> {
        unsafe {
            self.vulkan_context.device().wait_for_fences(
                &[self.draw_commands_reuse_fence],
                true,
                u64::MAX,
            )?;
            self.vulkan_context
                .device()
                .reset_fences(&[self.draw_commands_reuse_fence])?;

            let (present_index, _) = self.swapchain_loader.acquire_next_image(
                self.swapchain,
                u64::MAX,
                self.present_complete_semaphore,
                vk::Fence::null(),
            )?;

            let command_buffer_begin_info = vk::CommandBufferBeginInfo::default()
                .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
            self.vulkan_context
                .device()
                .begin_command_buffer(self.draw_command_buffer, &command_buffer_begin_info)?;

            let clear_values = [
                vk::ClearValue {
                    color: vk::ClearColorValue {
                        float32: [0.0, 0.0, 0.0, 0.0],
                    },
                },
                vk::ClearValue {
                    depth_stencil: vk::ClearDepthStencilValue {
                        depth: 1.0,
                        stencil: 0,
                    },
                },
            ];

            let render_pass_begin_info = vk::RenderPassBeginInfo::default()
                .render_pass(self.renderpass)
                .framebuffer(self.framebuffers[present_index as usize])
                .render_area(self.surface_resolution.into())
                .clear_values(&clear_values);

            self.vulkan_context.device().cmd_begin_render_pass(
                self.draw_command_buffer,
                &render_pass_begin_info,
                vk::SubpassContents::INLINE,
            );

            Ok((self.draw_command_buffer, present_index as usize))
        }
    }

    // Part 2: Drawing a specific mesh
    pub unsafe fn draw_mesh_data(
        &mut self,
        draw_cb: vk::CommandBuffer, // Current command buffer
        vertices: &[Vertex],
        indices: &[u32],
        model_matrix: &Matrix4<f32>,
        view_matrix: &Matrix4<f32>,
        proj_matrix: &Matrix4<f32>,
    ) -> Result<(), Box<dyn Error>> {
        // Update Vertex Buffer
        let vertex_buffer_size = (mem::size_of::<Vertex>() * vertices.len()) as vk::DeviceSize;
        Self::ensure_buffer_size(
            &self.vulkan_context,
            &mut self.vertex_buffer,
            vertex_buffer_size,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;
        let vert_ptr = self
            .vertex_buffer
            .map_memory(0, vk::MemoryMapFlags::empty())?;
        let mut vert_align = Align::new(
            vert_ptr,
            mem::align_of::<Vertex>() as u64,
            vertex_buffer_size, // Use actual data size for copy
        );
        vert_align.copy_from_slice(vertices);
        self.vertex_buffer.unmap_memory();

        // Update Index Buffer
        let index_buffer_size = (mem::size_of::<u32>() * indices.len()) as vk::DeviceSize;
        Self::ensure_buffer_size(
            &self.vulkan_context,
            &mut self.index_buffer,
            index_buffer_size,
            vk::BufferUsageFlags::INDEX_BUFFER,
            vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
        )?;
        let index_ptr = self
            .index_buffer
            .map_memory(0, vk::MemoryMapFlags::empty())?;
        let mut index_align = Align::new(
            index_ptr,
            mem::align_of::<u32>() as u64,
            index_buffer_size, // Use actual data size for copy
        );
        index_align.copy_from_slice(indices);
        self.index_buffer.unmap_memory();

        self.index_count = indices.len() as u32; // Update index count for this draw

        // Update UBO
        let current_scene_ubo = SceneUbo {
            model: *model_matrix,
            view: *view_matrix,
            proj: *proj_matrix,
        };
        let ubo_ptr = self
            .scene_ubo_buffer
            .map_memory(0, vk::MemoryMapFlags::empty())?;
        let ubo_slice = std::slice::from_raw_parts_mut(ubo_ptr as *mut SceneUbo, 1);
        ubo_slice[0] = current_scene_ubo;
        self.scene_ubo_buffer.unmap_memory();

        // Bind pipeline and descriptor sets (assuming these are common for all meshes for now)
        self.vulkan_context.device().cmd_bind_pipeline(
            draw_cb,
            vk::PipelineBindPoint::GRAPHICS,
            self.graphics_pipeline,
        );

        let viewports = [vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: self.surface_resolution.width as f32,
            height: self.surface_resolution.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        }];
        let scissors = [self.surface_resolution.into()];
        self.vulkan_context
            .device()
            .cmd_set_viewport(draw_cb, 0, &viewports);
        self.vulkan_context
            .device()
            .cmd_set_scissor(draw_cb, 0, &scissors);

        self.vulkan_context.device().cmd_bind_descriptor_sets(
            draw_cb,
            vk::PipelineBindPoint::GRAPHICS,
            self.pipeline_layout,
            0,
            &[self.scene_ubo_descriptor_set],
            &[],
        );

        // Bind buffers and draw
        self.vulkan_context.device().cmd_bind_vertex_buffers(
            draw_cb,
            0,
            &[self.vertex_buffer.buffer],
            &[0],
        );
        self.vulkan_context.device().cmd_bind_index_buffer(
            draw_cb,
            self.index_buffer.buffer,
            0,
            vk::IndexType::UINT32,
        );
        self.vulkan_context
            .device()
            .cmd_draw_indexed(draw_cb, self.index_count, 1, 0, 0, 0);

        Ok(())
    }

    // Part 3: Finalizing and presenting the frame
    pub fn finalize_frame_and_present(
        &self,
        draw_cb: vk::CommandBuffer,
        present_index: usize,
    ) -> Result<(), Box<dyn Error>> {
        unsafe {
            self.vulkan_context.device().cmd_end_render_pass(draw_cb);
            self.vulkan_context.device().end_command_buffer(draw_cb)?;

            let submit_infos = [vk::SubmitInfo::default()
                .wait_semaphores(std::slice::from_ref(&self.present_complete_semaphore))
                .wait_dst_stage_mask(&[vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT])
                .command_buffers(std::slice::from_ref(&draw_cb))
                .signal_semaphores(std::slice::from_ref(&self.rendering_complete_semaphore))];

            self.vulkan_context.device().queue_submit(
                self.present_queue,
                &submit_infos,
                self.draw_commands_reuse_fence,
            )?;

            let wait_semaphors = [self.rendering_complete_semaphore];
            let swapchains = [self.swapchain];
            let image_indices = [present_index as u32];
            let present_info = vk::PresentInfoKHR::default()
                .wait_semaphores(&wait_semaphors)
                .swapchains(&swapchains)
                .image_indices(&image_indices);

            self.swapchain_loader
                .queue_present(self.present_queue, &present_info)?;
        }
        Ok(())
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            // It's crucial that device_wait_idle is called before any dependent resources
            // are destroyed. Since VulkanContext is now dropped last, its Drop impl
            // (which should contain device destruction) will run after all other fields
            // in Renderer are dropped. The device_wait_idle here in Renderer::drop
            // ensures that the GPU is idle before Renderer starts dropping its own fields
            // (like ManagedBuffers, semaphores, etc.).
            self.vulkan_context.device().device_wait_idle().unwrap();

            // Destroy new UBO-related resources (order can matter)
            // Descriptor sets are implicitly freed with the pool
            if self.scene_ubo_descriptor_pool != vk::DescriptorPool::null() {
                self.vulkan_context
                    .device()
                    .destroy_descriptor_pool(self.scene_ubo_descriptor_pool, None);
            }
            if self.scene_ubo_descriptor_set_layout != vk::DescriptorSetLayout::null() {
                self.vulkan_context
                    .device()
                    .destroy_descriptor_set_layout(self.scene_ubo_descriptor_set_layout, None);
            }
            // scene_ubo_buffer (ManagedBuffer) will be dropped automatically,
            // no need to explicitly destroy buffer or free memory here.

            // Existing cleanup (ensure order is still correct relative to new items if dependencies exist)
            self.vulkan_context
                .device()
                .destroy_shader_module(self.vertex_shader_module, None);
            self.vulkan_context
                .device()
                .destroy_shader_module(self.fragment_shader_module, None);
            // Pipeline must be destroyed before its layout
            self.vulkan_context
                .device()
                .destroy_pipeline(self.graphics_pipeline, None);
            self.vulkan_context
                .device()
                .destroy_pipeline_layout(self.pipeline_layout, None);
            // RenderPass can be destroyed after pipelines that use it
            self.vulkan_context
                .device()
                .destroy_render_pass(self.renderpass, None);
            for framebuffer in &self.framebuffers {
                self.vulkan_context
                    .device()
                    .destroy_framebuffer(*framebuffer, None);
            }
            self.vulkan_context.device().free_command_buffers(
                self.pool,
                &[self.draw_command_buffer, self.setup_command_buffer],
            );
            self.vulkan_context
                .device()
                .destroy_command_pool(self.pool, None);
            self.vulkan_context
                .device()
                .destroy_image_view(self.depth_image_view, None);
            self.vulkan_context
                .device()
                .destroy_image(self.depth_image, None);
            self.vulkan_context
                .device()
                .free_memory(self.depth_image_memory, None);
            for &image_view in self.present_image_views.iter() {
                self.vulkan_context
                    .device()
                    .destroy_image_view(image_view, None);
            }
            self.swapchain_loader
                .destroy_swapchain(self.swapchain, None);
            self.vulkan_context
                .device()
                .destroy_semaphore(self.present_complete_semaphore, None);
            self.vulkan_context
                .device()
                .destroy_semaphore(self.rendering_complete_semaphore, None);
            self.vulkan_context
                .device()
                .destroy_fence(self.draw_commands_reuse_fence, None);
            self.vulkan_context
                .device()
                .destroy_fence(self.setup_commands_reuse_fence, None);
            self.surface_loader.destroy_surface(self.surface, None);
            // VulkanContext (which holds device and instance) is dropped after Renderer's other fields,
            // handling their cleanup IF `vulkan_context` is the last field.
            // The `device_wait_idle()` call at the start of this function is for Renderer's resources.
            // The `VulkanContext` itself should manage the wait/destroy for device/instance in its own Drop.
        }
    }
}
