//! Swapchain and presentation management.
//!
//! This module handles swapchain creation, image views, depth buffer,
//! framebuffers, and frame synchronization.

use std::sync::Arc;

use ash::{
    khr::{surface, swapchain},
    vk,
};
use winit::{
    raw_window_handle::{HasDisplayHandle, HasWindowHandle},
    window::Window,
};

use crate::core::command_buffer::ManagedCommandBuffer;
use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, VkResultExt};
use crate::core::vulkan_context::{find_memorytype_index, VulkanContext};

/// Frame synchronization primitives.
pub struct FrameSync {
    pub present_complete: vk::Semaphore,
    pub rendering_complete: vk::Semaphore,
    pub draw_fence: vk::Fence,
    device: Arc<ManagedDevice>,
}

impl FrameSync {
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        let semaphore_info = vk::SemaphoreCreateInfo::default();
        let fence_info = vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);

        unsafe {
            let present_complete = device
                .device
                .create_semaphore(&semaphore_info, None)
                .sync_context("create present semaphore")?;

            let rendering_complete = device
                .device
                .create_semaphore(&semaphore_info, None)
                .sync_context("create rendering semaphore")?;

            let draw_fence = device
                .device
                .create_fence(&fence_info, None)
                .sync_context("create draw fence")?;

            Ok(Self {
                present_complete,
                rendering_complete,
                draw_fence,
                device,
            })
        }
    }

    /// Wait for the draw fence and reset it.
    pub fn wait_and_reset(&self) -> EngineResult<()> {
        unsafe {
            self.device
                .device
                .wait_for_fences(&[self.draw_fence], true, u64::MAX)
                .sync_context("wait for draw fence")?;

            self.device
                .device
                .reset_fences(&[self.draw_fence])
                .sync_context("reset draw fence")?;
        }
        Ok(())
    }
}

impl Drop for FrameSync {
    fn drop(&mut self) {
        unsafe {
            self.device
                .device
                .destroy_semaphore(self.present_complete, None);
            self.device
                .device
                .destroy_semaphore(self.rendering_complete, None);
            self.device.device.destroy_fence(self.draw_fence, None);
        }
    }
}

/// Depth buffer resources.
pub struct DepthBuffer {
    pub image: vk::Image,
    pub view: vk::ImageView,
    pub memory: vk::DeviceMemory,
    device: Arc<ManagedDevice>,
}

impl DepthBuffer {
    pub fn new(
        vulkan_context: &VulkanContext,
        extent: vk::Extent2D,
        format: vk::Format,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        unsafe {
            let image_info = vk::ImageCreateInfo::default()
                .image_type(vk::ImageType::TYPE_2D)
                .format(format)
                .extent(extent.into())
                .mip_levels(1)
                .array_layers(1)
                .samples(vk::SampleCountFlags::TYPE_1)
                .tiling(vk::ImageTiling::OPTIMAL)
                .usage(vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT)
                .sharing_mode(vk::SharingMode::EXCLUSIVE);

            let image =
                device
                    .device
                    .create_image(&image_info, None)
                    .map_err(|e| EngineError::Image {
                        operation: crate::core::error::ImageOperation::Create,
                        width: extent.width,
                        height: extent.height,
                        reason: format!("{:?}", e),
                    })?;

            let memory_req = device.device.get_image_memory_requirements(image);
            let memory_type_index = find_memorytype_index(
                &memory_req,
                &device.device_memory_properties,
                vk::MemoryPropertyFlags::DEVICE_LOCAL,
            )
            .ok_or_else(|| EngineError::Image {
                operation: crate::core::error::ImageOperation::AllocateMemory,
                width: extent.width,
                height: extent.height,
                reason: "no suitable memory type".to_string(),
            })?;

            let alloc_info = vk::MemoryAllocateInfo::default()
                .allocation_size(memory_req.size)
                .memory_type_index(memory_type_index);

            let memory = device
                .device
                .allocate_memory(&alloc_info, None)
                .map_err(|e| EngineError::Image {
                    operation: crate::core::error::ImageOperation::AllocateMemory,
                    width: extent.width,
                    height: extent.height,
                    reason: format!("{:?}", e),
                })?;

            device
                .device
                .bind_image_memory(image, memory, 0)
                .map_err(|e| EngineError::Image {
                    operation: crate::core::error::ImageOperation::Bind,
                    width: extent.width,
                    height: extent.height,
                    reason: format!("{:?}", e),
                })?;

            // Transition depth image layout
            Self::transition_layout(vulkan_context, image)?;

            let view_info = vk::ImageViewCreateInfo::default()
                .image(image)
                .view_type(vk::ImageViewType::TYPE_2D)
                .format(format)
                .subresource_range(
                    vk::ImageSubresourceRange::default()
                        .aspect_mask(vk::ImageAspectFlags::DEPTH)
                        .level_count(1)
                        .layer_count(1),
                );

            let view = device
                .device
                .create_image_view(&view_info, None)
                .map_err(|e| EngineError::Image {
                    operation: crate::core::error::ImageOperation::CreateView,
                    width: extent.width,
                    height: extent.height,
                    reason: format!("{:?}", e),
                })?;

            Ok(Self {
                image,
                view,
                memory,
                device,
            })
        }
    }

    /// Transitions a depth image from `UNDEFINED` to `DEPTH_STENCIL_ATTACHMENT_OPTIMAL` layout.
    ///
    /// In Vulkan, images must be in a specific layout to be used efficiently by the GPU. When an
    /// image is first created, it starts in the `UNDEFINED` layout, which means the image contents
    /// are undefined and the GPU may not access it in a meaningful way. Before using an image as a
    /// depth attachment in a render pass, it must be transitioned to `DEPTH_STENCIL_ATTACHMENT_OPTIMAL`,
    /// which tells the GPU that the image will be used for depth/stencil testing and should be
    /// arranged in memory for optimal depth buffer access.
    ///
    /// This function:
    /// 1. Creates a one-time submit command buffer
    /// 2. Records a pipeline barrier that transitions the image layout
    /// 3. Sets up proper access masks for depth/stencil attachment read/write operations
    /// 4. Submits the command and waits for completion
    ///
    /// # Parameters
    /// * `vulkan_context` - The Vulkan context containing the command buffer manager
    /// * `image` - The depth image to transition
    ///
    /// # Returns
    /// * `Ok(())` if the layout transition completed successfully
    /// * `Err(EngineError)` if command buffer creation or submission failed
    ///
    /// # Note
    /// This is a blocking operation that waits for the GPU to complete the transition before
    /// returning. This is safe for initialization code but should be avoided in hot paths.
    fn transition_layout(vulkan_context: &VulkanContext, image: vk::Image) -> EngineResult<()> {
        let cmd_buffer = vulkan_context
            .command_buffer_manager
            .create_one_time_submit_buffer()?;

        vulkan_context
            .command_buffer_manager
            .submit_graphics_commands_and_wait(&cmd_buffer, |device, cb| {
                let barrier = vk::ImageMemoryBarrier::default()
                    .image(image)
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

                unsafe {
                    device.cmd_pipeline_barrier(
                        cb,
                        vk::PipelineStageFlags::BOTTOM_OF_PIPE,
                        vk::PipelineStageFlags::LATE_FRAGMENT_TESTS,
                        vk::DependencyFlags::empty(),
                        &[],
                        &[],
                        &[barrier],
                    );
                }
            })
    }
}

impl Drop for DepthBuffer {
    fn drop(&mut self) {
        unsafe {
            self.device.device.destroy_image_view(self.view, None);
            self.device.device.destroy_image(self.image, None);
            self.device.device.free_memory(self.memory, None);
        }
    }
}

/// Swapchain and presentation resources.
pub struct Swapchain {
    pub handle: vk::SwapchainKHR,
    pub loader: swapchain::Device,
    pub surface_loader: surface::Instance,
    pub surface: vk::SurfaceKHR,
    pub extent: vk::Extent2D,
    pub image_views: Vec<vk::ImageView>,
    pub framebuffers: Vec<vk::Framebuffer>,
    /// Depth buffer used by all framebuffers. Kept alive to ensure the depth buffer view
    /// referenced by the framebuffers remains valid. Automatically cleaned up when Swapchain is dropped.
    #[allow(dead_code)] // Kept for lifetime management, not direct access
    depth_buffer: DepthBuffer,
    pub sync: FrameSync,
    pub draw_command_buffer: ManagedCommandBuffer,
    device: Arc<ManagedDevice>,
}

/// Surface and format information needed for swapchain creation.
pub struct SurfaceInfo {
    pub surface: vk::SurfaceKHR,
    pub surface_loader: surface::Instance,
    pub format: vk::SurfaceFormatKHR,
}

impl SurfaceInfo {
    /// Create a surface and query its preferred format.
    /// Prefers sRGB formats for correct gamma handling.
    pub fn new(vulkan_context: &VulkanContext, window: &Window) -> EngineResult<Self> {
        unsafe {
            let display_handle = window
                .display_handle()
                .map_err(|e| EngineError::Surface(format!("display handle: {:?}", e)))?;
            let window_handle = window
                .window_handle()
                .map_err(|e| EngineError::Surface(format!("window handle: {:?}", e)))?;

            let surface = ash_window::create_surface(
                &vulkan_context.entry,
                &vulkan_context.instance.instance,
                display_handle.as_raw(),
                window_handle.as_raw(),
                None,
            )
            .map_err(|e| EngineError::Surface(format!("creation: {:?}", e)))?;

            let surface_loader =
                surface::Instance::new(&vulkan_context.entry, &vulkan_context.instance.instance);

            let formats = surface_loader
                .get_physical_device_surface_formats(vulkan_context.physical_device(), surface)
                .map_err(|e| EngineError::Surface(format!("get formats: {:?}", e)))?;

            // Prefer sRGB format for correct gamma handling
            let format = formats
                .iter()
                .find(|f| {
                    f.format == vk::Format::B8G8R8A8_SRGB || f.format == vk::Format::R8G8B8A8_SRGB
                })
                .copied()
                .unwrap_or(formats[0]);

            Ok(Self {
                surface,
                surface_loader,
                format,
            })
        }
    }
}

impl Swapchain {
    pub fn new(
        vulkan_context: Arc<VulkanContext>,
        surface_info: SurfaceInfo,
        renderpass: vk::RenderPass,
        window_width: u32,
        window_height: u32,
    ) -> EngineResult<Self> {
        let device = Arc::clone(&vulkan_context.device);

        unsafe {
            let SurfaceInfo {
                surface,
                surface_loader,
                format,
            } = surface_info;

            // Query surface capabilities
            let capabilities = surface_loader
                .get_physical_device_surface_capabilities(vulkan_context.physical_device(), surface)
                .map_err(|e| EngineError::Surface(format!("get capabilities: {:?}", e)))?;

            let mut image_count = capabilities.min_image_count + 1;
            if capabilities.max_image_count > 0 && image_count > capabilities.max_image_count {
                image_count = capabilities.max_image_count;
            }

            let extent = if capabilities.current_extent.width == u32::MAX {
                vk::Extent2D {
                    width: window_width,
                    height: window_height,
                }
            } else {
                capabilities.current_extent
            };

            let pre_transform = if capabilities
                .supported_transforms
                .contains(vk::SurfaceTransformFlagsKHR::IDENTITY)
            {
                vk::SurfaceTransformFlagsKHR::IDENTITY
            } else {
                capabilities.current_transform
            };

            let present_modes = surface_loader
                .get_physical_device_surface_present_modes(
                    vulkan_context.physical_device(),
                    surface,
                )
                .map_err(|e| EngineError::Surface(format!("get present modes: {:?}", e)))?;

            let present_mode = present_modes
                .iter()
                .copied()
                .find(|&m| m == vk::PresentModeKHR::MAILBOX)
                .unwrap_or(vk::PresentModeKHR::FIFO);

            // Create swapchain
            let loader = swapchain::Device::new(&vulkan_context.instance.instance, &device.device);

            let create_info = vk::SwapchainCreateInfoKHR::default()
                .surface(surface)
                .min_image_count(image_count)
                .image_format(format.format)
                .image_color_space(format.color_space)
                .image_extent(extent)
                .image_array_layers(1)
                .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT)
                .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
                .pre_transform(pre_transform)
                .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
                .present_mode(present_mode)
                .clipped(true);

            let handle = loader
                .create_swapchain(&create_info, None)
                .map_err(|e| EngineError::Swapchain(format!("creation: {:?}", e)))?;

            // Get swapchain images and create views
            let images = loader
                .get_swapchain_images(handle)
                .map_err(|e| EngineError::Swapchain(format!("get images: {:?}", e)))?;

            let image_views = images
                .iter()
                .map(|&image| {
                    let view_info = vk::ImageViewCreateInfo::default()
                        .image(image)
                        .view_type(vk::ImageViewType::TYPE_2D)
                        .format(format.format)
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
                        });

                    device.device.create_image_view(&view_info, None)
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| EngineError::Swapchain(format!("create image views: {:?}", e)))?;

            // Create depth buffer
            let depth_buffer = DepthBuffer::new(&vulkan_context, extent, vk::Format::D16_UNORM)?;

            // Create framebuffers
            let framebuffers = image_views
                .iter()
                .map(|&view| {
                    let attachments = [view, depth_buffer.view];
                    let fb_info = vk::FramebufferCreateInfo::default()
                        .render_pass(renderpass)
                        .attachments(&attachments)
                        .width(extent.width)
                        .height(extent.height)
                        .layers(1);

                    device.device.create_framebuffer(&fb_info, None)
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| EngineError::Framebuffer(format!("creation: {:?}", e)))?;

            // Create sync objects
            let sync = FrameSync::new(Arc::clone(&device))?;

            // Create command buffer
            let draw_command_buffer = vulkan_context
                .command_buffer_manager
                .create_primary_buffer()?;

            Ok(Self {
                handle,
                loader,
                surface_loader,
                surface,
                extent,
                image_views,
                framebuffers,
                depth_buffer,
                sync,
                draw_command_buffer,
                device,
            })
        }
    }

    /// Acquire the next swapchain image for rendering.
    pub fn acquire_next_image(&self) -> EngineResult<u32> {
        unsafe {
            let (index, _suboptimal) = self
                .loader
                .acquire_next_image(
                    self.handle,
                    u64::MAX,
                    self.sync.present_complete,
                    vk::Fence::null(),
                )
                .map_err(|e| EngineError::Swapchain(format!("acquire image: {:?}", e)))?;

            Ok(index)
        }
    }

    /// Present the rendered frame.
    pub fn present(&self, image_index: u32, queue: vk::Queue) -> EngineResult<()> {
        let wait_semaphores = [self.sync.rendering_complete];
        let swapchains = [self.handle];
        let indices = [image_index];

        let present_info = vk::PresentInfoKHR::default()
            .wait_semaphores(&wait_semaphores)
            .swapchains(&swapchains)
            .image_indices(&indices);

        unsafe {
            self.loader
                .queue_present(queue, &present_info)
                .map_err(|e| EngineError::Swapchain(format!("present: {:?}", e)))?;
        }

        Ok(())
    }
}

impl Drop for Swapchain {
    fn drop(&mut self) {
        unsafe {
            for &framebuffer in &self.framebuffers {
                self.device.device.destroy_framebuffer(framebuffer, None);
            }
            for &view in &self.image_views {
                self.device.device.destroy_image_view(view, None);
            }
            self.loader.destroy_swapchain(self.handle, None);
            self.surface_loader.destroy_surface(self.surface, None);
        }
    }
}
