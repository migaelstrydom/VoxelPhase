//! Presentation to a window surface.

use std::sync::Arc;

use ash::{
    khr::{surface, swapchain},
    vk,
};
use winit::{
    raw_window_handle::{HasDisplayHandle, HasWindowHandle},
    window::Window,
};

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::target::output::{AcquiredFrame, FrameOutput};
use crate::rendering::target::sync::FrameSync;

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

/// Swapchain images plus the surface they present to.
///
/// Holds nothing but the presentation chain — the depth buffer, HDR scene
/// target and framebuffers that used to live here now belong to `FrameTargets`,
/// which is built the same way whether the output is this or an offscreen image.
pub struct SwapchainOutput {
    handle: vk::SwapchainKHR,
    loader: swapchain::Device,
    surface_loader: surface::Instance,
    surface: vk::SurfaceKHR,
    extent: vk::Extent2D,
    format: vk::Format,
    images: Vec<vk::Image>,
    image_views: Vec<vk::ImageView>,
    /// One per image, signalled by the submit that draws into it and waited on
    /// by its present. Per image rather than per frame in flight: a semaphore
    /// handed to a present may not be signalled again until that present has
    /// consumed it, and the only proof of that is the image being acquired
    /// again.
    rendering_complete: Vec<vk::Semaphore>,
    device: Arc<ManagedDevice>,
}

impl SwapchainOutput {
    pub fn new(
        vulkan_context: &VulkanContext,
        surface_info: SurfaceInfo,
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

            let loader = swapchain::Device::new(&vulkan_context.instance.instance, &device.device);

            let create_info = vk::SwapchainCreateInfoKHR::default()
                .surface(surface)
                .min_image_count(image_count)
                .image_format(format.format)
                .image_color_space(format.color_space)
                .image_extent(extent)
                .image_array_layers(1)
                .image_usage(
                    vk::ImageUsageFlags::COLOR_ATTACHMENT | vk::ImageUsageFlags::TRANSFER_DST,
                )
                .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
                .pre_transform(pre_transform)
                .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
                .present_mode(present_mode)
                .clipped(true);

            let handle = loader
                .create_swapchain(&create_info, None)
                .map_err(|e| EngineError::Swapchain(format!("creation: {:?}", e)))?;

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

            let rendering_complete = images
                .iter()
                .map(|_| {
                    device
                        .device
                        .create_semaphore(&vk::SemaphoreCreateInfo::default(), None)
                })
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| EngineError::Swapchain(format!("create semaphores: {:?}", e)))?;

            Ok(Self {
                handle,
                loader,
                surface_loader,
                surface,
                extent,
                format: format.format,
                images,
                image_views,
                rendering_complete,
                device,
            })
        }
    }
}

impl FrameOutput for SwapchainOutput {
    fn extent(&self) -> vk::Extent2D {
        self.extent
    }

    fn format(&self) -> vk::Format {
        self.format
    }

    fn images(&self) -> &[vk::Image] {
        &self.images
    }

    fn image_views(&self) -> &[vk::ImageView] {
        &self.image_views
    }

    fn final_layout(&self) -> vk::ImageLayout {
        vk::ImageLayout::PRESENT_SRC_KHR
    }

    fn acquire(&self, sync: &FrameSync) -> EngineResult<AcquiredFrame> {
        unsafe {
            // A suboptimal image still presents correctly, so it is drawn to
            // rather than discarded; only an outright out-of-date swapchain
            // needs the frame skipped.
            let (index, _suboptimal) = match self.loader.acquire_next_image(
                self.handle,
                u64::MAX,
                sync.present_complete,
                vk::Fence::null(),
            ) {
                Ok(pair) => pair,
                Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    return Err(EngineError::SwapchainOutOfDate)
                }
                Err(e) => return Err(EngineError::Swapchain(format!("acquire image: {:?}", e))),
            };

            Ok(AcquiredFrame {
                index,
                wait: Some(sync.present_complete),
                signal: Some(self.rendering_complete[index as usize]),
            })
        }
    }

    fn release(&self, frame: &AcquiredFrame, queue: vk::Queue) -> EngineResult<()> {
        let wait_semaphores: Vec<vk::Semaphore> = frame.signal.into_iter().collect();
        let swapchains = [self.handle];
        let indices = [frame.index];

        let present_info = vk::PresentInfoKHR::default()
            .wait_semaphores(&wait_semaphores)
            .swapchains(&swapchains)
            .image_indices(&indices);

        unsafe {
            match self.loader.queue_present(queue, &present_info) {
                Ok(_) => {}
                Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    return Err(EngineError::SwapchainOutOfDate)
                }
                Err(e) => return Err(EngineError::Swapchain(format!("present: {:?}", e))),
            }
        }

        Ok(())
    }
}

impl Drop for SwapchainOutput {
    fn drop(&mut self) {
        unsafe {
            for &view in &self.image_views {
                self.device.device.destroy_image_view(view, None);
            }
            for &semaphore in &self.rendering_complete {
                self.device.device.destroy_semaphore(semaphore, None);
            }
            self.loader.destroy_swapchain(self.handle, None);
            self.surface_loader.destroy_surface(self.surface, None);
        }
    }
}
