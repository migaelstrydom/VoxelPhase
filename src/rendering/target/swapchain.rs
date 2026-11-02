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
///
/// The chain is rebuilt at a new size by [`FrameOutput::resize`]; the surface
/// it presents to lives as long as this does.
pub struct SwapchainOutput {
    /// The current chain, its images and their semaphores.
    chain: Chain,
    loader: swapchain::Device,
    surface_loader: surface::Instance,
    surface: vk::SurfaceKHR,
    surface_format: vk::SurfaceFormatKHR,
    physical_device: vk::PhysicalDevice,
    device: Arc<ManagedDevice>,
}

/// One swapchain and what is made per image of it.
struct Chain {
    handle: vk::SwapchainKHR,
    extent: vk::Extent2D,
    images: Vec<vk::Image>,
    image_views: Vec<vk::ImageView>,
    /// One per image, signalled by the submit that draws into it and waited on
    /// by its present. Per image rather than per frame in flight: a semaphore
    /// handed to a present may not be signalled again until that present has
    /// consumed it, and the only proof of that is the image being acquired
    /// again.
    rendering_complete: Vec<vk::Semaphore>,
}

impl SwapchainOutput {
    pub fn new(
        vulkan_context: &VulkanContext,
        surface_info: SurfaceInfo,
        window_width: u32,
        window_height: u32,
    ) -> EngineResult<Self> {
        let SurfaceInfo {
            surface,
            surface_loader,
            format,
        } = surface_info;
        let device = Arc::clone(&vulkan_context.device);
        let loader = swapchain::Device::new(&vulkan_context.instance.instance, &device.device);
        let mut output = Self {
            chain: Chain::empty(),
            loader,
            surface_loader,
            surface,
            surface_format: format,
            physical_device: vulkan_context.physical_device(),
            device,
        };
        output.chain = output.create_chain(window_width, window_height)?;
        Ok(output)
    }

    /// Build a chain for the surface as it is now, sized to the window where
    /// the surface leaves the size to the swapchain.
    fn create_chain(&self, window_width: u32, window_height: u32) -> EngineResult<Chain> {
        let device = &self.device;
        let surface = self.surface;
        let format = self.surface_format;

        unsafe {
            let capabilities = self
                .surface_loader
                .get_physical_device_surface_capabilities(self.physical_device, surface)
                .map_err(|e| EngineError::Surface(format!("get capabilities: {:?}", e)))?;

            let mut image_count = capabilities.min_image_count + 1;
            if capabilities.max_image_count > 0 && image_count > capabilities.max_image_count {
                image_count = capabilities.max_image_count;
            }

            let extent = if capabilities.current_extent.width == u32::MAX {
                vk::Extent2D {
                    width: window_width.clamp(
                        capabilities.min_image_extent.width,
                        capabilities.max_image_extent.width,
                    ),
                    height: window_height.clamp(
                        capabilities.min_image_extent.height,
                        capabilities.max_image_extent.height,
                    ),
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

            let present_modes = self
                .surface_loader
                .get_physical_device_surface_present_modes(self.physical_device, surface)
                .map_err(|e| EngineError::Surface(format!("get present modes: {:?}", e)))?;

            let present_mode = present_modes
                .iter()
                .copied()
                .find(|&m| m == vk::PresentModeKHR::MAILBOX)
                .unwrap_or(vk::PresentModeKHR::FIFO);

            let create_info = vk::SwapchainCreateInfoKHR::default()
                .surface(surface)
                .min_image_count(image_count)
                .image_format(format.format)
                .image_color_space(format.color_space)
                .image_extent(extent)
                .image_array_layers(1)
                .image_usage(
                    vk::ImageUsageFlags::COLOR_ATTACHMENT
                        | vk::ImageUsageFlags::TRANSFER_DST
                        | vk::ImageUsageFlags::TRANSFER_SRC,
                )
                .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
                .pre_transform(pre_transform)
                .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
                .present_mode(present_mode)
                .clipped(true);

            let handle = self
                .loader
                .create_swapchain(&create_info, None)
                .map_err(|e| EngineError::Swapchain(format!("creation: {:?}", e)))?;
            let mut chain = Chain {
                handle,
                extent,
                ..Chain::empty()
            };

            chain.images = self
                .loader
                .get_swapchain_images(handle)
                .map_err(|e| EngineError::Swapchain(format!("get images: {:?}", e)))?;

            for &image in &chain.images {
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
                match device.device.create_image_view(&view_info, None) {
                    Ok(view) => chain.image_views.push(view),
                    Err(e) => {
                        self.destroy_chain(&mut chain);
                        return Err(EngineError::Swapchain(format!(
                            "create image views: {:?}",
                            e
                        )));
                    }
                }
            }

            for _ in &chain.images {
                match device
                    .device
                    .create_semaphore(&vk::SemaphoreCreateInfo::default(), None)
                {
                    Ok(semaphore) => chain.rendering_complete.push(semaphore),
                    Err(e) => {
                        self.destroy_chain(&mut chain);
                        return Err(EngineError::Swapchain(format!(
                            "create semaphores: {:?}",
                            e
                        )));
                    }
                }
            }

            Ok(chain)
        }
    }

    /// Whether the surface's extent differs from the chain's. A surface that
    /// leaves its extent to the swapchain never has.
    fn surface_changed_size(&self) -> bool {
        let capabilities = unsafe {
            self.surface_loader
                .get_physical_device_surface_capabilities(self.physical_device, self.surface)
        };
        capabilities.is_ok_and(|capabilities| {
            let current = capabilities.current_extent;
            current.width != u32::MAX && current != self.chain.extent
        })
    }

    /// Destroy a chain's views, semaphores and swapchain. The device must be
    /// done with all of them.
    fn destroy_chain(&self, chain: &mut Chain) {
        unsafe {
            for view in chain.image_views.drain(..) {
                self.device.device.destroy_image_view(view, None);
            }
            for semaphore in chain.rendering_complete.drain(..) {
                self.device.device.destroy_semaphore(semaphore, None);
            }
            if chain.handle != vk::SwapchainKHR::null() {
                self.loader.destroy_swapchain(chain.handle, None);
                chain.handle = vk::SwapchainKHR::null();
            }
        }
    }
}

impl Chain {
    fn empty() -> Self {
        Self {
            handle: vk::SwapchainKHR::null(),
            extent: vk::Extent2D::default(),
            images: Vec::new(),
            image_views: Vec::new(),
            rendering_complete: Vec::new(),
        }
    }
}

impl FrameOutput for SwapchainOutput {
    fn extent(&self) -> vk::Extent2D {
        self.chain.extent
    }

    fn format(&self) -> vk::Format {
        self.surface_format.format
    }

    fn images(&self) -> &[vk::Image] {
        &self.chain.images
    }

    fn image_views(&self) -> &[vk::ImageView] {
        &self.chain.image_views
    }

    /// Replace the chain with one sized to the surface as it is now. The
    /// caller must have waited for the device to finish with the old one.
    ///
    /// The old chain is destroyed first and the new one made from nothing,
    /// not handed the old as `old_swapchain`: on MoltenVK a chain made that
    /// way mid-run presented a single flat colour from then on.
    fn resize(&mut self, width: u32, height: u32) -> EngineResult<()> {
        let mut old = std::mem::replace(&mut self.chain, Chain::empty());
        self.destroy_chain(&mut old);
        let created = self.create_chain(width, height);
        self.chain = created?;
        Ok(())
    }

    fn final_layout(&self) -> vk::ImageLayout {
        vk::ImageLayout::PRESENT_SRC_KHR
    }

    fn acquire(&self, sync: &FrameSync) -> EngineResult<AcquiredFrame> {
        unsafe {
            // A suboptimal image still presents correctly, so it is drawn to
            // rather than discarded; its present reports it, and the chain is
            // rebuilt after. Only an outright out-of-date swapchain needs the
            // frame skipped.
            let (index, _suboptimal) = match self.loader.acquire_next_image(
                self.chain.handle,
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
                signal: Some(self.chain.rendering_complete[index as usize]),
            })
        }
    }

    fn release(&self, frame: &AcquiredFrame, queue: vk::Queue) -> EngineResult<()> {
        let wait_semaphores: Vec<vk::Semaphore> = frame.signal.into_iter().collect();
        let swapchains = [self.chain.handle];
        let indices = [frame.index];

        let present_info = vk::PresentInfoKHR::default()
            .wait_semaphores(&wait_semaphores)
            .swapchains(&swapchains)
            .image_indices(&indices);

        unsafe {
            match self.loader.queue_present(queue, &present_info) {
                Ok(false) => {}
                // Presented, but suboptimal. Rebuilt only if the surface has
                // changed size since the chain was made: MoltenVK reports
                // suboptimal for as long as the window is larger than its
                // screen allows, and a chain rebuilt at the surface's own
                // extent would be rebuilt every frame.
                Ok(true) => {
                    if self.surface_changed_size() {
                        return Err(EngineError::SwapchainOutOfDate);
                    }
                }
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
        let mut chain = std::mem::replace(&mut self.chain, Chain::empty());
        self.destroy_chain(&mut chain);
        unsafe {
            self.surface_loader.destroy_surface(self.surface, None);
        }
    }
}
