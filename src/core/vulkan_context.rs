use crate::core::debug_manager::DebugManager;
use ash::{vk, Device, Entry};
use std::{ffi::CStr, os::raw::c_char, sync::Arc};
use winit::raw_window_handle::HasDisplayHandle;

use super::command_buffer::CommandBufferManager;
use super::device::ManagedDevice;
use super::error::{EngineError, EngineResult};
use super::instance::ManagedInstance;

/// The Khronos validation layer, loaded only when [`validation_requested`].
const VALIDATION_LAYER: &CStr = c"VK_LAYER_KHRONOS_validation";

/// Environment variable that turns the validation layer on (`1`) or off (`0`)
/// in any build.
const VALIDATION_ENV: &str = "VP_VALIDATION";

/// Whether to load the validation layer. [`VALIDATION_ENV`] decides when it
/// is set; otherwise debug builds validate and release builds do not.
///
/// The layer checks every command as it is recorded, which costs the CPU a
/// few microseconds a draw: about 3.4 ms a frame on Stack Yard's ~1,900
/// draws. A release build measures what the game costs without it.
fn validation_requested() -> bool {
    match std::env::var(VALIDATION_ENV) {
        Ok(value) => value != "0",
        Err(_) => cfg!(debug_assertions),
    }
}

/// Manages Vulkan instance, device, and debug utilities
pub struct VulkanContext {
    pub entry: Entry,
    pub instance: Arc<ManagedInstance>,
    _debug_manager: DebugManager,
    pub device: Arc<ManagedDevice>,
    pub command_buffer_manager: CommandBufferManager,
}

impl VulkanContext {
    /// Create a context able to present to the given window's display.
    pub fn new(window: &impl HasDisplayHandle) -> EngineResult<Self> {
        let display_handle = window
            .display_handle()
            .map_err(|e| EngineError::Window(format!("failed to get display handle: {:?}", e)))?;
        let surface_extensions = ash_window::enumerate_required_extensions(display_handle.as_raw())
            .map_err(|e| {
                EngineError::InstanceCreation(format!(
                    "failed to enumerate required extensions: {:?}",
                    e
                ))
            })?
            .to_vec();

        Self::create(surface_extensions)
    }

    /// Create a context with no connection to a windowing system.
    ///
    /// Everything the renderer needs works without a surface — the surface
    /// extensions exist only so a swapchain can be created later, and an
    /// offscreen output never creates one. This is what lets the visual bench
    /// run from a shell with no display attached.
    pub fn headless() -> EngineResult<Self> {
        // `ManagedDevice` always enables VK_KHR_swapchain, which is only valid
        // if VK_KHR_surface is enabled on the instance. Nothing here creates a
        // surface, but the extension has to be present for the device to be
        // built the same way it is for a window.
        Self::create(vec![ash::khr::surface::NAME.as_ptr()])
    }

    fn create(mut extension_names: Vec<*const c_char>) -> EngineResult<Self> {
        let entry = Entry::linked();
        let app_name = c"VulkanTriangle";
        let validate = validation_requested();
        log::info!(
            "Vulkan validation layer {}",
            if validate { "on" } else { "off" }
        );
        let layers_names_raw: Vec<*const c_char> = if validate {
            vec![VALIDATION_LAYER.as_ptr()]
        } else {
            Vec::new()
        };
        extension_names.push(ash::ext::debug_utils::NAME.as_ptr());
        #[cfg(any(target_os = "macos", target_os = "ios"))]
        {
            extension_names.push(ash::khr::portability_enumeration::NAME.as_ptr());
            extension_names.push(ash::khr::get_physical_device_properties2::NAME.as_ptr());
        }
        let appinfo = vk::ApplicationInfo::default()
            .application_name(app_name)
            .application_version(0)
            .engine_name(app_name)
            .engine_version(0)
            .api_version(vk::make_api_version(0, 1, 0, 0));
        let create_flags = if cfg!(any(target_os = "macos", target_os = "ios")) {
            vk::InstanceCreateFlags::ENUMERATE_PORTABILITY_KHR
        } else {
            vk::InstanceCreateFlags::default()
        };
        let create_info = vk::InstanceCreateInfo::default()
            .application_info(&appinfo)
            .enabled_layer_names(&layers_names_raw)
            .enabled_extension_names(&extension_names)
            .flags(create_flags);
        let instance = Arc::new(ManagedInstance::new(&entry, &create_info)?);
        let debug_manager = DebugManager::new(&entry, Arc::clone(&instance))?;

        let device = Arc::new(ManagedDevice::new(Arc::clone(&instance))?);
        let command_buffer_manager = CommandBufferManager::new(Arc::clone(&device))?;

        Ok(Self {
            entry,
            instance,
            _debug_manager: debug_manager,
            device,
            command_buffer_manager,
        })
    }

    pub fn device(&self) -> &Device {
        &self.device.device
    }

    pub fn physical_device(&self) -> vk::PhysicalDevice {
        self.device.physical_device
    }
}

pub fn find_memorytype_index(
    memory_req: &vk::MemoryRequirements,
    memory_prop: &vk::PhysicalDeviceMemoryProperties,
    flags: vk::MemoryPropertyFlags,
) -> Option<u32> {
    memory_prop.memory_types[..memory_prop.memory_type_count as _]
        .iter()
        .enumerate()
        .find(|(index, memory_type)| {
            (1 << index) & memory_req.memory_type_bits != 0
                && memory_type.property_flags & flags == flags
        })
        .map(|(index, _memory_type)| index as _)
}
