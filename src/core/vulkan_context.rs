use crate::core::debug_manager::DebugManager;
use ash::{vk, Device, Entry, Instance};
use std::{error::Error, os::raw::c_char, sync::Arc};
use winit::raw_window_handle::HasDisplayHandle;

// Import the moved items
use crate::core::command_buffer::CommandBufferManager;
use crate::core::instance::ManagedInstance;

use super::device::ManagedDevice;

/// Manages Vulkan instance, device, and debug utilities
pub struct VulkanContext {
    pub entry: Entry,
    pub instance: Arc<ManagedInstance>,
    _debug_manager: DebugManager,
    pub device: Arc<ManagedDevice>,
    pub command_buffer_manager: CommandBufferManager,
}

impl VulkanContext {
    pub fn new(window: &impl HasDisplayHandle) -> Result<Self, Box<dyn Error>> {
        let entry = Entry::linked();
        let app_name = c"VulkanTriangle";
        let layer_names = [c"VK_LAYER_KHRONOS_validation"];
        let layers_names_raw: Vec<*const c_char> = layer_names
            .iter()
            .map(|raw_name| raw_name.as_ptr())
            .collect();
        let mut extension_names =
            ash_window::enumerate_required_extensions(window.display_handle()?.as_raw())
                .unwrap()
                .to_vec();
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
