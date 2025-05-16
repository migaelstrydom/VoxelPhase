use crate::core::debug_manager::DebugManager;
use ash::{vk, Device, Entry, Instance};
use std::{os::raw::c_char, sync::Arc};
use winit::raw_window_handle::HasDisplayHandle;

pub struct ManagedInstance {
    pub instance: Instance,
}

impl ManagedInstance {
    pub fn new(
        entry: &Entry,
        create_info: &vk::InstanceCreateInfo,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        unsafe {
            let instance = entry.create_instance(create_info, None)?;
            Ok(Self { instance })
        }
    }
}

impl Drop for ManagedInstance {
    fn drop(&mut self) {
        unsafe {
            self.instance.destroy_instance(None);
        }
    }
}

pub struct ManagedDevice {
    pub device: Device,
    pub physical_device: vk::PhysicalDevice,
    pub queue_family_index: u32,
    pub device_memory_properties: vk::PhysicalDeviceMemoryProperties,
    _instance: Arc<ManagedInstance>,
}

impl ManagedDevice {
    pub fn new(instance: Arc<ManagedInstance>) -> Result<Self, Box<dyn std::error::Error>> {
        unsafe {
            let pdevices = instance
                .instance
                .enumerate_physical_devices()
                .expect("Physical device error");
            // We need a surface to check for present support, but surface creation is outside this context for now.
            // So, for now, just pick a graphics queue family.
            let (physical_device, queue_family_index) = pdevices
                .iter()
                .find_map(|pdevice| {
                    instance
                        .instance
                        .get_physical_device_queue_family_properties(*pdevice)
                        .iter()
                        .enumerate()
                        .find_map(|(index, info)| {
                            if info.queue_flags.contains(vk::QueueFlags::GRAPHICS) {
                                Some((*pdevice, index))
                            } else {
                                None
                            }
                        })
                })
                .expect("Couldn't find suitable device.");

            let features = vk::PhysicalDeviceFeatures {
                shader_clip_distance: 1,
                ..Default::default()
            };
            let device_extension_names_raw = [
                ash::khr::swapchain::NAME.as_ptr(),
                #[cfg(any(target_os = "macos", target_os = "ios"))]
                ash::khr::portability_subset::NAME.as_ptr(),
            ];

            let queue_family_index = queue_family_index as u32;
            let priorities = [1.0];
            let queue_info = vk::DeviceQueueCreateInfo::default()
                .queue_family_index(queue_family_index)
                .queue_priorities(&priorities);
            let device_create_info = vk::DeviceCreateInfo::default()
                .queue_create_infos(std::slice::from_ref(&queue_info))
                .enabled_extension_names(&device_extension_names_raw)
                .enabled_features(&features);
            let device =
                instance
                    .instance
                    .create_device(physical_device, &device_create_info, None)?;
            let device_memory_properties = instance
                .instance
                .get_physical_device_memory_properties(physical_device);
            Ok(Self {
                device,
                physical_device,
                queue_family_index,
                device_memory_properties,
                _instance: instance,
            })
        }
    }
}

impl Drop for ManagedDevice {
    fn drop(&mut self) {
        unsafe {
            self.device.destroy_device(None);
        }
    }
}

/// Manages Vulkan instance, device, and debug utilities
pub struct VulkanContext {
    pub entry: Entry,
    pub instance: Arc<ManagedInstance>,
    _debug_manager: DebugManager, // Never used directly, only through the instance
    pub device: Arc<ManagedDevice>,
}

impl VulkanContext {
    /// Creates a new Vulkan context
    pub fn new(window: &impl HasDisplayHandle) -> Result<Self, Box<dyn std::error::Error>> {
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

        Ok(Self {
            entry,
            instance,
            _debug_manager: debug_manager,
            device,
        })
    }

    pub fn device(&self) -> &Device {
        &self.device.device
    }

    pub fn physical_device(&self) -> vk::PhysicalDevice {
        self.device.physical_device
    }
}

/// Helper function for submitting command buffers. Immediately waits for the fence before the command buffer
/// is executed. That way we can delay the waiting for the fences by 1 frame which is good for performance.
/// Make sure to create the fence in a signaled state on the first use.
#[allow(clippy::too_many_arguments)]
pub fn record_submit_commandbuffer<F: FnOnce(&Device, vk::CommandBuffer)>(
    device: &Device,
    command_buffer: vk::CommandBuffer,
    command_buffer_reuse_fence: vk::Fence,
    submit_queue: vk::Queue,
    wait_mask: &[vk::PipelineStageFlags],
    wait_semaphores: &[vk::Semaphore],
    signal_semaphores: &[vk::Semaphore],
    f: F,
) {
    unsafe {
        device
            .reset_command_buffer(
                command_buffer,
                vk::CommandBufferResetFlags::RELEASE_RESOURCES,
            )
            .expect("Reset command buffer failed.");

        let command_buffer_begin_info = vk::CommandBufferBeginInfo::default()
            .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);

        device
            .begin_command_buffer(command_buffer, &command_buffer_begin_info)
            .expect("Begin commandbuffer");
        f(device, command_buffer);
        device
            .end_command_buffer(command_buffer)
            .expect("End commandbuffer");

        let command_buffers = vec![command_buffer];

        let submit_info = vk::SubmitInfo::default()
            .wait_semaphores(wait_semaphores)
            .wait_dst_stage_mask(wait_mask)
            .command_buffers(&command_buffers)
            .signal_semaphores(signal_semaphores);

        device
            .queue_submit(submit_queue, &[submit_info], command_buffer_reuse_fence)
            .expect("queue submit failed.");
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
