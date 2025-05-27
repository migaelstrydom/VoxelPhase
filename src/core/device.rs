use std::{error::Error, sync::Arc};

use ash::{vk, Device};

use super::instance::ManagedInstance;

pub struct ManagedDevice {
    pub device: Device,
    pub physical_device: vk::PhysicalDevice,
    pub queue_family_index: u32,
    pub queue_family_indices: QueueFamilyIndices,
    pub device_memory_properties: vk::PhysicalDeviceMemoryProperties,
    _instance: Arc<ManagedInstance>, // Keep instance alive
}

impl ManagedDevice {
    pub fn new(instance: Arc<ManagedInstance>) -> Result<Self, Box<dyn Error>> {
        unsafe {
            let pdevices = instance
                .instance
                .enumerate_physical_devices()
                .expect("Physical device error");
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
                                Some((*pdevice, index as u32))
                            } else {
                                None
                            }
                        })
                })
                .expect("Couldn't find suitable device.");

            let queue_family_indices =
                OptionalQueueFamilyIndices::new(&instance.instance, physical_device)
                    .get_indices()
                    .expect("Couldn't find suitable queue family indices.");

            let features = vk::PhysicalDeviceFeatures {
                shader_clip_distance: 1,
                ..Default::default()
            };
            let device_extension_names_raw = [
                ash::khr::swapchain::NAME.as_ptr(),
                #[cfg(any(target_os = "macos", target_os = "ios"))]
                ash::khr::portability_subset::NAME.as_ptr(),
            ];

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
                queue_family_indices,
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

pub struct QueueFamilyIndices {
    pub graphics: u32, // Index of family supporting graphics
    pub compute: u32,  // Index of family supporting compute
    pub transfer: u32, // Index of family supporting transfer
}

struct OptionalQueueFamilyIndices {
    pub graphics: Option<u32>, // Index of family supporting graphics
    pub compute: Option<u32>,  // Index of family supporting compute
    pub transfer: Option<u32>, // Index of family supporting transfer
}

impl OptionalQueueFamilyIndices {
    pub fn new(instance: &ash::Instance, physical_device: vk::PhysicalDevice) -> Self {
        let queue_families =
            unsafe { instance.get_physical_device_queue_family_properties(physical_device) };

        let mut indices = OptionalQueueFamilyIndices {
            graphics: None,
            compute: None,
            transfer: None,
        };

        for (index, queue_family) in queue_families.iter().enumerate() {
            let index = index as u32;

            // Check for graphics support
            if queue_family.queue_flags.contains(vk::QueueFlags::GRAPHICS)
                && indices.graphics.is_none()
            {
                indices.graphics = Some(index);
            }

            // Check for compute support
            if queue_family.queue_flags.contains(vk::QueueFlags::COMPUTE)
                && indices.compute.is_none()
            {
                indices.compute = Some(index);
            }

            // Check for transfer support
            if queue_family.queue_flags.contains(vk::QueueFlags::TRANSFER)
                && indices.transfer.is_none()
            {
                indices.transfer = Some(index);
            }
        }

        indices
    }

    pub fn get_indices(&self) -> Option<QueueFamilyIndices> {
        if self.graphics.is_some() && self.compute.is_some() && self.transfer.is_some() {
            Some(QueueFamilyIndices {
                graphics: self.graphics.unwrap(),
                compute: self.compute.unwrap(),
                transfer: self.transfer.unwrap(),
            })
        } else {
            None
        }
    }
}
