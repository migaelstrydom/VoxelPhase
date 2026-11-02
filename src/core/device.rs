use std::{collections::HashSet, sync::Arc};

use ash::{vk, Device};

use super::error::{EngineError, EngineResult};
use super::instance::ManagedInstance;

pub struct ManagedDevice {
    pub device: Device,
    pub physical_device: vk::PhysicalDevice,
    pub queue_family_indices: QueueFamilyIndices,
    pub device_memory_properties: vk::PhysicalDeviceMemoryProperties,
    _instance: Arc<ManagedInstance>, // Keep instance alive
}

impl ManagedDevice {
    pub fn new(instance: Arc<ManagedInstance>) -> EngineResult<Self> {
        unsafe {
            let pdevices = instance
                .instance
                .enumerate_physical_devices()
                .map_err(|e| EngineError::PhysicalDevice(format!("enumeration failed: {:?}", e)))?;

            // The most capable device that has a queue family to run on:
            // a discrete GPU over an integrated one over anything else.
            let (physical_device, family) = pdevices
                .iter()
                .filter_map(|&pdevice| {
                    let families = instance
                        .instance
                        .get_physical_device_queue_family_properties(pdevice);
                    pick_queue_family(&families).map(|family| (pdevice, family))
                })
                .max_by_key(|&(pdevice, _)| {
                    let kind = instance
                        .instance
                        .get_physical_device_properties(pdevice)
                        .device_type;
                    device_rank(kind)
                })
                .ok_or_else(|| {
                    EngineError::PhysicalDevice(
                        "no GPU with a queue family supporting graphics and compute".to_string(),
                    )
                })?;
            let queue_family_indices = QueueFamilyIndices {
                graphics: family,
                compute: family,
                transfer: family,
            };

            let features = vk::PhysicalDeviceFeatures {
                shader_clip_distance: 1,
                sampler_anisotropy: 1,
                fill_mode_non_solid: 1,
                // Reflection probes are sampled as one cube-map array.
                image_cube_array: 1,
                ..Default::default()
            };
            let supported = instance
                .instance
                .get_physical_device_features(physical_device);
            let missing = missing_features(&features, &supported);
            if !missing.is_empty() {
                return Err(EngineError::PhysicalDevice(format!(
                    "the GPU lacks required features: {}",
                    missing.join(", ")
                )));
            }
            let device_extension_names_raw = [
                ash::khr::swapchain::NAME.as_ptr(),
                #[cfg(any(target_os = "macos", target_os = "ios"))]
                ash::khr::portability_subset::NAME.as_ptr(),
            ];

            let priorities = [1.0];
            // Create VkDeviceQueueCreateInfo for each unique queue family index
            let mut unique_queue_family_indices = HashSet::new();
            unique_queue_family_indices.insert(queue_family_indices.graphics);
            unique_queue_family_indices.insert(queue_family_indices.compute);
            unique_queue_family_indices.insert(queue_family_indices.transfer);

            let queue_create_infos: Vec<vk::DeviceQueueCreateInfo> = unique_queue_family_indices
                .iter()
                .map(|&index| {
                    vk::DeviceQueueCreateInfo::default()
                        .queue_family_index(index)
                        .queue_priorities(&priorities)
                })
                .collect();

            let device_create_info = vk::DeviceCreateInfo::default()
                .queue_create_infos(&queue_create_infos) // Use the collected infos
                .enabled_extension_names(&device_extension_names_raw)
                .enabled_features(&features);
            let device = instance
                .instance
                .create_device(physical_device, &device_create_info, None)
                .map_err(|e| EngineError::DeviceCreation(format!("{:?}", e)))?;
            let device_memory_properties = instance
                .instance
                .get_physical_device_memory_properties(physical_device);
            Ok(Self {
                device,
                physical_device,
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

/// The queue families the engine runs on.
///
/// All three are the same family. The frame records its compute (the fire
/// simulation) into its graphics command buffer, and uploads blit their mip
/// chains, a graphics command, with no ownership transfers between families;
/// so a separate compute or transfer family would be wrong, not faster.
pub struct QueueFamilyIndices {
    pub graphics: u32,
    pub compute: u32,
    pub transfer: u32,
}

/// The first queue family that supports graphics and compute.
///
/// Transfer is not asked for: a family supporting graphics or compute
/// supports transfer commands whether or not it reports the bit, which the
/// specification leaves optional for them.
fn pick_queue_family(families: &[vk::QueueFamilyProperties]) -> Option<u32> {
    let needed = vk::QueueFlags::GRAPHICS | vk::QueueFlags::COMPUTE;
    families
        .iter()
        .position(|family| family.queue_count > 0 && family.queue_flags.contains(needed))
        .map(|index| index as u32)
}

/// How strongly a kind of device is preferred: higher is better.
fn device_rank(kind: vk::PhysicalDeviceType) -> u32 {
    match kind {
        vk::PhysicalDeviceType::DISCRETE_GPU => 4,
        vk::PhysicalDeviceType::INTEGRATED_GPU => 3,
        vk::PhysicalDeviceType::VIRTUAL_GPU => 2,
        vk::PhysicalDeviceType::CPU => 1,
        _ => 0,
    }
}

/// The names of the features `wanted` enables that `supported` lacks.
fn missing_features(
    wanted: &vk::PhysicalDeviceFeatures,
    supported: &vk::PhysicalDeviceFeatures,
) -> Vec<&'static str> {
    [
        (
            "shader_clip_distance",
            wanted.shader_clip_distance,
            supported.shader_clip_distance,
        ),
        (
            "sampler_anisotropy",
            wanted.sampler_anisotropy,
            supported.sampler_anisotropy,
        ),
        (
            "fill_mode_non_solid",
            wanted.fill_mode_non_solid,
            supported.fill_mode_non_solid,
        ),
        (
            "image_cube_array",
            wanted.image_cube_array,
            supported.image_cube_array,
        ),
    ]
    .into_iter()
    .filter(|&(_, wanted, supported)| wanted == vk::TRUE && supported != vk::TRUE)
    .map(|(name, _, _)| name)
    .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn family(flags: vk::QueueFlags) -> vk::QueueFamilyProperties {
        vk::QueueFamilyProperties {
            queue_flags: flags,
            queue_count: 1,
            ..Default::default()
        }
    }

    /// A dedicated transfer family listed first is passed over, and a
    /// graphics family that does not report transfer is taken.
    #[test]
    fn the_queue_family_is_one_that_draws_and_computes() {
        let families = [
            family(vk::QueueFlags::TRANSFER),
            family(vk::QueueFlags::COMPUTE),
            family(vk::QueueFlags::GRAPHICS | vk::QueueFlags::COMPUTE),
        ];
        assert_eq!(pick_queue_family(&families), Some(2));
        assert_eq!(pick_queue_family(&families[..2]), None);
    }

    #[test]
    fn a_discrete_gpu_is_preferred() {
        assert!(
            device_rank(vk::PhysicalDeviceType::DISCRETE_GPU)
                > device_rank(vk::PhysicalDeviceType::INTEGRATED_GPU)
        );
    }

    #[test]
    fn missing_features_are_named() {
        let wanted = vk::PhysicalDeviceFeatures {
            image_cube_array: vk::TRUE,
            sampler_anisotropy: vk::TRUE,
            ..Default::default()
        };
        let supported = vk::PhysicalDeviceFeatures {
            sampler_anisotropy: vk::TRUE,
            ..Default::default()
        };
        assert_eq!(
            missing_features(&wanted, &supported),
            vec!["image_cube_array"]
        );
    }
}
