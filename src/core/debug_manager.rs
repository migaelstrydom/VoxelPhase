use crate::utils::Destroy;
use ash::{Entry, Instance, ext::debug_utils, vk};
use std::{borrow::Cow, error::Error, ffi};

extern "system" fn vulkan_debug_callback(
    message_severity: vk::DebugUtilsMessageSeverityFlagsEXT,
    message_type: vk::DebugUtilsMessageTypeFlagsEXT,
    p_callback_data: *const vk::DebugUtilsMessengerCallbackDataEXT<'_>,
    _user_data: *mut std::os::raw::c_void,
) -> vk::Bool32 {
    let callback_data = unsafe { *p_callback_data };
    let message_id_number = callback_data.message_id_number;

    let message_id_name = if callback_data.p_message_id_name.is_null() {
        Cow::from("")
    } else {
        unsafe { ffi::CStr::from_ptr(callback_data.p_message_id_name).to_string_lossy() }
    };

    let message = if callback_data.p_message.is_null() {
        Cow::from("")
    } else {
        unsafe { ffi::CStr::from_ptr(callback_data.p_message).to_string_lossy() }
    };

    let message = format!(
        "{message_type:?} [{message_id_name} ({message_id_number})] : {message}",
        message_type = message_type,
        message_id_name = message_id_name,
        message_id_number = message_id_number,
        message = message,
    );

    match message_severity {
        vk::DebugUtilsMessageSeverityFlagsEXT::ERROR => log::error!("{}", message),
        vk::DebugUtilsMessageSeverityFlagsEXT::WARNING => log::warn!("{}", message),
        vk::DebugUtilsMessageSeverityFlagsEXT::INFO => log::info!("{}", message),
        _ => (),
    }

    vk::FALSE
}

pub struct DebugManager {
    // The debug manager must be dropped before the instance
    debug_utils_loader: debug_utils::Instance,
    debug_callback: vk::DebugUtilsMessengerEXT,
}

impl DebugManager {
    pub fn new(entry: &Entry, instance: &Instance) -> Result<Self, Box<dyn Error>> {
        let debug_info = vk::DebugUtilsMessengerCreateInfoEXT::default()
            .message_severity(
                vk::DebugUtilsMessageSeverityFlagsEXT::ERROR
                    | vk::DebugUtilsMessageSeverityFlagsEXT::WARNING
                    | vk::DebugUtilsMessageSeverityFlagsEXT::INFO,
            )
            .message_type(
                vk::DebugUtilsMessageTypeFlagsEXT::GENERAL
                    | vk::DebugUtilsMessageTypeFlagsEXT::VALIDATION
                    | vk::DebugUtilsMessageTypeFlagsEXT::PERFORMANCE,
            )
            .pfn_user_callback(Some(vulkan_debug_callback));

        let debug_utils_loader = debug_utils::Instance::new(entry, instance);
        let debug_callback = unsafe {
            debug_utils_loader
                .create_debug_utils_messenger(&debug_info, None)
                .map_err(|e| format!("Failed to create debug messenger: {}", e))?
        };

        Ok(Self {
            debug_utils_loader,
            debug_callback,
        })
    }
}

impl Destroy for DebugManager {
    fn destroy(&self) {
        unsafe {
            self.debug_utils_loader
                .destroy_debug_utils_messenger(self.debug_callback, None);
        }
    }
}
