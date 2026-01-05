use ash::{vk, Entry, Instance};

use super::error::{EngineError, EngineResult};

pub struct ManagedInstance {
    pub instance: Instance,
}

impl ManagedInstance {
    pub fn new(entry: &Entry, create_info: &vk::InstanceCreateInfo) -> EngineResult<Self> {
        unsafe {
            let instance = entry
                .create_instance(create_info, None)
                .map_err(|e| EngineError::InstanceCreation(format!("{:?}", e)))?;
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
