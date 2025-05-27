use ash::{vk, Entry, Instance};
use std::error::Error;

pub struct ManagedInstance {
    pub instance: Instance,
}

impl ManagedInstance {
    pub fn new(
        entry: &Entry,
        create_info: &vk::InstanceCreateInfo,
    ) -> Result<Self, Box<dyn Error>> {
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
