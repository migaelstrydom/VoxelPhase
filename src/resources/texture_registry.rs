//! Reference-counted bookkeeping for loaded textures.
//!
//! The registry answers one question: how many live handles point at a texture,
//! and has the last one just gone away? It deliberately performs no Vulkan
//! calls - freeing a descriptor set needs the descriptor manager, which the
//! texture manager owns - so the release path hands the descriptor set back to
//! the caller instead.
//!
//! ```text
//!   TextureHandle ──retain/release──► TextureRegistry ──ReleaseOutcome──► TextureManager
//!                                     (counts handles)                    (frees GPU resources)
//! ```

use std::collections::HashMap;
use std::sync::Arc;

use ash::vk;

use crate::rendering::texture::ManagedTexture;

/// A registered texture and the state tracked alongside it.
struct TextureEntry<T> {
    /// The texture itself, shared with every live handle.
    texture: Arc<T>,
    /// Descriptor set bound to this texture, allocated on first use.
    descriptor_set: Option<vk::DescriptorSet>,
    /// Number of live handles referring to this texture.
    handle_count: usize,
}

/// Result of dropping one handle to a texture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReleaseOutcome {
    /// Other handles remain, so the texture stays alive.
    Retained,
    /// The last handle went away and the entry has been removed. The descriptor
    /// set it owned, if one was ever allocated, must now be freed by the caller.
    Released {
        descriptor_set: Option<vk::DescriptorSet>,
    },
    /// No entry with that id exists.
    Unknown,
}

/// Table of textures keyed by id, reference counted by live handle.
///
/// The generic parameter exists so the counting logic can be exercised without
/// a Vulkan device; production code uses the default `ManagedTexture`.
pub struct TextureRegistry<T = ManagedTexture> {
    entries: HashMap<u64, TextureEntry<T>>,
    next_id: u64,
}

impl<T> TextureRegistry<T> {
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
            next_id: 0,
        }
    }

    /// Register a texture with a single live handle and return its id.
    pub fn insert(&mut self, texture: Arc<T>) -> u64 {
        let id = self.next_id;
        self.next_id += 1;

        self.entries.insert(
            id,
            TextureEntry {
                texture,
                descriptor_set: None,
                handle_count: 1,
            },
        );

        id
    }

    /// Record that another handle to `id` now exists.
    pub fn retain(&mut self, id: u64) {
        if let Some(entry) = self.entries.get_mut(&id) {
            entry.handle_count += 1;
        }
    }

    /// Drop one handle to `id`, removing the entry if it was the last one.
    #[must_use]
    pub fn release(&mut self, id: u64) -> ReleaseOutcome {
        let Some(entry) = self.entries.get_mut(&id) else {
            return ReleaseOutcome::Unknown;
        };

        entry.handle_count -= 1;
        if entry.handle_count > 0 {
            return ReleaseOutcome::Retained;
        }

        let entry = self.entries.remove(&id).expect("entry looked up above");
        ReleaseOutcome::Released {
            descriptor_set: entry.descriptor_set,
        }
    }

    /// The texture registered under `id`, if it is still alive.
    pub fn texture(&self, id: u64) -> Option<&Arc<T>> {
        self.entries.get(&id).map(|entry| &entry.texture)
    }

    /// The descriptor set bound to `id`, if one has been allocated.
    pub fn descriptor_set(&self, id: u64) -> Option<vk::DescriptorSet> {
        self.entries.get(&id).and_then(|entry| entry.descriptor_set)
    }

    /// Bind a descriptor set to `id`, which the registry then owns.
    pub fn set_descriptor_set(&mut self, id: u64, descriptor_set: vk::DescriptorSet) {
        if let Some(entry) = self.entries.get_mut(&id) {
            entry.descriptor_set = Some(descriptor_set);
        }
    }

    /// Number of live handles to `id`, or zero if it is not registered.
    pub fn handle_count(&self, id: u64) -> usize {
        self.entries.get(&id).map_or(0, |entry| entry.handle_count)
    }

    /// Number of textures currently alive.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

impl<T> Default for TextureRegistry<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use ash::vk::Handle;

    use super::*;

    fn registry() -> TextureRegistry<u32> {
        TextureRegistry::new()
    }

    #[test]
    fn insert_starts_with_one_handle() {
        let mut registry = registry();
        let id = registry.insert(Arc::new(7));

        assert_eq!(registry.handle_count(id), 1);
        assert_eq!(registry.len(), 1);
    }

    #[test]
    fn ids_are_unique() {
        let mut registry = registry();
        let first = registry.insert(Arc::new(1));
        let second = registry.insert(Arc::new(2));

        assert_ne!(first, second);
    }

    #[test]
    fn releasing_a_clone_keeps_the_texture_alive() {
        let mut registry = registry();
        let id = registry.insert(Arc::new(7));
        registry.retain(id);

        assert_eq!(registry.release(id), ReleaseOutcome::Retained);
        assert_eq!(registry.handle_count(id), 1);
        assert_eq!(registry.texture(id).map(|t| **t), Some(7));
    }

    #[test]
    fn releasing_the_last_handle_removes_the_entry() {
        let mut registry = registry();
        let id = registry.insert(Arc::new(7));
        registry.retain(id);

        let _ = registry.release(id);

        assert_eq!(
            registry.release(id),
            ReleaseOutcome::Released {
                descriptor_set: None
            }
        );
        assert_eq!(registry.handle_count(id), 0);
        assert!(registry.texture(id).is_none());
        assert!(registry.is_empty());
    }

    #[test]
    fn descriptor_set_is_returned_once_on_final_release() {
        let mut registry = registry();
        let id = registry.insert(Arc::new(7));
        registry.retain(id);

        let set = vk::DescriptorSet::from_raw(42);
        registry.set_descriptor_set(id, set);

        assert_eq!(registry.release(id), ReleaseOutcome::Retained);
        assert_eq!(registry.descriptor_set(id), Some(set));

        assert_eq!(
            registry.release(id),
            ReleaseOutcome::Released {
                descriptor_set: Some(set)
            }
        );
        assert_eq!(registry.release(id), ReleaseOutcome::Unknown);
    }

    #[test]
    fn releasing_an_unknown_id_is_inert() {
        let mut registry = registry();

        assert_eq!(registry.release(99), ReleaseOutcome::Unknown);
        assert!(registry.is_empty());
    }
}
