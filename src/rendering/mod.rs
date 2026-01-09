//! Rendering module: Vulkan-based graphics pipeline and resources.

pub mod camera;
pub mod colour;
pub mod deletion_queue;
pub mod descriptors;
pub mod frame;
pub mod material;
pub mod pipeline;
pub mod renderer;
pub mod swapchain;
pub mod texture;
pub mod vertex;

// Re-export commonly used types
pub use colour::Colour;
