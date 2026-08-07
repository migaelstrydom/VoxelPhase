//! Rendering module: Vulkan-based graphics pipeline and resources.

pub mod camera;
pub mod colour;
pub mod compute;
pub mod debug_render;
pub mod deletion_queue;
pub mod descriptors;
pub mod frame;
pub mod material;
pub mod overlay;
pub mod pipeline;
pub mod post;
pub mod renderer;
pub mod shaders;
pub mod sky;
pub mod target;
pub mod texture;
pub mod vertex;
pub mod visual_bench;
pub mod water;

// Re-export commonly used types
pub use colour::Colour;
