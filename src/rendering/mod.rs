//! Rendering module: Vulkan-based graphics pipeline and resources.

pub mod camera;
pub mod colour;
pub mod compute;
pub mod debug_render;
pub mod deletion_queue;
pub mod descriptors;
pub mod frame;
pub mod grain;
pub mod material;
pub mod overlay;
pub mod pattern;
pub mod physical_finish;
pub mod pipeline;
pub mod post;
pub mod profile;
pub mod renderer;
pub mod shaders;
pub mod shadow;
pub mod sky;
pub mod substance;
pub mod surface_buffer;
pub mod surface_source;
pub mod target;
pub mod texture;
pub mod transparency;
pub mod triplanar;
pub mod vertex;
pub mod visual_bench;
pub mod water;

// Re-export commonly used types
pub use colour::Colour;
