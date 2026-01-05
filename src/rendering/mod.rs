// Rendering module will contain all rendering-related systems and components
pub mod camera;
pub mod descriptors;
pub mod frame;
pub mod pipeline;
pub mod renderer;
pub mod swapchain;
pub mod texture;
pub mod vertex;

// Re-export commonly used types
pub use descriptors::DescriptorManager;
pub use frame::{FrameData, ManagedBuffer, SceneUbo};
pub use pipeline::{GraphicsPipeline, GraphicsPipelineConfig};
pub use renderer::Renderer as NewRenderer;
pub use swapchain::Swapchain;
