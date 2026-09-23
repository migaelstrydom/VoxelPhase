//! Core error types for the Voxel Phase engine.
//!
//! This module provides a unified error type that enables proper error handling
//! and recovery throughout the engine, replacing `Box<dyn Error>` with typed errors.

use ash::vk;
use std::fmt;

/// Primary error type for all engine operations.
#[derive(Debug)]
pub enum EngineError {
    /// Vulkan instance creation failed
    InstanceCreation(String),

    /// Physical device enumeration or selection failed
    PhysicalDevice(String),

    /// Logical device creation failed
    DeviceCreation(String),

    /// Surface creation or query failed
    Surface(String),

    /// Swapchain creation or acquisition failed
    Swapchain(String),

    /// The swapchain no longer matches the surface and must be rebuilt.
    ///
    /// Separate from `Swapchain` because it is an expected outcome of a resize
    /// or display change, not a failure: the frame is skipped and the next one
    /// proceeds. Folding it into the generic variant is what lets a routine
    /// window event look like a fatal render error.
    SwapchainOutOfDate,

    /// Buffer creation or operation failed
    Buffer {
        operation: BufferOperation,
        size: u64,
        reason: String,
    },

    /// Image creation or operation failed
    Image {
        operation: ImageOperation,
        width: u32,
        height: u32,
        reason: String,
    },

    /// Shader compilation or module creation failed
    Shader { stage: ShaderStage, reason: String },

    /// Graphics or compute pipeline creation failed
    Pipeline(String),

    /// Render pass creation failed
    RenderPass(String),

    /// Framebuffer creation failed
    Framebuffer(String),

    /// Command buffer recording or submission failed
    CommandBuffer(String),

    /// Descriptor set or pool operation failed
    Descriptor(String),

    /// Synchronization primitive (fence, semaphore) operation failed
    Synchronization(String),

    /// Query pool creation or readback failed
    Query(String),

    /// Texture loading or creation failed
    Texture {
        path: Option<String>,
        reason: String,
    },

    /// Mesh or geometry loading failed
    Mesh {
        path: Option<String>,
        reason: String,
    },

    /// Invalid state or precondition
    InvalidState(String),

    /// I/O error (file operations)
    Io { path: String, reason: String },

    /// Window or display error
    Window(String),

    /// Vulkan API returned an error
    Vulkan(vk::Result),
}

/// Buffer operations for error context
#[derive(Debug, Clone, Copy)]
pub enum BufferOperation {
    Create,
    Map,
    Bind,
}

/// Image operations for error context
#[derive(Debug, Clone, Copy)]
pub enum ImageOperation {
    Create,
    AllocateMemory,
    Bind,
    CreateView,
    TransitionLayout,
    GenerateMipmaps,
}

/// Shader stages for error context
#[derive(Debug, Clone, Copy)]
pub enum ShaderStage {
    Vertex,
    Fragment,
    Compute,
}

impl fmt::Display for EngineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InstanceCreation(msg) => write!(f, "Vulkan instance creation failed: {}", msg),
            Self::PhysicalDevice(msg) => write!(f, "Physical device error: {}", msg),
            Self::DeviceCreation(msg) => write!(f, "Device creation failed: {}", msg),
            Self::Surface(msg) => write!(f, "Surface error: {}", msg),
            Self::Swapchain(msg) => write!(f, "Swapchain error: {}", msg),
            Self::SwapchainOutOfDate => write!(f, "Swapchain out of date"),
            Self::Buffer {
                operation,
                size,
                reason,
            } => {
                write!(
                    f,
                    "Buffer {:?} failed ({} bytes): {}",
                    operation, size, reason
                )
            }
            Self::Image {
                operation,
                width,
                height,
                reason,
            } => {
                write!(
                    f,
                    "Image {:?} failed ({}x{}): {}",
                    operation, width, height, reason
                )
            }
            Self::Shader { stage, reason } => {
                write!(f, "{:?} shader error: {}", stage, reason)
            }
            Self::Pipeline(msg) => write!(f, "Pipeline creation failed: {}", msg),
            Self::RenderPass(msg) => write!(f, "Render pass error: {}", msg),
            Self::Framebuffer(msg) => write!(f, "Framebuffer error: {}", msg),
            Self::CommandBuffer(msg) => write!(f, "Command buffer error: {}", msg),
            Self::Descriptor(msg) => write!(f, "Descriptor error: {}", msg),
            Self::Synchronization(msg) => write!(f, "Synchronization error: {}", msg),
            Self::Query(msg) => write!(f, "Query error: {}", msg),
            Self::Texture { path, reason } => match path {
                Some(p) => write!(f, "Texture error ({}): {}", p, reason),
                None => write!(f, "Texture error: {}", reason),
            },
            Self::Mesh { path, reason } => match path {
                Some(p) => write!(f, "Mesh error ({}): {}", p, reason),
                None => write!(f, "Mesh error: {}", reason),
            },
            Self::InvalidState(msg) => write!(f, "Invalid state: {}", msg),
            Self::Io { path, reason } => write!(f, "I/O error ({}): {}", path, reason),
            Self::Window(msg) => write!(f, "Window error: {}", msg),
            Self::Vulkan(result) => write!(f, "Vulkan error: {:?}", result),
        }
    }
}

impl std::error::Error for EngineError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}

// Conversion from Vulkan result
impl From<vk::Result> for EngineError {
    fn from(result: vk::Result) -> Self {
        Self::Vulkan(result)
    }
}

// Conversion from std::io::Error
impl From<std::io::Error> for EngineError {
    fn from(err: std::io::Error) -> Self {
        Self::Io {
            path: String::new(),
            reason: err.to_string(),
        }
    }
}

// Conversion from image crate errors
impl From<image::ImageError> for EngineError {
    fn from(err: image::ImageError) -> Self {
        Self::Texture {
            path: None,
            reason: err.to_string(),
        }
    }
}

/// Result type alias using EngineError
pub type EngineResult<T> = Result<T, EngineError>;

/// Extension trait for Vulkan Results
pub trait VkResultExt<T> {
    /// Convert to EngineError with image context
    fn image_context(self, op: ImageOperation, width: u32, height: u32) -> EngineResult<T>;

    /// Convert to EngineError with descriptor context
    fn descriptor_context(self, msg: &str) -> EngineResult<T>;

    /// Convert to EngineError with command buffer context
    fn command_context(self, msg: &str) -> EngineResult<T>;

    /// Convert to EngineError with synchronization context
    fn sync_context(self, msg: &str) -> EngineResult<T>;
}

impl<T> VkResultExt<T> for Result<T, vk::Result> {
    fn image_context(self, op: ImageOperation, width: u32, height: u32) -> EngineResult<T> {
        self.map_err(|e| EngineError::Image {
            operation: op,
            width,
            height,
            reason: format!("{:?}", e),
        })
    }

    fn descriptor_context(self, msg: &str) -> EngineResult<T> {
        self.map_err(|e| EngineError::Descriptor(format!("{}: {:?}", msg, e)))
    }

    fn command_context(self, msg: &str) -> EngineResult<T> {
        self.map_err(|e| EngineError::CommandBuffer(format!("{}: {:?}", msg, e)))
    }

    fn sync_context(self, msg: &str) -> EngineResult<T> {
        self.map_err(|e| EngineError::Synchronization(format!("{}: {:?}", msg, e)))
    }
}
