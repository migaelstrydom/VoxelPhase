//! Centralized shader management.
//!
//! Provides compile-time embedded shaders and uniform loading functions
//! for all pipeline types in the engine.

use std::io::Cursor;

use ash::{util::read_spv, vk};

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ShaderStage};

/// Embedded shader bytecode.
mod bytecode {
    /// Main 3D pipeline shaders
    pub const MAIN_VERTEX: &[u8] = include_bytes!("../../shader/vert.spv");
    pub const MAIN_FRAGMENT: &[u8] = include_bytes!("../../shader/frag.spv");

    /// Overlay/UI pipeline shaders
    pub const OVERLAY_VERTEX: &[u8] = include_bytes!("../../shader/overlay.vert.spv");
    pub const OVERLAY_FRAGMENT: &[u8] = include_bytes!("../../shader/overlay.frag.spv");

    /// Particle system shaders
    pub const PARTICLE_VERTEX: &[u8] = include_bytes!("../../shader/particle.vert.spv");
    pub const PARTICLE_FRAGMENT: &[u8] = include_bytes!("../../shader/particle.frag.spv");

    /// Sky rendering shaders
    pub const SKY_VERTEX: &[u8] = include_bytes!("../../shader/sky.vert.spv");
    pub const SKY_FRAGMENT: &[u8] = include_bytes!("../../shader/sky.frag.spv");

    /// Water rendering shaders
    pub const WATER_VERTEX: &[u8] = include_bytes!("../../shader/water.vert.spv");
    pub const WATER_FRAGMENT: &[u8] = include_bytes!("../../shader/water.frag.spv");

    /// Post-processing shaders (HDR resolve, bloom)
    pub const POST_FULLSCREEN_VERTEX: &[u8] =
        include_bytes!("../../shader/post/fullscreen.vert.spv");
    pub const POST_BRIGHT_PASS: &[u8] = include_bytes!("../../shader/post/bright_pass.frag.spv");
    pub const POST_BLUR: &[u8] = include_bytes!("../../shader/post/blur.frag.spv");
    pub const POST_COMPOSITE: &[u8] = include_bytes!("../../shader/post/composite.frag.spv");
    pub const POST_BLOOM_OVERLAY: &[u8] =
        include_bytes!("../../shader/post/bloom_overlay.frag.spv");
}

/// Centralized shader loading and management.
///
/// All shader modules are created through this manager to ensure consistent
/// error handling and resource management.
pub struct ShaderManager;

impl ShaderManager {
    /// Load the main 3D vertex shader.
    pub fn load_main_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::MAIN_VERTEX, ShaderStage::Vertex)
    }

    /// Load the main 3D fragment shader.
    pub fn load_main_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::MAIN_FRAGMENT, ShaderStage::Fragment)
    }

    /// Load the shared fullscreen-triangle vertex shader used by every
    /// post-processing stage.
    pub fn load_post_fullscreen_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(
            device,
            bytecode::POST_FULLSCREEN_VERTEX,
            ShaderStage::Vertex,
        )
    }

    /// Load the bloom bright-pass fragment shader.
    pub fn load_post_bright_pass(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::POST_BRIGHT_PASS, ShaderStage::Fragment)
    }

    /// Load the separable blur fragment shader.
    pub fn load_post_blur(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::POST_BLUR, ShaderStage::Fragment)
    }

    /// Load the tonemap/bloom composite fragment shader.
    pub fn load_post_composite(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::POST_COMPOSITE, ShaderStage::Fragment)
    }

    /// Load the fragment shader that blends bloom over the finished frame.
    pub fn load_post_bloom_overlay(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::POST_BLOOM_OVERLAY, ShaderStage::Fragment)
    }

    /// Load the overlay vertex shader.
    pub fn load_overlay_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::OVERLAY_VERTEX, ShaderStage::Vertex)
    }

    /// Load the overlay fragment shader.
    pub fn load_overlay_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::OVERLAY_FRAGMENT, ShaderStage::Fragment)
    }

    /// Load the particle vertex shader.
    pub fn load_particle_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::PARTICLE_VERTEX, ShaderStage::Vertex)
    }

    /// Load the particle fragment shader.
    pub fn load_particle_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::PARTICLE_FRAGMENT, ShaderStage::Fragment)
    }

    /// Load the sky vertex shader.
    pub fn load_sky_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::SKY_VERTEX, ShaderStage::Vertex)
    }

    /// Load the sky fragment shader.
    pub fn load_sky_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::SKY_FRAGMENT, ShaderStage::Fragment)
    }

    /// Load the water vertex shader.
    pub fn load_water_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::WATER_VERTEX, ShaderStage::Vertex)
    }

    /// Load the water fragment shader.
    pub fn load_water_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::WATER_FRAGMENT, ShaderStage::Fragment)
    }

    /// Load a compute shader module from SPIR-V bytecode.
    ///
    /// Unlike vertex/fragment loaders, this takes raw bytecode because compute
    /// shaders are owned by their respective subsystems (e.g. fire), not
    /// centrally embedded in `ShaderManager`.
    pub fn load_compute(device: &ManagedDevice, bytecode: &[u8]) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode, ShaderStage::Compute)
    }

    /// Load a shader module from SPIR-V bytecode.
    fn load_shader(
        device: &ManagedDevice,
        bytecode: &[u8],
        stage: ShaderStage,
    ) -> EngineResult<vk::ShaderModule> {
        let mut cursor = Cursor::new(bytecode);

        let code = read_spv(&mut cursor).map_err(|e| EngineError::Shader {
            stage,
            reason: format!("failed to read SPIR-V: {:?}", e),
        })?;

        let create_info = vk::ShaderModuleCreateInfo::default().code(&code);

        unsafe {
            device
                .device
                .create_shader_module(&create_info, None)
                .map_err(|e| EngineError::Shader {
                    stage,
                    reason: format!("module creation: {:?}", e),
                })
        }
    }
}
