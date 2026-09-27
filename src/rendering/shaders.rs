//! Centralized shader management.
//!
//! Provides compile-time embedded shaders and uniform loading functions
//! for all pipeline types in the engine.

use std::io::Cursor;

use ash::{util::read_spv, vk};

use crate::core::device::ManagedDevice;
use crate::core::error::{EngineError, EngineResult, ShaderStage};

/// Includes the SPIR-V that `build.rs` compiled from `shader/<source>`.
#[macro_export]
macro_rules! embedded_spirv {
    ($source:literal) => {
        include_bytes!(concat!(env!("OUT_DIR"), "/shader/", $source, ".spv"))
    };
}

/// Embedded shader bytecode.
mod bytecode {
    /// Main 3D pipeline shaders
    pub const MAIN_VERTEX: &[u8] = embedded_spirv!("triangle.vert");
    pub const MAIN_FRAGMENT: &[u8] = embedded_spirv!("triangle.frag");

    /// Overlay/UI pipeline shaders
    pub const OVERLAY_VERTEX: &[u8] = embedded_spirv!("overlay.vert");
    pub const OVERLAY_FRAGMENT: &[u8] = embedded_spirv!("overlay.frag");

    /// Particle system shaders
    pub const PARTICLE_VERTEX: &[u8] = embedded_spirv!("particle.vert");
    pub const PARTICLE_FRAGMENT: &[u8] = embedded_spirv!("particle.frag");

    /// Sun shadow map pass (depth only)
    pub const SHADOW_VERTEX: &[u8] = embedded_spirv!("shadow.vert");
    pub const SHADOW_FRAGMENT: &[u8] = embedded_spirv!("shadow.frag");

    /// Sky rendering shaders
    pub const SKY_VERTEX: &[u8] = embedded_spirv!("sky.vert");
    pub const SKY_FRAGMENT: &[u8] = embedded_spirv!("sky.frag");

    /// Water rendering shaders
    pub const WATER_VERTEX: &[u8] = embedded_spirv!("water.vert");
    pub const WATER_FRAGMENT: &[u8] = embedded_spirv!("water.frag");
    pub const RIPPLE_VERTEX: &[u8] = embedded_spirv!("ripple.vert");
    pub const RIVER_VERTEX: &[u8] = embedded_spirv!("river.vert");
    pub const FALL_VERTEX: &[u8] = embedded_spirv!("fall.vert");
    pub const FALL_FRAGMENT: &[u8] = embedded_spirv!("fall.frag");

    /// Post-processing shaders (HDR resolve, bloom)
    pub const POST_FULLSCREEN_VERTEX: &[u8] = embedded_spirv!("post/fullscreen.vert");
    pub const POST_BRIGHT_PASS: &[u8] = embedded_spirv!("post/bright_pass.frag");
    pub const POST_BLUR: &[u8] = embedded_spirv!("post/blur.frag");
    pub const POST_COMPOSITE: &[u8] = embedded_spirv!("post/composite.frag");
    pub const POST_BLOOM_OVERLAY: &[u8] = embedded_spirv!("post/bloom_overlay.frag");
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

    /// Load the shadow map vertex shader.
    pub fn load_shadow_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::SHADOW_VERTEX, ShaderStage::Vertex)
    }

    /// Load the shadow map fragment shader (writes nothing; see shadow.frag).
    pub fn load_shadow_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::SHADOW_FRAGMENT, ShaderStage::Fragment)
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

    /// Load the vertex shader of an awake ripple tile's fine surface.
    pub fn load_ripple_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::RIPPLE_VERTEX, ShaderStage::Vertex)
    }

    /// Load the vertex shader of a reach's surface.
    pub fn load_river_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::RIVER_VERTEX, ShaderStage::Vertex)
    }

    /// Load the vertex shader of a fall's sheet.
    pub fn load_fall_vertex(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::FALL_VERTEX, ShaderStage::Vertex)
    }

    /// Load the fragment shader of a fall's sheet.
    pub fn load_fall_fragment(device: &ManagedDevice) -> EngineResult<vk::ShaderModule> {
        Self::load_shader(device, bytecode::FALL_FRAGMENT, ShaderStage::Fragment)
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
