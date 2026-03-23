//! GPU resource management for fire simulation volumes.
//!
//! Each fire instance owns a set of 3D textures that store the fluid simulation
//! state. This module manages their allocation, ping-pong swapping, and cleanup.

use std::sync::Arc;

use ash::vk;

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::rendering::texture::ManagedTexture;

/// Default volume resolution (width x height x depth).
/// Height is taller because fire rises upward.
pub const DEFAULT_VOLUME_WIDTH: u32 = 32;
pub const DEFAULT_VOLUME_HEIGHT: u32 = 48;
pub const DEFAULT_VOLUME_DEPTH: u32 = 32;

/// GPU resources for a single fire simulation volume.
///
/// Owns two sets of field + velocity textures for ping-pong double buffering,
/// plus two single-channel pressure textures for the Jacobi solver.
pub struct FireVolume {
    /// Field textures (temperature, fuel, smoke, divergence). Ping-pong pair.
    pub field: [ManagedTexture; 2],
    /// Velocity textures (vx, vy, vz, pressure). Ping-pong pair.
    pub velocity: [ManagedTexture; 2],
    /// Pressure textures for Jacobi iteration. Ping-pong pair.
    pub pressure: [ManagedTexture; 2],
    /// Which index (0 or 1) is the current "source" for reads.
    pub read_index: usize,
    /// Volume dimensions.
    pub width: u32,
    pub height: u32,
    pub depth: u32,
}

impl FireVolume {
    /// Allocate a new fire volume with the default resolution.
    pub fn new(device: Arc<ManagedDevice>) -> EngineResult<Self> {
        Self::with_resolution(device, DEFAULT_VOLUME_WIDTH, DEFAULT_VOLUME_HEIGHT, DEFAULT_VOLUME_DEPTH)
    }

    /// Allocate a new fire volume with a custom resolution.
    pub fn with_resolution(
        device: Arc<ManagedDevice>,
        width: u32,
        height: u32,
        depth: u32,
    ) -> EngineResult<Self> {
        let field_usage = vk::ImageUsageFlags::STORAGE | vk::ImageUsageFlags::SAMPLED;
        let format = vk::Format::R16G16B16A16_SFLOAT;

        let field = [
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
        ];

        let velocity = [
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
        ];

        let pressure = [
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
            ManagedTexture::new_3d(device.clone(), width, height, depth, format, field_usage)?,
        ];

        Ok(Self {
            field,
            velocity,
            pressure,
            read_index: 0,
            width,
            height,
            depth,
        })
    }

    /// Swap read/write indices after a simulation step.
    pub fn swap(&mut self) {
        self.read_index = 1 - self.read_index;
    }

    /// Index of the texture to read from (source).
    pub fn src(&self) -> usize {
        self.read_index
    }

    /// Index of the texture to write to (destination).
    pub fn dst(&self) -> usize {
        1 - self.read_index
    }

    /// Workgroup count for dispatching compute shaders over this volume.
    /// Assumes workgroup size of (4, 4, 4).
    pub fn dispatch_size(&self) -> (u32, u32, u32) {
        (
            (self.width + 3) / 4,
            (self.height + 3) / 4,
            (self.depth + 3) / 4,
        )
    }
}
