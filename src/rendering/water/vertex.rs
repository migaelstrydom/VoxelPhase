use ash::vk;
use nalgebra::Vector2;

/// A water surface vertex. Its height is not stored: every draw pushes its
/// body's level, so the mesh outlives a change of level.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BasinVertex {
    /// World (x, z).
    pub xz: Vector2<f32>,
    /// The floor under this vertex, for the swell's shore fade.
    pub floor: f32,
    /// The share of the swell's height the geometry carries: 1, except in
    /// the ring past the map's edge, whose quads are too coarse to follow
    /// it. Its normal is the fragment shader's, at full swell.
    pub swell_share: f32,
}

impl BasinVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 3] {
        let float = std::mem::size_of::<f32>() as u32;
        [
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 2 * float,
            },
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 3 * float,
            },
        ]
    }
}

/// A vertex of the fine ripple-tile grid: its position within the 8 m tile.
/// The tile's origin, its ripples and its body's level come from the draw.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FineVertex {
    pub local: Vector2<f32>,
}

impl FineVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 1] {
        [vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32_SFLOAT,
            offset: 0,
        }]
    }
}

/// A vertex of a river surface: the water's own surface there, its depth and
/// its flow.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RiverVertex {
    pub xz: Vector2<f32>,
    /// Water surface height.
    pub surface: f32,
    /// Depth of water: positive under water, negative over a bank standing
    /// above it. The draw cuts the surface below its shore depth, so a shore
    /// is a contour, not the edge of a column.
    pub depth: f32,
    /// Depth-averaged velocity, m/s, x and z.
    pub flow: Vector2<f32>,
}

impl RiverVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 3] {
        let f = std::mem::size_of::<f32>() as u32;
        [
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            // (surface, depth)
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 2 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 4 * f,
            },
        ]
    }
}
