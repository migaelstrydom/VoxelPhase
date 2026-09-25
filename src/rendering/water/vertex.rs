use ash::vk;
use nalgebra::{Vector2, Vector3};

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

/// A vertex of a reach's surface. Its height is the section's bed plus the
/// design depth, scaled per draw to the reach's current discharge.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RiverVertex {
    pub xz: Vector2<f32>,
    /// Bed height at the section's centre.
    pub bed: f32,
    /// Water depth at the design discharge.
    pub depth: f32,
    /// Distance down the reach, m.
    pub along: f32,
    /// Flow velocity at the design discharge.
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
            // (bed, depth, along)
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 2 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 5 * f,
            },
        ]
    }
}

/// A vertex of a fall's sheet: a point on the arc, and the horizontal
/// direction across the sheet there. The shader sets the sheet's width from
/// the discharge now.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallVertex {
    /// On the arc.
    pub centre: Vector3<f32>,
    /// Unit, horizontal, across the sheet.
    pub side: Vector3<f32>,
    /// Which edge: −1 or +1.
    pub across: f32,
    /// Seconds from the lip.
    pub time: f32,
}

impl FallVertex {
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
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 0,
            },
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 3 * f,
            },
            // (across, time)
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 6 * f,
            },
        ]
    }
}
