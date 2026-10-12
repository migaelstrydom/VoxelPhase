use ash::vk;
use nalgebra::{Vector2, Vector3, Vector4};

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

/// A vertex of a river surface: the water's own surface there, its depth,
/// its flow, and on a falling sheet the way the sheet faces.
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
    /// On a sheet of water falling over an edge, the way it faces: out over
    /// the edge, x and z. Zero on the surface, which faces up.
    pub face: Vector2<f32>,
    /// On a falling sheet, (how far this vertex hangs below the lip, the
    /// sheet's whole fall at this end, m; which end of the sheet's edge it
    /// is on, -1 or 1, pointing away from the other; where along the edge it
    /// hangs from, m): (0, fall, side, along) at the lip and (fall, fall,
    /// side, along) at the foot, wherever the foot is thrown. Zero on the
    /// surface.
    pub sheet: Vector4<f32>,
    /// On falling water, the way its face faces: a sheet or jet is drawn as
    /// a body with a top face and an under face (`nappe.rs`). Up on the
    /// surface.
    pub normal: Vector3<f32>,
    /// On falling water, 1 at an end of a fall, which frays out, and 0 at
    /// an end that joins the sheet beside it. Zero on the surface.
    pub fray: f32,
}

impl RiverVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 7] {
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
            vk::VertexInputAttributeDescription {
                location: 3,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 6 * f,
            },
            // (drop below the lip, whole fall, side, along)
            vk::VertexInputAttributeDescription {
                location: 4,
                binding: 0,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: 8 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 5,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 12 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 6,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 15 * f,
            },
        ]
    }
}
