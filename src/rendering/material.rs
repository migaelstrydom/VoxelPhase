use crate::rendering::colour::Colour;
use crate::resources::textures::TextureHandle;

/// Index into MaterialManager's materials array.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MaterialId(pub u32);

/// How a surface responds to incoming light, independent of its colour.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceFinish {
    /// Perceptual roughness: 0 is a mirror-sharp highlight, 1 is fully diffuse.
    pub roughness: f32,

    /// 0 is a dielectric (white highlight, full diffuse), 1 is a metal
    /// (highlight tinted by base colour, no diffuse).
    pub metallic: f32,
}

impl SurfaceFinish {
    /// Matte, non-metallic. The look of the pre-BRDF renderer.
    pub const MATTE: Self = Self {
        roughness: 1.0,
        metallic: 0.0,
    };

    /// Glossy dielectric with a tight highlight — plastic, glazed ceramic, wet stone.
    pub const POLISHED: Self = Self {
        roughness: 0.15,
        metallic: 0.0,
    };

    /// Polished metal: highlight takes the base colour, no diffuse response.
    #[allow(dead_code)]
    pub const METAL: Self = Self {
        roughness: 0.25,
        metallic: 1.0,
    };
}

impl Default for SurfaceFinish {
    fn default() -> Self {
        Self::MATTE
    }
}

/// Light a surface radiates on its own, regardless of incoming light.
///
/// Emissive surfaces bloom; illuminating their surroundings is a separate
/// concern owned by `PointLight` (see `src/lighting/`). The two are deliberately
/// independent — a glowing sign need not light the street — so an object that
/// should do both carries both, and their colours are kept in step by hand.
#[derive(Clone, Copy, Debug)]
pub struct Emission {
    /// Linear colour of the emitted light. Sets the hue only; how bright the
    /// surface glows is `strength` alone.
    pub colour: Colour,

    /// Emitted brightness, as luminance.
    ///
    /// Deliberately *not* a plain multiplier on `colour`. Multiplying would make
    /// the same number mean different brightness at every hue, because a
    /// saturated blue carries far less luminance than a yellow of the same
    /// magnitude — so a red gem and a cyan orb given the same value would glow
    /// differently, and only one of them might cross the bloom threshold.
    /// Expressed as luminance, the number means the same thing at any hue and
    /// is directly comparable to `PostProcessConfig::bloom_threshold`: above it
    /// the surface blooms, below it the surface is merely bright.
    ///
    /// The scene target is floating point, so values above 1.0 are preserved
    /// rather than clipped.
    pub strength: f32,

    /// Extra emissive brightening towards the silhouette, where a curved
    /// surface turns away from the viewer. Sells a glowing volume.
    pub rim_strength: f32,

    /// Tightness of the rim falloff. Higher values confine it to the very edge.
    pub rim_power: f32,
}

impl Emission {
    /// No self-illumination.
    pub const NONE: Self = Self {
        colour: Colour::BLACK,
        strength: 0.0,
        rim_strength: 0.0,
        rim_power: 3.0,
    };

    /// Uniform glow in the given colour at the given luminance, with no rim
    /// enhancement.
    #[allow(dead_code)]
    pub fn glow(colour: Colour, strength: f32) -> Self {
        Self {
            colour,
            strength,
            ..Self::NONE
        }
    }

    /// Scalar the shader multiplies `colour` by to reach `strength` luminance.
    ///
    /// Folding the hue normalisation into this scalar rather than into the
    /// packed colour keeps `colour` available to the shader at its authored
    /// magnitude. The rim term (shader/triangle.frag) reads the normalised
    /// product instead, via `materialEmissive()`, so its brightness stays
    /// proportional to emitted luminance rather than to the authored colour's
    /// magnitude.
    ///
    /// A colour with no luminance (black) cannot be scaled to any brightness,
    /// so it emits nothing.
    fn radiance_scale(&self) -> f32 {
        let luminance = self.colour.luminance();
        if luminance <= f32::EPSILON {
            0.0
        } else {
            self.strength / luminance
        }
    }
}

impl Default for Emission {
    fn default() -> Self {
        Self::NONE
    }
}

/// Defines how a surface is rendered.
///
/// `base_colour` is stored for future PBR support but not yet applied;
/// colours are currently baked into vertex data.
#[derive(Clone)]
pub struct Material {
    /// Base colour (stored for future PBR, currently unused by renderer).
    #[allow(dead_code)]
    pub base_colour: Colour,

    /// Optional diffuse texture. If None, the fallback white texture is used.
    pub diffuse_texture: Option<TextureHandle>,

    /// Specular response.
    pub finish: SurfaceFinish,

    /// Self-illumination.
    pub emission: Emission,
}

impl Material {
    /// Create a solid colour material with no texture.
    pub fn coloured(colour: Colour) -> Self {
        Self {
            base_colour: colour,
            diffuse_texture: None,
            finish: SurfaceFinish::default(),
            emission: Emission::default(),
        }
    }

    /// Create a textured material with white base colour.
    #[allow(dead_code)]
    pub fn textured(texture: TextureHandle) -> Self {
        Self {
            base_colour: Colour::WHITE,
            diffuse_texture: Some(texture),
            finish: SurfaceFinish::default(),
            emission: Emission::default(),
        }
    }

    /// Set the specular response.
    pub fn with_finish(mut self, finish: SurfaceFinish) -> Self {
        self.finish = finish;
        self
    }

    /// Set the self-illumination.
    pub fn with_emission(mut self, emission: Emission) -> Self {
        self.emission = emission;
        self
    }

    /// Pack this material's lighting parameters for the fragment push constant.
    pub fn surface_params(&self) -> SurfaceParams {
        SurfaceParams {
            emissive: [
                self.emission.colour.r,
                self.emission.colour.g,
                self.emission.colour.b,
                self.emission.radiance_scale(),
            ],
            surface: [
                self.finish.roughness,
                self.finish.metallic,
                self.emission.rim_strength,
                self.emission.rim_power,
            ],
        }
    }
}

/// GPU-facing material parameters, pushed per draw call.
///
/// Layout must match the `MaterialPushConstants` block in shader/material.glsl,
/// which declares it at byte offset `SURFACE_PARAMS_OFFSET`.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct SurfaceParams {
    /// rgb = emissive colour, w = emissive strength.
    pub emissive: [f32; 4],

    /// x = roughness, y = metallic, z = rim strength, w = rim power.
    pub surface: [f32; 4],
}

/// Byte offset of `SurfaceParams` within the fragment push-constant range.
/// Follows the 64-byte vertex model matrix and the 16-byte colour override.
pub const SURFACE_PARAMS_OFFSET: u32 = 80;

impl SurfaceParams {
    /// Parameters for an unlit-looking matte surface, used where no material
    /// is available (debug overlays, procedural meshes).
    pub const MATTE: Self = Self {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [1.0, 0.0, 0.0, 3.0],
    };

    /// View as raw bytes for `cmd_push_constants`.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            std::slice::from_raw_parts(
                self as *const Self as *const u8,
                std::mem::size_of::<Self>(),
            )
        }
    }
}

/// Mutable builder for registering materials during initialization.
pub struct MaterialManagerBuilder {
    materials: Vec<Material>,
}

impl MaterialManagerBuilder {
    pub fn new() -> Self {
        Self {
            materials: Vec::new(),
        }
    }

    /// Register a material and return its ID.
    pub fn register(&mut self, material: Material) -> MaterialId {
        let id = MaterialId(self.materials.len() as u32);
        self.materials.push(material);
        id
    }

    /// Freeze into an immutable MaterialManager.
    pub fn build(self, fallback_texture: TextureHandle) -> MaterialManager {
        MaterialManager {
            materials: self.materials,
            fallback_texture,
        }
    }
}

impl Default for MaterialManagerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Immutable material manager for runtime access.
pub struct MaterialManager {
    materials: Vec<Material>,
    fallback_texture: TextureHandle,
}

impl MaterialManager {
    /// Get a material by ID.
    pub fn get(&self, id: MaterialId) -> &Material {
        &self.materials[id.0 as usize]
    }

    /// Get the effective texture for a material (fallback if None).
    pub fn get_effective_texture(&self, id: MaterialId) -> &TextureHandle {
        self.get(id)
            .diffuse_texture
            .as_ref()
            .unwrap_or(&self.fallback_texture)
    }

    /// Get the fallback (white) texture.
    pub fn fallback_texture(&self) -> &TextureHandle {
        &self.fallback_texture
    }

    /// Get the packed lighting parameters for a material.
    pub fn get_surface_params(&self, id: MaterialId) -> SurfaceParams {
        self.get(id).surface_params()
    }
}
