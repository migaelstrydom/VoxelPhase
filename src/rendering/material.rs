use nalgebra::Vector3;

use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::reflection::{ProbeSlot, Reflects};
use crate::rendering::surface_source::SurfaceSource;
use crate::rendering::transparency::Transparency;
use crate::rendering::triplanar::TriplanarProjection;
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

    /// How the diffuse texture is addressed. Disabled by default, which uses
    /// the mesh's own vertex texture coordinates.
    pub projection: TriplanarProjection,

    /// The microstructure this surface shows under a highlight. None by
    /// default: grain on everything costs the surfaces that need it their
    /// contrast against the ones that do not.
    pub grain: GrainSpec,

    /// Which shading inputs this material asks for. Set by the grain builders
    /// rather than by hand; a material that says nothing takes `PLAIN`.
    pub source: SurfaceSource,

    /// How much light passes through the surface. Opaque by default, and the
    /// one dial that decides which of the frame's two geometry passes a draw
    /// belongs to — see `rendering::transparency`.
    pub transparency: Transparency,

    /// Depth of the relief carried in the diffuse texture's alpha, in texture
    /// coordinates. Zero — the default — means the alpha is coverage, as it
    /// is for every texture that was not baked with relief.
    pub relief: f32,

    /// What the surface's reflection shows: the sky, by default, or the
    /// object's actual surroundings from a reflection probe.
    pub reflects: Reflects,
}

impl Material {
    /// Create a solid colour material with no texture.
    pub fn coloured(colour: Colour) -> Self {
        Self {
            base_colour: colour,
            diffuse_texture: None,
            finish: SurfaceFinish::default(),
            emission: Emission::default(),
            projection: TriplanarProjection::default(),
            grain: GrainSpec::NONE,
            source: SurfaceSource::PLAIN,
            transparency: Transparency::OPAQUE,
            relief: 0.0,
            reflects: Reflects::Sky,
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
            projection: TriplanarProjection::default(),
            grain: GrainSpec::NONE,
            source: SurfaceSource::PLAIN,
            transparency: Transparency::OPAQUE,
            relief: 0.0,
            reflects: Reflects::Sky,
        }
    }

    /// Set the specular response.
    pub fn with_finish(mut self, finish: SurfaceFinish) -> Self {
        self.finish = finish;
        self
    }

    /// Give this material a microstructure, projected from the object's own
    /// frame so that it stays put when the object moves.
    pub fn with_grain(mut self, grain: GrainSpec) -> Self {
        self.grain = grain;
        if grain.is_enabled() {
            self.source = self.source.with(SurfaceSource::GRAIN_OBJECT_SPACE);
        }
        self
    }

    /// Give this material a microstructure addressed by its texture
    /// coordinates, for grain with a direction the mesh knows — wood fibre.
    pub fn with_uv_grain(mut self, grain: GrainSpec) -> Self {
        self.grain = grain;
        if grain.is_enabled() {
            self.source = self.source.with(SurfaceSource::GRAIN_BY_UV);
        }
        self
    }

    /// Let light through this surface.
    ///
    /// Moves the material into the sorted blended pass; see
    /// [`Transparency::is_blended`].
    pub fn with_transparency(mut self, transparency: Transparency) -> Self {
        self.transparency = transparency;
        self
    }

    /// Read the diffuse texture's alpha as relief `depth` texture coordinates
    /// deep, rather than as coverage.
    ///
    /// For a texture baked from a pattern that cuts into the surface — see
    /// [`Pattern::relief_depth`](crate::rendering::pattern::Pattern::relief_depth).
    /// The depth is in texture coordinates rather than metres so that the
    /// relief keeps its proportions whatever scale the mesh lays the texture
    /// at: a crack is as deep, relative to its width, on a big block as on a
    /// small one.
    pub fn with_relief(mut self, depth: f32) -> Self {
        self.relief = depth;
        if depth > 0.0 {
            self.source = self.source.with(SurfaceSource::RELIEF_IN_ALPHA);
        }
        self
    }

    /// Choose what the surface reflects. See [`Reflects`].
    pub fn with_reflections(mut self, reflects: Reflects) -> Self {
        self.reflects = reflects;
        self
    }

    /// Set the self-illumination.
    pub fn with_emission(mut self, emission: Emission) -> Self {
        self.emission = emission;
        self
    }

    /// Texture this material by world position rather than by vertex UVs.
    #[allow(dead_code)]
    pub fn with_projection(mut self, projection: TriplanarProjection) -> Self {
        self.projection = projection;
        self
    }

    /// This material's shading parameters, for the frame's surface table.
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
            projection: self.projection.packed(),
            source: self.source,
            grain: self.grain,
            transparency: self.transparency,
            relief: self.relief,
            reflects: self.reflects,
            probe: None,
            projection_anchor: [0.0; 3],
        }
    }
}

/// Per-instance adjustment applied on top of a material's authored parameters.
///
/// Materials are shared: every grenade in flight draws from the same
/// `Material`, so a material cannot hold state that differs between instances.
/// This carries the part of a surface that *does* differ — how hot this
/// particular object is right now — and is folded into the pushed
/// [`SurfaceParams`] at draw time, leaving the material itself untouched.
///
/// Kept deliberately narrow: only what an animator needs to drive per frame.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceModulation {
    /// Multiplier on the material's emissive luminance. 1.0 draws the surface
    /// exactly as authored; above 1.0 pushes it further past the bloom
    /// threshold, and 0.0 puts the glow out entirely.
    pub emissive_scale: f32,
}

impl SurfaceModulation {
    /// Draw the material exactly as authored.
    pub const IDENTITY: Self = Self {
        emissive_scale: 1.0,
    };

    /// Scale the material's emissive luminance.
    pub const fn emissive(scale: f32) -> Self {
        Self {
            emissive_scale: scale,
        }
    }
}

impl Default for SurfaceModulation {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// GPU-facing surface parameters, one entry in the frame's surface table.
///
/// Layout must match the `GpuSurface` struct in shader/material.glsl. Whole
/// `vec4`s exactly: std430 aligns a struct to its largest member, so anything
/// that is not a multiple of 16 bytes here would be padded to one on the GPU
/// and every entry after the first would be read from the wrong offset.
///
/// The spare slots are deliberate headroom. They are what the old
/// push-constant layout had no room for, and what per-material surface detail
/// is written into.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct GpuSurface {
    /// rgb = emissive colour, w = emissive radiance scale.
    pub emissive: [f32; 4],

    /// x = roughness, y = metallic, z = rim strength, w = rim power.
    pub surface: [f32; 4],

    /// x = albedo triplanar scale in texture repeats per world unit,
    /// y = blend sharpness, z = grain scale, w = grain strength.
    pub projection: [f32; 4],

    /// x = [`SurfaceSource`] flags, y = grain index, zw spare.
    pub control: [u32; 4],

    /// x = head-on opacity, y = reflectance at normal incidence, zw spare.
    /// See [`Transparency`].
    pub optics: [f32; 4],

    /// x = depth of the relief in the diffuse alpha, in texture coordinates
    /// (see [`Material::with_relief`]), yzw = the projection anchor (see
    /// [`SurfaceParams::anchored_at`]).
    pub detail: [f32; 4],
}

impl GpuSurface {
    /// `control.z` of a surface that reflects no probe. Matches `NO_PROBE` in
    /// material.glsl.
    pub const NO_PROBE: u32 = u32::MAX;

    /// Parameters for an unlit-looking matte surface, used where no material
    /// is available (debug overlays, procedural meshes).
    pub const MATTE: Self = Self {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [1.0, 0.0, 0.0, 3.0],
        projection: [0.0, 0.0, 0.0, 0.0],
        control: [0, 0, Self::NO_PROBE, 0],
        optics: [1.0, 0.04, 0.0, 0.0],
        detail: [0.0; 4],
    };
}

/// A surface's shading parameters, as the CPU describes them.
///
/// Kept distinct from [`GpuSurface`] so that call sites read in terms of the
/// dials they are setting rather than in terms of packed vectors, and so that
/// the GPU layout can be repacked without touching them.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceParams {
    pub emissive: [f32; 4],
    pub surface: [f32; 4],
    pub projection: [f32; 2],

    /// Which shading inputs this surface asks for.
    pub source: SurfaceSource,

    /// The microstructure this surface shows under a highlight.
    pub grain: GrainSpec,

    /// How much light passes through. Decides which geometry pass the draw
    /// carrying these parameters is recorded into.
    pub transparency: Transparency,

    /// Depth of the relief in the diffuse alpha, in texture coordinates. Zero
    /// when the alpha is coverage.
    pub relief: f32,

    /// What the surface asks to reflect.
    pub reflects: Reflects,

    /// The probe this draw reflects, when it asked for one and its object
    /// holds one. Set per draw by the renderer, never by a material.
    pub probe: Option<ProbeSlot>,

    /// Added to model position to address a model-space triplanar albedo.
    /// Read only with [`SurfaceSource::ALBEDO_MODEL_SPACE`]; set per draw.
    pub projection_anchor: [f32; 3],
}

impl SurfaceParams {
    /// Parameters for an unlit-looking matte surface, used where no material
    /// is available (debug overlays, procedural meshes).
    pub const MATTE: Self = Self {
        emissive: [0.0, 0.0, 0.0, 0.0],
        surface: [1.0, 0.0, 0.0, 3.0],
        projection: [0.0, 0.0],
        source: SurfaceSource::PLAIN,
        grain: GrainSpec::NONE,
        transparency: Transparency::OPAQUE,
        relief: 0.0,
        reflects: Reflects::Sky,
        probe: None,
        projection_anchor: [0.0; 3],
    };

    /// Give this surface a microstructure, projected from the object's own
    /// frame so that it stays put when the object moves.
    ///
    /// A grain with no scale or no strength is left off rather than costing a
    /// texture read that changes nothing.
    pub fn with_grain(mut self, grain: GrainSpec) -> Self {
        self.grain = grain;
        if grain.is_enabled() {
            self.source = self.source.with(SurfaceSource::GRAIN_OBJECT_SPACE);
        }
        self
    }

    /// Give this surface a microstructure addressed by its own texture
    /// coordinates, for grain with a direction the mesh already knows.
    pub fn with_uv_grain(mut self, grain: GrainSpec) -> Self {
        self.grain = grain;
        if grain.is_enabled() {
            self.source = self.source.with(SurfaceSource::GRAIN_BY_UV);
        }
        self
    }

    /// Let light through this surface, moving its draw into the sorted
    /// blended pass.
    pub fn with_transparency(mut self, transparency: Transparency) -> Self {
        self.transparency = transparency;
        self
    }

    /// Select this surface's shading inputs.
    pub fn with_source(mut self, source: SurfaceSource) -> Self {
        self.source = source;
        self
    }

    /// Texture by world position rather than by the mesh's vertex UVs.
    pub fn with_projection(mut self, projection: TriplanarProjection) -> Self {
        self.projection = projection.packed();
        self
    }

    /// Address the triplanar albedo by model position plus `anchor` rather
    /// than by world position, so the texture rides with the mesh. A mesh
    /// whose model position plus `anchor` was its world position when it
    /// stopped being still ground shows the same texture it had there.
    pub fn anchored_at(mut self, anchor: Vector3<f32>) -> Self {
        self.source = self.source.with(SurfaceSource::ALBEDO_MODEL_SPACE);
        self.projection_anchor = anchor.into();
        self
    }

    /// Reflect the probe in `slot`, or the sky alone when `None`.
    pub fn with_probe(mut self, slot: Option<ProbeSlot>) -> Self {
        self.probe = slot;
        self
    }

    /// Apply a per-instance modulation to these parameters.
    ///
    /// The emissive scale lands on `emissive.w` — the CPU-side radiance scale —
    /// rather than on the colour, so the hue normalisation established by
    /// `Emission::radiance_scale` survives and the result still means "this
    /// luminance, whatever the hue". The rim term reads the same product, so
    /// the silhouette glow brightens in step with the surface.
    pub fn modulated(mut self, modulation: SurfaceModulation) -> Self {
        self.emissive[3] *= modulation.emissive_scale;
        self
    }

    /// Pack for the surface table.
    pub fn to_gpu(self) -> GpuSurface {
        GpuSurface {
            emissive: self.emissive,
            surface: self.surface,
            projection: [
                self.projection[0],
                self.projection[1],
                self.grain.scale,
                self.grain.strength,
            ],
            control: [
                self.source.0,
                self.grain.layer.index(),
                self.probe.map_or(GpuSurface::NO_PROBE, |slot| slot.0),
                0,
            ],
            optics: [
                self.transparency.opacity,
                self.transparency.reflectance(),
                0.0,
                0.0,
            ],
            detail: [
                self.relief,
                self.projection_anchor[0],
                self.projection_anchor[1],
                self.projection_anchor[2],
            ],
        }
    }
}

/// Byte offset of the surface index within the fragment push-constant range.
/// Follows the 64-byte vertex model matrix and the 16-byte colour override.
pub const SURFACE_INDEX_OFFSET: u32 = 80;

/// Byte offset of the vertex stage's clip plane: after the surface index,
/// padded to a vec4's alignment.
pub const CLIP_PLANE_OFFSET: u32 = 96;

/// Bytes of the geometry pipelines' push-constant range.
pub const GEOMETRY_PUSH_SIZE: u32 = CLIP_PLANE_OFFSET + 16;

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

#[cfg(test)]
mod tests {
    use super::*;

    /// Vulkan only guarantees 128 bytes of push constants. The surface index
    /// exists precisely so that this budget stops being the material system's
    /// ceiling — but the model matrix, the colour override, the index and the
    /// clip plane still have to fit inside it.
    #[test]
    fn the_push_constants_fit_the_guaranteed_budget() {
        const GUARANTEED_BUDGET: usize = 128;
        let used = GEOMETRY_PUSH_SIZE as usize;
        assert!(
            used <= GUARANTEED_BUDGET,
            "push constants use {used} bytes of a guaranteed {GUARANTEED_BUDGET}"
        );
    }

    /// The point of the move: there is now room to grow. A material dial added
    /// later must land in the surface table, never back in the push constants.
    #[test]
    fn the_surface_table_carries_the_parameters_not_the_push_constants() {
        assert_eq!(std::mem::size_of::<SurfaceParams>() > 4, true);
        assert_eq!(std::mem::size_of::<GpuSurface>(), 96);
    }

    #[test]
    fn a_material_textures_by_its_uvs_unless_told_otherwise() {
        let params = Material::coloured(Colour::WHITE).surface_params();
        assert_eq!(params.projection[0], 0.0);
    }

    #[test]
    fn a_projection_survives_the_trip_into_the_push_constant() {
        let params = Material::coloured(Colour::WHITE)
            .with_projection(TriplanarProjection::new(0.1, 4.0))
            .surface_params();
        assert_eq!(
            params.projection,
            TriplanarProjection::new(0.1, 4.0).packed()
        );
    }

    /// Relief is opt-in: a material says nothing about it and its texture's
    /// alpha stays coverage.
    #[test]
    fn relief_is_asked_for_and_reaches_the_surface_table() {
        let plain = Material::coloured(Colour::WHITE).surface_params();
        assert!(!plain.source.contains(SurfaceSource::RELIEF_IN_ALPHA));
        assert_eq!(plain.to_gpu().detail[0], 0.0);

        let carved = Material::coloured(Colour::WHITE)
            .with_relief(0.004)
            .surface_params();
        assert!(carved.source.contains(SurfaceSource::RELIEF_IN_ALPHA));
        assert_eq!(carved.to_gpu().detail[0], 0.004);
        assert!(!Material::coloured(Colour::WHITE)
            .with_relief(0.0)
            .source
            .contains(SurfaceSource::RELIEF_IN_ALPHA));
    }

    /// An anchored projection reaches the table beside the relief without
    /// disturbing it, and only an anchored surface asks for model space.
    #[test]
    fn an_anchor_reaches_the_surface_table_beside_the_relief() {
        let plain = Material::coloured(Colour::WHITE).surface_params();
        assert!(!plain.source.contains(SurfaceSource::ALBEDO_MODEL_SPACE));

        let anchored = Material::coloured(Colour::WHITE)
            .with_relief(0.004)
            .surface_params()
            .anchored_at(Vector3::new(1.0, -2.0, 3.5));
        assert!(anchored.source.contains(SurfaceSource::ALBEDO_MODEL_SPACE));
        assert!(anchored.source.contains(SurfaceSource::RELIEF_IN_ALPHA));
        assert_eq!(anchored.to_gpu().detail, [0.004, 1.0, -2.0, 3.5]);
    }
}
