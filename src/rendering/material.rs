use crate::rendering::colour::Colour;
use crate::resources::textures::TextureHandle;

/// Index into MaterialManager's materials array.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MaterialId(pub u32);

/// Defines how a surface is rendered.
///
/// Currently, only `diffuse_texture` is used by the renderer.
/// `base_colour` is stored for future PBR support but not yet applied;
/// colors are currently baked into vertex data.
#[derive(Clone)]
pub struct Material {
    /// Base colour (stored for future PBR, currently unused by renderer).
    #[allow(dead_code)]
    pub base_colour: Colour,

    /// Optional diffuse texture. If None, the fallback white texture is used.
    pub diffuse_texture: Option<TextureHandle>,
}

impl Material {
    /// Create a solid colour material with no texture.
    pub fn coloured(colour: Colour) -> Self {
        Self {
            base_colour: colour,
            diffuse_texture: None,
        }
    }

    /// Create a textured material with white base colour.
    pub fn textured(texture: TextureHandle) -> Self {
        Self {
            base_colour: Colour::WHITE,
            diffuse_texture: Some(texture),
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
}
