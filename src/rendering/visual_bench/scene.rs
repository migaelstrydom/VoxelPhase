//! What a visual scene is made of.

use nalgebra::{Matrix4, Point3, Vector3};

use crate::core::error::EngineResult;
use crate::lighting::{ActiveLight, ActiveLights};
use crate::rendering::colour::Colour;
use crate::rendering::frame::SceneLighting;
use crate::rendering::material::{MaterialManager, SurfaceParams};
use crate::rendering::post::PostProcessConfig;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::{TextureHandle, TextureManager};

/// Resources a scene may draw on while building its shots.
///
/// Passed in rather than owned because textures need a live GPU context. A
/// scene that only wants vertex-coloured geometry can ignore it.
pub struct SceneContext<'a> {
    pub textures: &'a TextureManager,
    pub materials: &'a MaterialManager,
}

/// Where the camera sits for a shot.
///
/// Explicit rather than orbit-based: a shot that is compared against its own
/// past must frame its subject identically every time.
#[derive(Clone, Copy, Debug)]
pub struct SceneCamera {
    pub eye: Point3<f32>,
    pub target: Point3<f32>,
    pub fov_y_degrees: f32,
    pub near: f32,
    pub far: f32,
}

impl SceneCamera {
    /// A camera looking at `target` from `eye` with sensible clip planes.
    pub fn looking_at(eye: Point3<f32>, target: Point3<f32>) -> Self {
        Self {
            eye,
            target,
            fov_y_degrees: 45.0,
            near: 0.1,
            far: 500.0,
        }
    }

    pub fn with_fov(mut self, degrees: f32) -> Self {
        self.fov_y_degrees = degrees;
        self
    }

    pub fn view(&self) -> Matrix4<f32> {
        Matrix4::look_at_rh(&self.eye, &self.target, &Vector3::y())
    }

    /// Projection matrix, with the Y flip Vulkan's clip space needs.
    pub fn projection(&self, aspect: f32) -> Matrix4<f32> {
        let mut proj =
            Matrix4::new_perspective(aspect, self.fov_y_degrees.to_radians(), self.near, self.far);
        proj[(1, 1)] *= -1.0;
        proj
    }
}

/// The lighting environment a shot is rendered under.
#[derive(Clone, Debug, Default)]
pub struct SceneEnvironment {
    /// Sun direction, colour and intensity. The direction also drives where the
    /// sky draws its disc, so shaded geometry and the visible sun agree.
    pub lighting: SceneLighting,

    /// Point lights, authored directly rather than collected from a world.
    pub point_lights: Vec<ActiveLight>,

    /// Tonemap and bloom settings. Part of the environment because a shot that
    /// is judging emissive materials needs to pin the exposure it judged them at.
    pub post: PostProcessConfig,
}

impl SceneEnvironment {
    /// Point the sun at a given direction (from surface towards the sun).
    pub fn with_sun(mut self, direction: Vector3<f32>) -> Self {
        self.lighting.sun_direction = direction.normalize();
        self
    }

    pub fn with_ambient(mut self, colour: Colour) -> Self {
        self.lighting.ambient_colour = colour;
        self
    }

    pub fn with_point_light(mut self, light: ActiveLight) -> Self {
        self.point_lights.push(light);
        self
    }

    pub fn active_lights(&self) -> ActiveLights {
        ActiveLights::from_lights(self.point_lights.clone())
    }
}

/// One drawable in a shot.
///
/// Geometry is carried as raw vertices and indices rather than as a model
/// handle, so a scene can generate exactly the mesh it wants to look at without
/// going through the asset pipeline.
pub struct SceneMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub transform: Matrix4<f32>,

    /// Roughness, metallic, emissive and rim for this draw.
    pub surface: SurfaceParams,

    /// Texture to sample. `None` uses the material manager's fallback white, so
    /// vertex colours come through unmodified.
    pub texture: Option<TextureHandle>,
}

impl SceneMesh {
    pub fn new(vertices: Vec<Vertex>, indices: Vec<u32>) -> Self {
        Self {
            vertices,
            indices,
            transform: Matrix4::identity(),
            surface: SurfaceParams::MATTE,
            texture: None,
        }
    }

    pub fn at(mut self, position: Vector3<f32>) -> Self {
        self.transform = Matrix4::new_translation(&position);
        self
    }

    pub fn with_transform(mut self, transform: Matrix4<f32>) -> Self {
        self.transform = transform;
        self
    }

    pub fn with_surface(mut self, surface: SurfaceParams) -> Self {
        self.surface = surface;
        self
    }

    pub fn with_texture(mut self, texture: TextureHandle) -> Self {
        self.texture = Some(texture);
        self
    }
}

/// One rendered image: a complete description of a frame.
pub struct SceneShot {
    /// Caption for the contact sheet, and the filename stem when shots are
    /// written individually. Keep it short and stable — it is how a tile is
    /// identified across runs.
    pub label: String,
    pub camera: SceneCamera,
    pub environment: SceneEnvironment,
    pub meshes: Vec<SceneMesh>,
}

impl SceneShot {
    pub fn new(label: impl Into<String>, camera: SceneCamera) -> Self {
        Self {
            label: label.into(),
            camera,
            environment: SceneEnvironment::default(),
            meshes: Vec::new(),
        }
    }

    pub fn with_environment(mut self, environment: SceneEnvironment) -> Self {
        self.environment = environment;
        self
    }

    pub fn with_mesh(mut self, mesh: SceneMesh) -> Self {
        self.meshes.push(mesh);
        self
    }

    pub fn with_meshes(mut self, meshes: impl IntoIterator<Item = SceneMesh>) -> Self {
        self.meshes.extend(meshes);
        self
    }
}

/// A named set of shots that exercise one aspect of the renderer.
///
/// Scenes come in two useful shapes. A *sweep* varies one parameter across its
/// shots — a roughness ladder, a sun-angle series — and is read as a contact
/// sheet; this is how material and lighting work is actually tuned, and it is
/// the thing a windowed viewer cannot do. A *tableau* is a single considered
/// composition, for judging a look as a whole.
pub trait VisualScene {
    fn name(&self) -> &str;

    /// One line explaining what to look at. Printed by `--list`.
    fn description(&self) -> &str;

    /// Build every shot. Called once per run, with the bench's GPU resources
    /// available for anything that needs them.
    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>>;
}
