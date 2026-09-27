use ash::vk;

use crate::rendering::geometry_draw::{GeometryDraw, GeometryPush};
use crate::rendering::mesh_source::{MeshBindings, MeshSource};
use crate::rendering::renderer::PUSH_CONSTANT_STAGES;

/// What every geometry draw in a frame binds the same: the viewport and the
/// scene descriptor set, and the buffer pairs its meshes come from.
#[derive(Clone, Debug)]
pub struct SharedBindings {
    /// The extent the viewport and scissor cover.
    pub extent: vk::Extent2D,
    /// Set 0: scene uniforms, lights, surface table, shadow map.
    pub scene_set: vk::DescriptorSet,
    /// The frame's streamed buffers and the resident arena's blocks, as they
    /// stand once the frame's last mesh is committed.
    pub meshes: MeshBindings,
}

/// Records geometry draws into one command buffer, binding only what differs
/// from the draw before.
///
/// ```text
///   draw ──▶ shared bound? ──no──▶ viewport, scene set
///        mesh source same? ──no──▶ vertex and index buffers
///             pipeline same? ──no──▶ bind pipeline
///          texture set same? ──no──▶ bind set 1
///           push constants same? ──no──▶ push
///                                        ──▶ draw indexed
/// ```
///
/// A blended mesh is drawn twice in a row, back faces then front, so the
/// second draw of the pair costs a pipeline bind and the draw itself.
///
/// It tracks only what it bound itself. Anything else that records into the
/// same command buffer between two draws must be followed by
/// [`Self::interrupted`], which makes the next draw bind everything again.
pub struct GeometryRecorder<'a> {
    device: &'a ash::Device,
    cb: vk::CommandBuffer,
    layout: vk::PipelineLayout,
    shared: SharedBindings,
    /// Whether `shared` is bound.
    shared_bound: bool,
    /// The buffer pair last bound.
    source: Option<MeshSource>,
    /// The pipeline last bound.
    pipeline: Option<vk::Pipeline>,
    /// The texture set last bound to set 1.
    texture_set: Option<vk::DescriptorSet>,
    /// The push constants last pushed, if they still hold.
    pushed: Option<GeometryPush>,
}

impl<'a> GeometryRecorder<'a> {
    /// A recorder with nothing bound yet: the first draw binds everything.
    pub fn new(
        device: &'a ash::Device,
        cb: vk::CommandBuffer,
        layout: vk::PipelineLayout,
        shared: SharedBindings,
    ) -> Self {
        Self {
            device,
            cb,
            layout,
            shared,
            shared_bound: false,
            source: None,
            pipeline: None,
            texture_set: None,
            pushed: None,
        }
    }

    /// Record `draw` through `pipeline`.
    pub fn draw(&mut self, pipeline: vk::Pipeline, draw: &GeometryDraw) {
        self.bind_shared();
        self.bind_mesh_source(draw.draw.source);
        self.bind_pipeline(pipeline);
        self.bind_texture_set(draw.texture_set);
        self.push(draw.push());
        self.draw_indexed(draw);
    }

    /// Draw `draw` again through `pipeline` in a flat `colour`: the debug
    /// wireframe over a mesh just drawn.
    pub fn overdraw(&mut self, pipeline: vk::Pipeline, colour: [f32; 4], draw: &GeometryDraw) {
        self.draw(pipeline, draw);
        unsafe {
            self.device.cmd_push_constants(
                self.cb,
                self.layout,
                PUSH_CONSTANT_STAGES,
                GeometryPush::COLOUR_OVERRIDE_OFFSET,
                &bytes_of(&colour),
            );
        }
        self.pushed = self.pushed.map(|pushed| GeometryPush {
            colour_override: colour,
            ..pushed
        });
        self.draw_indexed(draw);
    }

    /// Something else recorded into the command buffer: forget what is bound.
    pub fn interrupted(&mut self) {
        self.shared_bound = false;
        self.source = None;
        self.pipeline = None;
        self.texture_set = None;
        self.pushed = None;
    }

    fn bind_shared(&mut self) {
        if self.shared_bound {
            return;
        }
        let shared = &self.shared;
        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: shared.extent.width as f32,
            height: shared.extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };
        unsafe {
            self.device.cmd_set_viewport(self.cb, 0, &[viewport]);
            self.device
                .cmd_set_scissor(self.cb, 0, &[shared.extent.into()]);
            self.device.cmd_bind_descriptor_sets(
                self.cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.layout,
                0,
                &[shared.scene_set],
                &[],
            );
        }
        self.shared_bound = true;
    }

    fn bind_mesh_source(&mut self, source: MeshSource) {
        if self.source == Some(source) {
            return;
        }
        self.shared.meshes.of(source).bind(self.device, self.cb);
        self.source = Some(source);
    }

    fn bind_pipeline(&mut self, pipeline: vk::Pipeline) {
        if self.pipeline == Some(pipeline) {
            return;
        }
        unsafe {
            self.device
                .cmd_bind_pipeline(self.cb, vk::PipelineBindPoint::GRAPHICS, pipeline);
        }
        self.pipeline = Some(pipeline);
    }

    fn bind_texture_set(&mut self, texture_set: vk::DescriptorSet) {
        if self.texture_set == Some(texture_set) {
            return;
        }
        unsafe {
            self.device.cmd_bind_descriptor_sets(
                self.cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.layout,
                1,
                &[texture_set],
                &[],
            );
        }
        self.texture_set = Some(texture_set);
    }

    fn push(&mut self, push: GeometryPush) {
        if self.pushed == Some(push) {
            return;
        }
        unsafe {
            self.device.cmd_push_constants(
                self.cb,
                self.layout,
                PUSH_CONSTANT_STAGES,
                0,
                push.as_bytes(),
            );
        }
        self.pushed = Some(push);
    }

    fn draw_indexed(&self, draw: &GeometryDraw) {
        unsafe {
            self.device.cmd_draw_indexed(
                self.cb,
                draw.draw.index_count,
                1,
                draw.draw.first_index,
                draw.draw.vertex_offset,
                0,
            );
        }
    }
}

fn bytes_of(colour: &[f32; 4]) -> [u8; 16] {
    let mut bytes = [0u8; 16];
    for (chunk, value) in bytes.chunks_exact_mut(4).zip(colour) {
        chunk.copy_from_slice(&value.to_ne_bytes());
    }
    bytes
}
