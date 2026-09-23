/// One timed piece of the CPU's work on a rendered frame.
///
/// ```text
///   begin_frame ─┬─ FenceWait   (the GPU finishing the previous frame)
///                └─ Acquire     (the next output image)
///   record ──────┬─ Setup       (scene, lights, sky)
///                ├─ Terrain · Models · Rigs · DebugShapes
///                ├─ Particles   (sort, build billboards, upload)
///                ├─ TransparencyFlush (sorted blended draws + particles)
///                └─ Water · Fire · Overlay
///   end_frame ───── Submit      (submit and present)
/// ```
///
/// `FenceWait` is time the CPU spent idle on the GPU, not work of its own: a
/// frame that is GPU-bound shows up there, and nowhere else on this side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RenderStage {
    FenceWait,
    Acquire,
    Setup,
    Terrain,
    Models,
    Rigs,
    DebugShapes,
    Particles,
    TransparencyFlush,
    Water,
    Fire,
    Overlay,
    Submit,
}

impl RenderStage {
    pub const COUNT: usize = 13;

    /// Every stage, in the order a frame runs them.
    pub const ALL: [RenderStage; Self::COUNT] = [
        RenderStage::FenceWait,
        RenderStage::Acquire,
        RenderStage::Setup,
        RenderStage::Terrain,
        RenderStage::Models,
        RenderStage::Rigs,
        RenderStage::DebugShapes,
        RenderStage::Particles,
        RenderStage::TransparencyFlush,
        RenderStage::Water,
        RenderStage::Fire,
        RenderStage::Overlay,
        RenderStage::Submit,
    ];

    pub fn label(self) -> &'static str {
        match self {
            RenderStage::FenceWait => "fence_wait",
            RenderStage::Acquire => "acquire",
            RenderStage::Setup => "setup",
            RenderStage::Terrain => "terrain",
            RenderStage::Models => "models",
            RenderStage::Rigs => "rigs",
            RenderStage::DebugShapes => "debug_shapes",
            RenderStage::Particles => "particles",
            RenderStage::TransparencyFlush => "transparency_flush",
            RenderStage::Water => "water",
            RenderStage::Fire => "fire",
            RenderStage::Overlay => "overlay",
            RenderStage::Submit => "submit",
        }
    }

    /// Whether the stage is the CPU waiting rather than working.
    pub fn is_wait(self) -> bool {
        matches!(self, RenderStage::FenceWait | RenderStage::Acquire)
    }

    /// Position in [`RenderStage::ALL`], and in any per-stage array.
    pub(super) fn slot(self) -> usize {
        self as usize
    }
}
