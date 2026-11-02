/// One timed piece of the CPU's work on a rendered frame.
///
/// ```text
///   begin_frame ─┬─ FenceWait   (the GPU finishing the previous frame)
///                └─ Acquire     (the next output image)
///   record ──────┬─ Setup       (scene, lights, sky)
///                ├─ Terrain · Models · Rigs · DebugShapes
///                │              (upload, and hold each draw back)
///                ├─ Particles   (sort, build billboards, upload)
///                ├─ SceneDraws  (record the held draws: opaque, then
///                │               sorted blended draws + particles)
///                └─ Water · Fire · Overlay
///   end_frame ───┬─ Probes      (record the reflection probe faces)
///                ├─ Submit      (record shadows, close the frame, present)
///                └─ QueueSubmit (hand the frame to the GPU queue)
/// ```
///
/// `FenceWait` is time the CPU spent idle on the GPU, not work of its own: a
/// frame that is GPU-bound shows up there, and nowhere else on this side.
///
/// `Acquire` and `QueueSubmit` are where the frame waits for vsync. Which of
/// the two depends on the driver: MoltenVK's acquire returns at once and its
/// queue submit blocks until a drawable is free to render into. The driver's
/// own submit work rides along in `QueueSubmit` and is counted as waiting; an
/// offscreen run (`render_perf`), which has no vsync, shows it on its own.
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
    SceneDraws,
    Water,
    Fire,
    Overlay,
    Probes,
    Submit,
    QueueSubmit,
}

impl RenderStage {
    pub const COUNT: usize = 15;

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
        RenderStage::SceneDraws,
        RenderStage::Water,
        RenderStage::Fire,
        RenderStage::Overlay,
        RenderStage::Probes,
        RenderStage::Submit,
        RenderStage::QueueSubmit,
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
            RenderStage::SceneDraws => "scene_draws",
            RenderStage::Water => "water",
            RenderStage::Fire => "fire",
            RenderStage::Overlay => "overlay",
            RenderStage::Probes => "probes",
            RenderStage::Submit => "submit",
            RenderStage::QueueSubmit => "queue_submit",
        }
    }

    /// Whether the stage is the CPU waiting rather than working.
    pub fn is_wait(self) -> bool {
        matches!(
            self,
            RenderStage::FenceWait | RenderStage::Acquire | RenderStage::QueueSubmit
        )
    }

    /// Position in [`RenderStage::ALL`], and in any per-stage array.
    pub(super) fn slot(self) -> usize {
        self as usize
    }
}
