//! Scenario-based visual testing, the counterpart to the physics bench.
//!
//! A `VisualScene` describes a handful of shots — geometry, camera, lighting —
//! and the `VisualBench` renders each one through the real Vulkan pipeline into
//! an image. Nothing here re-implements any shading: the whole point is that
//! what comes out is what the game would draw.
//!
//! ```text
//!   VisualScene ──shots──► VisualBench ──► RgbaImage ──► PNG / contact sheet
//!                            (offscreen Renderer)
//! ```
//!
//! Determinism is a design requirement, not a nicety. Every shot fixes its
//! camera, sun, exposure and geometry, so two runs of the same scene differ
//! only where the renderer differs — which is what makes before/after
//! comparison meaningful.

pub mod bench;
pub mod pool;
pub mod scene;
pub mod scenes;
pub mod sheet;

pub use bench::VisualBench;
pub use pool::ScenePool;
pub use scene::{SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene};
pub use sheet::contact_sheet;
