/// How the reflection probes are sized and refreshed.
#[derive(Clone, Copy, Debug)]
pub struct ProbeConfig {
    /// How many probes can be live at once. Fixed when the renderer is built.
    /// Objects beyond this many, counted from the camera outwards, reflect
    /// the sky.
    pub capacity: u32,
    /// Faces refreshed per frame, shared round-robin between the live probes.
    /// A probe's six faces are all captured on the frame it is admitted,
    /// whatever this says.
    ///
    /// The CPU pays per face for recording its draws, so this is the dial for
    /// trading refresh rate against frame time: with `n` live probes, each
    /// face is refreshed every `6n / faces_per_frame` frames.
    pub faces_per_frame: u32,
    /// How far a probe sees other objects, in metres. The terrain is always
    /// seen, however far it extends.
    pub reach: f32,
    /// The near clip distance of each face, in metres.
    pub near: f32,
    /// The far clip distance of each face, in metres. Anything beyond it is
    /// reflected as sky.
    pub far: f32,
}

impl Default for ProbeConfig {
    fn default() -> Self {
        Self {
            capacity: 16,
            faces_per_frame: 6,
            reach: 20.0,
            near: 0.05,
            far: 250.0,
        }
    }
}
