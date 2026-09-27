/// What a surface's reflection shows.
///
/// A material's choice, like its transparency: whether a glossy surface
/// reads as glossy depends on having something to reflect, and only the
/// material knows how glossy it is. Where the probe sits is the object's
/// business and is worked out when it is drawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reflects {
    /// The analytic sky, and a flat ground colour below the horizon. Costs
    /// nothing, and is all a rough or matte surface can show anyway.
    #[default]
    Sky,
    /// What is actually around the object — ground, structures, other
    /// objects — from a small cube map captured at its centre, over the sky
    /// wherever that map saw nothing. For metals, polished stone, ice and
    /// glass: surfaces whose colour is largely what they reflect.
    ///
    /// Probes are a budget. An object that asks for one when all are in use
    /// by objects nearer the camera shows the sky, as `Sky` would.
    Surroundings,
}
