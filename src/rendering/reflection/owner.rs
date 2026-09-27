/// Names the object a probe serves, stably from one frame to the next.
///
/// A probe's cube map is refreshed a few faces at a time, so it has to be the
/// same probe next frame. The renderer has no idea what an entity is, so the
/// caller names the object with whatever identity it already has; the ECS
/// packs an entity's index and generation, so a reused index is a different
/// owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ProbeOwner(pub u64);
