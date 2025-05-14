/// Trait for types that need explicit cleanup
pub trait Destroy {
    fn destroy(&self);
}
