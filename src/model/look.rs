//! How the parts of a body look: a mesh per collider, in the collider's own
//! frame, drawn wherever the collider is.
//!
//! ```text
//!   PartLooks { collider ──▶ Arc<Look> }      on the piece's entity
//!        │
//!        ▼  drawn(world, body): each look at its collider's offset,
//!   Model    merged by material
//! ```
//!
//! A look is made once, when its collider is (at spawn, or by whatever cut
//! the collider out of a larger one), with its texture coordinates baked in.
//! It never changes after that, whichever body the collider ends up on, so
//! a piece that comes off an object looks exactly as it did on it.

use std::sync::Arc;

use rustc_hash::FxHashMap;
use specs::{Component, VecStorage};

use super::assemble::{assemble_by_material, PlacedMesh};
use super::parts::{MeshPrimitive, Model};
use crate::physics::{ColliderHandle, PhysicsWorld, RigidBodyHandle};
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// What one collider looks like: mesh primitives in the collider's frame,
/// each with its own material and its texture coordinates baked.
#[derive(Clone, Default)]
pub struct Look {
    /// The meshes, in the collider's frame.
    pub primitives: Vec<MeshPrimitive>,
}

impl Look {
    /// A look of one mesh in one material.
    pub fn single(vertices: Vec<Vertex>, indices: Vec<u32>, material: MaterialId) -> Self {
        Self {
            primitives: vec![MeshPrimitive {
                vertices,
                indices,
                material,
            }],
        }
    }
}

/// How each drawn collider of a piece looks, keyed by collider handle.
///
/// A component on the piece's entity: a piece's share of it is a filter,
/// and it goes when the entity goes. A collider with no entry is not drawn.
#[derive(Component, Clone, Default)]
#[storage(VecStorage)]
pub struct PartLooks {
    /// The look of each drawn collider.
    looks: FxHashMap<ColliderHandle, Arc<Look>>,
    /// Whether the looks changed since the entity's model was last drawn
    /// from them.
    changed: bool,
}

impl PartLooks {
    /// Looks for these colliders, not yet drawn.
    pub fn new(looks: impl IntoIterator<Item = (ColliderHandle, Arc<Look>)>) -> Self {
        Self {
            looks: looks.into_iter().collect(),
            changed: true,
        }
    }

    /// Give `collider` this look.
    pub fn insert(&mut self, collider: ColliderHandle, look: Arc<Look>) {
        self.looks.insert(collider, look);
        self.changed = true;
    }

    /// Forget `collider`'s look.
    pub fn remove(&mut self, collider: ColliderHandle) {
        if self.looks.remove(&collider).is_some() {
            self.changed = true;
        }
    }

    /// The look of `collider`, if it is drawn.
    pub fn get(&self, collider: ColliderHandle) -> Option<&Arc<Look>> {
        self.looks.get(&collider)
    }

    /// How many colliders are drawn.
    pub fn len(&self) -> usize {
        self.looks.len()
    }

    pub fn is_empty(&self) -> bool {
        self.looks.is_empty()
    }

    /// The looks of `colliders` alone: a piece's share.
    pub fn share(&self, colliders: &[ColliderHandle]) -> Self {
        Self::new(
            colliders
                .iter()
                .filter_map(|c| Some((*c, self.looks.get(c)?.clone()))),
        )
    }

    /// Keep the looks of `colliders` alone.
    pub fn retain(&mut self, colliders: &[ColliderHandle]) {
        let before = self.looks.len();
        self.looks
            .retain(|collider, _| colliders.contains(collider));
        self.changed |= self.looks.len() != before;
    }

    /// Mark the looks for drawing again, as a body that re-centred needs.
    pub fn mark_changed(&mut self) {
        self.changed = true;
    }

    /// Whether the looks changed since they were last drawn, clearing it.
    pub fn take_changed(&mut self) -> bool {
        std::mem::take(&mut self.changed)
    }

    /// The model of `body` as its colliders stand now: each look placed at
    /// its collider's offset, in the body's collider order, merged by
    /// material. `None` if nothing of the body is drawn.
    pub fn drawn(&self, world: &PhysicsWorld, body: RigidBodyHandle) -> Option<Arc<Model>> {
        let body = world.body(body)?;
        let pieces: Vec<PlacedMesh> = body
            .colliders()
            .iter()
            .filter_map(|handle| {
                let look = self.looks.get(handle)?;
                let offset = world.collider(*handle)?.offset();
                Some(look.primitives.iter().map(move |primitive| PlacedMesh {
                    vertices: primitive.vertices.clone(),
                    indices: primitive.indices.clone(),
                    offset: offset.translation.vector,
                    rotation: offset.rotation,
                    material: primitive.material,
                }))
            })
            .flatten()
            .collect();
        (!pieces.is_empty()).then(|| assemble_by_material(pieces))
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, UnitQuaternion, Vector3, Vector4};

    use super::*;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    fn vertex(x: f32) -> Vertex {
        Vertex {
            pos: Vector3::new(x, 0.0, 0.0),
            color: Vector4::repeat(1.0),
            tex_coords: nalgebra::Vector2::zeros(),
            normal: Vector3::y(),
            ao: 1.0,
        }
    }

    fn look(material: u32) -> Arc<Look> {
        Arc::new(Look::single(
            vec![vertex(0.0), vertex(0.1), vertex(0.2)],
            vec![0, 1, 2],
            MaterialId(material),
        ))
    }

    /// A share is the looks of the piece's colliders, the same looks, and
    /// nothing of the colliders that stayed.
    #[test]
    fn a_share_is_a_filter_of_the_same_looks() {
        let [a, b, c] = [0, 1, 2].map(ColliderHandle::for_test);
        let mut looks = PartLooks::new([(a, look(1)), (b, look(2)), (c, look(1))]);
        let share = looks.share(&[b, c]);
        looks.retain(&[a]);

        assert_eq!(share.len(), 2);
        assert!(Arc::ptr_eq(share.get(b).unwrap(), looks_of(&share, b)));
        assert!(share.get(a).is_none());
        assert_eq!(looks.len(), 1);
        assert!(looks.get(b).is_none());
        assert!(looks.take_changed(), "losing looks is a change to draw");
    }

    fn looks_of(looks: &PartLooks, collider: ColliderHandle) -> &Arc<Look> {
        looks.get(collider).unwrap()
    }

    /// A capsule is drawn like any other collider: its look is placed where
    /// the capsule is, turned as it is.
    #[test]
    fn a_capsule_child_is_drawn_where_it_is() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        let quarter =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        let capsule = world
            .attach_collider(
                body,
                ColliderDesc::capsule(0.5, 0.2)
                    .offset_translation(Vector3::new(1.0, 0.0, 0.0))
                    .offset_rotation(quarter),
            )
            .unwrap();
        let bare = world
            .attach_collider(body, ColliderDesc::sphere(0.3))
            .unwrap();
        let looks = PartLooks::new([(capsule, look(4))]);

        let model = looks.drawn(&world, body).expect("the capsule is drawn");
        let primitive = &model.parts[0].primitives[0];
        assert_eq!(model.parts[0].primitives.len(), 1);
        assert_eq!(primitive.material, MaterialId(4));
        let ends: Vec<_> = primitive.vertices.iter().map(|v| v.pos).collect();
        assert!((ends[0] - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-5);
        assert!(
            (ends[2] - Vector3::new(1.0, 0.2, 0.0)).norm() < 1e-5,
            "the look was not turned with its collider: {:?}",
            ends[2]
        );
        let _ = bare;
    }

    /// Looks of one material are drawn as one primitive, whatever their
    /// colliders.
    #[test]
    fn looks_are_merged_by_material() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic());
        let colliders: Vec<_> = (0..3)
            .map(|i| {
                world
                    .attach_collider(
                        body,
                        ColliderDesc::sphere(0.2)
                            .offset_translation(Vector3::new(i as f32, 0.0, 0.0)),
                    )
                    .unwrap()
            })
            .collect();
        let looks = PartLooks::new([
            (colliders[0], look(1)),
            (colliders[1], look(2)),
            (colliders[2], look(1)),
        ]);
        let model = looks.drawn(&world, body).unwrap();
        assert_eq!(model.parts[0].primitives.len(), 2);
        assert_eq!(model.parts[0].primitives[0].vertices.len(), 6);
    }
}
