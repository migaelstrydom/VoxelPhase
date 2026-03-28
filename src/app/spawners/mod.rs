mod beach_ball;
mod box_entity;
mod camera;
mod capsule_entity;
mod player;
mod table_entity;

pub use beach_ball::spawn_beach_ball;
pub use box_entity::{
    create_box_material_for_style, create_box_materials, spawn_box, spawn_house, BoxPhysics,
};
pub use camera::spawn_camera;
pub use capsule_entity::{create_capsule_material, spawn_capsule, CapsulePhysics};
pub use player::spawn_player;
pub use table_entity::{
    create_table_material, spawn_table, TableDimensions, TablePhysics,
};
