mod beach_ball;
mod box_entity;
mod camera;
mod player;

pub use beach_ball::spawn_beach_ball;
pub use box_entity::{
    create_box_material_for_style, create_box_materials, spawn_box, spawn_house, BoxPhysics,
};
pub use camera::spawn_camera;
pub use player::spawn_player;
