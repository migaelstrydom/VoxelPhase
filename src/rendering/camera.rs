use nalgebra::{Matrix4, Point3, Vector3};

#[derive(Debug)]
pub struct Camera {
    pub position: Point3<f32>,
    pub target: Point3<f32>,
    pub up: Vector3<f32>,

    // Frustum parameters
    pub fov_y_radians: f32,
    pub aspect_ratio: f32,
    pub znear: f32,
    pub zfar: f32,
}

impl Camera {
    pub fn new(
        position: Point3<f32>,
        target: Point3<f32>,
        up: Vector3<f32>,
        fov_y_radians: f32,
        aspect_ratio: f32,
        znear: f32,
        zfar: f32,
    ) -> Self {
        Self {
            position,
            target,
            up,
            fov_y_radians,
            aspect_ratio,
            znear,
            zfar,
        }
    }

    pub fn get_view_matrix(&self) -> Matrix4<f32> {
        Matrix4::look_at_rh(&self.position, &self.target, &self.up)
    }

    pub fn get_projection_matrix(&self) -> Matrix4<f32> {
        Matrix4::new_perspective(self.aspect_ratio, self.fov_y_radians, self.znear, self.zfar)
    }
}

// Default camera settings
impl Default for Camera {
    fn default() -> Self {
        Self {
            position: Point3::new(0.0, 0.0, 3.0),       // Default position
            target: Point3::new(0.0, 0.0, 0.0),         // Looking at origin
            up: Vector3::y(),                           // Standard up vector
            fov_y_radians: std::f32::consts::FRAC_PI_4, // 45 degrees FOV
            aspect_ratio: 16.0 / 9.0,                   // Common aspect ratio
            znear: 0.1,
            zfar: 100.0,
        }
    }
}
