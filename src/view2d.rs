use bytemuck::{Pod, Zeroable};

#[derive(Debug)]
/// Top-down 2D camera
pub struct Camera {
    // Movement
    pub slow_spd_fac: f32,
    pub spd: f32,
    pub fast_spd_fac: f32,
    // Camera
    pub pos: glam::Vec2,
    pub scl: f32,
    pub asp: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// Camera uniforms including view projection
// Might need to do more with this in the future
pub struct Uniforms {
    pub view_pos: [f32; 2],
    pub view_scl: [f32; 2],
}
unsafe impl Pod for Uniforms {}
unsafe impl Zeroable for Uniforms {}
impl Uniforms {
    pub fn default() -> Self {
        Self {
            view_pos: [0.0, 0.0],
            view_scl: [1.0, 1.0],
        }
    }

    /// Update the view projection based on the camera
    pub fn update_view_proj(&mut self, camera: &Camera) {
        if camera.asp > 1.0 {
            self.view_pos = camera.pos.into();
            self.view_scl = [(1.0 / camera.asp) * camera.scl, camera.scl];
        } else {
            self.view_pos = camera.pos.into();
            self.view_scl = [camera.scl, camera.scl * camera.asp];
        };
    }
}
