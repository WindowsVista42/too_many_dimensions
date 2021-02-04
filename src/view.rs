use bytemuck::{Pod, Zeroable};
use winit::event::VirtualKeyCode;
use winit_input_helper::WinitInputHelper;

#[derive(Debug)]
/// Top-down 2D camera
pub struct Camera {
    // Movement
    pub slow_spd: f32,
    pub fast_spd_fac: f32,

    // Camera
    pub pos: glam::Vec2,
    pub scl: f32,
    pub asp: f32,
}
impl Camera {
    /// Camera controller function
    pub fn update(&mut self, input: &WinitInputHelper, delta: f32) -> bool {
        let mut changed = false;
        // Moving around the camera
        {
            let mut cam_pos_delta = glam::Vec2::zero();
            if input.key_held(VirtualKeyCode::A) {
                cam_pos_delta.x -= self.slow_spd;
            }
            if input.key_held(VirtualKeyCode::D) {
                cam_pos_delta.x += self.slow_spd;
            }
            if input.key_held(VirtualKeyCode::S) {
                cam_pos_delta.y -= self.slow_spd;
            }
            if input.key_held(VirtualKeyCode::W) {
                cam_pos_delta.y += self.slow_spd;
            }
            if cam_pos_delta != glam::Vec2::zero() {
                changed = true;
                cam_pos_delta = cam_pos_delta.normalize() * delta;

                if input.key_held(VirtualKeyCode::LShift) {
                    cam_pos_delta *= self.fast_spd_fac;
                }

                self.pos += cam_pos_delta;
            }
        }

        // Zooming the camera
        {
            let scroll_diff = input.scroll_diff();
            if scroll_diff != 0.0 {
                changed = true;
                let zoom_fac = self.scl.log2() + (scroll_diff * 0.25);
                self.scl = (2f32).powf(zoom_fac);
            }
        }

        changed
    }
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
