use bytemuck::{Pod, Zeroable};
use winit::event::VirtualKeyCode;
use winit_input_helper::WinitInputHelper;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Vertex {
    pub pos: [f32; 2],
    pub col: [f32; 3],
}
unsafe impl Pod for Vertex {}
unsafe impl Zeroable for Vertex {}
impl Vertex {
    pub fn desc<'a>() -> wgpu::VertexBufferDescriptor<'a> {
        wgpu::VertexBufferDescriptor {
            stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::InputStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttributeDescriptor {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float2,
                },
                wgpu::VertexAttributeDescriptor {
                    offset: std::mem::size_of::<[f32; 2]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float3,
                },
            ],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FlowVertex {
    pub pos: [f32; 2],
}
unsafe impl Pod for FlowVertex {}
unsafe impl Zeroable for FlowVertex {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FlowParticle {
    pub pos: [f32; 2],
    pub vel: [f32; 2],
}
unsafe impl Pod for FlowParticle {}
unsafe impl Zeroable for FlowParticle {}

// Top-down 2D camera
#[derive(Debug)]
pub struct Camera {
    pub changed: bool,
    pub slow_spd: f32,
    pub fast_spd_fac: f32,
    pub pos: glam::Vec2,
    pub scl: f32,
    pub asp: f32,
}
impl Camera {
    // Might need to do something more here in the future
    // ([x_pos, y_pos], [x_scl, y_scl])
    pub fn build_view_proj(&self) -> ([f32; 2], [f32; 2]) {
        if self.asp > 1.0 {
            (self.pos.into(), [(1.0 / self.asp) * self.scl, self.scl])
        } else {
            (self.pos.into(), [self.scl, self.scl * self.asp])
        }
    }

    // Camera controller
    pub fn update(&mut self, input: &WinitInputHelper, delta: f32) {
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
                self.changed = true;
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
                self.changed = true;
                let zoom_fac = self.scl.log2() + (scroll_diff * 0.25);
                self.scl = (2f32).powf(zoom_fac);
            }
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ViewUniforms {
    pub view_pos: [f32; 2],
    pub view_scl: [f32; 2],
}
unsafe impl Pod for ViewUniforms {}
unsafe impl Zeroable for ViewUniforms {}
impl ViewUniforms {
    pub fn default() -> Self {
        Self {
            view_pos: [0.0, 0.0],
            view_scl: [1.0, 1.0],
        }
    }

    pub fn update_view_proj(&mut self, camera: &Camera) {
        (self.view_pos, self.view_scl) = camera.build_view_proj();
    }
}
