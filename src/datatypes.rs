use bytemuck::{Pod, Zeroable};

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
    pub pos: glam::Vec2,
    pub scl: f32,
    pub asp: f32,
}
impl Camera {
    // Might need to do something more here in the future
    pub fn build_view_proj(&self) -> [[f32; 2]; 2] {
        // [x_pos, y_pos], [x_scl, y_scl]
        if self.asp > 1.0 {
            [self.pos.into(), [(1.0 / self.asp) * self.scl, self.scl]]
        } else {
            [self.pos.into(), [self.scl, self.scl * self.asp]]
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
        let data = camera.build_view_proj();
        self.view_pos = data[0];
        self.view_scl = data[1];
    }
}
