mod datatypes;
use datatypes::*;

use futures::executor::block_on;
use wgpu::util::DeviceExt;
use wgpu::ProgrammableStageDescriptor;
use winit::event::{Event, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::{Window, WindowBuilder};
use winit_input_helper::WinitInputHelper;

const VERTICES: &[Vertex] = &[
    Vertex {
        pos: [-0.086824, 0.492403],
        col: [0.7, 0.2, 0.5],
    },
    Vertex {
        pos: [-0.495134, 0.069586],
        col: [0.1, 0.1, 0.5],
    },
    Vertex {
        pos: [-0.219185, -0.449397],
        col: [0.2, 0.4, 0.9],
    },
    Vertex {
        pos: [0.359669, -0.347329],
        col: [0.9, 0.1, 0.2],
    },
    Vertex {
        pos: [0.441473, 0.234735],
        col: [0.2, 0.8, 0.2],
    },
];

const INDICES: &[u16] = &[0, 1, 4, 1, 2, 4, 2, 3, 4];

const FLOW_SHAPE_VERTICES: [FlowVertex; 4] = [
    FlowVertex {
        pos: [-0.01, -0.01],
    },
    FlowVertex { pos: [-0.01, 0.01] },
    FlowVertex { pos: [0.01, 0.01] },
    FlowVertex { pos: [0.01, -0.01] },
];

const FLOW_SHAPE_INDICES: [u16; 6] = [0, 1, 2, 0, 2, 3];

const NUM_FLOW: usize = 5_000;

fn main() {
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new()
        .with_title("Too Many Dimensions")
        //.with_fullscreen(Some(winit::window::Fullscreen::Borderless(None)))
        .build(&event_loop)
        .unwrap();
    let mut state = block_on(State::new(&window));

    event_loop.run(move |event, _, control_flow| {
        state.update(&event);
        if state.quit {
            *control_flow = ControlFlow::Exit;
            return;
        }

        match event {
            Event::RedrawRequested(_) => {
                state.render();
            }
            Event::WindowEvent {
                event: WindowEvent::Resized(physical_size),
                ..
            } => {
                state.resize(physical_size);
            }
            Event::MainEventsCleared => {
                window.request_redraw();
            }
            _ => {}
        }
    });
}

struct State {
    // INPUT
    input: WinitInputHelper,

    // FLAGS
    quit: bool,
    pause: bool,
    dbuf: usize,

    // INSTANCE
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    sc_desc: wgpu::SwapChainDescriptor,
    swap_chain: wgpu::SwapChain,

    // UNIFORMS
    camera: Camera,
    view_uniforms: ViewUniforms,
    view_uniform_buffer: wgpu::Buffer,
    view_uniform_bind_group: wgpu::BindGroup,

    // TIME
    last_tick: std::time::Instant,
    delta: f32,

    // BACKGROUND
    render_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,

    // FLOW
    flow_compute_pipeline: wgpu::ComputePipeline,
    flow_render_pipeline: wgpu::RenderPipeline,
    flow_sim_buffer: wgpu::Buffer,
    flow_bind_groups: Vec<wgpu::BindGroup>, // Alternating buffer
    flow_buffers: Vec<wgpu::Buffer>,        // Alternating buffer
    flow_vertices_buffer: wgpu::Buffer,
    flow_indices_buffer: wgpu::Buffer,
    flow_work_group_count: u32,
    flow_num_indices: u32,

    // WINDOW
    size: winit::dpi::PhysicalSize<u32>,
    frame_num: usize,
}

impl State {
    async fn new(window: &Window) -> Self {
        // INPUT
        let input = WinitInputHelper::new();

        // FLAGS
        let quit = false;
        let pause = false;
        let dbuf = 0;

        // INSTANCE
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::BackendBit::VULKAN);
        let surface = unsafe { instance.create_surface(window) };
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
            })
            .await
            .unwrap();

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    features: wgpu::Features::default(),
                    limits: wgpu::Limits::default(),
                    shader_validation: true,
                },
                None,
            )
            .await
            .unwrap();

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };

        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        // SHADER LOADING
        let vs_module = device.create_shader_module(wgpu::include_spirv!("../spirv/vertex.vert.spv"));
        let fs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/fragment.frag.spv"));
        let flow_cs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flow.comp.spv"));
        let flow_vs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flowvert.vert.spv"));
        let flow_fs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flowfrag.frag.spv"));

        // UNIFORMS
        let camera = Camera {
            pos: glam::Vec2::zero(),
            scl: 1.0,
            asp: sc_desc.width as f32 / sc_desc.height as f32,
        };

        let mut view_uniforms = ViewUniforms::default();
        view_uniforms.update_view_proj(&camera);

        let delta = 0.0;

        let view_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("UNIFORM BUFFER"),
            contents: bytemuck::cast_slice(&[view_uniforms]),
            usage: wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        });

        let view_uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("UNIFORM BIND GROUP LAYOUT"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStage::VERTEX,
                    ty: wgpu::BindingType::UniformBuffer {
                        dynamic: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let view_uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("UNIFORM BIND GROUP"),
            layout: &view_uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(view_uniform_buffer.slice(..)),
            }],
        });

        // BACKGROUND
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("RENDER PIPELINE LAYOUT"),
                bind_group_layouts: &[&view_uniform_bind_group_layout],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex_stage: ProgrammableStageDescriptor {
                module: &vs_module,
                entry_point: "main",
            },
            fragment_stage: Some(ProgrammableStageDescriptor {
                module: &fs_module,
                entry_point: "main",
            }),
            rasterization_state: Some(wgpu::RasterizationStateDescriptor {
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: wgpu::CullMode::None,
                clamp_depth: false,
                depth_bias: 0,
                depth_bias_slope_scale: 0.0,
                depth_bias_clamp: 0.0,
            }),
            color_states: &[wgpu::ColorStateDescriptor {
                format: sc_desc.format,
                color_blend: wgpu::BlendDescriptor::REPLACE,
                alpha_blend: wgpu::BlendDescriptor::REPLACE,
                write_mask: wgpu::ColorWrite::ALL,
            }],
            primitive_topology: wgpu::PrimitiveTopology::TriangleList,
            depth_stencil_state: None,
            vertex_state: wgpu::VertexStateDescriptor {
                index_format: wgpu::IndexFormat::Uint16,
                vertex_buffers: &[Vertex::desc()],
            },
            sample_count: 1,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("VERTEX BUFFER"),
            contents: &bytemuck::cast_slice(&VERTICES),
            usage: wgpu::BufferUsage::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("INDEX BUFFER"),
            contents: &bytemuck::cast_slice(&INDICES),
            usage: wgpu::BufferUsage::INDEX,
        });

        // FLOW
        let flow_sim_data = [0.0f32].to_vec();
        let flow_sim_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("FLOW SIM DATA"),
            contents: bytemuck::cast_slice(&flow_sim_data),
            usage: wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        });

        let flow_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: None,
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::UniformBuffer {
                            dynamic: false,
                            min_binding_size: wgpu::BufferSize::new(flow_sim_data.len() as _),
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::StorageBuffer {
                            dynamic: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (NUM_FLOW * std::mem::size_of::<FlowParticle>()) as _,
                            ),
                            readonly: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::StorageBuffer {
                            dynamic: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (NUM_FLOW * std::mem::size_of::<FlowParticle>()) as _,
                            ),
                            readonly: false,
                        },
                        count: None,
                    },
                ],
            });

        let flow_compute_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("FLOW COMPUTE LAYOUT"),
                bind_group_layouts: &[&flow_bind_group_layout],
                push_constant_ranges: &[],
            });

        let flow_compute_pipeline =
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("FLOW COMPUTE PIPELINE"),
                layout: Some(&flow_compute_pipeline_layout),
                compute_stage: ProgrammableStageDescriptor {
                    module: &flow_cs_module,
                    entry_point: "main",
                },
            });

        let flow_render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("FLOW RENDER LAYOUT"),
                bind_group_layouts: &[&view_uniform_bind_group_layout],
                push_constant_ranges: &[],
            });

        let flow_render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("FLOW RENDER PIPELINE"),
            layout: Some(&flow_render_pipeline_layout),
            vertex_stage: wgpu::ProgrammableStageDescriptor {
                module: &flow_vs_module,
                entry_point: "main",
            },
            fragment_stage: Some(wgpu::ProgrammableStageDescriptor {
                module: &flow_fs_module,
                entry_point: "main",
            }),
            rasterization_state: Some(wgpu::RasterizationStateDescriptor {
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: wgpu::CullMode::None,
                ..Default::default()
            }),
            primitive_topology: wgpu::PrimitiveTopology::TriangleList,
            color_states: &[sc_desc.format.into()],
            depth_stencil_state: None,
            vertex_state: wgpu::VertexStateDescriptor {
                index_format: wgpu::IndexFormat::Uint16,
                vertex_buffers: &[
                    wgpu::VertexBufferDescriptor {
                        stride: 4 * 4,
                        step_mode: wgpu::InputStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![0 => Float2, 1 => Float2],
                    },
                    wgpu::VertexBufferDescriptor {
                        stride: 4 * 2,
                        step_mode: wgpu::InputStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![2 => Float2],
                    },
                ],
            },
            sample_count: 1,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });

        let flow_vertices_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("VERTEX BUFFER"),
            contents: bytemuck::bytes_of(&FLOW_SHAPE_VERTICES),
            usage: wgpu::BufferUsage::VERTEX | wgpu::BufferUsage::COPY_DST,
        });
        let flow_indices_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("INDEX BUFFER"),
            contents: bytemuck::bytes_of(&FLOW_SHAPE_INDICES),
            usage: wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
        });

        let mut initial_flow_data = vec![
            FlowParticle {
                pos: glam::Vec2::zero().into(),
                vel: glam::Vec2::zero().into(),
            };
            (4 * NUM_FLOW) as usize
        ];
        for p in initial_flow_data.iter_mut() {
            p.pos[0] = 2.0 * (rand::random::<f32>() - 0.5); //posx
            p.pos[1] = 2.0 * (rand::random::<f32>() - 0.5); //posy
            p.vel[0] = 2.0 * (rand::random::<f32>() - 0.5) * 0.1; //velx
            p.vel[1] = 2.0 * (rand::random::<f32>() - 0.5) * 0.1; //vely
        }

        let mut flow_buffers = Vec::<wgpu::Buffer>::with_capacity(2);
        let mut flow_bind_groups = Vec::<wgpu::BindGroup>::with_capacity(2);
        for i in 0..2 {
            flow_buffers.push(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("FLOW BUFFER {}", i)),
                    contents: bytemuck::cast_slice(&initial_flow_data.as_slice()),
                    usage: wgpu::BufferUsage::VERTEX
                        | wgpu::BufferUsage::STORAGE
                        | wgpu::BufferUsage::COPY_DST,
                }),
            );
        }

        for i in 0..2 {
            flow_bind_groups.push(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some(&format!("FLOW BIND GROUP {}", i)),
                layout: &flow_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::Buffer(flow_sim_buffer.slice(..)),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Buffer(flow_buffers[i].slice(..)),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Buffer(
                            flow_buffers[(i + 1) % 2].slice(..),
                        ),
                    },
                ],
            }));
        }

        let flow_work_group_count = ((NUM_FLOW as f32) / (64.0)).ceil() as u32;

        Self {
            // INPUT
            input,

            // FLAGS
            quit,
            pause,
            dbuf,

            // INSTANCE
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,

            // UNIFORMS
            camera,
            view_uniforms,
            view_uniform_buffer,
            view_uniform_bind_group,

            // TIME
            last_tick: std::time::Instant::now(),
            delta,

            // BACKGROUND
            render_pipeline,
            vertex_buffer,
            index_buffer,
            num_indices: INDICES.len() as u32,

            // FLOW
            flow_compute_pipeline,
            flow_render_pipeline,
            flow_sim_buffer,
            flow_bind_groups,
            flow_buffers,
            flow_vertices_buffer,
            flow_indices_buffer,
            flow_work_group_count,
            flow_num_indices: 8,

            // WINDOW
            size,
            frame_num: 0,
        }
    }

    fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.sc_desc.width = new_size.width;
        self.sc_desc.height = new_size.height;
        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);

        self.camera.asp = self.sc_desc.width as f32 / self.sc_desc.height as f32;

        self.update_view_uniforms();
        self.render();
    }

    fn update(&mut self, event: &Event<()>) {
        if self.input.update(event) {
            if self.input.key_pressed(VirtualKeyCode::Escape) || self.input.quit() {
                self.quit = true;
            } else if self.input.key_pressed(VirtualKeyCode::Space) {
                self.pause = !self.pause;
            }

            // Moving around the camera
            {
                let mut cam_pos_delta = glam::Vec2::zero();
                if self.input.key_held(VirtualKeyCode::A) {
                    cam_pos_delta[0] -= 1.0;
                }
                if self.input.key_held(VirtualKeyCode::D) {
                    cam_pos_delta[0] += 1.0;
                }
                if self.input.key_held(VirtualKeyCode::S) {
                    cam_pos_delta[1] -= 1.0;
                }
                if self.input.key_held(VirtualKeyCode::W) {
                    cam_pos_delta[1] += 1.0;
                }
                if cam_pos_delta != glam::Vec2::zero() {
                    cam_pos_delta = cam_pos_delta.normalize() * self.delta;
                    self.camera.pos += cam_pos_delta;
                    self.update_view_uniforms();
                }
            }

            // Zooming the camera
            {
                let scroll_diff = self.input.scroll_diff();
                if scroll_diff != 0.0 {
                    self.camera.scl += scroll_diff * 0.1;
                    self.update_view_uniforms();
                }
            }
        }
    }

    fn render(&mut self) {
        if !self.pause {
            self.dbuf ^= 1;
        }

        let frame = self
            .swap_chain
            .get_current_frame()
            .expect("Timeout getting texture")
            .output;

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("RENDER ENCODER"),
            });

        if !self.pause {
            let mut cpass = encoder.begin_compute_pass();
            cpass.set_pipeline(&self.flow_compute_pipeline);
            cpass.set_bind_group(0, &self.flow_bind_groups[self.dbuf], &[]);
            cpass.dispatch(self.flow_work_group_count, 1, 1);
        }

        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &frame.view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: true,
                    },
                }],
                depth_stencil_attachment: None,
            });

            // RENDER BACKGROUND
            rpass.set_pipeline(&self.render_pipeline);
            rpass.set_bind_group(0, &self.view_uniform_bind_group, &[]);
            rpass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            rpass.set_index_buffer(self.index_buffer.slice(..));
            rpass.draw_indexed(0..self.num_indices, 0, 0..1);

            // RENDER FLOW PARTICLES
            rpass.set_pipeline(&self.flow_render_pipeline);
            rpass.set_bind_group(0, &self.view_uniform_bind_group, &[]);
            rpass.set_vertex_buffer(0, self.flow_buffers[self.dbuf ^ 1].slice(..));
            rpass.set_vertex_buffer(1, self.flow_vertices_buffer.slice(..));
            rpass.set_index_buffer(self.flow_indices_buffer.slice(..));
            rpass.draw_indexed(0..self.flow_num_indices, 0, 0..NUM_FLOW as _);
        }

        self.frame_num += 1;
        self.queue.submit(Some(encoder.finish()));
        self.update_delta();
    }

    fn update_view_uniforms(&mut self) {
        self.view_uniforms.update_view_proj(&self.camera);
        self.queue.write_buffer(
            &self.view_uniform_buffer,
            0,
            bytemuck::cast_slice(&[self.view_uniforms]),
        );
    }

    // Update all sim deltas
    fn update_delta(&mut self) {
        self.delta = self.last_tick.elapsed().as_secs_f32();
        self.queue.write_buffer(
            &self.flow_sim_buffer,
            0,
            bytemuck::cast_slice(&[self.delta]),
        );
        self.last_tick = std::time::Instant::now();
    }
}
