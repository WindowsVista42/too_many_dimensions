mod data;
mod flowdata;

use data::*;
use flowdata::*;

use futures::executor::block_on;
use wgpu::util::DeviceExt;
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::{Window, WindowBuilder};
use winit_input_helper::WinitInputHelper;

#[rustfmt::skip]
const FLOW_SHAPE_VERTICES: [FlowVertex; 4] = [
    FlowVertex { pos: [-0.01, -0.01], },
    FlowVertex { pos: [-0.01,  0.01], },
    FlowVertex { pos: [ 0.01,  0.01], },
    FlowVertex { pos: [ 0.01, -0.01], },
];

#[rustfmt::skip]
const FLOW_SHAPE_INDICES: [u16; 6] = [
    0, 1, 2,
    0, 2, 3,
];

const MAX_NUM_FLOW: usize = 1_000_000;
const NUM_FLOW: usize = 100_000;

fn toggle_fullscreen(state: &mut State, window: &Window) {
    if state.fullscreen {
        window.set_fullscreen(None);
        state.fullscreen = false;
    } else {
        window.set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
        state.fullscreen = true;
    }
}

fn main() {
    let event_loop = EventLoop::new();

    let window = WindowBuilder::new()
        .with_title("Too Many Dimensions")
        .build(&event_loop)
        .unwrap();

    let mut state = block_on(State::new(&window, 8));

    event_loop.run(move |event, _, control_flow| {
        state.update(&event);
        if state.quit {
            *control_flow = ControlFlow::Exit;
            return;
        }

        match event {
            #[rustfmt::skip]
            Event::WindowEvent {
                event: WindowEvent::KeyboardInput {
                    input: KeyboardInput {
                        state: ElementState::Released,
                        virtual_keycode: Some(VirtualKeyCode::Return),
                        ..
                    }, ..
                }, ..
            } => {
                if state.input.held_alt() {
                    toggle_fullscreen(&mut state, &window);
                }
            },
            #[rustfmt::skip]
            Event::WindowEvent {
                event: WindowEvent::KeyboardInput {
                    input: KeyboardInput {
                        state: ElementState::Released,
                        virtual_keycode: Some(VirtualKeyCode::F11),
                        ..
                    }, ..
                }, ..
            } => {
                toggle_fullscreen(&mut state, &window);
            }
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
    fullscreen: bool, // Modified externally

    // INSTANCE
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    sc_desc: wgpu::SwapChainDescriptor,
    swap_chain: wgpu::SwapChain,
    sample_count: u32,

    // UNIFORMS
    camera: Camera,
    view_uniforms: ViewUniforms,
    view_uniform_buffer: wgpu::Buffer,
    view_uniform_bind_group: wgpu::BindGroup,

    // TIME
    last_tick: std::time::Instant,
    delta: f32,
    flow_elapsed_time: std::time::Duration,

    // FLOW
    flow_compute_pipeline: wgpu::ComputePipeline,
    flow_render_pipeline: wgpu::RenderPipeline,
    flow_uniforms: FlowUniforms,
    flow_uniform_buffer: wgpu::Buffer,
    flow_bind_groups: Vec<wgpu::BindGroup>, // Alternating buffer
    flow_buffers: Vec<wgpu::Buffer>,        // Alternating buffer

    flow_vertices_buffer: wgpu::Buffer,
    flow_indices_buffer: wgpu::Buffer,
    flow_num_indices: u32,

    flow_work_group_count: u32,
    buf_idx: usize, // Buffer idx for compute
    flow_cap: u32,
    flow_count: u32,

    // BUILD AFTER SELF
    multisampled_framebuffer: Option<wgpu::TextureView>,

    // WINDOW
    size: winit::dpi::PhysicalSize<u32>,
    frame_num: usize,
    active: Option<usize>,
}

impl State {
    async fn new(window: &Window, sample_count: u32) -> Self {
        // INPUT
        let input = WinitInputHelper::new();

        // FLAGS
        let quit = false;
        let pause = false;
        let fullscreen = match window.fullscreen() {
            Some(_) => true,
            None => false,
        };
        let buf_idx = 0;

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
            .expect("COULD NOT FIND GPU");

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
            .expect("COULD NOT BUILD LOGICAL DEVICE");

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };

        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        // SHADER LOADING
        let flow_cs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flow.comp.spv"));
        let flow_vs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flowvert.vert.spv"));
        let flow_fs_module =
            device.create_shader_module(wgpu::include_spirv!("../spirv/flowfrag.frag.spv"));

        // UNIFORMS
        let camera = Camera {
            changed: false,
            slow_spd: 1.5,
            fast_spd_fac: 2.0,
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

        // FLOW
        let flow_uniforms = FlowUniforms {
            dt: 0.0,
            count: NUM_FLOW as u32,
            part_ext: 0.0,
            part_acc: 0.0,
            part_max: 0.0,
            part_flag: 0,
            flow_scl: 0.0,
            flow_off: 0.0,
            flow_flag: 0,
            coll_scl: 0.0,
            coll_flag: 0,
            mani_acc: 0.0,
            mani_spd: 0.0,
            mani_flag: 0,
            spaw_rate: 0.0,
            spaw_scl: 0.0,
            spaw_var: 0.0,
            spaw_col: [0, 0, 0, 0],
            spaw_flag: 0,
            accu_gain: 0.0,
            accu_flag: 0,
        };
        let flow_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("FLOW SIM DATA"),
            contents: bytemuck::cast_slice(&[flow_uniforms]),
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
                            min_binding_size: wgpu::BufferSize::new(
                                std::mem::size_of::<FlowUniforms>() as _,
                            ),
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
                compute_stage: wgpu::ProgrammableStageDescriptor {
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
            color_states: &[wgpu::ColorStateDescriptor {
                format: sc_desc.format,
                alpha_blend: wgpu::BlendDescriptor::REPLACE,
                color_blend: wgpu::BlendDescriptor {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                write_mask: wgpu::ColorWrite::ALL,
            }],
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
            sample_count,
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
                vel: glam::Vec2::zero().into()
            };
            (4 * MAX_NUM_FLOW) as usize
        ];
        for p in initial_flow_data.iter_mut() {
            p.pos[0] = 12.0 * (rand::random::<f32>() - 0.5); // posx
            p.pos[1] = 12.0 * (rand::random::<f32>() - 0.5); // posy
            p.vel[0] = 2.0 * (rand::random::<f32>() - 0.5) * 0.1; // velx
            p.vel[1] = 2.0 * (rand::random::<f32>() - 0.5) * 0.1; // vely
            if p.pos[0] == 0.0 {
                p.pos[0] = 0.01;
            }
            if p.pos[1] == 0.0 {
                p.pos[1] = 0.01;
            }
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
                        resource: wgpu::BindingResource::Buffer(flow_uniform_buffer.slice(..)),
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

        let flow_num_indices = 8;

        let mut s = Self {
            // INPUT
            input,

            // FLAGS
            quit,
            pause,
            fullscreen,
            buf_idx,

            // INSTANCE
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            sample_count,

            // UNIFORMS
            camera,
            view_uniforms,
            view_uniform_buffer,
            view_uniform_bind_group,

            // TIME
            last_tick: std::time::Instant::now(),
            delta,
            flow_elapsed_time: std::time::Duration::new(0, 0),

            // FLOW
            flow_compute_pipeline,
            flow_render_pipeline,
            flow_uniforms,
            flow_uniform_buffer,
            flow_bind_groups,
            flow_buffers,
            flow_vertices_buffer,
            flow_indices_buffer,
            flow_num_indices,

            flow_work_group_count,
            flow_cap: MAX_NUM_FLOW as u32,
            flow_count: NUM_FLOW as u32,

            multisampled_framebuffer: None,

            // WINDOW
            size,
            frame_num: 0,
            active: Some(0),
        };

        s.create_multisampled_framebuffer();

        s
    }

    fn create_multisampled_framebuffer(&mut self) {
        if self.sample_count > 1 {
            let multisampled_texture_extent = wgpu::Extent3d {
                width: self.sc_desc.width,
                height: self.sc_desc.height,
                depth: 1,
            };
            let multisampled_frame_descriptor = &wgpu::TextureDescriptor {
                size: multisampled_texture_extent,
                mip_level_count: 1,
                sample_count: self.sample_count,
                dimension: wgpu::TextureDimension::D2,
                format: self.sc_desc.format,
                usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
                label: None,
            };
            self.multisampled_framebuffer = Some(
                self.device
                    .create_texture(multisampled_frame_descriptor)
                    .create_view(&wgpu::TextureViewDescriptor::default()),
            );
        } else {
            self.multisampled_framebuffer = None;
        }
    }

    fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.sc_desc.width = new_size.width;
        self.sc_desc.height = new_size.height;

        // Taken from the wgpu-rs "Water" example
        if self.sc_desc.width == 0 && self.sc_desc.height == 0 {
            // Stop rendering altogether.
            self.active = None;
            return;
        } else {
            // The next frame queued is the wrong size: (0, 0),
            // so we skip a frame to avoid crashes where our
            // textures are the correct (window) size, and the
            // frame is still (0, 0).
            self.active = Some(self.frame_num + 1);
        }

        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
        self.create_multisampled_framebuffer();

        self.camera.asp = self.sc_desc.width as f32 / self.sc_desc.height as f32;

        self.update_view_uniforms();
        self.render();
    }

    fn update(&mut self, event: &Event<()>) {
        if self.input.update(event) {
            if self.input.key_pressed(VirtualKeyCode::Escape) || self.input.quit() {
                self.quit = true;
                return;
            }
            if self.input.key_pressed(VirtualKeyCode::Space) {
                self.pause = !self.pause;
            }

            self.camera.update(&self.input, self.delta);
            if self.camera.changed {
                self.update_view_uniforms();
                self.camera.changed = false;
            }
        }
    }

    fn change_flow_count(&mut self) {
        self.flow_count = 50000 * self.flow_elapsed_time.as_secs_f32() as u32;
        self.flow_count =
            ((((self.flow_elapsed_time.as_secs_f32()) * 0.5 + std::f32::consts::PI).cos() + 1.0)
                / 2.0
                * self.flow_cap as f32) as u32;
        self.queue.write_buffer(
            &self.flow_uniform_buffer,
            std::mem::size_of::<f32>() as _,
            bytemuck::cast_slice(&[self.flow_count]),
        );
        self.flow_work_group_count = ((self.flow_count as f32) / (64.0)).ceil() as u32;
    }

    fn render(&mut self) {
        self.frame_num += 1;

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("RENDER ENCODER"),
            });

        if !self.pause {
            self.change_flow_count();
            std::mem::swap(
                &mut self.flow_buffers[0].slice(..),
                &mut self.flow_buffers[1].slice(..),
            ); // Swap buffers for rendering
            self.buf_idx ^= 1;

            let mut cpass = encoder.begin_compute_pass();
            cpass.set_pipeline(&self.flow_compute_pipeline);
            cpass.set_bind_group(0, &self.flow_bind_groups[self.buf_idx], &[]);
            cpass.dispatch(self.flow_work_group_count, 1, 1);
        }

        // Only render valid frames. See resize method.
        if let Some(active) = self.active {
            if active >= self.frame_num {
                self.queue.submit(std::iter::once(encoder.finish()));
                self.update_time_info();
                return;
            }
        } else {
            self.queue.submit(std::iter::once(encoder.finish()));
            self.update_time_info();
            return;
        }

        let frame = self
            .swap_chain
            .get_current_frame()
            .expect("Timeout getting texture")
            .output;

        {
            let ops = wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.005,
                    g: 0.005,
                    b: 0.005,
                    a: 1.0,
                }),
                store: true,
            };

            let rpass_color_attachment = match self.multisampled_framebuffer {
                Some(_) => wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &self.multisampled_framebuffer.as_ref().unwrap(),
                    resolve_target: Some(&frame.view),
                    ops,
                },
                None => wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &frame.view,
                    resolve_target: None,
                    ops,
                },
            };

            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[rpass_color_attachment],
                depth_stencil_attachment: None,
            });

            // RENDER FLOW PARTICLES
            rpass.set_pipeline(&self.flow_render_pipeline);
            rpass.set_bind_group(0, &self.view_uniform_bind_group, &[]);
            rpass.set_vertex_buffer(0, self.flow_buffers[0].slice(..));
            rpass.set_vertex_buffer(1, self.flow_vertices_buffer.slice(..));
            rpass.set_index_buffer(self.flow_indices_buffer.slice(..));
            rpass.draw_indexed(0..self.flow_num_indices, 0, 0..self.flow_count as _);
        }

        self.queue.submit(std::iter::once(encoder.finish()));

        self.update_time_info();
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
    fn update_time_info(&mut self) {
        self.delta = self.last_tick.elapsed().as_secs_f32();
        if !self.pause {
            self.flow_elapsed_time += self.last_tick.elapsed();
        }
        self.queue.write_buffer(
            &self.flow_uniform_buffer,
            0,
            bytemuck::cast_slice(&[self.delta]),
        );
        self.last_tick = std::time::Instant::now();
    }
}
