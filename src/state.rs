use crate::{flow, view, GlobalConfig};
use memoffset::*;
use pollster::block_on;
use rand::{Rng, SeedableRng};
use rand_xorshift::XorShiftRng;
use rayon::prelude::*;
use wgpu::util::DeviceExt;
use winit::event::{Event, VirtualKeyCode};
use winit::window::Window;
use winit_input_helper::WinitInputHelper;

pub struct State {
    // INPUT
    pub input: WinitInputHelper,

    // FLAGS
    pub quit: bool,
    pub pause: bool,
    /// Modified externally
    pub fullscreen: bool,

    // INSTANCE
    pub surface: wgpu::Surface,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub sc_desc: wgpu::SwapChainDescriptor,
    pub swap_chain: wgpu::SwapChain,
    pub sample_count: u32,

    // UNIFORMS
    pub camera: view::Camera,
    pub view_uniforms: view::Uniforms,
    pub view_uniform_buffer: wgpu::Buffer,
    pub view_uniform_bind_group: wgpu::BindGroup,

    // TIME
    pub last_tick: std::time::Instant,
    pub delta: f32,
    pub flow_elapsed_time: std::time::Duration,

    // FLOW
    pub flow_compute_pipeline: wgpu::ComputePipeline,
    pub flow_render_pipeline: wgpu::RenderPipeline,
    pub flow_uniforms: flow::Uniforms,
    pub flow_uniform_buffer: wgpu::Buffer,
    pub flow_atomics: flow::Atomics,
    pub flow_atomic_buffer: wgpu::Buffer,
    pub flow_bind_groups: Vec<wgpu::BindGroup>,
    /// Alternating buffer
    pub flow_buffers: Vec<wgpu::Buffer>,
    /// Alternating buffer
    pub flow_vertices_buffer: wgpu::Buffer,
    pub flow_indices_buffer: wgpu::Buffer,
    pub flow_num_indices: u32,
    pub flow_work_group_count: u32,
    /// Buffer idx for compute
    pub flow_buff_idx: usize,
    pub flow_cap: u32,
    pub flow_count: u32,
    // Spawners
    pub spaw_coll_buffer: wgpu::Buffer,
    pub spawner_buffer: wgpu::Buffer,
    // Manipulators
    pub mani_coll_buffer: wgpu::Buffer,
    pub manipulator_buffer: wgpu::Buffer,
    // Accumulators
    pub accu_coll_buffer: wgpu::Buffer,
    pub accumulator_buffer: wgpu::Buffer,

    // BUILD AFTER SELF
    pub multisampled_framebuffer: Option<wgpu::TextureView>,

    // WINDOW
    pub size: winit::dpi::PhysicalSize<u32>,
    pub frame_num: usize,
    pub active: Option<usize>,
}

impl State {
    pub async fn new(window: &Window, global_config: &GlobalConfig) -> Self {
        let now = std::time::Instant::now();
        // CONFIG
        let sample_count = global_config.window.msaa;

        // INPUT
        let input = WinitInputHelper::new();

        // FLAGS
        let fullscreen = window.fullscreen().is_some();

        // INSTANCE
        dinfo!("Instance ({} ms)", now.elapsed().as_millis());
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
                    label: None,
                    features: wgpu::Features::default() | wgpu::Features::MAPPABLE_PRIMARY_BUFFERS,
                    limits: wgpu::Limits {
                        max_storage_buffers_per_shader_stage: 10,
                        ..Default::default()
                    },
                },
                None,
            )
            .await
            .expect("COULD NOT BUILD LOGICAL DEVICE/QUEUE");

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::RENDER_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8Unorm,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };

        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        // SHADER LOADING
        dinfo!("Shader Loading ({} ms)", now.elapsed().as_millis());
        let flow_cs_module =
            device.create_shader_module(&wgpu::include_spirv!("../spirv/flow.comp.spv"));
        let flow_vs_module =
            device.create_shader_module(&wgpu::include_spirv!("../spirv/flow.vert.spv"));
        let flow_fs_module =
            device.create_shader_module(&wgpu::include_spirv!("../spirv/flow.frag.spv"));

        // UNIFORMS
        dinfo!("View Uniforms ({} ms)", now.elapsed().as_millis());
        let camera = view::Camera {
            slow_spd: 1.5,
            fast_spd_fac: 2.0,
            pos: glam::Vec2::zero(),
            scl: 1.0,
            asp: sc_desc.width as f32 / sc_desc.height as f32,
        };

        let mut view_uniforms = view::Uniforms::default();
        view_uniforms.update_view_proj(&camera);

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
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: wgpu::BufferSize::new(
                            std::mem::size_of::<view::Uniforms>() as _,
                        ),
                    },
                    count: None,
                }],
            });

        let view_uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("UNIFORM BIND GROUP"),
            layout: &view_uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: view_uniform_buffer.as_entire_binding(),
            }],
        });

        // FLOW
        dinfo!("Flow ({} ms)", now.elapsed().as_millis());
        let flow_uniforms = global_config.flow;
        let flow_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("FLOW SIM DATA"),
            contents: bytemuck::cast_slice(&[flow_uniforms]),
            usage: wgpu::BufferUsage::UNIFORM
                | wgpu::BufferUsage::COPY_DST
                | wgpu::BufferUsage::MAP_READ,
        });

        let flow_atomics = flow::Atomics { atom_ct: 0 };

        let flow_atomic_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("FLOW ATOMIC SIM DATA"),
            contents: bytemuck::cast_slice(&[flow_atomics]),
            usage: wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST
                | wgpu::BufferUsage::COPY_SRC,
        });

        let flow_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: None,
                entries: &[
                    // Flow Uniforms
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStage::COMPUTE
                            | wgpu::ShaderStage::VERTEX
                            | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<
                                flow::Uniforms,
                            >()
                                as _),
                        },
                        count: None,
                    },
                    // Flow Atomics
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                std::mem::size_of::<flow::Atomics>() as _,
                            ),
                        },
                        count: None,
                    },
                    // Flow particle 0
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_FLOW as usize * std::mem::size_of::<flow::Particle>())
                                    as _,
                            ),
                        },
                        count: None,
                    },
                    // Flow particle 1
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStage::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_FLOW as usize * std::mem::size_of::<flow::Particle>())
                                    as _,
                            ),
                        },
                        count: None,
                    },
                    // Spawner Colliders
                    wgpu::BindGroupLayoutEntry {
                        binding: 4,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_SPAW as usize * std::mem::size_of::<flow::SpawnerCollider>()) as _,
                            ),
                        },
                        count: None,
                    },
                    // Spawners
                    wgpu::BindGroupLayoutEntry {
                        binding: 5,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_SPAW as usize * std::mem::size_of::<flow::Spawner>()) as _,
                            ),
                        },
                        count: None,
                    },
                    // Manipulator Colliders
                    wgpu::BindGroupLayoutEntry {
                        binding: 6,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_MANI as usize * std::mem::size_of::<flow::Collider>()) as _,
                            ),
                        },
                        count: None,
                    },
                    // Manipulators
                    wgpu::BindGroupLayoutEntry {
                        binding: 7,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_MANI as usize * std::mem::size_of::<flow::Manipulator>()) as _,
                            ),
                        },
                        count: None,
                    },
                    // Accumulator Colliders
                    wgpu::BindGroupLayoutEntry {
                        binding: 8,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_ACCU as usize * std::mem::size_of::<flow::Collider>()) as _,
                            ),
                        },
                        count: None,
                    },
                    // Accumulators
                    wgpu::BindGroupLayoutEntry {
                        binding: 9,
                        visibility: wgpu::ShaderStage::COMPUTE | wgpu::ShaderStage::VERTEX | wgpu::ShaderStage::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: wgpu::BufferSize::new(
                                (flow::MAX_NUM_ACCU as usize * std::mem::size_of::<flow::Accumulator>()) as _,
                            ),
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
                module: &flow_cs_module,
                entry_point: "main",
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
            vertex: wgpu::VertexState {
                module: &flow_vs_module,
                entry_point: "main",
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: 4 * 4,
                        step_mode: wgpu::InputStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![0 => Float2, 1 => Float2],
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: 2 * 4,
                        step_mode: wgpu::InputStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![2 => Float2],
                    },
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &flow_fs_module,
                entry_point: "main",
                targets: &[wgpu::ColorTargetState {
                    format: sc_desc.format,
                    alpha_blend: wgpu::BlendState::REPLACE,
                    color_blend: wgpu::BlendState {
                        src_factor: wgpu::BlendFactor::SrcAlpha,
                        dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                        operation: wgpu::BlendOperation::Add,
                    },
                    write_mask: wgpu::ColorWrite::ALL,
                }],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: wgpu::CullMode::None,
                polygon_mode: wgpu::PolygonMode::Fill,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: sample_count,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
        });

        let flow_vertices_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("VERTEX BUFFER"),
            contents: bytemuck::bytes_of(&flow::FLOW_SHAPE_VERTICES),
            usage: wgpu::BufferUsage::VERTEX | wgpu::BufferUsage::COPY_DST,
        });
        let flow_indices_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("INDEX BUFFER"),
            contents: bytemuck::bytes_of(&flow::FLOW_SHAPE_INDICES),
            usage: wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
        });

        dinfo!("Initial Flow Data ({} ms)", now.elapsed().as_millis());
        let mut initial_flow_data = vec![
            flow::Particle {
                pos: [0.0, 0.0],
                vel: [0.0, 0.0],
            };
            flow::MAX_NUM_FLOW as _
        ];
        initial_flow_data
            .par_chunks_mut((flow_uniforms.ct / 32) as usize)
            .enumerate()
            .for_each(|(i, c)| {
                let mut xsrng = XorShiftRng::seed_from_u64(i as _);
                let flow_ext = flow_uniforms.flow_ext;
                let flow_max = flow_uniforms.flow_max;
                for p in c {
                    p.pos[0] = xsrng.gen_range(-flow_ext..flow_ext); // posx
                    p.pos[1] = xsrng.gen_range(-flow_ext..flow_ext); // posy
                    p.vel[0] = xsrng.gen_range(-flow_max..flow_max); // velx
                    p.vel[1] = xsrng.gen_range(-flow_max..flow_max); // vely
                }
            });


        let (init_spaw_coll, mut init_mani_coll, mut init_accu_coll) = (
            // Spawner 'Colliders'
            vec![flow::SpawnerCollider { rte: 1.0 }; flow::MAX_NUM_SPAW],
            // Manipulators
            vec![
                flow::Collider {
                    x: 0.0,
                    y: 0.0,
                    r2: 2.0
                };
                flow::MAX_NUM_MANI
            ],
            // Accumulators
            vec![
                flow::Collider {
                    x: 0.0,
                    y: 0.0,
                    r2: 4.0,
                };
                flow::MAX_NUM_ACCU
            ],
        );

        let rand_num: u64 = rand::prelude::thread_rng().gen();

        // Spawners
        let mut init_spawner_data = vec![
            flow::Spawner {
                x: 0.0,
                y: 0.0,
                scl: 0.0,
                var: 0.0,
                r: 1.0,
                g: 1.0,
                b: 1.0,
            };
            flow::MAX_NUM_SPAW
        ];
        init_spawner_data.iter_mut().enumerate().for_each(|(i, p)| {
            let mut xsrng = XorShiftRng::seed_from_u64(i as u64 + rand_num);
            let flow_ext = flow_uniforms.flow_ext;
            p.x = xsrng.gen_range(-flow_ext..flow_ext); // posx
            p.y = xsrng.gen_range(-flow_ext..flow_ext); // posx
        });
        let spaw_coll_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("SPAWNER COLLIDER BUFFER"),
            contents: &bytemuck::cast_slice(&*init_spaw_coll),
            usage: wgpu::BufferUsage::VERTEX
                | wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });
        let spawner_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("SPAWNER BUFFER"),
            contents: &bytemuck::cast_slice(&*init_spawner_data),
            usage: wgpu::BufferUsage::VERTEX
                | wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });

        // Manipulators
        init_mani_coll.iter_mut().enumerate().for_each(|(i, p)| {
            let mut xsrng = XorShiftRng::seed_from_u64(i as u64 + rand_num);
            let flow_ext = flow_uniforms.flow_ext;
            p.x = xsrng.gen_range(-flow_ext..flow_ext); // posx
            p.y = xsrng.gen_range(-flow_ext..flow_ext); // posx
        });
        let mani_coll_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("MANIPULATOR COLLIDERS BUFFER"),
            contents: &bytemuck::cast_slice(&*init_mani_coll),
            usage: wgpu::BufferUsage::VERTEX
                | wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });
        let manipulator_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("MANIPULATOR BUFFER"),
            contents: &bytemuck::cast_slice(
                vec![
                    flow::Manipulator {
                        acc: 1.0,
                        spd: 1.0,
                        rot: 0.0
                    };
                    flow::MAX_NUM_MANI
                ]
                .as_slice(),
            ),
            usage: wgpu::BufferUsage::VERTEX
                | wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });

        // Accumulators
        // WHAT THE ACTUAL FUCK IS THIS DOING
        let mut xsrng = XorShiftRng::seed_from_u64(rand_num);
        //let mut xsrng = rand::prelude::thread_rng();
        init_accu_coll.iter_mut().for_each(|p| {
            let flow_ext = flow_uniforms.flow_ext - 2.0;
            p.x = xsrng.gen_range((-flow_ext)..(flow_ext)); // posx
            p.y = xsrng.gen_range((-flow_ext)..(flow_ext)); // posy
        });
        let accu_coll_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("ACCUMULATOR COLLIDERS"),
            contents: &bytemuck::cast_slice(init_accu_coll.as_slice()),
            usage: wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });
        let accumulator_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("ACCUMULATOR BUFFER"),
            contents: &bytemuck::cast_slice(
                &*vec![flow::Accumulator { rte: 1.0 }; flow::MAX_NUM_ACCU],
            ),
            usage: wgpu::BufferUsage::VERTEX
                | wgpu::BufferUsage::STORAGE
                | wgpu::BufferUsage::COPY_DST,
        });

        let mut flow_buffers = Vec::<wgpu::Buffer>::with_capacity(2);
        let mut flow_bind_groups = Vec::<wgpu::BindGroup>::with_capacity(2);
        for i in 0..2 {
            flow_buffers.push(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("FLOW BUFFER {}", i)),
                    contents: bytemuck::cast_slice(&*initial_flow_data),
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
                        resource: flow_uniform_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: flow_atomic_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: flow_buffers[i].as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: flow_buffers[(i + 1) % 2].as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: spaw_coll_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: spawner_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: mani_coll_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: manipulator_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: accu_coll_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 9,
                        resource: accumulator_buffer.as_entire_binding(),
                    },
                ],
            }));
        }

        let flow_work_group_count = ((flow_uniforms.ct as f32) / (64.0)).ceil() as u32;

        let flow_num_indices = 8;

        let mut s = Self {
            // INPUT
            input,

            // FLAGS
            quit: false,
            pause: false,
            fullscreen,
            flow_buff_idx: 0,

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
            delta: 0.0,
            flow_elapsed_time: std::time::Duration::new(0, 0),

            // FLOW
            flow_compute_pipeline,
            flow_render_pipeline,
            flow_uniforms,
            flow_uniform_buffer,
            flow_atomics,
            flow_atomic_buffer,
            flow_bind_groups,
            flow_buffers,
            flow_vertices_buffer,
            flow_indices_buffer,
            flow_num_indices,

            flow_work_group_count,
            flow_cap: flow::MAX_NUM_FLOW as u32,
            flow_count: flow_uniforms.ct as u32,

            spaw_coll_buffer,
            spawner_buffer,
            mani_coll_buffer,
            manipulator_buffer,
            accu_coll_buffer,
            accumulator_buffer,

            multisampled_framebuffer: None,

            // WINDOW
            size,
            frame_num: 0,
            active: Some(0),
        };

        dinfo!(
            "Multisampled Framebuffer ({} ms)",
            now.elapsed().as_millis()
        );
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
                usage: wgpu::TextureUsage::RENDER_ATTACHMENT,
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

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
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

    pub fn update(&mut self, event: &Event<()>) {
        if self.input.update(event) {
            if self.input.key_pressed(VirtualKeyCode::Escape) || self.input.quit() {
                self.quit = true;
                return;
            }
            if self.input.key_pressed(VirtualKeyCode::Space) {
                self.pause = !self.pause;
            }

            if self.camera.update(&self.input, self.delta) {
                self.update_view_uniforms();
            }
        }
    }

    // TODO: AGGRESSIVELY REFACTOR
    // This code is messy and needs to be cleaned up
    pub fn render(&mut self) {
        self.frame_num += 1;

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("COMPUTE ENCODER"),
            });

        if !self.pause {
            self.flow_buff_idx ^= 1;
            self.flow_work_group_count = ((self.flow_count as f32) / (64.0)).ceil() as u32;

            let mut cpass =
                encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: None });
            cpass.set_pipeline(&self.flow_compute_pipeline);
            cpass.set_bind_group(0, &self.flow_bind_groups[self.flow_buff_idx], &[]);
            cpass.dispatch(self.flow_work_group_count, 1, 1);
        }

        if !self.pause {
            self.queue
                .write_buffer(&self.flow_atomic_buffer, 0u64, bytemuck::cast_slice(&[0]));

            encoder.copy_buffer_to_buffer(
                &self.flow_atomic_buffer,
                offset_of!(flow::Atomics, atom_ct) as _,
                &self.flow_uniform_buffer,
                offset_of!(flow::Uniforms, ct) as _,
                std::mem::size_of::<u32>() as _,
            );
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

        if !self.pause {
            self.queue.submit(std::iter::once(encoder.finish()));

            // Set internal count to correct value
            block_on(async {
                let buffer_slice = self.flow_uniform_buffer.slice(..);
                let buffer_future = buffer_slice.map_async(wgpu::MapMode::Read);
                self.device.poll(wgpu::Maintain::Wait);

                buffer_future.await.unwrap();
                let buffer = buffer_slice.get_mapped_range();

                let (_, raw, _) = unsafe { buffer.align_to::<flow::Uniforms>() };

                self.flow_count = raw[0].ct;

                drop(buffer);
                self.flow_uniform_buffer.unmap();
            });
        }
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("COMPUTE ENCODER"),
            });

        let frame = self
            .swap_chain
            .get_current_frame()
            .expect("TIMEOUT GETTING TEXTURE")
            .output;

        {
            let ops = wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.0,
                    g: 0.0,
                    b: 0.0,
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
                label: None,
                color_attachments: &[rpass_color_attachment],
                depth_stencil_attachment: None,
            });

            // RENDER FLOW PARTICLES
            rpass.set_pipeline(&self.flow_render_pipeline);
            rpass.set_bind_group(0, &self.view_uniform_bind_group, &[]);
            rpass.set_vertex_buffer(0, self.flow_buffers[self.flow_buff_idx ^ 1].slice(..));
            rpass.set_vertex_buffer(1, self.flow_vertices_buffer.slice(..));
            rpass.set_index_buffer(
                self.flow_indices_buffer.slice(..),
                wgpu::IndexFormat::Uint16,
            );
            rpass.draw_indexed(0..self.flow_num_indices, 0, 0..self.flow_count as _);
        }

        self.queue.submit(std::iter::once(encoder.finish()));

        self.update_time_info();
        assert!(self.flow_count <= flow::MAX_NUM_FLOW as u32);
    }

    fn update_view_uniforms(&mut self) {
        self.view_uniforms.update_view_proj(&self.camera);
        self.queue.write_buffer(
            &self.view_uniform_buffer,
            offset_of!(view::Uniforms, view_pos) as _,
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
            offset_of!(flow::Uniforms, dt) as _,
            bytemuck::cast_slice(&[self.delta]),
        );
        self.last_tick = std::time::Instant::now();
    }
}
