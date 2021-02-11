use crate::GlobalConfig;
use pollster::block_on;
use winit::event::{Event, VirtualKeyCode};
use winit::window::Window;
use winit_input_helper::WinitInputHelper;

pub trait World {
    fn resize(
        &mut self,
        resources: &Resources,
    );
    fn update(
        &mut self,
        resources: &Resources,
    );
    fn render(
        &mut self,
        resources: &Resources,
    );
}

pub struct Mastermind {
    pub resources: Resources,
    pub world:     Option<Box<dyn World>>,
}
impl Mastermind {
    pub fn update(
        &mut self,
        event: &Event<()>,
    ) {
        self.resources.update(event);
        self.world
            .as_mut()
            .expect("World not initialized")
            .update(&self.resources);
    }

    pub fn resize(
        &mut self,
        new_size: winit::dpi::PhysicalSize<u32>,
    ) {
        self.resources.resize(new_size);
        self.world
            .as_mut()
            .expect("World not initialized")
            .resize(&self.resources);
        self.world
            .as_mut()
            .expect("World not initialized")
            .render(&self.resources);
    }

    pub fn render(&mut self) {
        self.resources.render();
        self.world
            .as_mut()
            .expect("World not initialized")
            .render(&self.resources);
    }
}

/// Global constant resources
/// Different worlds will share these
pub struct Resources {
    //
    // GLOBAL
    pub window:        Window,
    pub global_config: GlobalConfig,
    //
    // INPUT
    pub input:         WinitInputHelper,
    //
    // FLAGS
    pub quit:          bool,
    pub pause:         bool,
    /// Modified externally
    pub fullscreen:    bool,
    //
    // INSTANCE
    pub surface:       wgpu::Surface,
    pub device:        wgpu::Device,
    pub queue:         wgpu::Queue,
    pub sc_desc:       wgpu::SwapChainDescriptor,
    pub swap_chain:    wgpu::SwapChain,
    pub sample_count:  u32,
    //
    // TIME
    pub last_tick:     std::time::Instant,
    pub delta:         f32,
    pub elapsed_time:  std::time::Duration,
    //
    // BUILD AFTER SELF
    pub msaa_fbuffer:  Option<wgpu::TextureView>,
    //
    // WINDOW
    pub size:          winit::dpi::PhysicalSize<u32>,
    pub frame_num:     usize,
    pub active:        Option<usize>,
}

impl Resources {
    pub fn new(
        window: Window,
        global_config: GlobalConfig,
    ) -> Self {
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
        let surface = unsafe { instance.create_surface(&window) };
        let adapter = block_on(async {
            instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference:   wgpu::PowerPreference::HighPerformance,
                    compatible_surface: Some(&surface),
                })
                .await
        })
        .expect("COULD NOT FIND GPU");

        let (device, queue) = block_on(async {
            adapter
                .request_device(
                    &wgpu::DeviceDescriptor {
                        label:    None,
                        features: wgpu::Features::default()
                            | wgpu::Features::MAPPABLE_PRIMARY_BUFFERS,
                        limits:   wgpu::Limits {
                            max_storage_buffers_per_shader_stage: 10,
                            ..Default::default()
                        },
                    },
                    None,
                )
                .await
        })
        .expect("COULD NOT BUILD LOGICAL DEVICE/QUEUE");

        let sc_desc = wgpu::SwapChainDescriptor {
            usage:        wgpu::TextureUsage::RENDER_ATTACHMENT,
            format:       wgpu::TextureFormat::Bgra8Unorm,
            width:        size.width,
            height:       size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };

        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        let mut s = Self {
            window,
            global_config,
            input,
            quit: false,
            pause: false,
            fullscreen,
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            sample_count,
            last_tick: std::time::Instant::now(),
            delta: 0.0,
            elapsed_time: std::time::Duration::new(0, 0),
            msaa_fbuffer: None,
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

    pub fn update(
        &mut self,
        event: &Event<()>,
    ) {
        if self.input.update(event) {
            if self.input.key_pressed(VirtualKeyCode::Escape) || self.input.quit() {
                self.quit = true;
                return;
            }
            if self.input.key_pressed(VirtualKeyCode::Space) {
                self.pause = !self.pause;
            }
        }
    }

    /// Toggle windows fullscreen setting when called
    pub fn toggle_fullscreen(&mut self) {
        dinfo!("Fullscreen Toggled");
        if self.fullscreen {
            self.window.set_fullscreen(None);
            self.fullscreen = false;
        } else {
            self.window
                .set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
            self.fullscreen = true;
        }
    }

    fn create_multisampled_framebuffer(&mut self) {
        if self.sample_count > 1 {
            let multisampled_texture_extent = wgpu::Extent3d {
                width:  self.sc_desc.width,
                height: self.sc_desc.height,
                depth:  1,
            };
            let multisampled_frame_descriptor = &wgpu::TextureDescriptor {
                size:            multisampled_texture_extent,
                mip_level_count: 1,
                sample_count:    self.sample_count,
                dimension:       wgpu::TextureDimension::D2,
                format:          self.sc_desc.format,
                usage:           wgpu::TextureUsage::RENDER_ATTACHMENT,
                label:           None,
            };
            self.msaa_fbuffer = Some(
                self.device
                    .create_texture(multisampled_frame_descriptor)
                    .create_view(&wgpu::TextureViewDescriptor::default()),
            );
        } else {
            self.msaa_fbuffer = None;
        }
    }

    pub fn resize(
        &mut self,
        new_size: winit::dpi::PhysicalSize<u32>,
    ) {
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
    }

    pub fn render(&mut self) {
        self.frame_num += 1;
        self.update_time_info();
    }

    fn update_time_info(&mut self) {
        self.delta = self.last_tick.elapsed().as_secs_f32();
        if !self.pause {
            self.elapsed_time += self.last_tick.elapsed();
        }
        self.last_tick = std::time::Instant::now();
    }
}
