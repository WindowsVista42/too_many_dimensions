#![feature(const_ptr_offset_from, const_maybe_uninit_as_ptr, const_raw_ptr_deref)]

use std::fs;
use std::io::Read;

use serde::{Deserialize, Serialize};
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;

use flow_world::*;
use mastermind::Mastermind;

use crate::resources::Resources;

macro_rules! dinfo {
    ($($arg:tt)*) => {
        log!(log::Level::Info, "{}\nLOCATION : {}:{}:{}\n", format!($($arg)*), file!(), line!(), column!());
    };
}

#[macro_use]
extern crate log;

mod flow;
mod flow_world;
mod mastermind;
mod resources;
mod view2d;

#[derive(Debug, Serialize, Deserialize)]
// TODO: Add pretty printing
pub struct GlobalConfig {
    window: WindowConfig,
    flow: flow::Uniforms,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct WindowConfig {
    msaa: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let p_st = std::time::Instant::now();

    flexi_logger::Logger::with(
        flexi_logger::LogSpecification::default(flexi_logger::LevelFilter::max()).build(),
    )
    .log_to_file()
    .directory("log")
    .suffix("log")
    .print_message()
    .rotate(
        flexi_logger::Criterion::Size(u64::MAX),
        flexi_logger::Naming::Numbers,
        flexi_logger::Cleanup::KeepLogAndCompressedFiles(1, 15),
    )
    .create_symlink("current_run")
    .start()?;
    log_panics::init();

    debug!(
        "Logger Started: {}\n",
        chrono::Local::now().format("%Y-%m-%d_%H-%M-%S")
    );

    dinfo!("Program Start ({} ms)", p_st.elapsed().as_millis());

    dinfo!("Config Start");
    let now = std::time::Instant::now();
    let mut conf_str = String::new();
    fs::File::open("config/config.toml")?.read_to_string(&mut conf_str)?;
    let global_config: GlobalConfig = toml::from_str(conf_str.as_str())?;
    dinfo!("Config End ({} ms)", now.elapsed().as_millis());

    let mut exited = false;
    let event_loop = EventLoop::new();
    dinfo!("Event Loop Created");

    let window = WindowBuilder::new()
        .with_title("Too Many Dimensions")
        .build(&event_loop)
        .unwrap();
    dinfo!("Window Created");

    let now = std::time::Instant::now();
    dinfo!("State Start");
    let mut mastermind = Mastermind {
        resources: Resources::new(window, global_config),
        world: None,
    };
    mastermind.world = Some(Box::new(FlowWorld::new(&mastermind.resources)));
    dinfo!("State End ({} ms)", now.elapsed().as_millis());

    event_loop.run(move |event, _, control_flow| {
        mastermind.update(&event);
        if mastermind.resources.quit {
            *control_flow = ControlFlow::Exit;
            if !exited {
                dinfo!("Program Close ({} s)", p_st.elapsed().as_secs());
                exited = true;
            }
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
                if mastermind.resources.input.held_alt() {
                    mastermind.resources.toggle_fullscreen();
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
                mastermind.resources.toggle_fullscreen();
            }
            Event::RedrawRequested(_) => {
                mastermind.render();
            }
            Event::WindowEvent {
                event: WindowEvent::Resized(physical_size),
                ..
            } => {
                dinfo!("Window Resized");
                mastermind.resize(physical_size);
            }
            Event::MainEventsCleared => {
                mastermind.resources.window.request_redraw();
            }
            _ => {}
        }
    });
}
