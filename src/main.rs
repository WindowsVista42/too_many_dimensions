#![feature(const_ptr_offset_from, const_maybe_uninit_as_ptr, const_raw_ptr_deref)]

macro_rules! debug_info {
    ($($arg:tt)*) => {
        log!(log::Level::Info, "{}, {}:{}:{}", format!($($arg)*), file!(), line!(), column!());
    };
}

#[macro_use]
extern crate log;

mod flow;
mod state;
mod view;

use pollster::block_on;
use state::*;
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::{Window, WindowBuilder};

/// Toggle windows fullscreen setting when called
fn toggle_fullscreen(state: &mut State, window: &Window) {
    if state.fullscreen {
        window.set_fullscreen(None);
        state.fullscreen = false;
    } else {
        window.set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));
        state.fullscreen = true;
    }
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
        flexi_logger::Cleanup::KeepLogAndCompressedFiles(2, 15),
    )
    .create_symlink("current_run")
    .start()?;

    debug!("{}", chrono::Local::now().format("%Y-%m-%d_%H-%M-%S"));
    debug_info!("Program Start");

    let mut exited = false;

    let event_loop = EventLoop::new();
    debug_info!("Event Loop Created");

    let window = WindowBuilder::new()
        .with_title("Too Many Dimensions")
        .build(&event_loop)
        .unwrap();
    debug_info!("Window Created");

    let now = std::time::Instant::now();
    debug_info!("State Start");
    let mut state = block_on(State::new(&window, 8));
    debug_info!("State End ({} ms)", now.elapsed().as_millis());

    event_loop.run(move |event, _, control_flow| {
        state.update(&event);
        if state.quit {
            *control_flow = ControlFlow::Exit;
            if !exited {
                debug_info!("Program Close ({} s)", p_st.elapsed().as_secs());
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
