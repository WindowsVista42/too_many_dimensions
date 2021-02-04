#![feature(const_ptr_offset_from, const_maybe_uninit_as_ptr, const_raw_ptr_deref)]

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

fn main() {
    println!("{:>5} ms | Game init", 0);
    let p_st = std::time::Instant::now();
    let event_loop = EventLoop::new();
    println!("{:>5} ms | Event Loop created", p_st.elapsed().as_millis());

    let window = WindowBuilder::new()
        .with_title("Too Many Dimensions")
        .build(&event_loop)
        .unwrap();
    println!("{:>5} ms | Window created", p_st.elapsed().as_millis());

    println!("{:>5} ms | Game state started", p_st.elapsed().as_millis());
    let now = std::time::Instant::now();
    let mut state = block_on(State::new(&window, 8));
    println!(
        "{:>5} ms | Game state finished ({} ms)",
        p_st.elapsed().as_millis(),
        now.elapsed().as_millis()
    );

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
