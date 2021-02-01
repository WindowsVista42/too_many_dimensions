#![feature(const_ptr_offset_from, const_maybe_uninit_as_ptr, const_raw_ptr_deref)]

mod flow;
mod view;
mod state;

use futures::executor::block_on;
use state::*;
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::{Window, WindowBuilder};

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
