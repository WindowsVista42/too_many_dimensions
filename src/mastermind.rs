use winit::event::Event;

use crate::resources::Resources;

pub struct Mastermind {
    pub resources: Resources,
    pub world: Option<Box<dyn World>>,
}

impl Mastermind {
    pub fn update(&mut self, event: &Event<()>) {
        self.resources.update(event);
        self.world
            .as_mut()
            .expect("World not initialized")
            .update(&self.resources);
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
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

pub trait World {
    fn resize(&mut self, resources: &Resources);
    fn update(&mut self, resources: &Resources);
    fn render(&mut self, resources: &Resources);
}
