use crate::resources::Resources;
use crate::threads::ThreadPool;
use once_cell::sync::Lazy;
use std::cell::UnsafeCell;
use std::fmt;
use std::fmt::Debug;

static GLOBAL_THREAD_POOL: Lazy<ThreadPool> = Lazy::new(|| ThreadPool::new(num_cpus::get()));

#[macro_export]
macro_rules! dinfo {
    ($($arg:tt)*) => {
        log!(log::Level::Info, "{}\nLOCATION : {}:{}:{}\n", format!($($arg)*), file!(), line!(), column!());
    };
}

pub struct Mastermind {
    pub resources: Resources,
    pub world: Option<Box<dyn World>>,
}

impl Mastermind {
    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.resources.resize(new_size);
        let world = self.world.as_mut().expect("World not initialized");
        world.run_resize(&self.resources);
        world.run_render(&self.resources);
    }

    pub fn update(&mut self) {
        self.resources.render();

        let world = self.world.as_mut().expect("World not initialized").as_mut();
        world.run_update(&self.resources);
        world.run_render(&self.resources);
    }
}

/// Trait for world types, this is intended to be used with MultiRef
/// for *dangerous* multiple mutable references across threads.
/// SAFETY: Systems must be MANUALLY CHECKED for aliasing and data races.
pub unsafe trait World {
    fn run_resize(&self, resources: &Resources);
    /// Call get_mut() to get as many mutable references as needed
    /// MUST MANUALLY CHECK for aliasing and data races
    fn run_update(&self, resources: &Resources);
    /// Call get_mut() to get as many mutable references as needed
    /// MUST MANUALLY CHECK for aliasing and data races
    fn run_render(&self, resources: &Resources);
}

/// Convenience trait for dangerous things.
pub unsafe trait MultiRef<'a, T> {
    /// Grabs a ref from the inner UnsafeCell
    fn get_ref(&self) -> &'a T;
    /// Grabs a mut ref from the inner UnsafeCell
    /// MUST MANUALLY CHECK for aliasing and data races.
    fn get_mut(&self) -> &'a mut T;
}

pub fn spawn<'a>(f: impl FnOnce() + Send + Sync + 'a) {
    let f = Box::new(f);
    GLOBAL_THREAD_POOL.execute(|| f());
}

pub fn join() {
    GLOBAL_THREAD_POOL.join();
}

pub struct Executor<T: Sized> {
    pub world: UnsafeCell<T>,
}

unsafe impl<'a, T> MultiRef<'a, T> for Executor<T> {
    fn get_ref(&self) -> &'a T {
        unsafe { &*self.world.get() }
    }

    fn get_mut(&self) -> &'a mut T {
        unsafe { &mut *self.world.get() }
    }
}

impl<T> Debug for Executor<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let world = self.get_mut();
        world.fmt(f)
    }
}

unsafe impl<T> Send for Executor<T> {}
unsafe impl<T> Sync for Executor<T> {}
