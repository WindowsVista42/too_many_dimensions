use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
// Sim info
// Will probably have to add more in the future
pub struct FlowUniforms {
    pub dt:         f32,    // Simulation delta time
    pub count:      u32,    // Global particle count

    // Global Particle Settings
    pub part_ext:   f32,    // Max/Min xy before wrapping
    pub part_acc:   f32,    // Global acceleration factor
    pub part_max:   f32,    // Global max speed
    pub part_flag:  u16,    // Global flags

    // Global Flow Field Settings
    pub flow_scl:   f32,    // Global flow field scale
    pub flow_off:   f32,    // Global flow field offset
    pub flow_flag:  u16,    // Global flags

    // Global Collider Settings
    pub coll_scl:   f32,    // Global size factor
    pub coll_flag:  u16,    // Global flags

    // Global Manipulator Settings
    pub mani_acc:   f32,    // Global inner acceleration factor
    pub mani_spd:   f32,    // Global inner speed factor
    pub mani_flag:  u16,    // Global flags

    // Global Spawner Settings
    pub spaw_rate:  f32,    // Global spawn rate factor
    pub spaw_scl:   f32,    // Global radius factor
    pub spaw_var:   f32,    // Global scale variance factor
    pub spaw_col:  [u8; 3], // Global default spawn color
    pub spaw_flag:  u16,    // Global flags

    // Global Accumulator Settings
    pub accu_gain:  f32,    // Global resource gain factor
    pub accu_flag:  u16,    // Global flags
}
unsafe impl Pod for FlowUniforms {}
unsafe impl Zeroable for FlowUniforms {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
// Checking for collision.
// All colliders are circles.
// Separate buffers for
// FlowManipulators and FlowCollectors
pub struct FlowCollider {
    pub position:  [f32; 2],
    pub radius:     f32,
}
unsafe impl Pod for FlowCollider {}
unsafe impl Zeroable for FlowCollider {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowManipulator {
    // Has Flow Collider //
    pub accel:      f32,    // Inner particle accel
    pub speed:      f32,    // Inner particle speed
    pub rotation:   u16,    // Map 0 -> u16::MAX to 0 -> 2pi
    pub flags:      u8,     // Dictates manipulator characteristics
}
unsafe impl Pod for FlowManipulator {}
unsafe impl Zeroable for FlowManipulator {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowSpawner {
    // Spawners do not use collider type
    // because they spawn particles.
    // They do not need to check for collisions,
    // meaning would only bloat the FlowCollider buffers.
    pub position:  [f32; 2],// Spawn position
    pub radius:     f32,    // Spawn radius
    pub rate:       f32,    // Spawn rate
    pub scale:      f32,    // Particle scale
    pub variance:   f32,    // Variance of particle scale
    pub color:     [u8; 3], // Color of particles
    pub flags:      u8,     // Dictates spawner characteristics
}
unsafe impl Pod for FlowSpawner {}
unsafe impl Zeroable for FlowSpawner {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowAccumulator {
    // Has Flow Collider //
    pub gain:       f32,    // Gain factor, larger = more
    pub flags:      u8,     // Dictates accumulator characteristics
    // Not sure if flags are needed here
}
unsafe impl Pod for FlowAccumulator {}
unsafe impl Zeroable for FlowAccumulator {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FlowVertex {
    pub pos: [f32; 2],
}
unsafe impl Pod for FlowVertex {}
unsafe impl Zeroable for FlowVertex {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FlowParticle {
    pub pos: [f32; 2],
    pub vel: [f32; 2],
}
unsafe impl Pod for FlowParticle {}
unsafe impl Zeroable for FlowParticle {}
