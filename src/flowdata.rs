use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
// Sim info
// Will probably have to add more in the future
pub struct FlowUniforms {
    pub dt:         f32,    // Simulation delta time
    pub ct:         u32,    // Global particle count
    // Transparency gets decided based on count //

    // Global Particle Settings
    pub part_ext:   f32,    // Max/Min xy before wrapping
    pub part_acc:   f32,    // Global acceleration factor
    pub part_max:   f32,    // Global max speed

    // Global Flow Field Settings
    pub flow_scl:   f32,    // Global flow field scale
    pub flow_off:   f32,    // Global flow field offset

    // Global Collider Settings
    pub coll_scl:   f32,    // Global size factor

    // Global Manipulator Settings
    pub mani_acc:   f32,    // Global inner acceleration factor
    pub mani_spd:   f32,    // Global inner speed factor

    // Global Spawner Settings
    pub spaw_rte:   f32,    // Global spawn rate factor
    pub spaw_scl:   f32,    // Global radius factor
    pub spaw_var:   f32,    // Global scale variance factor
    pub spaw_col:  [f32; 3], // Global default spawn color

    // Global Accumulator Settings
    pub accu_rte:   f32,    // Global resource rate factor
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
// Atomic sim info
pub struct FlowAtomics {
    pub atom_count: u32,    // Atomic particle count
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
// Checking for collision.
// All colliders are circles.
// Separate buffers for
// FlowManipulators and FlowCollectors
pub struct FlowCollider {
    pub pos:       [f32; 2],
    pub rad:        f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowManipulator {
    // Has Flow Collider //
    pub acc:        f32,    // Inner particle accel
    pub spd:        f32,    // Inner particle speed
    pub rot:        f32,    // 0 -> 2pi
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowSpawner {
    // Spawners do not use collider type
    // because they spawn particles.
    // They do not need to check for collisions,
    // meaning would only bloat the FlowCollider buffers.
    pub pos:       [f32; 2],// Spawn position
    pub rad:        f32,    // Spawn radius
    pub rte:        f32,    // Spawn rate
    pub scl:        f32,    // Particle scale
    pub var:        f32,    // Variance of particle scale
    pub col:       [f32; 3],// Color of particles
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowAccumulator {
    // Has Flow Collider //
    pub rte:        f32,    // Gain factor, larger = more
    // Not sure if flags are needed here
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowVertex {
    pub pos:       [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
#[rustfmt::skip]
pub struct FlowParticle {
    pub pos:       [f32; 2],
    pub vel:       [f32; 2],
}

unsafe impl Pod for FlowUniforms {}
unsafe impl Zeroable for FlowUniforms {}
unsafe impl Pod for FlowAtomics {}
unsafe impl Zeroable for FlowAtomics {}
unsafe impl Pod for FlowCollider {}
unsafe impl Zeroable for FlowCollider {}
unsafe impl Pod for FlowManipulator {}
unsafe impl Zeroable for FlowManipulator {}
unsafe impl Pod for FlowSpawner {}
unsafe impl Zeroable for FlowSpawner {}
unsafe impl Pod for FlowAccumulator {}
unsafe impl Zeroable for FlowAccumulator {}
unsafe impl Pod for FlowVertex {}
unsafe impl Zeroable for FlowVertex {}
unsafe impl Pod for FlowParticle {}
unsafe impl Zeroable for FlowParticle {}