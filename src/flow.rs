use bytemuck::{Pod, Zeroable};

#[rustfmt::skip]
pub const FLOW_SHAPE_VERTICES: [Vertex; 4] = [
    Vertex { pos: [-0.01, -0.01], },
    Vertex { pos: [-0.01,  0.01], },
    Vertex { pos: [ 0.01,  0.01], },
    Vertex { pos: [ 0.01, -0.01], },
];

#[rustfmt::skip]
pub const FLOW_SHAPE_INDICES: [u16; 6] = [
    0, 1, 2,
    0, 2, 3,
];

pub const MAX_NUM_FLOW: usize = 1_000_000; // MAX u32 SIZE
pub const NUM_FLOW: usize = 1_000_000;

pub const CONFIG: Config = Config {
    unif: Uniforms {
        dt: 0.0,
        ct: NUM_FLOW as u32,

        part_ext: 12.0,
        part_acc: 1.3,
        part_max: 0.5,

        flow_scl: 0.5,
        flow_off: [123.0, 934.0],

        coll_scl: 0.0,

        mani_acc: 1.5,
        mani_spd: 1.0,

        spaw_rte: 1.0,
        spaw_scl: 1.0,
        spaw_var: 1.0,
        spaw_col: [1.0, 1.0, 1.0],

        accu_rte: 1.0
    },
};

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
// Sim info
// Will probably have to add more in the future
pub struct Uniforms {
    pub dt:         f32,    // Simulation delta time
    pub ct:         u32,    // Global particle count
    // Transparency gets decided based on count //

    // Global Particle Settings
    pub part_ext:   f32,    // Max/Min xy before wrapping
    pub part_acc:   f32,    // Global acceleration factor
    pub part_max:   f32,    // Global max speed

    // Global Flow Field Settings
    pub flow_scl:   f32,    // Global flow field scale
    pub flow_off:  [f32; 2],    // Global flow field offset

    // Global Collider Settings
    pub coll_scl:   f32,    // Global size factor

    // Global Manipulator Settings
    pub mani_acc:   f32,    // Global inner acceleration factor
    pub mani_spd:   f32,    // Global inner speed factor

    // Global Spawner Settings
    pub spaw_rte:   f32,    // Global spawn rate factor
    pub spaw_scl:   f32,    // Global radius factor
    pub spaw_var:   f32,    // Global scale variance factor
    pub spaw_col:  [f32; 3],// Global default spawn color

    // Global Accumulator Settings
    pub accu_rte:   f32,    // Global resource rate factor
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
// Atomic sim info
pub struct Atomics {
    pub atom_ct:    u32,    // Atomic particle count
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
// Checking for collision.
// All colliders are circles.
// Separate buffers for
// FlowManipulators and FlowCollectors
pub struct Collider {
    pub pos:       [f32; 2],
    pub rad:        f32,
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Manipulator {
    // Has Flow Collider //
    pub acc:        f32,    // Inner particle accel
    pub spd:        f32,    // Inner particle speed
    pub rot:        f32,    // 0 -> 2pi
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Spawner {
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
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Accumulator {
    // Has Flow Collider //
    pub rte:        f32,    // Gain factor, larger = more
    // Not sure if flags are needed here
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub pos:       [f32; 2],
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Particle {
    pub pos:       [f32; 2],
    pub vel:       [f32; 2],
}

#[rustfmt::skip]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub unif: Uniforms,
}