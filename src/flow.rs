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

#[rustfmt::skip]
/// Default values for flow sim
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
/// Sim info
// Will probably have to add more in the future
pub struct Uniforms {
    /// Simulation delta time
    pub dt:         f32,
    /// Global particle count
    pub ct:         u32,
    // Transparency gets decided based on count //

    // Global Flow Field Settings
    /// Max/Min xy before wrapping
    pub flow_ext:   f32,
    /// Global acceleration factor
    pub flow_acc:   f32,
    /// Global max speed
    pub flow_max:   f32,
    /// Global flow field scale
    pub flow_scl:   f32,
    /// Global flow jitter scale
    pub flow_jit:   f32,
    /// Global flow field offset
    pub flow_off:  [f32; 2],

    // Global Collider Settings
    /// Global size factor
    pub coll_scl:   f32,

    // Global Manipulator Settings
    /// Manipulator count
    pub mani_ct:    u32,
    /// Global inner acceleration factor
    pub mani_acc:   f32,
    /// Global inner speed factor
    pub mani_spd:   f32,

    // Global Spawner Settings
    /// Spawner count
    pub spaw_ct:    u32,
    /// Global spawn rate factor
    pub spaw_rte:   f32,
    /// Global radius factor
    pub spaw_scl:   f32,
    /// Global scale variance factor
    pub spaw_var:   f32,
    /// Global default spawn color
    pub spaw_col:  [f32; 3],

    // Global Accumulator Settings
    /// Accumulator count
    pub accu_ct:    u32,
    /// Global resource rate factor
    pub accu_rte:   f32,
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Atomic sim info
pub struct Atomics {
    /// Atomic particle count
    pub atom_ct:    u32,    
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Checking for collision.
/// All colliders are circles.
/// Separate buffers for
/// FlowManipulators and FlowCollectors
pub struct Collider {
    /// Collider position
    pub pos:       [f32; 2],
    /// Collider collision radius
    pub rad2:       f32,
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle manipulator
pub struct Manipulator {
    // Has Flow Collider //
    /// Inner particle accel
    pub acc:        f32,
    /// Inner particle speed
    pub spd:        f32,
    /// 0 -> 2pi 
    pub rot:        f32,
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle spawner
pub struct Spawner {
    // Spawners do not use collider type
    // because they spawn particles.
    // They do not need to check for collisions,
    // meaning would only bloat the FlowCollider buffers.
    /// Spawn position
    pub pos:       [f32; 2],
    /// Spawn radius
    pub rad:        f32,
    /// Spawn rate
    pub rte:        f32,
    /// Particle scale
    pub scl:        f32,
    /// Variance of particle scale
    pub var:        f32,
    /// Color of particles
    pub col:       [f32; 3],
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle accumulator
pub struct Accumulator {
    // Has Flow Collider //
    /// Gain factor, larger = more
    pub rte:        f32,
    // Not sure if flags are needed here
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle vertex
pub struct Vertex {
    pub pos:       [f32; 2],
}

#[repr(C)]
#[rustfmt::skip]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow sim particle
pub struct Particle {
    pub pos:       [f32; 2],
    pub vel:       [f32; 2],
}

#[rustfmt::skip]
#[derive(Copy, Clone, Debug)]
/// Config data for flow sim
pub struct Config {
    pub uniforms: Uniforms,
}
