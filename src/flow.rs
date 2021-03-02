use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

#[rustfmt::skip]
pub const FLOW_SHAPE_VERTICES: [Vertex; 4] = [
    Vertex { pos: [-0.02, -0.02], },
    Vertex { pos: [-0.02,  0.02], },
    Vertex { pos: [ 0.02,  0.02], },
    Vertex { pos: [ 0.02, -0.02], },
];

#[rustfmt::skip]
pub const FLOW_SHAPE_INDICES: [u16; 6] = [
    0, 1, 2,
    0, 2, 3,
];

// flow.comp
pub const MAX_NUM_FLOW: usize = 3_000_000;
pub const MAX_NUM_SPAW: usize = 200;
pub const MAX_NUM_MANI: usize = 200;
pub const MAX_NUM_ACCU: usize = 200;

fn zero_f32() -> f32 {
    0.0f32
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable, Serialize, Deserialize)]
#[serde(rename = "flow")]
/// Sim info
// Will probably have to add more in the future
pub struct Uniforms {
    #[serde(default = "zero_f32", skip)]
    /// Simulation delta time
    pub dt: f32,
    #[serde(rename = "flow_initial_count")]
    /// Global particle count
    pub ct: u32,
    // Transparency gets decided based on count //
    /*  Global Flow Field Settings
     */
    #[serde(rename = "flow_particle_acceleration")]
    /// Global acceleration factor
    pub flow_acc: f32,
    #[serde(rename = "flow_particle_jitter")]
    /// Global flow jitter scale
    pub flow_jit: f32,
    #[serde(rename = "flow_particle_max_speed")]
    /// Global max speed
    pub flow_max: f32,
    #[serde(rename = "flow_field_extent")]
    /// Max/Min xy before wrapping
    pub flow_ext: f32,
    #[serde(rename = "flow_field_scale")]
    /// Global flow field scale
    pub flow_scl: f32,
    /*  Global Collider Settings
     */
    #[serde(rename = "collider_scale")]
    /// Global size factor
    pub coll_scl: f32,
    /*  Global Manipulator Settings
     */
    #[serde(rename = "manipulator_initial_count")]
    /// Manipulator count
    pub mani_ct: u32,
    #[serde(rename = "manipulator_acceleration")]
    /// Global inner acceleration factor
    pub mani_acc: f32,
    #[serde(rename = "manipulator_max_speed")]
    /// Global inner speed factor
    pub mani_spd: f32,
    /*  Global Spawner Settings
     */
    #[serde(rename = "spawner_initial_count")]
    /// Spawner count
    pub spaw_ct: u32,
    #[serde(rename = "spawner_spawn_rate")]
    /// Global spawn rate factor
    pub spaw_rte: f32,
    #[serde(rename = "spawner_scale")]
    /// Global radius factor
    pub spaw_scl: f32,
    #[serde(rename = "spawner_particle_scale_variance")]
    /// Global scale variance factor
    pub spaw_var: f32,
    /*  Global Accumulator Settings
     */
    #[serde(rename = "accumulator_initial_count")]
    /// Accumulator count
    pub accu_ct: u32,
    #[serde(rename = "accumulator_resource_rate_factor")]
    /// Global resource rate factor
    pub accu_rte: f32,
    #[serde(rename = "accumulator_scale")]
    /// Global scale factor
    pub accu_scl: f32,
    // These two little fuckers like to mess things up
    // so we're gonna make them play nicely
    #[serde(rename = "flow_field_x")]
    /// Global flow field x
    pub flow_x: f32,
    #[serde(rename = "flow_field_y")]
    /// Global flow field y
    pub flow_y: f32,
    #[serde(rename = "spawner_particle_r")]
    /// Global default spawn color r
    pub spaw_r: f32,
    #[serde(rename = "spawner_particle_g")]
    /// Global default spawn color g
    pub spaw_g: f32,
    #[serde(rename = "spawner_particle_b")]
    /// Global default spawn color b
    pub spaw_b: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Atomic sim info
pub struct Atomics {
    /// Atomic particle count
    pub atom_ct: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Checking for collision.
/// All colliders are circles.
/// Separate buffers for
/// FlowManipulators and FlowCollectors
pub struct Collider {
    /// Collider position
    pub x: f32,
    pub y: f32,
    /// Collider collision radius
    pub r2: f32,
}

pub struct Colliders {
    pub x: wgpu::Buffer,
    pub y: wgpu::Buffer,
    pub r2: wgpu::Buffer,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle manipulator
pub struct Manipulator {
    // Has Flow Collider //
    /// Inner particle accel
    pub acc: f32,
    /// Inner particle speed
    pub spd: f32,
    /// 0 -> 2pi
    pub rot: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle spawner
pub struct Spawner {
    // Spawners do not use collider type
    // because they spawn particles.
    // They do not need to check for collisions,
    // meaning would only bloat the FlowCollider buffers.
    /// Spawn position
    pub x: f32,
    pub y: f32,
    /// Particle scale
    pub scl: f32,
    /// Variance of particle scale
    pub var: f32,
    /// Color of particles
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle spawner 'collider'
/// Effectively houses hot data
pub struct SpawnerCollider {
    /// Spawn rate
    pub rte: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle accumulator
pub struct Accumulator {
    // Has Flow Collider //
    /// Gain factor, larger = more
    pub rte: f32,
    // Not sure if flags are needed here
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow particle vertex
pub struct Vertex {
    pub pos: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
/// Flow sim particle
pub struct Particle {
    pub pos: [f32; 2],
    pub vel: [f32; 2],
}
