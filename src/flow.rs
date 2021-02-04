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

pub const MAX_NUM_FLOW: usize = 3_000_000;
pub const MAX_NUM_SPAW: usize = 1_000;
pub const MAX_NUM_MANI: usize = 1_000;
pub const MAX_NUM_ACCU: usize = 1_000;

#[rustfmt::skip]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Config {
    initial_count:              u32,

    flow_extent:                f32,
    flow_acceleration:          f32,
    flow_max_speed:             f32,
    flow_scale:                 f32,
    flow_jitter:                f32,
    flow_offset:               [f32; 2],
    
    collider_scale:             f32,
    
    initial_manipulator_count:  u32,
    manipulator_acceleration:   f32,
    manipulator_speed_factor:   f32,
    
    initial_spawner_count:      u32,
    spawner_spawn_rate:         f32,
    spawner_scale:              f32,
    spawner_variance:           f32,
    spawner_color:             [f32; 3],
    
    initial_accumulator_count:  u32,
    accumulator_rate:           f32,
    accumulator_scale:          f32,
}
impl From<Uniforms> for Config {
    fn from(uniforms: Uniforms) -> Self {
        Self {
            initial_count: uniforms.ct,
            flow_extent: uniforms.flow_ext,
            flow_acceleration: uniforms.flow_acc,
            flow_max_speed: uniforms.flow_max,
            flow_scale: uniforms.flow_scl,
            flow_jitter: uniforms.flow_jit,
            flow_offset: uniforms.flow_off,
            collider_scale: uniforms.coll_scl,
            initial_manipulator_count: uniforms.mani_ct,
            manipulator_acceleration: uniforms.mani_acc,
            manipulator_speed_factor: uniforms.mani_spd,
            initial_spawner_count: uniforms.spaw_ct,
            spawner_spawn_rate: uniforms.spaw_rte,
            spawner_scale: uniforms.spaw_scl,
            spawner_variance: uniforms.spaw_var,
            spawner_color: uniforms.spaw_col,
            initial_accumulator_count: uniforms.accu_ct,
            accumulator_rate: uniforms.accu_rte,
            accumulator_scale: uniforms.accu_scl,
        }
    }
}

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
    /// Global scale factor
    pub accu_scl:   f32,
    //TODO: Add missing fields
}
impl From<Config> for Uniforms {
    fn from(config: Config) -> Self {
        Self {
            dt: 0.0,
            ct: config.initial_count,
            flow_ext: config.flow_extent,
            flow_acc: config.flow_acceleration,
            flow_max: config.flow_max_speed,
            flow_scl: config.flow_scale,
            flow_jit: config.flow_jitter,
            flow_off: config.flow_offset,
            coll_scl: config.collider_scale,
            mani_ct: config.initial_manipulator_count,
            mani_acc: config.manipulator_acceleration,
            mani_spd: config.manipulator_speed_factor,
            spaw_ct: config.initial_spawner_count,
            spaw_rte: config.spawner_spawn_rate,
            spaw_scl: config.spawner_scale,
            spaw_var: config.spawner_variance,
            spaw_col: config.spawner_color,
            accu_ct: config.initial_accumulator_count,
            accu_rte: config.accumulator_rate,
            accu_scl: config.accumulator_scale,
        }
    }
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
    pub rad2:       f32,
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
/// Flow particle spawner 'collider'
/// Effectively houses hot data
pub struct SpawnerCollider {
    /// Spawn rate
    pub rte:        f32,
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
