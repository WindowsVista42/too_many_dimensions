use crate::flow;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
// TODO: Add pretty printing
pub struct GlobalConfig {
    pub window: WindowConfig,
    pub flow: flow::Uniforms,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct WindowConfig {
    pub msaa: u32,
}
