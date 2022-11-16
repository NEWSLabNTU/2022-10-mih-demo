use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    pub namespace: String,
    pub topic: String,
    pub video_device: String,
    pub format: String,
    pub resolution: (u32, u32),
    pub interval: (u32, u32),
}
