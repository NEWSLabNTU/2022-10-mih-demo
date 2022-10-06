use serde::Deserialize;
use serde_semver::SemverReq;

#[derive(Debug, Clone, SemverReq)]
#[version("0.1.0")]
pub struct Version;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    /// Config format version.
    pub version: Version,

    /// ROS Namespace.
    pub namespace: String,

    /// Input topic for point cloud.
    pub pcd_topic: String,

    /// Input topic for detected objects.
    pub det_topic: String,
}
