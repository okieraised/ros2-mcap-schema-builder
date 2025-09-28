use anyhow::{Result, anyhow};
use once_cell::sync::OnceCell;
use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::{env, fs};

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";

pub static GLOBAL_RESOLVER: OnceCell<CentralSchemaResolver> = OnceCell::new();

#[derive(Debug, Default)]
pub struct CentralSchemaResolver {
    resolved_map: BTreeMap<String, PathBuf>,
    search_paths: Vec<PathBuf>,
}

impl CentralSchemaResolver {
    pub fn new() -> Self {
        Self {
            resolved_map: BTreeMap::new(),
            search_paths: vec![],
        }
    }

    pub fn global() -> Result<&'static CentralSchemaResolver> {
        GLOBAL_RESOLVER.get_or_try_init(|| {
            let mut r = CentralSchemaResolver::new();
            r.register_ros2_from_env()?;
            if r.resolved_map.is_empty() {
                return Err(anyhow!(
                    "No .msg files found under AMENT_PREFIX_PATH."
                ));
            }
            Ok(r)
        })
    }

    pub fn register_msg_dir(
        &mut self, package: &str, path: PathBuf,
    ) -> Result<()> {
        for entry in fs::read_dir(&path)? {
            let entry = entry?;
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("msg") {
                let msg_name = path.file_stem().unwrap().to_string_lossy();
                let key = format!("{}/msg/{}", package, msg_name);
                self.resolved_map.insert(key, path);
            }
        }
        Ok(())
    }

    pub fn register_ros2_from_env(&mut self) -> Result<()> {
        let os = env::var_os(AMENT_PREFIX_PATH)
            .ok_or_else(|| anyhow!("AMENT_PREFIX_PATH is not set"))?;

        self.register_ros2_standard_paths_any(env::split_paths(&os))
    }

    pub fn resolve(&self, typename: &str) -> Result<String> {
        let path = self
            .resolved_map
            .get(typename)
            .ok_or_else(|| anyhow!("Definition not found for: {}", typename))?;

        Ok(fs::read_to_string(path)?.trim().to_string())
    }

    pub fn register_ros2_standard_paths(
        &mut self, ament_prefixes: &str,
    ) -> Result<()> {
        for prefix in ament_prefixes.split(':') {
            let share_dir = Path::new(prefix).join("share");
            if !share_dir.exists() {
                continue;
            }

            for entry in fs::read_dir(&share_dir)? {
                let entry = entry?;
                let package_dir = entry.path();
                if package_dir.is_dir() {
                    let msg_dir = package_dir.join("msg");
                    if msg_dir.is_dir() {
                        let package_name = package_dir
                            .file_name()
                            .unwrap_or_default()
                            .to_string_lossy()
                            .to_string();
                        self.register_msg_dir(&package_name, msg_dir)?;
                    }
                }
            }
        }
        Ok(())
    }

    fn register_ros2_standard_paths_any<I>(&mut self, prefixes: I) -> Result<()>
    where
        I: IntoIterator<Item = PathBuf>,
    {
        let mut prefixes: Vec<PathBuf> = prefixes.into_iter().collect();
        prefixes.sort();
        for prefix in prefixes {
            println!("cargo:rerun-if-changed={}", prefix.display());

            let share_dir = prefix.join("share");
            if !share_dir.exists() {
                continue;
            }
            let mut packages: Vec<_> = fs::read_dir(&share_dir)?
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            packages.sort();

            for package_dir in packages {
                let msg_dir = package_dir.join("msg");
                if msg_dir.is_dir() {
                    let package_name = package_dir
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or_default()
                        .to_string();
                    self.register_msg_dir(&package_name, msg_dir)?;
                }
            }
        }
        Ok(())
    }

    pub fn flatten(&self, root_type: &str) -> Result<String> {
        let mut visited = HashSet::new();
        let mut flat = vec![];
        let definition = self.resolve(root_type)?;
        flat.push(definition.clone());
        self.flatten_inner(root_type, &definition, &mut visited, &mut flat)?;
        Ok(flat.join("\n\n"))
    }

    fn is_builtin_type(&self, raw: &str) -> bool {
        // Remove any array suffixes: [], [N], [<=N]
        let base = strip_array_suffix(raw);

        match base {
            "bool" | "byte" | "char" | "int8" | "uint8" | "int16" | "uint16"
            | "int32" | "uint32" | "int64" | "uint64" | "float32" | "float64"
            | "string" | "wstring" => return true,
            _ => {},
        }

        // Bounded strings: string<=N / wstring<=N
        if let Some(rest) = base.strip_prefix("string") {
            return rest.is_empty()
                || (rest.starts_with("<=")
                    && rest[2..].chars().all(|c| c.is_ascii_digit()));
        }
        if let Some(rest) = base.strip_prefix("wstring") {
            return rest.is_empty()
                || (rest.starts_with("<=")
                    && rest[2..].chars().all(|c| c.is_ascii_digit()));
        }

        false
    }

    fn resolve_custom_type(
        &self, raw: &str, current_package: &str,
    ) -> Option<String> {
        if self.is_builtin_type(raw) {
            return None; // don't recurse for builtins (incl. string<=N)
        }

        let base = strip_array_suffix(raw);

        if base.contains('/') {
            if base.contains("/msg/") {
                Some(base.to_string())
            } else {
                let segments = base.split('/').collect::<Vec<_>>();
                if segments.len() == 2 {
                    Some(format!("{}/msg/{}", segments[0], segments[1]))
                } else {
                    None
                }
            }
        } else {
            Some(format!("{}/msg/{}", current_package, base))
        }
    }

    fn flatten_inner(
        &self, current_type: &str, definition: &str,
        visited: &mut HashSet<String>, flat: &mut Vec<String>,
    ) -> Result<()> {
        visited.insert(current_type.to_string());
        let current_package = current_type.split('/').next().unwrap_or("");

        for line in definition.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 2 {
                continue;
            }
            let typename = parts[0];
            if let Some(nested_type) =
                self.resolve_custom_type(typename, current_package)
            {
                let typename = nested_type.replace("/msg/", "/");
                if !visited.contains(&nested_type) {
                    let nested_def = self.resolve(&nested_type)?;
                    flat.push(format!(
                        "================================================================================\nMSG: {}\n{}",
                        typename,
                        nested_def.trim()
                    ));
                    self.flatten_inner(&nested_type, &nested_def, visited, flat)?;
                }
            }
        }

        Ok(())
    }

    pub fn generate_all_raw(&self) -> Result<BTreeMap<String, String>> {
        let mut out = BTreeMap::new();
        for key in self.resolved_map.keys() {
            out.insert(key.clone(), self.resolve(key)?);
        }
        Ok(out)
    }

    /// Build a map of every known message → flattened text (root + deps).
    pub fn generate_all_flattened(&self) -> Result<BTreeMap<String, String>> {
        let mut out = BTreeMap::new();
        for key in self.resolved_map.keys() {
            println!("key: {}", key.clone());

            out.insert(key.clone(), self.flatten(key)?);
        }
        Ok(out)
    }
}

/// Strips ROS 2 `.msg` suffixes from a type and returns a subslice of the input.
///
/// This removes one or more **trailing** array designators:
/// - `[]`     — unbounded array
/// - `[N]`    — fixed-size array (where `N` is digits)
/// - `[<=N]`  — bounded array with an upper limit
///
/// It trims valid array suffixes from the **end** of the string
/// until no more are found, then returns the remaining base type. It does **not**
/// allocate and does not modify the input
///
/// It does **not** touch non-array qualifiers like bounded strings (`string<=256`);
/// for example, `string<=256[<=8]` becomes `string<=256`.
///
/// Invalid or non-matching bracket tails are left intact (the function stops trimming
/// when it encounters something that is not a valid array suffix).
fn strip_array_suffix(mut s: &str) -> &str {
    loop {
        let Some(lb) = s.rfind('[') else {
            break;
        };
        if !s.ends_with(']') || lb == 0 {
            break;
        }

        let inner = s[lb + 1..s.len() - 1].trim();
        let ok = inner.is_empty()
            || inner.chars().all(|c| c.is_ascii_digit())
            || (inner.starts_with("<=")
                && inner[2..].trim().chars().all(|c| c.is_ascii_digit()));

        if ok {
            s = &s[..lb];
        } else {
            break;
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    const PREFIXES: &str = "/opt/ros/jazzy";

    #[test]
    fn test_strip_array_suffix() {
        println!("{}", strip_array_suffix("FloatingPointRange[<=1]"));
        println!("{}", strip_array_suffix("int32[]"));
    }

    #[test]
    fn test_register_and_flatten_tf() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let expected = r#"geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://docs.ros.org/en/rolling/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component. 
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1"#;

        let flat = resolver.flatten("tf2_msgs/msg/TFMessage")?;
        assert_eq!(flat, expected);

        Ok(())
    }

    #[test]
    fn test_register_and_flatten_image() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("sensor_msgs/msg/CompressedImage")?;
        println!("{}", flat);
        Ok(())
    }

    #[test]
    fn test_register_joint_msg() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("std_msgs/msg/Float64MultiArray")?;
        println!("{}", flat);
        Ok(())
    }

    #[test]
    fn test_register_battery() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("sensor_msgs/msg/BatteryState")?;
        println!("{}", flat);
        Ok(())
    }

    #[test]
    fn test_register_lowstate() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("unitree_hg/msg/LowState")?;
        println!("{}", flat);
        Ok(())
    }

    #[test]
    fn test_register_floating_point_range() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("rcl_interfaces/msg/FloatingPointRange")?;
        println!("{}", flat);
        Ok(())
    }

    #[test]
    fn test_register_all() -> Result<()> {
        let resolver = CentralSchemaResolver::global()?;
        // let raw_map = resolver.generate_all_raw()?;
        let flat_map = resolver.generate_all_flattened()?;

        Ok(())
    }
}
