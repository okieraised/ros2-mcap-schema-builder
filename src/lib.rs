use anyhow::{Result, anyhow};
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

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

    pub fn resolve(&self, typename: &str) -> Result<String> {
        let path = self
            .resolved_map
            .get(typename)
            .ok_or_else(|| anyhow!("Definition not found for: {}", typename))?;
        Ok(fs::read_to_string(path)?.trim().to_string())
    }

    pub fn flatten(&self, root_type: &str) -> Result<String> {
        let mut visited = HashSet::new();
        let mut flat = vec![];
        let definition = self.resolve(root_type)?;
        flat.push(definition.clone());
        self.flatten_inner(root_type, &definition, &mut visited, &mut flat)?;
        Ok(flat.join("\n\n"))
    }

    fn resolve_custom_type(
        &self, raw: &str, current_package: &str,
    ) -> Option<String> {
        let base = strip_array_suffix(raw);

        let builtin_types = [
            "bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32",
            "uint32", "int64", "uint64", "float32", "float64", "string",
            "wstring",
        ];

        if builtin_types.contains(&base) {
            return None;
        }

        if base.contains('/') {
            if base.contains("/msg/") {
                Some(base.to_string())
            } else {
                let mut segs = base.split('/').collect::<Vec<_>>();
                if segs.len() == 2 {
                    Some(format!("{}/msg/{}", segs[0], segs[1]))
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
}

fn strip_array_suffix(s: &str) -> &str {
    let mut end = s.len();
    let mut result = s;

    loop {
        if let Some(start) = result.rfind('[') {
            if result.ends_with(']') && start < end - 1 {
                let inner = &result[start + 1..end - 1];
                if inner.is_empty() || inner.chars().all(|c| c.is_ascii_digit()) {
                    // Trim this suffix
                    end = start;
                    result = &result[..end];
                    continue; // Check if there are more suffixes
                }
            }
        }
        break; // No more valid suffixes found
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    const PREFIXES: &str = "/opt/ros/humble";

    #[test]
    fn test_register_and_flatten_tf() -> Result<()> {
        let mut resolver = CentralSchemaResolver::new();
        resolver.register_ros2_standard_paths(PREFIXES)?;

        let flat = resolver.flatten("tf2_msgs/msg/TFMessage")?;
        println!("{}", flat);
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
}
