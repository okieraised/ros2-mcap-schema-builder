# ros2-mcap-schema-builder
Generating MCAP schemas definitions from ROS 2 interfaces in pure Rust

A library in Rust to convert the ROS2 ```.msg``` files into a single, self-contained schema data to use with ```MCAP```.

### Example:
```rust
use anyhow::Result;
use std::path::Path;

fn main() -> Result<()> {
    let r = CentralSchemaResolver::global()?;

    let tf = r.flatten("tf2_msgs/msg/TFMessage")?;
    println!("{tf}");

    Ok(())
}
```
