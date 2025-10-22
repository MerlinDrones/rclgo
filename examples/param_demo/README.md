# Parameter Demo

Demonstrates ROS 2 parameter functionality in rclgo, including:
- Default parameter declaration
- Environment variable overrides
- YAML configuration loading
- CLI parameter overrides
- Dynamic parameter changes via `ros2 param` CLI

## Building

### As ROS 2 Package

```bash
colcon build --packages-select rclgo_param_demo_pkg
source install/setup.bash
```

### As Standalone Binary

```bash
cd examples/param_demo
source /opt/ros/humble/setup.bash
source ../../cgo-flags.env
go build -o param_demo
```

## Running

### Default Values Only

```bash
./param_demo
# Uses default values: fps=15, frame_id="camera"
```

### With Environment Variables

Set parameters via `RCL_PARAM_*` environment variables:

```bash
# Single parameter
RCL_PARAM_camera.fps=60 ./param_demo

# Multiple parameters
RCL_PARAM_camera.fps=60 \
RCL_PARAM_camera.frame_id=front_camera \
RCL_PARAM_camera.exposure=0.02 \
./param_demo
```

### With YAML Configuration

Create a config file `my_params.yaml`:

```yaml
param_demo:
  ros__parameters:
    camera.fps: 30
    camera.frame_id: "left_camera"
    camera.exposure: 0.015
```

Run with YAML:

```bash
./param_demo --params-file my_params.yaml
```

### With CLI Overrides (Highest Priority)

Override individual parameters from command line:

```bash
# Single override
./param_demo --ros-args -p camera.fps:=120 --

# Multiple overrides
./param_demo --ros-args \
  -p camera.fps:=120 \
  -p camera.frame_id:=right_camera \
  --
```

### Combining All Sources

Parameters are applied in priority order: **CLI > Env Vars > YAML > Defaults**

```bash
# Set different values from each source
RCL_PARAM_camera.fps=60 \
./param_demo \
  --params-file my_params.yaml \
  --ros-args -p camera.fps:=120 --

# Result: fps=120 (CLI wins over env var and YAML)
```

## ROS 2 CLI Interaction

While the node is running, interact with parameters using ROS 2 CLI:

```bash
# List all parameters
ros2 param list

# Get parameter value
ros2 param get /param_demo camera.fps

# Set parameter value (dynamic update)
ros2 param set /param_demo camera.fps 30

# Describe parameter (shows constraints)
ros2 param describe /param_demo camera.exposure

# Dump all parameters to YAML
ros2 param dump /param_demo > params.yaml
```

## Parameter Priority Examples

### Example 1: CLI Overrides Everything

```bash
# Default: fps=15
# YAML: fps=30
# Env: fps=60
# CLI: fps=120
RCL_PARAM_camera.fps=60 ./param_demo \
  --params-file config.yaml \
  --ros-args -p camera.fps:=120 --

# Output: fps=120 (CLI wins)
```

### Example 2: YAML Overrides Env Vars

```bash
# Default: fps=15
# Env: fps=60
# YAML: fps=30
# CLI: (none)
RCL_PARAM_camera.fps=60 ./param_demo --params-file config.yaml

# Output: fps=30 (YAML overrides env var)
```

### Example 3: Partial Overrides

```bash
# Set fps from env, frame_id from CLI
RCL_PARAM_camera.fps=60 ./param_demo \
  --ros-args -p camera.frame_id:=custom_camera --

# Output: fps=60 (from env), frame_id=custom_camera (from CLI)
```

## Expected Output

When running, you should see output like:

```
[param_demo] Applied CLI parameter overrides
[param_demo] camera.fps -> 30
[param_demo] camera.frame_id -> camera
[param_demo] camera.exposure -> 0.010000
```

The node will continue running, publishing parameter events and responding to `ros2 param` commands.
