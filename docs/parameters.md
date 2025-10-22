# Parameters in rclgo (Humble)

> **Note**: This is a quick reference guide. For comprehensive documentation including all parameter sources (CLI, environment variables, YAML, defaults) and priority ordering, see [`pkg/rclgo/params/README.md`](../pkg/rclgo/params/README.md).

This doc explains how to declare, read, update, and undeclare parameters in **rclgo**, and how they interact with ROS 2 tools.

## Quick start

1. Create a manager in your node:

```go
mgr, _ := params.NewManager(node)
```

2. Declare parameters (with optional constraints):

```go
_, _ = mgr.Declare("camera.fps",
    params.Value{Kind: params.KindInt64, Int64: 15},
    params.Descriptor{MinInt: i64(1), MaxInt: i64(120), Description: "Output frame rate"})
```

3. React to changes:

```go
mgr.OnSet(func(changes []params.Parameter) params.SetResult {
    // validate/apply
    return params.SetResult{Successful: true}
})
```

4. Use ROS 2 CLI:

```bash
ros2 param list
ros2 param get /param_demo camera.fps
ros2 param set /param_demo camera.fps 30
```

## Features

* **Services**: The manager registers parameter services (`GetParameters`, `SetParameters`, `ListParameters`, `DescribeParameters`, `UndeclareParameters`) and publishes `/parameter_events` using `rcl_interfaces` types.
* **Validation**: Per-parameter descriptor bounds are checked (`MinInt`, `MaxInt`, `AllowedStrings`, etc.); add cross-field validation in `OnSet`.
* **Descriptors**: `DescribeParameters` now includes type, description, read-only, and constraint metadata.
* **Undeclare**: Parameters can be removed with the `Undeclare` service.
* **Parameter Sources**: Full support for all parameter sources with priority order:
  - **CLI overrides**: `--ros-args -p param:=value` (highest priority)
  - **Environment variables**: `RCL_PARAM_name=value`
  - **YAML files**: `params.LoadYAML(mgr, nodeName, filePath)`
  - **Defaults**: `mgr.Declare(name, defaultValue, descriptor)` (lowest priority)
* **Threading**: Reads from your own goroutines should copy values into local fields; the manager itself is thread-safe.
* **Events**: `/parameter_events` emits on declare, set, and undeclare.
* **QoS**: QoS profiles for parameter events/services align with `rclcpp` defaults (transient local, reliable).

## Conventions

* Names are dot-separated (`camera.fps`, `planner.max_speed`).
* Keep read-only metadata (build info, hardware IDs) marked `ReadOnly`.
* Use arrays for lists (`KindStringArray`, `KindInt64Array`, etc.).

## Testing checklist

* `ros2 param list` shows your parameters.
* `ros2 param get` returns defaults.
* Out-of-range sets are rejected with your reason.
* `OnSet` fires on every accepted change.
* `/parameter_events` emits on declare, set, and undeclare.
* YAML files can preload parameters successfully.

## Status

✅ **Full rclcpp/rclpy parameter parity achieved** (as of v0.5.0)

See [`ROADMAP.md`](../ROADMAP.md) for implementation status and future enhancements.
