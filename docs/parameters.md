# Parameters in rclgo (Humble)

This doc explains how to declare, read, and update parameters in **rclgo**, and how they interact with ROS 2 tools.

## Quick start

1. Create a manager in your node:

```go
mgr, _ := params.NewManager(node)
````

2. Declare parameters (with optional constraints):

```go
_, _ = mgr.Declare("camera.fps", params.Value{Kind: params.KindInt64, Int64: 15},
    params.Descriptor{MinInt: i64(1), MaxInt: i64(120)})
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

## Design notes

* **Services**: The manager registers parameter services (Get/Set/List/Describe) and publishes `/parameter_events` using `rcl_interfaces` types.
* **Validation**: Per‑parameter descriptor bounds are checked; add cross‑field validation in `OnSet` and return `{Successful:false, Reason:"..."}` to reject.
* **Threading**: Reads from your own goroutines should copy values into local fields; the manager itself is thread‑safe.
* **Timestamps**: `ParameterEvent.Stamp` uses system time for MVP. When a Clock module is added, wire via `EnableUseSimTimeHook`.
* **Namespaces**: Services are global for MVP. You can later prefix with the node’s FQN without changing calling code.

## Conventions

* Names are dot‑separated (`camera.fps`, `planner.max_speed`).
* Keep read‑only metadata (build info, hardware IDs) marked `ReadOnly`.
* Use arrays for lists (`KindStringArray`, `KindInt64Array`, etc.).

## Testing checklist

* `ros2 param list` shows your parameters.
* `ros2 param get` returns defaults.
* Out‑of‑range sets are rejected with your reason.
* `OnSet` fires on every accepted change.
* `/parameter_events` emits on declare and on set.

## Roadmap hooks (future work)

* **YAML preload**: Load defaults from a file at startup (profile‑based).
* **Node‑scoped services**: Switch to FQN‑prefixed service names.
* **QoS**: Expose dedicated QoS profiles if needed.
* **CLI parity**: Add `SetParametersAtomically`/`UndeclareParameters` variants.

