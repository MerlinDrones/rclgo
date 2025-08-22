# Changelog

## [0.4.0] – 2025-08-XX

### Added
- **ROS Time / Clock API (Parity with rclcpp/rclpy)**
    - `rostime.Clock` with support for:
        - Wall time (`time.Now()`)
        - Simulated time via `/use_sim_time` parameter and `/clock` subscription
        - Methods: `Now()`, `NowMsg()`, `SleepUntil()`
    - `rostime.Rate` for time-aware loop execution
    - `/parameter_events` timestamps respect current time source
    - Explicit QoS profile for `/clock` subscription (BestEffort, Volatile)
- **Examples**
    - New `clock_demo` under `examples/` showing wall vs. sim time usage
    - Includes `params.yaml` and usage README
- **Documentation**
    - Added `docs/rostime.md` describing time concepts, usage, and a comparison of Wall vs. ROS Time

### Changed
- Integrated `params.Manager` with `rostime.Clock` to stamp parameter events with the active clock.

### Roadmap
- Time features now marked as **done** in `ROADMAP.md`.
- Next milestones: QoS validation & defaults, Actions, Lifecycle Nodes, Executors.

## [Unreleased]

### Added
- **Parameters API (Parity with rclcpp/rclpy)**
    - `params.Manager` with support for:
        - `Declare`, `Get`, `Set`, `Undeclare`, `Describe`
        - Per-parameter `Descriptor` constraints (bounds, read-only, allowed values)
        - `OnSet` callback for cross-field validation
    - Emits `/parameter_events` using `rcl_interfaces` types.
    - Integrated `/use_sim_time` hook to coordinate with ROS time/clock.
    - YAML preload (`params.LoadYAML`) for node-specific and global parameter files.
    - Full CLI interop: `ros2 param get/set/list/describe/undeclare` works against rclgo nodes.
- **QoS Policy surface parity with rclcpp**
    - Reliability, Durability, History, Depth, Liveliness, Deadline, Lifespan.
    - Default profile helpers (`NewDefaultQosProfile`, `NewDefaultServiceQosProfile`).

### Changed
- `param_demo` example updated:
    - Demonstrates declaring parameters, handling updates via `OnSet`, publishing events.
    - Optionally loads defaults from YAML when `PARAM_YAML` is set.

### Roadmap
- Marked Parameters as **done** in `ROADMAP.md`.
- QoS policies at feature parity (validation + event callbacks still pending).
- Next milestones: Actions, Lifecycle Nodes, ROS Time, Executors.

---

## [0.0.1] – 2025-08-XX
Initial public snapshot: package skeleton, message generation, minimal pub/sub.
