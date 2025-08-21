# Changelog

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
