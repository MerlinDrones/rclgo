# Changelog

All notable changes to this project will be documented in this file.
This project follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
and uses Semantic Versioning.

## [Unreleased]

- QoS event streams (liveliness, subscription matched, deadline/lifespan missed).
- Validation helpers (common “safe defaults” and topic-specific presets).
- Actions, Lifecycle Nodes, Executors (see `ROADMAP.md`).

---

## [0.3.0] – 2025-08-23

### Added
- **QoS Policy API**
    - New `qos` package with `qos.Profile` covering: History, Depth, Reliability, Durability, Liveliness, Deadline, Lifespan, and `AvoidROSNamespaceConventions`.
    - Helpers: `qos.NewDefaultQosProfile()` and `qos.NewDefaultServiceQosProfile()`.
- **C interop**
    - Safe C-side shim to fill `rmw_qos_profile_t` and cast enum values consistently (`qos/cinterop.go`).
    - `Profile.AsCStruct` / `(*Profile).FromCStruct` round-trip.
- **Tests**
    - `qos/cinterop_test.go` (pure-Go test that calls a tiny `//go:build cgo` helper).
    - `qos/profile_test.go` and `qos/profiles_test.go` for defaults, round-trips, and invariants.
- **Docs**
    - `docs/qos.md` with policy descriptions, examples, and interop notes.

### Changed
- **Breaking:** QoS types moved from `rclgo` to the new `qos` package.
    - Replace imports of `github.com/merlindrones/rclgo/pkg/rclgo` QoS symbols with:
      ```go
      import "github.com/merlindrones/rclgo/pkg/rclgo/qos"
      ```
    - Renames:
        - `rclgo.QosProfile` → `qos.Profile`
        - `rclgo.NewDefaultQosProfile` → `qos.NewDefaultQosProfile`
        - `rclgo.NewDefaultServiceQosProfile` → `qos.NewDefaultServiceQosProfile`
        - Policy constants like `HistoryKeepLast`, `ReliabilityReliable`, `DurabilityTransientLocal`, etc. now live under `qos`.

### Fixed
- cgo enum/typedef mismatches across compilation units when targeting ROS 2 Humble with GCC 11+ via explicit C-side casts.

---

## [0.2.0] – 2025-08-XX  _(tag: `0.2.0-params`)_

### Added
- **Parameters API (parity with rclcpp/rclpy)**
    - `params.Manager` with `Declare`, `Get`, `Set`, `Undeclare`, `Describe`.
    - Per-parameter `Descriptor` constraints (bounds, read-only, allowed values).
    - `OnSet` callback for cross-field validation.
    - Emits `/parameter_events` (`rcl_interfaces` types).
    - `/use_sim_time` hook to coordinate with ROS time/clock.
    - YAML preload (`params.LoadYAML`) for node-specific and global parameter files.
    - Full CLI interop: `ros2 param get/set/list/describe/undeclare` works against rclgo nodes.
- **QoS surface (initial)**
    - Reliability, Durability, History, Depth, Liveliness, Deadline, Lifespan.
    - Default profile helpers (`NewDefaultQosProfile`, `NewDefaultServiceQosProfile`).

### Changed
- `param_demo` example:
    - Demonstrates declarations, `OnSet` validation, and publishing events.
    - Optional YAML defaults via `PARAM_YAML`.

### Roadmap
- Parameters marked **done** in `ROADMAP.md`.
- QoS events/validation pending.
- Next milestones: Actions, Lifecycle Nodes, ROS Time, Executors.

---

## [0.0.1] – 2025-08-XX
Initial public snapshot: package skeleton, message generation, minimal pub/sub.
