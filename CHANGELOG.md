# Changelog

## [v0.6.0] – 2026-05-06

### ⚠️ Breaking Change: ROS 2 Jazzy Only

**This release drops support for ROS 2 Humble. The minimum supported ROS 2 version is now Jazzy (LTS).**

All library paths, CGO flags, and generated bindings target `/opt/ros/jazzy`. If you are still on Humble, stay on v0.5.x.

### Features
- **jazzy**: port rclgo to ROS 2 Jazzy — new `rcl_timer_init2` API (adds `autostart` param), `RequestID.WriterGUID` type changed to `[16]byte`, new QoS constants (`BestAvailable` reliability/durability/liveliness) and `NewBestAvailableProfile()` helper
- **jazzy**: new generated message packages: `service_msgs`, `type_description_interfaces`, `rosidl_dynamic_typesupport`
- **jazzy**: add Jazzy compatibility test suite (`Taskfile.yml` in tests)

### Build
- **gen_msgs**: remove `--ignore-ros-distro-mismatch` flag (not needed on Jazzy)
- **gen_cgo_flags, gen_msgs**: switch shebang from `sh` to `bash`
- **qos**: update fallback CGO flags to `/opt/ros/jazzy` paths
- **gogen**: fix generator output path for `tests/gogen/flags.gen.go`
- **msgs**: regenerate all message bindings for Jazzy

### Bug Fixes
- **test_script_launch**: use `$ROS_DISTRO` env var instead of hardcoded path

### Documentation
- Replace all `humble` path references with `jazzy` or `$ROS_DISTRO` throughout docs, READMEs, and examples
- Update ROADMAP, overview, and parameter docs to target Jazzy


## [v0.5.1] – 2025-11-17

(no user-facing changes)


## [v0.5.1] – 2025-11-17

(no user-facing changes)


## [v0.5.0] – 2025-10-22

### Features
- merge params
- **param_demo**: add environment variable support
- **params**: add environment variable parameter support (RCL_PARAM_*)
- **logging**: merge logging improvements from feature/logging
- **logging**: add realtime logging support and fix log flush on exit

### Bug Fixes
- **tests**: resolve absolute paths in launch file for ExecuteProcess

### Documentation
- **CHANGELOG**: add unreleased section for params feature completion
- **params**: add comprehensive documentation and usage examples
- **ROADMAP**: mark parameters as complete with full parity
- **logging**: add comprehensive logging improvements summary
- **tests**: finalize test results and add production warnings

### Tests
- **params**: add priority integration tests and fix YAML override behavior
- **params**: add comprehensive tests for CLI parameter overrides

### Build
- **gen**: regenerate ROS 2 messages

### Chores
- **gitignore**: ignore go.work files (managed at parent level)
- bump VERSION to 0.5.0 for v0.5.0
- **repo**: housekeeping cleanup for v0.5.0 release
- add commit message template and ignore local AI context
- Merge tag 'v0.4.1' into humble


## [v0.4.1] – 2025-10-19

### Features
- **logging**: replace `rcl_logging_configure_with_output_handler` with `rcl_logging_configure` for improved backend initialization
- **param_demo**: enhance argument parsing and support CLI parameter overrides
- **examples**: add `rclgo_param_demo_pkg` example package
- **params**: add `ApplyOverrides` method to Manager
- add `px4_msgs` to message generation and update import paths to `pkg/msgs`
- **params**: add `DeclareIfMissing` method to Manager

### Documentation
- **ROADMAP**: update progress on parameters and logging

### Build
- **gen**: regenerate ROS 2 messages

### Chores
- **examples**: update golang.org/x/tools dependencies
- **deps**: update golang.org/x/tools to fix build issue
- bump VERSION to 0.4.1 for v0.4.1
- **examples**: remove local `rclgo` replacements from `go.mod` files
- **examples**: update `go.mod` to replace `rclgo` with local path for development
- update .gitignore to include CLAUDE.md and TODO.md
- Merge tag 'v0.4.0' into humble


## [v0.4.0] – 2025-08-26

### Features
- add shimgen tool for generating alias shims for ROS message packages

### Refactoring
- update imports to reflect `/internal` to `/pkg` migration in unit tests and implementation
- moved msgs from /internal to /pkg
- update message generation paths to pkg/msgs

### Chores
- bump VERSION to 0.4.0 for v0.4.0
- Merge tag 'v0.3.0' into humble


## [Unreleased] – 2025-08-23

### Chores

- add changelog.sh script for generating changelog sections
## [0.3.0] – 2025-08-23

### Features

- **qos**: qos changes
- **rostime**: rosgraph for rostime package
- **qos**: qos profile functionality
- **qos**: qos.md
- **rostime**: add ROS time and clock support

### Build

- **gen**: regenerate code
- **gen**: regenerate code

### Chores

- bump VERSION to 0.3.0 for release
- reset VERSION
- bumped minor
- bump VERSION to 0.1.0 for release
- Added to help IDE run certain tests
- Added as helper to generate messages
- Updated for qos
- Updated rostime
- Added new changes from commits

## [0.2.0] – 2025-08-21

### Features

- **params**: ros2 parameter implementation

### Chores

- bump VERSION to 0.2.0 for release
- reset VERSION
