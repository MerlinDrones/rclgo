# Changelog

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
