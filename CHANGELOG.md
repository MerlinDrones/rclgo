# Changelog

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


## [Unreleased]

### Features
- **params**: add environment variable parameter support (`RCL_PARAM_*`)
  - Scan environment for `RCL_PARAM_name=value` variables
  - Automatic type inference from string values
  - Support for scalar and array types with bracket syntax `[1,2,3]`
  - Follows ROS 2 standard pattern matching rclcpp/rclpy
- **params**: complete parameter priority system
  - Priority order: CLI > Env Vars > YAML > Defaults
  - YAML loader now sets existing parameters instead of failing
  - Full integration testing for all priority combinations

### Testing
- **params**: add comprehensive CLI parameter override tests
  - Test scalar types (bool, int, double, string)
  - Test array types (bool[], int[], double[], string[])
  - Test wildcard node matching (/**)
  - Test integration with YAML loading
  - Test nil/empty args edge cases
- **params**: add environment variable parameter tests
  - Test scalar and array type inference
  - Test type detection from existing parameters
  - Test dot notation in parameter names
  - Test empty environment (no-op case)
- **params**: add parameter source priority integration tests
  - Test complete priority chain (CLI > Env > YAML > Defaults)
  - Test partial overrides from different sources
  - Test realistic usage patterns with all sources

### Documentation
- **ROADMAP**: mark parameters API as 100% complete with full parity
  - Environment variable overrides complete
  - M1 (Foundations) milestone fully complete
- **params**: add comprehensive package README
  - Complete API documentation with examples
  - Document all parameter sources (defaults, env vars, YAML, CLI)
  - Priority order examples and best practices
  - ROS 2 CLI interaction examples
  - Parameter constraints and OnSet callbacks
- **param_demo**: enhance README with environment variable examples
  - Document all parameter source combinations
  - Add priority order examples
  - Include expected output and usage patterns

### Status
- ✅ **Parameters API: COMPLETE** - Full rclcpp/rclpy parity achieved

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
