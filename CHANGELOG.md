# Changelog

## [v0.6.1] – 2026-11-12

### Features
- refuse bounded sequences longer than their bound before sending

### Bug Fixes
- **params**: reject YAML lists that mix integers and floats, as rcl does
- **params, utils**: empty and mixed YAML lists, remapped nodes, NotSet env vars, timestamps past 2038
- **gogen**: overlay semantics, flag order, unreadable roots and config files
- **params, rostime**: post-set callbacks in registration order; Shutdown wins over an in-flight use_sim_time change
- **params**: a failed declaration of an overridden parameter leaves it undeclared
- **params**: a parameter section for "/name" applies only to the root namespace
- **params**: read values as rcl does (YAML 1.1 booleans, quoted arrays); one clock reading in ROSTimeNow
- **logging**: hold the logging mutex for the whole of rcutils_log and the level queries
- **params**: declare, undeclare and post-set like rclcpp; serialize use_sim_time switches
- **gogen**: resolve unqualified message types to their own package, as rosidl does
- log a failure to finalize a serialized message
- **params**: LoadYAML applies every matching section, like rcl's --params-file
- **logging**: recover panics in the logging output handler
- **params**: apply every matching parameter section in ApplyOverrides
- **gogen**: rename the Go-to-C conversion helpers so stale bindings don't compile
- **params**: flatten nested YAML maps into dotted parameter names
- **gogen**: free C memory that conversion replaces; clear empty sequences
- **qos**: negative durations are infinite; compatibility as rmw decides it
- every call on a closed entity matches ErrClosed; action watches end on Close
- serialize creating and finalizing entities on the same node
- **qos**: accept best-available policies, infinite durations and leases with automatic liveliness
- **clock**: wake Spin when a clock switches between system and simulated time
- smaller defects from review (default context, logging shutdown, timers, errors)
- refuse nil and wrong-type messages with an error instead of panicking
- **action**: a nil result from a type-safe action server no longer crashes the process
- **qos**: validate profiles when creating entities; zero action options take defaults
- **wait**: a WaitSet can run again after a Run that ended without a stop
- **action**: log a goal-status error even when the goal has finished
- clocks outlive the timers and action servers that use them; no cancel wait in WatchGoal
- **action**: answer result requests even if the goal's final state can't be set
- keep nodes and contexts alive while entities are created; Node.Close stops Node.Spin
- **action**: action client after Close, ordered handlers, failed goals
- **node**: serialize rcl_node_init and rcl_node_fini with the logging mutex
- **gogen**: string values and comments as rosidl reads them; name collisions; short paths
- **params**: let on-set callbacks read parameters; report dropped errors
- handle failing rcl calls on error paths; generate error types per run
- **gogen**: render bool and integer values as rosidl reads them
- **gogen**: report invalid interface definitions; test the generator end to end
- report unknown dynamic message types; make typemap safe for concurrent use
- **logging**: serialize rcl logging state like rclcpp's global logging mutex
- **rostime**: make simulated time work like rclcpp, including timers
- **gogen**: embed only the ROS install's paths in generated cgo flags
- **params**: make rclgo parameters work with ros2 param and other ROS clients
- **action**: report current goal status to watchers and never orphan a WatchGoal goal
- keep rcl error text with the goroutine that caused it; reject short CDR input
- make user calls safe against a concurrent Close
- **spin**: service each entity from one running wait set at a time
- **qos**: convert QoS durations to and from rmw's seconds and nanoseconds
- plug leaks and small error-handling defects in the core
- **params**: apply declared constraints and types to overridden parameters
- **timer**: keep a periodic timer's schedule when Spin services it
- make closing entities and contexts safe while Spin is running
- **spin**: leave nil-callback subscriptions and timers to their owner
- **action**: make the action server answer every request and shut down cleanly
- **spin**: service resources created after Spin has started
- **gogen**: convert the goal response stamp to seconds
- **examples**: tidy param_demo's go.mod so a plain go build works
- **build**: include paths for qos and params without CGO_CFLAGS
- **examples**: repair example modules so the examples tests build
- added test for command line args
- **node**: publish to /rosout from rclgo nodes
- **changelog**: strip [Unreleased] section when generating versioned release

### Performance
- **errors**: no stack trace for take-failed results; trim stack traces

### Documentation
- **fuzz**: correct the cause of FuzzDeserialize memory growth under Cyclone
- add concurrency, migration and rcl API coverage notes
- **testing**: eleven fault-sweep scenarios, return codes, domain reservation, new tests
- **params**: put undeclare's comment back on undeclare
- **roadmap**: action interop is tested; list features not implemented yet
- document override application, endpoint-info mangling and Spin panics; fix param_demo
- add a test coverage map and bring the roadmap up to date
- **qos**: warn that bursts of service requests overflow the default depth
- **changelog**: consolidate v0.6.0 release notes, remove Unreleased duplicate

### Tests
- **faultinject**: more scenarios and injected return codes
- **examples**: don't leave a ros2 daemon behind
- give ros2 CLI commands time to discover the node
- **params**: every parameter type over the services with rclpy; deleted events
- reserve a DDS domain per test process so parallel packages never share one
- **errors**: check errorsCastC against every case of its generated switch
- **services**: two clients, reordered, late and duplicate responses
- wider conversion fuzzing; generated defaults compared with rclpy
- string and wstring edge cases converted as rclpy converts them
- **params**: smoke test of an rclgo node started by ros2 launch
- harden the test infrastructure
- RCLGO_REQUIRE_ROS2 turns skips for missing ROS 2 dependencies into failures
- fuzz Go -> C -> Go conversion, converting twice into the same buffer
- **interop**: with Zenoh, every QoS pair delivers
- **fuzz**: commit seed corpora for the four fuzzers
- wait for conditions instead of settle sleeps where one exists
- **interop**: QoS compatibility matrix with rclpy in both directions
- edge values through serialization, topics, test_msgs services and actions
- runnable action example with checked output; test-rmw checks its Zenoh router
- run tests shuffled; t.Setenv; order-independent logging tests; real leak metric
- keep test packages off other ROS systems; robust timer drift check
- stop the ros2 daemon that a test's ros2 CLI call starts
- **action**: cancel all, cancel by stamp, cancel a finished goal, server gone, close from a callback
- create/close stress per RMW; rosout publisher under the node's entity lock
- **gogen**: bound checks for service requests and responses, goals and feedback
- check for leaked goroutines with goleak
- make every rcl call fail in turn; test signals; check coverage in CI
- run the suite on every installed RMW (Fast DDS, Cyclone DDS, Zenoh)
- add interop tests with rclcpp and rclpy
- add fuzz targets and benchmarks; run CI with the race detector
- add graph, node/context and message-fidelity suites
- **examples**: build the examples in a temporary copy

### Build
- **gen**: regenerate ROS 2 messages

### CI
- remove the old CI workflow, replaced by go-ci.yml on jazzy
- resync swarmos-devtools (go-ci module selection)
- resync swarmos-devtools (go test -short on push)
- add CI helper script ignored by .gitignore
- resync swarmos-devtools (pre-push base, +x hooks, public repos)
- adopt swarmos-devtools hooks, workflows and issue forms

### Style
- **params**: gofmt the test added in b448d84

### Chores
- bump VERSION to 0.6.1 for v0.6.1
- Merge pull request #77 from MerlinDrones/bugfix/fuzz-deserialize-comment
- Merge pull request #61 from MerlinDrones/bugfix/rclgo-7-tests-docs
- Merge pull request #60 from MerlinDrones/bugfix/rclgo-6-qos-params
- **deps**: go mod tidy (goleak is a direct dependency)
- Merge pull request #59 from MerlinDrones/bugfix/rclgo-5-codegen
- Merge pull request #58 from MerlinDrones/bugfix/rclgo-4-actions-logging
- Merge pull request #57 from MerlinDrones/bugfix/rclgo-3-errors-lifetimes
- Merge pull request #56 from MerlinDrones/bugfix/rclgo-2-spin-close
- Merge pull request #55 from MerlinDrones/bugfix/rclgo-1-build
- **msgs**: regenerate the internal/msgs compatibility shims
- **examples**: remove the committed param_demo binary
- add license headers to source files inherited from jazzy
- Merge pull request #63 from MerlinDrones/feature/devtools-sync
- Merge pull request #14 from kurieu-06/fix/rosout-publisher
- merge chore/fix-gitflow-scripts into jazzy
- removed changelog.sh from tracking
- added .gitflow-release, updated go.sum
- **release**: merge release/v0.6.0 to jazzy


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
