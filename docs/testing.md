# Testing rclgo

rclgo is a ROS 2 client library in its own right, like rclcpp and rclpy: it calls rcl and rcl_action (C) directly
through cgo and implements the client-library layer (Spin, entity lifetimes, the action server, parameter services)
in Go. Its tests are therefore modelled on the rclcpp and rclpy test suites for the same areas. This page lists what
is covered, how to run it, and what is not covered yet.

## Running the tests

```bash
source /opt/ros/jazzy/setup.bash
go test -count=1 ./...          # what CI runs (make test)
go test -race -count=1 ./...    # make test-race; what CI runs. Most defects found were concurrency defects
```

- With a Go workspace (`go.work`) that does not contain rclgo, set `GOWORK=off`.
- `TestMain` (`pkg/rclgo/rcl_test.go`) moves the test binary to an unused DDS domain, so tests do not see other ROS
  traffic on the machine. For extra isolation set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- `examples/` builds and runs the example programs (`go generate`, `colcon build`), so it needs `colcon`.
- The interop and `ros2` CLI tests start rclcpp/rclpy programs (`demo_nodes_cpp`, `demo_nodes_py`,
  `action_tutorials_cpp`, rclpy scripts in `pkg/rclgo/testdata/interop/`) as separate processes. They skip when the
  `ros2` tools, a package or rclpy is missing, and with `-short`.
- Memory errors in cgo code often go unnoticed because freed memory still holds the old bytes. To make use-after-free
  visible, run a compiled test binary under glibc's checks:

  ```bash
  go test -c -o /tmp/rclgo.test ./pkg/rclgo/
  cd pkg/rclgo && MALLOC_PERTURB_=165 MALLOC_CHECK_=3 /tmp/rclgo.test -test.run 'Close|Racing' -test.count=10
  ```

## Every middleware (RMW)

`make test-rmw` (`tools/test-rmw.sh`) runs the suite once per installed RMW, as rcl's
`call_for_each_rmw_implementation` does: Fast DDS (the Jazzy default), Cyclone DDS, Connext and Zenoh, whichever are
installed (`RMWS="..."` picks a subset; extra arguments go to `go test`). For Zenoh it starts a router that listens on
`127.0.0.1` only and stops it afterwards. The interop tests then run against rclcpp/rclpy nodes on the same RMW.

Known upstream issues, reproduced with plain rcl in C (so not rclgo defects); the affected tests skip under that RMW:

| RMW | Issue | Skipped test |
|---|---|---|
| rmw_zenoh_cpp 0.2.11 | Many contexts shut down concurrently: the process aborts (uncaught `zenoh::ZException`, "Failed to close the session") | `TestNodesCreatedAndClosedConcurrently` |
| rmw_zenoh_cpp 0.2.11 | Creating and destroying subscriptions on a topic while another thread publishes on it: `rcl_publish` never returns | `TestEntityCallsRacingClose` |
| rmw_fastrtps_cpp 8.4.4 | `rmw_deserialize` crashes on a truncated buffer (also from rclpy) | `FuzzDeserialize` (fuzzing only) |

Zenoh also differs from DDS where ROS leaves behaviour open, and the tests allow for it: the order of a topic's types
in graph queries, and matching (and counting) endpoints whose durability is incompatible under DDS rules.

## Go-specific checks

These come from Go's runtime and cgo rather than from ROS, so the rclcpp/rclpy suites have no counterpart.

| Check | How | Covered by |
|---|---|---|
| rcl error text stays with its goroutine (rcl's error state is per OS thread) | always on | `TestRCLErrorTextBelongsToItsCall` |
| Many contexts in parallel goroutines | always on | `TestContextsInParallel` |
| No goroutine left after everything is closed | always on | `TestNoGoroutineLeaks` |
| A panic in a callback reaches Spin's caller and leaves the context closable | always on | `TestPanicInCallbackReachesSpinCaller` |
| Garbage-collector stress on finalizers (goal handles, `Args`, loggers) | `GOGC=1 go test ./pkg/...` | whole suite |
| cgo pointer-passing rules | `GOEXPERIMENT=cgocheck2 go test ./pkg/...` | whole suite |
| Fuzzing | `go test -run '^$' -fuzz <Target> <pkg>` | `FuzzLoadYAML`, `FuzzParseEnvVarValue` (`params/`), `FuzzExpandTopicName`, `FuzzDeserialize` |
| Benchmarks with allocation counts | `go test -run '^$' -bench . ./pkg/rclgo/` | `bench_test.go` |

`ros2.env` sets `GODEBUG=cgocheck=0`, which turns cgo's pointer checks off; do not use it for test runs.

`FuzzDeserialize` reaches the RMW's C deserializer. On Jazzy, Fast DDS crashes on truncated input (so the target
skips under it) and Cyclone DDS has driven a fuzz worker past 20 GB of memory. Fuzz it only under another RMW and a
memory cap, for example
`(ulimit -v 4000000; RMW_IMPLEMENTATION=rmw_cyclonedds_cpp go test -run '^$' -fuzz FuzzDeserialize -parallel 2 ./pkg/rclgo/)`.

## Coverage by area

| Area | rclgo tests (`pkg/rclgo/` unless noted) | rclcpp / rclpy counterpart |
|---|---|---|
| Node, namespace, name validation | `node_context_test.go`, `node_test.go` | `test_node`, `test_node.py` |
| Init/shutdown, contexts, domain ID | `node_context_test.go`, `context_test.go` | `test_init_shutdown`, `test_context`, rcl `test_domain_id` |
| Closing entities, close order, Close during Spin | `shutdown_test.go`, `close_order_internal_test.go`, `wait_internal_test.go` | rclpy `test_destruction`, `test_destruction_order`, `test_create_while_spinning` |
| Spin: entities created while spinning, poll-only subscriptions, one Spin per entity | `spin_test.go`, `spin_owner_test.go` | rclcpp `test_executors`, rclpy `test_create_while_spinning`, `test_executor` |
| Publishers, subscriptions, invalid names | `pubsub_test.go`, `messages_test.go` | `test_publisher`, `test_subscription`, `test_publisher.py`, `test_subscription.py` |
| Message fidelity: every `test_msgs` type, serialization and publish/subscribe | `messages_test.go` | `test_serialized_message`, `test_serialization.py`, system_tests `test_communication` |
| Services and clients: concurrent, deferred responses, cancellation, Close during a call | `services_test.go`, `service_test.go` | `test_service`, `test_client`, `test_service.py`, `test_client.py` |
| Calls racing Close (`TakeMessage`, `Publish`, responses) | `services_test.go` | rclpy handle tests (`test_destruction`) |
| Timers: period kept under load, accessors, concurrent timers on one clock | `timer_test.go`, `services_test.go`, `shutdown_test.go` | `test_timer`, `test_timer.py` |
| Graph: node names, per-node names and types, endpoint info, name expansion | `graph_test.go`, `node_test.go` | `test_node_graph`, `test_expand_topic_or_service_name` |
| QoS: profiles, validation, C round trip, durations | `qos/`, `qos_durations_test.go` | `test_qos`, `test_qos.py` |
| Action server: accept/reject, abort, cancel, expiry, Close, concurrency | `action_server_test.go`, `action_internal_test.go` | rclcpp_action `test_server`, `test_action_server.py` |
| Action client: feedback, status, results, cancellation | `action_client_test.go` | rclcpp_action `test_client`, `test_action_client.py` |
| Parameters: declare, overrides (YAML, CLI, environment), constraints, types, events, services | `params/` | `test_parameter`, `test_node_parameters`, `test_parameter_service`, rcl_yaml_param_parser tests |
| Logging, `/rosout` | `logging_test.go`, `node_test.go` | `test_logging`, `test_rosout_subscription`, `test_logging_rosout.py` |
| ROS time, clock, rate | `rostime/` | `test_time_source`, `test_rate`, `test_clock` |
| Error handling (rcl error state) | `error_internal_test.go`, `misc_test.go` | rcl `test_common` |
| Code generator | `gogen/` | rosidl generator tests |
| **Interop with rclcpp and rclpy** (other processes): every `test_msgs` type both ways with rclpy; topics, services and actions both ways with C++ and Python demo nodes; `ros2 service call`, `ros2 action send_goal`; transient-local and best-effort QoS | `interop_test.go`, `testdata/interop/*.py` | system_tests `test_communication`, `test_quality_of_service` |
| **Parameter services** through real services and the `ros2 param` tool; `/parameter_events` | `params/services_test.go` | `test_parameter_service`, `test_parameter_client`, ros2cli tests |
| **Simulated time**: `/clock`, `use_sim_time` at runtime, sleeps, `Rate`, time jumps, timers following `/clock` | `rostime/simtime_test.go` | `test_time_source`, `test_time_source.py` |

## Not covered yet

These areas have no rclgo implementation or no tests yet; they are planned next.

- **Lifecycle nodes, QoS event callbacks, executors and callback groups, `logger_level` parameter, waiting for a
  service**: not implemented in rclgo (see `ROADMAP.md`).
- **Failure injection**: rclcpp forces individual rcl calls to fail (mimick, `RCUTILS_FAULT_INJECTION`) to test error
  paths. rclgo has no equivalent, so paths such as a failing `rcl_action_process_cancel_request` are untested.
- **Signals**: SIGINT/SIGTERM handling of a running node (system_tests `test_sigint_handler`).
- **Benchmarks**: a first set exists (`bench_test.go`); none yet for Spin wake-up latency or executors.
