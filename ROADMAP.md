# rclgo ↔ rclcpp/rclpy Parity Roadmap (ROS 2 Humble)

> Working dir: `/home/dave/Git/merlin/rclgo`
> Target: ROS 2 **Humble** APIs; production-grade gaps first, then nice-to-haves.

---

## Legend

* **P0**: Blocks production parity or many downstream nodes
* **P1**: Commonly used; improves ergonomics/perf
* **P2**: Nice-to-have / advanced

Checkbox key: ☐ not started · ◐ in progress · ☐⚙ needs design · ☑ done

---

## Top Priorities (P0)

### \[P0] Parameters API (node-local + remote)

* ☐⚙ Design: `node.Parameters` surface similar to rclcpp (`declare`, `get`, `set`, `onSetCallback`)
* ☐ Bindings: rcl `rcl_parameters_*` and parameter events publisher/subscriber
* ☐ `/use_sim_time` integration hook
* ☐ CLI compatibility: `ros2 param get/set/list` works against rclgo nodes
* **Definition of Done**: Parameter events topic emits; declare/get/set work; callbacks and rejection path covered; e2e test with `ros2 param`.

### \[P0] Actions (client & server)

* ☐⚙ Design: idiomatic Go API (`ActionServer[TGoal,TResult,TFeedback]`, ctx-based cancellation)
* ☐ Bindings to `rcl_action`
* ☐ Goal handles, feedback streaming, preempt, cancel
* ☐ Simple and composed examples (`Fibonacci`, `FollowPath`)
* **DoD**: Interop with rclcpp/rclpy examples; cancel/preempt tested; goal state transitions logged.

### \[P0] Lifecycle Nodes

* ☐⚙ Design: `LifecycleNode` with states (unconfigured→inactive→active→finalized) and transition callbacks
* ☐ Wire to `rcl_lifecycle` (or implement thin state machine + `managed nodes` pattern if `rcl_lifecycle` not exposed)
* ☐ Parameter-change validation during `configure`
* **DoD**: Transitions callable via `ros2 lifecycle`; node publishes lifecycle state; example package passes.

### \[P0] ROS Time / Clock

* ☐ Clock types: ROS time vs system
* ☐ Time source: subscribe to `/clock`; parameter `/use_sim_time`
* ☐ Time jump callbacks (basic)
* **DoD**: Timers run correctly under simulated time; clock switch unit-tested.

### \[P0] Executors & Callback Groups

* ☐ Single-threaded executor solidified (wakeups, waitset mgmt)
* ☐ Multi-threaded executor (N workers), cooperative shutdown
* ☐ Callback groups (MutuallyExclusive, Reentrant); assign to sub/pub/svc/timer
* **DoD**: Concurrency tests show no races; perf parity on pub/sub stress; examples demonstrate groups.

---

## Quality of Service (QoS) (P0→P1)

* ◐ Policy surface parity (`reliability`, `durability`, `history`, `depth`, `liveliness`, `deadline`, `lifespan`)
* ☐ Constructor validation (reject invalid combos)
* ☐ QoS event callbacks: deadline missed, liveliness lost/changed, incompatible QoS
* ☐ Default profile resolution (system defaults, rmw defaults)
* **DoD**: Events observable; invalid profiles error; interoperability tests across rmw.

---

## Services (P1)

* ☐ Async client pattern with `context.Context` + futures/promise
* ☐ Deadline/cancel support
* ☐ Callback groups integration
* **DoD**: rclcpp service examples pass; concurrent calls safe.

---

## Discovery & Graph (P1)

* ☐ `GetTopicNamesAndTypes`, `GetServiceNamesAndTypes`
* ☐ Graph events (on pub/sub added/removed)
* ☐ Waitables abstraction (custom wait handles)
* **DoD**: Topic/service listings match `ros2 topic|service list`; graph event test.

---

## Composition & Intra-process (P1)

* ☐ Node options: intra-process enable/disable
* ☐ Zero-copy/intra-process shortcuts when enabled
* ☐ (If feasible) Composable nodes API (static linking pattern in Go)
* **DoD**: Intra-process yields measurable lower copy counts; sample app.

---

## Logging (P1)

* ☐ Named loggers per node with severity control
* ☐ Bridge to rcl logging; parameter-driven level change
* **DoD**: `ros2 run` shows ROS log formatting; `ros2 param set /node logger_level` affects output.

---

## Introspection & Dynamic Types (P2)

* ☐ Serialization helpers (CDR) for `[]byte`↔ROS2
* ☐ `AnyMsg`-like support (opaque passthrough)
* ☐ Type introspection hooks
* **DoD**: Can record/play raw messages; basic reflect tests.

---

## Tools & Ergonomics (P2)

* ☐ `rclgo` launch-lite (YAML or Go DSL mini-launch)
* ☐ Node scaffolding tool (template generator)
* ☐ Examples: minimal pub/sub, svc, action, lifecycle, parameters
* **DoD**: New users can build/run examples with `colcon` or `go build`.

---

# Implementation Plan by Area

## 1) Parameters

* **Packages**: `pkg/rclgo/params`, integrate in `Node`
* **API Sketch**:

    * `Declare[T any](name string, default T, opts ...ParamOption) (T, error)`
    * `Get[T any](name string) (T, error)`
    * `OnSet(func([]Parameter) SetResult)`
* **Tasks**:

    * Bind `rcl_node_params` / events
    * Internal store w/ type safety & validation
    * `/use_sim_time` hook to Clock
    * CLI e2e
* **Tests**: unit + e2e with `ros2 param` (get/set/list), rejection paths

## 2) Actions

* **Packages**: `pkg/rclgo/action`
* **API Sketch**:

    * `NewActionServer[TG,TR,TF]` with `Execute(ctx, goal)` callback
    * `ActionClient` with `SendGoal(ctx, goal)` returns `{GoalHandle, Feedback <-chan TF, Result <-chan TR}`
* **Tasks**:

    * Bindings for `rcl_action` structs
    * Goal state machine
    * Feedback publisher cadence
    * Cancel & preempt
* **Tests**: Interop with rclcpp Fibonacci; cancel/preempt; stress goals

## 3) Lifecycle

* **Packages**: `pkg/rclgo/lifecycle`
* **API Sketch**:

    * `LifecycleNode` embeds `Node`, exposes `OnConfigure/OnActivate/OnDeactivate/OnCleanup/OnShutdown`
* **Tasks**:

    * State machine + transition services/topics
    * Guard transitions; parameter validation on configure
* **Tests**: `ros2 lifecycle` CLI; bad transitions rejected

## 4) Executors & Callback Groups

* **Packages**: `pkg/rclgo/executor`, `pkg/rclgo/cbgroups`
* **API Sketch**:

    * `ExecutorOptions{NumWorkers int}`
    * `CallbackGroup{MutuallyExclusive|Reentrant}`
* **Tasks**:

    * Waitset orchestration
    * Work-stealing worker pool
    * Assign entities to groups
* **Tests**: Race detector, heavy pub/sub/service load, orderly shutdown

## 5) ROS Time / Clock

* **Packages**: `pkg/rclgo/time`
* **API**: `Clock`, `Duration`, `Rate`, `Now()`, `SleepUntil()`, time jump callbacks
* **Tasks**:

    * `/clock` subscription
    * `/use_sim_time` parameter
    * Timer integration uses ROS time
* **Tests**: Sim time flips mid-run; timer accuracy under clock pause

## 6) QoS Events & Validation

* **Packages**: extend `pkg/rclgo/qos`
* **Tasks**:

    * Validate QoS profiles at creation
    * Surface event callbacks
* **Tests**: Deadline missed, liveliness lost tests using short deadlines

## 7) Discovery & Graph

* **Packages**: `pkg/rclgo/graph`
* **Tasks**:

    * Names & types queries
    * Graph event subscriptions
* **Tests**: Compare against `ros2 topic|service list` outputs

## 8) Services Ergonomics

* **Tasks**:

    * Context-aware calls; client-side timeout/cancel
    * Futures/Promise helper
* **Tests**: Concurrency + cancellation

## 9) Composition/Intra-process

* **Tasks**:

    * Intra-process shared message path
    * Options on node creation
* **Tests**: Copy counters; throughput benchmarks

---

# Ready-to-File Issue Templates

### Issue: Implement Parameters API (Humble parity)

**Priority**: P0
**Scope**: declare/get/set, parameter events, callbacks, `/use_sim_time`
**Acceptance**: `ros2 param` works against Go node; unit/e2e tests green.

### Issue: Add Action Server/Client bindings

**Priority**: P0
**Scope**: goal mgmt, feedback, result, cancel/preempt
**Acceptance**: Interop with rclcpp Fibonacci; cancel works.

### Issue: Lifecycle Node support

**Priority**: P0
**Scope**: transitions, services, events
**Acceptance**: `ros2 lifecycle` manages node; example included.

### Issue: Multi-threaded executor & callback groups

**Priority**: P0
**Scope**: N-worker executor, groups, shutdown
**Acceptance**: Concurrency tests; no data races with `-race`.

### Issue: ROS Time & `/clock`

**Priority**: P0
**Scope**: ROS time source, timers, time jumps
**Acceptance**: Sim time tests pass; timers respect ROS time.

### Issue: QoS validation + event callbacks

**Priority**: P1
**Scope**: validation; deadline/liveliness/incompatible events
**Acceptance**: Event tests trigger as expected.

### Issue: Graph & discovery utilities

**Priority**: P1
**Scope**: names/types, graph events
**Acceptance**: Matches CLI listings; event callbacks fire.

### Issue: Services ergonomics (async/cancel)

**Priority**: P1
**Scope**: ctx-based async client; timeouts; cancel
**Acceptance**: Client cancel path test; interop sample.

### Issue: Intra-process communication

**Priority**: P2
**Scope**: zero-copy path; node option
**Acceptance**: Perf delta vs inter-process demonstrated.

### Issue: Logging parity

**Priority**: P2
**Scope**: named loggers; param-driven level
**Acceptance**: Level toggled via param; matches ROS format.

---

# Milestones & Sequencing

1. **M1 (Foundations)**: Parameters, ROS Time, QoS validation
2. **M2 (Core interop)**: Actions, Lifecycle, Services async/cancel
3. **M3 (Concurrency)**: Executors + callback groups
4. **M4 (Ecosystem)**: Graph/discovery, logging parity
5. **M5 (Perf & UX)**: Intra-process, tooling, examples

Each milestone includes: examples, docs, and e2e interop tests.

---

# Testing Strategy

* **Unit**: wrappers, conversions, error paths
* **Interop**: run rclcpp/rclpy sample nodes against rclgo nodes via `colcon test`
* **CLI**: `ros2 param|topic|service|action|lifecycle` against rclgo demos
* **Sim time**: Gazebo or `/clock` publisher harness
* **Stress**: pub/sub rate & executor contention; `-race`

---

# Example Repos/Packages to Mirror (for tests/examples)

* `demo_nodes_cpp` / `demo_nodes_py` (talker/listener, parameters, services)
* `rclcpp_action` Fibonacci example
* `lifecycle` demo nodes

---

# Risks & Mitigations

* **rcl bindings drift** → pin to Humble headers; generate with `cgo` carefully; CI builds on rmw implementations
* **Executor correctness** → start simple; add MT after robust waitset tests
* **API stability** → `internal` for experimental packages; semver pre-1.0

---

# Tracking Matrix (snapshot)

| Feature               | rclcpp | rclpy | rclgo |
| --------------------- | -----: | ----: | ----: |
| Parameters            |      ✅ |     ✅ |     ☐ |
| Actions               |      ✅ |     ✅ |     ☐ |
| Lifecycle             |      ✅ |    ⚠️ |     ☐ |
| ROS Time              |      ✅ |     ✅ |     ☐ |
| Executors (MT)        |      ✅ |    ⚠️ |     ☐ |
| Callback Groups       |      ✅ |    ⚠️ |     ☐ |
| QoS Policies          |      ✅ |     ✅ |     ◐ |
| QoS Events            |      ✅ |     ✅ |     ☐ |
| Services Async/Cancel |      ✅ |     ✅ |     ◐ |
| Discovery/Graph       |      ✅ |     ✅ |     ☐ |
| Intra-process         |      ✅ |     — |     ☐ |
| Logging Parity        |      ✅ |     ✅ |     ◐ |

---

## Next Steps (suggested)

1. Spin up `pkg/rclgo/params` and wire `/use_sim_time` (unblocks Clock & Timers)
2. Start Action bindings with Fibonacci interop demo
3. Introduce Multi-threaded Executor and Callback Groups (design doc first)
4. Add QoS event callbacks and validation in pub/sub constructors
5. Build sample nodes and CI jobs for interop & CLI tests
