# rclgo ↔ rclcpp/rclpy Parity Roadmap (ROS 2 Humble)

> Target: ROS 2 **Humble** APIs; production-grade gaps first, then nice-to-haves.

---

## Legend

* **P0**: Blocks production parity or many downstream nodes
* **P1**: Commonly used; improves ergonomics/perf
* **P2**: Nice-to-have / advanced

Checkbox key: ☐ not started · ◐ in progress · ⚙ needs design · ☑ done

---

## Top Priorities (P0)

### [P0] Parameters API (node-local + remote)

* ☑ Design: `pkg/rclgo/params` surface (`Declare`, `Get`, `Set`, `OnSet`, `Undeclare`)
* ☑ Bindings: rcl_interfaces services + `/parameter_events` publisher
* ☑ `/use_sim_time` integration hook
* ☑ CLI compatibility: `ros2 param get/set/list/describe/undeclare` works against rclgo nodes
* ☑ YAML preload loader (`params.LoadYAML`)
* **Definition of Done**:
  Parameters can be declared/undeclared, described, loaded from YAML.
  CLI interop passes (`ros2 param`).
  Event topic emits on declare/set/undeclare.
  Constraints and OnSet callbacks enforced.

### [P0] Actions (client & server)

* ⚙ Design: idiomatic Go API (`ActionServer[TGoal,TResult,TFeedback]`, ctx-based cancellation)
* ☐ Bindings to `rcl_action`
* ☐ Goal handles, feedback streaming, preempt, cancel
* ☐ Simple and composed examples (`Fibonacci`, `FollowPath`)
* **DoD**: Interop with rclcpp/rclpy examples; cancel/preempt tested; goal state transitions logged.

### [P0] Lifecycle Nodes

* ⚙ Design: `LifecycleNode` with states (unconfigured→inactive→active→finalized) and transition callbacks
* ☐ Wire to `rcl_lifecycle`
* ☐ Parameter-change validation during `configure`
* **DoD**: Transitions callable via `ros2 lifecycle`; node publishes lifecycle state; example package passes.

### [P0] ROS Time / Clock

* ☑ Clock types: ROS time vs system
* ☑ Time source: subscribe to `/clock`; parameter `/use_sim_time`
* ☑ Time jump callbacks (basic)
* **DoD**: Timers run correctly under simulated time; clock switch unit-tested.

### [P0] Executors & Callback Groups

* ☐ Single-threaded executor solidified (wakeups, waitset mgmt)
* ☐ Multi-threaded executor (N workers), cooperative shutdown
* ☐ Callback groups (MutuallyExclusive, Reentrant)
* **DoD**: Concurrency tests show no races; perf parity on pub/sub stress.

---

## Quality of Service (QoS) (P0→P1)

* ☑ Policy surface parity (`reliability`, `durability`, `history`, `depth`, `liveliness`, `deadline`, `lifespan`)
* ☐ Constructor validation (reject invalid combos)
* ☐ QoS event callbacks: deadline missed, liveliness lost/changed, incompatible QoS
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
* ☐ Waitables abstraction
* **DoD**: Topic/service listings match `ros2 topic|service list`; graph event test.

---

## Composition & Intra-process (P1)

* ☐ Node options: intra-process enable/disable
* ☐ Zero-copy/intra-process shortcuts
* **DoD**: Intra-process yields measurable lower copy counts.

---

## Logging (P1)

* ☐ Named loggers per node with severity control
* ☐ Bridge to rcl logging; parameter-driven level change
* **DoD**: `ros2 run` shows ROS log formatting; `ros2 param set /node logger_level` affects output.

---

## Introspection & Dynamic Types (P2)

* ☐ Serialization helpers (CDR)
* ☐ `AnyMsg`-like support
* ☐ Type introspection hooks
* **DoD**: Can record/play raw messages; basic reflect tests.

---

## Tools & Ergonomics (P2)

* ☐ `rclgo` launch-lite (YAML or Go DSL mini-launch)
* ☐ Node scaffolding tool
* ☐ Examples: minimal pub/sub, svc, action, lifecycle, parameters
* **DoD**: New users can build/run examples with `colcon` or `go build`.

---

# Milestones & Sequencing

1. **M1 (Foundations)**: ✅ Parameters, ◐ QoS validation, ✅ ROS Time
2. **M2 (Core interop)**: ☐ Actions, ☐ Lifecycle, ☐ Services async/cancel
3. **M3 (Concurrency)**: ☐ Executors + callback groups
4. **M4 (Ecosystem)**: ☐ Graph/discovery, ☐ logging parity
5. **M5 (Perf & UX)**: ☐ Intra-process, ☐ tooling, examples

---

# Tracking Matrix (snapshot)

| Feature               | rclcpp | rclpy | rclgo |
| --------------------- | -----: | ----: |------:|
| Parameters            |      ✅ |     ✅ |     ✅ |
| Actions               |      ✅ |     ✅ |     ☐ |
| Lifecycle             |      ✅ |    ⚠️ |     ☐ |
| ROS Time              |      ✅ |     ✅ |     ✅ |
| Executors (MT)        |      ✅ |    ⚠️ |     ☐ |
| Callback Groups       |      ✅ |    ⚠️ |     ☐ |
| QoS Policies          |      ✅ |     ✅ |     ✅ |
| QoS Events            |      ✅ |     ✅ |     ☐ |
| Services Async/Cancel |      ✅ |     ✅ |     ☐ |
| Discovery/Graph       |      ✅ |     ✅ |     ☐ |
| Intra-process         |      ✅ |     — |     ☐ |
| Logging Parity        |      ✅ |     ✅ |     ☐ |


---
