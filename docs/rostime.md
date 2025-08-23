# ROS Time in rclgo

The `rostime` package provides ROS 2–compatible time primitives for Go:

* **`Clock`** — unified time source (wall clock or simulated `/clock`).
* **`Rate`** — loop rate helper that sleeps until the next iteration.
* Integration with **`/use_sim_time`** parameter.
* Correct QoS for `/clock` subscription (matches `rclcpp`/`rclpy`).

This mirrors the functionality of `rclcpp::Clock` and `rclcpp::Rate`.

## Why use ROS Time (sim time) vs. System Time?

ROS 2 distinguishes between **system wall time** and **ROS time** (which may be simulated).
Choosing the right one is important for reproducibility, testing, and real-time control.

### System Wall Time

* Comes directly from the OS clock (`time.Now()`).
* Always monotonic, follows real elapsed time.
* Use it when your node interacts with the outside world (e.g., wall-clock scheduling, logging, file timestamps).

### ROS Time (`/clock` + `use_sim_time`)

* Driven by messages on the `/clock` topic.
* Commonly published by simulators (Gazebo, Ignition, etc.) or log replay (`ros2 bag play`).
* Can pause, jump forward/backward, or run faster/slower than wall time.

This makes ROS Time critical for:

* **Simulation** — ensures your node stays synchronized with the simulated environment, even if the simulator runs slower or faster than real time.
* **Deterministic replay** — playing back a bag file re-creates the exact timing of events, letting your node behave as if it were live.
* **Testing & debugging** — pause and step through time, or rerun scenarios identically for regression testing.
* **Distributed coordination** — multiple nodes see the same consistent clock, even across machines.

### Rule of thumb

* **Use system wall time** for logs, performance measurement, and real-world scheduling.
* **Use ROS time** (`use_sim_time=true`) for any node logic that depends on timing in a simulation, bag replay, or time-controlled testing.

### Wall Time vs. ROS Time — quick comparison

| Capability / Behavior         | System Wall Time (`time.Now()`) | ROS Time (`/clock` + `use_sim_time`) |
|------------------------------:|:-------------------------------:|:------------------------------------:|
| Source                        | OS clock                        | `/clock` messages (simulator / bag)  |
| Monotonic (no backward jumps) | ✅ Usually\*                    | ❌ Can jump/pause/step                |
| Can pause/resume              | ❌                              | ✅                                    |
| Can run faster/slower         | ❌                              | ✅                                    |
| Deterministic replay          | ❌                              | ✅ (with bag/sim)                     |
| Global synchronization        | ❌ Local only                   | ✅ All nodes share the same clock     |
| Real‑world scheduling         | ✅                              | ⚠️ Only if `/clock` ≈ wall time       |
| Best for                      | Logging, file timestamps, ops   | Simulation, testing, bag replay      |

\* Wall time can still jump if the system time is adjusted (NTP, manual change). For precise elapsed timing on wall clock, prefer Go’s monotonic durations (e.g., `time.Since(start)`).


---
## Clock

`Clock` provides ROS 2 time, either:

* **Wall clock** (default).
* **Simulated time** when:

    * The node declares or sets the parameter `/use_sim_time := true`.
    * A `/clock` publisher is active in the system (e.g. Gazebo, Ignition).

### Construction

```go
import (
    "context"
    "github.com/merlindrones/rclgo/pkg/rclgo"
    "github.com/merlindrones/rclgo/pkg/rclgo/params"
    "github.com/merlindrones/rclgo/pkg/rclgo/rostime"
)

// Inside your node setup:
pm, _ := params.NewManager(node)
clock, _ := rostime.NewClock(context.Background(), node, pm)
```

### Getting the current time

```go
t := clock.Now()            // Go time.Time
msg := clock.NowMsg()       // builtin_interfaces/msg/Time
```

### Sleeping

```go
ctx := context.Background()
deadline := clock.Now().Add(500 * time.Millisecond)
if err := clock.SleepUntil(ctx, deadline); err != nil {
    // handle context cancellation or timeout
}
```

If sim time is enabled, `SleepUntil` blocks until `/clock` has advanced to the target time.

---

## Rate

`Rate` is a convenience wrapper for steady loop timing.

```go
r := rostime.NewRate(clock, 10) // 10 Hz loop

for {
    // Do work here

    if err := r.Sleep(context.Background()); err != nil {
        break // context canceled or timeout
    }
}
```

* Uses the provided `Clock` (so sim time or wall time automatically apply).
* Maintains the requested frequency regardless of execution time (like ROS 2).

---

## `/use_sim_time`

* Declared automatically by `params.Manager` (`false` by default).
* When set to `true`, `Clock` switches to `/clock` subscription.
* QoS is explicitly set to **KeepLast(1), BestEffort, Volatile** (ROS 2 standard).

To enable sim time:

```bash
ros2 param set /your_node use_sim_time true
```

---

## Examples

### Wall clock loop

```go
pm, _ := params.NewManager(node)
clock, _ := rostime.NewClock(ctx, node, pm)
r := rostime.NewRate(clock, 1) // 1 Hz

for {
    log.Infof("tick at %v", clock.Now())
    if err := r.Sleep(ctx); err != nil {
        break
    }
}
```

### Simulated time loop

Launch with:

```bash
ros2 run mynode --ros-args -p use_sim_time:=true
```

If a simulator publishes `/clock`, the same loop above will tick according to sim time.

## Example app

A small demo that prints the current time at 1 Hz and follows `/use_sim_time`.

**Path:** `examples/clock_demo/main.go`

### Run (wall clock)
go run ./examples/clock_demo

### Enable simulated time (requires a /clock publisher)
# in a separate terminal, run a simulator or a clock publisher that publishes /clock
ros2 param set /clock_demo use_sim_time true

# flip back to wall time
ros2 param set /clock_demo use_sim_time false
```

This example:

* initializes rclgo
* creates a node `clock_demo`
* wires `params.Manager` → `rostime.Clock` (so `/parameter_events` use ROS time when sim is on)
* prints `now` each tick + whether it’s wall or sim time
* sleeps with `Rate` that respects your active time source


---

## References

* [ROS 2 Design: Time](https://design.ros2.org/articles/clock_and_time.html)
* `rclcpp::Clock`, `rclcpp::Rate` (C++)
* `rclpy.clock.Clock`, `rclpy.timer.Rate` (Python)
