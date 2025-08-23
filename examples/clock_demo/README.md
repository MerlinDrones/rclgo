
### `examples/clock_demo/README.md`

````markdown
# To Use

This example can be placed in a ROS 2 workspace and launched with `ros2 run`.

> Package name below assumes you wrap the example as `rclgo_clock_demo_pkg`
> (mirroring how you structured `rclgo_param_demo_pkg`). Adjust if you use a different name.

```bash
# Build the package
colcon build --packages-select rclgo_clock_demo_pkg
source install/setup.bash

# Run the node without a YAML file (defaults to wall time)
ros2 run rclgo_clock_demo_pkg clock_demo

# Or run with a params file that enables simulated time
ros2 run rclgo_clock_demo_pkg clock_demo --params-file /absolute/path/to/params.yaml
````

While it runs, you can inspect or toggle the parameter:

```bash
ros2 param list
ros2 param get /clock_demo use_sim_time
ros2 param set /clock_demo use_sim_time true   # switch to sim time (requires a /clock publisher)
ros2 param set /clock_demo use_sim_time false  # back to wall time
```

### Providing `/clock`

To see simulated time in action, run a simulator (Gazebo/Ignition) or any node that publishes `/clock`.
Once `/clock` is live and `use_sim_time` is `true`, the example will report `src=sim` and follow the `/clock` timestamps.

---

## Alternatively: build and run directly

You can run the Go binary without wrapping it as a ROS 2 package.

```bash
go run ./examples/clock_demo
# or
go build -o clock_demo ./examples/clock_demo
./clock_demo &
```

Interact with parameters as usual:

```bash
ros2 param list
ros2 param get /clock_demo use_sim_time
ros2 param set /clock_demo use_sim_time true
ros2 param set /clock_demo use_sim_time false
```

### What you should see

* With wall time (default): the node logs the current system time once per second (`src=wall`).
* With sim time enabled and a `/clock` publisher running: the node logs time according to `/clock` (`src=sim`), even if wall time is paused or moving at a different rate.

