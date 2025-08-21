# rclgo Overview

## What is rclgo?

`rclgo` is the Go client library for ROS 2. It binds to the core C library (`rcl`) and lets you write ROS 2 nodes directly in Go. The goals are:

- Provide the same primitives as `rclcpp` and `rclpy` (publishers, subscriptions, services, actions, parameters).
- Expose idiomatic Go APIs (contexts, channels, type‑safe messages).
- Interoperate with existing ROS 2 systems — you can run a Go node alongside C++ and Python nodes in the same graph.

Currently supported in Humble:
- Node creation/destruction
- Publishers and subscriptions for generated message types
- Services (servers and clients)
- Parameters (MVP implementation)
- Timers, executors, logging (basic)

See the [roadmap](./roadmap.md) for planned parity features (lifecycle nodes, actions, QoS events, etc.).

---

## Creating a basic node

### 1. Initialize ROS 2 context

```go
if err := rclgo.Init(nil); err != nil { panic(err) }
defer rclgo.Uninit()
````

### 2. Create a node

```go
n, err := rclgo.NewNode("talker", "")
if err != nil { panic(err) }
defer n.Close()
```

The first argument is the node name, the second is the namespace (empty string for default).

### 3. Create a publisher

```go
pub, _ := n.NewPublisher("/chatter", std_msgs_msg.StringTypeSupport, nil)
```

### 4. Publish messages in a loop

```go
msg := std_msgs_msg.NewString()
for i := 0; i < 10; i++ {
    msg.Data = fmt.Sprintf("Hello #%d", i)
    _ = pub.Publish(msg)
    time.Sleep(1 * time.Second)
}
```

### 5. Spin the node

```go
ctx, cancel := context.WithCancel(context.Background())
defer cancel()

if err := n.Spin(ctx); err != nil && ctx.Err() == nil {
    log.Printf("Spin error: %v", err)
}
```

---

## Next steps

* Try the `examples/param_demo` to see parameters in action.
* Look at `examples/subscriber_demo` to learn about subscriptions.
* Use `ros2 topic echo` and `ros2 param` CLI to verify interoperability.

## References

* ROS 2 design: [https://design.ros2.org/](https://design.ros2.org/)
* rcl (C API): [https://docs.ros2.org](https://docs.ros2.org)
* rclcpp: [https://docs.ros2.org/latest/api/rclcpp](https://docs.ros2.org/latest/api/rclcpp)
* rclpy: [https://docs.ros2.org/latest/api/rclpy](https://docs.ros2.org/latest/api/rclpy)


