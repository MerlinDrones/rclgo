package main

import (
	"context"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/rostime"
)

func main() {
	// Ctrl‑C handling
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	// Parse ROS args and init rclgo (Init expects *rclgo.Args)
	args, _, err := rclgo.ParseArgs(os.Args[1:])
	if err != nil {
		panic(err)
	}
	if err := rclgo.Init(args); err != nil {
		panic(err)
	}
	// NOTE: no rclgo.Shutdown symbol in your API; node.Close() is sufficient below.

	// Create a node
	node, err := rclgo.NewNode("clock_demo", "")
	if err != nil {
		panic(err)
	}
	defer node.Close()

	// Parameters manager (declares use_sim_time and publishes /parameter_events)
	pm, err := params.NewManager(node)
	if err != nil {
		panic(err)
	}

	// ROS‑aware clock (hooks use_sim_time and /clock; stamps parameter events)
	clk, err := rostime.NewClock(ctx, node, pm)
	if err != nil {
		panic(err)
	}
	defer clk.Shutdown()

	// 1 Hz loop using the clock (works in wall or sim time)
	rate := clk.Rate(1)

	node.Logger().Info("clock_demo started; toggle 'use_sim_time' to switch sources.")
	node.Logger().Info("  ros2 param set /clock_demo use_sim_time true   # requires a /clock publisher")
	node.Logger().Info("  ros2 param set /clock_demo use_sim_time false  # back to wall time")

	for ctx.Err() == nil {
		now := clk.Now()
		src := "wall"
		if clk.IsSimTime() {
			src = "sim"
		}
		node.Logger().Infof("now=%s  src=%s", now.Format(time.RFC3339Nano), src)

		if err := rate.Sleep(ctx); err != nil {
			break
		}
	}

	node.Logger().Info("clock_demo stopping…")
}
