/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

//nolint:revive
package rclgo_test

import (
	"context"
	"fmt"
	"time"

	example_interfaces_action2 "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// An action server and client in one process: the client sends a goal and
// prints the result. (The ActionServer and ActionClient examples show each
// side on its own; they need a peer, so they are only compiled.)
func Example_actionRoundTrip() {
	rclctx, err := rclgo.NewContext(rclgo.ClockTypeROSTime, nil)
	if err != nil {
		panic(err)
	}
	defer rclctx.Close()
	node, err := rclctx.NewNode("fibonacci_example", "/examples")
	if err != nil {
		panic(err)
	}
	if _, err = node.NewActionServer("fibonacci", fibonacci, nil); err != nil {
		panic(err)
	}
	client, err := example_interfaces_action2.NewFibonacciClient(node, "fibonacci", nil)
	if err != nil {
		panic(err)
	}
	spinCtx, stop := context.WithCancel(context.Background())
	defer stop()
	go rclctx.Spin(spinCtx) //nolint:errcheck

	goal := example_interfaces_action2.NewFibonacci_Goal()
	goal.Order = 8
	// A goal sent before the client has discovered the server is lost, so
	// retry with a short timeout until the server answers.
	for attempt := 0; attempt < 50; attempt++ {
		ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
		result, _, err := client.WatchGoal(ctx, goal, nil)
		cancel()
		if err == nil {
			fmt.Println("result:", result.Result.Sequence)
			return
		}
	}
	fmt.Println("the server did not answer")
	// Output: result: [0 1 1 2 3 5 8 13]
}
