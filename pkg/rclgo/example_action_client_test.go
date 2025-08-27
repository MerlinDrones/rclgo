//nolint:revive
package rclgo_test

import (
	"context"
	"fmt"

	example_interfaces_action2 "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

func ExampleActionClient() {
	err := rclgo.Init(nil)
	if err != nil {
		// handle error
	}
	defer rclgo.Uninit()
	node, err := rclgo.NewNode("my_node", "my_namespace")
	if err != nil {
		// handle error
	}
	client, err := node.NewActionClient(
		"fibonacci",
		example_interfaces_action2.FibonacciTypeSupport,
		nil,
	)
	if err != nil {
		// handle error
	}
	ctx := context.Background()
	goal := example_interfaces_action2.NewFibonacci_Goal()
	goal.Order = 10
	result, _, err := client.WatchGoal(ctx, goal, func(ctx context.Context, feedback types.Message) {
		fmt.Println("Got feedback:", feedback)
	})
	if err != nil {
		// handle error
	}
	fmt.Println("Got result:", result)
}

func ExampleActionClient_type_safe_wrapper() {
	err := rclgo.Init(nil)
	if err != nil {
		// handle error
	}
	defer rclgo.Uninit()
	node, err := rclgo.NewNode("my_node", "my_namespace")
	if err != nil {
		// handle error
	}
	client, err := example_interfaces_action2.NewFibonacciClient(
		node,
		"fibonacci",
		nil,
	)
	if err != nil {
		// handle error
	}
	ctx := context.Background()
	goal := example_interfaces_action2.NewFibonacci_Goal()
	goal.Order = 10
	result, _, err := client.WatchGoal(ctx, goal, func(ctx context.Context, feedback *example_interfaces_action2.Fibonacci_FeedbackMessage) {
		fmt.Println("Got feedback:", feedback)
	})
	if err != nil {
		// handle error
	}
	fmt.Println("Got result:", result)
}
