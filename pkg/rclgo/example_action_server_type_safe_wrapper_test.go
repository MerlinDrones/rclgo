//nolint:revive
package rclgo_test

import (
	"context"
	"errors"

	example_interfaces_action2 "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
)

var typeSafeFibonacci = example_interfaces_action2.NewFibonacciAction(
	func(
		ctx context.Context, goal *example_interfaces_action2.FibonacciGoalHandle,
	) (*example_interfaces_action2.Fibonacci_Result, error) {
		if goal.Description.Order < 0 {
			return nil, errors.New("order must be non-negative")
		}
		sender, err := goal.Accept()
		if err != nil {
			return nil, err
		}
		result := example_interfaces_action2.NewFibonacci_Result()
		fb := example_interfaces_action2.NewFibonacci_Feedback()
		var x, y, i int32
		for y = 1; i < goal.Description.Order; x, y, i = y, x+y, i+1 {
			result.Sequence = append(result.Sequence, x)
			fb.Sequence = result.Sequence
			if err = sender.Send(fb); err != nil {
				goal.Logger().Error("failed to send feedback: ", err)
			}
		}
		return result, nil
	},
)

func ExampleActionServer_type_safe_wrapper() {
	err := rclgo.Init(nil)
	if err != nil {
		// handle error
	}
	defer rclgo.Uninit()
	node, err := rclgo.NewNode("my_node", "my_namespace")
	if err != nil {
		// handle error
	}
	_, err = example_interfaces_action2.NewFibonacciServer(node, "fibonacci", typeSafeFibonacci, nil)
	if err != nil {
		// handle error
	}
	ctx := context.Background()
	if err = rclgo.Spin(ctx); err != nil {
		// handle error
	}
}
