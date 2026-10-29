/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"context"
	"errors"
	"testing"
	"time"

	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// A generated (type-safe) action server whose ExecuteGoal returns a nil
// *X_Result, the idiomatic Go way to give up on a goal, aborts or cancels the
// goal and answers the client with an empty result. A nil *X_Result used to
// become a non-nil types.Message and crash the process when the result was
// sent.
func TestTypedActionNilResult(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("typed_nil_result", "/action_test")
	require.NoError(t, err)
	accepted := make(chan struct{}, 1)
	_, err = example_interfaces_action.NewFibonacciServer(node, "typed_nil", example_interfaces_action.NewFibonacciAction(
		func(ctx context.Context, g *example_interfaces_action.FibonacciGoalHandle) (*example_interfaces_action.Fibonacci_Result, error) {
			if _, err := g.Accept(); err != nil {
				return nil, err
			}
			switch g.Description.Order {
			case 1:
				return nil, errors.New("giving up")
			case 2:
				return nil, nil
			default: // wait to be canceled
				accepted <- struct{}{}
				<-ctx.Done()
				return nil, ctx.Err()
			}
		}), nil)
	require.NoError(t, err)
	client, err := example_interfaces_action.NewFibonacciClient(node, "typed_nil", nil)
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)

	send := func(order int32) *types.GoalID {
		goal := example_interfaces_action.NewFibonacci_Goal()
		goal.Order = order
		var id *types.GoalID
		requireEventually(t, "goal accepted", func() error {
			ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
			defer cancel()
			resp, goalID, err := client.SendGoal(ctx, goal)
			if err == nil && !resp.Accepted {
				err = errors.New("goal rejected")
			}
			id = goalID
			return err
		})
		return id
	}
	result := func(id *types.GoalID) *example_interfaces_action.Fibonacci_GetResult_Response {
		ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
		defer cancel()
		resp, err := client.GetResult(ctx, id)
		require.NoError(t, err)
		return resp
	}

	t.Run("error", func(t *testing.T) {
		require.EqualValues(t, rclgo.GoalAborted, result(send(1)).Status)
	})
	t.Run("nil error", func(t *testing.T) {
		require.EqualValues(t, rclgo.GoalAborted, result(send(2)).Status)
	})
	t.Run("canceled", func(t *testing.T) {
		id := send(3)
		<-accepted
		ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
		defer cancel()
		req := action_msgs_srv.NewCancelGoal_Request()
		req.GoalInfo.GoalId.Uuid = *id
		_, err := client.CancelGoal(ctx, req)
		require.NoError(t, err)
		require.EqualValues(t, rclgo.GoalCanceled, result(id).Status)
	})
}
