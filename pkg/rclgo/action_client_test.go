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
	"sync"
	"testing"
	"time"

	action_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/msg"
	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Action client, after rclcpp_action's test_client.

// WatchGoal delivers feedback while the goal runs and returns its result.
func TestActionClientWatchGoalFeedbackAndResult(t *testing.T) {
	f := newActionFixture(t, "watch_goal", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		sender, err := goal.Accept()
		if err != nil {
			return nil, err
		}
		// Give the client's feedback subscription time to match.
		time.Sleep(300 * time.Millisecond)
		fb := example_interfaces_action.NewFibonacci_Feedback()
		for i := int32(0); i < 5; i++ {
			fb.Sequence = append(fb.Sequence, i)
			if err := sender.Send(fb); err != nil {
				return nil, err
			}
			time.Sleep(30 * time.Millisecond)
		}
		return fibResult(fb.Sequence...), nil
	})
	var (
		mu       sync.Mutex
		feedback [][]int32
	)
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	result, id, err := f.client.WatchGoal(ctx, fibGoal(5), func(_ context.Context, m types.Message) {
		mu.Lock()
		defer mu.Unlock()
		fb := m.(*example_interfaces_action.Fibonacci_FeedbackMessage)
		feedback = append(feedback, fb.Feedback.Sequence)
	})
	require.NoError(t, err)
	require.NotNil(t, id)
	res := result.(*example_interfaces_action.Fibonacci_GetResult_Response)
	require.EqualValues(t, rclgo.GoalSucceeded, res.Status)
	require.Equal(t, []int32{0, 1, 2, 3, 4}, res.Result.Sequence)

	mu.Lock()
	defer mu.Unlock()
	require.NotEmpty(t, feedback, "no feedback received")
	for i := 1; i < len(feedback); i++ {
		require.Greater(t, len(feedback[i]), len(feedback[i-1]), "feedback out of order: %v", feedback)
	}
}

// WatchStatus reports the goal's state changes.
func TestActionClientWatchStatus(t *testing.T) {
	release := make(chan struct{})
	f := newActionFixture(t, "watch_status", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		<-release
		return fibResult(1), nil
	})
	resp, id := f.sendGoal(1)
	require.True(t, resp.Accepted)

	statuses := make(chan int8, 16)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	f.client.WatchStatus(ctx, id, func(_ context.Context, m types.Message) {
		statuses <- m.(*action_msgs_msg.GoalStatus).Status
	})
	waitFor := func(want rclgo.GoalStatus) {
		t.Helper()
		deadline := time.After(actionTimeout)
		for {
			select {
			case s := <-statuses:
				if s == int8(want) {
					return
				}
			case <-deadline:
				t.Fatalf("status %v never reported", want)
			}
		}
	}
	waitFor(rclgo.GoalExecuting) // the status topic is transient-local
	close(release)
	waitFor(rclgo.GoalSucceeded)
}

func TestActionClientGetResultOfUnknownGoal(t *testing.T) {
	f := newActionFixture(t, "unknown_result", nil, func(context.Context, *rclgo.GoalHandle) (types.Message, error) {
		return nil, nil
	})
	var id types.GoalID
	id[0] = 7
	require.EqualValues(t, rclgo.GoalUnknown, f.getResult(&id).Status)
}

// Canceling WatchGoal's context cancels the goal on the server.
func TestActionClientWatchGoalCancelsOnContext(t *testing.T) {
	executing := make(chan struct{})
	canceled := make(chan struct{})
	f := newActionFixture(t, "watch_cancel", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		close(executing)
		<-ctx.Done()
		close(canceled)
		return nil, ctx.Err()
	})
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() {
		_, _, err := f.client.WatchGoal(ctx, fibGoal(3), nil)
		done <- err
	}()
	requireClosed(t, executing, "goal never started")
	cancel()
	select {
	case err := <-done:
		require.ErrorIs(t, err, context.Canceled)
	case <-time.After(actionTimeout):
		t.Fatal("WatchGoal did not return after its context was canceled")
	}
	requireClosed(t, canceled, "the server goal was not canceled")
}

// Requests on a closed action client fail with ErrClosed instead of panicking.
// Nothing spins the context, so Close finalizes the client at once.
func TestActionClientRequestsAfterClose(t *testing.T) {
	ctx := newSpinTestContext(t)
	node, err := ctx.NewNode("after_close_node", "/action_test")
	require.NoError(t, err)
	client, err := node.NewActionClient("after_close", example_interfaces_action.FibonacciTypeSupport, nil)
	require.NoError(t, err)
	require.NoError(t, client.Close())

	cctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	var id types.GoalID
	id[0] = 1
	_, _, err = client.SendGoal(cctx, fibGoal(1))
	require.ErrorIs(t, err, rclgo.ErrClosed)
	_, _, err = client.WatchGoal(cctx, fibGoal(1), nil)
	require.ErrorIs(t, err, rclgo.ErrClosed)
	_, err = client.GetResult(cctx, &id)
	require.ErrorIs(t, err, rclgo.ErrClosed)
	_, err = client.CancelGoal(cctx, action_msgs_srv.NewCancelGoal_Request())
	require.ErrorIs(t, err, rclgo.ErrClosed)
}

// WatchGoal reports a goal that did not succeed with a *GoalFailedError and
// still returns its result.
func TestActionClientWatchGoalReportsAbortedGoal(t *testing.T) {
	f := newActionFixture(t, "watch_aborted", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return nil, errors.New("failed while executing")
	})
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	result, _, err := f.client.WatchGoal(ctx, fibGoal(3), nil)
	var failed *rclgo.GoalFailedError
	require.ErrorAs(t, err, &failed)
	require.Equal(t, rclgo.GoalAborted, failed.Status)
	require.NotNil(t, result)
	require.EqualValues(t, rclgo.GoalAborted, result.(*example_interfaces_action.Fibonacci_GetResult_Response).Status)
}

// Feedback reaches a handler in the order it was sent, even in a burst, and a
// panicking handler doesn't end the process.
func TestActionClientFeedbackInOrderAndPanicsRecovered(t *testing.T) {
	const n = 40
	f := newActionFixture(t, "feedback_order", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		sender, err := goal.Accept()
		if err != nil {
			return nil, err
		}
		// Give the client's feedback subscription time to match.
		time.Sleep(300 * time.Millisecond)
		for i := int32(1); i <= n; i++ {
			fb := example_interfaces_action.NewFibonacci_Feedback()
			fb.Sequence = []int32{i}
			if err := sender.Send(fb); err != nil {
				return nil, err
			}
		}
		time.Sleep(200 * time.Millisecond)
		return fibResult(1), nil
	})
	var (
		mu  sync.Mutex
		got []int32
	)
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	_, _, err := f.client.WatchGoal(ctx, fibGoal(3), func(_ context.Context, m types.Message) {
		v := m.(*example_interfaces_action.Fibonacci_FeedbackMessage).Feedback.Sequence[0]
		mu.Lock()
		got = append(got, v)
		mu.Unlock()
		if v == 1 {
			panic("handler panic must be recovered")
		}
		time.Sleep(time.Millisecond) // a slow handler must not reorder later messages
	})
	require.NoError(t, err)
	mu.Lock()
	defer mu.Unlock()
	require.NotEmpty(t, got, "no feedback received")
	for i := 1; i < len(got); i++ {
		require.Greater(t, got[i], got[i-1], "feedback out of order: %v", got)
	}
}
