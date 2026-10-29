/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"context"
	"sync"
	"testing"
	"time"

	action_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/msg"
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
