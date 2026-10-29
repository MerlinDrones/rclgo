/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"context"
	"os"
	"strconv"
	"sync/atomic"
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Public API that no other test exercises.

// InitWithOpts initializes the default context with options; a second call
// is a no-op.
func TestInitWithOpts(t *testing.T) {
	domain, err := strconv.Atoi(os.Getenv("ROS_DOMAIN_ID"))
	require.NoError(t, err)
	opts := rclgo.NewDefaultContextOptions()
	opts.DomainID = uint(domain)
	require.NoError(t, rclgo.InitWithOpts(nil, opts))
	first := rclgo.DefaultContext()
	require.NotNil(t, first)
	require.NoError(t, rclgo.InitWithOpts(nil, nil), "a second InitWithOpts is a no-op")
	require.Same(t, first, rclgo.DefaultContext())
	require.NoError(t, rclgo.Uninit())
	require.Nil(t, rclgo.DefaultContext())
}

// WatchFeedback delivers the feedback of one goal until its context ends; the
// accessors of goals, servers and clients return their owners.
func TestActionClientWatchFeedback(t *testing.T) {
	release := make(chan struct{})
	var f *actionFixture
	var accessorsOK atomic.Bool
	f = newActionFixture(t, "watch_feedback", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		accessorsOK.Store(goal.Server() == f.server && goal.Logger() != nil)
		sender, err := goal.Accept()
		if err != nil {
			return nil, err
		}
		fb := example_interfaces_action.NewFibonacci_Feedback()
		for {
			fb.Sequence = append(fb.Sequence, int32(len(fb.Sequence)))
			if err := sender.Send(fb); err != nil {
				return nil, err
			}
			select {
			case <-release:
				return fibResult(fb.Sequence...), nil
			case <-ctx.Done():
				return nil, ctx.Err()
			case <-time.After(20 * time.Millisecond):
			}
		}
	})
	defer close(release)
	require.Same(t, f.node, f.server.Node())
	require.Same(t, f.node, f.client.Node())

	_, id := f.sendGoal(3)
	got := make(chan struct{}, 100)
	otherGot := make(chan struct{}, 100)
	watchCtx, stop := context.WithCancel(context.Background())
	errc := f.client.WatchFeedback(watchCtx, id, func(_ context.Context, m types.Message) {
		if fb := m.(*example_interfaces_action.Fibonacci_FeedbackMessage); types.GoalID(fb.GoalID.Uuid) != *id {
			t.Errorf("feedback of goal %v, watching %v", fb.GoalID.Uuid, *id)
		}
		got <- struct{}{}
	})
	other := types.GoalID{1, 2, 3}
	otherErrc := f.client.WatchFeedback(watchCtx, &other, func(context.Context, types.Message) { otherGot <- struct{}{} })
	for i := 0; i < 3; i++ {
		select {
		case <-got:
		case <-time.After(actionTimeout):
			t.Fatal("no feedback")
		}
	}
	stop()
	require.ErrorIs(t, <-errc, context.Canceled)
	require.ErrorIs(t, <-otherErrc, context.Canceled)
	require.Empty(t, otherGot, "feedback of another goal was delivered")
	require.True(t, accessorsOK.Load(), "GoalHandle.Server or Logger")
}

func TestGoalStatusString(t *testing.T) {
	for status, want := range map[rclgo.GoalStatus]string{
		rclgo.GoalUnknown:    "Unknown",
		rclgo.GoalAccepted:   "Accepted",
		rclgo.GoalExecuting:  "Executing",
		rclgo.GoalCanceling:  "Canceling",
		rclgo.GoalSucceeded:  "Succeeded",
		rclgo.GoalCanceled:   "Canceled",
		rclgo.GoalAborted:    "Aborted",
		rclgo.GoalStatus(99): "GoalStatus(99)",
	} {
		require.Equal(t, want, status.String())
	}
}
