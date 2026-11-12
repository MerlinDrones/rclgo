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
	"fmt"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// actionTimeout bounds every wait for an action response.
const actionTimeout = 5 * time.Second

// probeOrder marks a goal used only to wait for discovery. Every test action
// rejects it before accepting.
const probeOrder = -1

type actionFixture struct {
	t      *testing.T
	ctx    *rclgo.Context
	node   *rclgo.Node
	server *rclgo.ActionServer
	client *rclgo.ActionClient
	stop   func() error
}

// newActionFixture creates an action server running execute and a client for
// it, starts spinning, and waits until the client can reach the server.
func newActionFixture(t *testing.T, name string, opts *rclgo.ActionServerOptions, execute rclgo.ExecuteGoalFunc) *actionFixture {
	t.Helper()
	f := &actionFixture{t: t, ctx: newSpinTestContext(t)}
	var err error
	f.node, err = f.ctx.NewNode(name+"_node", "/action_test")
	require.NoError(t, err)
	f.server, err = f.node.NewActionServer(name, rclgo.NewAction(
		example_interfaces_action.FibonacciTypeSupport,
		func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
			if goal.Description.(*example_interfaces_action.Fibonacci_Goal).Order == probeOrder {
				return nil, errors.New("probe")
			}
			return execute(ctx, goal)
		},
	), opts)
	require.NoError(t, err)
	// Deep service queues on both sides: with the default KEEP_LAST(10)
	// (the same as rclcpp) a burst of more than ten requests overflows the
	// queue and requests are dropped silently.
	copts := rclgo.NewDefaultActionClientOptions()
	copts.GoalServiceQos.Depth = 100
	copts.CancelServiceQos.Depth = 100
	copts.ResultServiceQos.Depth = 100
	f.client, err = f.node.NewActionClient(name, example_interfaces_action.FibonacciTypeSupport, copts)
	require.NoError(t, err)
	f.stop = startSpin(t, f.ctx.Spin)
	f.waitForServer()
	return f
}

// waitForServer sends probe goals until one gets a response. Requests sent
// before discovery completes are dropped, and a late probe is rejected
// harmlessly.
func (f *actionFixture) waitForServer() {
	f.t.Helper()
	deadline := time.Now().Add(spinTimeout)
	for time.Now().Before(deadline) {
		ctx, cancel := context.WithTimeout(context.Background(), 300*time.Millisecond)
		_, _, err := f.client.SendGoal(ctx, fibGoal(probeOrder))
		cancel()
		if err == nil {
			return
		}
	}
	f.t.Fatal("action server never became reachable")
}

func fibGoal(order int32) *example_interfaces_action.Fibonacci_Goal {
	g := example_interfaces_action.NewFibonacci_Goal()
	g.Order = order
	return g
}

func fibResult(seq ...int32) *example_interfaces_action.Fibonacci_Result {
	r := example_interfaces_action.NewFibonacci_Result()
	r.Sequence = seq
	return r
}

// sendGoal sends a goal and returns the goal response and goal ID.
func (f *actionFixture) sendGoal(order int32) (*example_interfaces_action.Fibonacci_SendGoal_Response, *types.GoalID) {
	f.t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	resp, id, err := f.client.SendGoal(ctx, fibGoal(order))
	require.NoError(f.t, err, "no goal response")
	return resp.(*example_interfaces_action.Fibonacci_SendGoal_Response), id
}

func (f *actionFixture) getResult(id *types.GoalID) *example_interfaces_action.Fibonacci_GetResult_Response {
	f.t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	resp, err := f.client.GetResult(ctx, id)
	require.NoError(f.t, err, "no result response")
	return resp.(*example_interfaces_action.Fibonacci_GetResult_Response)
}

func (f *actionFixture) cancelGoal(id *types.GoalID) *action_msgs_srv.CancelGoal_Response {
	f.t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), actionTimeout)
	defer cancel()
	req := action_msgs_srv.NewCancelGoal_Request()
	req.GoalInfo.GoalId.Uuid = *id
	resp, err := f.client.CancelGoal(ctx, req)
	require.NoError(f.t, err, "no cancel response")
	return resp.(*action_msgs_srv.CancelGoal_Response)
}

func requireClosed[T any](t *testing.T, ch <-chan T, what string) {
	t.Helper()
	select {
	case <-ch:
	case <-time.After(actionTimeout):
		t.Fatal(what)
	}
}

func TestActionServerSucceedsGoal(t *testing.T) {
	f := newActionFixture(t, "succeed", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return fibResult(0, 1, 1, 2), nil
	})
	before := time.Now()
	resp, id := f.sendGoal(4)
	require.True(t, resp.Accepted)
	// The response must carry the time the goal was accepted, not zero.
	stamp := time.Unix(int64(resp.Stamp.Sec), int64(resp.Stamp.Nanosec))
	require.WithinDuration(t, before, stamp, actionTimeout, "goal response stamp %v", stamp)

	result := f.getResult(id)
	require.EqualValues(t, rclgo.GoalSucceeded, result.Status)
	require.Equal(t, []int32{0, 1, 1, 2}, result.Result.Sequence)
}

func TestActionServerRejectsGoal(t *testing.T) {
	t.Run("execute returns an error before Accept", func(t *testing.T) {
		f := newActionFixture(t, "reject_err", nil, func(context.Context, *rclgo.GoalHandle) (types.Message, error) {
			return nil, errors.New("rejected")
		})
		resp, _ := f.sendGoal(3)
		require.False(t, resp.Accepted)
	})

	// A goal that is never accepted must still get a response.
	t.Run("execute returns a result without calling Accept", func(t *testing.T) {
		f := newActionFixture(t, "reject_noaccept", nil, func(context.Context, *rclgo.GoalHandle) (types.Message, error) {
			return fibResult(1), nil
		})
		resp, _ := f.sendGoal(3)
		require.False(t, resp.Accepted)
	})
}

func TestActionServerAbortsGoal(t *testing.T) {
	f := newActionFixture(t, "abort", nil, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return nil, errors.New("failed while executing")
	})
	resp, id := f.sendGoal(3)
	require.True(t, resp.Accepted)
	require.EqualValues(t, rclgo.GoalAborted, f.getResult(id).Status)
}

func TestActionServerCancelsGoal(t *testing.T) {
	executing := make(chan struct{})
	f := newActionFixture(t, "cancel", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		close(executing)
		<-ctx.Done()
		// A result returned after a cancel request must reach the
		// client.
		return fibResult(0, 1), nil
	})
	resp, id := f.sendGoal(10)
	require.True(t, resp.Accepted)
	requireClosed(t, executing, "goal never started executing")

	cancelResp := f.cancelGoal(id)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_NONE, cancelResp.ReturnCode)
	require.Len(t, cancelResp.GoalsCanceling, 1)
	require.Equal(t, *id, types.GoalID(cancelResp.GoalsCanceling[0].GoalId.Uuid))

	result := f.getResult(id)
	require.EqualValues(t, rclgo.GoalCanceled, result.Status)
	require.Equal(t, []int32{0, 1}, result.Result.Sequence)
}

func TestActionServerCancelUnknownGoal(t *testing.T) {
	f := newActionFixture(t, "cancel_unknown", nil, func(context.Context, *rclgo.GoalHandle) (types.Message, error) {
		return nil, errors.New("unused")
	})
	var id types.GoalID
	id[0] = 42
	cancelResp := f.cancelGoal(&id)
	require.Equal(t, action_msgs_srv.CancelGoal_Response_ERROR_UNKNOWN_GOAL_ID, cancelResp.ReturnCode)
	require.Empty(t, cancelResp.GoalsCanceling)
}

// Goals that end in any terminal state must expire after ResultTimeout,
// after which their result is no longer known.
func TestActionServerExpiresFinishedGoals(t *testing.T) {
	for _, tc := range []struct {
		name   string
		status rclgo.GoalStatus
	}{
		{"succeeded", rclgo.GoalSucceeded},
		{"aborted", rclgo.GoalAborted},
		{"canceled", rclgo.GoalCanceled},
	} {
		t.Run(tc.name, func(t *testing.T) {
			opts := rclgo.NewDefaultActionServerOptions()
			opts.ResultTimeout = 300 * time.Millisecond
			executing := make(chan struct{}, 1)
			f := newActionFixture(t, "expire_"+tc.name, opts, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
				if _, err := goal.Accept(); err != nil {
					return nil, err
				}
				switch tc.status {
				case rclgo.GoalAborted:
					return nil, errors.New("abort")
				case rclgo.GoalCanceled:
					executing <- struct{}{}
					<-ctx.Done()
					return nil, ctx.Err()
				}
				return fibResult(0), nil
			})
			resp, id := f.sendGoal(1)
			require.True(t, resp.Accepted)
			if tc.status == rclgo.GoalCanceled {
				requireClosed(t, executing, "goal never started executing")
				f.cancelGoal(id)
			}
			require.EqualValues(t, tc.status, f.getResult(id).Status)

			deadline := time.Now().Add(actionTimeout)
			for {
				status := f.getResult(id).Status
				if status == int8(rclgo.GoalUnknown) {
					return
				}
				if time.Now().After(deadline) {
					t.Fatalf("goal still known with status %d after ResultTimeout", status)
				}
				time.Sleep(100 * time.Millisecond)
			}
		})
	}
}

// Stopping Spin must not cancel goals that are executing.
func TestActionServerGoalSurvivesSpinStop(t *testing.T) {
	executing := make(chan struct{})
	release := make(chan struct{})
	var goalCtxErr atomic.Value
	f := newActionFixture(t, "spin_stop", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		close(executing)
		<-release
		goalCtxErr.Store(fmt.Sprint(ctx.Err()))
		return fibResult(0, 1, 1), nil
	})
	resp, id := f.sendGoal(3)
	require.True(t, resp.Accepted)
	requireClosed(t, executing, "goal never started executing")

	require.NoError(t, f.stop())
	close(release)
	f.stop = startSpin(t, f.ctx.Spin)

	result := f.getResult(id)
	require.Equal(t, "<nil>", goalCtxErr.Load(), "goal context was canceled by stopping Spin")
	require.EqualValues(t, rclgo.GoalSucceeded, result.Status)
	require.Equal(t, []int32{0, 1, 1}, result.Result.Sequence)
}

// Close must not hang once a finished goal's result timeout has
// passed.
func TestActionServerCloseAfterResultTimeout(t *testing.T) {
	opts := rclgo.NewDefaultActionServerOptions()
	opts.ResultTimeout = 50 * time.Millisecond
	f := newActionFixture(t, "close_timeout", opts, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return fibResult(0), nil
	})
	_, id := f.sendGoal(1)
	require.EqualValues(t, rclgo.GoalSucceeded, f.getResult(id).Status)
	time.Sleep(300 * time.Millisecond)

	require.NoError(t, f.stop())
	closed := make(chan struct{})
	go func() {
		f.server.Close() //nolint:errcheck
		close(closed)
	}()
	requireClosed(t, closed, "ActionServer.Close hung")
}

// Closing the server must cancel the context of goals that are executing so
// they can stop, and their result must be ABORTED rather than lost.
func TestActionServerCloseCancelsExecutingGoals(t *testing.T) {
	executing := make(chan struct{})
	stopped := make(chan struct{})
	f := newActionFixture(t, "close_executing", nil, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		close(executing)
		<-ctx.Done()
		close(stopped)
		return nil, ctx.Err()
	})
	resp, _ := f.sendGoal(3)
	require.True(t, resp.Accepted)
	requireClosed(t, executing, "goal never started executing")

	require.NoError(t, f.stop())
	closed := make(chan struct{})
	go func() {
		f.server.Close() //nolint:errcheck
		close(closed)
	}()
	requireClosed(t, closed, "ActionServer.Close hung")
	requireClosed(t, stopped, "goal context was not canceled by Close")
}

// Concurrent goals, cancels and expiry must not deadlock the server.
func TestActionServerConcurrentGoalsAndCancels(t *testing.T) {
	const goals = 20
	opts := rclgo.NewDefaultActionServerOptions()
	opts.ResultTimeout = 10 * time.Millisecond
	// See newActionFixture: the default queue depth drops a burst of goals.
	opts.GoalServiceQos.Depth = 100
	opts.CancelServiceQos.Depth = 100
	opts.ResultServiceQos.Depth = 100
	f := newActionFixture(t, "concurrent", opts, func(ctx context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(time.Duration(goal.Description.(*example_interfaces_action.Fibonacci_Goal).Order) * time.Millisecond):
			return fibResult(1), nil
		}
	})
	var wg sync.WaitGroup
	errs := make(chan error, goals)
	for i := range goals {
		wg.Add(1)
		go func() {
			defer wg.Done()
			ctx, cancel := context.WithTimeout(context.Background(), 4*actionTimeout)
			defer cancel()
			resp, id, err := f.client.SendGoal(ctx, fibGoal(int32(5+i*3)))
			if err != nil {
				errs <- fmt.Errorf("goal %d: send: %w", i, err)
				return
			}
			if !resp.(*example_interfaces_action.Fibonacci_SendGoal_Response).Accepted {
				errs <- fmt.Errorf("goal %d: rejected", i)
				return
			}
			if i%2 == 0 {
				req := action_msgs_srv.NewCancelGoal_Request()
				req.GoalInfo.GoalId.Uuid = *id
				if _, err := f.client.CancelGoal(ctx, req); err != nil {
					errs <- fmt.Errorf("goal %d: cancel: %w", i, err)
					return
				}
			}
			if _, err := f.client.GetResult(ctx, id); err != nil {
				errs <- fmt.Errorf("goal %d: result: %w", i, err)
			}
		}()
	}
	done := make(chan struct{})
	go func() { wg.Wait(); close(done) }()
	select {
	case <-done:
	case <-time.After(6 * actionTimeout):
		t.Fatal("concurrent goals did not finish (deadlock?)")
	}
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}
