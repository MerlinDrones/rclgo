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
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Entities created and graph queries made while their context closes either
// succeed or fail with ErrClosed; none uses a finalized node, and none
// outlives the context (rclcpp keeps a node alive while entities are created
// on it in the same way).
func TestEntityCreationRacingClose(t *testing.T) {
	for round := 0; round < 20; round++ {
		rclctx, err := newDefaultRCLContext()
		require.NoError(t, err)
		node, err := rclctx.NewNode(fmt.Sprintf("racing_close_%d", round), "/race_test")
		require.NoError(t, err)

		create := []func(i int) error{
			func(i int) error {
				_, err := node.NewPublisher(fmt.Sprintf("pub%d", i), std_msgs_msg.StringTypeSupport, nil)
				return err
			},
			func(i int) error {
				_, err := node.NewSubscription(fmt.Sprintf("sub%d", i), std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
				return err
			},
			func(i int) error {
				_, err := node.NewClient(fmt.Sprintf("cli%d", i), example_interfaces_srv.AddTwoIntsTypeSupport, nil)
				return err
			},
			func(int) error { _, err := node.GetTopicNamesAndTypes(true); return err },
			func(int) error { _, _, err := node.GetNodeNames(); return err },
			func(int) error {
				_, err := rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
				return err
			},
		}
		var (
			wg   sync.WaitGroup
			mu   sync.Mutex
			errs []error
		)
		start := make(chan struct{})
		for _, f := range create {
			wg.Add(1)
			go func() {
				defer wg.Done()
				<-start
				for i := 0; ; i++ {
					if err := f(i); err != nil {
						mu.Lock()
						errs = append(errs, err)
						mu.Unlock()
						return
					}
				}
			}()
		}
		close(start)
		time.Sleep(time.Duration(round%5) * time.Millisecond)
		require.NoError(t, rclctx.Close())
		wg.Wait()
		for _, err := range errs {
			require.ErrorIs(t, err, rclgo.ErrClosed, "round %d", round)
		}
	}
}

// Closing a node makes its Node.Spin return, like closing a context does for
// Context.Spin.
func TestNodeCloseStopsNodeSpin(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("close_stops_spin", "/race_test")
	require.NoError(t, err)
	_, err = node.NewSubscription("topic", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	done := make(chan error, 1)
	go func() { done <- node.Spin(context.Background()) }()
	time.Sleep(spinSettle)
	require.NoError(t, node.Close())
	select {
	case err := <-done:
		require.NoError(t, err)
	case <-time.After(closeTimeout):
		t.Fatal("Node.Spin did not return after the node was closed")
	}
	require.ErrorIs(t, node.Spin(context.Background()), rclgo.ErrClosed)
}

// Requests sent while their client is closed return instead of waiting for a
// response forever.
func TestClientSendRacingClose(t *testing.T) {
	for round := 0; round < 20; round++ {
		rclctx := newSpinTestContext(t)
		node, err := rclctx.NewNode(fmt.Sprintf("send_racing_close_%d", round), "/race_test")
		require.NoError(t, err)
		client, err := node.NewClient("no_server", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
		require.NoError(t, err)
		const senders = 8
		done := make(chan error, senders)
		for i := 0; i < senders; i++ {
			go func() {
				_, _, err := client.Send(context.Background(), example_interfaces_srv.NewAddTwoInts_Request())
				done <- err
			}()
		}
		time.Sleep(time.Duration(round%4) * time.Millisecond)
		require.NoError(t, client.Close())
		for i := 0; i < senders; i++ {
			select {
			case err := <-done:
				require.Error(t, err)
				if !errors.Is(err, rclgo.ErrClosed) {
					require.Contains(t, err.Error(), "closed before a response")
				}
			case <-time.After(closeTimeout):
				t.Fatalf("round %d: Send did not return after Close", round)
			}
		}
	}
}

// A clock stays alive while timers use it, as rclcpp's timers hold their
// clock: closing it doesn't free the handle under a running timer, and calls
// on the closed clock return ErrClosed.
func TestClockClosedWhileTimerUsesIt(t *testing.T) {
	rclctx := newSpinTestContext(t)
	clk, err := rclctx.NewClock(rclgo.ClockTypeSteadyTime)
	require.NoError(t, err)
	rclctx.SetClock(clk)
	var (
		mu    sync.Mutex
		ticks int
	)
	timer, err := rclctx.NewTimer(5*time.Millisecond, func(*rclgo.Timer) {
		mu.Lock()
		ticks++
		mu.Unlock()
	})
	require.NoError(t, err)
	rclctx.SetClock(nil)
	stop := startSpin(t, rclctx.Spin)

	require.NoError(t, clk.Close())
	require.ErrorIs(t, clk.EnableROSTimeOverride(), rclgo.ErrClosed)
	mu.Lock()
	before := ticks
	mu.Unlock()
	time.Sleep(100 * time.Millisecond)
	mu.Lock()
	after := ticks
	mu.Unlock()
	require.Greater(t, after, before, "the timer stopped when its clock was closed")
	require.NoError(t, timer.Close())
	require.NoError(t, stop())
}

// A manual WaitSet can be run again after a Run ended for a reason other than
// a stop (here a panicking callback): the next Run doesn't see a leftover
// stop request.
func TestWaitSetRunsAgainAfterPanic(t *testing.T) {
	rclctx := newSpinTestContext(t)
	ws, err := rclctx.NewWaitSet()
	require.NoError(t, err)
	var (
		mu    sync.Mutex
		ticks int
		panik = true
	)
	timer, err := rclctx.NewTimer(5*time.Millisecond, func(*rclgo.Timer) {
		mu.Lock()
		defer mu.Unlock()
		if panik {
			panik = false
			panic("first tick")
		}
		ticks++
	})
	require.NoError(t, err)
	ws.AddTimers(timer)

	func() {
		defer func() { require.NotNil(t, recover(), "the callback's panic did not reach Run's caller") }()
		_ = ws.Run(context.Background())
	}()
	time.Sleep(20 * time.Millisecond) // let the wake-up goroutine trigger

	ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
	defer cancel()
	require.ErrorIs(t, ws.Run(ctx), context.DeadlineExceeded)
	mu.Lock()
	defer mu.Unlock()
	require.Positive(t, ticks, "the second Run did not service the timer")
}

// An invalid QoS profile (here a negative depth, which would wrap to a huge
// size in C) is an error instead of reaching rmw.
func TestNegativeQoSDepthIsAnError(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("bad_qos", "/race_test")
	require.NoError(t, err)
	opts := rclgo.NewDefaultPublisherOptions()
	opts.Qos.History = qos.HistorySystemDefault
	opts.Qos.Depth = -1
	_, err = node.NewPublisher("topic", std_msgs_msg.StringTypeSupport, opts)
	require.ErrorContains(t, err, "negative")
}

// A zero ActionServerOptions takes the defaults, as rclcpp starts from
// rcl_action_server_get_default_options: finished goals keep their results
// for clients to fetch (a zero result timeout discarded them at once).
func TestZeroActionServerOptionsKeepResults(t *testing.T) {
	f := newActionFixture(t, "zero_opts", &rclgo.ActionServerOptions{}, func(_ context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
		if _, err := goal.Accept(); err != nil {
			return nil, err
		}
		return fibResult(1, 1), nil
	})
	resp, id := f.sendGoal(2)
	require.True(t, resp.Accepted)
	time.Sleep(200 * time.Millisecond) // the goal finishes; its result must still be there
	res := f.getResult(id)
	require.EqualValues(t, rclgo.GoalSucceeded, res.Status)
	require.Equal(t, []int32{1, 1}, res.Result.Sequence)
}
