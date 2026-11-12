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
	"testing"
	"time"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// closeTimeout bounds every Close in these tests.
const closeTimeout = 5 * time.Second

// requireReturns runs f in a goroutine and fails the test if it does not
// return within closeTimeout. It returns f's error.
func requireReturns(t *testing.T, what string, f func() error) error {
	t.Helper()
	done := make(chan error, 1)
	go func() { done <- f() }()
	select {
	case err := <-done:
		return err
	case <-time.After(closeTimeout):
		t.Fatalf("%s did not return", what)
		return nil
	}
}

// spinErrors runs spin in a goroutine and returns a channel that receives its
// error. Unlike startSpin it does not stop Spin in a cleanup, so tests can
// close the context under a running Spin.
func spinErrors(spin func(context.Context) error) (<-chan error, context.CancelFunc) {
	ctx, cancel := context.WithCancel(context.Background())
	errs := make(chan error, 1)
	go func() { errs <- spin(ctx) }()
	time.Sleep(spinSettle)
	return errs, cancel
}

func populateNode(t *testing.T, rclctx *rclgo.Context, name string) *rclgo.Node {
	t.Helper()
	node, err := rclctx.NewNode(name, "/shutdown_test")
	require.NoError(t, err)
	_, err = node.NewSubscription("topic", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	_, err = node.NewService("srv", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	require.NoError(t, err)
	_, err = node.NewClient("srv", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)
	_, err = rclctx.NewTimer(10*time.Millisecond, func(*rclgo.Timer) {})
	require.NoError(t, err)
	return node
}

// Closing a context while Spin runs on it must not hang, and Spin must
// return.
func TestContextCloseWhileSpinning(t *testing.T) {
	for _, tc := range []struct {
		name string
		spin func(*rclgo.Context, *rclgo.Node) func(context.Context) error
	}{
		{"Context.Spin", func(c *rclgo.Context, _ *rclgo.Node) func(context.Context) error { return c.Spin }},
		{"Node.Spin", func(_ *rclgo.Context, n *rclgo.Node) func(context.Context) error { return n.Spin }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rclctx, err := newDefaultRCLContext()
			require.NoError(t, err)
			node := populateNode(t, rclctx, "close_while_spinning")
			errs, cancel := spinErrors(tc.spin(rclctx, node))
			defer cancel()

			require.NoError(t, requireReturns(t, "Context.Close", rclctx.Close))
			select {
			case <-errs:
			case <-time.After(closeTimeout):
				t.Fatal("Spin did not return after its context was closed")
			}
		})
	}
}

// Closing an entity while Spin runs must neither break Spin nor free memory
// Spin still uses.
func TestCloseEntityWhileSpinning(t *testing.T) {
	type closer interface{ Close() error }
	for _, tc := range []struct {
		name   string
		create func(t *testing.T, rclctx *rclgo.Context, node *rclgo.Node) closer
	}{
		{"subscription", func(t *testing.T, _ *rclgo.Context, node *rclgo.Node) closer {
			s, err := node.NewSubscription("closing", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
			require.NoError(t, err)
			return s
		}},
		{"service", func(t *testing.T, _ *rclgo.Context, node *rclgo.Node) closer {
			s, err := node.NewService("closing", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
				func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
			require.NoError(t, err)
			return s
		}},
		{"client", func(t *testing.T, _ *rclgo.Context, node *rclgo.Node) closer {
			c, err := node.NewClient("closing", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
			require.NoError(t, err)
			return c
		}},
		{"timer", func(t *testing.T, rclctx *rclgo.Context, _ *rclgo.Node) closer {
			tm, err := rclctx.NewTimer(5*time.Millisecond, func(*rclgo.Timer) {})
			require.NoError(t, err)
			return tm
		}},
		{"action server", func(t *testing.T, _ *rclgo.Context, node *rclgo.Node) closer {
			s, err := node.NewActionServer("closing", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
				func(context.Context, *rclgo.GoalHandle) (types.Message, error) { return nil, errors.New("unused") }), nil)
			require.NoError(t, err)
			return s
		}},
		{"action client", func(t *testing.T, _ *rclgo.Context, node *rclgo.Node) closer {
			c, err := node.NewActionClient("closing", example_interfaces_action.FibonacciTypeSupport, nil)
			require.NoError(t, err)
			return c
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rclctx := newSpinTestContext(t)
			node, err := rclctx.NewNode("close_entity", "/shutdown_test")
			require.NoError(t, err)
			stop := startSpin(t, rclctx.Spin)
			for i := range 20 {
				entity := tc.create(t, rclctx, node)
				time.Sleep(time.Duration(i%4) * time.Millisecond)
				require.NoError(t, requireReturns(t, "Close", entity.Close))
			}
			// Spin must still be alive and servicing new entities.
			requireSubscriptionServiced(t, node, "after_close", func(cb rclgo.SubscriptionCallback) {
				_, err := node.NewSubscription("after_close", std_msgs_msg.StringTypeSupport, nil, cb)
				require.NoError(t, err)
			})
			require.NoError(t, stop())
		})
	}
}

// Closing a subscription from its own callback runs on the Spin goroutine; it
// must not deadlock waiting for Spin.
func TestCloseSubscriptionFromItsCallback(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("self_close", "/shutdown_test")
	require.NoError(t, err)
	stop := startSpin(t, rclctx.Spin)
	closed := make(chan error, 1)
	var sub *rclgo.Subscription
	sub, err = node.NewSubscription("self_close", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
		s.TakeMessage(std_msgs_msg.NewString()) //nolint:errcheck
		select {
		case closed <- sub.Close():
		default:
		}
	})
	require.NoError(t, err)
	pub, err := node.NewPublisher("self_close", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	deadline := time.After(spinTimeout)
	for done := false; !done; {
		require.NoError(t, pub.Publish(std_msgs_msg.NewString()))
		select {
		case err := <-closed:
			require.NoError(t, err)
			done = true
		case <-deadline:
			t.Fatal("subscription callback never ran or Close deadlocked")
		case <-time.After(50 * time.Millisecond):
		}
	}
	requireSubscriptionServiced(t, node, "after_self_close", func(cb rclgo.SubscriptionCallback) {
		_, err := node.NewSubscription("after_self_close", std_msgs_msg.StringTypeSupport, nil, cb)
		require.NoError(t, err)
	})
	require.NoError(t, stop())
}

// Timers on one clock register jump callbacks in an array that rcl does not
// synchronize. Creating and closing timers from several goroutines at once
// must not corrupt it.
func TestConcurrentTimersOnOneClock(t *testing.T) {
	rclctx := newSpinTestContext(t)
	errs := make(chan error, 8)
	for range 8 {
		go func() {
			for range 100 {
				tm, err := rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
				if err == nil {
					err = tm.Close()
				}
				if err != nil {
					errs <- err
					return
				}
			}
			errs <- nil
		}()
	}
	for range 8 {
		require.NoError(t, <-errs)
	}
}

// Context.Close while Spin is busy (a 1 ms timer keeps it waking up): Spin
// must still see that its wait set was closed and return.
func TestContextCloseWhileSpinningBusy(t *testing.T) {
	for i := range 50 {
		rclctx, err := newDefaultRCLContext()
		require.NoError(t, err)
		node := populateNode(t, rclctx, fmt.Sprintf("busy_%d", i))
		_, err = rclctx.NewTimer(time.Millisecond, func(*rclgo.Timer) {})
		require.NoError(t, err)
		errs, cancel := spinErrors(node.Context().Spin)
		require.NoError(t, requireReturns(t, "Context.Close", rclctx.Close))
		select {
		case <-errs:
		case <-time.After(closeTimeout):
			cancel()
			t.Fatalf("iteration %d: Spin did not return after its context was closed", i)
		}
		cancel()
	}
}
