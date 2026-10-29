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
	"os"
	"sync"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Services and clients, after rclcpp's test_service and test_client.

type addHandler = func(*example_interfaces_srv.AddTwoInts_Request, rclgo.ServiceResponseSender)

// newAddService creates an AddTwoInts service and client on one node, starts
// spinning, and waits until a call succeeds.
func newAddService(t *testing.T, name string, handle addHandler) (*rclgo.Node, *rclgo.Service, *rclgo.Client) {
	t.Helper()
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode(name+"_node", "/services_test")
	require.NoError(t, err)
	keepAll := qos.NewDefault()
	keepAll.History = qos.HistoryKeepAll
	srv, err := node.NewService(name, example_interfaces_srv.AddTwoIntsTypeSupport, &rclgo.ServiceOptions{Qos: keepAll},
		func(_ *rclgo.ServiceInfo, m types.Message, send rclgo.ServiceResponseSender) {
			handle(m.(*example_interfaces_srv.AddTwoInts_Request), send)
		})
	require.NoError(t, err)
	cli, err := node.NewClient(name, example_interfaces_srv.AddTwoIntsTypeSupport, &rclgo.ClientOptions{Qos: keepAll})
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)
	requireEventually(t, "service reachable", func() error {
		_, err := callAdd(cli, 0, 0, 300*time.Millisecond)
		return err
	})
	return node, srv, cli
}

func respondSum(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) error {
	resp := example_interfaces_srv.NewAddTwoInts_Response()
	resp.Sum = req.A + req.B
	return send.SendResponse(resp)
}

func callAdd(cli *rclgo.Client, a, b int64, timeout time.Duration) (int64, error) {
	ctx, cancel := context.WithTimeout(context.Background(), timeout)
	defer cancel()
	req := example_interfaces_srv.NewAddTwoInts_Request()
	req.A, req.B = a, b
	resp, _, err := cli.Send(ctx, req)
	if err != nil {
		return 0, err
	}
	return resp.(*example_interfaces_srv.AddTwoInts_Response).Sum, nil
}

func TestConcurrentServiceCalls(t *testing.T) {
	_, _, cli := newAddService(t, "concurrent_add", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		respondSum(req, send) //nolint:errcheck
	})
	var wg sync.WaitGroup
	errs := make(chan error, 50)
	for g := range 10 {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := range 5 {
				a, b := int64(g*100), int64(i)
				sum, err := callAdd(cli, a, b, actionTimeout)
				if err == nil && sum != a+b {
					err = fmt.Errorf("%d+%d = %d", a, b, sum)
				}
				if err != nil {
					errs <- err
				}
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}

// A handler may answer later, from another goroutine.
func TestDeferredServiceResponse(t *testing.T) {
	_, _, cli := newAddService(t, "deferred_add", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		go func() {
			time.Sleep(50 * time.Millisecond)
			respondSum(req, send) //nolint:errcheck
		}()
	})
	sum, err := callAdd(cli, 20, 22, actionTimeout)
	require.NoError(t, err)
	require.EqualValues(t, 42, sum)
}

// A deferred response sent after the service was closed must fail cleanly,
// not use the finalized service.
func TestServiceResponseAfterClose(t *testing.T) {
	senders := make(chan rclgo.ServiceResponseSender, 1)
	_, srv, cli := newAddService(t, "late_add", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 0 && req.B == 0 { // discovery probe
			respondSum(req, send) //nolint:errcheck
			return
		}
		senders <- send
	})
	go callAdd(cli, 1, 2, time.Second) //nolint:errcheck
	var send rclgo.ServiceResponseSender
	select {
	case send = <-senders:
	case <-time.After(actionTimeout):
		t.Fatal("request never arrived")
	}
	require.NoError(t, srv.Close())
	time.Sleep(100 * time.Millisecond) // let Spin drop the service so it is finalized
	err := respondSum(example_interfaces_srv.NewAddTwoInts_Request(), send)
	require.ErrorIs(t, err, rclgo.ErrClosed, "responding on a closed service must fail cleanly")
}

func TestClientSendHonoursContext(t *testing.T) {
	_, _, cli := newAddService(t, "silent_add", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 0 && req.B == 0 {
			respondSum(req, send) //nolint:errcheck
		} // otherwise never answer
	})
	start := time.Now()
	_, err := callAdd(cli, 1, 1, 200*time.Millisecond)
	require.ErrorIs(t, err, context.DeadlineExceeded)
	require.Less(t, time.Since(start), 2*time.Second)
}

// Closing a client wakes callers waiting for a response.
func TestClientCloseWakesPendingCalls(t *testing.T) {
	_, _, cli := newAddService(t, "closing_client_add", func(req *example_interfaces_srv.AddTwoInts_Request, send rclgo.ServiceResponseSender) {
		if req.A == 0 && req.B == 0 {
			respondSum(req, send) //nolint:errcheck
		}
	})
	result := make(chan error, 1)
	go func() {
		_, err := callAdd(cli, 1, 1, time.Minute)
		result <- err
	}()
	time.Sleep(100 * time.Millisecond)
	require.NoError(t, cli.Close())
	select {
	case err := <-result:
		require.Error(t, err)
		require.False(t, errors.Is(err, context.DeadlineExceeded), "the call waited for its deadline")
	case <-time.After(actionTimeout):
		t.Fatal("pending call was not woken by Close")
	}
}

func TestInvalidServiceNames(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("invalid_services", "/services_test")
	require.NoError(t, err)
	for _, name := range []string{"", "bad name", "/trailing/"} {
		_, err := node.NewService(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil,
			func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
		require.Error(t, err, "service %q", name)
		_, err = node.NewClient(name, example_interfaces_srv.AddTwoIntsTypeSupport, nil)
		require.Error(t, err, "client %q", name)
	}
}

// Timer accessors, after rclcpp's test_timer.
func TestTimerAccessors(t *testing.T) {
	rclctx := newSpinTestContext(t)
	tm, err := rclctx.NewTimer(time.Second, func(*rclgo.Timer) {})
	require.NoError(t, err)
	require.Same(t, rclctx, tm.Context())
	left, err := tm.GetTimeUntilNextCall()
	require.NoError(t, err)
	require.Greater(t, left, int64(800*time.Millisecond))
	require.LessOrEqual(t, left, int64(time.Second))

	time.Sleep(300 * time.Millisecond)
	left, err = tm.GetTimeUntilNextCall()
	require.NoError(t, err)
	require.Less(t, left, int64(800*time.Millisecond), "time until the next call does not decrease")
	require.NoError(t, tm.Reset())
	left, err = tm.GetTimeUntilNextCall()
	require.NoError(t, err)
	require.Greater(t, left, int64(900*time.Millisecond), "Reset did not restart the period")
}

// Calls from the user's own goroutines (polling TakeMessage, Publish) may race
// with Close of the same entity; skyhook polls an ACK subscription while
// another goroutine closes it. They must fail cleanly, not use freed memory.
func TestEntityCallsRacingClose(t *testing.T) {
	if os.Getenv("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" {
		// rmw_zenoh_cpp 0.2.11: rcl_publish never returns after subscriptions
		// on the same topic are created and destroyed while publishing; plain
		// rcl in C does the same, so this is not an rclgo defect.
		t.Skip("rmw_zenoh_cpp hangs in rcl_publish after subscription churn (upstream)")
	}
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("racing_close", "/services_test")
	require.NoError(t, err)
	for i := range 50 {
		sub, err := node.NewSubscription(fmt.Sprintf("racing_%d", i), example_interfaces_srv.AddTwoInts_RequestTypeSupport, nil, nil)
		require.NoError(t, err)
		pub, err := node.NewPublisher(fmt.Sprintf("racing_%d", i), example_interfaces_srv.AddTwoInts_RequestTypeSupport, nil)
		require.NoError(t, err)
		stop := make(chan struct{})
		var wg sync.WaitGroup
		wg.Add(2)
		go func() {
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
					sub.TakeMessage(example_interfaces_srv.NewAddTwoInts_Request()) //nolint:errcheck
				}
			}
		}()
		go func() {
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
					pub.Publish(example_interfaces_srv.NewAddTwoInts_Request()) //nolint:errcheck
				}
			}
		}()
		time.Sleep(time.Millisecond)
		require.NoError(t, sub.Close())
		require.NoError(t, pub.Close())
		time.Sleep(time.Millisecond)
		close(stop)
		wg.Wait()
		_, err = sub.TakeMessage(example_interfaces_srv.NewAddTwoInts_Request())
		require.ErrorIs(t, err, rclgo.ErrClosed)
		require.ErrorIs(t, pub.Publish(example_interfaces_srv.NewAddTwoInts_Request()), rclgo.ErrClosed)
	}
}
