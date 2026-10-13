/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package faultinject_test

import (
	"context"
	"errors"
	"fmt"
	"os"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"

	action_msgs_srv "github.com/merlindrones/rclgo/pkg/msgs/action_msgs/srv"
	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	rcl_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/faultinject"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// Every rcl call a scenario makes is made to fail in turn, as rclcpp's
// RCUTILS_FAULT_INJECTION_TEST does. Whatever fails, rclgo must not crash or
// hang, must hand the failure to the caller as an error, and must leave the
// process able to create and use new contexts.
//
// The full sweep runs with `make test-faults`; RCLGO_FAULT_CALLS sets how many
// calls of each function are made to fail (default 2), and RCLGO_FAULT_EXCLUDE
// lists functions not to fail (comma-separated), for investigating.

// errs collects errors from the scenario and its callbacks.
type errs struct {
	mu   sync.Mutex
	list []error
}

func (e *errs) add(err error) {
	if err != nil && !onlyCanceled(err) {
		e.mu.Lock()
		e.list = append(e.list, err)
		e.mu.Unlock()
	}
}

// onlyCanceled reports whether err is context.Canceled, possibly wrapped,
// with no other error joined to it.
func onlyCanceled(err error) bool {
	switch u := err.(type) {
	case interface{ Unwrap() []error }:
		for _, e := range u.Unwrap() {
			if !onlyCanceled(e) {
				return false
			}
		}
		return true
	case interface{ Unwrap() error }:
		return onlyCanceled(u.Unwrap())
	}
	return err == context.Canceled
}

func (e *errs) all() []error {
	e.mu.Lock()
	defer e.mu.Unlock()
	return append([]error(nil), e.list...)
}

// errTimeout is observed when a step does not complete in time.
var errTimeout = errors.New("timed out")

const stepTimeout = 3 * time.Second

// spin runs ctx.Spin until the returned stop is called, and records its error.
func spin(ctx *rclgo.Context, e *errs) (stop func()) {
	c, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		e.add(ctx.Spin(c))
	}()
	return func() {
		cancel()
		select {
		case <-done:
		case <-time.After(stepTimeout):
			e.add(fmt.Errorf("Spin did not return after cancel: %w", errTimeout))
		}
	}
}

// withContext creates a context and a node, runs body, and closes the node
// and context, recording every error.
func withContext(e *errs, name string, body func(ctx *rclgo.Context, node *rclgo.Node)) {
	ctx, err := rclgo.NewContext(0, nil)
	if err != nil {
		e.add(err)
		return
	}
	defer func() { e.add(ctx.Close()) }()
	node, err := ctx.NewNode(name, "/faultinject")
	if err != nil {
		e.add(err)
		return
	}
	defer func() { e.add(node.Close()) }()
	body(ctx, node)
}

func scenarioNode(*testing.T) []error {
	var e errs
	withContext(&e, "lifecycle", func(_ *rclgo.Context, node *rclgo.Node) {
		_ = node.Name() + node.Namespace() + node.FullyQualifiedName()
		e.add(node.Logger().Info("hello"))
		_, _, err := node.GetNodeNames()
		e.add(err)
		_, err = node.GetTopicNamesAndTypes(true)
		e.add(err)
		_, err = node.GetPublishersInfoByTopic("/rosout", false)
		e.add(err)
	})
	return e.all()
}

func scenarioPubSub(*testing.T) []error {
	var e errs
	withContext(&e, "pubsub", func(ctx *rclgo.Context, node *rclgo.Node) {
		got := make(chan string, 10)
		sub, err := node.NewSubscription("chatter", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
			m := std_msgs_msg.NewString()
			_, err := s.TakeMessage(m)
			if errors.Is(err, rclgo.ErrClosed) {
				return
			}
			e.add(err)
			if err == nil {
				got <- m.Data
			}
		})
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(sub.Close()) }()
		pub, err := node.NewPublisher("chatter", std_msgs_msg.StringTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(pub.Close()) }()
		defer spin(ctx, &e)()
		msg := std_msgs_msg.NewString()
		msg.Data = "hello"
		deadline := time.After(stepTimeout)
		for {
			if err := pub.Publish(msg); err != nil {
				e.add(err)
				return
			}
			select {
			case <-got:
				return
			case <-deadline:
				e.add(fmt.Errorf("message not received: %w", errTimeout))
				return
			case <-time.After(20 * time.Millisecond):
			}
		}
	})
	return e.all()
}

func scenarioService(*testing.T) []error {
	var e errs
	withContext(&e, "service", func(ctx *rclgo.Context, node *rclgo.Node) {
		srv, err := node.NewService("add", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
			func(_ *rclgo.ServiceInfo, req types.Message, rs rclgo.ServiceResponseSender) {
				r := example_interfaces_srv.NewAddTwoInts_Response()
				r.Sum = req.(*example_interfaces_srv.AddTwoInts_Request).A + 1
				e.add(rs.SendResponse(r))
			})
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(srv.Close()) }()
		cli, err := node.NewClient("add", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(cli.Close()) }()
		defer spin(ctx, &e)()
		req := example_interfaces_srv.NewAddTwoInts_Request()
		req.A = 41
		deadline := time.Now().Add(stepTimeout)
		for {
			c, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
			resp, _, err := cli.Send(c, req)
			cancel()
			if err == nil {
				if sum := resp.(*example_interfaces_srv.AddTwoInts_Response).Sum; sum != 42 {
					e.add(fmt.Errorf("wrong response %d", sum))
				}
				return
			}
			if !errors.Is(err, context.DeadlineExceeded) {
				e.add(err)
				return
			}
			if time.Now().After(deadline) {
				e.add(fmt.Errorf("no service response: %w", errTimeout))
				return
			}
		}
	})
	return e.all()
}

func scenarioTimer(*testing.T) []error {
	var e errs
	withContext(&e, "timer", func(ctx *rclgo.Context, _ *rclgo.Node) {
		fired := make(chan struct{}, 10)
		tm, err := ctx.NewTimer(10*time.Millisecond, func(*rclgo.Timer) {
			select {
			case fired <- struct{}{}:
			default:
			}
		})
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(tm.Close()) }()
		defer spin(ctx, &e)()
		for i := 0; i < 3; i++ {
			select {
			case <-fired:
			case <-time.After(stepTimeout):
				e.add(fmt.Errorf("timer did not fire: %w", errTimeout))
				return
			}
			if i == 1 {
				e.add(tm.Reset())
			}
		}
	})
	return e.all()
}

func scenarioAction(*testing.T) []error {
	var e errs
	withContext(&e, "action", func(ctx *rclgo.Context, node *rclgo.Node) {
		server, err := node.NewActionServer("fib", rclgo.NewAction(example_interfaces_action.FibonacciTypeSupport,
			func(c context.Context, goal *rclgo.GoalHandle) (types.Message, error) {
				order := goal.Description.(*example_interfaces_action.Fibonacci_Goal).Order
				fs, err := goal.Accept()
				if err != nil {
					e.add(err)
					return nil, err
				}
				fb := example_interfaces_action.NewFibonacci_Feedback()
				fb.Sequence = []int32{0, 1}
				e.add(fs.Send(fb))
				if order < 0 { // cancelled by the client
					<-c.Done()
					return nil, c.Err()
				}
				r := example_interfaces_action.NewFibonacci_Result()
				r.Sequence = []int32{0, 1, 1}
				return r, nil
			}), nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(server.Close()) }()
		client, err := node.NewActionClient("fib", example_interfaces_action.FibonacciTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(client.Close()) }()
		defer spin(ctx, &e)()

		goal := example_interfaces_action.NewFibonacci_Goal()
		goal.Order = 3
		// Wait for discovery with goals; the server accepts every goal.
		var id *types.GoalID
		deadline := time.Now().Add(stepTimeout)
		for id == nil {
			c, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
			_, gid, err := client.SendGoal(c, goal)
			cancel()
			switch {
			case err == nil:
				id = gid
			case !errors.Is(err, context.DeadlineExceeded):
				e.add(err)
				return
			case time.Now().After(deadline):
				e.add(fmt.Errorf("no goal response: %w", errTimeout))
				return
			}
		}
		c, cancel := context.WithTimeout(context.Background(), stepTimeout)
		defer cancel()
		_, err = client.GetResult(c, id)
		e.add(err)

		// A goal with feedback, cancelled while executing.
		goal.Order = -1
		feedback := make(chan struct{}, 1)
		watchCtx, stopWatch := context.WithTimeout(context.Background(), stepTimeout)
		defer stopWatch()
		watchDone := make(chan struct{})
		go func() {
			defer close(watchDone)
			_, _, err := client.WatchGoal(watchCtx, goal, func(context.Context, types.Message) {
				select {
				case feedback <- struct{}{}:
				default:
				}
			})
			if !errors.Is(err, context.Canceled) {
				e.add(err)
			}
		}()
		select {
		case <-feedback:
		case <-watchDone:
			return
		case <-time.After(stepTimeout):
			e.add(fmt.Errorf("no feedback: %w", errTimeout))
			stopWatch()
			<-watchDone
			return
		}
		req := action_msgs_srv.NewCancelGoal_Request() // cancels every goal
		_, err = client.CancelGoal(c, req)
		e.add(err)
		<-watchDone
	})
	return e.all()
}

func scenarioParams(*testing.T) []error {
	var e errs
	withContext(&e, "params", func(ctx *rclgo.Context, node *rclgo.Node) {
		pm, err := params.NewManager(node)
		if err != nil {
			e.add(err)
			return
		}
		_, err = pm.Declare("rate", params.Value{Kind: params.KindInt64, Int64: 5}, params.Descriptor{})
		e.add(err)
		_, err = pm.Set(params.Parameter{Name: "rate", Value: params.Value{Kind: params.KindInt64, Int64: 6}})
		e.add(err)
		cli, err := node.NewClient("/faultinject/params/get_parameters", rcl_interfaces_srv.GetParametersTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(cli.Close()) }()
		defer spin(ctx, &e)()
		req := rcl_interfaces_srv.NewGetParameters_Request()
		req.Names = []string{"rate"}
		deadline := time.Now().Add(stepTimeout)
		for {
			c, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
			resp, _, err := cli.Send(c, req)
			cancel()
			if err == nil {
				if v := resp.(*rcl_interfaces_srv.GetParameters_Response).Values; len(v) != 1 || v[0].IntegerValue != 6 {
					e.add(fmt.Errorf("wrong parameter values %+v", v))
				}
				return
			}
			if !errors.Is(err, context.DeadlineExceeded) {
				e.add(err)
				return
			}
			if time.Now().After(deadline) {
				e.add(fmt.Errorf("no parameter service response: %w", errTimeout))
				return
			}
		}
	})
	return e.all()
}

// healthy checks that the process can still create and use a context.
func healthy(t *testing.T, r faultinject.Run) {
	t.Helper()
	if err := errors.Join(scenarioPubSub(t)...); err != nil {
		t.Errorf("after %s the process no longer works: %v", r, err)
	}
}

func maxCalls() int {
	if n, err := strconv.Atoi(os.Getenv("RCLGO_FAULT_CALLS")); err == nil && n > 0 {
		return n
	}
	return 2
}

func TestFaultSweep(t *testing.T) {
	if os.Getenv("RCLGO_FAULT_SWEEP") == "" {
		t.Skip("set RCLGO_FAULT_SWEEP=1 (make test-faults) to run")
	}
	if ex := os.Getenv("RCLGO_FAULT_EXCLUDE"); ex != "" {
		for _, fn := range strings.Split(ex, ",") {
			faultinject.Excluded[fn] = "RCLGO_FAULT_EXCLUDE"
		}
	}
	for _, sc := range []struct {
		name string
		run  func(*testing.T) []error
	}{
		{"node", scenarioNode},
		{"pubsub", scenarioPubSub},
		{"service", scenarioService},
		{"timer", scenarioTimer},
		{"action", scenarioAction},
		{"params", scenarioParams},
	} {
		t.Run(sc.name, func(t *testing.T) {
			faultinject.Sweep(t, maxCalls(), faultinject.RetError, sc.run, func(t *testing.T, r faultinject.Run) {
				switch {
				case !r.Hit:
				case r.Reported():
				case r.Logged != "":
					t.Logf("logged: %s", r.Logged)
				default:
					t.Errorf("%s failed, but the failure was neither returned nor logged; errors: %v", r, r.Errs)
				}
				healthy(t, r)
			})
		})
	}
}
