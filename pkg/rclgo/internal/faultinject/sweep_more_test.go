/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package faultinject_test

import (
	"errors"
	"fmt"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// More scenarios for the sweep: graph queries, logging configuration, clocks
// and the time override, serialization, and entities added while Spin waits
// (which wakes it with a guard condition).

// eventually retries f until it succeeds, fails with an error retry doesn't
// accept, or stepTimeout passes.
func eventually(what string, retry func(error) bool, f func() error) error {
	deadline := time.Now().Add(stepTimeout)
	for {
		err := f()
		if err == nil || !retry(err) {
			return err
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("%s: %w (last error: %v)", what, errTimeout, err)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

func scenarioGraph(*testing.T) []error {
	var e errs
	withContext(&e, "graph", func(_ *rclgo.Context, node *rclgo.Node) {
		pub, err := node.NewPublisher("graph_topic", std_msgs_msg.StringTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(pub.Close()) }()
		srv, err := node.NewService("graph_srv", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
			func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(srv.Close()) }()
		// The node may not be in the graph yet: retry while rcl doesn't find it.
		notYet := func(err error) bool {
			var missing *rclgo.NodeNameNonExistent
			return errors.As(err, &missing)
		}
		byNode := func(f func(string, string) (map[string][]string, error)) func() error {
			return func() error {
				_, err := f("graph", "/faultinject")
				return err
			}
		}
		for what, f := range map[string]func() error{
			"publishers by node": byNode(func(n, ns string) (map[string][]string, error) {
				return node.GetPublisherNamesAndTypesByNode(false, n, ns)
			}),
			"subscribers by node": byNode(func(n, ns string) (map[string][]string, error) {
				return node.GetSubscriberNamesAndTypesByNode(false, n, ns)
			}),
			"services by node":       byNode(node.GetServiceNamesAndTypesByNode),
			"clients by node":        byNode(node.GetClientNamesAndTypesByNode),
			"action servers by node": byNode(node.GetActionServerNamesAndTypesByNode),
			"action clients by node": byNode(node.GetActionClientNamesAndTypesByNode),
			"subscriptions by topic": func() error {
				_, err := node.GetSubscriptionsInfoByTopic("/faultinject/graph_topic", false)
				return err
			},
		} {
			e.add(eventually(what, notYet, f))
		}
	})
	return e.all()
}

func scenarioLogging(*testing.T) []error {
	var e errs
	args, _, err := rclgo.ParseArgs([]string{"--ros-args", "--log-level", "faultinject.logging:=debug"})
	if err != nil {
		return []error{err}
	}
	e.add(rclgo.InitLogging(args))
	withContext(&e, "logging", func(_ *rclgo.Context, node *rclgo.Node) {
		l := node.Logger()
		e.add(l.SetLevel(rclgo.LogSeverityDebug))
		_, err := l.EffectiveLevel()
		e.add(err)
		e.add(l.Debug("debug"))
		e.add(l.Child("child").Info("from a child"))
		e.add(l.SetLevel(rclgo.LogSeverityUnset))
	})
	return e.all()
}

func scenarioClock(*testing.T) []error {
	var e errs
	withContext(&e, "clock", func(ctx *rclgo.Context, _ *rclgo.Node) {
		for _, typ := range []rclgo.ClockType{rclgo.ClockTypeROSTime, rclgo.ClockTypeSteadyTime} {
			clock, err := ctx.NewClock(typ)
			if err != nil {
				e.add(err)
				continue
			}
			if typ == rclgo.ClockTypeROSTime {
				e.add(clock.EnableROSTimeOverride())
				e.add(clock.SetROSTimeOverride(5 * time.Second))
				e.add(clock.DisableROSTimeOverride())
			}
			e.add(clock.Close())
		}
		// A timer on the context's clock follows the time override.
		clock := ctx.Clock()
		if err := clock.EnableROSTimeOverride(); err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(clock.DisableROSTimeOverride()) }()
		e.add(clock.SetROSTimeOverride(time.Second))
		fired := make(chan struct{}, 1)
		tm, err := ctx.NewTimer(100*time.Millisecond, func(*rclgo.Timer) {
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
		e.add(clock.SetROSTimeOverride(time.Second + 200*time.Millisecond))
		select {
		case <-fired:
		case <-time.After(stepTimeout):
			e.add(fmt.Errorf("timer did not fire on the overridden time: %w", errTimeout))
		}
	})
	return e.all()
}

func scenarioSerialization(*testing.T) []error {
	var e errs
	msg := std_msgs_msg.NewString()
	msg.Data = "serialized"
	buf, err := rclgo.Serialize(msg)
	if err != nil {
		return []error{err}
	}
	if m, err := rclgo.Deserialize(buf, std_msgs_msg.StringTypeSupport); err != nil {
		e.add(err)
	} else if got := m.(*std_msgs_msg.String).Data; got != msg.Data {
		e.add(fmt.Errorf("deserialized %q", got))
	}
	withContext(&e, "serialization", func(ctx *rclgo.Context, node *rclgo.Node) {
		got := make(chan []byte, 10)
		sub, err := node.NewSubscription("serialized", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
			b, _, err := s.TakeSerializedMessage()
			if errors.Is(err, rclgo.ErrClosed) {
				return
			}
			e.add(err)
			if err == nil {
				got <- b
			}
		})
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(sub.Close()) }()
		pub, err := node.NewPublisher("serialized", std_msgs_msg.StringTypeSupport, nil)
		if err != nil {
			e.add(err)
			return
		}
		defer func() { e.add(pub.Close()) }()
		defer spin(ctx, &e)()
		deadline := time.After(stepTimeout)
		for {
			if err := pub.PublishSerialized(buf); err != nil {
				e.add(err)
				return
			}
			select {
			case b := <-got:
				if string(b) != string(buf) {
					e.add(fmt.Errorf("took %x, published %x", b, buf))
				}
				return
			case <-deadline:
				e.add(fmt.Errorf("serialized message not received: %w", errTimeout))
				return
			case <-time.After(20 * time.Millisecond):
			}
		}
	})
	return e.all()
}

// Entities created and closed while Spin waits: Spin is woken through a
// guard condition to wait on them.
func scenarioGuard(*testing.T) []error {
	var e errs
	withContext(&e, "guard", func(ctx *rclgo.Context, _ *rclgo.Node) {
		defer spin(ctx, &e)()
		time.Sleep(50 * time.Millisecond) // Spin waits
		for i := 0; i < 2; i++ {
			fired := make(chan struct{}, 1)
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
			select {
			case <-fired:
			case <-time.After(stepTimeout):
				e.add(fmt.Errorf("timer added during Spin did not fire: %w", errTimeout))
			}
			e.add(tm.Close())
		}
	})
	return e.all()
}
