/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package params_test

import (
	"encoding/json"
	"fmt"
	"math"
	"strings"
	"testing"
	"time"

	rcl_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

// Parameters of every type over the real services, from rclpy's parameter
// client: values read by rclpy equal the declared ones, values set by rclpy
// arrive unchanged in the manager, and describe reports each type.
func TestParameterValuesOverTheWire(t *testing.T) {
	server := rclgotest.Node(t, "wire", "/params_test")
	m, err := params.NewManager(server)
	require.NoError(t, err)
	declared := []params.Parameter{
		{Name: "flag", Value: params.Value{Kind: params.KindBool, Bool: true}},
		{Name: "count", Value: params.Value{Kind: params.KindInt64, Int64: 42}},
		{Name: "ratio", Value: params.Value{Kind: params.KindDouble, Double: 0.25}},
		{Name: "label", Value: params.Value{Kind: params.KindString, Str: "hi"}},
		{Name: "blob", Value: params.Value{Kind: params.KindBytes, Bytes: []byte{1, 2}}},
		{Name: "flags", Value: params.Value{Kind: params.KindBoolArray, Bools: []bool{true, false, true}}},
		{Name: "counts", Value: params.Value{Kind: params.KindInt64Array, Int64s: []int64{1, 2, 3}}},
		{Name: "ratios", Value: params.Value{Kind: params.KindDoubleArray, Doubles: []float64{0.5}}},
		{Name: "labels", Value: params.Value{Kind: params.KindStringArray, Strs: []string{"x", "y"}}},
	}
	for _, p := range declared {
		_, err := m.Declare(p.Name, p.Value, params.Descriptor{})
		require.NoError(t, err)
	}
	rclgotest.Spin(t, server.Context().Spin)

	py := rclgotest.Python(t, "testdata/param_wire.py", server.FullyQualifiedName())
	py.WaitFor("DONE")
	type line struct {
		Op, Name, Reason, Error string
		Type                    uint8
		OK                      bool
		Value                   any
	}
	var lines []line
	for _, l := range strings.Split(py.Output(), "\n") {
		if !strings.HasPrefix(l, "{") {
			continue
		}
		var x line
		require.NoError(t, json.Unmarshal([]byte(l), &x), l)
		require.Empty(t, x.Error, l)
		lines = append(lines, x)
	}
	byOp := func(op string) map[string]line {
		out := map[string]line{}
		for _, x := range lines {
			if x.Op == op {
				out[x.Name] = x
			}
		}
		return out
	}
	// JSON renders every number as float64.
	get := byOp("get")
	want := map[string]any{
		"flag": true, "count": 42.0, "ratio": 0.25, "label": "hi", "blob": []any{1.0, 2.0},
		"flags": []any{true, false, true}, "counts": []any{1.0, 2.0, 3.0}, "ratios": []any{0.5},
		"labels": []any{"x", "y"},
	}
	for _, p := range declared {
		require.Equal(t, p.Value.Kind, get[p.Name].Type, "type of %s read by rclpy", p.Name)
		require.Equal(t, want[p.Name], get[p.Name].Value, "value of %s read by rclpy", p.Name)
	}
	for name, x := range byOp("set") {
		require.True(t, x.OK, "rclpy set %s: %s", name, x.Reason)
	}
	for name, x := range byOp("describe") {
		p, _ := m.Get(name)
		require.Equal(t, p.Value.Kind, x.Type, "describe %s", name)
	}

	// The values rclpy set arrive in the manager unchanged.
	for name, want := range map[string]params.Value{
		"flag":   {Kind: params.KindBool, Bool: false},
		"count":  {Kind: params.KindInt64, Int64: -7},
		"ratio":  {Kind: params.KindDouble, Double: 2.5},
		"label":  {Kind: params.KindString, Str: `é€ "x"`},
		"blob":   {Kind: params.KindBytes, Bytes: []byte{0, 0xff, 0x10}},
		"flags":  {Kind: params.KindBoolArray, Bools: []bool{false, true}},
		"counts": {Kind: params.KindInt64Array, Int64s: []int64{math.MinInt64, 0, math.MaxInt64}},
		"ratios": {Kind: params.KindDoubleArray, Doubles: []float64{-1.5, 0, 1e300}},
		"labels": {Kind: params.KindStringArray, Strs: []string{"", "a,b", "ハロー"}},
	} {
		p, ok := m.Get(name)
		require.True(t, ok, name)
		require.Equal(t, want, p.Value, "%s after rclpy's set", name)
	}
	// rclpy reads back what it set (the round trip through rclgo).
	after := byOp("get-after-set")
	require.Equal(t, []any{-9.223372036854776e18, 0.0, 9.223372036854776e18}, after["counts"].Value)
	require.Equal(t, []any{0.0, 255.0, 16.0}, after["blob"].Value)
}

// /parameter_events reports removed parameters in deleted_parameters: an
// undeclared parameter, and an override whose declaration failed.
func TestParameterDeletedEvents(t *testing.T) {
	server := rclgotest.Node(t, "deleted_events", "/params_test")
	m, err := params.NewManager(server)
	require.NoError(t, err)
	observer := rclgotest.Node(t, "deleted_events_observer", "/params_test")
	events := make(chan *rcl_interfaces_msg.ParameterEvent, 64)
	_, err = observer.NewSubscription("/parameter_events", rcl_interfaces_msg.ParameterEventTypeSupport,
		&rclgo.SubscriptionOptions{Qos: qos.NewParameterEventsProfile()}, func(s *rclgo.Subscription) {
			e := rcl_interfaces_msg.NewParameterEvent()
			if _, err := s.TakeMessage(e); err == nil && e.Node == server.FullyQualifiedName() {
				events <- e
			}
		})
	require.NoError(t, err)
	rclgotest.Spin(t, server.Context().Spin)
	rclgotest.Spin(t, observer.Context().Spin)
	rclgotest.Eventually(t, "event subscription matched", func() error {
		if _, err := m.Declare(fmt.Sprintf("probe_%d", time.Now().UnixNano()), params.Value{Kind: params.KindBool}, params.Descriptor{}); err != nil {
			return err
		}
		select {
		case <-events:
			return nil
		case <-time.After(200 * time.Millisecond):
			return fmt.Errorf("no event")
		}
	})
	waitDeleted := func(name string) {
		t.Helper()
		deadline := time.After(rclgotest.Timeout)
		for {
			select {
			case e := <-events:
				for _, p := range e.DeletedParameters {
					if p.Name == name {
						return
					}
				}
			case <-deadline:
				t.Fatalf("no deleted_parameters event for %s", name)
			}
		}
	}

	_, err = m.Declare("temporary", params.Value{Kind: params.KindInt64, Int64: 1}, params.Descriptor{})
	require.NoError(t, err)
	require.NoError(t, m.Undeclare("temporary"))
	waitDeleted("temporary")

	args, _, err := rclgo.ParseArgs([]string{"--ros-args", "-p", "limited:=500", "--"})
	require.NoError(t, err)
	require.NoError(t, params.ApplyOverrides(m, server.Name(), args))
	_, err = m.Declare("limited", params.Value{Kind: params.KindInt64, Int64: 1}, params.Descriptor{MaxInt: i64(100)})
	require.Error(t, err, "an override above the declared max")
	waitDeleted("limited")
}
