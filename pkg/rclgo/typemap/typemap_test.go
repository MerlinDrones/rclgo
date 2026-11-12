/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package typemap_test

import (
	"fmt"
	"sync"
	"testing"

	example_interfaces_action "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/action"
	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo/typemap"
	"github.com/stretchr/testify/require"
)

// Generated packages register their types on import, under both the short
// and the full ROS name.
func TestGeneratedTypesAreRegistered(t *testing.T) {
	for _, name := range []string{"std_msgs/String", "std_msgs/msg/String"} {
		ts, ok := typemap.GetMessage(name)
		require.True(t, ok, name)
		require.Equal(t, std_msgs_msg.StringTypeSupport, ts)
	}
	srv, ok := typemap.GetService("example_interfaces/srv/AddTwoInts")
	require.True(t, ok)
	require.Equal(t, example_interfaces_srv.AddTwoIntsTypeSupport, srv)
	act, ok := typemap.GetAction("example_interfaces/action/Fibonacci")
	require.True(t, ok)
	require.Equal(t, example_interfaces_action.FibonacciTypeSupport, act)

	_, ok = typemap.GetMessage("no_such/msg/Type")
	require.False(t, ok)
	require.Panics(t, func() { typemap.GetMessageMust("no_such/msg/Type") })
	require.Panics(t, func() { typemap.GetServiceMust("no_such/srv/Type") })
	require.Panics(t, func() { typemap.GetActionMust("no_such/action/Type") })
	require.Equal(t, std_msgs_msg.StringTypeSupport, typemap.GetMessageMust("std_msgs/msg/String"))
}

// Registering types at runtime while other goroutines look them up is safe.
func TestRegisterWhileLookingUp(t *testing.T) {
	var wg sync.WaitGroup
	for g := range 8 {
		wg.Add(2)
		go func() {
			defer wg.Done()
			for i := range 200 {
				name := fmt.Sprintf("runtime_%d/msg/T%d", g, i)
				typemap.RegisterMessage(name, std_msgs_msg.StringTypeSupport)
				typemap.RegisterService(name, example_interfaces_srv.AddTwoIntsTypeSupport)
				typemap.RegisterAction(name, example_interfaces_action.FibonacciTypeSupport)
			}
		}()
		go func() {
			defer wg.Done()
			for i := range 200 {
				typemap.GetMessage(fmt.Sprintf("runtime_%d/msg/T%d", g, i))
				typemap.GetService("example_interfaces/srv/AddTwoInts")
				typemap.GetAction("example_interfaces/action/Fibonacci")
			}
		}()
	}
	wg.Wait()
	_, ok := typemap.GetMessage("runtime_3/msg/T150")
	require.True(t, ok)
}
