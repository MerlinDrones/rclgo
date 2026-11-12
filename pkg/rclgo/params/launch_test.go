/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package params_test

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

// An rclgo node started by ros2 launch, after launch_testing's smoke tests:
// launch_ros passes the node name and namespace as remappings, parameters as
// --params-file and -p, and on Ctrl-C it sends SIGINT and expects the node to
// exit cleanly.

// TestLaunchHelper is the node TestLaunch starts with ros2 launch. Launch
// appends "--ros-args ..." after the test flags and a "--".
func TestLaunchHelper(t *testing.T) {
	if os.Getenv("RCLGO_LAUNCH_HELPER") == "" {
		t.Skip("the node TestLaunch starts")
	}
	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()
	rclArgs, _, err := rclgo.ParseArgs(flag.Args())
	if err != nil {
		t.Fatal(err)
	}
	rctx, err := rclgo.NewContext(rclgo.ClockTypeROSTime, rclArgs)
	if err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := rctx.Close(); err != nil {
			t.Errorf("Close: %v", err)
		}
		fmt.Println("CLOSED")
	}()
	node, err := rctx.NewNode("launch_helper", "")
	if err != nil {
		t.Fatal(err)
	}
	mgr, err := params.NewManager(node)
	if err != nil {
		t.Fatal(err)
	}
	if err := params.ApplyOverrides(mgr, node.Name(), rclArgs); err != nil {
		t.Fatal(err)
	}
	for name, v := range map[string]params.Value{
		"greeting": {Kind: params.KindString, Str: "default"},
		"rate":     {Kind: params.KindInt64, Int64: 1},
		"unset":    {Kind: params.KindString, Str: "default"},
	} {
		if err := mgr.DeclareIfMissing(name, v, params.Descriptor{}); err != nil {
			t.Fatal(err)
		}
		p, _ := mgr.Get(name)
		fmt.Printf("PARAM %s=%s\n", name, p.Value)
	}
	fmt.Printf("NODE %s\n", node.FullyQualifiedName())
	fmt.Println("READY")
	if err := rctx.Spin(ctx); !errors.Is(err, context.Canceled) {
		t.Errorf("Spin: %v, want context.Canceled", err)
	}
}

func TestLaunch(t *testing.T) {
	rclgotest.RequireROS2(t)
	dir := t.TempDir()
	paramsFile := filepath.Join(dir, "params.yaml")
	mustNot(t, os.WriteFile(paramsFile, []byte("/**:\n  ros__parameters:\n    rate: 5\n"), 0o644))
	launchFile := filepath.Join(dir, "smoke.launch.py")
	mustNot(t, os.WriteFile(launchFile, []byte(fmt.Sprintf(`
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        executable=%q,
        arguments=['-test.run=^TestLaunchHelper$', '-test.v', '--'],
        name='launched',
        namespace='launch_test',
        parameters=[%q, {'greeting': 'from launch'}],
        additional_env={'RCLGO_LAUNCH_HELPER': '1', 'RCLGO_TEST_DOMAIN_ID': %q},
        output='screen',
    )])
`, os.Args[0], paramsFile, os.Getenv("ROS_DOMAIN_ID"))), 0o644))

	launch := rclgotest.Start(t, "ros2", "launch", launchFile)
	launch.WaitFor("READY")
	out := launch.Output()
	for _, want := range []string{
		"NODE /launch_test/launched", // name and namespace remapped
		"PARAM rate=5",               // from the parameter file
		"PARAM greeting=from launch", // from the parameters dictionary
		"PARAM unset=default",        // declared default
	} {
		if !strings.Contains(out, want) {
			t.Errorf("output lacks %q:\n%s", want, out)
		}
	}
	observer := rclgotest.Node(t, "launch_observer", "/launch_test")
	rclgotest.Eventually(t, "the launched node in the graph", func() error {
		names, namespaces, err := observer.GetNodeNames()
		if err != nil {
			return err
		}
		for i := range names {
			if names[i] == "launched" && namespaces[i] == "/launch_test" {
				return nil
			}
		}
		return fmt.Errorf("nodes %v in %v", names, namespaces)
	})

	// Ctrl-C reaches launch, which forwards SIGINT to the node.
	mustNot(t, launch.Signal(syscall.SIGINT))
	launch.WaitFor("CLOSED")
	launch.WaitFor("process has finished cleanly")
	if strings.Contains(launch.Output(), "--- FAIL") {
		t.Errorf("the node failed:\n%s", launch.Output())
	}
}

func mustNot(t *testing.T, err error) {
	t.Helper()
	if err != nil {
		t.Fatal(err)
	}
}
