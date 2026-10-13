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
	"go.uber.org/goleak"
	"strings"
	"testing"
	"time"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/faultinject"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
)

func TestMain(m *testing.M) {
	rclgotest.UseIsolatedDomain()
	goleak.VerifyTestMain(m)
}

// A call from rclgo is routed through its wrapper: the chosen call fails with
// the rcl error message, the next one does not.
func TestInjectedFaultReachesCaller(t *testing.T) {
	node := rclgotest.Node(t, "smoke", "/faultinject")
	pub, err := node.NewPublisher("smoke", std_msgs_msg.StringTypeSupport, nil)
	if err != nil {
		t.Fatal(err)
	}
	faultinject.Arm(1, "rcl_publish", faultinject.RetError)
	err = pub.Publish(std_msgs_msg.NewString())
	calls, failed := faultinject.Disarm()
	if err == nil || !strings.Contains(err.Error(), "injected fault in rcl_publish") {
		t.Fatalf("Publish with an injected fault: %v", err)
	}
	if calls != 1 || failed != "rcl_publish" {
		t.Fatalf("calls %d, failed %q", calls, failed)
	}
	if err := pub.Publish(std_msgs_msg.NewString()); err != nil {
		t.Fatalf("Publish after Disarm: %v", err)
	}
}

// A node that cannot be finalized is leaked with its context, rather than
// freed while rcl's rosout map or the middleware still use them.
func TestNodeFiniFailureLeaksNodeAndContext(t *testing.T) {
	for _, fn := range []string{"rcl_node_fini", "rcl_logging_rosout_fini_publisher_for_node"} {
		t.Run(fn, func(t *testing.T) {
			ctx, err := rclgo.NewContext(0, nil)
			if err != nil {
				t.Fatal(err)
			}
			node, err := ctx.NewNode("leaked", "/faultinject")
			if err != nil {
				t.Fatal(err)
			}
			faultinject.Arm(1, fn, faultinject.RetError)
			err = node.Close()
			faultinject.Disarm()
			if err == nil || !strings.Contains(err.Error(), "not finalized; its memory is leaked") {
				t.Fatalf("node.Close: %v", err)
			}
			if err := ctx.Close(); err == nil || !strings.Contains(err.Error(), "context not finalized") {
				t.Fatalf("ctx.Close: %v", err)
			}
			// New nodes with the same name, in new contexts, keep working.
			for i := 0; i < 20; i++ {
				if errs := scenarioNode(t); len(errs) > 0 {
					t.Fatalf("after the leak: %v", errs)
				}
			}
		})
	}
}

// Spin returns after its context is canceled even if waking it fails once.
func TestSpinReturnsWhenWakingFails(t *testing.T) {
	ctx := rclgotest.Context(t)
	spinCtx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- ctx.Spin(spinCtx) }()
	time.Sleep(100 * time.Millisecond)
	faultinject.Arm(1, "rcl_trigger_guard_condition", faultinject.RetError)
	cancel()
	select {
	case err := <-done:
		if _, failed := faultinject.Disarm(); failed == "" {
			t.Fatal("the trigger was not made to fail")
		}
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("Spin: %v", err)
		}
	case <-time.After(5 * time.Second):
		faultinject.Disarm()
		t.Fatal("Spin did not return")
	}
}
