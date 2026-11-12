/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"fmt"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"
	"unsafe"

	rcl_interfaces_msg "github.com/merlindrones/rclgo/pkg/msgs/rcl_interfaces/msg"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

// The Logger API end to end, after rclcpp's test_logging and
// test_rosout_subscription: every logging method reaches /rosout with its
// level, the node's logger name, the formatted text and the caller's location.

type rosoutCollector struct {
	mu   sync.Mutex
	logs []*rcl_interfaces_msg.Log
}

func (c *rosoutCollector) find(msg string) *rcl_interfaces_msg.Log {
	c.mu.Lock()
	defer c.mu.Unlock()
	for _, l := range c.logs {
		if l.Msg == msg {
			return l
		}
	}
	return nil
}

func collectRosout(t *testing.T, node *rclgo.Node) *rosoutCollector {
	t.Helper()
	c := &rosoutCollector{}
	// rmw_qos_profile_rosout_default
	profile := qos.NewDefault()
	profile.Depth = 1000
	profile.Durability = qos.DurabilityTransientLocal
	_, err := node.NewSubscription("/rosout", rcl_interfaces_msg.LogTypeSupport, &rclgo.SubscriptionOptions{Qos: profile},
		func(s *rclgo.Subscription) {
			m := rcl_interfaces_msg.NewLog()
			if _, err := s.TakeMessage(m); err == nil {
				c.mu.Lock()
				c.logs = append(c.logs, m)
				c.mu.Unlock()
			}
		})
	require.NoError(t, err)
	return c
}

func TestLoggerMethodsReachRosout(t *testing.T) {
	logger := rclgotest.Node(t, "logging_node", "/logger_test")
	listener := rclgotest.Node(t, "rosout_listener", "/logger_test")
	rosout := collectRosout(t, listener)
	rclgotest.Spin(t, listener.Context().Spin)
	l := logger.Logger()
	require.NoError(t, l.SetLevel(rclgo.LogSeverityDebug))

	// Wait until the /rosout subscription matches the logger's publisher.
	rclgotest.Eventually(t, "/rosout matched", func() error {
		probe := fmt.Sprintf("probe %d", time.Now().UnixNano())
		l.Info(probe) //nolint:errcheck
		time.Sleep(50 * time.Millisecond)
		if rosout.find(probe) == nil {
			return fmt.Errorf("probe not received")
		}
		return nil
	})

	type call struct {
		level uint8
		msg   string
		log   func() error
	}
	calls := []call{
		{10, "debug a1", func() error { return l.Debug("debug ", "a1") }},
		{10, "debugf 2", func() error { return l.Debugf("debugf %d", 2) }},
		{10, "debugln x 3", func() error { return l.Debugln("debugln", "x", 3) }},
		{20, "info a1", func() error { return l.Info("info ", "a1") }},
		{20, "infof 2", func() error { return l.Infof("infof %d", 2) }},
		{20, "infoln x 3", func() error { return l.Infoln("infoln", "x", 3) }},
		{30, "warn", func() error { return l.Warn("warn") }},
		{30, "warnf 2", func() error { return l.Warnf("warnf %d", 2) }},
		{30, "warnln x", func() error { return l.Warnln("warnln", "x") }},
		{40, "error", func() error { return l.Error("error") }},
		{40, "errorf 2", func() error { return l.Errorf("errorf %d", 2) }},
		{40, "errorln x", func() error { return l.Errorln("errorln", "x") }},
		{50, "fatal", func() error { return l.Fatal("fatal") }},
		{50, "fatalf 2", func() error { return l.Fatalf("fatalf %d", 2) }},
		{50, "fatalln x", func() error { return l.Fatalln("fatalln", "x") }},
		{30, "log", func() error { return l.Log(rclgo.LogSeverityWarn, "log") }},
		{30, "logf 2", func() error { return l.Logf(rclgo.LogSeverityWarn, "logf %d", 2) }},
		{30, "logln x", func() error { return l.Logln(rclgo.LogSeverityWarn, "logln", "x") }},
	}
	for _, c := range calls {
		require.NoError(t, c.log(), c.msg)
	}
	for _, c := range calls {
		var got *rcl_interfaces_msg.Log
		rclgotest.Eventually(t, "log "+c.msg, func() error {
			if got = rosout.find(c.msg); got == nil {
				return fmt.Errorf("not on /rosout")
			}
			return nil
		})
		require.EqualValues(t, c.level, got.Level, c.msg)
		require.Equal(t, "logger_test.logging_node", got.Name, c.msg)
		require.Equal(t, "logger_rosout_test.go", filepath.Base(got.File), "%s: location is not the caller", c.msg)
		require.Contains(t, got.Function, "TestLoggerMethodsReachRosout", c.msg)
		require.NotZero(t, got.Line, c.msg)
	}

	// Below the logger's level nothing is logged.
	require.NoError(t, l.SetLevel(rclgo.LogSeverityError))
	require.NoError(t, l.Warn("suppressed warning"))
	require.NoError(t, l.Error("after suppressed warning"))
	rclgotest.Eventually(t, "error after level change", func() error {
		if rosout.find("after suppressed warning") == nil {
			return fmt.Errorf("not yet")
		}
		return nil
	})
	require.Nil(t, rosout.find("suppressed warning"), "a warning below the logger's level reached /rosout")
}

func TestLoggerLevels(t *testing.T) {
	// Logger levels are process-wide: a unique name keeps repeated and
	// reordered runs independent.
	name := fmt.Sprintf("levels_test%d", time.Now().UnixNano())
	parent := rclgo.GetLogger(name)
	child := parent.Child("child")
	require.Equal(t, name+".child", child.Name())
	require.Equal(t, name, child.Parent().Name())

	require.NoError(t, parent.SetLevel(rclgo.LogSeverityWarn))
	level, err := parent.Level()
	require.NoError(t, err)
	require.Equal(t, rclgo.LogSeverityWarn, level)

	// The child has no level of its own and inherits the parent's.
	eff, err := child.EffectiveLevel()
	require.NoError(t, err)
	require.Equal(t, rclgo.LogSeverityWarn, eff)
	require.False(t, child.IsEnabledFor(rclgo.LogSeverityInfo))
	require.True(t, child.IsEnabledFor(rclgo.LogSeverityError))

	require.NoError(t, child.SetLevel(rclgo.LogSeverityDebug))
	require.True(t, child.IsEnabledFor(rclgo.LogSeverityDebug))
	require.False(t, parent.IsEnabledFor(rclgo.LogSeverityDebug))
	require.Equal(t, "WARN", rclgo.LogSeverityWarn.String())
}

// cString reads a NUL-terminated C string without cgo (test files cannot use
// it).
func cString(p unsafe.Pointer) string {
	if p == nil {
		return ""
	}
	var b strings.Builder
	for i := uintptr(0); ; i++ {
		c := *(*byte)(unsafe.Add(p, i))
		if c == 0 {
			return b.String()
		}
		b.WriteByte(c)
	}
}

// A custom output handler sees every message with its severity and logger
// name; restoring the default handler stops it.
func TestCustomLoggingOutputHandler(t *testing.T) {
	// The handler receives messages once logging is initialized.
	rclgotest.Context(t)
	var (
		calls atomic.Int64
		mu    sync.Mutex
		seen  []string
	)
	rclgo.SetLoggingOutputHandler(func(_ unsafe.Pointer, severity int, name unsafe.Pointer, _ int64, _, _ unsafe.Pointer) {
		calls.Add(1)
		mu.Lock()
		seen = append(seen, fmt.Sprintf("%d %s", severity, cString(name)))
		mu.Unlock()
	})
	t.Cleanup(func() { rclgo.SetLoggingOutputHandler(nil) })
	require.NotNil(t, rclgo.GetLoggingOutputHandler())

	l := rclgo.GetLogger("handler_test")
	require.NoError(t, l.SetLevel(rclgo.LogSeverityInfo))
	require.NoError(t, l.Warn("to the custom handler"))
	mu.Lock()
	require.Contains(t, seen, "30 handler_test")
	mu.Unlock()

	rclgo.SetLoggingOutputHandler(nil)
	before := calls.Load()
	require.NoError(t, l.Warn("to the default handler"))
	require.Equal(t, before, calls.Load(), "the custom handler is still called after restoring the default")
}

// A dynamically loaded type support handles serialized messages: publishing
// with it reaches a typed subscriber. Its typed methods panic, as documented.
func TestDynamicMessageTypeSupport(t *testing.T) {
	ts, err := rclgo.LoadDynamicMessageTypeSupport("std_msgs", "String")
	require.NoError(t, err)
	require.Panics(t, func() { ts.New() })

	node := rclgotest.Node(t, "dynamic", "/dynamic_test")
	got := make(chan string, 1)
	_, err = node.NewSubscription("dynamic", std_msgs_msg.StringTypeSupport, nil, func(s *rclgo.Subscription) {
		m := std_msgs_msg.NewString()
		if _, err := s.TakeMessage(m); err == nil {
			select {
			case got <- m.Data:
			default:
			}
		}
	})
	require.NoError(t, err)
	rclgotest.Spin(t, node.Context().Spin)
	pub, err := node.NewPublisher("dynamic", ts, nil)
	require.NoError(t, err)
	msg := std_msgs_msg.NewString()
	msg.Data = "through a dynamic type support"
	buf, err := rclgo.Serialize(msg)
	require.NoError(t, err)
	rclgotest.Eventually(t, "message through dynamic type support", func() error {
		if err := pub.PublishSerialized(buf); err != nil {
			return err
		}
		select {
		case data := <-got:
			if data != msg.Data {
				return fmt.Errorf("got %q", data)
			}
			return nil
		case <-time.After(100 * time.Millisecond):
			return fmt.Errorf("not received")
		}
	})

	for _, bad := range [][2]string{{"no_such_package", "String"}, {"std_msgs", "NoSuchMessage"}} {
		_, err := rclgo.LoadDynamicMessageTypeSupport(bad[0], bad[1])
		require.Error(t, err, "%s/%s", bad[0], bad[1])
	}
}
