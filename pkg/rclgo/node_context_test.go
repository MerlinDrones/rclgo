/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"errors"
	"slices"
	"testing"
	"time"

	example_interfaces_srv "github.com/merlindrones/rclgo/pkg/msgs/example_interfaces/srv"
	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Node, context and entity lifecycle, after rclcpp's test_node,
// test_init_shutdown and test_context.

func TestNodeNames(t *testing.T) {
	rclctx := newSpinTestContext(t)
	for _, tc := range []struct{ name, ns, wantNS, wantFQN string }{
		{"plain", "", "/", "/plain"},
		{"in_ns", "ns", "/ns", "/ns/in_ns"},
		{"in_nested", "/a/b", "/a/b", "/a/b/in_nested"},
	} {
		node, err := rclctx.NewNode(tc.name, tc.ns)
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.name, node.Name())
		require.Equal(t, tc.wantNS, node.Namespace(), tc.name)
		require.Equal(t, tc.wantFQN, node.FullyQualifiedName(), tc.name)
		require.Same(t, rclctx, node.Context())
	}
	for _, tc := range []struct{ name, ns string }{
		{"bad name", ""},
		{"1starts_with_digit", ""},
		{"", ""},
		{"ok", "bad ns"},
		{"ok", "/trailing/"},
	} {
		_, err := rclctx.NewNode(tc.name, tc.ns)
		require.Error(t, err, "name %q namespace %q must be rejected", tc.name, tc.ns)
	}
}

// Closing anything twice returns an error instead of crashing.
func TestDoubleClose(t *testing.T) {
	rclctx, err := newDefaultRCLContext()
	require.NoError(t, err)
	node, err := rclctx.NewNode("double_close", "/lifecycle_test")
	require.NoError(t, err)
	pub, err := node.NewPublisher("t", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	sub, err := node.NewSubscription("t", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	srv, err := node.NewService("s", example_interfaces_srv.AddTwoIntsTypeSupport, nil,
		func(*rclgo.ServiceInfo, types.Message, rclgo.ServiceResponseSender) {})
	require.NoError(t, err)
	cli, err := node.NewClient("s", example_interfaces_srv.AddTwoIntsTypeSupport, nil)
	require.NoError(t, err)
	tm, err := rclctx.NewTimer(time.Hour, func(*rclgo.Timer) {})
	require.NoError(t, err)
	for _, c := range []struct {
		name  string
		close func() error
	}{
		{"publisher", pub.Close}, {"subscription", sub.Close}, {"service", srv.Close},
		{"client", cli.Close}, {"timer", tm.Close}, {"node", node.Close}, {"context", rclctx.Close},
	} {
		require.NoError(t, c.close(), "first close of %s", c.name)
		require.Error(t, c.close(), "second close of %s", c.name)
	}
}

// Closing a node closes its entities.
func TestNodeCloseClosesEntities(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("closes_entities", "/lifecycle_test")
	require.NoError(t, err)
	sub, err := node.NewSubscription("t", std_msgs_msg.StringTypeSupport, nil, func(*rclgo.Subscription) {})
	require.NoError(t, err)
	pub, err := node.NewPublisher("t", std_msgs_msg.StringTypeSupport, nil)
	require.NoError(t, err)
	require.NoError(t, node.Close())
	require.Error(t, sub.Close(), "subscription was not closed with its node")
	require.Error(t, pub.Close(), "publisher was not closed with its node")
}

func TestInitUninit(t *testing.T) {
	require.NoError(t, rclgo.Init(nil))
	first := rclgo.DefaultContext()
	require.NotNil(t, first)
	require.NoError(t, rclgo.Init(nil), "a second Init is a no-op")
	require.Same(t, first, rclgo.DefaultContext())
	node, err := rclgo.NewNode("init_uninit", "/lifecycle_test")
	require.NoError(t, err)
	require.Same(t, first, node.Context())

	require.NoError(t, rclgo.Uninit())
	require.Nil(t, rclgo.DefaultContext())
	require.NoError(t, rclgo.Uninit(), "a second Uninit is a no-op")
	_, err = rclgo.NewNode("after_uninit", "/lifecycle_test")
	require.Error(t, err, "NewNode without Init must fail")
	require.Error(t, rclgo.Spin(t.Context()), "Spin without Init must fail")
}

// Contexts in different DDS domains do not see each other (rcl's
// test_domain_id).
func TestContextDomainID(t *testing.T) {
	newCtx := func(domain uint) *rclgo.Context {
		opts := rclgo.NewDefaultContextOptions()
		opts.DomainID = domain
		c, err := rclgo.NewContextWithOpts(nil, opts)
		require.NoError(t, err)
		t.Cleanup(func() { c.Close() })
		return c
	}
	a, b, c := newCtx(150), newCtx(150), newCtx(151)
	_, err := a.NewNode("domain_peer", "/lifecycle_test")
	require.NoError(t, err)
	sameDomain, err := b.NewNode("same_domain", "/lifecycle_test")
	require.NoError(t, err)
	otherDomain, err := c.NewNode("other_domain", "/lifecycle_test")
	require.NoError(t, err)

	requireEventually(t, "peer in the same domain", func() error {
		names, _, err := sameDomain.GetNodeNames()
		if err != nil {
			return err
		}
		if !slices.Contains(names, "domain_peer") {
			return errors.New("not discovered")
		}
		return nil
	})
	names, _, err := otherDomain.GetNodeNames()
	require.NoError(t, err)
	require.NotContains(t, names, "domain_peer", "a node in another domain was discovered")
}

func TestContextSetClock(t *testing.T) {
	rclctx := newSpinTestContext(t)
	def := rclctx.Clock()
	require.NotNil(t, def)
	steady, err := rclctx.NewClock(rclgo.ClockTypeSteadyTime)
	require.NoError(t, err)
	rclctx.SetClock(steady)
	require.Same(t, steady, rclctx.Clock())
	rclctx.SetClock(nil)
	require.Same(t, def, rclctx.Clock(), "SetClock(nil) restores the default clock")
}
