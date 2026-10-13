/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

//#nosec G404

package rclgo_test

import (
	"fmt"
	"go.uber.org/goleak"
	"math/rand"
	"os"
	"strconv"
	"sync"
	"testing"

	"github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

var (
	usedDomainIDs      = map[int]bool{}
	usedDomainIDsMutex = sync.Mutex{}
)

func getUnusedDomainID() int {
	usedDomainIDsMutex.Lock()
	defer usedDomainIDsMutex.Unlock()
	if len(usedDomainIDs) == 102 {
		panic("ran out of unused domain IDs")
	}
	oldID, _ := strconv.Atoi(os.Getenv("ROS_DOMAIN_ID"))
	usedDomainIDs[oldID] = true
	newID := rand.Intn(102)
	for usedDomainIDs[newID] {
		newID = rand.Intn(102)
	}
	usedDomainIDs[newID] = true
	return newID
}

// setNewDomainID moves the process to a domain no earlier test used, or to the
// domain pinned with RCLGO_TEST_DOMAIN_ID.
func setNewDomainID() {
	if id := os.Getenv("RCLGO_TEST_DOMAIN_ID"); id != "" {
		os.Setenv("ROS_DOMAIN_ID", id)
		return
	}
	os.Setenv("ROS_DOMAIN_ID", strconv.Itoa(getUnusedDomainID()))
}

func TestMain(m *testing.M) {
	rclgotest.UseIsolatedDomain()
	setNewDomainID()
	goleak.VerifyTestMain(m)
}

func ExampleParseArgs() {
	rosArgs, restArgs, err := rclgo.ParseArgs(
		[]string{
			"--extra0",
			"args0",
			"--ros-args",
			"--log-level",
			"DEBUG",
			"--",
			"--extra1",
			"args1",
		},
	)
	if err != nil {
		panic(err)
	}
	fmt.Printf("rosArgs: [%v]\n", rosArgs)
	fmt.Printf("restArgs: %+v\n\n", restArgs)

	rosArgs, restArgs, err = rclgo.ParseArgs(
		[]string{"--ros-args", "--log-level", "INFO"},
	)
	if err != nil {
		panic(err)
	}
	fmt.Printf("rosArgs: [%v]\n", rosArgs)
	fmt.Printf("restArgs: %+v\n\n", restArgs)

	rosArgs, restArgs, err = rclgo.ParseArgs(
		[]string{"--extra0", "args0", "--extra1", "args1"},
	)
	if err != nil {
		panic(err)
	}
	fmt.Printf("rosArgs: [%v]\n", rosArgs)
	fmt.Printf("restArgs: %+v\n\n", restArgs)

	rosArgs, restArgs, err = rclgo.ParseArgs(nil)
	if err != nil {
		panic(err)
	}
	fmt.Printf("rosArgs: [%v]\n", rosArgs)
	fmt.Printf("restArgs: %+v\n", restArgs)

	// Output: rosArgs: [--log-level DEBUG]
	// restArgs: [--extra0 args0 --extra1 args1]
	//
	// rosArgs: [--log-level INFO]
	// restArgs: []
	//
	// rosArgs: []
	// restArgs: [--extra0 args0 --extra1 args1]
	//
	// rosArgs: []
	// restArgs: []
}

func requireValue[T any](t *testing.T, expected T) func(actual T, err error) {
	t.Helper()
	return func(actual T, err error) {
		t.Helper()
		require.NoError(t, err)
		require.Equal(t, expected, actual)
	}
}

func noErr[T any](v T, err error) func(*testing.T) T {
	return func(t *testing.T) T {
		t.Helper()
		require.NoError(t, err)
		return v
	}
}

func noopSub(*rclgo.Subscription) {}

func TestPubSubCount(t *testing.T) {
	setNewDomainID()

	rclctx := noErr(newDefaultRCLContext())(t)
	defer rclctx.Close()

	pubNode := noErr(rclctx.NewNode("node1", "test_pubsub_counts"))(t)
	subNode := noErr(rclctx.NewNode("node2", "test_pubsub_counts"))(t)

	pub := noErr(pubNode.NewPublisher("string", std_msgs_msg.StringTypeSupport, nil))(t)

	requireValue(t, 0)(pub.GetSubscriptionCount())

	sub1 := noErr(subNode.NewSubscription("string", std_msgs_msg.StringTypeSupport, nil, noopSub))(t)

	requireValue(t, 1)(pub.GetSubscriptionCount())
	requireValue(t, 1)(sub1.GetPublisherCount())

	sub2 := noErr(subNode.NewSubscription("string", std_msgs_msg.StringTypeSupport, nil, noopSub))(t)

	requireValue(t, 2)(pub.GetSubscriptionCount())
	requireValue(t, 1)(sub1.GetPublisherCount())
	requireValue(t, 1)(sub2.GetPublisherCount())

	if os.Getenv("RMW_IMPLEMENTATION") == "rmw_zenoh_cpp" {
		// The rest checks that endpoints with incompatible QoS (a
		// transient-local subscription and a volatile publisher) do not
		// match, which is DDS behaviour; rmw_zenoh_cpp counts them.
		return
	}
	subOptsUnrealiable := rclgo.NewDefaultSubscriptionOptions()
	subOptsUnrealiable.Qos.Reliability = qos.ReliabilityBestEffort
	subOptsUnrealiable.Qos.Durability = qos.DurabilityTransientLocal
	subUnreliable := noErr(subNode.NewSubscription("string", std_msgs_msg.StringTypeSupport, subOptsUnrealiable, noopSub))(t)

	requireValue(t, 2)(pub.GetSubscriptionCount())
	requireValue(t, 1)(sub1.GetPublisherCount())
	requireValue(t, 1)(sub2.GetPublisherCount())
	requireValue(t, 0)(subUnreliable.GetPublisherCount())

	pubOptsUnrealiable := rclgo.NewDefaultPublisherOptions()
	pubOptsUnrealiable.Qos.Reliability = qos.ReliabilityBestEffort
	pubOptsUnrealiable.Qos.Durability = qos.DurabilityTransientLocal
	pubUnreliable := noErr(pubNode.NewPublisher("string", std_msgs_msg.StringTypeSupport, pubOptsUnrealiable))(t)

	requireValue(t, 2)(pub.GetSubscriptionCount())
	requireValue(t, 1)(pubUnreliable.GetSubscriptionCount())
	requireValue(t, 1)(sub1.GetPublisherCount())
	requireValue(t, 1)(sub2.GetPublisherCount())
	requireValue(t, 1)(subUnreliable.GetPublisherCount())

	subOther := noErr(subNode.NewSubscription("other_string", std_msgs_msg.StringTypeSupport, nil, noopSub))(t)

	requireValue(t, 2)(pub.GetSubscriptionCount())
	requireValue(t, 1)(pubUnreliable.GetSubscriptionCount())
	requireValue(t, 1)(sub1.GetPublisherCount())
	requireValue(t, 1)(sub2.GetPublisherCount())
	requireValue(t, 1)(subUnreliable.GetPublisherCount())
	requireValue(t, 0)(subOther.GetPublisherCount())
}
