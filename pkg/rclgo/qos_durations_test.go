/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"math"
	"strings"
	"testing"
	"time"

	std_msgs_msg "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/qos"
	"github.com/stretchr/testify/require"
)

// Durations with a seconds part must survive the trip to rmw and back through
// the graph API.
func TestEndpointInfoReportsQoSDurations(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("qos_durations", "/qos_test")
	require.NoError(t, err)
	profile := qos.NewDefault()
	profile.Deadline = 2500 * time.Millisecond
	profile.Lifespan = 10 * time.Second
	profile.Liveliness = qos.LivelinessManualByTopic
	profile.LivelinessLeaseDuration = 3 * time.Second
	_, err = node.NewPublisher("durations", std_msgs_msg.StringTypeSupport, &rclgo.PublisherOptions{Qos: profile})
	require.NoError(t, err)
	// mangle=true: the name is a ROS topic name. Discovery is asynchronous.
	var infos []rclgo.TopicEndpointInfo
	require.Eventually(t, func() bool {
		infos, err = node.GetPublishersInfoByTopic("/qos_test/durations", true)
		require.NoError(t, err)
		return len(infos) == 1
	}, spinTimeout, 50*time.Millisecond, "publisher not in the graph")
	got := infos[0].QosProfile
	require.Equal(t, profile.Deadline, got.Deadline, "deadline")
	require.Equal(t, profile.Lifespan, got.Lifespan, "lifespan")
	require.Equal(t, profile.LivelinessLeaseDuration, got.LivelinessLeaseDuration, "liveliness lease duration")
}

// Every valid profile creates entities unchanged, after rclcpp's test_qos:
// a lease duration with automatic liveliness, infinite durations and the
// best-available policies (resolved by rmw when the entity is created).
func TestQoSProfilesAccepted(t *testing.T) {
	rclctx := newSpinTestContext(t)
	node, err := rclctx.NewNode("qos_profiles", "/qos_test")
	require.NoError(t, err)
	// Every policy is set explicitly: the graph reports the system default
	// liveliness as automatic and an unspecified duration as infinite.
	explicit := func() qos.Profile {
		p := qos.NewDefault()
		p.Liveliness = qos.LivelinessAutomatic
		p.Deadline = qos.DurationInfinite
		p.Lifespan = qos.DurationInfinite
		p.LivelinessLeaseDuration = qos.DurationInfinite
		return p
	}
	automatic := explicit()
	automatic.LivelinessLeaseDuration = 3 * time.Second
	infinite := explicit()
	nearlyInfinite := explicit()
	nearlyInfinite.Deadline = time.Duration(math.MaxInt64 - 1) // RMW_QOS_DEADLINE_BEST_AVAILABLE
	for name, profile := range map[string]qos.Profile{
		"automatic liveliness with lease": automatic,
		"infinite durations":              infinite,
		"best-available deadline":         nearlyInfinite,
		"best available":                  qos.NewBestAvailableProfile(),
	} {
		t.Run(name, func(t *testing.T) {
			topic := "accepted_" + strings.ReplaceAll(strings.ReplaceAll(name, " ", "_"), "-", "_")
			pub, err := node.NewPublisher(topic, std_msgs_msg.StringTypeSupport, &rclgo.PublisherOptions{Qos: profile})
			require.NoError(t, err)
			defer pub.Close()
			sub, err := node.NewSubscription(topic, std_msgs_msg.StringTypeSupport, &rclgo.SubscriptionOptions{Qos: profile}, func(*rclgo.Subscription) {})
			require.NoError(t, err)
			defer sub.Close()
			if profile.Reliability == qos.ReliabilityBestAvailable || profile.Deadline == math.MaxInt64-1 {
				return // the DDS RMWs report the resolved policies, not best-available
			}
			var infos []rclgo.TopicEndpointInfo
			require.Eventually(t, func() bool {
				infos, err = node.GetPublishersInfoByTopic("/qos_test/"+topic, true)
				require.NoError(t, err)
				return len(infos) == 1
			}, spinTimeout, 50*time.Millisecond, "publisher not in the graph")
			got := infos[0].QosProfile
			require.Equal(t, profile.Liveliness, got.Liveliness, "liveliness")
			require.Equal(t, profile.Deadline, got.Deadline, "deadline")
			require.Equal(t, profile.Lifespan, got.Lifespan, "lifespan")
			require.Equal(t, profile.LivelinessLeaseDuration, got.LivelinessLeaseDuration, "liveliness lease duration")
		})
	}
}
