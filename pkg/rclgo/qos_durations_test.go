/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
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
