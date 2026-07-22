/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"testing"

	std_msgs_msg2 "github.com/merlindrones/rclgo/pkg/msgs/std_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// TestNodeAndNamespaceRemap verifies that "-r __node:=..." and "-r __ns:=..."
// ROS command line remap rules, passed via ParseArgs inside a
// "--ros-args ... --" block, are actually applied when a node is created.
func TestNodeAndNamespaceRemap(t *testing.T) {
	setNewDomainID()

	rclArgs, restArgs, err := rclgo.ParseArgs([]string{
		"--ros-args",
		"-r", "__node:=renamed_node",
		"-r", "__ns:=/remapped_ns",
		"--",
	})
	require.NoError(t, err)
	require.Empty(t, restArgs)

	ctx, err := rclgo.NewContext(0, rclArgs)
	require.NoError(t, err)
	defer ctx.Close()

	node, err := ctx.NewNode("original_node", "/original_ns")
	require.NoError(t, err)
	defer node.Close()

	require.Equal(t, "renamed_node", node.Name())
	require.Equal(t, "/remapped_ns", node.Namespace())
	require.Equal(t, "/remapped_ns/renamed_node", node.FullyQualifiedName())
}

// TestTopicRemap verifies that a "-r <topic>:=<new_topic>" ROS command line
// remap rule is applied to entities (here, a publisher) created on a node
// whose context was initialized with the remap rule.
func TestTopicRemap(t *testing.T) {
	setNewDomainID()

	rclArgs, restArgs, err := rclgo.ParseArgs([]string{
		"--ros-args",
		"-r", "chatter:=/new_chatter",
		"--",
	})
	require.NoError(t, err)
	require.Empty(t, restArgs)

	ctx, err := rclgo.NewContext(0, rclArgs)
	require.NoError(t, err)
	defer ctx.Close()

	node, err := ctx.NewNode("topic_remap_node", "")
	require.NoError(t, err)
	defer node.Close()

	pub, err := std_msgs_msg2.NewStringPublisher(node, "chatter", nil)
	require.NoError(t, err)
	defer pub.Close()

	requireTopicNamesAndTypes(t, node, map[string][]string{
		"/new_chatter": {"std_msgs/msg/String"},
	})
}

// TestUnwrappedRemapIsIgnored documents the ROS 2 CLI convention: remap rules
// (and any other ROS arguments) must be wrapped between "--ros-args" and
// "--". A bare "-r" outside that block is treated as a regular, non-ROS
// argument and is NOT applied as a remap - it is a common mistake to invoke a
// node directly (not via `ros2 run`) and omit the "--ros-args" wrapper.
func TestUnwrappedRemapIsIgnored(t *testing.T) {
	setNewDomainID()

	rclArgs, restArgs, err := rclgo.ParseArgs([]string{
		"-r", "__node:=renamed_node",
	})
	require.NoError(t, err)
	require.Equal(t, []string{"-r", "__node:=renamed_node"}, restArgs)

	ctx, err := rclgo.NewContext(0, rclArgs)
	require.NoError(t, err)
	defer ctx.Close()

	node, err := ctx.NewNode("original_node", "")
	require.NoError(t, err)
	defer node.Close()

	require.Equal(t, "original_node", node.Name())
}
