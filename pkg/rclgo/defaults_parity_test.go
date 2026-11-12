/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"encoding/hex"
	"fmt"
	"os/exec"
	"strings"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/stretchr/testify/require"
)

// Every test_msgs message as constructed by the generated New (its default
// values) equals the same message default-constructed by rclpy: rclpy's
// serialization of T() deserializes in rclgo to exactly what New returns.
// The golden test only compares the generator with itself.
func TestDefaultsMatchRclpy(t *testing.T) {
	rclgotest.RequireROS2(t)
	args := []string{"-P", "testdata/interop/defaults_hex.py"}
	for _, tc := range testMessageTypes {
		args = append(args, "test_msgs/msg/"+tc.name)
	}
	out, err := exec.Command("python3", args...).CombinedOutput()
	if err != nil {
		rclgotest.Unavailable(t, "rclpy or test_msgs not usable: %v\n%s", err, out)
	}
	byType := map[string]string{}
	for _, line := range strings.Split(strings.TrimSpace(string(out)), "\n") {
		if f := strings.Fields(line); len(f) == 2 {
			byType[f[0]] = f[1]
		}
	}
	for _, tc := range testMessageTypes {
		t.Run(tc.name, func(t *testing.T) {
			h, ok := byType["test_msgs/msg/"+tc.name]
			require.True(t, ok, "no output for %s:\n%s", tc.name, out)
			buf, err := hex.DecodeString(h)
			require.NoError(t, err)
			fromPy, err := rclgo.Deserialize(buf, tc.ts)
			require.NoError(t, err)
			require.Equal(t, fmt.Sprintf("%#v", tc.ts.New()), fmt.Sprintf("%#v", fromPy), "defaults of %s", tc.name)
		})
	}
}
