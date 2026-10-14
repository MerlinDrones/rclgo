/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"math/rand"
	"os"
	"strconv"
	"testing"
)

// TestMain isolates the generator tests, which build and run ROS packages,
// from other ROS traffic, as rclgo's own tests do
// (pkg/rclgo/internal/rclgotest.UseIsolatedDomain, which this package can't
// import): discovery limited to this machine unless the environment sets a
// range, and the domain from RCLGO_TEST_DOMAIN_ID or a random one in 1-101.
// Subprocesses inherit both.
func TestMain(m *testing.M) {
	if os.Getenv("RCLGO_TEST_DOMAIN_ID") != "" {
		os.Setenv("ROS_DOMAIN_ID", os.Getenv("RCLGO_TEST_DOMAIN_ID"))
	} else {
		os.Setenv("ROS_DOMAIN_ID", strconv.Itoa(1+rand.Intn(101))) //nolint:gosec
	}
	if os.Getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") == "" {
		os.Setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
	}
	os.Exit(m.Run())
}

// unavailable skips a test that needs something missing from the
// environment (a sourced ROS 2, colcon), or fails it if RCLGO_REQUIRE_ROS2 is
// set (pkg/rclgo/internal/rclgotest.Unavailable, which this package can't
// import).
func unavailable(t *testing.T, msg string) {
	t.Helper()
	if os.Getenv("RCLGO_REQUIRE_ROS2") != "" {
		t.Fatal("RCLGO_REQUIRE_ROS2 is set: " + msg)
	}
	t.Skip(msg)
}
