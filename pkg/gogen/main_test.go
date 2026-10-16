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
