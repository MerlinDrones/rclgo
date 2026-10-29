/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"sync/atomic"
	"testing"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/stretchr/testify/require"
)

// rcl timers on a ROS-time clock with the time override enabled (simulated
// time, as with Gazebo's /clock), after rclcpp's test_time_source and
// test_timer: a timer fires when simulated time passes its due time, not
// with wall time; a jump of several periods fires it once; a jump backwards
// resets it.
func TestTimerFollowsROSTimeOverride(t *testing.T) {
	rclctx := newSpinTestContext(t)
	clock := rclctx.Clock()
	require.NoError(t, clock.EnableROSTimeOverride())
	const t0 = 1000 * time.Second
	require.NoError(t, clock.SetROSTimeOverride(t0))

	var ticks atomic.Int32
	timer, err := rclctx.NewTimer(time.Second, func(*rclgo.Timer) { ticks.Add(1) })
	require.NoError(t, err)
	startSpin(t, rclctx.Spin)

	// waitTicks waits until the timer has fired want times in total, then
	// checks that it doesn't fire again within settle.
	waitTicks := func(want int32, what string) {
		t.Helper()
		require.Eventually(t, func() bool { return ticks.Load() >= want }, spinTimeout, 10*time.Millisecond,
			"%s: %d ticks, want %d", what, ticks.Load(), want)
		time.Sleep(300 * time.Millisecond) // nothing else must fire
		require.Equal(t, want, ticks.Load(), what)
	}

	// Simulated time stands still: no tick, however long the wall clock runs.
	time.Sleep(1500 * time.Millisecond)
	require.Zero(t, ticks.Load(), "the timer fired while simulated time stood still")
	left, err := timer.GetTimeUntilNextCall()
	require.NoError(t, err)
	require.Equal(t, int64(time.Second), left, "time until the next call in simulated time")

	// Simulated time passes the due time: one tick.
	require.NoError(t, clock.SetROSTimeOverride(t0+1500*time.Millisecond))
	waitTicks(1, "after 1.5 s of simulated time")

	// A jump over several periods fires once (rcl doesn't replay missed calls).
	require.NoError(t, clock.SetROSTimeOverride(t0+5500*time.Millisecond))
	waitTicks(2, "after a jump of 4 s")

	// A jump backwards resets the timer: the next call is a full period
	// after the new time.
	require.NoError(t, clock.SetROSTimeOverride(t0))
	left, err = timer.GetTimeUntilNextCall()
	require.NoError(t, err)
	require.Equal(t, int64(time.Second), left, "time until the next call after a backward jump")
	require.NoError(t, clock.SetROSTimeOverride(t0+500*time.Millisecond))
	time.Sleep(300 * time.Millisecond)
	require.Equal(t, int32(2), ticks.Load(), "fired before a full period after the backward jump")
	require.NoError(t, clock.SetROSTimeOverride(t0+1100*time.Millisecond))
	waitTicks(3, "a period after the backward jump")

	// Disabling the override returns to system time: the timer fires within
	// a period of system time (Spin must recompute its wait for that).
	require.NoError(t, clock.DisableROSTimeOverride())
	require.Eventually(t, func() bool { return ticks.Load() >= 4 }, 3*time.Second, 10*time.Millisecond,
		"no tick on system time after disabling the override")
}
