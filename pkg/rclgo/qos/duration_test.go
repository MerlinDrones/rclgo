//go:build cgo

/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package qos

import (
	"math"
	"testing"
	"time"
)

// rmw durations are {sec, nsec} with nsec below one second; RMW_DURATION_INFINITE
// is {9223372036, 854775807}.
func TestDurationsUseSecondsAndNanoseconds(t *testing.T) {
	const infinite = time.Duration(math.MaxInt64)
	for _, tc := range []struct {
		d         time.Duration
		sec, nsec uint64
	}{
		{0, 0, 0},
		{123 * time.Nanosecond, 0, 123},
		{2500 * time.Millisecond, 2, 500_000_000},
		{infinite, 9223372036, 854775807},
	} {
		sec, nsec := rmwDurationsForTest(Profile{Deadline: tc.d, Lifespan: tc.d, LivelinessLeaseDuration: tc.d})
		for i := range sec {
			if sec[i] != tc.sec || nsec[i] != tc.nsec {
				t.Errorf("write %v: field %d = {%d, %d}, want {%d, %d}", tc.d, i, sec[i], nsec[i], tc.sec, tc.nsec)
			}
		}
		p := profileFromRMWDurationsForTest(tc.sec, tc.nsec)
		for i, got := range []time.Duration{p.Deadline, p.Lifespan, p.LivelinessLeaseDuration} {
			if got != tc.d {
				t.Errorf("read {%d, %d}: field %d = %v, want %v", tc.sec, tc.nsec, i, got, tc.d)
			}
		}
	}
}
